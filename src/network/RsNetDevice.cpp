// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.
#include <httplib.h>

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#include "RsNetDevice.h"

#include <api.h>
#include <librealsense2-net/rs_net.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <thread>
#include <functional>

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>
#include <jpeg.h>

#include <stdlib.h>
#include <math.h>

using namespace std::placeholders;

////

class RsMediaSubsession; // forward

class RsMediaSession : public MediaSession
{
public:
    static RsMediaSession* createNew(UsageEnvironment& env, char const* sdpDescription);

protected:
    RsMediaSession(UsageEnvironment& env) : MediaSession(env) {};
    virtual ~RsMediaSession() {};

    virtual MediaSubsession* createNewMediaSubsession();

    friend class RsMediaSubsessionIterator;
};

class RsMediaSubsessionIterator
{
public:
    RsMediaSubsessionIterator(RsMediaSession const& session) : fOurSession(session) { reset(); };
    virtual ~RsMediaSubsessionIterator() {};

    RsMediaSubsession* next();
    void reset();

private:
    RsMediaSession const& fOurSession;
    RsMediaSubsession* fNextPtr;
};

class RsMediaSubsession : public MediaSubsession
{
protected:
    friend class RsMediaSession;
    friend class RsMediaSubsessionIterator;

    RsMediaSubsession(RsMediaSession& parent) : MediaSubsession(parent) {};
    virtual ~RsMediaSubsession() {};

    virtual Boolean createSourceObjects(int useSpecialRTPoffset) {    
        if (strcmp(fCodecName, "RS") == 0) {
            fReadSource = fRTPSource = RsVideoRTPSource::createNew(env(), fRTPSocket, fRTPPayloadFormat, fRTPTimestampFrequency, "video/LZ4");
            return True;
        }
        return MediaSubsession::createSourceObjects(useSpecialRTPoffset);
    };

};

RsMediaSession* RsMediaSession::createNew(UsageEnvironment& env, char const* sdpDescription)
{ 
    RsMediaSession* newSession = new RsMediaSession(env);
    if(newSession != NULL) {
        if(!newSession->initializeWithSDP(sdpDescription)) {
            delete newSession;
            return NULL;
        }
    }
    return newSession;
}

MediaSubsession* RsMediaSession::createNewMediaSubsession() {
    return new RsMediaSubsession(*this); 
}

RsMediaSubsession* RsMediaSubsessionIterator::next() {
    RsMediaSubsession* result = fNextPtr;
    if (fNextPtr != NULL) fNextPtr = (RsMediaSubsession*)(fNextPtr->fNext);
    return result;
}

void RsMediaSubsessionIterator::reset() {
    fNextPtr = (RsMediaSubsession*)(fOurSession.fSubsessionsHead); 
}

////

void rs_net_sensor::doRTP() {
    std::stringstream ss;
    ss << std::setiosflags(std::ios::left) << std::setw(14) << m_name << ": RTP support thread started" << std::endl;
    std::cout << ss.str();

    TaskScheduler* scheduler = BasicTaskScheduler::createNew(/* 1000 */); // Check this later
    m_env = BasicUsageEnvironment::createNew(*scheduler);

    // Start the watch thread  
    m_env->taskScheduler().scheduleDelayedTask(100000, doControl, this);

    // Start the scheduler
    m_env->taskScheduler().doEventLoop(&m_eventLoopWatchVariable);

    std::cout << m_name << " : RTP support thread exited" << std::endl;
}

void rs_net_sensor::doControl() {
    bool streaming = is_streaming();
    if (streaming != m_streaming) {
        // sensor state changed
        m_streaming = streaming;
        if (is_streaming()) {
            std::cout << "Sensor enabled\n";

            // Create RTSP client
            RTSPClient::responseBufferSize = 100000;
            m_rtspClient = RSRTSPClient::createNew(*m_env, m_mrl.c_str());
            if (m_rtspClient == NULL) {
                std::cout << "Failed to create a RTSP client for URL '" << m_mrl << "': " << m_env->getResultMsg() << std::endl;
                throw std::runtime_error("Cannot create RTSP client");
            }
            std::cout << "Connected to " << m_mrl << std::endl;

            // Prepare profiles list and allocate the queues
            m_streams.clear();
            // m_dev_flag = true;
            for (auto stream_profile : m_sw_sensor->get_active_streams()) {
                rs_net_stream* net_stream = new rs_net_stream(stream_profile);
                uint64_t key = slib::profile2key(net_stream->profile);
                m_streams[key] = net_stream;
            }
            
            // Start playing streams
            m_rtspClient->playStreams(m_streams);

            // Start SW device thread
            m_dev_flag = true;
            for (auto ks : m_streams) {
                rs_net_stream* net_stream = ks.second;
                net_stream->thread = std::thread( [&](){ doDevice(ks.first); });
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }
            // m_dev = std::thread( [&](){ doDevice(); });
        } else {
            std::cout << "Sensor disabled\n";

            // Stop SW device thread
            m_dev_flag = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            for (auto ks : m_streams) {
                auto net_stream = ks.second;
                if (net_stream->thread.joinable()) net_stream->thread.join();
            }

            // disable running RTP sessions
            m_rtspClient->shutdownStream();
            m_rtspClient = NULL;

            // remove the queues and their content
            for (auto ks : m_streams) {
                auto net_stream = ks.second;
                while (!net_stream->queue->empty()) {
                    delete net_stream->queue->front();
                    net_stream->queue->pop();
                }
                // delete net_stream->queue;
                delete net_stream;
            }
            m_streams.clear();
        }
    }

    m_env->taskScheduler().scheduleDelayedTask(100000, doControl, this);
}

uint32_t chunks_allocated = 0;

void rs_net_sensor::doDevice(uint64_t key) {

    uint32_t fps_frame_count = 0;
    auto beginning = std::chrono::system_clock::now();

#define RSTPER 1
    uint32_t rst_num = 0;

    uint32_t frame_size;
    rs_net_stream* net_stream = m_streams[key];
    if (net_stream->profile.is<rs2::video_stream_profile>()) {
        rs2::video_stream_profile vsp = net_stream->profile.as<rs2::video_stream_profile>();
        uint32_t bpp = (vsp.stream_type() == RS2_STREAM_INFRARED) ? 1 : 2; // IR:1 COLOR and DEPTH:2
        frame_size = vsp.width() * vsp.height() * bpp;

        int w = 0;
        if (vsp.width() / 16 * 16 == vsp.width()) w = vsp.width();
        else w = (vsp.width() / 16 + 1) * 16;
        
        int h = 0;
        if (vsp.height() / 16 * 16 == vsp.height()) h = vsp.height();
        else h = (vsp.height() / 16 + 1) * 16;
        
        rst_num = (w * h) / 256 / RSTPER; // MCU size = ((8*size_v) * (8*size_h)) = 256 because size_v = size_h = 2
    } else if (net_stream->profile.is<rs2::motion_stream_profile>()) {
        frame_size = 32;
    } else throw std::runtime_error("Unknown profile on SW device support thread start.");

    std::vector<uint8_t*> markers(rst_num);
    for (int i = 0; i < rst_num; i++) markers[i] = 0;

    // rs2_video_stream s = slib::key2stream(key);
    std::cout << m_name << "/" << rs2_stream_to_string(net_stream->profile.stream_type()) << "\t: SW device support thread started" << std::endl;

    int frame_count = 0; 
    bool prev_sensor_state = false;

    while (m_dev_flag) {
        if (net_stream->queue->empty()) continue;

        auto start = std::chrono::system_clock::now();

        uint32_t size = 0;
        uint32_t offset = 0;
        uint32_t total_size = 0;

        uint32_t  rst = 0;
        uint8_t*  off;
        uint8_t*  ptr;
        uint8_t*  sos = NULL;
        bool sos_flag = false;

        uint8_t*  marker_buffer;
        uint8_t*  marker_data;
        uint32_t* marker_size;
        
        while (offset < frame_size) {
            uint8_t* data = 0;
            do {
                if (!m_dev_flag) goto out;
                data = net_stream->queue->front();
            } while (data == 0);
            chunk_header_t* ch = (chunk_header_t*)data;

            if (ch->offset < offset) break;

            m_sw_sensor->set_metadata((rs2_frame_metadata_value)(ch->meta_id), (rs2_metadata_type)(ch->meta_data));

            total_size += ch->size;
            offset = ch->offset;
            int ret = 0;
            switch (ch->status & 3) {
            case 0:
                ret = ch->size - CHUNK_HLEN;
                memcpy((void*)(net_stream->m_frame_raw + offset), (void*)(data + CHUNK_HLEN), ret);
                break;
            case 1: 
                ret = ZSTD_decompress((void*)(net_stream->m_frame_raw + offset), CHUNK_SIZE, (void*)(data + CHUNK_HLEN), ch->size - CHUNK_HLEN); 
                break;
            case 2: 
                ret = LZ4_decompress_safe((const char*)(data + CHUNK_HLEN), (char*)(net_stream->m_frame_raw + offset), ch->size - CHUNK_HLEN, CHUNK_SIZE);
                ret = ch->size - CHUNK_HLEN;
                break;
            case 3:
#define HEAD_LEN 41
                // std::cout << "JPEG not implemented yet" << std::endl;
                ret = ch->size - CHUNK_HLEN;

                // build the array of RST markers
                rst = ch->index; // the index of starting RST marker
                off = data + CHUNK_HLEN; // skip the header
                ptr = off;
                ptr++;
                marker_buffer = NULL;

                // find next markers
                while (ptr - data < ch->size + CHUNK_HLEN) {
                    if (*ptr == 0xFF) {
                        // marker detected
                        switch(*(ptr + 1)) {
                        case 0xD9: //std::cout << " EOI : " << std::endl;
                        case 0xD0: 
                        case 0xD1: 
                        case 0xD2: 
                        case 0xD3: 
                        case 0xD4: 
                        case 0xD5: 
                        case 0xD6: 
                        case 0xD7: 
                            marker_buffer = new uint8_t[ptr - off  + sizeof(uint32_t)];
                            marker_size = (uint32_t*)marker_buffer;
                            marker_data = marker_buffer + sizeof(uint32_t);

                            *marker_size = ptr - off;
                            memcpy(marker_data, off, ptr - off);

                            if (markers[rst]) delete [] markers[rst];
                            markers[rst] = marker_buffer;
                            off = ptr;
                            rst++;
                        }
                    }
                    ptr++;
                }

                marker_buffer = new uint8_t[ptr - off  + sizeof(uint32_t)];
                marker_size = (uint32_t*)marker_buffer;
                marker_data = marker_buffer + sizeof(uint32_t);

                *marker_size = ptr - off;
                memcpy(marker_data, off, ptr - off);

                if (markers[rst]) delete [] markers[rst];
                markers[rst] = marker_buffer;
                off = ptr;
                rst++;

                break;
            }
            size += ret;
            // offset += CHUNK_SIZE;
            offset += ret;

            net_stream->queue->pop();
            delete [] data;
        } 

        uint8_t* frame_raw = new uint8_t[frame_size];
        if (net_stream->profile.stream_type() == RS2_STREAM_COLOR) {
            rs2::video_stream_profile vsp = net_stream->profile.as<rs2::video_stream_profile>();
            // recreate missing headers

            u_char *p = net_stream->m_frame_raw;
            u_char *start = p;

            /* convert from blocks to pixels */
            int w = vsp.width();
            int h = vsp.height();

            *p++ = 0xff;
            *p++ = 0xd8;            /* SOI */

            *p++ = 0xff;
            *p++ = 0xc0;            /* SOF */
            *p++ = 0;               /* length msb */
            *p++ = 17;              /* length lsb */
            *p++ = 8;               /* 8-bit precision */
            *p++ = h >> 8;          /* height msb */
            *p++ = h;               /* height lsb */
            *p++ = w >> 8;          /* width msb */
            *p++ = w;               /* wudth lsb */
            *p++ = 3;               /* number of components */
            *p++ = 0;               /* comp 0, should be 1? */ 
            *p++ = 0x22;            /* hsamp = 2, vsamp = 2 */
            *p++ = 0;               /* quant table 0 */
            *p++ = 1;               /* comp 1, should be 2? */
            *p++ = 0x11;            /* hsamp = 1, vsamp = 1 */
            *p++ = 1;               /* quant table 1 */
            *p++ = 2;               /* comp 2, should be 3? */
            *p++ = 0x11;            /* hsamp = 1, vsamp = 1 */
            *p++ = 1;               /* quant table 1 */

            u_short dri = RSTPER;
            *p++ = 0xff;
            *p++ = 0xdd;            /* DRI */
            *p++ = 0x0;             /* length msb */
            *p++ = 4;               /* length lsb */
            *p++ = dri >> 8;        /* dri msb */
            *p++ = dri & 0xff;      /* dri lsb */

            *p++ = 0xff;
            *p++ = 0xda;            /* SOS */
            *p++ = 0;               /* length msb */
            *p++ = 12;              /* length lsb */
            *p++ = 3;               /* 3 components */
            *p++ = 0;               /* comp 0 */
            *p++ = 0;               /* huffman table 0 */
            *p++ = 1;               /* comp 1 */
            *p++ = 0x11;            /* huffman table 1 */
            *p++ = 2;               /* comp 2 */
            *p++ = 0x11;            /* huffman table 1 */
            *p++ = 0;               /* first DCT coeff */
            *p++ = 63;              /* last DCT coeff */
            *p++ = 0;               /* sucessive approx. */

            for (int i = 0; i < markers.size(); i++) {
                marker_buffer = markers[i];
                if (marker_buffer == NULL) {
                    if (i != 0) {
                        int val = 0xd0 | (i%8);
                        *p++ = 0xff;
                        *p++ = val;
                    }

                    for (int j = 0; j < 256 * RSTPER; j++) *p++ = 0;
                } else {
                    marker_size = (uint32_t*)marker_buffer;
                    marker_data = marker_buffer + sizeof(uint32_t);

                    memcpy(p, marker_data, *marker_size);
                    p += *marker_size;
                }
            }

            *p++ = 0xff;
            *p++ = 0xd9; /* SOS */

            // decompress the JPEG
            try {
                jpeg::decompress(net_stream->m_frame_raw, total_size + (p - start), frame_raw, frame_size);
                size = frame_size;
                memset(net_stream->m_frame_raw, 0, frame_size);

                // // clean the markers array for debugging purposes
                // for (int i = 0; i < markers.size(); i++) {
                //     if (markers[i]) {
                //         delete [] markers[i];
                //         markers[i] = NULL;
                //     }
                // }

            } catch (...) {
                std::cout << "Cannot decompress the frame, of size " << total_size << " to the buffer of " << frame_size << std::endl;
            }
        } else {
            memcpy(frame_raw, net_stream->m_frame_raw, frame_size);
        }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::chrono::duration<double> total_time = end - beginning;
        fps_frame_count++;
        double fps;
        if (total_time.count() > 0) fps = (double)fps_frame_count / (double)total_time.count();
        else fps = 0;

        std::stringstream ss_name;
        ss_name << "Frame '" << std::setiosflags(std::ios::left);
        ss_name << std::setw(13) << m_name << " / " << rs2_stream_to_string(net_stream->profile.stream_type());
        if (net_stream->profile.stream_index()) ss_name << " " << net_stream->profile.stream_index();
        ss_name << "'";

        std::stringstream ss;
        ss << std::setiosflags(std::ios::left) << std::setw(35) << ss_name.str();
        ss << std::setiosflags(std::ios::right) << std::setiosflags(std::ios::fixed) << std::setprecision(2); 
        ss << " decompression time " << std::setw(7) << elapsed.count() * 1000 << "ms, ";
        ss << "size " << std::setw(7) << total_size << " => " << std::setw(7) << size << ", ";
        ss << "FPS: " << std::setw(7) << fps << std::endl;

        std::cout << ss.str();
        
        if (total_time > std::chrono::seconds(1)) {
            beginning = std::chrono::system_clock::now();
            fps_frame_count = 0;
        }

        // std::cout << "Chunks: " << chunks_allocated << "\n"; 

        // send it into device
        uint8_t* frame_raw_converted = frame_raw;
        if (net_stream->profile.is<rs2::video_stream_profile>()) {
            rs2::video_stream_profile vsp = net_stream->profile.as<rs2::video_stream_profile>();

            // set bpp
            int bpp = 1;
            switch(vsp.format()) {
            case RS2_FORMAT_Z16   : bpp = 2; break;
            case RS2_FORMAT_YUYV  : bpp = 2; break;
            case RS2_FORMAT_Y8    : bpp = 1; break;
            case RS2_FORMAT_UYVY  : bpp = 2; break;
            case RS2_FORMAT_RGB8  : bpp = 3; break;
            case RS2_FORMAT_BGR8  : bpp = 3; break;
            case RS2_FORMAT_RGBA8 : bpp = 4; break;
            case RS2_FORMAT_BGRA8 : bpp = 4; break;
            default: bpp = 0;
            }

            // convert the format if necessary
            switch(vsp.format()) {
            case RS2_FORMAT_RGB8  :
            case RS2_FORMAT_BGR8  :
                frame_raw_converted = new uint8_t[vsp.width() * vsp.height() * bpp];
                // perform the conversion
                for (int y = 0; y < vsp.height(); y++) {
                    for (int x = 0; x < vsp.width(); x += 2) {                
                        {
                            uint8_t Y = frame_raw[y * vsp.width() * 2 + x * 2 + 0];
                            uint8_t U = frame_raw[y * vsp.width() * 2 + x * 2 + 1];
                            uint8_t V = frame_raw[y * vsp.width() * 2 + x * 2 + 3];

                            uint8_t R = fmax(0, fmin(255, Y + 1.402 * (V - 128)));
                            uint8_t G = fmax(0, fmin(255, Y - 0.344 * (U - 128) - 0.714 * (V - 128)));
                            uint8_t B = fmax(0, fmin(255, Y + 1.772 * (U - 128)));

                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 0] = R;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 1] = G;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 2] = B;
                        }

                        {
                            uint8_t Y = frame_raw[y * vsp.width() * 2 + x * 2 + 2];
                            uint8_t U = frame_raw[y * vsp.width() * 2 + x * 2 + 1];
                            uint8_t V = frame_raw[y * vsp.width() * 2 + x * 2 + 3];

                            uint8_t R = fmax(0, fmin(255, Y + 1.402 * (V - 128)));
                            uint8_t G = fmax(0, fmin(255, Y - 0.344 * (U - 128) - 0.714 * (V - 128)));
                            uint8_t B = fmax(0, fmin(255, Y + 1.772 * (U - 128)));

                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 3] = R;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 4] = G;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 5] = B;
                        }                        
                    }
                }
                delete [] frame_raw;
                break;
            case RS2_FORMAT_RGBA8 :
            case RS2_FORMAT_BGRA8 :
                frame_raw_converted = new uint8_t[vsp.width() * vsp.height() * bpp];
                // perform the conversion
                for (int y = 0; y < vsp.height(); y++) {
                    for (int x = 0; x < vsp.width(); x += 2) {                
                        {
                            uint8_t Y = frame_raw[y * vsp.width() * 2 + x * 2 + 0];
                            uint8_t U = frame_raw[y * vsp.width() * 2 + x * 2 + 1];
                            uint8_t V = frame_raw[y * vsp.width() * 2 + x * 2 + 3];

                            uint8_t R = fmax(0, fmin(255, Y + 1.402 * (V - 128)));
                            uint8_t G = fmax(0, fmin(255, Y - 0.344 * (U - 128) - 0.714 * (V - 128)));
                            uint8_t B = fmax(0, fmin(255, Y + 1.772 * (U - 128)));

                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 0] = R;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 1] = G;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 2] = B;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 3] = 0xFF;
                        }

                        {
                            uint8_t Y = frame_raw[y * vsp.width() * 2 + x * 2 + 2];
                            uint8_t U = frame_raw[y * vsp.width() * 2 + x * 2 + 1];
                            uint8_t V = frame_raw[y * vsp.width() * 2 + x * 2 + 3];

                            uint8_t R = fmax(0, fmin(255, Y + 1.402 * (V - 128)));
                            uint8_t G = fmax(0, fmin(255, Y - 0.344 * (U - 128) - 0.714 * (V - 128)));
                            uint8_t B = fmax(0, fmin(255, Y + 1.772 * (U - 128)));

                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 4] = R;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 5] = G;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 6] = B;
                            frame_raw_converted[y * vsp.width() * bpp + x * bpp + 7] = 0xFF;
                        }                        
                    }
                }
                delete [] frame_raw;
                break;
            }

            // send the frame
            m_sw_sensor->on_video_frame(
                { 
                    (void*)frame_raw_converted, 
                    [] (void* f) { delete [] (uint8_t*)f; }, 
                    vsp.width() * bpp,
                    bpp,                                                  
                    (double)time(NULL), 
                    RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME, 
                    ++frame_count, 
                    vsp
                }
            );
        } else if (net_stream->profile.is<rs2::motion_stream_profile>()){
            rs2::motion_stream_profile msp = net_stream->profile.as<rs2::motion_stream_profile>();
            m_sw_sensor->on_motion_frame(
                { 
                    (void*)frame_raw_converted, 
                    [] (void* f) { delete [] (uint8_t*)f; }, 
                    (double)time(NULL), 
                    RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME, 
                    ++frame_count, 
                    msp
                }
            );
        } else throw std::runtime_error("Unknown profile on frame departure.");
    }

out:
    std::this_thread::sleep_for(std::chrono::milliseconds(std::rand()%10)); 
    std::cout << m_name << "/" << rs2_stream_to_string(net_stream->profile.stream_type()) << "\t: SW device support thread exited" << std::endl;
}

rs_net_device::rs_net_device(rs2::software_device sw_device, std::string ip_address)
    : m_device(sw_device)
{
    // parse the parameters and set address and port
    int colon = ip_address.find(":");
    m_ip_address = ip_address.substr(0, colon); // 10.10.10.10:8554 => 10.10.10.10
    m_ip_port = 8554; // default RTSP port
    if (colon != -1)
        try
        {
            m_ip_port = std::stoi(ip_address.substr(colon + 1)); // 10.10.10.10:8554 => 8554
        }
        catch(...) // ignore wrong port and use default
        {
            std::cout << "WARNING: invalid port specified, using default";
        }

    uint32_t pos = 0;

    // Obtain device information via HTTP and set it for the software device. 
    httplib::Client client(m_ip_address, 8080);
    auto ver = client.Get("/version");
    if (ver->status == 200) {
        // parse the response in form:
        // <LRS-Net version: X.Y.Z>

        std::string version;
        version += "LRS-Net version: ";
        version += std::to_string(RS2_NET_MAJOR_VERSION) + ".";
        version += std::to_string(RS2_NET_MINOR_VERSION) + ".";
        version += std::to_string(RS2_NET_PATCH_VERSION) + "\r\n";

        std::string server_version = ver->body;

        if (std::strcmp(version.c_str(), server_version.c_str())) {
            throw std::runtime_error("Version mismatch.");
        }
    }

    auto inf = client.Get("/devinfo");
    if (inf->status == 200) {
        // parse the response in form:
        // <info_index|info_desc|info_val>

        std::string devinfo = inf->body;
        while (!devinfo.empty()) {
            // get the index
            pos = devinfo.find("|");
            uint32_t idx = std::stoul(devinfo.substr(0, pos).c_str());
            devinfo.erase(0, pos + 1);

            // get the description
            pos = devinfo.find("|");
            std::string desc = devinfo.substr(0, pos);
            devinfo.erase(0, pos + 1);

            pos = devinfo.find("\r\n");
            std::string val = devinfo.substr(0, pos);
            devinfo.erase(0, pos + 2);
            if (std::strcmp(val.c_str(), "n/a") != 0) {
                std::cout << std::left << std::setw(30) << desc << " : " << val << std::endl;
                m_device.register_info((rs2_camera_info)idx, val);
            }
        }
    }

    // Obtain number of sensors and their names via HTTP and construct the software device. 
    auto res = client.Get("/query");
    if (res->status == 200) {
        // parse the response in form:
        // <sensor_name>|<sensor_mrl>|<sensor_profile1,intrinsics1>|<sensor_profile2,intrinsics2>|...|<sensor_profileN,intrinsicsN>
        // intrinsics = [width,heigth,ppx,ppy,fx,fy,coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],model|n/a]

        std::string query = res->body;
        while (!query.empty()) {
            // get the sensor line
            uint32_t line_pos = query.find("\r\n");
            std::string sensor = query.substr(0, line_pos) + "|";
            query.erase(0, line_pos + 2);

            // get the sensor name
            pos = sensor.find("|");
            std::string sensor_name = sensor.substr(0, pos);
            sensor.erase(0, pos + 1);

            // TODO: move the following code to the NetSensor's constructor
            NetSensor netsensor(new rs_net_sensor(m_device, sensor_name));

            // get the sensor MRL
            pos = sensor.find("|");
            netsensor->set_mrl(sensor.substr(0, pos));
            sensor.erase(0, pos + 1);

            while (!sensor.empty()) {
                pos = sensor.find(",");
                uint64_t key = std::stoull(sensor.substr(0, pos).c_str());
                sensor.erase(0, pos + 1);

                // get intrinsics
                rs2_intrinsics intrinsics_val;
                pos = sensor.find("|");
                std::string intrinsics_str = sensor.substr(0, pos);
                sensor.erase(0, pos + 1);
                if (std::strcmp(intrinsics_str.c_str(), "n/a") != 0) {
                    pos = intrinsics_str.find(",");
                    intrinsics_val.width = std::stoi(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    pos = intrinsics_str.find(",");
                    intrinsics_val.height = std::stoi(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    pos = intrinsics_str.find(",");
                    intrinsics_val.ppx = std::stof(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    pos = intrinsics_str.find(",");
                    intrinsics_val.ppy = std::stof(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    pos = intrinsics_str.find(",");
                    intrinsics_val.fx = std::stof(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    pos = intrinsics_str.find(",");
                    intrinsics_val.fy = std::stof(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);

                    for (int c = 0; c < 5; c++) {
                        pos = intrinsics_str.find(",");
                        intrinsics_val.coeffs[c] = std::stof(intrinsics_str.substr(0, pos).c_str());
                        intrinsics_str.erase(0, pos + 1);
                    }

                    pos = intrinsics_str.find("|");
                    intrinsics_val.model = (rs2_distortion)std::stoi(intrinsics_str.substr(0, pos).c_str());
                    intrinsics_str.erase(0, pos + 1);
                }

                netsensor->add_profile(key, intrinsics_val);
            }

            sensors.emplace_back(netsensor);
        }
    }

    // Obtain sensors options via HTTP and update the software device. 
    auto opt = client.Get("/options");
    if (opt->status == 200) {
        // parse the response in form:
        // <sensor_name>|<opt1_index>,[n/a|<opt1_value>,<opt1_min>,<opt1_max>,<opt1_def>,<opt1_step>]|...|<optN_index>,[n/a|<optN_value>,<optN_min>,<optN_max>,<optN_def>,<optN_step>]>

        std::string query = opt->body;
        while (!query.empty()) {
            // get the sensor line
            uint32_t line_pos = query.find("\r\n");
            std::string sensor = query.substr(0, line_pos) + "|";
            query.erase(0, line_pos + 2);

            // get the sensor name
            uint32_t pos = sensor.find("|");
            std::string sensor_name = sensor.substr(0, pos);
            sensor.erase(0, pos + 1);

            // locate the proper NetSensor
            auto s = sensors.begin();
            for (; s != sensors.end(); s++) {
                if (std::strcmp((*s)->get_name().c_str(), sensor_name.c_str()) == 0) {
                    break;
                }
            }

            while (!sensor.empty()) {
                pos = sensor.find(",");
                uint32_t idx = std::stoul(sensor.substr(0, pos).c_str());
                sensor.erase(0, pos + 1);
                pos = sensor.find("|");
                std::string vals = sensor.substr(0, pos + 1);
                sensor.erase(0, pos + 1);
                if (std::strcmp(vals.c_str(), "n/a|") != 0) {
                    float val = 0; 
                    rs2::option_range range = {0};

                    pos = vals.find(",");
                    val = std::stof(vals.substr(0, pos).c_str());
                    vals.erase(0, pos + 1);

                    pos = vals.find(",");
                    range.min = std::stof(vals.substr(0, pos).c_str());
                    vals.erase(0, pos + 1);

                    pos = vals.find(",");
                    range.max = std::stof(vals.substr(0, pos).c_str());
                    vals.erase(0, pos + 1);

                    pos = vals.find(",");
                    range.def = std::stof(vals.substr(0, pos).c_str());
                    vals.erase(0, pos + 1);

                    pos = vals.find("|");
                    range.step = std::stof(vals.substr(0, pos).c_str());
                    vals.erase(0, pos + 1);

                    (*s)->add_option(idx, val, range);
                }
            }
        }
    }

    std::cout << "Software device is ready" << std::endl;

    m_options = std::thread( [this](){ doOptions(); } ); 

    for (auto netsensor : sensors) netsensor->start();

    m_extrinsics = std::thread( [this](){ doExtrinsics(); } ); 
}

void rs_net_device::doExtrinsics() {
    std::cout << "Extrinsics initialization thread started" << std::endl;

    // Prepare extrinsics map
    httplib::Client client(m_ip_address, 8080);
    auto ext = client.Get("/extrinsics");
    if (ext->status == 200) {
        // parse the response in form:
        // <sensor_name_from>|<sensor_name_to>|<profie_name_from>|<profie_name_to>|<translation1>,...,<translation3>|<rotation1>,...,<rotation9>

        std::string query = ext->body;
        while (!query.empty()) {
            // get the one extrinsics line
            uint32_t line_pos = query.find("\r\n");
            std::string extrinsics = query.substr(0, line_pos);
            query.erase(0, line_pos + 2);

            // get the first stream name
            uint32_t pos = extrinsics.find("|");
            std::string from_name = extrinsics.substr(0, pos);
            extrinsics.erase(0, pos + 1);

            // get the second stream name
            pos = extrinsics.find("|");
            std::string to_name = extrinsics.substr(0, pos);
            extrinsics.erase(0, pos + 1);

            // get the first stream profile
            pos = extrinsics.find("|");
            uint64_t from_profile = std::stoll(extrinsics.substr(0, pos));
            extrinsics.erase(0, pos + 1);
            rs2_video_stream from_stream = slib::key2stream(from_profile);

            // get the second stream profile
            pos = extrinsics.find("|");
            uint64_t to_profile = std::stoll(extrinsics.substr(0, pos));
            extrinsics.erase(0, pos + 1);
            rs2_video_stream to_stream = slib::key2stream(to_profile);

            rs2_extrinsics extr;

            // translation
            for (int t = 0; t < 3; t++) {
                pos = extrinsics.find(",");
                extr.translation[t] = std::stof(extrinsics.substr(0, pos).c_str());
                extrinsics.erase(0, pos + 1);
            }

            // rotation
            for (int r = 0; r < 9; r++) {
                pos = extrinsics.find(",");
                extr.rotation[r] = std::stof(extrinsics.substr(0, pos).c_str());
                extrinsics.erase(0, pos + 1);
            }

            // set the extrinsincs
            m_extrinsics_map[StreamPair(StreamIndex(from_stream.type, from_stream.index), StreamIndex(to_stream.type, to_stream.index))] = extr;
        }
    }

    std::vector<rs2::stream_profile> profiles;
    for (rs2::sensor sensor : m_device.query_sensors()) {
        auto sprofiles = sensor.get_stream_profiles(); 
        profiles.insert(profiles.end(), sprofiles.begin(), sprofiles.end());
    }

    for (auto from_profile : profiles) {
        for (auto to_profile : profiles) {
            from_profile.register_extrinsics_to(to_profile, 
                m_extrinsics_map[StreamPair(StreamIndex(from_profile.stream_type(), from_profile.stream_index()), StreamIndex(to_profile.stream_type(), to_profile.stream_index()))]);
        }
    }

    std::cout << "Extrinsics initialization thread exited" << std::endl;
}

void rs_net_device::doOptions() {
    std::cout << "Options synchronization thread started" << std::endl;

    std::string options_prev;
    std::string options;

    while (1) {
        options.clear();
        for (auto netsensor : sensors) {
            options += netsensor->get_name();

            for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
                options += "|";

                rs2_option option_type = static_cast<rs2_option>(i);

                options += std::to_string(i); // option index
                options += ",";

                if (netsensor->get_sensor()->supports(option_type) && !netsensor->get_sensor()->is_option_read_only(option_type)) {
                    // Get the current value of the option
                    float current_value = netsensor->get_sensor()->get_option(option_type);
                    options += std::to_string(current_value);
                } else {
                    options += "n/a";
                }
            }
            options += "\r\n";
        }

        if (std::strcmp(options.c_str(), options_prev.c_str())) {
            httplib::Client client(m_ip_address, 8080);
            client.Post("/options", options.c_str(), "text/plain");
            options_prev = options;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    std::cout << "Options synchronization thread exited" << std::endl;
}

void RSRTSPClient::shutdownStream() {
    // First, check whether any subsessions have still to be closed:
    if (m_session != NULL) {
        Boolean someSubsessionsWereActive = False;
        MediaSubsessionIterator iter(*m_session);
        MediaSubsession* subsession;

        while ((subsession = iter.next()) != NULL) {
            if (subsession->sink != NULL) {
                Medium::close(subsession->sink);
                subsession->sink = NULL;
                if (subsession->rtcpInstance() != NULL) {
                    subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
                }
                someSubsessionsWereActive = True;
            }
        }

        if (someSubsessionsWereActive) {
            // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
            // Don't bother handling the response to the "TEARDOWN".
            sendTeardownCommand(*m_session, NULL);
        }

        m_session->close(m_session);
        m_session = NULL;
    }

    std::cout << "Closing the stream.\n";
    Medium::close(this);
}

// static callback
void RSRTSPClient::continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
    return ((RSRTSPClient*)rtspClient)->continueAfterDESCRIBE(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterDESCRIBE(int resultCode, char* resultString) {
    if (resultCode != 0) {
        std::cout << "Failed to get a SDP description: " << resultString << "\n";
        delete[] resultString;
        throw std::runtime_error("Failed to get a SDP description");
    } else {
        m_sdp = std::string(resultString);
        delete[] resultString; // because we don't need it anymore
    }

    prepareSession();
    playSession();
}

void RSRTSPClient::prepareSession() {
    UsageEnvironment& env = envir(); // alias

    // Create a media session object from this SDP description:
    // std::cout << "SDP description:\n" << m_sdp << "\n";
    m_session = RsMediaSession::createNew(env, m_sdp.c_str());

    if (m_session == NULL) {
        std::cout << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
        throw std::runtime_error("Malformed server response");
    } else if (!m_session->hasSubsessions()) {
        std::cout << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
        throw std::runtime_error("No profiles found");
    } else {
        std::cout << "Session created " << m_session->name() << "/" << m_session->sessionDescription() << "/" << m_session->controlPath() << "\n";
        return;
    }

    // An unrecoverable error occurred with this stream.
    shutdownStream();
}

void RSRTSPClient::playSession() {
    // setup streams first
    if (m_streams_it == NULL) m_streams_it = new std::map<uint64_t, rs_net_stream*>::iterator(m_streams.begin());

    while (*m_streams_it != m_streams.end()) {
        rs2::stream_profile profile = (*m_streams_it)->second->profile;
        uint64_t profile_key = slib::profile2key(profile);

        // check for fake formats for color streams
        if (profile.stream_type() == RS2_STREAM_COLOR) {
            profile_key = slib::profile2key(profile, RS2_FORMAT_YUYV);
        }

        bool profile_found = false;
        MediaSubsessionIterator it(*m_session);

        std::cout << "Looking  for " << slib::profile2key(profile) << "\t" << slib::print_profile(profile) << " => " << profile_key << std::endl;
        while (m_subsession = it.next()) {
            uint64_t subsession_key = std::stoull(m_subsession->attrVal_str("key"));
            rs2_video_stream vs = slib::key2stream(subsession_key);
            std::cout << "Checking for " << subsession_key << "\t" << slib::print_stream(&vs) << std::endl;
            if (profile_key == subsession_key) {
                std::cout << "Profile match for " << m_subsession->controlPath() << std::endl;
                profile_found = true;

                int useSpecialRTPoffset = -1; // for supported codecs
                if (strcmp(m_subsession->codecName(), "RS") == 0) {
                    useSpecialRTPoffset = 0;
                }

                if (!m_subsession->initiate(useSpecialRTPoffset)) {
                    std::cout << "Failed to initiate the \"" << m_subsession->controlPath() << "\" subsession: " << envir().getResultMsg() << "\n";
                } else {
                    std::cout << "Initiated the '" << std::setw(10) << m_subsession->controlPath() << "' " 
                                                << m_subsession->mediumName()   << "/" 
                                                << m_subsession->protocolName() << "/" 
                                                << m_subsession->videoWidth() << "x" << m_subsession->videoHeight() << "x" << m_subsession->videoFPS() << " subsession (";
                    if (m_subsession->rtcpIsMuxed()) {
                        std::cout << "client port " << m_subsession->clientPortNum();
                    } else {
                        std::cout << "client ports " << m_subsession->clientPortNum() << "-" << m_subsession->clientPortNum() + 1;
                    }
                    std::cout << ") [" << m_subsession->readSource()->name() << " : " << m_subsession->readSource()->MIMEtype() << "]\n";

                    // Continue setting up this subsession, by sending a RTSP "SETUP" command:
                    sendSetupCommand(*m_subsession, RSRTSPClient::continueAfterSETUP);
                    return;
                }
            }
        }
        if (!profile_found) throw std::runtime_error("Cannot match a profile");
    }

    delete m_streams_it;
    m_streams_it = NULL;

    sendPlayCommand(*m_session, continueAfterPLAY);
}

// static callback
void RSRTSPClient::continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
    return ((RSRTSPClient*)rtspClient)->continueAfterSETUP(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterSETUP(int resultCode, char* resultString)
{
    UsageEnvironment& env = envir(); // alias
    
    // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
    // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
    // after we've sent a RTSP "PLAY" command.)
    m_subsession->sink = RSSink::createNew(env, *m_subsession, url(), (*m_streams_it)->second->queue, m_subsession->videoWidth() * m_subsession->videoHeight() * m_subsession->videoFPS());

    // do not wait for the out of order packets
    // m_scs.subsession->rtpSource()->setPacketReorderingThresholdTime(0); 

    if (m_subsession->sink == NULL) {
        env << "Failed to create a data sink for the '" << m_subsession->controlPath() << "' subsession: " << env.getResultMsg() << "\n";
    } else {
        env << "Created a data sink for the \"" << m_subsession->controlPath() << "\" subsession\n";
        m_subsession->miscPtr = this; // a hack to let subsession handler functions get the "RTSPClient" from the subsession
        m_subsession->sink->startPlaying(*(m_subsession->readSource()), subsessionAfterPlaying, m_subsession);
        // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
        if (m_subsession->rtcpInstance() != NULL) {
            m_subsession->rtcpInstance()->setByeWithReasonHandler(subsessionByeHandler, m_subsession);
        }
    }

    // setup the remainded streams
    (*m_streams_it)++;
    playSession();
}

void RSRTSPClient::playStreams(std::map<uint64_t, rs_net_stream*> streams) {
    m_streams = streams;
    sendDescribeCommand(RSRTSPClient::continueAfterDESCRIBE);
}

// static callback
void RSRTSPClient::continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
    return ((RSRTSPClient*)rtspClient)->continueAfterPLAY(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterPLAY(int resultCode, char* resultString) {
    UsageEnvironment& env = envir(); // alias

    if (resultCode != 0) {
        env << "Failed to start playing session: " << resultString << "\n";

        // An unrecoverable error occurred with this stream.
        shutdownStream();
    } else {
        env << "Started playing session...\n";
    }

    delete[] resultString;
}

void RSRTSPClient::subsessionAfterPlaying(void* clientData)
{
    MediaSubsession* subsession = (MediaSubsession*)clientData;
    RSRTSPClient* rtspClient = (RSRTSPClient*)(subsession->miscPtr);

    std::cout << "Closing " << subsession->controlPath() << "session" << std::endl;

    // Begin by closing this subsession's stream:
    Medium::close(subsession->sink);
    subsession->sink = NULL;

    // Next, check whether *all* subsessions' streams have now been closed:
    MediaSession& session = subsession->parentSession();
    MediaSubsessionIterator iter(session);
    while((subsession = iter.next()) != NULL)
    {
        if(subsession->sink != NULL)
            return; // this subsession is still active
    }

    // All subsessions' streams have now been closed, so shutdown the client:
    rtspClient->shutdownStream();
}

void RSRTSPClient::subsessionByeHandler(void* clientData, char const* reason)
{
    MediaSubsession* subsession = (MediaSubsession*)clientData;
    RSRTSPClient* rtspClient = (RSRTSPClient*)subsession->miscPtr;
    UsageEnvironment& env = rtspClient->envir(); // alias

    env << "Received RTCP \"BYE\"";
    if(reason != NULL)
    {
        env << " (reason:\"" << reason << "\")";
        delete[](char*) reason;
    }
    env << " on \"" << subsession->controlPath() << "\" subsession\n";

    // Now act as if the subsession had closed:
    rtspClient->subsessionAfterPlaying(subsession);
}

// Implementation of "RSSink":
#define RS_SINK_RECEIVE_BUFFER_SIZE (CHUNK_SIZE + CHUNK_HLEN + 20)

RSSink* RSSink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId, SafeQueue* q, uint32_t threshold)
{
    return new RSSink(env, subsession, streamId, q, threshold);
}

RSSink::RSSink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId, SafeQueue* q, uint32_t threshold)
    : MediaSink(env), fSubsession(subsession), m_threshold(threshold), m_frames(q)
{
    fStreamId = strDup(streamId);
    fReceiveBuffer = new u_int8_t[RS_SINK_RECEIVE_BUFFER_SIZE];
    chunks_allocated++;
}

RSSink::~RSSink()
{
    if (fReceiveBuffer) delete[] fReceiveBuffer;
    delete[] fStreamId;
}

void RSSink::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds)
{
    RSSink* sink = (RSSink*)clientData;
    sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void RSSink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned /*durationInMicroseconds*/)
{
    // introducing scope for the lock_guard
    { 
        // std::lock_guard<std::mutex> lck (m_frames_mutex);
        // if (m_threshold / CHUNK_SIZE < m_frames.size()) {
        //     std::cout << "Dropping " << m_frames.size() << " chunks" << std::endl;
        //     chunks_allocated -= m_frames.size();
        //     while (!m_frames.empty()) {
        //         delete [] m_frames.front();
        //         m_frames.pop();
        //     }
        // }
        m_frames->push(fReceiveBuffer);
        fReceiveBuffer = new u_int8_t[RS_SINK_RECEIVE_BUFFER_SIZE];
        chunks_allocated++;
    }
 
    // Then continue, to request the next frame of data:
    continuePlaying();
}

void RSSink::getNextFrame(RSSink* sink) {
    sink->continuePlaying();
}

Boolean RSSink::continuePlaying() {
    if(fSource == NULL) return False; // sanity check (should not happen)

    // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
    fSource->getNextFrame(fReceiveBuffer, RS_SINK_RECEIVE_BUFFER_SIZE, afterGettingFrame, this, onSourceClosure, this);
    return True;
}
