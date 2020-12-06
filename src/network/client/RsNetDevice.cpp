// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

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

#include "JPEG2000DecodeFilter.h"
#include "JPEGDecodeFilter.h"
#include "LZ4DecodeFilter.h"
#include "LZ4VideoRTPSource.h"

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

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
        if (strcmp(fCodecName, "LZ4") == 0) {
            fReadSource = fRTPSource = LZ4VideoRTPSource::createNew(env(), fRTPSocket, fRTPPayloadFormat, fRTPTimestampFrequency, "video/LZ4");
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

RSRTSPClient* RSRTSPClient::createNew(UsageEnvironment& env, char const* rtspURL, rs_net_device& parent)
{
    return new RSRTSPClient(env, rtspURL, parent);
}

RSRTSPClient::RSRTSPClient(UsageEnvironment& env, char const* rtspURL, rs_net_device& parent)
    : RTSPClient(env, rtspURL, 0, "", 0, -1), ready(false), m_parent(parent)
{}

RSRTSPClient::~RSRTSPClient() {}

// Implementation of "StreamClientState":

StreamClientState::StreamClientState()
    : iter(NULL)
    , session(NULL)
    , subsession(NULL)
    , streamTimerTask(NULL)
    , duration(0.0)
{}

StreamClientState::~StreamClientState()
{
    delete iter;
    if(session != NULL)
    {
        // We also need to delete "session", and unschedule "streamTimerTask" (if set)
        UsageEnvironment& env = session->envir(); // alias

        env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
        Medium::close(session);
    }
}

void* rs_net_device::doRTP() {
    std::cout << "RTP support thread started" << std::endl;

    std::string rtspURL;
    rtspURL.append("rtsp://").append(m_ip_address).append(":").append(std::to_string(m_ip_port)).append("/");
    // rtspURL.append("rtsp://").append(m_ip_address).append(":").append(std::to_string(m_ip_port)).append("/RGB Camera");
    // rtspURL.append("rtsp://").append(m_ip_address).append(":").append(std::to_string(m_ip_port)).append("/Stereo Module");

    std::cout << "Connecting to " << rtspURL << std::endl;

    // Start the RTSP client
    TaskScheduler* scheduler = BasicTaskScheduler::createNew(/* 1000 */); // Check this later
    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);

    RTSPClient::responseBufferSize = 100000;
    m_rtspClient = RSRTSPClient::createNew(*env, rtspURL.c_str(), *this);

    if(m_rtspClient == NULL)
    {
        std::cout << "Failed to create a RTSP client for URL \"" << rtspURL << "\": " << env->getResultMsg() << "\n";
        throw std::runtime_error("Cannot create RTSP client");
    }

    m_rtspClient->sendListCommand(RSRTSPClient::continueAfterLIST);

    env->taskScheduler().doEventLoop(&m_eventLoopWatchVariable);
}

uint32_t chunks_allocated = 0;

void rs_net_device::doDevice() try {
    static uint32_t fps_frame_count = 0;
    static auto beginning = std::chrono::system_clock::now();

    std::cout << "Sensors support thread started" << std::endl;

    int frame_count = 0; 
    bool prev_sensor_state = false;

    while (m_eventLoopWatchVariable == 0) {
        for (auto sensor : sensors) {
            NetSensor netsensor = sensor.second;
            auto user_streams = netsensor->sw_sensor->get_active_streams();
            netsensor->current_state = (user_streams.size() > 0);

            if (netsensor->current_state != netsensor->prev_state) {
                // sensor state changed
                if (netsensor->current_state) {
                    std::cout << "Sensor enabled\n";
                    for (auto stream : user_streams) {
                        rs2::video_stream_profile s = static_cast<rs2::video_stream_profile>(stream);
                        std::cout << " Stream : " << std::setw(15) << rs2_stream_to_string(s.stream_type()) 
                                                  << std::setw(15) << rs2_format_to_string(s.format())      
                                                  << std::setw(15) << s.width() << "x" << s.height() << "x" << s.fps() << std::endl;

                        if (m_rtspClient) {
                            if (m_rtspClient->ready) {
                                m_rtspClient->startRTPSession(s);
                            } else {
                                std::cout << "Cannot start session: no session exists yet\n";
                                continue;
                            }
                        } else {
                            std::cout << "Cannot start session: no client exists yet\n";
                            continue;
                        }
                    }
                } else {
                    std::cout << "Sensor disabled\n";
                    // disable running RTP sessions
                    if (m_rtspClient) {
                        m_rtspClient->shutdownStream();
                    }
                }

                netsensor->prev_state = netsensor->current_state;
            }

            if (netsensor->current_state)  {
                if (m_rtspClient)  {
                    if (m_rtspClient->m_scs.subsession) {
                        RSSink* sink = (RSSink*)m_rtspClient->m_scs.subsession->sink;
                        if (sink) { // the session might be not created yet
                            auto start = std::chrono::system_clock::now();

                            uint32_t size = 0;
                            uint32_t offset = 0;
                            uint32_t total_size = 0;

                            while (offset < FRAME_SIZE) {
                                uint8_t* data = 0;
                                do {
                                    data = sink->getFrame();
                                } while (data == 0);
                                chunk_header_t* ch = (chunk_header_t*)data;

                                if (ch->offset < offset) break;

                                total_size += ch->size;
                                offset = ch->offset;
                                int ret = 0;
                                switch (ch->status & 3) {
                                case 0:
                                    ret = ch->size - CHUNK_HLEN;
                                    memcpy((void*)(m_frame_raw + offset), (void*)(data + CHUNK_HLEN), ret);
                                    break;
                                case 1: 
                                    ret = ZSTD_decompress((void*)(m_frame_raw + offset), CHUNK_SIZE, (void*)(data + CHUNK_HLEN), ch->size - CHUNK_HLEN); 
                                    break;
                                case 2: 
                                    ret = LZ4_decompress_safe((const char*)(data + CHUNK_HLEN), (char*)(m_frame_raw + offset), ch->size - CHUNK_HLEN, CHUNK_SIZE);
                                    ret = ch->size - CHUNK_HLEN;
                                    break;
                                case 3:
                                    std::cout << "JPEG not implemented yet" << std::endl;
                                    break;
                                }
                                size += ret;
                                // offset += CHUNK_SIZE;
                                offset += ret;

                                sink->popFrame();
                                delete [] data;
                            } 

                            uint8_t* frame_raw = new uint8_t[640*480*2];
                            memcpy(frame_raw, m_frame_raw, FRAME_SIZE);

                            auto end = std::chrono::system_clock::now();
                            std::chrono::duration<double> elapsed = end - start;
                            std::chrono::duration<double> total_time = end - beginning;
                            fps_frame_count++;
                            double fps;
                            if (total_time.count() > 0) fps = (double)fps_frame_count / (double)total_time.count();
                            else fps = 0;
                            std::cout << "Frame decompression time " << std::fixed << std::setw(5) << std::setprecision(2) 
                                                                    << elapsed.count() * 1000 << " ms, size " << total_size << " => " << size << ", FPS: " << fps << "\n";                            

                            if (total_time > std::chrono::seconds(1)) {
                                beginning = std::chrono::system_clock::now();
                                fps_frame_count = 0;
                            }

                            // std::cout << "Chunks: " << chunks_allocated << "\n"; 

                            if (frame_raw) {
                                // send it into device
                                netsensor->sw_sensor->on_video_frame(
                                    { 
                                        (void*)frame_raw, 
                                        [] (void* f) { delete [] (uint8_t*)f; }, 
                                        m_rtspClient->m_scs.subsession->videoWidth() * 1,   // IR:1 COLOR and DEPTH:2
                                        1,                                                  // IR:1 COLOR and DEPTH:2
                                        (double)time(NULL), 
                                        RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME, 
                                        ++frame_count, 
                                        sink->profile->get()
                                    }
                                );
                            }
                        } else std::cout << "No sink exists yet\n";
                    } // else std::cout << "No subsession exists yet\n";
                } else std::cout << "No client exists yet\n";
            }
        }        
    }

    std::cout << "Sensors support thread exited" << std::endl;
} catch (...) {
    std::cout << "Device support thread crashed.\n";
}

rs_net_device::rs_net_device(rs2::software_device sw_device, std::string ip_address)
    :m_device(sw_device), m_eventLoopWatchVariable(0), m_rtspClient(0)
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

    //TODO// Obtain number of sensors and their names via HTTP (possible with corresponding SDP to use later with DESCRIBE)

    // typedef void * (*THREADFUNCPTR)(void *);
    // if (pthread_attr_init(&m_th_rtp_attr) == 0) {
    //     if (pthread_attr_setschedpolicy(&m_th_rtp_attr, SCHED_RR) == 0) {
    //         struct sched_param sch_param = {99};
    //         if (pthread_attr_setschedparam(&m_th_rtp_attr, &sch_param) == 0) {
    //             if (pthread_create(&m_th_rtp, &m_th_rtp_attr, (THREADFUNCPTR) &rs_net_device::doRTP, this) == 0) {
    //                 std::cout << "RTP thread created\n";
    //             } else std::cout << "Cannot create thread\n";
    //         } else std::cout << "Cannot set scheduling priority\n";
    //     } else std::cout << "Cannot set scheduling policy\n";
    // } else std::cout << "Cannot initialize attributes\n";

    m_rtp = std::thread( [this](){ doRTP(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); 

    // This section constructs software device. 
    // rgb = std::make_shared<rs2::software_sensor>(m_device.add_sensor("COLOR"));
    // stereo = std::make_shared<rs2::software_sensor>(m_device.add_sensor("DEPTH"));

    intrinsics = { 640, 480, (float)640 / 2, (float)480 / 2, (float)640 / 2, (float)480 / 2, RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

    // rs2_intrinsics rgb_intrinsics = { 640, 480, (float)640 / 2, (float)480 / 2, (float)640 / 2, (float)480 / 2, RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };
    // rs2_intrinsics stereo_intrinsics = { 640, 480, (float)640 / 2, (float)480 / 2, (float)640 / 2, (float)480 / 2, RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

    // Color
    // sp = std::make_shared<rs2::stream_profile>(rgb->add_video_stream({ RS2_STREAM_COLOR, 0,  9, 640, 480,  6, 8, RS2_FORMAT_YUYV, rgb_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(rgb->add_video_stream({ RS2_STREAM_COLOR, 0, 19, 640, 480, 15, 8, RS2_FORMAT_YUYV, rgb_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(rgb->add_video_stream({ RS2_STREAM_COLOR, 0, 29, 640, 480, 30, 8, RS2_FORMAT_YUYV, rgb_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(rgb->add_video_stream({ RS2_STREAM_COLOR, 0, 39, 640, 480, 60, 8, RS2_FORMAT_YUYV, rgb_intrinsics }, true));

    // Depth
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_DEPTH, 0,  9, 640, 480,  6, 8, RS2_FORMAT_Z16, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_DEPTH, 0, 19, 640, 480, 15, 8, RS2_FORMAT_Z16, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_DEPTH, 0, 29, 640, 480, 30, 8, RS2_FORMAT_Z16, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_DEPTH, 0, 39, 640, 480, 60, 8, RS2_FORMAT_Z16, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_DEPTH, 0, 49, 640, 480, 90, 8, RS2_FORMAT_Z16, stereo_intrinsics }, true));

    // IR 1
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 1,  9, 640, 480,  6, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 1, 19, 640, 480, 15, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 1, 29, 640, 480, 30, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 1, 39, 640, 480, 60, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));

    // IR 2
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 2,  9, 640, 480,  6, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 2, 19, 640, 480, 15, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 2, 29, 640, 480, 30, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));
    // sp = std::make_shared<rs2::stream_profile>(stereo->add_video_stream({ RS2_STREAM_INFRARED, 2, 39, 640, 480, 60, 8, RS2_FORMAT_Y8, stereo_intrinsics }, true));

    m_frame_raw = new uint8_t[640*480*2];
    m_dev = std::thread( [this](){ doDevice(); } ); 
}

rs_net_device::~rs_net_device() 
{
    m_eventLoopWatchVariable = 1;
    if (m_rtp.joinable()) m_rtp.join();
    if (m_dev.joinable()) m_dev.join();

    if (m_frame_raw) {
        delete [] m_frame_raw;
        m_frame_raw = NULL;
    }
}

void RSRTSPClient::shutdownStream()
{
    UsageEnvironment& env = envir(); // alias

    // First, check whether any subsessions have still to be closed:
    if(m_scs.session != NULL)
    {
        Boolean someSubsessionsWereActive = False;
        MediaSubsessionIterator iter(*m_scs.session);
        MediaSubsession* subsession;

        while((subsession = iter.next()) != NULL)
        {
            if(subsession->sink != NULL)
            {
                Medium::close(subsession->sink);
                subsession->sink = NULL;

                if(subsession->rtcpInstance() != NULL)
                {
                    subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
                }

                someSubsessionsWereActive = True;
            }
        }

        if(someSubsessionsWereActive)
        {
            // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
            // Don't bother handling the response to the "TEARDOWN".
            sendTeardownCommand(*m_scs.session, NULL);
        }
    }

    std::cout << "Closing the stream.\n";
    Medium::close(this);
    // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.
}

unsigned RSRTSPClient::sendListCommand(responseHandler* responseHandler, Authenticator* authenticator) {
    std::cout << "Sending LIST command" << std::endl;

    if (fCurrentAuthenticator < authenticator) fCurrentAuthenticator = *authenticator;
    return sendRequest(new RequestRecord(++fCSeq, "LIST", responseHandler));
}

// static callback
void RSRTSPClient::continueAfterLIST(RTSPClient* rtspClient, int resultCode, char* resultString)
{
    return ((RSRTSPClient*)rtspClient)->continueAfterLIST(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterLIST(int resultCode, char* resultString)
{
    // std::cout << "Got response for the LIST command: " << std::endl << resultString << std::endl;
    sendDescribeCommand(RSRTSPClient::continueAfterDESCRIBE);
}

// static callback
void RSRTSPClient::continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString)
{
    return ((RSRTSPClient*)rtspClient)->continueAfterDESCRIBE(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterDESCRIBE(int resultCode, char* resultString)
{
    do {
        UsageEnvironment& env = envir(); // alias

        if(resultCode != 0) {
            envir() << "Failed to get a SDP description: " << resultString << "\n";
            delete[] resultString;
            throw std::runtime_error("Failed to get a SDP description");
            break;
        }

        char* const sdpDescription = resultString;
        envir() << "Got a SDP description:\n" << sdpDescription << "\n";

        // Create a media session object from this SDP description:
        // m_scs.session = MediaSession::createNew(env, sdpDescription);
        m_scs.session = RsMediaSession::createNew(env, sdpDescription);
        delete[] sdpDescription; // because we don't need it anymore

        if (m_scs.session == NULL) {
            envir() << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
            throw std::runtime_error("Malformed server response");
            break;
        } else if (!m_scs.session->hasSubsessions()) {
            envir() << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
            throw std::runtime_error("No profiles found");
            break;
        } else {
            envir() << "Session created " << m_scs.session->name() << "/" << m_scs.session->sessionDescription() << "/" << m_scs.session->controlPath() << "\n";
            ready = true;

            // Create the profiles
            m_scs.iter = new MediaSubsessionIterator(*m_scs.session);
            while (m_scs.subsession = m_scs.iter->next()) {
                uint32_t uid = 0;
                std::string sensorName = m_scs.subsession->attrVal_str("sensor");
                rs2_video_stream stream = {
                    (rs2_stream)m_scs.subsession->attrVal_unsigned("type"),
                    m_scs.subsession->attrVal_unsigned("index"),
                    uid++,
                    m_scs.subsession->videoWidth(),
                    m_scs.subsession->videoHeight(),
                    m_scs.subsession->videoFPS(),
                    m_scs.subsession->attrVal_unsigned("bpp"),
                    (rs2_format)m_scs.subsession->attrVal_unsigned("format"),
                    m_parent.intrinsics
                };

                // add the profile to the SW device
                m_parent.add_stream(sensorName, stream);

                // RSSink* sink = RSSink::createNew(env, *m_scs.subsession, url(), m_scs.subsession->videoWidth() * m_scs.subsession->videoHeight() * m_scs.subsession->videoFPS());
                // m_scs.subsession->sink = sink;
                // sink->profile = m_parent.add_stream(sensorName, stream);
            }
        }
        return;
    } while (0);

    // An unrecoverable error occurred with this stream.
    shutdownStream();
}

void RSRTSPClient::startRTPSession(rs2::video_stream_profile stream) {
    bool profile_found = false;

    // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
    // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
    // (Each 'subsession' will have its own data source.)
    m_scs.iter = new MediaSubsessionIterator(*m_scs.session);

    uint32_t stype = stream.stream_type();
    std::cout << "Looking  for " << stream.width() << "x" << stream.height() << "x" << stream.fps() << ", type " << stype << ", index " << stream.stream_index() << std::endl;
    while (m_scs.subsession = m_scs.iter->next()) {
        uint32_t videoType  = m_scs.subsession->attrVal_unsigned("type");
        uint32_t videoIndex = m_scs.subsession->attrVal_unsigned("index");
    std::cout << "Checking for " << m_scs.subsession->videoWidth() << "x" << m_scs.subsession->videoHeight() << "x" << m_scs.subsession->videoFPS() << ", type " << videoType << ", index " << videoIndex << std::endl;
        if ((m_scs.subsession->videoWidth()  == stream.width())  &&
            (m_scs.subsession->videoHeight() == stream.height()) &&
            (m_scs.subsession->videoFPS()    == stream.fps())    &&
            (videoType == stream.stream_type()) && 
            (videoIndex == stream.stream_index()))
        {
            envir() << "Profile match for " << m_scs.subsession->controlPath() << "\n";
            profile_found = true;

            int useSpecialRTPoffset = -1; // for supported codecs
            if (strcmp(m_scs.subsession->codecName(), "LZ4") == 0) {
                useSpecialRTPoffset = 0;
            }

            if (!m_scs.subsession->initiate(useSpecialRTPoffset)) {
                std::cout << "Failed to initiate the \"" << m_scs.subsession->controlPath() << "\" subsession: " << envir().getResultMsg() << "\n";
            } else {
                std::cout << "Initiated the '" << std::setw(10) << m_scs.subsession->controlPath() << "' " 
                                               << m_scs.subsession->mediumName()   << "/" 
                                               << m_scs.subsession->protocolName() << "/" 
                                               << m_scs.subsession->videoWidth() << "x" << m_scs.subsession->videoHeight() << "x" << m_scs.subsession->videoFPS() << " subsession (";
                if (m_scs.subsession->rtcpIsMuxed()) {
                    std::cout << "client port " << m_scs.subsession->clientPortNum();
                } else {
                    std::cout << "client ports " << m_scs.subsession->clientPortNum() << "-" << m_scs.subsession->clientPortNum() + 1;
                }
                std::cout << ") [" << m_scs.subsession->readSource()->name() << " : " << m_scs.subsession->readSource()->MIMEtype() << "]\n";

#if 0
#  define ENCODER_LZ4

#  if   defined(ENCODER_JPEG) // JPEG filter 
                FramedFilter* jpeg = JPEGDecodeFilter::createNew(envir(), m_scs.subsession->readSource());
                m_scs.subsession->addFilter(jpeg);
                std::cout << "Set filter [" << m_scs.subsession->readSource()->name() << "]\n";
#  elif defined(ENCODER_JPEG2000) // JPEG2000 filter 
                FramedFilter* jpeg2000 = JPEG2000DecodeFilter::createNew(envir(), m_scs.subsession->readSource());
                m_scs.subsession->addFilter(jpeg2000);
                std::cout << "Set filter [" << m_scs.subsession->readSource()->name() << "]\n";
#  elif defined(ENCODER_LZ4) // LZ4 filter 
                FramedFilter* lz4 = LZ4DecodeFilter::createNew(envir(), m_scs.subsession->readSource());
                m_scs.subsession->addFilter(lz4);
                std::cout << "Set filter [" << m_scs.subsession->readSource()->name() << "]\n";
#  endif
#endif

                // Continue setting up this subsession, by sending a RTSP "SETUP" command:
                sendSetupCommand(*m_scs.subsession, RSRTSPClient::continueAfterSETUP);
                break;
            }
        }
    }

    if (!profile_found) envir() << "Cannot match a profile\n";
}

// static callback
void RSRTSPClient::continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString)
{
    return ((RSRTSPClient*)rtspClient)->continueAfterSETUP(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterSETUP(int resultCode, char* resultString)
{
    UsageEnvironment& env = envir(); // alias
    
    // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
    // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
    // after we've sent a RTSP "PLAY" command.)
    RSSink* sink = RSSink::createNew(env, *m_scs.subsession, url(), m_scs.subsession->videoWidth() * m_scs.subsession->videoHeight() * m_scs.subsession->videoFPS());

    rs2_video_stream stream = {
        (rs2_stream)m_scs.subsession->attrVal_unsigned("type"),
        m_scs.subsession->attrVal_unsigned("index"),
        0,
        m_scs.subsession->videoWidth(),
        m_scs.subsession->videoHeight(),
        m_scs.subsession->videoFPS(),
        m_scs.subsession->attrVal_unsigned("bpp"),
        (rs2_format)m_scs.subsession->attrVal_unsigned("format"),
        m_parent.intrinsics
    };

    sink->profile = m_parent.get_profile(stream);

    m_scs.subsession->sink = sink;

    // do not wait for the out of order packets
    // m_scs.subsession->rtpSource()->setPacketReorderingThresholdTime(0); 

    // perhaps use your own custom "MediaSink" subclass instead
    if(m_scs.subsession->sink == NULL)
    {
        env << "Failed to create a data sink for the '" << m_scs.subsession->controlPath() << "' subsession: " << env.getResultMsg() << "\n";
    }
    else
    {
        env << "Created a data sink for the \"" << m_scs.subsession->controlPath() << "\" subsession\n";
        m_scs.subsession->miscPtr = this; // a hack to let subsession handler functions get the "RTSPClient" from the subsession
        m_scs.subsession->sink->startPlaying(*(m_scs.subsession->readSource()), subsessionAfterPlaying, m_scs.subsession);
        // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
        if(m_scs.subsession->rtcpInstance() != NULL)
        {
            m_scs.subsession->rtcpInstance()->setByeWithReasonHandler(subsessionByeHandler, m_scs.subsession);
        }

        // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
        if(m_scs.session->absStartTime() != NULL)
        {
            // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
            sendPlayCommand(*m_scs.session, continueAfterPLAY, m_scs.session->absStartTime(), m_scs.session->absEndTime());
        }
        else
        {
            m_scs.duration = m_scs.session->playEndTime() - m_scs.session->playStartTime();
            sendPlayCommand(*m_scs.session, continueAfterPLAY);
        }

    }
}

// static callback
void RSRTSPClient::continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString)
{
    return ((RSRTSPClient*)rtspClient)->continueAfterPLAY(resultCode, resultString);
}

// member
void RSRTSPClient::continueAfterPLAY(int resultCode, char* resultString)
{
    Boolean success = False;

    do
    {
        UsageEnvironment& env = envir(); // alias

        if(resultCode != 0)
        {
            env << "Failed to start playing session: " << resultString << "\n";
            break;
        }

        // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
        // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
        // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
        // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
        if(m_scs.duration > 0)
        {
            unsigned const delaySlop = 2; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
            m_scs.duration += delaySlop;
            unsigned uSecsToDelay = (unsigned)(m_scs.duration * 1000000);
            m_scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*)RSRTSPClient::streamTimerHandler, this);
        }

        env << "Started playing session";
        if(m_scs.duration > 0)
        {
            env << " (for up to " << m_scs.duration << " seconds)";
        }
        env << "...\n";

        success = True;
    } while(0);
    delete[] resultString;

    if(!success)
    {
        // An unrecoverable error occurred with this stream.
        shutdownStream();
    }
}

void RSRTSPClient::streamTimerHandler(void* clientData)
{
    RSRTSPClient* rtspClient = (RSRTSPClient*)clientData;

    rtspClient->m_scs.streamTimerTask = NULL;
    rtspClient->shutdownStream();
}


void RSRTSPClient::subsessionAfterPlaying(void* clientData)
{
    MediaSubsession* subsession = (MediaSubsession*)clientData;
    RSRTSPClient* rtspClient = (RSRTSPClient*)(subsession->miscPtr);

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
// #define RS_SINK_RECEIVE_BUFFER_SIZE 1048576
#define RS_SINK_RECEIVE_BUFFER_SIZE (CHUNK_SIZE + CHUNK_HLEN)

RSSink* RSSink::createNew(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId, uint32_t threshold)
{
    return new RSSink(env, subsession, streamId, threshold);
}

RSSink::RSSink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId, uint32_t threshold)
    : MediaSink(env), fSubsession(subsession), m_threshold(threshold)
{
    fStreamId = strDup(streamId);
    fReceiveBuffer = new u_int8_t[RS_SINK_RECEIVE_BUFFER_SIZE];
    chunks_allocated++;
}

RSSink::~RSSink()
{
    if (fReceiveBuffer) delete[] fReceiveBuffer;
    std::lock_guard<std::mutex> lck (m_frames_mutex);
    while (!m_frames.empty()) {
        delete[] m_frames.front();
        m_frames.pop();
    }
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
        std::lock_guard<std::mutex> lck (m_frames_mutex);
        // if (m_threshold / CHUNK_SIZE < m_frames.size()) {
        //     std::cout << "Dropping " << m_frames.size() << " chunks" << std::endl;
        //     chunks_allocated -= m_frames.size();
        //     while (!m_frames.empty()) {
        //         delete [] m_frames.front();
        //         m_frames.pop();
        //     }
        // }
        m_frames.push(fReceiveBuffer);
        fReceiveBuffer = new u_int8_t[RS_SINK_RECEIVE_BUFFER_SIZE];
        chunks_allocated++;
    }
 
    // Then continue, to request the next frame of data:

    continuePlaying();
    // nextTask() = envir().taskScheduler().scheduleDelayedTask(10, (TaskFunc*)getNextFrame, this);
}

void RSSink::getNextFrame(RSSink* sink) {
    //// std::cout << "RsDeviceSource::waitForFrame" << std::endl;
    sink->continuePlaying();
};

Boolean RSSink::continuePlaying()
{
    if(fSource == NULL)
        return False; // sanity check (should not happen)

    // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
    fSource->getNextFrame(fReceiveBuffer, RS_SINK_RECEIVE_BUFFER_SIZE, afterGettingFrame, this, onSourceClosure, this);
    return True;
}

uint8_t* RSSink::getFrame() {
    if (m_frames.empty()) return NULL;
    std::lock_guard<std::mutex> lck (m_frames_mutex);
    return m_frames.front();;
}

void RSSink::popFrame() {
    if (m_frames.empty()) return;
    std::lock_guard<std::mutex> lck (m_frames_mutex);
    m_frames.pop();
    chunks_allocated--;
}

