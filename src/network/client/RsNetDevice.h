// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include <option.h>
#include <software-device.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <list>

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#include <jpeglib.h>
#include <openjpeg.h>

#define MAX_ACTIVE_STREAMS 4

#define NUM_OF_SENSORS 2

#define POLLING_SW_DEVICE_STATE_INTERVAL 100

#define DEFAULT_PROFILE_FPS 15

#define DEFAULT_PROFILE_WIDTH 424

#define DEFAULT_PROFILE_HIGHT 240

#define DEFAULT_PROFILE_COLOR_FORMAT RS2_FORMAT_RGB8 

#define FRAME_SIZE 640*480*2

class mem_stream1 {
public:
    mem_stream1(uint8_t* buffer, uint64_t size) : m_stream(NULL), m_buffer(buffer), m_pos(buffer), m_size(size) { m_stream_buffer() = this; };
   ~mem_stream1() {};
    
    static void     close_cb (void* stream)                            { return m_stream_buffer()->close(); };
    static size_t   read_cb  (void* buffer, size_t size, void* stream) { return m_stream_buffer()->read(buffer, size); };
    static size_t   write_cb (void* buffer, size_t size, void* stream) { return m_stream_buffer()->write(buffer, size); };
    static int64_t  skip_cb  (int64_t num, void* stream)               { return m_stream_buffer()->skip(num); };
    static int64_t  seek_cb  (int64_t num, void* stream)               { return m_stream_buffer()->seek(num); };

    void     close() {};
    size_t   read(void* buffer, size_t size)  { 
        // std::cout << "Read for " << size << " bytes\n"; 
        if (size > m_size) size = m_size; memcpy(buffer, m_pos, size); m_pos += size; return size; };
    size_t   write(void* buffer, size_t size) { 
        // static int fnum = 1;
        // FILE* f = 0;
        // char fname[64];
        // sprintf(fname, "/tmp/wr%04u.j2k", fnum++);
        // f = fopen(fname, "wb");
        // fwrite(buffer, 1, size, f);
        // fclose(f);
        // std::cout << "Write for " << size << " bytes\n"; 
        memcpy(m_pos, buffer, size); m_pos += size; m_size += size; return size; 
    };
    int64_t  skip(int64_t size) { std::cout << "Skip for " << size << " bytes\n"; m_pos += size; return m_pos - m_buffer; };
    int64_t  seek(int64_t size) { 
        // std::cout << "Seek for " << size << " bytes\n"; 
        if (m_pos + size < m_buffer + m_size) m_pos += size;
        else m_pos = m_buffer + m_size;
        return m_pos - m_buffer; 
    };

    int64_t size() { return /* m_pos - m_buffer */ m_size; }

private:
    void* m_stream;

    uint8_t* m_buffer;
    uint8_t* m_pos;

    int64_t m_size;

    static mem_stream1*& m_stream_buffer() {
        static mem_stream1* ms = 0;

        return ms;
    }
};

class jpeg2k1 {
public:
    jpeg2k1() {};
   ~jpeg2k1() {};

    static void print(const char *msg, void *priv) {
        std::cout << "jpeg2k: " << msg << std::endl;
    };

    static int compress(uint8_t * yuyv, uint32_t width, uint32_t height, uint8_t * j2k, uint32_t size) { 
        int ret = -1; // Failure

        opj_cparameters_t parameters;
        opj_set_default_encoder_parameters(&parameters);
        if(parameters.tcp_numlayers == 0)
        {
            parameters.tcp_rates[0] = 0; /* MOD antonin : losslessbug */
            parameters.tcp_numlayers++;
            parameters.cp_disto_alloc = 1;
        }

#define nr_comp 3

        uint32_t sub_dx = (unsigned int)parameters.subsampling_dx;
        uint32_t sub_dy = (unsigned int)parameters.subsampling_dy;
        opj_image_cmptparm_t cmptparm[nr_comp] = {0};
        for (int i = 0; i < nr_comp; i++) {
            cmptparm[i].prec = 8;
            cmptparm[i].bpp = 8;
            cmptparm[i].sgnd = 0;
            cmptparm[i].dx = sub_dx;
            cmptparm[i].dy = sub_dy;
            cmptparm[i].w = width;
            cmptparm[i].h = height;
        }
        opj_image_t *image = opj_image_create(nr_comp, cmptparm, OPJ_CLRSPC_SYCC);
        if (image) {
            // TODO: initialize the image from the YUYV data supplied
            // for (uint32_t i = 0; i < width * height; i++) {
            //     unsigned int compno;
            //     for(compno = 0; compno < 3; compno++) {
            //         image->comps[compno].data[i] = 0;
            //     }
            // }        
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (x%2 == 0) {
                        image->comps[0].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 0]; // Y
                        image->comps[1].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 1]; // U
                        image->comps[2].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 3]; // V
                    } else {
                        image->comps[0].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 0]; // Y
                        image->comps[1].data[y * width + x] = yuyv[y * width * 2 + x * 2 - 1]; // U
                        image->comps[2].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 1]; // V
                    }
                }
            }

            image->x0 = 0;
            image->y0 = 0;
            image->x1 = (width - 1) * sub_dx + 1;
            image->y1 = (height - 1) * sub_dy + 1;            

            parameters.tcp_mct = image->numcomps == nr_comp ? 1 : 0;
            opj_codec_t* l_codec = opj_create_compress(OPJ_CODEC_J2K);
            if (l_codec) {
                // opj_set_info_handler   (l_codec, print, NULL);
                // opj_set_warning_handler(l_codec, print, NULL);
                // opj_set_error_handler  (l_codec, print, NULL);

                if (opj_setup_encoder(l_codec, &parameters, image)) {
                    opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_FALSE);
                    if (l_stream) {
                        mem_stream1 ms(j2k, size);

                        opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn) NULL);
                        opj_stream_set_user_data_length(l_stream, size);
                        opj_stream_set_read_function(l_stream, (opj_stream_read_fn) mem_stream1::read_cb);
                        opj_stream_set_write_function(l_stream, (opj_stream_write_fn) mem_stream1::write_cb);
                        opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn) mem_stream1::skip_cb);
                        opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn) mem_stream1::seek_cb);

                        if (opj_start_compress(l_codec,image,l_stream)) {
                            if (opj_encode(l_codec, l_stream)) {
                                if (opj_end_compress(l_codec, l_stream)) {
                                    ret = ms.size(); // Success
                                    std::cout << "Compression done: " << ret << std::endl;
                                } else print("opj_end_compress failed", NULL);
                            } else print("opj_encode failed", NULL);
                        } else print("opj_start_compress failed", NULL);
                        opj_stream_destroy(l_stream);
                    } else print("opj_stream_create failed", NULL);
                } else print("opj_setup_encoder failed", NULL);
                opj_destroy_codec(l_codec);
            } else print("opj_create_compress failed", NULL);
            opj_image_destroy(image);
        } else print("opj_image_create failed", NULL);

        return ret; 
    };

    static int decompress(uint8_t * j2k, uint32_t size, uint8_t * yuyv) { 
        int ret  = -1;

        opj_dparameters_t dparameters;

        opj_image_t *image = {0};

        opj_codec_t* d_codec = opj_create_decompress(OPJ_CODEC_J2K);
        // opj_set_info_handler   (d_codec, print, NULL);
        // opj_set_warning_handler(d_codec, print, NULL);
        // opj_set_error_handler  (d_codec, print, NULL);

        if (opj_setup_decoder(d_codec, &dparameters)) {
            opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_TRUE);
            if (l_stream) {
                mem_stream1 ms(j2k, size);

                opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn) NULL);
                opj_stream_set_user_data_length(l_stream, size);
                opj_stream_set_read_function(l_stream, (opj_stream_read_fn) mem_stream1::read_cb);
                opj_stream_set_write_function(l_stream, (opj_stream_write_fn) mem_stream1::write_cb);
                opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn) mem_stream1::skip_cb);
                opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn) mem_stream1::seek_cb);

                if (opj_read_header(l_stream, d_codec, &image)) {
                    if (opj_decode(d_codec, l_stream, image)) {
                        if (opj_end_decompress(d_codec, l_stream)) {

                            ret = (image->x1 - image->x0) * (image->y1 - image->y0) * image->numcomps;
                            // TODO: convert image to the YUYV buffer supplied
                            int width = image->x1 - image->x0;
                            int height = image->y1 - image->y0;
                            for (int y = 0; y < height; y++) {
                                for (int x = 0; x < width; x++) {
                                    if (x%2 == 0) {
                                        yuyv[y * width * 2 + x * 2 + 0] = image->comps[0].data[y * width + x]; // Y
                                        yuyv[y * width * 2 + x * 2 + 1] = image->comps[1].data[y * width + x]; // U
                                        yuyv[y * width * 2 + x * 2 + 3] = image->comps[2].data[y * width + x]; // V
                                    } else {
                                        yuyv[y * width * 2 + x * 2 + 0] = image->comps[0].data[y * width + x]; // Y
                                        // image->comps[1].data[y * width + x] = yuyv[y * width * 2 + x * 2 - 1]; // U
                                        // image->comps[2].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 1]; // V
                                    }
                                }
                            }
                            ret = FRAME_SIZE;
                        } else print("opj_end_decompress failed", NULL);
                    } else print("opj_decode failed", NULL);
                } else print("opj_read_header failed", NULL);

                opj_stream_destroy(l_stream);
            } else print("opj_stream_create failed", NULL);
        } else print("opj_stream_create failed", NULL);

        opj_destroy_codec(d_codec);

        if (image) opj_image_destroy(image);

        return ret;
    };

private:
 
};

class JPEG2000DecodeFilter : public FramedFilter
{
public:
    static JPEG2000DecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEG2000DecodeFilter(t_env, source); };

protected:
    JPEG2000DecodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) { m_framebuf = new uint8_t[FRAME_SIZE]; }
    virtual ~JPEG2000DecodeFilter() { delete[] m_framebuf; };

private:

    uint8_t* m_framebuf; // frame buffer for plain YUYV image from the camera

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEG2000DecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds)
    {
        static uint32_t fnum = 1;
        char fname[32] = {0};
        FILE* f = 0;
        
        // sprintf(fname, "/tmp/in%04u.j2k", fnum++);
        // f = fopen(fname, "wb");
        // fwrite(m_framebuf, 1, frameSize, f);
        // fclose(f);

        auto start = std::chrono::system_clock::now();
        // memcpy(fTo, m_framebuf, frameSize);
        jpeg2k1::decompress(m_framebuf, frameSize, fTo);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;

        std::cout << "Frame decompression time " << elapsed.count() * 1000 << "ms,\treceived size " << frameSize << "\n";
        fFrameSize = FRAME_SIZE;

        afterGetting(this);
    }

protected:
    virtual char const* MIMEtype() const { if (inputSource()) inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); if (inputSource() != NULL) inputSource()->stopGettingFrames(); };
};

class JPEGDecodeFilter : public FramedFilter
{
public:
    static JPEGDecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGDecodeFilter(t_env, source); };

protected:
    JPEGDecodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) { m_framebuf = new uint8_t[FRAME_SIZE]; }
    virtual ~JPEGDecodeFilter() { delete[] m_framebuf; };

private:

    uint8_t* m_framebuf; // frame buffer for plain YUYV image from the camera

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEGDecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds)
    {
        static uint32_t fnum = 1;
        char fname[32] = {0};
        FILE* f = 0;
//#define USE_ABBREVIATION
#ifdef USE_ABBREVIATION
        decompress(m_framebuf, frameSize, fTo);
        fFrameSize = FRAME_SIZE;
#else

        auto start = std::chrono::system_clock::now();

        // sprintf(fname, "/tmp/in%04u", fnum++);
        // f = fopen(fname, "w");
        // fwrite(m_framebuf, 1, frameSize, f);
        // fclose(f);

        // find the second SOI marker to use the original headers
        uint32_t i = 1;
        while (i < frameSize) {
            if ((m_framebuf[i] == 0xFF) && (m_framebuf[i+1] == 0xD8)) break;
            i++;
        }

        // memcpy(fTo, &m_framebuf[i], frameSize - i + 1);
        // fFrameSize = frameSize - i + 1;

        // sprintf(fname, "/tmp/in%04u", fnum++);
        // f = fopen(fname, "w");
        // fwrite(fTo, 1, fFrameSize, f);
        // fclose(f);

        decompress(&m_framebuf[i], frameSize - i + 1, fTo);
        fFrameSize = FRAME_SIZE;

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;

        // std::cout << "Frame decompression time " << elapsed.count() * 1000 << "ms,\tfrom size " << frameSize << "\n";

        // f = fopen("/tmp/mjpeg", "a+");
        // fwrite(fTo, 1, fFrameSize, f);
        // fclose(f);
#endif
        afterGetting(this);
    }


    int decompress(unsigned char* in, int t_compressedSize, unsigned char* out)
    {
        struct jpeg_decompress_struct m_dinfo = {0};
        struct jpeg_error_mgr m_jerr = {0};

        m_dinfo.err = jpeg_std_error(&m_jerr);
        jpeg_create_decompress(&m_dinfo);
        // m_dinfo.out_color_space = JCS_UNKNOWN;

        jpeg_mem_src(&m_dinfo, in, t_compressedSize);
        jpeg_read_header(&m_dinfo, TRUE);

        std::vector<uint8_t> row(640 * 3);
        JSAMPROW row_pointer[1];
        row_pointer[0] = &row[0];

        jpeg_start_decompress(&m_dinfo);
        while(m_dinfo.output_scanline < m_dinfo.output_height)
        {
            int numLines = jpeg_read_scanlines(&m_dinfo, row_pointer, 1);
            for(int i = 0; i < m_dinfo.output_width; i += 2)
            {
                out[i * 2 + 0] = row[i * 3 + 0]; // Y
                out[i * 2 + 1] = row[i * 3 + 1]; // U
                out[i * 2 + 2] = row[i * 3 + 3]; // Y
                out[i * 2 + 3] = row[i * 3 + 2]; // V
            }
            out += m_dinfo.output_width * 2;
        }
        jpeg_finish_decompress(&m_dinfo);
        jpeg_destroy_decompress(&m_dinfo);

        return 0;
    }

protected:
    virtual char const* MIMEtype() const { if (inputSource()) inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); if (inputSource() != NULL) inputSource()->stopGettingFrames(); };
};

// Define a class to hold per-stream state that we maintain throughout each stream's lifetime:
class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  MediaSubsessionIterator* iter;
  MediaSession* session;
  MediaSubsession* subsession;
  TaskToken streamTimerTask;
  double duration;
};

class ip_sensor
{
public:
    ip_sensor() : is_enabled(false) {};
    ~ip_sensor() {};

    std::shared_ptr<rs2::software_sensor> sw_sensor; // TODO: remove smart ptr here
    std::list<long long int> active_streams_keys;
    std::map<rs2_option, float> sensors_option;

    bool is_enabled;

//    RsRtsp* rtsp_client; // TODO: get smart ptr from rtsp client creator
};

// Define a data sink (a subclass of "MediaSink") to receive the data for each subsession (i.e., each audio or video 'substream').
// In practice, this might be a class (or a chain of classes) that decodes and then renders the incoming audio or video.
// Or it might be a "FileSink", for outputting the received data into a file (as is done by the "openRTSP" application).
// In this example code, however, we define a simple 'dummy' sink that receives incoming data, but does nothing with it.

class RSSink : public MediaSink
{
public:
    static RSSink* createNew(UsageEnvironment& env,
                                MediaSubsession& subsession, // identifies the kind of data that's being received
                                char const* streamId = NULL); // identifies the stream itself (optional)

    uint8_t* getFrame();

private:
    RSSink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamId); // called only by "createNew()"
    virtual ~RSSink();

    static void afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds); // callback
    void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds); // member

private:
    // redefined virtual functions:
    virtual Boolean continuePlaying();

private:
    uint8_t* fReceiveBuffer;
    
    std::mutex m_frames_mutex;
    std::queue<uint8_t*> m_frames;

    MediaSubsession& fSubsession;
    char* fStreamId;
};

class RSRTSPClient : public RTSPClient
{
public:
    static RSRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL);

protected:
    RSRTSPClient(UsageEnvironment& env, char const* rtspURL); // called only by createNew();
    virtual ~RSRTSPClient();

public:
    StreamClientState m_scs;

    void startRTPSession(rs2::video_stream_profile stream);

    void shutdownStream();

    static void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterDESCRIBE(int resultCode, char* resultString); // member

    static void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterSETUP(int resultCode, char* resultString); // member

    static void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterPLAY(int resultCode, char* resultString); // member

    static void subsessionAfterPlaying(void* clientData);
    static void subsessionByeHandler(void* clientData, char const* reason);
    static void streamTimerHandler(void* clientData);

    bool ready;
};

class rs_net_device
{

public:
    rs_net_device(rs2::software_device sw_device, std::string ip_address);
   ~rs_net_device();

    // ip_sensor* remote_sensors[NUM_OF_SENSORS];

private:
    std::string  m_ip_address;
    unsigned int m_ip_port;

    rs2::software_device m_device;

    std::thread m_rtp;
    std::thread m_dev;

    RSRTSPClient* m_rtspClient;
    char m_eventLoopWatchVariable;

    void doRTP();

    void doDevice();
    std::shared_ptr<rs2::software_sensor> rgb;
    std::shared_ptr<rs2::stream_profile>  sp;

    std::mutex m_mutex;
    std::condition_variable m_init_done;

    // bool is_device_alive;

    // //todo: consider wrapp all maps to single container
    // //std::map<long long int, std::shared_ptr<rs_rtp_stream>> streams_collection;

    // std::map<long long int, std::thread> inject_frames_thread;

    // //std::map<long long int, rs_rtp_callback*> rtp_callbacks;

    // std::thread sw_device_status_check;

    // // bool init_device_data(rs2::software_device sw_device);

    // void polling_state_loop();

    // //void inject_frames_loop(std::shared_ptr<rs_rtp_stream> rtp_stream);

    // void stop_sensor_streams(int sensor_id);

    // void update_sensor_state(int sensor_index, std::vector<rs2::stream_profile> updated_streams, bool recover);
    
    // void set_option_value(int sensor_index, rs2_option opt, float val);
    // float get_option_value(int sensor_index, rs2_option opt);

    // std::vector<rs2_video_stream> query_streams(int sensor_id);

    // std::vector<IpDeviceControlData> get_controls(int sensor_id);

    // void recover_rtsp_client(int sensor_index);

    // // default device stream index per type + sensor_index
    // // streams will be loaded at runtime so here the place holder  
    // std::map<std::pair<rs2_stream,int>,int> default_streams = 
    // { 
    //     { std::make_pair(rs2_stream::RS2_STREAM_COLOR,0),-1},
    //     { std::make_pair(rs2_stream::RS2_STREAM_DEPTH,0),-1}
    // };

};
