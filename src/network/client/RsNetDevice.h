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


#define MAX_ACTIVE_STREAMS 4

#define NUM_OF_SENSORS 2

#define POLLING_SW_DEVICE_STATE_INTERVAL 100

#define DEFAULT_PROFILE_FPS 15

#define DEFAULT_PROFILE_WIDTH 424

#define DEFAULT_PROFILE_HIGHT 240

#define DEFAULT_PROFILE_COLOR_FORMAT RS2_FORMAT_RGB8 

#define FRAME_SIZE 640*480*2

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
