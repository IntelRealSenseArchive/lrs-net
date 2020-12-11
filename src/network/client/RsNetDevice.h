// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#include <option.h>
#include <software-device.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <list>

#include "JPEG2000DecodeFilter.h"
#include "JPEGDecodeFilter.h"
#include "LZ4DecodeFilter.h"
#include "LZ4VideoRTPSource.h"

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

#define MAX_ACTIVE_STREAMS 4

#define NUM_OF_SENSORS 2

#define POLLING_SW_DEVICE_STATE_INTERVAL 100

#define DEFAULT_PROFILE_FPS 15

#define DEFAULT_PROFILE_WIDTH 424

#define DEFAULT_PROFILE_HIGHT 240

#define DEFAULT_PROFILE_COLOR_FORMAT RS2_FORMAT_RGB8 

#define FRAME_SIZE (640*480*2)

// forward
class rs_net_device; 
class RSRTSPClient;
class RsMediaSession;

class SafeQueue {
public:    
    SafeQueue() {};
    ~SafeQueue() {};

    void push(uint8_t* e) {
        std::lock_guard<std::mutex> lck (m);
        return q.push(e);
    };

    uint8_t* front() {
        std::lock_guard<std::mutex> lck (m);
        if (q.empty()) return NULL;
        return q.front();
    };

    void pop() {
        std::lock_guard<std::mutex> lck (m);
        if (q.empty()) return;
        return q.pop();
    };

    bool empty() {
        std::lock_guard<std::mutex> lck (m);
        return q.empty(); 
    };

private:
    std::queue<uint8_t*> q;
    std::mutex m;
};

using SoftSensor      = std::shared_ptr<rs2::software_sensor>;
using StreamProfile   = std::shared_ptr<rs2::video_stream_profile>;
using ProfileQPair    = std::pair<rs2::stream_profile, SafeQueue*>;
using ProfileQMap     = std::map<rs2::stream_profile, SafeQueue*>;

class RSSink : public MediaSink {
public:
    static RSSink* createNew(UsageEnvironment& env,
                                MediaSubsession& subsession, // identifies the kind of data that's being received
                                char const* streamId,        // identifies the stream itself
                                SafeQueue* q,
                                uint32_t threshold);         // number of stored bytes for the drop

private:
    RSSink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamIdm, SafeQueue* q, uint32_t threshold); // called only by "createNew()"
    virtual ~RSSink();

    static void afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds); // callback
    void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds); // member

private:
    static void getNextFrame(RSSink* sink);

    // redefined virtual functions:
    virtual Boolean continuePlaying();

private:
    uint32_t m_threshold;

    uint8_t* fReceiveBuffer;
    
    SafeQueue* m_frames;
    
    MediaSubsession& fSubsession;
    char* fStreamId;
};

class rs_net_sensor {
public: 
    rs_net_sensor(rs2::software_device device, std::string name)
        : m_sw_device(device), m_name(name), m_streaming(false), m_dev_flag(false) 
    {
        m_eventLoopWatchVariable = 0;

        m_sw_sensor = std::make_shared<rs2::software_sensor>(m_sw_device.add_sensor(m_name));
        m_frame_raw = new uint8_t[640*480*2];
    };

    ~rs_net_sensor() {
        if (m_frame_raw) delete [] m_frame_raw;
    };

    void set_mrl(std::string mrl) { m_mrl  = mrl; };
    void add_profile(rs2_video_stream stream) { 
        StreamProfile sp = std::make_shared<rs2::video_stream_profile>(m_sw_sensor->add_video_stream(stream, false));
    };

    bool is_streaming() { return m_sw_sensor->get_active_streams().size() > 0; };

    void start() {
        m_rtp = std::thread( [&](){ doRTP(); });
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        // m_dev = std::thread( [&](){ doDevice(); });
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    };

    void doRTP();
    static void doControl(void* clientData) {
        rs_net_sensor* sensor = (rs_net_sensor*)clientData;
        sensor->doControl(); 
    };
    void doControl();
    void doDevice();

private:    
    rs2::software_device m_sw_device;

    std::string    m_name;
    std::string    m_mrl;

    SoftSensor     m_sw_sensor;
    ProfileQMap    m_stream_q;

    std::thread    m_rtp;
    std::thread    m_dev;
    volatile bool  m_dev_flag;

    RSRTSPClient*  m_rtspClient;
    char m_eventLoopWatchVariable;

    uint8_t* m_frame_raw;

    bool m_streaming;

    UsageEnvironment* m_env;
};
using NetSensor = std::shared_ptr<rs_net_sensor>;

// #define COMPRESSION_ENABLED
// #define COMPRESSION_ZSTD

#define CHUNK_SIZE (1024*2)
#pragma pack (push, 1)
typedef struct chunk_header{
    uint16_t size;
    uint32_t offset;
    uint8_t  status;
    uint8_t  meta_id;
    uint64_t meta_data;
} chunk_header_t;
#pragma pack(pop)
#define CHUNK_HLEN (sizeof(chunk_header_t))

class slib {
public:
    slib() {};
   ~slib() {};
    
    static uint64_t profile2key(rs2::video_stream_profile stream) {
        return 
            ((uint64_t)stream.stream_type()  & 0x00FF) << 56 | 
            ((uint64_t)stream.stream_index() & 0x00FF) << 48 | 
            ((uint64_t)stream.width()        & 0xFFFF) << 32 | 
            ((uint64_t)stream.height()       & 0xFFFF) << 16 | 
            ((uint64_t)stream.fps()          & 0x00FF) <<  8 | 
            ((uint64_t)stream.format()       & 0x00FF); 
    };

    static rs2_video_stream key2stream(uint64_t key) {
        rs2_video_stream stream;
        stream.type   = (rs2_stream)(key >> 56);
        stream.index  = key << 8  >> 56;
        stream.width  = key << 16 >> 48;
        stream.height = key << 32 >> 48;
        stream.fps    = key << 48 >> 56;
        stream.fmt    = (rs2_format)(key << 56 >> 56);

        stream.bpp    = (stream.type == RS2_STREAM_INFRARED) ? 1 : 2;

        return stream;
    };
};

class RSRTSPClient : public RTSPClient {
public:
    static RSRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL) {
        return new RSRTSPClient(env, rtspURL);
    }

protected:
    RSRTSPClient(UsageEnvironment& env, char const* rtspURL)
        : RTSPClient(env, rtspURL, 0, "", 0, -1), m_pqm_it(NULL) {}

    virtual ~RSRTSPClient() {};

public:
    void playStreams(ProfileQMap pqm);
    void shutdownStream();

    void prepareSession();
    void playSession();

    static void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterDESCRIBE(int resultCode, char* resultString); // member

    static void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterSETUP(int resultCode, char* resultString); // member

    static void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterPLAY(int resultCode, char* resultString); // member

    static void subsessionAfterPlaying(void* clientData);
    static void subsessionByeHandler(void* clientData, char const* reason);
    static void streamTimerHandler(void* clientData);

private:
    RsMediaSession*  m_session;
    MediaSubsession* m_subsession;

    std::string m_sdp;

    ProfileQMap m_pqm;
    ProfileQMap::iterator* m_pqm_it;
};

class rs_net_device
{
public:
    rs_net_device(rs2::software_device sw_device, std::string ip_address);
   ~rs_net_device() {};

    rs2_intrinsics intrinsics;

private:
    std::string  m_ip_address;
    unsigned int m_ip_port;

    rs2::software_device m_device;

    std::vector<NetSensor> sensors;
};
