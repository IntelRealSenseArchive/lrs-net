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

#define MAX_ACTIVE_STREAMS 4

#define NUM_OF_SENSORS 2

#define POLLING_SW_DEVICE_STATE_INTERVAL 100

#define DEFAULT_PROFILE_FPS 15

#define DEFAULT_PROFILE_WIDTH 424

#define DEFAULT_PROFILE_HIGHT 240

#define DEFAULT_PROFILE_COLOR_FORMAT RS2_FORMAT_RGB8 

#define FRAME_SIZE (640*480*2)

class rs_net_device; // forward

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

// class ip_sensor
// {
// public:
//     ip_sensor() : is_enabled(false) {};
//     ~ip_sensor() {};

//     std::shared_ptr<rs2::software_sensor> sw_sensor; // TODO: remove smart ptr here
//     std::list<long long int> active_streams_keys;
//     std::map<rs2_option, float> sensors_option;

//     bool is_enabled;

// //    RsRtsp* rtsp_client; // TODO: get smart ptr from rtsp client creator
// };

using SoftSensor     = std::shared_ptr<rs2::software_sensor>;
using StreamProfile  = std::shared_ptr<rs2::stream_profile>;
using StreamProfiles = std::shared_ptr<std::vector<StreamProfile>>;
using ActiveProfiles = std::map<std::shared_ptr<StreamProfile>, MediaSubsession*>;

typedef struct rs_net_sensor {
// public:    
    std::string    name;
    SoftSensor     sw_sensor;
    StreamProfiles stream_profiles;
    ActiveProfiles active_profiles;

// private:
    bool current_state;
    bool prev_state;

    MediaSubsessionIterator* iter;
    MediaSession* session;
} rs_net_sensor;

using NetSensor      = std::shared_ptr<rs_net_sensor>;

#define COMPRESSION_ENABLED
#define COMPRESSION_ZSTD

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

class RSSink : public MediaSink
{
public:
    static RSSink* createNew(UsageEnvironment& env,
                                MediaSubsession& subsession, // identifies the kind of data that's being received
                                char const* streamId,        // identifies the stream itself
                                uint32_t threshold);         // number of stored bytes for the drop

    uint8_t* getFrame();
    void     popFrame();

    StreamProfile profile;

private:
    RSSink(UsageEnvironment& env, MediaSubsession& subsession, char const* streamIdm, uint32_t threshold); // called only by "createNew()"
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
    
    std::mutex m_frames_mutex;
    std::queue<uint8_t*> m_frames;
    
    MediaSubsession& fSubsession;
    char* fStreamId;
};

class RSRTSPClient : public RTSPClient
{
public:
    static RSRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL, rs_net_device& parent);

protected:
    RSRTSPClient(UsageEnvironment& env, char const* rtspURL, rs_net_device& parent); // called only by createNew();
    virtual ~RSRTSPClient();

    virtual Boolean setRequestFields(RequestRecord* request,
                    char*& cmdURL, Boolean& cmdURLWasAllocated,
                    char const*& protocolStr,
                    char*& extraHeaders, Boolean& extraHeadersWereAllocated) {
        if(strcmp(request->commandName(), "LIST") == 0) {
            // behave like an OPTIONS command without the session
            extraHeaders = strDup("");
            extraHeadersWereAllocated = True;
            return True;
        }

        return RTSPClient::setRequestFields(request, cmdURL, cmdURLWasAllocated, protocolStr, extraHeaders, extraHeadersWereAllocated);
    };

public:
    StreamClientState m_scs;

    void startRTPSession(rs2::video_stream_profile stream);

    void shutdownStream();

    unsigned sendListCommand(responseHandler* responseHandler, Authenticator* authenticator = NULL);

    static void continueAfterLIST(RTSPClient* rtspClient, int resultCode, char* resultString); // callback
    void continueAfterLIST(int resultCode, char* resultString); // member

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
    rs_net_device& m_parent;
};

class rs_net_device
{

public:
    rs_net_device(rs2::software_device sw_device, std::string ip_address);
   ~rs_net_device();

    // ip_sensor* remote_sensors[NUM_OF_SENSORS];

    StreamProfile add_stream(std::string sensor_name, rs2_video_stream stream) {
        if (sensors.find(sensor_name) == sensors.end()) {
            NetSensor netsensor(new rs_net_sensor);
            netsensor->sw_sensor = std::make_shared<rs2::software_sensor>(m_device.add_sensor(sensor_name));

            StreamProfiles profiles(new std::vector<StreamProfile>);
            netsensor->stream_profiles = profiles;

            sensors.insert(std::pair<std::string, NetSensor>(sensor_name, netsensor));
        }

        NetSensor netsensor = sensors.find(sensor_name)->second;
        StreamProfile sp = std::make_shared<rs2::stream_profile>(netsensor->sw_sensor->add_video_stream(stream, true));
        netsensor->stream_profiles->emplace_back(sp);

        return sp;
    };

    StreamProfile get_profile(rs2_video_stream stream) {
        for (auto sensor : sensors) {
            for (auto p : *(sensor.second->stream_profiles)) {
                auto profile = p->as<rs2::video_stream_profile>();
                if (profile.stream_index() == stream.index &&
                    profile.stream_type()  == stream.type  &&
                    profile.format()       == stream.fmt   &&
                    profile.fps()          == stream.fps   &&
                    profile.width()        == stream.width &&
                    profile.height()       == stream.height) return p;
            }
        }

        return nullptr;
    };

    rs2_intrinsics intrinsics;

private:
    std::string  m_ip_address;
    unsigned int m_ip_port;

    rs2::software_device m_device;

    std::thread m_rtp;
    std::thread m_dev;

    // pthread_t m_th_rtp;
    // pthread_attr_t m_th_rtp_attr;

    // pthread_t m_th_dev;
    // pthread_attr_t m_th_dev_attr;
    
    RSRTSPClient* m_rtspClient;
    char m_eventLoopWatchVariable;

    void* doRTP();

    void doDevice();

    // std::shared_ptr<rs2::software_sensor> rgb;
    // std::shared_ptr<rs2::software_sensor> stereo;
    // std::shared_ptr<rs2::stream_profile>  sp;

    std::mutex m_mutex;
    std::condition_variable m_init_done;

    uint8_t* m_frame_raw;

    std::map<std::string, NetSensor>      sensors;
    // std::map<std::string, StreamProfiles>  sensor_profiles;

    // std::vector<rs_net_sensor> sensors;

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
