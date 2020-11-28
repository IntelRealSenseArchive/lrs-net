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

#define MAX_ACTIVE_STREAMS 4

#define NUM_OF_SENSORS 2

#define POLLING_SW_DEVICE_STATE_INTERVAL 100

#define DEFAULT_PROFILE_FPS 15

#define DEFAULT_PROFILE_WIDTH 424

#define DEFAULT_PROFILE_HIGHT 240

#define DEFAULT_PROFILE_COLOR_FORMAT RS2_FORMAT_RGB8 

#define FRAME_SIZE (640*480*2)

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

// #define COMPRESSION_ENABLED
// #define COMPRESSION_ZSTD

typedef struct chunk_header{
    uint32_t size;
    uint32_t offset;
    uint32_t count;
} chunk_header_t;
#define CHUNK_HLEN (sizeof(chunk_header_t))
#define CHUNK_SIZE (1450 - CHUNK_HLEN)
#define CHUNK_FULL (CHUNK_SIZE + CHUNK_HLEN)

class RSSink : public MediaSink
{
public:
    static RSSink* createNew(UsageEnvironment& env,
                                MediaSubsession& subsession, // identifies the kind of data that's being received
                                char const* streamId,        // identifies the stream itself
                                uint32_t threshold);         // number of stored bytes for the drop

    uint8_t* getFrame();
    void     popFrame();

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
    static RSRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL);

protected:
    RSRTSPClient(UsageEnvironment& env, char const* rtspURL); // called only by createNew();
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
        else if(strcmp(request->commandName(), "SETUP") == 0)
        {
            MediaSubsession& subsession = *request->subsession();
            Boolean streamUsingTCP = (request->booleanFlags() & 0x1) != 0;
            Boolean streamOutgoing = (request->booleanFlags() & 0x2) != 0;
            Boolean forceMulticastOnUnspecified = (request->booleanFlags() & 0x4) != 0;

            char const *prefix, *separator, *suffix;
            // constructSubsessionURL(subsession, prefix, separator, suffix);
            prefix = subsession.parentSession().controlPath();
            if(prefix == NULL)
                prefix = "";

            suffix = subsession.controlPath();
            if(suffix == NULL)
                suffix = "";

            unsigned prefixLen = strlen(prefix);
            separator = (prefixLen == 0 || prefix[prefixLen - 1] == '/' || suffix[0] == '/') ? "" : "/";

            char const* transportFmt;
            if(strcmp(subsession.protocolName(), "RTP") == 0)
            {
                transportFmt = "Transport: RTP/AVP%s%s%s=%d-%d\r\n";
            }
            else if(strcmp(subsession.protocolName(), "SRTP") == 0)
            {
                transportFmt = "Transport: RTP/SAVP%s%s%s=%d-%d\r\n";
            }
            else
            { // "UDP"
                transportFmt = "Transport: RAW/RAW/UDP%s%s%s=%d-%d\r\n";
            }

            cmdURL = new char[strlen(prefix) + strlen(separator) + strlen(suffix) + 1];
            cmdURLWasAllocated = True;
            sprintf(cmdURL, "%s%s%s", prefix, separator, suffix);

            // Construct a "Transport:" header.
            char const* transportTypeStr;
            char const* modeStr = streamOutgoing ? ";mode=receive" : "";
            // Note: I think the above is nonstandard, but DSS wants it this way
            char const* portTypeStr;
            portNumBits rtpNumber, rtcpNumber;
            { // normal RTP streaming
                unsigned connectionAddress = subsession.connectionEndpointAddress();
                Boolean requestMulticastStreaming = IsMulticastAddress(connectionAddress) || (connectionAddress == 0 && forceMulticastOnUnspecified);
                transportTypeStr = requestMulticastStreaming ? ";multicast" : ";unicast";
                portTypeStr = requestMulticastStreaming ? ";port" : ";client_port";
                rtpNumber = subsession.clientPortNum();
                if(rtpNumber == 0)
                {
                    envir().setResultMsg("Client port number unknown\n");
                    delete[] cmdURL;
                    return False;
                }
                rtcpNumber = subsession.rtcpIsMuxed() ? rtpNumber : rtpNumber + 1;
            }
            unsigned transportSize = strlen(transportFmt) + strlen(transportTypeStr) + strlen(modeStr) + strlen(portTypeStr) + 2 * 5 /* max port len */;
            char* transportStr = new char[transportSize];
            sprintf(transportStr, transportFmt, transportTypeStr, modeStr, portTypeStr, rtpNumber, rtcpNumber);

            // When sending more than one "SETUP" request, include a "Session:" header in the 2nd and later commands:
            char* sessionStr = strDup("");

            // The "Transport:", "Session:" (if present), "Blocksize:" (if present), and "KeyMgmt:" (if present)
            // headers make up the 'extra headers':
            extraHeaders = new char[transportSize + strlen(sessionStr) + 1];
            extraHeadersWereAllocated = True;
            sprintf(extraHeaders, "%s%s", transportStr, sessionStr);
            delete[] transportStr;
            delete[] sessionStr;
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

    pthread_t m_th_rtp;
    pthread_attr_t m_th_rtp_attr;

    pthread_t m_th_dev;
    pthread_attr_t m_th_dev_attr;
    
    RSRTSPClient* m_rtspClient;
    char m_eventLoopWatchVariable;

    void* doRTP();

    void doDevice();
    std::shared_ptr<rs2::software_sensor> rgb;
    std::shared_ptr<rs2::stream_profile>  sp;

    std::mutex m_mutex;
    std::condition_variable m_init_done;

    uint8_t* m_frame_raw;

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
