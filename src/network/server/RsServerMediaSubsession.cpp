// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "RsServerMediaSubsession.h"
#include "RsCommon.h"
#include "RsServerMediaSession.h"
#include "RsSimpleRTPSink.h"

#define CAPACITY 100

RsServerMediaSubsession* RsServerMediaSubsession::createNew(UsageEnvironment& t_env, rs2::video_stream_profile& t_videoStreamProfile, std::shared_ptr<RsDevice> rsDevice)
{
    return new RsServerMediaSubsession(t_env, t_videoStreamProfile, rsDevice);
}

RsServerMediaSubsession ::RsServerMediaSubsession(UsageEnvironment& env, rs2::video_stream_profile& t_videoStreamProfile, std::shared_ptr<RsDevice> device)
    : OnDemandServerMediaSubsession(env, false)
    , m_videoStreamProfile(t_videoStreamProfile)
{
    m_frameQueue = rs2::frame_queue(CAPACITY, true);
    m_rsDevice = device;
}

RsServerMediaSubsession::~RsServerMediaSubsession() {}

rs2::frame_queue& RsServerMediaSubsession::getFrameQueue()
{
    return m_frameQueue;
}

rs2::video_stream_profile RsServerMediaSubsession::getStreamProfile()
{
    return m_videoStreamProfile;
}

FramedSource* RsServerMediaSubsession::createNewStreamSource(unsigned /*t_clientSessionId*/, unsigned& t_estBitrate)
{
    t_estBitrate = 20000;
    return RsDeviceSource::createNew(envir(), m_videoStreamProfile, m_frameQueue);
}

#if 0
RTPSink* RsServerMediaSubsession ::createNewRTPSink(Groupsock* t_rtpGroupsock, unsigned char t_rtpPayloadTypeIfDynamic, FramedSource* /*t_inputSource*/)
{
    return RsSimpleRTPSink::createNew(envir(), t_rtpGroupsock, 96 + m_videoStreamProfile.stream_type(), RTP_TIMESTAMP_FREQ, RS_MEDIA_TYPE.c_str(), RS_PAYLOAD_FORMAT.c_str(), m_videoStreamProfile, m_rsDevice);
}
#else

RTPSink* RsServerMediaSubsession::createNewRTPSink(Groupsock* rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic, FramedSource* /*inputSource*/) 
{
    int pixelSize = 0;
    switch (m_videoStreamProfile.format())
    {
        case  RS2_FORMAT_RGB8:
        {
            pixelSize = 3;
            return  RsRawVideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic, 16, m_videoStreamProfile, "RGB", "BT709-2");
        }
        case  RS2_FORMAT_BGR8:
        {
            pixelSize = 3;
            return RsRawVideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic, 16, m_videoStreamProfile, "BGR", "BT709-2");
        }
        case  RS2_FORMAT_RGBA8:
        {
            pixelSize = 4;
            return RsRawVideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic, 16, m_videoStreamProfile, "RGBA", "BT709-2");
        }
        case  RS2_FORMAT_BGRA8:
        {
            pixelSize = 4;
            return RsRawVideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic, 16, m_videoStreamProfile, "BGRA", "BT709-2");
        }
        case  RS2_FORMAT_Z16:
        case  RS2_FORMAT_Y16:
        case  RS2_FORMAT_RAW16:
        case  RS2_FORMAT_YUYV:
        {
            pixelSize = 2;
            return RsRawVideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic, 16, m_videoStreamProfile, "YCbCr-4:2:2", "BT709-2");
        }
        default:
        {
            break;
        }
    }

    return NULL;
}
#endif