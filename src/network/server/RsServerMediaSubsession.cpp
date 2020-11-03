// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "RsServerMediaSubsession.h"
#include "RsCommon.h"
#include "RsServerMediaSession.h"
#include "RsSimpleRTPSink.h"

#include <iostream>

#define CAPACITY 100

const char* format_to_string(rs2_format f) {
    char * fmt = "N/A";

    switch (f) {
        case RS2_FORMAT_Z16   :  fmt = "Z16";         break; /**< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. */
        case RS2_FORMAT_YUYV  :  fmt = "YCbCr-4:2:2"; break; /**< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV */
        case RS2_FORMAT_UYVY  :  fmt = "YCbCr-4:2:2"; break; /**< Similar to the standard YUYV pixel format, but packed in a different order */
        case RS2_FORMAT_RGB8  :  fmt = "RGB";         break; /**< 8-bit red, green and blue channels */
        case RS2_FORMAT_BGR8  :  fmt = "BGR";         break; /**< 8-bit blue, green, and red channels -- suitable for OpenCV */
        case RS2_FORMAT_RGBA8 :  fmt = "RGBA";        break; /**< 8-bit red, green and blue channels + constant alpha channel equal to FF */
        case RS2_FORMAT_BGRA8 :  fmt = "BGR";         break; /**< 8-bit blue, green, and red channels + constant alpha channel equal to FF */
        case RS2_FORMAT_Y8    :  fmt = "Y8";          break; /**< 8-bit per-pixel grayscale image */
        case RS2_FORMAT_Y16   :  fmt = "Y16";         break; /**< 16-bit per-pixel grayscale image */
    }

    std::cout << "Format conversion: " << f << " => " << std::string(fmt) << std::endl;

    return fmt;
}

RsServerMediaSubsession* RsServerMediaSubsession::createNew(UsageEnvironment& t_env, RsSensor& sensor, rs2::video_stream_profile& t_videoStreamProfile)
{
     return new RsServerMediaSubsession(t_env, sensor, t_videoStreamProfile);
}

RsServerMediaSubsession::RsServerMediaSubsession(UsageEnvironment& env, RsSensor& sensor, rs2::video_stream_profile& t_videoStreamProfile)
    : OnDemandServerMediaSubsession(env, false)
    , m_videoStreamProfile(t_videoStreamProfile), m_rsSensor(sensor)
{}

RsServerMediaSubsession::~RsServerMediaSubsession() {}

char const* RsServerMediaSubsession::getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource)
{
    static char* privateAuxSDPLine = {0};
    if (privateAuxSDPLine == NULL) privateAuxSDPLine = new char[512]; // more than enough? :)

    const char* auxSDPLine = OnDemandServerMediaSubsession::getAuxSDPLine(rtpSink, inputSource);
    if (auxSDPLine == NULL) auxSDPLine = "";

    sprintf(privateAuxSDPLine, "%sa=x-dimensions:%d,%d\r\na=x-framerate: %d\r\n", auxSDPLine, m_videoStreamProfile.width(), m_videoStreamProfile.height(), m_videoStreamProfile.fps());
    return privateAuxSDPLine;
}

FramedSource* RsServerMediaSubsession::createNewStreamSource(unsigned /*t_clientSessionId*/, unsigned& t_estBitrate)
{
    t_estBitrate = 20000; // "estBitrate" is the stream's estimated bitrate, in kbps
    std::cout << std::endl << "Creating device source" << std::endl;
#if 1
    RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
    return JPEGEncodeFilter::createNew(envir(), rs_source);
#else
    return RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
#endif
}

RTPSink* RsServerMediaSubsession ::createNewRTPSink(Groupsock* t_rtpGroupsock, unsigned char t_rtpPayloadTypeIfDynamic, FramedSource* /*t_inputSource*/)
{
    /// RAW
    return RawVideoRTPSink::createNew(envir(), t_rtpGroupsock, t_rtpPayloadTypeIfDynamic, 
                                        m_videoStreamProfile.width(), m_videoStreamProfile.height(), 8 /* check RFC 4175, sec 6.1 */, format_to_string(m_videoStreamProfile.format()), "BT709-2");
}