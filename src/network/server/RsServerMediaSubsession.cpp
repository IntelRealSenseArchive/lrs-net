// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "RsServerMediaSubsession.h"
#include "RsCommon.h"
#include "RsServerMediaSession.h"
#include "RsSimpleRTPSink.h"

#include "JPEG2000EncodeFilter.h"
#include "JPEGEncodeFilter.h"
#include "LZ4EncodeFilter.h"
#include "LZ4VideoRTPSink.h"

#include <iostream>

#define CAPACITY 100

const char* format_to_string(rs2_format f) {
    switch (f) {
        case RS2_FORMAT_Z16   :  return "Z16";          /**< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. */
        case RS2_FORMAT_YUYV  :  return "YCbCr-4:2:2";  /**< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV */
        case RS2_FORMAT_UYVY  :  return "YCbCr-4:2:2";  /**< Similar to the standard YUYV pixel format, but packed in a different order */
        case RS2_FORMAT_RGB8  :  return "RGB";          /**< 8-bit red, green and blue channels */
        case RS2_FORMAT_BGR8  :  return "BGR";          /**< 8-bit blue, green, and red channels -- suitable for OpenCV */
        case RS2_FORMAT_RGBA8 :  return "RGBA";         /**< 8-bit red, green and blue channels + constant alpha channel equal to FF */
        case RS2_FORMAT_BGRA8 :  return "BGR";          /**< 8-bit blue, green, and red channels + constant alpha channel equal to FF */
        case RS2_FORMAT_Y8    :  return "Y8";           /**< 8-bit per-pixel grayscale image */
        case RS2_FORMAT_Y16   :  return "Y16";          /**< 16-bit per-pixel grayscale image */
    }

    return "N/A";
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

#define ENCODER_LZ4

#if   defined(ENCODER_JPEG200)
    RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
    return JPEG2000EncodeFilter::createNew(envir(), rs_source);
#elif defined(ENCODER_JPEG)
    RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
    return JPEGEncodeFilter::createNew(envir(), rs_source);
#elif defined(ENCODER_LZ4)
    RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
    return LZ4EncodeFilter::createNew(envir(), rs_source);
#else
    return RsDeviceSource::createNew(envir(), m_rsSensor, m_videoStreamProfile);
#endif
}

RTPSink* RsServerMediaSubsession ::createNewRTPSink(Groupsock* t_rtpGroupsock, unsigned char t_rtpPayloadTypeIfDynamic, FramedSource* t_inputSource)
{
    std::string mime(t_inputSource->MIMEtype());
    std::string media = mime.substr(0, mime.find('/'));
    std::string type  = mime.substr(mime.find('/') + 1);
    std::cout << "Source provides " << media << "/" << type << "\n";

    if (type == "JPEG2000") {
        /// JPEG2000 video
        std::cout << "Using JPEG2000VideoRTPSink\n";
        return JPEG2000VideoRTPSink::createNew(envir(), t_rtpGroupsock);
    } else if (type == "JPEG") {
        /// JPEG video
        std::cout << "Using JPEGVideoRTPSink\n";
        return JPEGVideoRTPSink::createNew(envir(), t_rtpGroupsock);
    } else if (type == "LZ4") {
        /// LZ4 compressed video
        // std::cout << "Should use LZ4VideoRTPSink, using RawVideoRTPSink now\n";
        // return RawVideoRTPSink::createNew(envir(), t_rtpGroupsock, t_rtpPayloadTypeIfDynamic, 
        //     m_videoStreamProfile.width(), m_videoStreamProfile.height(), 8 /* check RFC 4175, sec 6.1 */, 
        //     format_to_string(m_videoStreamProfile.format()), "BT709-2");
        std::cout << "Using LZ4VideoRTPSink\n";
        return LZ4VideoRTPSink::createNew(envir(), t_rtpGroupsock);
    } else {
        /// RAW
        std::cout << "Using RawVideoRTPSink\n";
        return RawVideoRTPSink::createNew(envir(), t_rtpGroupsock, t_rtpPayloadTypeIfDynamic, 
            m_videoStreamProfile.width(), m_videoStreamProfile.height(), 8 /* check RFC 4175, sec 6.1 */, 
            format_to_string(m_videoStreamProfile.format()), "BT709-2");
    }
}