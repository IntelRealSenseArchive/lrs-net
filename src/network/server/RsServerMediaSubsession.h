// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "RsDevice.hh"
#include <liveMedia.hh>

#ifndef _ON_DEMAND_SERVER_MEDIA_SUBSESSION_HH
#include "OnDemandServerMediaSubsession.hh"
#endif

#include <librealsense2/rs.hpp>

#include "RsSensor.hh"
#include "RsSource.hh"

#include "JPEG2000EncodeFilter.h"
#include "JPEGEncodeFilter.h"
#include "LZ4EncodeFilter.h"
#include "LZ4VideoRTPSink.h"

#include <iostream>

class RsServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
    static RsServerMediaSubsession* createNew(UsageEnvironment& t_env, frames_queue* psq) { return new RsServerMediaSubsession(t_env, psq); };

protected:
    RsServerMediaSubsession(UsageEnvironment& t_env, frames_queue* psq) : OnDemandServerMediaSubsession(t_env, false), m_queue(psq) {};
    virtual ~RsServerMediaSubsession() {};

    virtual char const* getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource) {
        static char* privateAuxSDPLine = {0};
        if (privateAuxSDPLine == NULL) privateAuxSDPLine = new char[512]; // more than enough? :)

        const char* auxSDPLine = OnDemandServerMediaSubsession::getAuxSDPLine(rtpSink, inputSource);
        if (auxSDPLine == NULL) auxSDPLine = "";

        sprintf(privateAuxSDPLine, "%sa=x-dimensions:%d,%d\r\na=x-framerate: %d\r\n", auxSDPLine, m_queue->get_width(), m_queue->get_height(), m_queue->get_fps());
        return privateAuxSDPLine;
    };

    virtual FramedSource* createNewStreamSource(unsigned t_clientSessionId, unsigned& t_estBitrate) {
        t_estBitrate = 40000; // "estBitrate" is the stream's estimated bitrate, in kbps
        std::cout << std::endl << "Creating device source" << std::endl;

#define ENCODER_LZ4

#if   defined(ENCODER_JPEG200)
        RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_queue);
        return JPEG2000EncodeFilter::createNew(envir(), rs_source);
#elif defined(ENCODER_JPEG)
        RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_queue);
        return JPEGEncodeFilter::createNew(envir(), rs_source);
#elif defined(ENCODER_LZ4)
  #if 0
        RsDeviceSource* rs_source = RsDeviceSource::createNew(envir(), m_queue);
        return LZ4EncodeFilter::createNew(envir(), rs_source);
  #else        
        return RsDeviceSource::createNew(envir(), m_queue); 
  #endif        
#else
        return RsDeviceSource::createNew(envir(), m_queue);
#endif

    };

    virtual RTPSink* createNewRTPSink(Groupsock* t_rtpGroupsock, unsigned char t_rtpPayloadTypeIfDynamic, FramedSource* t_inputSource) {
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
            std::cout << "Using LZ4VideoRTPSink\n";
            return LZ4VideoRTPSink::createNew(envir(), t_rtpGroupsock);
        } else {
            /// RAW
            std::cout << "Using RawVideoRTPSink\n";
            return RawVideoRTPSink::createNew(envir(), t_rtpGroupsock, t_rtpPayloadTypeIfDynamic, 
                m_queue->get_width(), m_queue->get_height(), 8, "YCbCr-4:2:2", "BT709-2");
        }
    };

private:
    char* m_pAuxSDPLine;

    frames_queue* m_queue;
};
