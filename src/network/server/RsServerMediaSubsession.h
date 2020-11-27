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
    RsServerMediaSubsession(UsageEnvironment& t_env, frames_queue* psq) : OnDemandServerMediaSubsession(t_env, false), m_queue(psq), m_pSDPLines(NULL) {};
    virtual ~RsServerMediaSubsession() {
        if (m_pSDPLines) delete[] m_pSDPLines;
    };

    virtual char const* sdpLines() {
        int estBitrate = 40000; // kilobits per second
        char const* mediaType = "video";
        unsigned char rtpPayloadType = 96;
        AddressString ipAddressStr(fServerAddressForSDP);
        char const* rtpmapLine = "a=rtpmap:96 LZ4/90000\r\n";
        char const* rtcpmuxLine = ""; // fMultiplexRTCPWithRTP ? "a=rtcp-mux\r\n" : "";
        char const* rangeLine = "";   // rangeSDPLine();
        char const* auxSDPLine = "";  // getAuxSDPLine(rtpSink, inputSource);

        char const* const sdpFmt = "m=%s %u UDP %d\r\n"
                                "c=IN IP4 %s\r\n"
                                "b=AS:%u\r\n"
                                "%s"
                                "%s"
                                "%s"
                                "a=x-dimensions:%d,%d\r\n"
                                "a=x-framerate: %d\r\n"
                                "a=control:%s\r\n";

        unsigned sdpFmtSize = 1024;
        char* sdpLines = new char[sdpFmtSize];
        sprintf(sdpLines,
                sdpFmt,
                mediaType, // m= <media>
                fPortNumForSDP, // m= <port>
                rtpPayloadType, // m= <fmt list>
                ipAddressStr.val(), // c= address
                estBitrate, // b=AS:<bandwidth>
                rtpmapLine, // a=rtpmap:... (if present)
                rtcpmuxLine, // a=rtcp-mux:... (if present)
                rangeLine, // a=range:... (if present)
                m_queue->get_width(), m_queue->get_height(), 
                m_queue->get_fps(),
                // auxSDPLine, // optional extra SDP line
                trackId()); // a=control:<track-id>

        fSDPLines = strDup(sdpLines);
        delete[] sdpLines;

        return fSDPLines;
    };

    // virtual char const* getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource) {
    //     static char* privateAuxSDPLine = {0};
    //     if (privateAuxSDPLine == NULL) privateAuxSDPLine = new char[512]; // more than enough? :)

    //     const char* auxSDPLine = OnDemandServerMediaSubsession::getAuxSDPLine(rtpSink, inputSource);
    //     if (auxSDPLine == NULL) auxSDPLine = "";

    //     sprintf(privateAuxSDPLine, "%sa=x-dimensions:%d,%d\r\na=x-framerate: %d\r\n", auxSDPLine, m_queue->get_width(), m_queue->get_height(), m_queue->get_fps());
    //     return privateAuxSDPLine;
    // };

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
        } else if (type == "UDP") {
            /// LZ4 compressed video
            std::cout << "Using BasicUDPSink\n";
            return NULL; // Should BasicUDPSink::createNew(envir(), t_rtpGroupsock);
        } else {
            /// RAW
            std::cout << "Using RawVideoRTPSink\n";
            return RawVideoRTPSink::createNew(envir(), t_rtpGroupsock, t_rtpPayloadTypeIfDynamic, 
                m_queue->get_width(), m_queue->get_height(), 8, "YCbCr-4:2:2", "BT709-2");
        }
    };

private:
    char* m_pSDPLines;
    char* m_pAuxSDPLine;

    frames_queue* m_queue;
};
