// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "RsDevice.hh"
#include "liveMedia.hh"
#include <librealsense2/hpp/rs_internal.hpp>

class RsRawVideoRTPSink : public RawVideoRTPSink
{
public:
    static RsRawVideoRTPSink*
        createNew(UsageEnvironment& env, Groupsock* RTPgs, u_int8_t rtpPayloadFormat,
            // The following headers provide the 'configuration' information, for the SDP description:
            unsigned depth,
            rs2::video_stream_profile& video_stream, char const* sampling, char const* colorimetry = "BT709-2");

protected:
    RsRawVideoRTPSink(UsageEnvironment& env, Groupsock* RTPgs,
        u_int8_t rtpPayloadFormat,
        unsigned depth,
        rs2::video_stream_profile& video_stream, char const* sampling, char const* colorimetry = "BT709-2");



protected: // redefined virtual functions:
    virtual void stopPlaying();

private:
    char* fFmtpSDPLine;
    char* fSampling;
    unsigned fWidth;
    unsigned fHeight;
    unsigned fDepth;
    char* fFormat;
    char* fColorimetry;
    virtual char const* auxSDPLine(); // for the "a=fmtp:" SDP line

};

class RsSimpleRTPSink : public SimpleRTPSink
{
public:
    static RsSimpleRTPSink* createNew(UsageEnvironment& env,
                                      Groupsock* RTPgs,
                                      unsigned char rtpPayloadFormat,
                                      unsigned rtpTimestampFrequency,
                                      char const* sdpMediaTypeString,
                                      char const* rtpPayloadFormatName,
                                      rs2::video_stream_profile& video_stream,
                                      std::shared_ptr<RsDevice> device,
                                      unsigned numChannels = 1,
                                      Boolean allowMultipleFramesPerPacket = True,
                                      Boolean doNormalMBitRule = True);

protected:
    RsSimpleRTPSink(UsageEnvironment& env,
                    Groupsock* RTPgs,
                    unsigned char rtpPayloadFormat,
                    unsigned rtpTimestampFrequency,
                    char const* sdpMediaTypeString,
                    char const* rtpPayloadFormatName,
                    rs2::video_stream_profile& video_stream,
                    std::shared_ptr<RsDevice> device,
                    unsigned numChannels = 1,
                    Boolean allowMultipleFramesPerPacket = True,
                    Boolean doNormalMBitRule = True);

private:
    char* m_fFmtpSDPLine;
    virtual char const* auxSDPLine(); // for the "a=fmtp:" SDP line
};
