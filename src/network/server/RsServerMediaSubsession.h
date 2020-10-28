// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "RsDevice.hh"
#include <liveMedia.hh>

#ifndef _ON_DEMAND_SERVER_MEDIA_SUBSESSION_HH
#include "OnDemandServerMediaSubsession.hh"
#endif

#include "RsSource.hh"

class RsServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
    static RsServerMediaSubsession* createNew(UsageEnvironment& t_env, RsSensor& sensor, rs2::video_stream_profile& t_videoStreamProfile);

protected:
    RsServerMediaSubsession(UsageEnvironment& t_env, RsSensor& sensor, rs2::video_stream_profile& t_video_stream_profile);
    virtual ~RsServerMediaSubsession();

    virtual char const* getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource);

    virtual FramedSource* createNewStreamSource(unsigned t_clientSessionId, unsigned& t_estBitrate);
    virtual RTPSink* createNewRTPSink(Groupsock* t_rtpGroupsock, unsigned char t_rtpPayloadTypeIfDynamic, FramedSource* t_inputSource);

private:
    char* m_pAuxSDPLine;
    rs2::video_stream_profile m_videoStreamProfile;
    RsSensor m_rsSensor;
};
