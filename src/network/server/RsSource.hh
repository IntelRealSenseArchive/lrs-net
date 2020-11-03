// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include "DeviceSource.hh"
#include "RsSensor.hh"

#include <condition_variable>
#include <mutex>
#include <librealsense2/rs.hpp>

#if 0
class JPEGEncodeFilter : public FramedSource
{
public:
    static JPEGEncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGEncodeFilter(t_env, source); };

protected:
    JPEGEncodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedSource(t_env), m_source(source) {};
    virtual ~JPEGEncodeFilter() { };

protected:
    virtual void doStopGettingFrames() { return FramedSource::doStopGettingFrames(); };

private:
    virtual void doGetNextFrame() { return m_source->doGetNextFrame(); };

private:
    FramedSource* m_source;
};
#else
class JPEGEncodeFilter : public /* FramedSource */ FramedFilter 
{
public:
    static JPEGEncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGEncodeFilter(t_env, source); }

    //????// const FramedSource* inputSource() { return fInputSource; }

protected:
    JPEGEncodeFilter(UsageEnvironment& t_env, FramedSource* source) : /* FramedSource(t_env), fInputSource(source) */ FramedFilter(t_env, source) {} 
    virtual ~JPEGEncodeFilter() {};

private:
    // FramedSource* fInputSource;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(fTo, fMaxSize, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEGEncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds)
    {
        fFrameSize = frameSize;
        afterGetting(this);
    }

protected:
    virtual char const* MIMEtype() const { if (fInputSource) return fInputSource->MIMEtype(); else return ""; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};

/*
class JPEGEncodeFilter : public FramedFilter
{
public:
    static JPEGEncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGEncodeFilter(t_env, source); };

protected:
    JPEGEncodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) {}
    virtual ~JPEGEncodeFilter() {};

private:
    virtual void doGetNextFrame() { if (inputSource()) inputSource()->doGetNextFrame(); };

protected:
    virtual char const* MIMEtype() const { if (inputSource()) inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); };
};
*/
#endif


class RsDeviceSource : public FramedSource
{
public:
    static RsDeviceSource* createNew(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    static void waitForFrame(RsDeviceSource* t_deviceSource);

protected:
    RsDeviceSource(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    virtual ~RsDeviceSource();

protected:
    virtual void doStopGettingFrames();

private:
    virtual void doGetNextFrame();
    rs2::frame_queue& getFramesQueue() { return m_framesQueue; };

private:
    RsSensor m_rsSensor;

    rs2::frame_queue m_framesQueue;
    rs2::video_stream_profile& m_streamProfile;
};
