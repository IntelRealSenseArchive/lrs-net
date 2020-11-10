#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <jpeg2k.h>

#define FRAME_SIZE (640*480*2)

class JPEG2000DecodeFilter : public FramedFilter
{
public:
    static JPEG2000DecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEG2000DecodeFilter(t_env, source); };

protected:
    JPEG2000DecodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) { m_framebuf = new uint8_t[FRAME_SIZE]; }
    virtual ~JPEG2000DecodeFilter() { delete[] m_framebuf; };

private:

    uint8_t* m_framebuf; // frame buffer for plain YUYV image from the camera

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEG2000DecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds);

protected:
    virtual char const* MIMEtype() const { if (inputSource()) inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); if (inputSource() != NULL) inputSource()->stopGettingFrames(); };
};
