#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <jpeg.h>

#include <chrono>

#define FRAME_SIZE (640*480*2)

class JPEGDecodeFilter : public FramedFilter
{
public:
    static JPEGDecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGDecodeFilter(t_env, source); };

protected:
    JPEGDecodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) { 
        m_framebuf = new uint8_t[FRAME_SIZE]; 
        m_beginning = std::chrono::system_clock::now();
    }
    virtual ~JPEGDecodeFilter() { delete[] m_framebuf; };

private:

    uint8_t* m_framebuf; // frame buffer for plain YUYV image from the camera

    uint32_t m_frame_count;
    std::chrono::system_clock::time_point m_beginning;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEGDecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
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
