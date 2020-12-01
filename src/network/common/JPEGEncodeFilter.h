#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <jpeg.h>

#include <chrono>

#define FRAME_SIZE (640*480*2)

class JPEGEncodeFilter : public JPEGVideoSource 
{
public:
    static JPEGEncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGEncodeFilter(t_env, source); }

    virtual Boolean isJPEGVideoSource() const { return True; };


protected:
    JPEGEncodeFilter(UsageEnvironment& t_env, FramedSource* source) : JPEGVideoSource(t_env), fInputSource(source) {
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE];

        m_beginning = std::chrono::system_clock::now();
    } 

    virtual ~JPEGEncodeFilter() {
        delete[] m_framebuf_in;
        delete[] m_framebuf_out;
    };

private:
    FramedSource* fInputSource;

    uint32_t m_frame_count;
    std::chrono::system_clock::time_point m_beginning;

    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    virtual u_int8_t type()    { return 0; };
    virtual u_int8_t qFactor() { return 50; };
    virtual u_int8_t width()   { return 640 / 8; } // # pixels/8 (or 0 for 2048 pixels)
    virtual u_int8_t height()  { return 480 / 8; } // # pixels/8 (or 0 for 2048 pixels)

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, /* fMaxSize */ FRAME_SIZE, afterGettingFrame, this, FramedSource::handleClosure, this);
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
                            unsigned durationInMicroseconds);
 
protected:
    virtual char const* MIMEtype() const { return "video/JPEG"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};
