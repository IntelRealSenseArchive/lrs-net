#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <jpeg2k.h>

#define FRAME_SIZE (640*480*2)

class JPEG2000EncodeFilter : public FramedFilter
{
public:
    static JPEG2000EncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEG2000EncodeFilter(t_env, source); }

    virtual Boolean isJPEG2000VideoSource() const { return True; };


protected:
    JPEG2000EncodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) {
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE];
    } 

    virtual ~JPEG2000EncodeFilter() {
        delete[] m_framebuf_in;
        delete[] m_framebuf_out;
    };

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, fMaxSize /* FRAME_SIZE */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEG2000EncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds);

protected:
    virtual char const* MIMEtype() const { return "video/JPEG2000"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};
