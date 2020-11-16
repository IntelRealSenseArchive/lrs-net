#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <zstd.h>
#include <zstd_errors.h>

#include <chrono>

#define FRAME_SIZE (640*480*2)

class LZ4EncodeFilter : public FramedFilter
{
public:
    static LZ4EncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new LZ4EncodeFilter(t_env, source); }

protected:
    LZ4EncodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source), m_frame_count(0) {
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE];

        m_beginning = std::chrono::system_clock::now();
    } 

    virtual ~LZ4EncodeFilter() {
        delete[] m_framebuf_in;
        delete[] m_framebuf_out;
    };

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    uint32_t m_frame_count;
    std::chrono::_V2::system_clock::time_point m_beginning;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, fMaxSize /* FRAME_SIZE */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((LZ4EncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds);

protected:
    virtual char const* MIMEtype() const { return "video/LZ4"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};
