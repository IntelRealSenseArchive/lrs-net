#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <zstd.h>
#include <zstd_errors.h>

#include <chrono>

#define FRAME_SIZE (640*480*2)
#define LZ4_PAYLOAD_HEADER_SIZE 4

class LZ4DecodeFilter : public FramedFilter
{
public:
    static LZ4DecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new LZ4DecodeFilter(t_env, source); };

protected:
    LZ4DecodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source), m_frame_count(0) { 
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE * 2];
        m_framebuf     = new uint8_t[FRAME_SIZE * 10];

        m_beginning = std::chrono::system_clock::now();

    }
    virtual ~LZ4DecodeFilter() { 
        delete[] m_framebuf;
        delete[] m_framebuf_out;
        delete[] m_framebuf_in;
    };

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;
    uint8_t* m_framebuf;

    uint32_t m_frame_count;
    std::chrono::_V2::system_clock::time_point m_beginning;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((LZ4DecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds);

    static void getAgain(void* clientData);

protected:
    virtual char const* MIMEtype() const { if (inputSource()) inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); if (inputSource() != NULL) inputSource()->stopGettingFrames(); };
};
