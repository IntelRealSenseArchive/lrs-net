#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <chrono>

#define FRAME_SIZE (640*480*2)

class LZ4EncodeFilter : public FramedFilter
{
public:
    static LZ4EncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new LZ4EncodeFilter(t_env, source); }

protected:
    LZ4EncodeFilter(UsageEnvironment& t_env, FramedSource* source);
    virtual ~LZ4EncodeFilter();

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    uint32_t m_processing;
    uint32_t m_size;
    uint32_t m_offset;
    uint32_t m_out_size;
    std::chrono::_V2::system_clock::time_point start;

    uint32_t m_frame_count;
    std::chrono::_V2::system_clock::time_point m_beginning;

    virtual void doGetNextFrame();
    static void afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);
    void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);

protected:
    virtual char const* MIMEtype() const { return "video/LZ4"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};
