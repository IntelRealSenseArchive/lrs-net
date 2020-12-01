#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include <chrono>

#define FRAME_SIZE (640*480*2)
#define LZ4_PAYLOAD_HEADER_SIZE 4

class LZ4DecodeFilter : public FramedFilter
{
public:
    static LZ4DecodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new LZ4DecodeFilter(t_env, source); };

protected:
    LZ4DecodeFilter(UsageEnvironment& t_env, FramedSource* source);
    virtual ~LZ4DecodeFilter();

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;
    uint8_t* m_framebuf;

    uint32_t m_processing;
    uint32_t m_size;
    uint32_t m_offset;
    uint32_t m_total_size;
    std::chrono::system_clock::time_point start;

    uint32_t m_frame_count;
    std::chrono::system_clock::time_point m_beginning;

    virtual void doGetNextFrame();
    static void afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);
    void afterGettingFrame( unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);

    static void getFrame(void* clientData);

protected:
    virtual char const* MIMEtype() const { if (inputSource()) return inputSource()->MIMEtype(); };
    virtual void getAttributes() const { if (inputSource()) inputSource()->getAttributes(); };
    virtual void doStopGettingFrames() { return FramedFilter::doStopGettingFrames(); if (inputSource() != NULL) inputSource()->stopGettingFrames(); };
};
