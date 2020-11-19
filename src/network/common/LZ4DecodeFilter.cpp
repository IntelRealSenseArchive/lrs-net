#include <LZ4DecodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

LZ4DecodeFilter::LZ4DecodeFilter(UsageEnvironment& t_env, FramedSource* source) 
    : FramedFilter(t_env, source), m_frame_count(0), m_processing(0), m_size(0), m_offset(0), m_total_size(0)  { 
    m_framebuf_in  = new uint8_t[FRAME_SIZE];
    m_framebuf_out = new uint8_t[FRAME_SIZE * 2];
    m_framebuf     = new uint8_t[FRAME_SIZE * 10];

    m_beginning = std::chrono::system_clock::now();
}

LZ4DecodeFilter::~LZ4DecodeFilter() { 
    delete[] m_framebuf;
    delete[] m_framebuf_out;
    delete[] m_framebuf_in;
}

void LZ4DecodeFilter::doGetNextFrame() 
{
    // fInputSource->getNextFrame(m_framebuf_in, FRAME_SIZE /* fMaxSize */, afterGettingFrame, this, FramedSource::handleClosure, this);
    getFrame(this);
}

void LZ4DecodeFilter::getFrame(void* clientData) {
    LZ4DecodeFilter* f = (LZ4DecodeFilter*)clientData;
    f->fInputSource->getNextFrame(f->m_framebuf_in, FRAME_SIZE /* fMaxSize */, afterGettingFrame, clientData, FramedSource::handleClosure, clientData);
}

void LZ4DecodeFilter::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds) { 
    ((LZ4DecodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void LZ4DecodeFilter::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds) {
    // static uint32_t fnum = 1;
    // char fname[32] = {0};
    // FILE* f = 0;
#define CHUNK_SIZE (2*1024)
    typedef struct chunk_header{
        uint32_t size;
        uint32_t offset;
    } chunk_header_t;
#define CHUNK_HLEN (sizeof(chunk_header_t))

    if (!m_processing) {
        // new frame arrived, let's store it's params
        m_processing = 1;

        start = std::chrono::system_clock::now();

        m_size = 0;
        m_offset = 0;
        m_total_size = 0;
    }

    // we have the reminds of the old frame to receive
    chunk_header_t* ch = (chunk_header_t*)m_framebuf_in;

    m_total_size += frameSize;
    m_offset = ch->offset;
#if 0    
  #if 0    
    int ret = ZSTD_decompress((void*)(m_framebuf_out + m_offset), CHUNK_SIZE, (void*)(m_framebuf_in + CHUNK_HLEN), frameSize - CHUNK_HLEN);
  #else
    int ret = LZ4_decompress_fast((const char*)(m_framebuf_in + CHUNK_HLEN), (char*)(m_framebuf_out + m_offset), CHUNK_SIZE);
  #endif    
#else
    int ret = frameSize - CHUNK_HLEN;
    memcpy((void*)(m_framebuf_out + m_offset), (void*)(m_framebuf_in + CHUNK_HLEN), ret);
#endif
    m_offset += CHUNK_SIZE;

    if (m_offset < FRAME_SIZE) {
        // get another chunk
        nextTask() = envir().taskScheduler().scheduleDelayedTask(10, getFrame, this);
    } else {
        // deliver the frame
        m_processing = 0;
        fFrameSize = FRAME_SIZE;
        memcpy(fTo, m_framebuf_out, fFrameSize);

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;
        std::chrono::duration<double> total_time = end-m_beginning;
        m_frame_count++;
        double fps;
        if (total_time.count() > 0) fps = (double)m_frame_count / (double)total_time.count();
        else fps = 0;
        std::cout << "Frame decompression time " << std::fixed << std::setw(5) << std::setprecision(2) << elapsed.count() * 1000 << " ms, size " << m_total_size << " => " << fFrameSize << ", FPS: " << fps << "\n";
        
        afterGetting(this);
    }
}
