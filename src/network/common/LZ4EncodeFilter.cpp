#include <LZ4EncodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

#define LZ4_COMPRESSION 1

LZ4EncodeFilter::LZ4EncodeFilter(UsageEnvironment& t_env, FramedSource* source) 
    : FramedFilter(t_env, source), m_frame_count(0), m_processing(0), m_size(0), m_offset(0), m_out_size(0) 
{
    m_framebuf_in  = new uint8_t[FRAME_SIZE];
    m_framebuf_out = new uint8_t[FRAME_SIZE];

    m_beginning = std::chrono::system_clock::now();
} 

LZ4EncodeFilter:: ~LZ4EncodeFilter() 
{
    delete[] m_framebuf_in;
    delete[] m_framebuf_out;
}

void LZ4EncodeFilter::doGetNextFrame() { 
    struct timeval presentationTime = {0};
    if (m_processing) {
        afterGettingFrame(this, 0, 0, presentationTime, 0);
    } else {
        fInputSource->getNextFrame(m_framebuf_in, fMaxSize /* FRAME_SIZE */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }
}

void LZ4EncodeFilter::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds) { 
    ((LZ4EncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

void LZ4EncodeFilter::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds) {
    // static uint32_t fnum = 1;
    // char fname[32] = "/tmp/stream.yuv";
    // FILE* f = 0;
    // f = fopen(fname, "ab+");
    // fwrite(fTo, 1, FRAME_SIZE, f);
    // fclose(f);

    if (!m_processing) {
        // new frame arrived, let's store it's params
        m_processing = 1;

        start = std::chrono::system_clock::now();

        m_size = frameSize;
        m_offset = 0;
        m_out_size = 0;
    }

    // we have the reminds of the old frame to send 
#define CHUNK_SIZE (2*1024)
    typedef struct chunk_header{
        uint32_t size;
        uint32_t offset;
    } chunk_header_t;
#define CHUNK_HLEN (sizeof(chunk_header_t))

    chunk_header_t* ch = (chunk_header_t*)fTo;

    ch->offset = m_offset;
    fFrameSize = sizeof(chunk_header_t);
#if 0    
  #if 0    
    int ret = ZSTD_compress((void*)(fTo + CHUNK_HLEN), FRAME_SIZE, (void*)(m_framebuf_in + m_offset), m_size - m_offset > CHUNK_SIZE ? CHUNK_SIZE : m_size - m_offset, LZ4_COMPRESSION);
  #else
    int ret = LZ4_compress_fast((const char*)(m_framebuf_in + m_offset), (char*)(fTo + CHUNK_HLEN), m_size - m_offset > CHUNK_SIZE ? CHUNK_SIZE : m_size - m_offset, FRAME_SIZE, 10);
  #endif    
#else   
    int ret = m_size - m_offset > CHUNK_SIZE ? CHUNK_SIZE : m_size - m_offset;
    memcpy((void*)(fTo + CHUNK_HLEN), (void*)(m_framebuf_in + m_offset), ret);
#endif
    fFrameSize += ret;
    ch->size = fFrameSize;
    m_offset += CHUNK_SIZE;
    m_out_size += fFrameSize;

    if (m_offset >= m_size) {
        // we have dome with this frame get another one
        m_processing = 0;

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;
        std::chrono::duration<double> total_time = end-m_beginning;
        m_frame_count++;
        double fps;
        if (total_time.count() > 0) fps = (double)m_frame_count / (double)total_time.count();
        else fps = 0;
        std::cout << "Frame compression time " << std::fixed << std::setw(5) << std::setprecision(2) << elapsed.count() * 1000 << "ms,\tsize " << m_size << " => " << m_out_size << " (" << (float)(FRAME_SIZE) / (float)m_out_size << " ), FPS: " << fps << "\n";
    }

    // sprintf(fname, "/tmp/cmp%04u.j2k", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);

    afterGetting(this);
}