#include <LZ4DecodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

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

    if (!m_processing) {
        // new frame arrived, let's store it's params
        m_processing = 1;

        start = std::chrono::system_clock::now();

        m_size = 0;
        m_offset = 0;
        m_total_size = 0;
    }

    // we have the reminds of the old frame to receive
    m_total_size += frameSize;
    m_offset = *(uint32_t*)m_framebuf_in;
#define CHUNK_SIZE (2*1024)
    int ret = ZSTD_decompress((void*)(m_framebuf_out + m_offset), CHUNK_SIZE, (void*)(m_framebuf_in + sizeof(uint32_t)), frameSize - sizeof(uint32_t));
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

/*
#if 0
  #if 0
    memset(m_framebuf_out, 0, FRAME_SIZE);
    // got the frame - it might be broken - repare it
    // find the chunk and it's size
    uint8_t* ptr = m_framebuf_in;
    while (ptr - m_framebuf_in < frameSize) {
        // find the beginning of the block in case the first one is broken
        if (*((uint32_t*)ptr) != ZSTD_MAGICNUMBER) {
            while ((*((uint32_t*)ptr) != ZSTD_MAGICNUMBER) && (ptr - m_framebuf_in < frameSize)) ptr++;
            off += CHUNK_SIZE;
        }

        if (ptr - m_framebuf_in != frameSize) {
            // we got the magic
            uint8_t* next = ptr + 4;
            // find the next one
            while ((*((uint32_t*)next) != ZSTD_MAGICNUMBER) && (next - m_framebuf_in < frameSize)) next++;
            size = next - ptr;

            if (ZSTD_isError(ret)) {
                // broken chunk
                std::cout << "Decompression issues" << std::endl;
            }
            off += CHUNK_SIZE;
            ptr = next;
        }

        if (off > FRAME_SIZE) {
            off = FRAME_SIZE;
            break;
        }
    }
    memcpy(fTo, m_framebuf_out, off);
    fFrameSize = off;
  #else
    // sprintf(fname, "/tmp/rs/in%04u.j2k", fnum++);
    // f = fopen(fname, "wb");
    // fwrite(m_framebuf, 1, frameSize, f);
    // fclose(f);
    // sprintf(fname, "/tmp/rs/out.yuv");

    // int ret = ZSTD_decompress((void*)m_framebuf, FRAME_SIZE * 10, (void*)m_framebuf_out, fOffset);
    int ret = ZSTD_decompress((void*)fTo, FRAME_SIZE, (void*)m_framebuf_in, frameSize);
    if (ZSTD_isError(ret)) {
        switch (ZSTD_getErrorCode(ret)) {
        case ZSTD_error_no_error:                           std::cout << "ERROR: No error"                              << std::endl; break;
        case ZSTD_error_GENERIC:                            std::cout << "ERROR: Generic"                               << std::endl; break;
        case ZSTD_error_prefix_unknown:                     std::cout << "ERROR: Prefix unknown"                        << std::endl; break;
        case ZSTD_error_version_unsupported:                std::cout << "ERROR: Version unsupported"                   << std::endl; break;
        case ZSTD_error_frameParameter_unsupported:         std::cout << "ERROR: Frame parameter unsupported"           << std::endl; break;
        case ZSTD_error_frameParameter_windowTooLarge:      std::cout << "ERROR: Frame parameter window too large"      << std::endl; break;
        case ZSTD_error_corruption_detected:                std::cout << "ERROR: Corruption detected"                   << std::endl; break;
        case ZSTD_error_checksum_wrong:                     std::cout << "ERROR: Wrong checksum"                        << std::endl; break;
        case ZSTD_error_dictionary_corrupted:               std::cout << "ERROR: Dictionary corrupted"                  << std::endl; break;
        case ZSTD_error_dictionary_wrong:                   std::cout << "ERROR: Dictionary wrong"                      << std::endl; break;
        case ZSTD_error_dictionaryCreation_failed:          std::cout << "ERROR: Dictionary creation failed"            << std::endl; break;
        case ZSTD_error_parameter_unsupported:              std::cout << "ERROR: Paramether unsupported"    << std::endl; break;
        case ZSTD_error_parameter_outOfBound:               std::cout << "ERROR: Paramether out of bound"    << std::endl; break;
        case ZSTD_error_tableLog_tooLarge:                  std::cout << "ERROR: Table log too large"                   << std::endl; break;
        case ZSTD_error_maxSymbolValue_tooLarge:            std::cout << "ERROR: Max symbol value is too large"         << std::endl; break;
        case ZSTD_error_maxSymbolValue_tooSmall:            std::cout << "ERROR: Max symbol value is too small"         << std::endl; break;
        case ZSTD_error_stage_wrong:                        std::cout << "ERROR: Wrong stage"                           << std::endl; break;
        case ZSTD_error_init_missing:                       std::cout << "ERROR: Init missing"                          << std::endl; break;
        case ZSTD_error_memory_allocation:                  std::cout << "ERROR: Memory allocation"                     << std::endl; break;
        case ZSTD_error_workSpace_tooSmall:                 std::cout << "ERROR: Warkspace is too small"                     << std::endl; break;
        case ZSTD_error_dstSize_tooSmall:                   std::cout << "ERROR: Destination size is too small"         << std::endl; break;
        case ZSTD_error_srcSize_wrong:                      std::cout << "ERROR: Source size is wrong"                  << std::endl; break;
        case ZSTD_error_dstBuffer_null:                     std::cout << "ERROR: Destination buffer is NULL"                     << std::endl; break;
        }
    } else {
        fFrameSize = ret;
    }
  #endif
#endif
*/
}
