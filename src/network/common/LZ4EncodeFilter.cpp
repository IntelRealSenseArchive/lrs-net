#include <LZ4EncodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

#define LZ4_COMPRESSION 1

void LZ4EncodeFilter::afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = "/tmp/stream.yuv";
    FILE* f = 0;

    // f = fopen(fname, "ab+");
    // fwrite(fTo, 1, FRAME_SIZE, f);
    // fclose(f);

    fFrameSize = 0;
    int size = 0;
    auto start = std::chrono::system_clock::now();
#define CHUNK_SIZE (16*1024)
    while (size < FRAME_SIZE) {
        fFrameSize += ZSTD_compress((void*)(fTo + fFrameSize), FRAME_SIZE, (void*)(m_framebuf_in + size), FRAME_SIZE - size > CHUNK_SIZE ? CHUNK_SIZE : FRAME_SIZE - size, LZ4_COMPRESSION);
        size += CHUNK_SIZE;
    }

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;
    std::chrono::duration<double> total_time = end-m_beginning;
    m_frame_count++;
    double fps;
    if (total_time.count() > 0) fps = (double)m_frame_count / (double)total_time.count();
    else fps = 0;
    std::cout << "Frame compression time " << std::fixed << std::setw(5) << std::setprecision(2) << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << " (" << (float)(FRAME_SIZE) / (float)fFrameSize << " ), FPS: " << fps << "\n";

    // sprintf(fname, "/tmp/cmp%04u.j2k", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);

    afterGetting(this);
}
