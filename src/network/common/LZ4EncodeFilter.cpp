#include <LZ4EncodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

#define LZ4_COMPRESSION 3

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

    // start to store compressed lines
    fFrameSize = 0;
    size_t line_size = 0;
    int todo = 0;
    int size = 0;
    auto start = std::chrono::system_clock::now();
#define CHUNK_SIZE (16*1024)
    while (size < FRAME_SIZE) {
#ifdef LZ4_CMP
        todo = FRAME_SIZE - size;
        line_size = LZ4_compress_destSize((char*)(m_framebuf_in + size), (char*)(fTo + fFrameSize), &todo, CHUNK_SIZE);
        size += todo;
#else
        line_size = ZSTD_compress((void*)(fTo + fFrameSize), FRAME_SIZE, (void*)(m_framebuf_in + size), FRAME_SIZE - size > CHUNK_SIZE ? CHUNK_SIZE : FRAME_SIZE - size, LZ4_COMPRESSION);
        size += CHUNK_SIZE;
#endif
        fFrameSize += line_size;
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;

    std::cout << "Frame compression time " << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << " (" << std::fixed << std::setw(5) << std::setprecision(2) << (float)(FRAME_SIZE) / (float)fFrameSize << " )\n";

    // sprintf(fname, "/tmp/cmp%04u.j2k", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);

    afterGetting(this);
}
