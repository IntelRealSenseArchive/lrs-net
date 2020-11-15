#include <LZ4EncodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

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

    // save the resolution into the frame - this is a hack WIDTH x HEIGHT => [uint16_t] x [uint16_t]
    uint16_t* hdr = (uint16_t*)fTo;
    hdr[0] = 640;
    hdr[1] = 480;
    fFrameSize = sizeof(uint16_t) * 2;

    // start to store compressed lines
    size_t line_size = 0;
    auto start = std::chrono::system_clock::now();
#define LINES_TO_COMPRESS 48
    for (int i = 0; i < 480 / LINES_TO_COMPRESS; i++) {
        line_size = engine_lz4.compress(m_framebuf_in + (640 * 2 * i * LINES_TO_COMPRESS), 640 * 2 * LINES_TO_COMPRESS, fTo + fFrameSize, 640 * 2 * LINES_TO_COMPRESS);

        // sprintf(fname, "/tmp/rs/cmp%04u.j2k", fnum++);
        // f = fopen(fname, "w");
        // fwrite(fTo + fFrameSize, 1, line_size, f);
        // fclose(f);

        // uint32_t* magic = (uint32_t*)(fTo + fFrameSize);
        // std::cout << "Line magic is " << std::hex << *magic << std::dec << " for size " << line_size << std::endl;
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
