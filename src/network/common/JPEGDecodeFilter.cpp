#include <JPEGDecodeFilter.h>

#include <chrono>
#include <iostream>

void JPEGDecodeFilter::afterGettingFrame( unsigned frameSize,
                        unsigned numTruncatedBytes,
                        struct timeval presentationTime,
                        unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = {0};
    FILE* f = 0;
//#define USE_ABBREVIATION
#ifdef USE_ABBREVIATION
    decompress(m_framebuf, frameSize, fTo);
    fFrameSize = FRAME_SIZE;
#else

    auto start = std::chrono::system_clock::now();

    // sprintf(fname, "/tmp/in%04u", fnum++);
    // f = fopen(fname, "w");
    // fwrite(m_framebuf, 1, frameSize, f);
    // fclose(f);

    // find the second SOI marker to use the original headers
    uint32_t i = 1;
    while (i < frameSize) {
        if ((m_framebuf[i] == 0xFF) && (m_framebuf[i+1] == 0xD8)) break;
        i++;
    }

    // memcpy(fTo, &m_framebuf[i], frameSize - i + 1);
    // fFrameSize = frameSize - i + 1;

    // sprintf(fname, "/tmp/in%04u", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);

    jpeg::decompress(&m_framebuf[i], frameSize - i + 1, fTo);
    fFrameSize = FRAME_SIZE;

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;

    // std::cout << "Frame decompression time " << elapsed.count() * 1000 << "ms,\tfrom size " << frameSize << "\n";

    // f = fopen("/tmp/mjpeg", "a+");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);
#endif
    afterGetting(this);
}
