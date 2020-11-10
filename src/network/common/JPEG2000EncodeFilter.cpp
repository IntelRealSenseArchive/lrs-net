#include <JPEG2000EncodeFilter.h>

#include <chrono>
#include <iostream>

void JPEG2000EncodeFilter::afterGettingFrame( unsigned frameSize,
                            unsigned numTruncatedBytes,
                            struct timeval presentationTime,
                            unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = {0};
    FILE* f = 0;

    auto start = std::chrono::system_clock::now();
    fFrameSize = jpeg2k::compress(m_framebuf_in, 640, 480, fTo, fMaxSize);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;

    std::cout << "Frame compression time " << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << "\n";

    // sprintf(fname, "/tmp/cmp%04u.j2k", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);

    afterGetting(this);
}
