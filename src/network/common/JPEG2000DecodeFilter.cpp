#include <JPEG2000DecodeFilter.h>

#include <chrono>
#include <iostream>

void JPEG2000DecodeFilter::afterGettingFrame( unsigned frameSize,
                        unsigned numTruncatedBytes,
                        struct timeval presentationTime,
                        unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = {0};
    FILE* f = 0;
    
    // sprintf(fname, "/tmp/in%04u.j2k", fnum++);
    // f = fopen(fname, "wb");
    // fwrite(m_framebuf, 1, frameSize, f);
    // fclose(f);

    auto start = std::chrono::system_clock::now();
    // memcpy(fTo, m_framebuf, frameSize);
    jpeg2k::decompress(m_framebuf, frameSize, fTo);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;

    std::cout << "Frame decompression time " << elapsed.count() * 1000 << "ms,\treceived size " << frameSize << "\n";
    fFrameSize = FRAME_SIZE;

    afterGetting(this);
}
