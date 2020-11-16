#include <JPEGEncodeFilter.h>

#include <chrono>
#include <iostream>
#include <iomanip>

void JPEGEncodeFilter::afterGettingFrame( unsigned frameSize,
                        unsigned numTruncatedBytes,
                        struct timeval presentationTime,
                        unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = {0};
    FILE* f = 0;
//#define USE_ABBREVIATION
#ifdef USE_ABBREVIATION
    int fSize = compress(m_framebuf_in, 640, 480, m_framebuf_out);

    // sprintf(fname, "/tmp/non%04u", fnum);
    // f = fopen(fname, "w");
    // fwrite(m_framebuf_out, 1, fSize, f);
    // fclose(f);

    // Find the SOS (Start Of Scan) marker
    uint32_t i = 0;
    while (i < fSize) {
        if ((m_framebuf_out[i] == 0xFF) && (m_framebuf_out[i+1] == 0xDA)) break;
        i++;
    }
    // ASSERT: i == fSize
    i++; // 0xDA
    i++; // 0x00 - dirty
    i++; // size - dirty
    i += (m_framebuf_out[i] - 2);
    i++;
    
    fFrameSize = fSize - i;
    memcpy(fTo, &m_framebuf_out[i], fFrameSize);
    
    // sprintf(fname, "/tmp/out%04u", fnum++);
    // f = fopen(fname, "w");
    // fwrite(fTo, 1, fFrameSize, f);
    // fclose(f);
#else
    auto start = std::chrono::system_clock::now();
    fFrameSize = jpeg::compress(m_framebuf_in, 640, 480, fTo);

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;
    std::chrono::duration<double> total_time = end-m_beginning;
    m_frame_count++;
    double fps;
    if (total_time.count() > 0) fps = (double)m_frame_count / (double)total_time.count();
    else fps = 0;
    std::cout << "Frame compression time " << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << " (" << std::fixed << std::setw(5) << std::setprecision(2) << (float)(FRAME_SIZE) / (float)fFrameSize << " ), FPS: " << fps << "\n";

    // std::cout << "Frame compression time " << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << "\n";

    // sprintf(fname, "/tmp/non%04u", fnum);
    // f = fopen(fname, "w");
    // fwrite(m_framebuf_out, 1, fFrameSize, f);
    // fclose(f);
#endif

    afterGetting(this);
}
