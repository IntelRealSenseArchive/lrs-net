#include <LZ4DecodeFilter.h>

#include <chrono>
#include <iostream>

void LZ4DecodeFilter::afterGettingFrame( unsigned frameSize,
                        unsigned numTruncatedBytes,
                        struct timeval presentationTime,
                        unsigned durationInMicroseconds)
{
    static uint32_t fnum = 1;
    char fname[32] = {0};
    FILE* f = 0;

    fFrameSize = 0;
    size_t size = 0;

    // sprintf(fname, "/tmp/rs/in%04u.j2k", fnum++);
    // f = fopen(fname, "wb");
    // fwrite(m_framebuf, 1, frameSize, f);
    // fclose(f);

    sprintf(fname, "/tmp/rs/out.yuv");

    if (fOffset == 0) start = std::chrono::system_clock::now();

    if (frameSize == (sizeof(uint16_t) * 2)) {
        // header received - new frame arrived
        if (fOffset) {
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed = end-start;

            std::cout << "Frame decompression time " << elapsed.count() * 1000 << " ms, size " << fOffset << "\n";

            memcpy(fTo, m_framebuf_out, fOffset);
            fFrameSize = fOffset;

            // std::cout << "Frame received size " << fFrameSize << "\n";

            // if (fFrameSize == FRAME_SIZE) {
            //     f = fopen(fname, "ab");
            //     fwrite(m_framebuf_out, 1, fFrameSize, f);
            //     fclose(f);
            // }

            fOffset = 0; 

            afterGetting(this);
            return;
        }
    } else {
        size = engine_lz4.decompress(m_framebuf_in, frameSize, m_framebuf_out + fOffset, 640 * 2 * 2 * 48);
        // std::cout << "received size " << frameSize << " => " << size << "\n";
        fOffset += size;
    }

    nextTask() = envir().taskScheduler().scheduleDelayedTask(50, (TaskFunc*)getAgain, this);
    return;
}

void LZ4DecodeFilter::getAgain(void* clientData) {
    LZ4DecodeFilter* f = (LZ4DecodeFilter*)clientData;
    f->fInputSource->getNextFrame(f->m_framebuf_in, FRAME_SIZE /* fMaxSize */, afterGettingFrame, f, FramedSource::handleClosure, f);
}
