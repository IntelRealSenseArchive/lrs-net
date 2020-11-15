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
    int size = 0;
    int s = 0;

    // sprintf(fname, "/tmp/rs/in%04u.j2k", fnum++);
    // f = fopen(fname, "wb");
    // fwrite(m_framebuf, 1, frameSize, f);
    // fclose(f);

    sprintf(fname, "/tmp/rs/out.yuv");

    auto start = std::chrono::system_clock::now();

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

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end-start;
    std::cout << "Frame decompression time " << elapsed.count() * 1000 << " ms, size " << frameSize << " => " << fFrameSize << "\n";
    
    afterGetting(this);
    return;
}

void LZ4DecodeFilter::getAgain(void* clientData) {
    LZ4DecodeFilter* f = (LZ4DecodeFilter*)clientData;
    f->fInputSource->getNextFrame(f->m_framebuf_in, FRAME_SIZE /* fMaxSize */, afterGettingFrame, f, FramedSource::handleClosure, f);
}
