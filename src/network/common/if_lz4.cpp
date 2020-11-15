#include <stdlib.h>

#include <map>
#include <vector>

#include <if_lz4.h>

#include <iostream>

// 1 - fast/not good -> 10 slow/best
#define LZ4_COMPRESSION 2

#define ZSTD_CTX
// #define ZSTD_EXT_DICT

lz4::lz4() {
#ifdef ZSTD_CTX
    cctx = ZSTD_createCCtx();
    dctx = ZSTD_createDCtx();

  #ifdef ZSTD_EXT_DICT    
    char dict_fname[] = "/tmp/dict.zstd";
    int  dict_size = 102400;

    FILE* dict_file = fopen(dict_fname, "rb");
    void* dict = malloc(dict_size);
    fread(dict, 1, dict_size, dict_file);
    fclose(dict_file);

    cdict = ZSTD_createCDict(dict, dict_size, LZ4_COMPRESSION);
    ddict = ZSTD_createDDict(dict, dict_size);
  #endif    
#endif    
}

lz4::~lz4() {
#ifdef ZSTD_CTX
  #ifdef ZSTD_EXT_DICT    
    ZSTD_freeDDict(ddict);
    ZSTD_freeCDict(cdict);
  #endif    
    ZSTD_freeCCtx(cctx);
    ZSTD_freeDCtx(dctx);    
#endif    
}

size_t lz4::compress(uint8_t* in, size_t size_in, uint8_t* out, size_t size_out)
{
    int out_size = 0;
    // int out_size = LZ4_compress_fast((char*)in, (char*)out, FRAME_SIZE, FRAME_SIZE, LZ4_COMPRESSION);
#ifdef ZSTD_CTX
  #ifdef ZSTD_EXT_DICT    
    out_size = ZSTD_compress_usingCDict(cctx, (void*)out, size_out, (void*)in, size_in, cdict);
  #else
    out_size = ZSTD_compressCCtx(cctx, (void*)out, size_out, (void*)in, size_in, LZ4_COMPRESSION);
  #endif
#else
    out_size = ZSTD_compress((void*)out, size_out, (void*)in, size_in, LZ4_COMPRESSION);
#endif
    if (ZSTD_isError(out_size)) {
        print_error(out_size);
    }

    // uint8_t* test = (uint8_t*)malloc(size_in);
    // if (test) {
    //     decompress(out, out_size, test, size_in);
    //     free(test);
    // }

    return out_size;
}

size_t lz4::decompress(uint8_t* in, size_t in_size, uint8_t* out, size_t out_size)
{
    size_t ret = 0;

    if ((*(uint32_t*)in) == ZSTD_MAGIC) {
        // LZ4_decompress_fast((const char*)in, (char*)out, FRAME_SIZE);
#ifdef ZSTD_CTX
  #ifdef ZSTD_EXT_DICT    
        ret = ZSTD_decompress_usingDDict(dctx, (void*)out, out_size, (void*)in, in_size, ddict);
  #else
        ret = ZSTD_decompressDCtx(dctx, (void*)out, out_size, (void*)in, in_size);
  #endif
#else
        ret = ZSTD_decompress((void*)out, out_size, (void*)in, in_size);
#endif  

        if (ZSTD_isError(ret)) {
            print_error(ret);
        }
    } else {
        std::cout << "ERROR: No magic: " << std::hex << (*(uint32_t*)in) << " @ " << (void*)in << " => " << (void*)out << std::dec << std::endl;
    }

    return ret;
}

void lz4::print_error(size_t err)
{
    switch (ZSTD_getErrorCode(err)) {
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

}


















