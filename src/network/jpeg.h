#pragma once

#include <jpeglib.h>

#include <stdint.h>

#if 1

class jpeg {
public:
    jpeg() {};
   ~jpeg() {};

    static int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size);
    static int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size);
};

#else

class compress_method {
public:    
    compress_method() {};
   ~compress_method() {};

    // for image
    virtual int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t out_size) = 0;
    // for block 
    virtual int compress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)           = 0;  
    // universal
    virtual int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)         = 0;  
};

class method_jpeg : public compress_method {
public:
    method_jpeg() {};
   ~method_jpeg() {};

    virtual int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size) { return 0; };
    virtual int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)     { return 0; };
    virtual int compress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)       { return 0; };  
};

class method_zstd : public compress_method {
public:
    method_zstd() {};
   ~method_zstd() {};

    virtual int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size) { return 0; };
    virtual int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)     { return 0; };
    virtual int compress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)       { return 0; };  
};

class method_lz4 : public compress_method {
public:
    method_lz4() {};
   ~method_lz4() {};

    virtual int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size) { return 0; };
    virtual int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)     { return 0; };
    virtual int compress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)       { return 0; };  
};

class method_pass : public compress_method {
public:
    method_pass() {};
   ~method_pass() {};

    virtual int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size) { return 0; };
    virtual int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)     { return 0; };
    virtual int compress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size)       { return 0; };  
};


using JPEG = std::shared_ptr<method_jpeg>;
using ZSTD = std::shared_ptr<method_zstd>;
using LZ4  = std::shared_ptr<method_lz4>;
using PASS = std::shared_ptr<method_pass>;

class compression {
public:
    compression();
   ~compression();

    static JPEG get_method_jpeg() { JPEG m(new method_jpeg()); return m; };
    static ZSTD get_method_zstd() { ZSTD m(new method_zstd()); return m; };
    static LZ4  get_method_lz4 () { LZ4  m(new method_lz4 ()); return m; };
    static PASS get_method_pass() { PASS m(new method_pass()); return m; };
};

#endif