#pragma once 

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <openjpeg.h>

#define FRAME_SIZE 640*480*2

class mem_stream {
public:
    mem_stream(uint8_t* buffer, uint64_t size, bool read_mode) : m_stream(NULL), m_buffer(buffer), m_pos(buffer), m_size_buf(size), m_size_use(0), m_mode(read_mode) { m_stream_buffer() = this; };
   ~mem_stream() {};
    
    static void     close_cb (void* stream)                            { return m_stream_buffer()->close(); };
    static size_t   read_cb  (void* buffer, size_t size, void* stream) { return m_stream_buffer()->read(buffer, size); };
    static size_t   write_cb (void* buffer, size_t size, void* stream) { return m_stream_buffer()->write(buffer, size); };
    static int64_t  skip_cb  (int64_t num, void* stream)               { return m_stream_buffer()->skip(num); };
    static int64_t  seek_cb  (int64_t num, void* stream)               { return m_stream_buffer()->seek(num); };

    void    close() {}; // not used for memory buffer
    size_t  read(void* buffer, size_t size);
    size_t  write(void* buffer, size_t size);
    int64_t skip(int64_t size);
    int64_t seek(int64_t size);

    int64_t size() { if (m_mode) return m_size_buf; else return m_size_use; };

private:
    void* m_stream;

    uint8_t* m_buffer;
    uint8_t* m_pos;

    bool    m_mode;
    int64_t m_size_buf; // size of the buffer suppled
    int64_t m_size_use; // size of the used chunk inside the buffer (for write)

    static mem_stream*& m_stream_buffer() {
        static mem_stream* ms = 0;

        return ms;
    }
};

class jpeg2k {
public:
    jpeg2k() {};
   ~jpeg2k() {};

    static void print(const char *msg, void *priv);

    static int compress(uint8_t * yuyv, uint32_t width, uint32_t height, uint8_t * j2k, uint32_t size);
    static int decompress(uint8_t * j2k, uint32_t size, uint8_t * yuyv);
};
