// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include "DeviceSource.hh"
#include "RsSensor.hh"

#include <condition_variable>
#include <mutex>
#include <librealsense2/rs.hpp>

#include <jpeglib.h>
#include <openjpeg.h>

#define FRAME_SIZE 640*480*2

class mem_stream {
public:
    mem_stream(uint8_t* buffer) : m_stream(NULL), m_buffer(buffer), m_pos(buffer), m_size(0) { m_stream_buffer() = this; };
   ~mem_stream() {};
    
    static void     close_cb (void* stream)                            { return m_stream_buffer()->close(); };
    static size_t   read_cb  (void* buffer, size_t size, void* stream) { return m_stream_buffer()->read(buffer, size); };
    static size_t   write_cb (void* buffer, size_t size, void* stream) { return m_stream_buffer()->write(buffer, size); };
    static int64_t  skip_cb  (int64_t num, void* stream)               { return m_stream_buffer()->skip(num); };
    static int64_t  seek_cb  (int64_t num, void* stream)               { return m_stream_buffer()->seek(num); };

    void     close() {};
    size_t   read(void* buffer, size_t size)  { memcpy(buffer, m_pos, size); m_pos += size; return size; };
    size_t   write(void* buffer, size_t size) { 
        // static int fnum = 1;
        // FILE* f = 0;
        // char fname[64];
        // sprintf(fname, "/tmp/wr%04u.j2k", fnum++);
        // f = fopen(fname, "wb");
        // fwrite(buffer, 1, size, f);
        // fclose(f);
        // std::cout << "Write for " << size << " bytes\n"; 
        memcpy(m_pos, buffer, size); m_pos += size; m_size += size; return size; 
    };
    int64_t  skip(int64_t size) { std::cout << "Skip for" << size << " bytes\n"; m_pos += size; return m_pos - m_buffer; };
    int64_t  seek(int64_t size) { std::cout << "Seek for" << size << " bytes\n"; m_pos += size; return m_pos - m_buffer; };

    int64_t size() { return /* m_pos - m_buffer */ m_size; }

private:
    void* m_stream;

    uint8_t* m_buffer;
    uint8_t* m_pos;

    int64_t m_size;

    static mem_stream*& m_stream_buffer() {
        static mem_stream* ms = 0;

        return ms;
    }
};

class jpeg2k {
public:
    jpeg2k() {};
   ~jpeg2k() {};

    static void print(const char *msg, void *priv) {
        std::cout << "jpeg2k: " << msg << std::endl;
    };

    static int compress(uint8_t * yuyv, uint32_t width, uint32_t height, uint8_t * j2k, uint32_t size) { 
        static int fnum = 1;
        int ret = -1; // Failure

        opj_cparameters_t parameters;
        opj_set_default_encoder_parameters(&parameters);
        if(parameters.tcp_numlayers == 0)
        {
            parameters.tcp_rates[0] = 0; /* MOD antonin : losslessbug */
            parameters.tcp_numlayers++;
            parameters.cp_disto_alloc = 1;
        }

#define nr_comp 3

        uint32_t sub_dx = (unsigned int)parameters.subsampling_dx;
        uint32_t sub_dy = (unsigned int)parameters.subsampling_dy;
        opj_image_cmptparm_t cmptparm[nr_comp] = {0};
        for (int i = 0; i < nr_comp; i++) {
            cmptparm[i].prec = 8;
            cmptparm[i].bpp = 8;
            cmptparm[i].sgnd = 0;
            cmptparm[i].dx = sub_dx;
            cmptparm[i].dy = sub_dy;
            cmptparm[i].w = width;
            cmptparm[i].h = height;
        }
        opj_image_t *image = opj_image_create(nr_comp, cmptparm, OPJ_CLRSPC_SRGB /* OPJ_CLRSPC_SYCC */);
        if (image) {
            // TODO: initialize the image from the YUYV data supplied
            // for (uint32_t i = 0; i < width * height; i++) {
            //     unsigned int compno;
            //     for(compno = 0; compno < 3; compno++) {
            //         image->comps[compno].data[i] = 0;
            //     }
            // }        
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (x%2 == 0) {
                        image->comps[0].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 0]; // Y
                        image->comps[1].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 1]; // U
                        image->comps[2].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 3]; // V
                    } else {
                        image->comps[0].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 0]; // Y
                        image->comps[1].data[y * width + x] = yuyv[y * width * 2 + x * 2 - 1]; // U
                        image->comps[2].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 1]; // V
                    }
                }
            }

            image->x0 = 0;
            image->y0 = 0;
            image->x1 = (width - 1) * sub_dx + 1;
            image->y1 = (height - 1) * sub_dy + 1;            

            parameters.tcp_mct = image->numcomps == nr_comp ? 1 : 0;
            opj_codec_t* l_codec = opj_create_compress(OPJ_CODEC_J2K);
            if (l_codec) {
                // opj_set_info_handler   (l_codec, print, NULL);
                // opj_set_warning_handler(l_codec, print, NULL);
                // opj_set_error_handler  (l_codec, print, NULL);

                if (opj_setup_encoder(l_codec, &parameters, image)) {
                    opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_FALSE);
                    // char fname[64] = {0};
                    // sprintf(fname, "/tmp/str%04u.j2k", fnum++);
                    // opj_stream_t* l_stream = opj_stream_create_default_file_stream(fname, false);
                    if (l_stream) {
                        mem_stream ms(j2k);

                        opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn) NULL);
                        opj_stream_set_user_data_length(l_stream, size);
                        opj_stream_set_read_function(l_stream, (opj_stream_read_fn) mem_stream::read_cb);
                        opj_stream_set_write_function(l_stream, (opj_stream_write_fn) mem_stream::write_cb);
                        opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn) mem_stream::skip_cb);
                        opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn) mem_stream::seek_cb);

                        if (opj_start_compress(l_codec,image,l_stream)) {
                            if (opj_encode(l_codec, l_stream)) {
                                if (opj_end_compress(l_codec, l_stream)) {
                                    ret = ms.size(); // Success
                                } else print("opj_end_compress failed", NULL);
                            } else print("opj_encode failed", NULL);
                        } else print("opj_start_compress failed", NULL);
                        opj_stream_destroy(l_stream);
                    } else print("opj_stream_create failed", NULL);
                } else print("opj_setup_encoder failed", NULL);
                opj_destroy_codec(l_codec);
            } else print("opj_create_compress failed", NULL);
            opj_image_destroy(image);
        } else print("opj_image_create failed", NULL);

        return ret; 
    };

    static int decompress(uint8_t * j2k, uint32_t size, uint8_t * yuyv) { 
        int ret  = -1;

        opj_dparameters_t dparameters;

        opj_image_t *image = {0};

        opj_codec_t* d_codec = opj_create_decompress(OPJ_CODEC_J2K);
        opj_set_info_handler   (d_codec, print, NULL);
        opj_set_warning_handler(d_codec, print, NULL);
        opj_set_error_handler  (d_codec, print, NULL);

        if (opj_setup_decoder(d_codec, &dparameters)) {
            opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_TRUE);
            if (l_stream) {
                mem_stream ms(j2k);

                opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn) NULL);
                opj_stream_set_user_data_length(l_stream, size);
                opj_stream_set_read_function(l_stream, (opj_stream_read_fn) mem_stream::read_cb);
                opj_stream_set_write_function(l_stream, (opj_stream_write_fn) mem_stream::write_cb);
                opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn) mem_stream::skip_cb);
                opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn) mem_stream::seek_cb);

                if (opj_read_header(l_stream, d_codec, &image)) {
                    if (opj_decode(d_codec, l_stream, image)) {
                        if (opj_end_decompress(d_codec, l_stream)) {
                            print("decompression done", NULL);
                            ret = (image->x1 - image->x0) * (image->y1 - image->y0) * image->numcomps;
                            // TODO: convert image to the YUYV buffer supplied
                        } else print("opj_end_decompress failed", NULL);
                    } else print("opj_decode failed", NULL);
                } else print("opj_read_header failed", NULL);

                opj_stream_destroy(l_stream);
            } else print("opj_stream_create failed", NULL);
        } else print("opj_stream_create failed", NULL);

        opj_destroy_codec(d_codec);

        if (image) opj_image_destroy(image);

        return ret;
    };

private:
 
};

class JPEGEncodeFilter : public JPEGVideoSource /* FramedFilter */
{
public:
    static JPEGEncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEGEncodeFilter(t_env, source); }

    virtual Boolean isJPEGVideoSource() const { return True; };


protected:
    JPEGEncodeFilter(UsageEnvironment& t_env, FramedSource* source) : JPEGVideoSource(t_env), fInputSource(source) {
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE];
    } 

    virtual ~JPEGEncodeFilter() {
        delete[] m_framebuf_in;
        delete[] m_framebuf_out;
    };

private:
    FramedSource* fInputSource;

    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    virtual u_int8_t type()    { return 0; };
    virtual u_int8_t qFactor() { return 50; };
    virtual u_int8_t width()   { return 640 / 8; } // # pixels/8 (or 0 for 2048 pixels)
    virtual u_int8_t height()  { return 480 / 8; } // # pixels/8 (or 0 for 2048 pixels)

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, /* fMaxSize */ FRAME_SIZE, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEGEncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
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
        fFrameSize = compress(m_framebuf_in, 640, 480, fTo);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end-start;

        // std::cout << "Frame compression time " << elapsed.count() * 1000 << "ms,\tsize " << fFrameSize << "\n";

        // sprintf(fname, "/tmp/non%04u", fnum);
        // f = fopen(fname, "w");
        // fwrite(m_framebuf_out, 1, fFrameSize, f);
        // fclose(f);
#endif

        afterGetting(this);
    }

    int compress(uint8_t* in, int width, int height, uint8_t* out)
    {
        struct jpeg_compress_struct m_cinfo = {0};
        struct jpeg_error_mgr m_jerr = {0};

        uint8_t* out_data = out;
        uint64_t out_size = FRAME_SIZE;

        m_cinfo.err = jpeg_std_error(&m_jerr);
        jpeg_create_compress(&m_cinfo);
        jpeg_mem_dest(&m_cinfo, &out_data, &out_size);

        m_cinfo.image_width = width;
        m_cinfo.image_height = height;
        m_cinfo.input_components = 3; //yuyv and uyvy is 2 bpp, converted to yuv that is 3 bpp.
        m_cinfo.in_color_space = JCS_RGB;

        jpeg_set_defaults(&m_cinfo);
        jpeg_set_quality(&m_cinfo, 92, true);
//         m_cinfo.write_JFIF_header = false; // no header, RTP should create one
// std::cout << "JPEG will be generated using " << ((m_cinfo.arith_code == true) ? "arithmetic" : "Huffman") << " coding, "
//                                              << "4:" << m_cinfo.comp_info[0].v_samp_factor << ":" << m_cinfo.comp_info[0].h_samp_factor << " subsumpling, "
//                                              << ((m_cinfo.restart_interval == 0) ? "without" : "with") << " restart MCUs, and "
//                                              << ((m_cinfo.restart_in_rows  == 0) ? "without" : "with") << " restart rows\n"; 
        jpeg_start_compress(&m_cinfo, TRUE);

        std::vector<uint8_t> row(m_cinfo.image_width * m_cinfo.input_components);
        JSAMPROW row_pointer[1];
        row_pointer[0] = &row[0];
        uint8_t* buffer = in;
        while(m_cinfo.next_scanline < m_cinfo.image_height)
        {
            for(int i = 0; i < m_cinfo.image_width; i += 2)
            {
                row[i * 3 + 0] = buffer[i * 2 + 0]; // Y
                row[i * 3 + 1] = buffer[i * 2 + 1]; // U
                row[i * 3 + 2] = buffer[i * 2 + 3]; // V
                row[i * 3 + 3] = buffer[i * 2 + 2]; // Y
                row[i * 3 + 4] = buffer[i * 2 + 1]; // U
                row[i * 3 + 5] = buffer[i * 2 + 3]; // V
            }
            buffer += m_cinfo.image_width * 2;

            jpeg_write_scanlines(&m_cinfo, row_pointer, 1);
        }
        jpeg_finish_compress(&m_cinfo);
        jpeg_destroy_compress(&m_cinfo);

        return out_size;
    }

protected:
    virtual char const* MIMEtype() const { return "video/JPEG"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};

class JPEG2000EncodeFilter : public FramedFilter
{
public:
    static JPEG2000EncodeFilter* createNew(UsageEnvironment& t_env, FramedSource* source) { return new JPEG2000EncodeFilter(t_env, source); }

    virtual Boolean isJPEG2000VideoSource() const { return True; };


protected:
    JPEG2000EncodeFilter(UsageEnvironment& t_env, FramedSource* source) : FramedFilter(t_env, source) {
        m_framebuf_in  = new uint8_t[FRAME_SIZE];
        m_framebuf_out = new uint8_t[FRAME_SIZE];
    } 

    virtual ~JPEG2000EncodeFilter() {
        delete[] m_framebuf_in;
        delete[] m_framebuf_out;
    };

private:
    uint8_t* m_framebuf_in;
    uint8_t* m_framebuf_out;

    virtual void doGetNextFrame() 
    { 
        fInputSource->getNextFrame(m_framebuf_in, fMaxSize /* FRAME_SIZE */, afterGettingFrame, this, FramedSource::handleClosure, this);
    }

    static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds)
    { 
        ((JPEG2000EncodeFilter*)clientData)->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
    }

    void afterGettingFrame( unsigned frameSize,
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


protected:
    virtual char const* MIMEtype() const { return "video/JPEG2000"; }
    virtual void getAttributes() const { if (fInputSource) return fInputSource->getAttributes(); }
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); if (fInputSource != NULL) fInputSource->stopGettingFrames(); }
};


class RsDeviceSource : public FramedSource
{
public:
    static RsDeviceSource* createNew(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    static void waitForFrame(RsDeviceSource* t_deviceSource);

protected:
    RsDeviceSource(UsageEnvironment& t_env, RsSensor sensor, rs2::video_stream_profile& t_videoStreamProfile);
    virtual ~RsDeviceSource();

protected:
    virtual void doStopGettingFrames();

private:
    virtual void doGetNextFrame();
    rs2::frame_queue& getFramesQueue() { return m_framesQueue; };

private:
    RsSensor m_rsSensor;

    rs2::frame_queue m_framesQueue;
    rs2::video_stream_profile& m_streamProfile;
};

#if 0
    int compress(uint8_t* t_buffer, int width, int height, uint8_t* t_compressedBuf)
    {
        struct jpeg_error_mgr m_jerr;
        struct jpeg_compress_struct m_cinfo;
        struct jpeg_decompress_struct m_dinfo;
        JSAMPROW m_row_pointer[1];
        JSAMPARRAY m_destBuffer;
        unsigned char* m_rowBuffer;

        m_cinfo.err = jpeg_std_error(&m_jerr);
        m_dinfo.err = jpeg_std_error(&m_jerr);
        jpeg_create_compress(&m_cinfo);
        jpeg_create_decompress(&m_dinfo);

        m_cinfo.in_color_space = JCS_YCbCr;
        m_cinfo.input_components = 3; //yuyv and uyvy is 2 bpp, converted to yuv that is 3 bpp.

        m_rowBuffer = new unsigned char[m_cinfo.input_components * width];
        m_destBuffer = (*m_cinfo.mem->alloc_sarray)((j_common_ptr)&m_cinfo, JPOOL_IMAGE, m_cinfo.input_components * width, 1);
        jpeg_set_defaults(&m_cinfo);


        long unsigned int compressedSize = 0;
        jpeg_mem_dest(&m_cinfo, &t_compressedBuf, &compressedSize);
        m_cinfo.image_width = width;
        m_cinfo.image_height = height;
        uint64_t row_stride = m_cinfo.image_width * m_cinfo.input_components;
        jpeg_start_compress(&m_cinfo, TRUE);
        while(m_cinfo.next_scanline < m_cinfo.image_height)
        {
            for(int i = 0; i < m_cinfo.image_width; i += 2)
            {
                m_rowBuffer[i * 3]     = t_buffer[i * 2 + 0]; // Y
                m_rowBuffer[i * 3 + 1] = t_buffer[i * 2 + 1]; // U
                m_rowBuffer[i * 3 + 2] = t_buffer[i * 2 + 3]; // V
                m_rowBuffer[i * 3 + 3] = t_buffer[i * 2 + 2]; // Y
                m_rowBuffer[i * 3 + 4] = t_buffer[i * 2 + 1]; // U
                m_rowBuffer[i * 3 + 5] = t_buffer[i * 2 + 3]; // V
            }
            m_row_pointer[0] = m_rowBuffer;
            (*t_buffer) += m_cinfo.image_width * m_cinfo.input_components;
            jpeg_write_scanlines(&m_cinfo, m_row_pointer, 1);
        }
        jpeg_finish_compress(&m_cinfo);
        return 0;
    }

#define READ_JP2 1
#define WRITE_JP2 0

    static opj_image_t* to_opj_image(const unsigned char* buf, int width, int height, int nr_comp, int sub_dx, int sub_dy)
    {
        const unsigned char* cs;
        opj_image_t* image;
        int *r, *g, *b, *a;
        int has_rgba, has_graya, has_rgb, comp;
        unsigned int i, max;
        opj_image_cmptparm_t cmptparm[4];

        memset(&cmptparm, 0, 4 * sizeof(opj_image_cmptparm_t));

        for(comp = 0; comp < nr_comp; ++comp)
        {
            cmptparm[comp].prec = 8;
            cmptparm[comp].bpp = 8;
            cmptparm[comp].sgnd = 0;
            cmptparm[comp].dx = sub_dx;
            cmptparm[comp].dy = sub_dy;
            cmptparm[comp].w = width;
            cmptparm[comp].h = height;
        }

        {
            OPJ_COLOR_SPACE csp = (nr_comp > 2 ? OPJ_CLRSPC_SRGB : OPJ_CLRSPC_GRAY);

            image = opj_image_create(nr_comp, &cmptparm[0], csp);
        }

        if(image == NULL)
        {
            fprintf(stderr, "%d: got no image\n", __LINE__);
            return NULL;
        }

        image->x0 = 0;
        image->y0 = 0;
        image->x1 = (width - 1) * sub_dx + 1;
        image->y1 = (height - 1) * sub_dy + 1;

        r = g = b = a = NULL;
        has_rgba = has_graya = has_rgb = 0;

        if(nr_comp == 4) /* RGBA */
        {
            has_rgba = 1;
            r = image->comps[0].data;
            g = image->comps[1].data;
            b = image->comps[2].data;
            a = image->comps[3].data;
        }
        else if(nr_comp == 2) /* GA */
        {
            has_graya = 1;
            r = image->comps[0].data;
            a = image->comps[1].data;
        }
        else if(nr_comp == 3) /* RGB */
        {
            has_rgb = 1;
            r = image->comps[0].data;
            g = image->comps[1].data;
            b = image->comps[2].data;
        }
        else /* G */
        {
            r = image->comps[0].data;
        }
        cs = buf;
        max = height * width;

        for(i = 0; i < max; ++i)
        {
            if(has_rgba)
            {
#ifdef OPJ_BIG_ENDIAN
                //RGBA
                *r++ = (int)*cs++;
                *g++ = (int)*cs++;
                *b++ = (int)*cs++;
                ;
                *a++ = (int)*cs++;
                continue;
#else
                //RGBA
                *r++ = (int)*cs++;
                *g++ = (int)*cs++;
                *b++ = (int)*cs++;
                ;
                *a++ = (int)*cs++;
                continue;
#endif
            }

            if(has_rgb)
            {
#ifdef OPJ_BIG_ENDIAN
                //RGB
                *r++ = (int)*cs++;
                *g++ = (int)*cs++;
                *b++ = (int)*cs++;
                continue;
#else
                //RGB
                *r++ = (int)*cs++;
                *g++ = (int)*cs++;
                *b++ = (int)*cs++;
                continue;
#endif
            }

            if(has_graya)
            {
#ifdef OPJ_BIG_ENDIAN
                //RA
                *r++ = (int)*cs++;
                *a++ = (int)*cs++;
                continue;
#else
                //RA
                *r++ = (int)*cs++;
                *a++ = (int)*cs++;
                continue;
#endif
            }
            /* G */
            *r++ = (int)*cs++;

        } // for(i = 0; i < max; ++i)

        return image;

    } // to_opj_image()

    //
    // rgb_buffer ==> image ==> file
    //
    void JP2_file_from_rgb(const unsigned char* buf, unsigned int width, unsigned int height, unsigned int numcomps, const char* write_idf)
    {
        opj_cparameters_t parameters;
        opj_image_t* image;
        int sub_dx, sub_dy;
        OPJ_CODEC_FORMAT codec_format;

        codec_format = OPJ_CODEC_JP2;

        opj_set_default_encoder_parameters(&parameters);

        if(parameters.cp_comment == NULL)
        {
            char buf[80];
#ifdef _WIN32
            sprintf_s(buf, 80, "Created by OpenJPEG version %s", opj_version());
#else
            snprintf(buf, 80, "Created by OpenJPEG version %s", opj_version());
#endif
            parameters.cp_comment = strdup(buf);
        }

        if(parameters.tcp_numlayers == 0)
        {
            parameters.tcp_rates[0] = 0; /* MOD antonin : losslessbug */
            parameters.tcp_numlayers++;
            parameters.cp_disto_alloc = 1;
        }
        sub_dx = parameters.subsampling_dx;
        sub_dy = parameters.subsampling_dy;

        //--------------------------------------------------------
        image = to_opj_image(buf, (int)width, (int)height, (int)numcomps, sub_dx, sub_dy);
        //--------------------------------------------------------
        if(image == NULL)
        {
            fprintf(stderr, "%d: write_jp2_file fails.\n", __LINE__);

            if(parameters.cp_comment)
                free(parameters.cp_comment);
            return;
        }
        {
            opj_stream_t* stream;
            opj_codec_t* codec;

            stream = NULL;
            parameters.tcp_mct = image->numcomps == 3 ? 1 : 0;

            codec = opj_create_compress(codec_format);

            if(codec == NULL)
                goto fin;

            opj_setup_encoder(codec, &parameters, image);

            stream = opj_stream_create_default_file_stream(write_idf, WRITE_JP2);

            if(stream == NULL)
                goto fin;

            if(!opj_start_compress(codec, image, stream))
                goto fin;

            if(!opj_encode(codec, stream))
                goto fin;

            opj_end_compress(codec, stream);

        fin:

            if(stream)
                opj_stream_destroy(stream);

            if(codec)
                opj_destroy_codec(codec);

            opj_image_destroy(image);

            if(parameters.cp_comment)
                free(parameters.cp_comment);
        }

    } //JP2_file_from_rgb()

#endif
