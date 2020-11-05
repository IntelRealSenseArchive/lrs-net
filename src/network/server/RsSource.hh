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

#define FRAME_SIZE 640*480*2

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
        fFrameSize = compress(m_framebuf_in, 640, 480, fTo);

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

/*
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
*/