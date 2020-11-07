#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <chrono>

#include "jpeg2k.h"

// #define OPENJPEG_DEBUG
#define THREAD_NUM 2

//// mem_stream implementation ///////////////////////////////////////////

size_t mem_stream::read(void* buffer, size_t size)
{
    if (m_pos + size > m_buffer + m_size_buf) {
        size = m_size_buf - (m_pos - m_buffer);
    }
    memcpy(buffer, m_pos, size);
    m_pos += size;
    return m_pos - m_buffer;
}

size_t mem_stream::write(void* buffer, size_t size)
{
    if (m_pos + size > m_buffer + m_size_buf) {
        size = m_size_buf - (m_pos - m_buffer);
    }
    memcpy(m_pos, buffer, size);
    m_pos += size;
    if ((m_pos - m_buffer) > m_size_use) m_size_use = m_pos - m_buffer;
    return m_pos - m_buffer;
}

int64_t mem_stream::skip(int64_t size)
{
    return seek(size);
}

int64_t mem_stream::seek(int64_t size)
{
    if(m_pos + size > m_buffer + m_size_buf) {
        m_pos = m_buffer + m_size_buf;
    } else {
        m_pos += size;
    }
    return m_pos - m_buffer;
}

//// jpeg2k implementation ///////////////////////////////////////////////

void jpeg2k::print(const char* msg, void* priv)
{
    std::cout << "jpeg2k: " << msg << std::endl;
}

int jpeg2k::compress(uint8_t* yuyv, uint32_t width, uint32_t height, uint8_t* j2k, uint32_t size)
{
    int ret = 0; // Failure

    opj_cparameters_t parameters;
    opj_set_default_encoder_parameters(&parameters);
    if (parameters.tcp_numlayers == 0) {
        parameters.tcp_rates[0] = 0; /* MOD antonin : losslessbug */
        parameters.tcp_numlayers++;
        parameters.cp_disto_alloc = 1;
    }

#define nr_comp 3

    uint32_t sub_dx = (unsigned int)parameters.subsampling_dx;
    uint32_t sub_dy = (unsigned int)parameters.subsampling_dy;
    opj_image_cmptparm_t cmptparm[nr_comp] = {0};
    for(int i = 0; i < nr_comp; i++) {
        cmptparm[i].prec = 8;
        cmptparm[i].bpp = 8;
        cmptparm[i].sgnd = 0;
        if (i) cmptparm[i].dx = sub_dx * 2;
        else   cmptparm[i].dx = sub_dx;
        cmptparm[i].dy = sub_dy;
        cmptparm[i].w = width;
        cmptparm[i].h = height;
    }
    opj_image_t* image = opj_image_create(nr_comp, cmptparm, OPJ_CLRSPC_SRGB /* OPJ_CLRSPC_SYCC */);
    if (image) {
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                image->comps[0].data[y * width + x] = yuyv[y * width * 2 + x * 2 + 0]; // Y
            }
            for(int x = 0; x < width / 2; x++) {
                image->comps[1].data[y * (width/2) + x] = yuyv[y * width * 2 + x * 4 + 1]; // U
                image->comps[2].data[y * (width/2) + x] = yuyv[y * width * 2 + x * 4 + 3]; // V
            }
        }

        image->x0 = 0;
        image->y0 = 0;
        image->x1 = (width - 1) * sub_dx + 1;
        image->y1 = (height - 1) * sub_dy + 1;

        parameters.tcp_mct = image->numcomps == nr_comp ? 1 : 0;
        opj_codec_t* l_codec = opj_create_compress(OPJ_CODEC_J2K);
        if (l_codec) {
#ifdef OPENJPEG_DEBUG            
            opj_set_info_handler   (l_codec, print, NULL);
            opj_set_warning_handler(l_codec, print, NULL);
            opj_set_error_handler  (l_codec, print, NULL);
#endif            

            if (opj_setup_encoder(l_codec, &parameters, image))
            {
                opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_FALSE);
                if(l_stream) {
                    mem_stream ms(j2k, size, false); // outgoing size

                    opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn)NULL);
                    opj_stream_set_user_data_length(l_stream, size);
                    opj_stream_set_read_function(l_stream, (opj_stream_read_fn)mem_stream::read_cb);
                    opj_stream_set_write_function(l_stream, (opj_stream_write_fn)mem_stream::write_cb);
                    opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn)mem_stream::skip_cb);
                    opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn)mem_stream::seek_cb);

                    opj_codec_set_threads(l_codec, THREAD_NUM);

#ifdef OPENJPEG_DEBUG            
                    auto start = std::chrono::system_clock::now();
#endif            
                    if(opj_start_compress(l_codec, image, l_stream)) {
                        if(opj_encode(l_codec, l_stream)) {
                            if(opj_end_compress(l_codec, l_stream)) {
#ifdef OPENJPEG_DEBUG            
                                auto end = std::chrono::system_clock::now();
                                std::chrono::duration<double> elapsed = end-start;
                                std::cout << "Pure compression time " << elapsed.count() * 1000 << " ms\n";
#endif            
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
}

int jpeg2k::decompress(uint8_t* j2k, uint32_t size, uint8_t* yuyv)
{
    int ret = 0;

    opj_dparameters_t dparameters;
    opj_image_t* image = {0};
    opj_codec_t* d_codec = opj_create_decompress(OPJ_CODEC_J2K);

#ifdef OPENJPEG_DEBUG            
    opj_set_info_handler(d_codec, print, NULL);
    opj_set_warning_handler(d_codec, print, NULL);
    opj_set_error_handler(d_codec, print, NULL);
#endif

    if (opj_setup_decoder(d_codec, &dparameters)) {
        opj_stream_t* l_stream = opj_stream_create(OPJ_J2K_STREAM_CHUNK_SIZE, OPJ_TRUE);
        if (l_stream) {
            mem_stream ms(j2k, size, true); // incoming size

            opj_stream_set_user_data(l_stream, (void*)j2k, (opj_stream_free_user_data_fn)NULL);
            opj_stream_set_user_data_length(l_stream, size);
            opj_stream_set_read_function(l_stream, (opj_stream_read_fn)mem_stream::read_cb);
            opj_stream_set_write_function(l_stream, (opj_stream_write_fn)mem_stream::write_cb);
            opj_stream_set_skip_function(l_stream, (opj_stream_skip_fn)mem_stream::skip_cb);
            opj_stream_set_seek_function(l_stream, (opj_stream_seek_fn)mem_stream::seek_cb);

            opj_codec_set_threads(d_codec, THREAD_NUM);

            if(opj_read_header(l_stream, d_codec, &image)) {
#ifdef OPENJPEG_DEBUG            
                auto start = std::chrono::system_clock::now();
#endif
                if(opj_decode(d_codec, l_stream, image)) {
                    if(opj_end_decompress(d_codec, l_stream)) {
#ifdef OPENJPEG_DEBUG            
                        auto end = std::chrono::system_clock::now();
                        std::chrono::duration<double> elapsed = end-start;
                        std::cout << "Pure decompression time " << elapsed.count() * 1000 << " ms\n";
#endif
                        // YUV planar image
                        ret = (image->x1 - image->x0) * (image->y1 - image->y0) * image->numcomps;
                        // convert image to the YUYV interlaced image
                        int width = image->x1 - image->x0;
                        int height = image->y1 - image->y0;
                        for(int y = 0; y < height; y++) {
                            for(int x = 0; x < width; x++) {
                                yuyv[y * width * 2 + x * 2 + 0] = image->comps[0].data[y * width + x]; // Y
                            }
                            for(int x = 0; x < width/2; x++) {
                                yuyv[y * width * 2 + x * 4 + 1] = image->comps[1].data[y * (width/2) + x]; // U
                                yuyv[y * width * 2 + x * 4 + 3] = image->comps[2].data[y * (width/2) + x]; // V
                            }
                        }
                        ret = FRAME_SIZE;
                    } else print("opj_end_decompress failed", NULL);
                } else print("opj_decode failed", NULL);
            } else print("opj_read_header failed", NULL);
            opj_stream_destroy(l_stream);
        } else print("opj_stream_create failed", NULL);
    } else print("opj_stream_create failed", NULL);

    opj_destroy_codec(d_codec);

    if (image) opj_image_destroy(image);

    return ret;
}
