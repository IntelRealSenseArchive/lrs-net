#include <stdlib.h>

#include <map>
#include <vector>
#include <iostream>

#include <jpeg.h>

int jpeg::compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size)
{
    struct jpeg_compress_struct m_cinfo = {0};
    struct jpeg_error_mgr m_jerr = {0};

    uint8_t* out_data = out;
    long unsigned int out_size = size;

    m_cinfo.err = jpeg_std_error(&m_jerr);
    jpeg_create_compress(&m_cinfo);
    jpeg_mem_dest(&m_cinfo, &out_data, &out_size);

    m_cinfo.image_width = width;
    m_cinfo.image_height = height;
    m_cinfo.input_components = 3; //yuyv and uyvy is 2 bpp, converted to yuv that is 3 bpp.
    m_cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&m_cinfo);
    jpeg_set_quality(&m_cinfo, 30, true);
    m_cinfo.dct_method = JDCT_FASTEST;
    // m_cinfo.arith_code == true;        // Huffman coding is much better than arithmetic
    // m_cinfo.write_JFIF_header = false; // no header, RTP should create one

#if 1
std::cout << "JPEG will be generated using " << ((m_cinfo.arith_code == true) ? "arithmetic" : "Huffman") << " coding, "
                                             << "4:" << m_cinfo.comp_info[0].v_samp_factor << ":" << m_cinfo.comp_info[0].h_samp_factor << " subsumpling, "
                                             << ((m_cinfo.restart_interval == 0) ? "without" : "with") << " restart MCUs, and "
                                             << ((m_cinfo.restart_in_rows  == 0) ? "without" : "with") << " restart rows\n"; 
#endif

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

int jpeg::decompress(unsigned char* in, int in_size, unsigned char* out, uint32_t out_size)
{
    struct jpeg_decompress_struct m_dinfo = {0};
    struct jpeg_error_mgr m_jerr = {0};

    m_dinfo.err = jpeg_std_error(&m_jerr);
    jpeg_create_decompress(&m_dinfo);
    // m_dinfo.out_color_space = JCS_UNKNOWN;

    jpeg_mem_src(&m_dinfo, in, in_size);
    jpeg_read_header(&m_dinfo, TRUE);

    std::vector<uint8_t> row(out_size);
    JSAMPROW row_pointer[1];
    row_pointer[0] = &row[0];

    m_dinfo.dct_method = JDCT_FASTEST;

    jpeg_start_decompress(&m_dinfo);
    while(m_dinfo.output_scanline < m_dinfo.output_height)
    {
        int numLines = jpeg_read_scanlines(&m_dinfo, row_pointer, 1);
        for(int i = 0; i < m_dinfo.output_width; i += 2)
        {
            out[i * 2 + 0] = row[i * 3 + 0]; // Y
            out[i * 2 + 1] = row[i * 3 + 1]; // U
            out[i * 2 + 2] = row[i * 3 + 3]; // Y
            out[i * 2 + 3] = row[i * 3 + 2]; // V
        }
        out += m_dinfo.output_width * 2;
    }
    jpeg_finish_decompress(&m_dinfo);
    jpeg_destroy_decompress(&m_dinfo);

    return 0;
}
