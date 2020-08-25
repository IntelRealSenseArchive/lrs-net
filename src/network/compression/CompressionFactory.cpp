// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "CompressionFactory.h"
#include "JpegCompression.h"
#include "Lz4Compression.h"
#include "RvlCompression.h"

std::shared_ptr<ICompression> CompressionFactory::getObject(int t_width, int t_height, rs2_format t_format, rs2_stream t_streamType, int t_bpp)
{
    ZipMethod zipMeth = ZipMethod::lz;
    if(t_streamType == RS2_STREAM_COLOR || t_streamType == RS2_STREAM_INFRARED)
    {
        zipMeth = ZipMethod::jpeg;
    }
    else if(t_streamType == RS2_STREAM_DEPTH)
    {
        zipMeth = ZipMethod::lz;
    }
    if(!isSupported(t_format, t_streamType))
    {
        return nullptr;
    }

    switch(zipMeth)
    {
    case ZipMethod::rvl:
        return std::make_shared<RvlCompression>(t_width, t_height, t_format, t_bpp);
        break;
    case ZipMethod::jpeg:
        return std::make_shared<JpegCompression>(t_width, t_height, t_format, t_bpp);
        break;
    case ZipMethod::lz:
        return std::make_shared<Lz4Compression>(t_width, t_height, t_format, t_bpp);
        break;
    default:
        LOG_ERROR("Unknown compression method");
        return nullptr;
    }
}

bool CompressionFactory::isEnabled()
{
    return set_mode(true, true);
}

bool CompressionFactory::set_mode(bool mode, bool read = false)
{
    static bool m_enabled = true;

    if (!read) m_enabled = mode;

    return m_enabled;
}

void CompressionFactory::enable()
{
    set_mode(true);
}

void CompressionFactory::disable()
{
    set_mode(false);
}

bool CompressionFactory::isSupported(rs2_format t_format, rs2_stream t_streamType)
{
    if (isEnabled() == false)
    {
        return false;
    }

    if ((t_streamType == RS2_STREAM_COLOR || t_streamType == RS2_STREAM_INFRARED) && 
        (t_format != RS2_FORMAT_BGR8 && t_format != RS2_FORMAT_RGB8 && t_format != RS2_FORMAT_Y8 && t_format != RS2_FORMAT_YUYV && t_format != RS2_FORMAT_UYVY))
    {
        return false;
    }
    return true;
}
