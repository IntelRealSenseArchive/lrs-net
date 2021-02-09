// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#include <option.h>
#include <software-device.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>

#include <list>
#include <sstream>

#include "RsVideoRTPSource.h"

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

#define RS2_NET_MAJOR_VERSION    1
#define RS2_NET_MINOR_VERSION    0
#define RS2_NET_PATCH_VERSION    0

#define COMPRESSION_ENABLED
#define COMPRESSION_ZSTD

#define CHUNK_SIZE (1024*2)
#pragma pack (push, 1)
typedef struct chunk_header{
    uint16_t size;
    uint32_t offset;
    uint32_t index;
    uint8_t  status;
    uint8_t  meta_id;
    uint64_t meta_data;
} chunk_header_t;
#pragma pack(pop)
#define CHUNK_HLEN (sizeof(chunk_header_t))

