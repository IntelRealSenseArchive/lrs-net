// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "RsRtpCallback.h"
#include <RsCommon.h>

#include <librealsense2/hpp/rs_internal.hpp>

#include <vector>

struct DeviceData
{
    std::string serialNum;
    std::string name;
    std::string usbType;
};

class RsRtsp
{
public:
    virtual std::vector<rs2_video_stream> getStreams() = 0;
    virtual int addStream(rs2_video_stream t_stream, rs_rtp_callback* t_frameCallBack) = 0;
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual int close() = 0;
    virtual float getOption(const std::string& t_sensorName, rs2_option t_option) = 0;
    virtual int setOption(const std::string& t_sensorName, rs2_option t_option, float t_value) = 0;
    virtual DeviceData getDeviceData() = 0;
    virtual std::vector<IpDeviceControlData> getControls() = 0;
};
