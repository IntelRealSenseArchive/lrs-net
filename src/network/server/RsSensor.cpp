// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "RsDevice.hh"
#include "RsUsageEnvironment.h"
#include "compression/CompressionFactory.h"
#include "string.h"
#include <BasicUsageEnvironment.hh>
#include <iostream>
#include <math.h>
#include <thread>

RsSensor::RsSensor(UsageEnvironment* t_env, rs2::device t_device, rs2::sensor t_sensor)
    : env(t_env)
    , m_sensor(t_sensor)
    , m_device(t_device)
{
    for(rs2::stream_profile streamProfile : m_sensor.get_stream_profiles())
    {
        if(streamProfile.is<rs2::video_stream_profile>())
        {
            //make a map with all the sensor's stream profiles
            m_streamProfiles.emplace(getStreamProfileKey(streamProfile), streamProfile.as<rs2::video_stream_profile>());
            m_prevSample.emplace(getStreamProfileKey(streamProfile), std::chrono::high_resolution_clock::now());
        }
    }
}

int RsSensor::open(rs2::video_stream_profile& profile)
{
    std::cout << "Sensor opened for stream : " << std::setw(15) << rs2_stream_to_string(profile.stream_type()) 
                                               << std::setw(15) << rs2_format_to_string(profile.format())      
                                               << std::setw(15) << profile.width() << "x" << profile.height() << "x" << profile.fps() << std::endl;    
    m_sensor.open(profile);
    return EXIT_SUCCESS;
}

int RsSensor::close()
{
    std::cout << "Sensor closed" << std::endl;
    m_sensor.close();
    return EXIT_SUCCESS;
}

int RsSensor::stop()
{
    std::cout << "Sensor stopped" << std::endl;
    try {
        m_sensor.stop();
    } catch (...) {
        std::cout << "Sensor stopped while stopped" << std::endl;
    };
    return EXIT_SUCCESS;
}

int RsSensor::start(rs2::video_stream_profile& profile, rs2::frame_queue& queue)
{
    std::cout << "Sensor started for stream: " << std::setw(15) << rs2_stream_to_string(profile.stream_type()) 
                                               << std::setw(15) << rs2_format_to_string(profile.format())      
                                               << std::setw(15) << profile.width() << "x" << profile.height() << "x" << profile.fps() << std::endl;    

    auto callback = [&](const rs2::frame& frame) {
        //push frame to its queue
        queue.enqueue(frame);
    };
    m_sensor.start(callback);
    return EXIT_SUCCESS;
}

long long int RsSensor::getStreamProfileKey(rs2::stream_profile t_profile)
{
    long long int key;
    key = t_profile.stream_type() * pow(10, 12) + t_profile.format() * pow(10, 10) + t_profile.fps() * pow(10, 8) + t_profile.stream_index();
    if(t_profile.is<rs2::video_stream_profile>())
    {
        rs2::video_stream_profile videoStreamProfile = t_profile.as<rs2::video_stream_profile>();
        key += videoStreamProfile.width() * pow(10, 4) + videoStreamProfile.height();
    }
    return key;
}

std::string RsSensor::getSensorName()
{
    if(m_sensor.supports(RS2_CAMERA_INFO_NAME))
    {
        return std::string(m_sensor.get_info(RS2_CAMERA_INFO_NAME));
    }
    else
    {
        return "unknown";
    }
}

int RsSensor::getStreamProfileBpp(rs2_format t_format)
{
    int bpp = 0;
    switch(t_format)
    {
    case RS2_FORMAT_RGB8:
    {
        bpp = 3;
        break;
    }
    case RS2_FORMAT_BGR8:
    {
        bpp = 3;
        break;
    }
    case RS2_FORMAT_RGBA8:
    {
        bpp = 3; //TODO: need to be 4 bpp, change it after add support for 4 bpp formats
        break;
    }
    case RS2_FORMAT_BGRA8:
    {
        bpp = 3; //TODO: need to be 4 bpp, change it after add support for 4 bpp formats
        break;
    }
    case RS2_FORMAT_Z16:
    case RS2_FORMAT_Y16:
    case RS2_FORMAT_Y8:
    case RS2_FORMAT_RAW16:
    case RS2_FORMAT_YUYV:
    case RS2_FORMAT_UYVY:
    {
        bpp = 2;
        break;
    }
    default:
        bpp = 0;
        break;
    }
    return bpp;
}

std::vector<RsOption> RsSensor::getSupportedOptions()
{
    std::vector<RsOption> returnedVector;
    try
    {

        std::vector<rs2_option> options = m_sensor.get_supported_options();
        for(auto opt : options)
        {
            if(!m_sensor.supports(opt))
                continue;

            RsOption option;
            option.m_opt = opt;
            option.m_range = m_sensor.get_option_range(opt);
            returnedVector.push_back(option);
        }
    }
    catch(const std::exception& e)
    {
        *env << e.what() << "\n";
    }
    return returnedVector;
}
