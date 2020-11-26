// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include <../client/RsNetDevice.h>

#include <iostream>
#include <iomanip>
#include <queue>
#include <map>
#include <chrono>
#include <memory>
#include <utility>
#include <mutex>

#include <librealsense2/rs.hpp>

#include <zstd.h>
#include <zstd_errors.h>

#include <lz4.h>

using FrameData     = std::shared_ptr<uint8_t[]>;
using FrameDataQ    = std::shared_ptr<std::queue<FrameData>>;
using ConsumerQMap  = std::map<void*, FrameDataQ>;
using ConsumerQPair = std::pair<void*, FrameDataQ>;

////////////////////////////////////////////////////////////

// #define CHUNK_SIZE (2*1024)
// typedef struct chunk_header{
//     uint32_t size;
//     uint32_t offset;
// } chunk_header_t;
// #define CHUNK_HLEN (sizeof(chunk_header_t))

class frames_queue {
public:
    frames_queue(rs2::sensor sensor, rs2::video_stream_profile stream)
        : m_sensor(sensor), m_stream(stream) {};
   ~frames_queue() { 
        for (ConsumerQMap::iterator it = m_queues.begin(); it != m_queues.end(); ++it)
            stop(it->first);
    }; // TODO: improve

    bool is_streaming() { return m_sensor.get_active_streams().size() > 0; };
    
    uint32_t get_type()   { return m_stream.stream_type();   };
    uint32_t get_index()  { return m_stream.stream_index();  };
    uint32_t get_fps()    { return m_stream.fps();           };

    uint32_t get_width()  { return m_stream.width();  };
    uint32_t get_height() { return m_stream.height(); };

    uint32_t get_size()   { return 640*480*2; };

    FrameData get_frame(void* consumer)  { 
        if (m_queues.find(consumer) == m_queues.end()) {
            // new consumer - allocate the queue for it
            FrameDataQ q(new std::queue<FrameData>);
            m_queues.insert(ConsumerQPair(consumer, q));
        }

        if (!is_streaming()) {
            // start the sensor
            std::cout << "Sensor started for stream: " << std::setw(15) << get_type()
                                                       << std::setw(15) << rs2_format_to_string(m_stream.format())      
                                                       << std::setw(15) << get_width() << "x" << get_height() << "x" << get_fps() << std::endl;    

            m_sensor.open(m_stream);
            m_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0); // TODO: should be removed

            auto callback = [&](const rs2::frame& frame) {
#if 1                
                static uint32_t frame_count = 0;    
                static std::chrono::_V2::system_clock::time_point beginning = std::chrono::system_clock::now();

                auto start = std::chrono::system_clock::now();

                uint8_t* data = (uint8_t*)frame.get_data();
                uint32_t size = frame.get_data_size();

                uint32_t offset = 0;
                uint32_t out_size = 0;

                while (offset < size) {
                    FrameData chunk(new uint8_t[CHUNK_SIZE + CHUNK_HLEN]);
                    chunk_header_t* ch = (chunk_header_t*)chunk.get();
                    ch->offset = offset;
                    ch->size   = sizeof(chunk_header_t);
  #ifdef COMPRESSION_ENABLED
    #ifdef COMPRESSION_ZSTD                    
                    ch->size  += ZSTD_compress((void*)(chunk.get() + CHUNK_HLEN), CHUNK_SIZE, (void*)(data + offset), size - offset > CHUNK_SIZE ? CHUNK_SIZE : size - offset, 1);
    #else
                    // ch->size  += LZ4_compress_fast((const char*)(data + offset), (char*)(chunk.get() + CHUNK_HLEN), size - offset > CHUNK_SIZE ? CHUNK_SIZE : size - offset, CHUNK_SIZE, 10);
                    ch->size  += LZ4_compress_default((const char*)(data + offset), (char*)(chunk.get() + CHUNK_HLEN), size - offset > CHUNK_SIZE ? CHUNK_SIZE : size - offset, CHUNK_SIZE);
    #endif
  #else
                    memcpy((void*)(chunk.get() + CHUNK_HLEN), (void*)(data + offset), size - offset > CHUNK_SIZE ? CHUNK_SIZE : size - offset);
                    ch->size  += CHUNK_SIZE;
  #endif                    
                    out_size  += ch->size;
                    offset    += CHUNK_SIZE;

                    // push the chunk to queues
                    for (ConsumerQMap::iterator it = m_queues.begin(); it != m_queues.end(); ++it) 
                    {
                        std::lock_guard<std::mutex> lck (m_queues_mutex);
                        (it->second)->push(chunk);
                    }
                }

                auto end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                std::chrono::duration<double> total_time = end - beginning;
                frame_count++;
                double fps = 0;
                if (total_time.count() > 0) fps = (double)frame_count / (double)total_time.count();
                std::cout << "Frame compression time " << std::fixed << std::setw(5) << std::setprecision(2) 
                          << elapsed.count() * 1000 << "ms,\tsize " << size << " => " << out_size << " (" << (float)(size) / (float)out_size << " ), FPS: " << fps << "\n";
                
                if (total_time > std::chrono::seconds(1)) {
                    beginning = std::chrono::system_clock::now();
                    frame_count = 0;
                }

#else
                uint8_t* data = (uint8_t*)frame.get_data();
                uint32_t size = frame.get_data_size();
                FrameData chunk(new uint8_t[size]);
                memcpy(chunk.get(), data, size);
                // push the chunk to queues
                for (ConsumerQMap::iterator it = m_queues.begin(); it != m_queues.end(); ++it)
                    (it->second)->push(chunk);

#endif                          
            };
            m_sensor.start(callback);            
        }

        FrameData frame = nullptr;
        if (!m_queues[consumer]->empty()) 
        {
            std::lock_guard<std::mutex> lck (m_queues_mutex);
            frame = m_queues[consumer]->front();
            m_queues[consumer]->pop(); 
        }

        return frame;
    };

    uint32_t stop(void* consumer) { 
        m_queues.erase(consumer);
        if (m_queues.empty() && is_streaming()) {
            m_sensor.stop();
            m_sensor.close();
        }
    };

private:
    rs2::sensor               m_sensor;
    rs2::video_stream_profile m_stream;

    std::mutex m_queues_mutex;
    ConsumerQMap  m_queues;
};
