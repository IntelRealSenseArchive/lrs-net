// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include <iostream>
#include <iomanip>
#include <queue>
#include <map>

#include <librealsense2/rs.hpp>

typedef std::map<void*, std::queue<uint8_t*>*> consumer_queue_map;

class frames_queue {
public:
    frames_queue(rs2::sensor sensor, rs2::video_stream_profile stream)
        : m_sensor(sensor), m_stream(stream) {};
   ~frames_queue() { 
        for (std::map<void*, std::queue<uint8_t*>*>::iterator it = m_queues.begin(); it != m_queues.end(); ++it)
            stop(it->first);
    }; // TODO: improve

    bool is_streaming() { return m_sensor.get_active_streams().size() > 0; };
    
    uint32_t get_type()   { return m_stream.stream_type();   };
    uint32_t get_index()  { return m_stream.stream_index();  };
    uint32_t get_fps()    { return m_stream.fps();           };

    uint32_t get_width()  { return m_stream.width();  };
    uint32_t get_height() { return m_stream.height(); };

    uint32_t get_size()   { return get_width() * get_height() * 2; /* Color is 2 BPP */ };

    uint8_t* get_frame(void* consumer)  { 
        if (m_queues.find(consumer) == m_queues.end()) {
            // new consumer - allocate the queue for it
            std::queue<uint8_t*>* q = new std::queue<uint8_t*>;
            m_queues.insert(std::pair<void*, std::queue<uint8_t*>*>(consumer, q));
        }

        if (!is_streaming()) {
            // start the sensor
            std::cout << "Sensor started for stream: " << std::setw(15) << get_type()
                                                       << std::setw(15) << rs2_format_to_string(m_stream.format())      
                                                       << std::setw(15) << get_width() << "x" << get_height() << "x" << get_fps() << std::endl;    

            m_sensor.open(m_stream);

            auto callback = [&](const rs2::frame& frame) {
                //push frame to its queue
                for (std::map<void*, std::queue<uint8_t*>*>::iterator it = m_queues.begin(); it != m_queues.end(); ++it)
                    (it->second)->push((uint8_t*)frame.get_data());
            };
            m_sensor.start(callback);            
        }

        uint8_t* frame = NULL;
        if (!m_queues[consumer]->empty()) {
            frame = m_queues[consumer]->front();
            m_queues[consumer]->pop(); 
        }

        return frame;
    };

    uint32_t stop(void* consumer) { 
        delete m_queues[consumer];
        m_queues.erase(consumer);
        if (m_queues.empty() && is_streaming()) {
            m_sensor.stop();
            m_sensor.close();
        }
    };

private:
    rs2::sensor               m_sensor;
    rs2::video_stream_profile m_stream;

    std::queue<uint8_t*> m_queue;
    std::map<void*, std::queue<uint8_t*>*> m_queues;
};
