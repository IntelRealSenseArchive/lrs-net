// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include "liveMedia.hh"
#include <BasicUsageEnvironment.hh>

#include "RsSensor.hh"

#include <librealsense2/rs.hpp>

///
#include <thread>
#include <chrono>
#include <iostream>

class RsDeviceSource : public FramedSource
{
public:
    static RsDeviceSource* createNew(UsageEnvironment& t_env, frames_queue* pqs) {
        return new RsDeviceSource(t_env, pqs);
    };

    virtual char const* MIMEtype() const { return "video/LZ4"; };

protected:
    RsDeviceSource(UsageEnvironment& t_env, frames_queue* pqs) : FramedSource(t_env), m_pqs(pqs) {};

    virtual ~RsDeviceSource() {};

protected:
    virtual void doStopGettingFrames() { FramedSource::doStopGettingFrames(); m_pqs->stop((void*)this); };

private:
    virtual void doGetNextFrame() {
        if (isCurrentlyAwaitingData()) {
            FrameData frame = m_pqs->get_frame((void*)this);
            if (frame != nullptr) {
                // we have got the data
                gettimeofday(&fPresentationTime, NULL); // If you have a more accurate time - e.g., from an encoder - then use that instead.

#if 0
                fFrameSize = m_pqs->get_size();
#else                                
                fFrameSize = ((chunk_header_t*)frame.get())->size;
#endif                
                memcpy(fTo, frame.get(), fFrameSize);
                
                afterGetting(this); // After delivering the data, inform the reader that it is now available:
            } else {
                // return here after 1ms
                nextTask() = envir().taskScheduler().scheduleDelayedTask(1000, (TaskFunc*)waitForFrame, this);
            }
        } else {
            std::cout << "Attempting to read the frame out of band" << std::endl;
        }
    };

    static void waitForFrame(RsDeviceSource* t_deviceSource) {
        //// std::cout << "RsDeviceSource::waitForFrame" << std::endl;
        t_deviceSource->doGetNextFrame();
    };


private:
    frames_queue* m_pqs;
};
