#pragma once

#ifndef _VIDEO_RTP_SINK_HH
#    include "VideoRTPSink.hh"
#endif

#ifndef _RAW_VIDEO_FRAME_PARAMETERS_HH
#    include "RawVideoFrameParameters.hh"
#endif

#define ZSTD_MAGIC (0xFD2FB528)

class LZ4VideoRTPSink : public VideoRTPSink {
public:
    static LZ4VideoRTPSink* createNew(UsageEnvironment& env, Groupsock* RTPgs) { return new LZ4VideoRTPSink(env, RTPgs); }

protected:
    LZ4VideoRTPSink(UsageEnvironment& env, Groupsock* RTPgs) : VideoRTPSink(env, RTPgs, 96, 90000, "LZ4"), fLineIndex(0), fTotalLines(0), fFrameNum(0){};
    virtual ~LZ4VideoRTPSink() {};

private: // redefined virtual functions:
    virtual void doSpecialFrameHandling(unsigned fragmentationOffset, unsigned char* frameStart, unsigned numBytesInFrame, struct timeval framePresentationTime, unsigned numRemainingBytes);
    virtual unsigned specialHeaderSize() const;
    virtual Boolean frameCanAppearAfterPacketStart(unsigned char const* frameStart, unsigned numBytesInFrame) const;
    virtual unsigned computeOverflowForNewFrame(unsigned newFrameSize) const;

private:
    uint16_t fLineIndex;
    uint16_t fTotalLines;
    uint16_t fFrameNum;

    uint16_t fSize[720]; // TODO: use dynamic allocation
};
