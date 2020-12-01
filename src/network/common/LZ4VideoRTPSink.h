#pragma once

#ifndef _VIDEO_RTP_SINK_HH
#    include "VideoRTPSink.hh"
#endif

#define LZ4_PAYLOAD_HEADER_SIZE (4)

class LZ4VideoRTPSink : public VideoRTPSink {
public:
    static LZ4VideoRTPSink* createNew(UsageEnvironment& env, Groupsock* RTPgs) { return new LZ4VideoRTPSink(env, RTPgs); }
    virtual Boolean frameCanAppearAfterPacketStart(unsigned char const* frameStart, unsigned numBytesInFrame) const { return False; };

protected:
    LZ4VideoRTPSink(UsageEnvironment& env, Groupsock* RTPgs) : VideoRTPSink(env, RTPgs, 96, 90000, "LZ4"), fFrameNum(0) {
        // Construct our "a=fmtp:" SDP line:
        // ASSERT: sampling != NULL && colorimetry != NULL
        unsigned const fmtpSDPLineMaxSize = 200; // more than enough space
        fFmtpSDPLine = new char[fmtpSDPLineMaxSize];
        sprintf(fFmtpSDPLine, "a=fmtp:%d ", rtpPayloadType());
    };

    virtual ~LZ4VideoRTPSink() {
        delete[] fFmtpSDPLine;
    };

    char const* auxSDPLine() { return fFmtpSDPLine; };

private: // redefined virtual functions:
    virtual void doSpecialFrameHandling(unsigned fragmentationOffset, unsigned char* frameStart, unsigned numBytesInFrame, struct timeval framePresentationTime, unsigned numRemainingBytes);
    virtual unsigned specialHeaderSize() const;

private:
    char* fFmtpSDPLine;
    uint8_t fFrameNum;
};
