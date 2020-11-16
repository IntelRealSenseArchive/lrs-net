#include "LZ4VideoRTPSource.h"

#include <iostream>

LZ4VideoRTPSource* LZ4VideoRTPSource::createNew(UsageEnvironment& env, Groupsock* RTPgs, unsigned char rtpPayloadFormat, unsigned rtpTimestampFrequency, char const* sampling)
{
    return new LZ4VideoRTPSource(env, RTPgs, rtpPayloadFormat, rtpTimestampFrequency, sampling);
}

LZ4VideoRTPSource ::LZ4VideoRTPSource(UsageEnvironment& env, Groupsock* RTPgs, unsigned char rtpPayloadFormat, unsigned rtpTimestampFrequency, char const* sampling)
    : MultiFramedRTPSource(env, RTPgs, rtpPayloadFormat, rtpTimestampFrequency)
{
    fSampling = strDup(sampling);
}

LZ4VideoRTPSource::~LZ4VideoRTPSource()
{
    delete[] fSampling;
}

#define LZ4_PAYLOAD_HEADER_SIZE 4

Boolean LZ4VideoRTPSource ::processSpecialHeader(BufferedPacket* packet, unsigned& resultSpecialHeaderSize)
{
    unsigned char* headerStart = packet->data();
    unsigned packetSize = packet->dataSize();
    // static u_int8_t frameNum = 0;
    // static u_int32_t frameSize = 0;

    // There should be enough space for a payload header:
    if(packetSize < LZ4_PAYLOAD_HEADER_SIZE)
        return False;

    u_int32_t fragmentOffset = (headerStart[3] << 16) | (headerStart[2] << 8) | (headerStart[1]);
    fCurrentPacketBeginsFrame = fragmentOffset == 0;
    fCurrentPacketCompletesFrame = packet->rtpMarkerBit();

    // u_int8_t newFrameNum = headerStart[0];
    // if (frameNum != newFrameNum) {
    //     frameNum = newFrameNum;
    //     frameSize = 0;
    //     std::cout << "Got frame " << (uint32_t)frameNum << std::endl;
    // }

    // if (frameSize != fragmentOffset) {
    //     std::cout << "Frame " << (uint32_t)frameNum << " is broken, expected " << frameSize << ", received " << fragmentOffset << std::endl;
    //     frameSize = fragmentOffset;
    // }
    // frameSize += (packetSize - LZ4_PAYLOAD_HEADER_SIZE);

    resultSpecialHeaderSize = LZ4_PAYLOAD_HEADER_SIZE;
    return True;
}

char const* LZ4VideoRTPSource::MIMEtype() const
{
    return "video/LZ4";
}
