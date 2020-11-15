#include "LZ4VideoRTPSource.h"

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

    // There should be enough space for a payload header:
    if(packetSize < LZ4_PAYLOAD_HEADER_SIZE)
        return False;

    u_int32_t fragmentOffset = (headerStart[3] << 16) | (headerStart[2] << 8) | (headerStart[1]);
    fCurrentPacketBeginsFrame = fragmentOffset == 0;
    fCurrentPacketCompletesFrame = packet->rtpMarkerBit();

    resultSpecialHeaderSize = LZ4_PAYLOAD_HEADER_SIZE;
    return True;
}

char const* LZ4VideoRTPSource::MIMEtype() const
{
    return "video/LZ4";
}
