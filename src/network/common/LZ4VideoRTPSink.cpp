#include <iostream>

#include "LZ4VideoRTPSink.h"

void LZ4VideoRTPSink::doSpecialFrameHandling(unsigned fragmentationOffset, unsigned char* frameStart, 
                                             unsigned numBytesInFrame, struct timeval framePresentationTime, unsigned numRemainingBytes)
{
    // std::cout << "doSpecialFrameHandling processing packet for frame " << (void*)frameStart << " with size of " << numBytesInFrame << " offset " << fragmentationOffset << " and " << numRemainingBytes << " bytes reminding" << std::endl;

    if (fragmentationOffset == 0) {
        // new frame arrived
        fLineIndex = fTotalLines = 0;
        fFrameNum++;

        uint8_t* magic = frameStart + sizeof(uint16_t) * 2;
        uint8_t* prev  = magic;
        unsigned size  = 0;

        // fill the fSize array
        while (magic - frameStart < numRemainingBytes) 
        {
            magic++;
            if (*(uint32_t*)(magic) == ZSTD_MAGIC) {
                size = magic - prev;
                // std::cout << "zstd frame of size " << size << std::endl;
                fSize[fTotalLines++] = size;
                prev = magic;
            }
        }
        size = magic - prev;
        // std::cout << "zstd frame of size " << size << std::endl;
        fSize[fTotalLines++] = size;
    } else {
        // process the frame and insert the line header
        fLineIndex++;
    }

    // our special header specifies the frame and line number: [uint16_t][uint16_t]
    struct hdr {
        uint16_t frame;
        uint16_t line;
    } header;

    header.frame = fFrameNum;
    header.line  = fLineIndex;

    setSpecialHeaderBytes((uint8_t*)&header, sizeof(header));

    if(numRemainingBytes == 0)
    {
        // This packet contains the last (or only) fragment of the frame.
        // Set the RTP 'M' ('marker') bit:
        setMarkerBit();
    }

    // Also set the RTP timestamp:
    setTimestamp(framePresentationTime);
}

Boolean LZ4VideoRTPSink::frameCanAppearAfterPacketStart(unsigned char const* frameStart, unsigned numBytesInFrame) const
{
    // Only one frame per packet:
    // std::cout << "frameCanAppearAfterPacketStart processing packet for frame " << (void*)frameStart << "with size of " << numBytesInFrame << " bytes" << std::endl;
    return False;
}

unsigned LZ4VideoRTPSink::specialHeaderSize() const
{
    return sizeof(uint16_t) * 2;
}

unsigned LZ4VideoRTPSink::computeOverflowForNewFrame(unsigned newFrameSize) const
{
    unsigned overflow = 0;

    if (curFragmentationOffset() == 0) { 
        // new frame arrived, sending header. it is a hack, 
        // doSpecialFrameHandling will fill sizes array with correct overflow values
        overflow = newFrameSize - sizeof(uint16_t) * 2; 
    } else {
        overflow = newFrameSize - fSize[fLineIndex];
    }

    // std::cout << "computeOverflowForNewFrame processing packet for frame with size of " << newFrameSize << " bytes, returned " << overflow << std::endl;
    return overflow;

}
