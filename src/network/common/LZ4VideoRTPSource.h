#pragma once

#ifndef _MULTI_FRAMED_RTP_SOURCE_HH
#include "MultiFramedRTPSource.hh"
#endif

class LZ4VideoRTPSource: public MultiFramedRTPSource {
public:
  static LZ4VideoRTPSource* createNew(UsageEnvironment& env, Groupsock* RTPgs,
					   unsigned char rtpPayloadFormat,
					   unsigned rtpTimestampFrequency,
					   char const* sampling);

protected:
  virtual ~LZ4VideoRTPSource();

protected:
  LZ4VideoRTPSource(UsageEnvironment& env, Groupsock* RTPgs,
			 unsigned char rtpPayloadFormat,
			 unsigned rtpTimestampFrequency,
			 char const* sampling);
    // called only by createNew()

private:
  // redefined virtual functions:
  virtual Boolean processSpecialHeader(BufferedPacket* packet,
                                       unsigned& resultSpecialHeaderSize);
  virtual char const* MIMEtype() const;

private:
  char* fSampling;
};
