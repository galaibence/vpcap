#ifndef PACKETHEADER_H
#define PACKETHEADER_H

#include <cstdint>

class PacketHeader {
public:
	uint32_t ts_sec = 0;
	uint32_t ts_usec = 0;
	uint32_t incl_len = 0;
	uint32_t orig_len = 0;

	void parseHeaderData(const unsigned char * data);
};

#endif