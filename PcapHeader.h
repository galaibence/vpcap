#ifndef PCAPHEADER_H
#define PCAPHEADER_H

#include <cstdint>

class PcapHeader {
public:
	uint32_t magic_number;
	uint16_t version_major;
	uint16_t version_minor;
	int32_t thiszone;
	uint32_t sigfigs;
	uint32_t snaplen;
	uint32_t network;
};

#endif