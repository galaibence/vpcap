#include "PacketHeader.h"

void PacketHeader::parseHeaderData(const unsigned char* data) {

	this->ts_sec = 0;
	this->ts_usec = 0;
	this->incl_len = 0;
	this->orig_len = 0;

	uint32_t tempu32;
	for (int i = 0; i < 16; i++) {
		tempu32 = data[i] << 8 * ((i % 4));
		if (i < 4) this->ts_sec += tempu32;
		else if (i < 8) this->ts_usec += tempu32;
		else if (i < 12) this->incl_len += tempu32;
		else if (i < 16) this->orig_len += tempu32;
	}
}