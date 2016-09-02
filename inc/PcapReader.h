#ifndef PCAPREADER_H
#define PCAPREADER_H

#include <cstdint>
#include <string>

#include "PcapHeader.h"
#include "Packet.h"

class PcapReader {
private:
	int _index;
	int _pcap_size;
	unsigned const char* _pcap;
	PcapHeader* _header;

public:
	PcapReader();
	PcapReader(std::string);
	~PcapReader();

	bool open(std::string);

	void release();
	void reset();

	void split(std::string file_name, int count);
	void skip(std::string file_name);

	PcapHeader header();
	bool nextPacket(Packet& packet);
	bool previousPacket(Packet& packet);
	
	int currentIndex() { return _index; };
	void setCurrentIndex(int index) { _index = index; };
};

#endif