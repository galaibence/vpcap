#ifndef PACKET_H
#define PACKET_H

#include "PacketHeader.h"
#include "PacketData.h"

class Packet {
private:
	PacketHeader* _header;
	PacketData* _data;

public:
	Packet();
	Packet(PacketHeader* header, PacketData* data);
	Packet(Packet& packet);
	~Packet();

	PacketHeader header();
	PacketData data();

	bool operator=(Packet&& packet);

	void release();
	int size();
};

#endif
