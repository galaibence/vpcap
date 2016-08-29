#ifndef PACKETDATA_H
#define PACKETDATA_H

class PacketData {
private:
	const unsigned char * _data;
	int _size;

public:
	PacketData();
	PacketData(const unsigned char* data);
	~PacketData();

	int size();
	void size(int size);

	const unsigned char* header();
	const unsigned char* payload();
};

#endif