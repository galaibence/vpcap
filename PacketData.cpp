#include "PacketData.h"

PacketData::PacketData() :_data { nullptr } {}

PacketData::PacketData(const unsigned char* data) :_data{ data } {}

PacketData::~PacketData() {
	_data = nullptr;
}

int PacketData::size() {
	return _size;
}

void PacketData::size(int size) {
	_size = size;
}

const unsigned char* PacketData::header() {
	return _data;
}

const unsigned char* PacketData::payload() {
	return _data + 42;
}