#include "Packet.h"

Packet::Packet() :_header{ nullptr }, _data{ nullptr } {}

Packet::Packet(PacketHeader* header, PacketData* data) : _header{ header }, _data{ data } {}

Packet::Packet(Packet& packet) {
	this->_header = packet._header;
	this->_data = packet._data;
}

Packet::~Packet() {
	release();
}


bool Packet::operator=(Packet&& packet) {
	this->_header = packet._header;
	packet._header = nullptr;
	this->_data = packet._data;
	packet._data = nullptr;

	return true;
}


PacketHeader Packet::header() {
	if (_header != nullptr) return *_header;

	_header = new PacketHeader();

	return *_header;
}

PacketData Packet::data() {
	if (_data != nullptr) return *_data;

	_data = new PacketData();


	return *_data;
}

void Packet::release() {
	if (_header != nullptr) {
		delete _header;
		_header = nullptr;
	}

	if (_data != nullptr) {
		delete _data;
		_data = nullptr;
	}
}

int Packet::size() {
	return _data->size() + 16;
}