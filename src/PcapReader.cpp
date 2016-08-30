#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "PcapReader.h"

PcapReader::PcapReader() :_index{ 24 }, _pcap{ nullptr }, _header{ nullptr } {}

PcapReader::PcapReader(std::string file_name) : _index{ 24 }, _pcap{ nullptr }, _header{ nullptr } {
	if (!open(file_name)) {
		_index = 24;
		if (_pcap != nullptr) {
			delete[] _pcap;
			_pcap = nullptr;
		}
		if (_header != nullptr) {
			delete _header;
			_header = nullptr;
		}
	}
}

PcapReader::~PcapReader() {
	release();
}

bool PcapReader::open(std::string file_name) {
	std::streampos size;
	char* memblock;

	std::ifstream file(file_name, std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open())
	{
		size = file.tellg();
		memblock = new char[size];
		file.seekg(0, std::ios::beg);
		file.read(memblock, size);
		file.close();

		_pcap_size = (int)size;
		_pcap = reinterpret_cast<const unsigned char*>(memblock);
	}
	else 
		return false;
	
	return true;
}

void PcapReader::split(std::string file_name, int count) {
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	std::vector<std::ofstream> ofs(count);
	if (file.is_open()) {
		file.seekg(0, file.end);
		long long size = file.tellg();
		float size_per_count = size / (float)count;
		file.seekg(0);

		for (int i = 0; i < count; i++) 
			ofs[i].open(file_name + "_" + std::to_string(i) + ".pcap", std::ifstream::out | std::ifstream::binary);
		
		char* block;
		block = new char[24];
		file.read(block, 24);
		_pcap = reinterpret_cast<const unsigned char*>(block);		
		header();

		for (int i = 0; i < count; i++)
			ofs[i].write(block, 24);
		
		delete[] block;
		_pcap = nullptr;

		long long index = 24;
		while (index < size) {
			
			PacketHeader pheader;
			
			block = new char[16];
			file.read(block, 16);
			pheader.parseHeaderData(reinterpret_cast<const unsigned char*>(block));
			ofs[(int)(index / size_per_count)].write(block, 16);
			index += 16;
			delete[] block;

			block = new char[pheader.incl_len];
			file.read(block, pheader.incl_len);
			ofs[(int)(index / size_per_count)].write(block, pheader.incl_len);
			
			index += pheader.incl_len;
			delete[] block;
		}
	}

	file.close();
}

void PcapReader::skip(std::string file_name) {
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	std::ofstream ofs;
	if (file.is_open()) {
		file.seekg(0, file.end);
		long long size = file.tellg();
		file.seekg(0);

		ofs.open(file_name + "_skip.pcap", std::ifstream::out | std::ifstream::binary);

		char* block;
		block = new char[24];
		file.read(block, 24);
		_pcap = reinterpret_cast<const unsigned char*>(block);
		header();

		ofs.write(block, 24);

		delete[] block;
		_pcap = nullptr;

		long long index = 24;
		bool skip = false;
		while (index < size) {
			skip = false;
			PacketHeader pheader;

			block = new char[16];
			file.read(block, 16);
			pheader.parseHeaderData(reinterpret_cast<const unsigned char*>(block));
			if (pheader.incl_len < 1248) {
				skip = true;
			}
			if (!skip) ofs.write(block, 16);
			index += 16;
			delete[] block;

			block = new char[pheader.incl_len];
			file.read(block, pheader.incl_len);
			if (!skip) ofs.write(block, pheader.incl_len);

			index += pheader.incl_len;
			delete[] block;
		}
	}

	file.close();
}


PcapHeader PcapReader::header() {
	if (_header != nullptr) 
		return *_header;

	_header = new PcapHeader();

	int32_t temp32;
	uint32_t tempu32;
	uint16_t tempu16;
	for (int i = 0; i < 24; i++) {
		temp32 = _pcap[i] << 8 * ((i % 4));
		tempu32 = _pcap[i] << 8 * ((i % 4));
		tempu16 = _pcap[i] << 8 * ((i % 4));
		if (i < 4) _header->magic_number += tempu32;
		else if (i < 6) _header->version_major += tempu16;
		else if (i < 8) _header->version_minor += tempu16;
		else if (i < 12) _header->thiszone += temp32;
		else if (i < 16) _header->sigfigs += tempu32;
		else if (i < 20) _header->snaplen += tempu32;
		else if (i < 24) _header->network += tempu32;
	}

	return *_header;
}

void PcapReader::release() {
	if (_header != nullptr) {
		delete _header;
		_header = nullptr;
	}

	if (_pcap != nullptr) {
		delete[] _pcap;
		_pcap = nullptr;
	}
}

void PcapReader::reset() {
	_index = 24;
}

bool PcapReader::nextPacket(Packet& packet) {
	if (_index >= _pcap_size) return false;

	PacketHeader* packet_h = new PacketHeader();
	packet_h->parseHeaderData(_pcap + _index);
	_index += 16;

	PacketData* packet_data = new PacketData(_pcap + _index);
	packet_data->size(packet_h->incl_len);
	_index += packet_data->size();	

	packet.release();
	packet = Packet(packet_h, packet_data);
	return true;
}
