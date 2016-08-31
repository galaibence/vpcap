#ifndef GPSINFO_H
#define GPSINFO_H

struct GPSInfo {
	unsigned int timestamp;
	float latitude; 
	char north_south; 
	float longitude; 
	char east_west; 
	float speed; 
	float true_course;
	float variation; 
	char east_west_variation;
};

#endif