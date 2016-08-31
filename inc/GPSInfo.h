#ifndef GPSINFO_H
#define GPSINFO_H

enum struct NESW { N, E, S, W };

struct GPSInfo {
	unsigned int timestamp;
	float latitude; 
	NESW north_south; 
	float longitude; 
	NESW east_west; 
	float speed_knots; 
	float true_course;
	float variation; 
	NESW east_west_variation;
};

#endif