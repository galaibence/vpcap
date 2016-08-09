#ifndef VPOINT_H
#define VPOINT_H

#define _USE_MATH_DEFINES
#include <math.h>
//#define PI 3.14159265359f

struct VPoint {
	float x, y, z; // Point coordinates in metres

        float elevation; // Elevation angle
        int azimuth; // Azimuth angle [0..36000] (360� - 0.01� resolution)
        float distance; // Measured distance [0..) in meters
	int intensity; // Point intensity [0..255]
        bool valid;
	int distanceINT; // Measured distance [0..) in milimeters
	int laser_id; // ID of laser listing from lowest angle to highest
        

	VPoint(float elevation = 0.0f, int _azimuth = 0.0f, float _distance = 0.0f, int _intensity = 0)
		:elevation{ elevation }, azimuth{ _azimuth }, distance{ _distance }, intensity{ _intensity }, valid{ false }
	{
		x = distance / 1000.0f * sinf(azimuth / 100.0f * M_PI / 180.0f);
		y = distance / 1000.0f * cosf(azimuth / 100.0f * M_PI / 180.0f);
		z = distance / 1000.0f * sinf(elevation * M_PI / 180.0f);
	};
};

#endif