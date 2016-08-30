#include "VelodyneStreamer.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>

#include "Packet.h"
#include "TauronTypes.h"

#define ROTCORRECTION 0
#define VERTCORRECTION 1
#define DISTCORRECTION 2
#define HORIZOFFSETCORRECTION 6
#define VERTOFFSETCORRECTION 5
#define MININTENSITY 8
#define MAXINTENSITY 7
#define FOCALDISTANCE 9
#define FOCALSLOPE 10

int laser_ids[16] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

float laser_ids32[32] = { -30.67f, -9.33f, -29.33f, -8.00f, -28.00f, -6.66f, -26.66f, -5.33f, 
                        -25.33f, -4.00f, -24.00f, -2.67f, -22.67f, -1.33f, -21.33f, 0.00f, 
                        -20.00f, 1.33f, -18.67f, 2.67f, -17.33f, 4.00f, -16.00f, 5.33f, 
                        -14.67f, 6.67f, -13.33f, 8.00f, -12.00f, 9.33f, -10.67f, 10.67f };


int laser_order32[32] = { 0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};
int laser_order64[64] = { 36,37,58,59,38,39,28,30,40,41,32,34,48,49,42,43,50,51,44,45,52,53,46,47,60,61,54,55,62,63,56,57,4,5,26,27,6,7,0,1,8,9,2,3,16,17,10,11,18,19,12,13,20,21,14,15,29,31,22,23,33,35,24,25 };

double DistLSB = 0.2;


double arr[64][11] = {
    //rotCorrection,vertCorrection,distCorrection,distCorrectionX,distCorrectionY,vertOffsetCorrection,horizOffsetCorrection,maxInt,minInt,focalDistance,focalSlope
    { -7.1559157,	-8.7686234,151.95264,155.00304,152.31381,19.548199,2.5999999,255,0,1200.0,1.4 },//1
    { -3.967427,	-8.3563347,151.45139,152.5696,154.91043,19.601112,-2.5999999,255,30,500,1 },//2
    { 4.7212691,	2.486347,149.63768,155.71011,154.56783,20.96954,2.5999999,255,40,500,0.89999998 },//3
    { 7.8958368,	2.9771359,137.71207,141.03835,143.43457,21.031273,-2.5999999,255,0,950,1.1 },//4
    { -0.64169002,	-7.7822719,129.47885,137.20456,140.25244,19.674604,2.5999999,255,60,1000,1 },//5
    { 2.623884,		-7.2642641,143.95787,148.01956,150.74649,19.740747,-2.5999999,255,20,700,1 },//6
    { -1.8131849,	-10.860783,136.18773,140.04077,139.00876,19.277746,2.5999999,255,50,500,1.1 },//7
    { 1.3411983,	-10.362929,153.25716,153.37143,154.52948,19.342419,-2.5999999,255,30,600,1 },//8
    { 5.7673278,	-6.860538,137.43323,144.74606,145.33472,19.792192,2.5999999,255,65,600,1 },//9
    { 9.1652756,	-6.2594609,149.69112,147.54176,148.88799,19.868624,-2.5999999,255,10,500,1 },//10
    { 4.7631793,	-9.8180084,143.4263,149.57901,151.91893,19.412971,2.5999999,255,30,500,0.89999998 },//11
    { 7.9426689,	-9.3397217,155.00841,154.2697,157.16737,19.474705,-2.5999999,255,10,800,1.2 },//12
    { -7.1952329,	-2.5933869,137.25992,143.53555,139.32405,20.331627,2.5999999,255,0,1100,1.1 },//13
    { -3.8482759,	-2.0555191,131.11591,134.70857,136.52054,20.399239,-2.5999999,255,35,600,0.69999999 },//14
    { -8.3530884,	-5.6685958,153.60803,155.28035,156.66801,19.943586,2.5999999,255,0,800,1.1 },//15
    { -5.0952554,	-5.1927018,147.55449,150.32428,153.26752,20.003851,-2.5999999,255,30,500,0.89999998 },//16
    { -0.74519908,	-1.587509,144.10587,148.70845,146.27965,20.458033,2.5999999,255,50,1800,0.5 },//17
    { 2.6336744,	-1.15441,144.34521,148.97508,147.91382,20.512417,-2.5999999,255,30,1100,0.80000001 },//18
    { -1.854838,	-4.7044559,132.43448,138.16666,136.47719,20.065582,2.5999999,255,20,1200,1 },//19
    { 1.3930181,	-4.1689248,145.65854,143.40645,145.50516,20.133196,-2.5999999,255,30,2000,0.69999999 },//20
    { 5.746419,		-0.568941,133.13776,138.19601,138.47791,20.585909,2.5999999,255,40,500,0.89999998 },//21
    { 9.0916338,	-0.217594,137.87535,139.78789,142.57806,20.630005,-2.5999999,255,0,1000,1.1 },//22
    { 4.6091719,	-3.714313,134.19412,140.59189,141.13303,20.190519,2.5999999,255,55,1100,1 },//23
    { 7.9456115,	-3.1891811,143.38304,142.12059,149.00565,20.256662,-2.5999999,255,0,1100,1.1 },//24
    { -7.1650081,	3.5024951,149.30911,155.72188,151.82672,21.097416,2.5999999,255,10,500,1 },//25
    { -3.904608,	4.0039558,143.67653,146.14236,145.87589,21.16062,-2.5999999,255,30,300,1 },//26
    { -8.3260279,	0.50854403,152.79892,156.86108,155.737,20.721134,2.5999999,255,0,750,1 },//27
    { -5.1018195,	0.97694802,150.77638,154.53738,154.83368,20.779928,-2.5999999,255,45,400,0.89999998 },//28
    { -0.69107699,	4.4931631,136.27797,142.32985,141.16524,21.222351,2.5999999,255,35,400,1 },//29
    { 2.5148804,	4.9700899,137.49608,140.45798,140.06631,21.282616,-2.5999999,255,40,400,1.1 },//30
    { -1.8946992,	1.445222,129.40709,139.2794,132.25565,20.838722,2.5999999,255,0,800,0.89999998 },//31
    { 1.4630644,	1.971796,139.07666,143.65166,145.01437,20.904865,-2.5999999,255,55,900,1 },//32
    { -7.6260486,	-22.727167,134.61819,136.78523,135.52881,10.812235,2.5999999,255,0,1100,1.5 },//33
    { -4.1488786,	-22.35751,124.87466,129.29747,134.38803,10.859233,-2.5999999,255,25,500,0.89999998 },//34
    { 4.7513733,	-11.520303,150.16498,153.84891,155.06186,12.148494,2.5999999,255,50,450,1 },//35
    { 8.0251369,	-10.945655,124.95708,128.91127,129.43669,12.213274,-2.5999999,255,0,1100,1.3 },//36
    { -0.68779838,	-21.885101,126.74536,132.98663,135.77469,10.918932,2.5999999,255,30,600,0.89999998 },//37
    { 2.7686412,	-21.307959,124.81956,128.627,129.235,10.991334,-2.5999999,255,0,1450,1.5 },//38
    { -2.052953,	-24.845081,145.16722,148.42308,148.68384,10.53787,2.5999999,255,30,400,1.1 },//39
    { 1.5403103,	-24.419271,138.43004,140.10727,143.32016,10.593759,-2.5999999,255,50,650,1.3 },//40
    { 6.1668367,	-20.859316,124.4738,131.31859,135.4245,11.047223,2.5999999,255,40,650,1 },//41
    { 9.621645,		-20.119377,132.84543,132.95518,138.09489,11.138678,-2.5999999,255,0,1250,1.7 },//42
    { 4.9340644,	-23.853561,140.55542,144.26663,143.0847,10.667431,2.5999999,255,90,150,1.1 },//43
    { 8.6081295,	-23.183969,135.29747,138.34918,140.29074,10.753805,-2.5999999,255,5,600,1.2 },//44
    { -7.4476075,	-16.55401,124.0738,131.71521,128.364,11.568009,2.5999999,255,0,1500,1.6 },//45
    { -4.0160227,	-16.111792,152.88052,159.04514,162.88455,11.620087,-2.5999999,255,45,1000,0.89999998 },//46
    { -8.7875204,	-19.570175,154.23979,159.19412,161.0177,11.205999,2.5999999,255,0,1300,1.7 },//47
    { -5.3164234,	-19.184532,130.86253,137.37134,141.77727,11.252997,-2.5999999,255,35,550,0.89999998 },//48
    { -0.6877985,	-15.689306,134.0533,145.12436,144.71468,11.669625,2.5999999,255,20,1350,1 },//49
    { 2.7199085,	-15.177824,129.94545,139.71344,136.849,11.729325,-2.5999999,255,25,1400,1 },//50
    { -1.9355294,	-18.723572,144.99763,151.11346,154.2395,11.308886,2.5999999,255,0,1300,1.2 },//51
    { 1.4515518,	-18.217821,135.16956,142.31837,143.01146,11.369856,-2.5999999,255,0,1300,1.1 },//52
    { 6.0057297,	-14.598064,123.74236,130.8008,135.49429,11.796646,2.5999999,255,35,500,0.89999998 },//53
    { 9.3459244,	-14.081364,137.30562,140.3678,143.47415,11.856346,-2.5999999,255,0,1300,1.4 },//54
    { 4.804235,		-17.762243,144.45888,150.25624,151.89749,11.424475,2.5999999,255,30,1000,1 },//55
    { 8.2360468,	-17.111965,138.61449,142.28815,146.20186,11.501958,-2.5999999,255,0,1500,1.6 },//56
    { -7.3005872,	-10.538668,146.35341,152.32283,150.63837,12.259002,2.5999999,255,0,1300,1.3 },//57
    { -3.926554,	-9.9943476,137.15826,141.86972,143.28435,12.319972,-2.5999999,255,55,350,1 },//58
    { -8.578989,	-13.407262,146.44832,150.95984,151.49265,11.933828,2.5999999,255,0,1100,1.4 },//59
    { -5.1414781,	-12.974276,130.74548,135.21214,138.30151,11.983367,-2.5999999,255,40,500,0.89999998 },//60
    { -0.67333692,	-9.6190624,143.78391,147.99687,148.89211,12.361888,2.5999999,255,25,500,1.1 },//61
    { 2.5721638,	-9.0717077,135.85466,141.35242,141.70488,12.422858,-2.5999999,255,50,200,0.89999998 },//62
    { -1.9503998,	-12.416959,144.78067,154.42529,155.12207,12.046877,2.5999999,255,10,1000,1 },//63
    { 1.4242532,	-12.070212,143.29738,148.17114,149.54124,12.086253,-2.5999999,255,35,900,0.80000001 },//64
};


VelodyneStreamer::VelodyneStreamer(std::string pcap) {
    _reader.open(pcap);

    Packet packet;
    while (_reader.nextPacket(packet)) {
        if (packet.size() >= 1248) {
            if ((int)packet.data().payload()[1247 - 42] == 0x22) {
                sensor = SensorType::VLP16;
                dual_distance_return = (int)packet.data().payload()[1246 - 42] == 0x39;
            }
            else if ((int)packet.data().payload()[1247 - 42] == 0x21) {
                sensor = SensorType::HDL32;
            }
            else {
                sensor = SensorType::HDL64;
            }
            break;
        }
    }

    _reader.reset();

}

VelodyneStreamer::~VelodyneStreamer() {
    _reader.release();
}

void VelodyneStreamer::parseAzimuth(const unsigned char* data, int& azimuth) {
    azimuth = ((int)(data[1] << 8) + (int)data[0]);
}

void VelodyneStreamer::parseDataBlock(const unsigned char* data, int& distance, int& intensity) {
    intensity = (int)data[2];
    distance = ((int)(data[1] << 8) + (int)data[0]) * 2;
}

void VelodyneStreamer::parseTimeStamp(const unsigned char* data, unsigned int& timestamp) {
    timestamp = (unsigned int)(data[3] << 24) + (unsigned int)(data[2] << 16) + (unsigned int)(data[1] << 8) + (unsigned int)data[0];
}

bool VelodyneStreamer::parseNMEASentance(
        const unsigned char* data, 
        int& timestamp, 
        float& latitude, int& north_south, 
        float& longitude, int& east_west, 
        float& speed_knots, 
        float& true_course,
        float& variation, int& east_west_variation) {
    
    return false;
}

int VelodyneStreamer::interpolate_azimuth(int previous_azimuth, int next_azimuth) {
    if (next_azimuth < previous_azimuth)
        next_azimuth = next_azimuth + 36000;

    float azimuth = previous_azimuth + ((next_azimuth - previous_azimuth) / 2);

    if (azimuth >= 36000) azimuth = azimuth - 36000;
    return azimuth;
}

char* prev_timestamp = nullptr;
char* coord = nullptr;
char* coord1 = nullptr;
void VelodyneStreamer::GPSPacket(Packet& packet) {
    const unsigned char *payload = packet.data().payload();

    if (coord == nullptr) {
        coord = new char[8];
        coord1 = new char[2];
    }

    payload += 198; // Unused 198 bytes
    
    unsigned int timestamp;
    parseTimeStamp(payload, timestamp);
    payload += 4; // Timestamp 4 bytes;

    payload += 4; // Unused 4 bytes;

    char *data = new char[72];
    memcpy(data, payload, 72);
    if (strncmp(data, "$GPRMC", 6) == 0) {
        if (prev_timestamp == nullptr) {
            prev_timestamp = new char[6];
        }
        if (strncmp(data + 7, prev_timestamp, 6) == 0) {}
        else {
            memcpy(prev_timestamp, data + 7, 6);
            // std::cout << atoi(&data[19]) * 10 + atoi(&data[20]) << "°";
            // for (auto i = 0; i < 8; i++) std::cout << data[21 + i];
            // std::cout << "' N ";
            // std::cout << atoi(&data[33]) * 10 + atoi(&data[34]) << "°";
            // for (auto i = 0; i < 8; i++) std::cout << data[35 + i];
            // std::cout << "' E";
            // std::cout << std::endl;

            memcpy(coord1, &data[33], 2);
            memcpy(coord, &data[35], 8);
            memcpy(coord1, &data[19], 2);
            memcpy(coord, &data[21], 8);
        }
    }
    else {	}
    payload += 72; // NMEA $GPRMC sentence 72 bytes

    payload += 234; // Unused 234 bytes
}

bool VelodyneStreamer::nextFrame(VCloud& cloud) {
    switch (sensor) {
    case SensorType::VLP16: { 
        if (dual_distance_return) return nextFrameVLP16DD(cloud);
        else return nextFrameVLP16(cloud); 
        break; 
    }
    case SensorType::HDL32: { 
        return nextFrameHDL32(cloud); 
        break; 
    }
    case SensorType::HDL64: { 
        return nextFrameHDL64(cloud); 
        break; 
    }
    default: 
        throw "Not a valid sensor type!";
    }

    return false;
}

bool VelodyneStreamer::nextFrameVLP16(VCloud& cloud) {
    cloud.clear();

    Packet packet;
    VPoint p;

    bool first = true;
    float previous_azimuth = -1.0f;
    float PIper180 = M_PI / 180.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) {
            if (packet.header().incl_len == 554) {
                GPSPacket(packet);
                continue;
            }
            else continue;
        }

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        unsigned int timestamp;
        parseTimeStamp(payload + 1200, timestamp);

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        p.intensity = r;
                        p.elevation = laser_ids[i];
                        p.laser_id = (int)(laser_ids[i] + 15) / 2;
                        p.azimuth = azimuth[k] >= 36000 ? 0 : azimuth[k];
                        p.distance = distance * 0.001f;
                        p.distanceINT = distance;
                        p.timestamp = timestamp;

                        p.x = distance * 0.001f * cosf(laser_ids[i] * PIper180) * sinf(azimuth[k] * 0.01f * PIper180);
                        p.y = distance * 0.001f * cosf(laser_ids[i] * PIper180) * cosf(azimuth[k] * 0.01f * PIper180);
                        p.z = distance * 0.001f * sinf(laser_ids[i] * PIper180);

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}

bool VelodyneStreamer::nextFrameVLP16DD(VCloud& cloud) {

    cloud.clear();

    Packet packet;
    VPoint p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        p.intensity = r;
                        p.elevation = laser_ids[i];
                        p.azimuth = azimuth[k];
                        p.distance = distance;

                        p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}

bool VelodyneStreamer::nextFrameHDL32(VCloud& cloud) {
    cloud.clear();

    Packet packet;
    VPoint p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
              parseDataBlock(payload, distance, r);
              payload = payload + 3; // DATABLOCK

              if (distance > 0) {
                p.intensity = r;
                p.elevation = laser_ids32[i];
                p.azimuth = azimuth >= 36000 ? 0 : azimuth;
                p.distance = distance * 0.001f;
                p.laser_id = laser_order32[i];
                p.valid = true;

                p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth / 100.0f * M_PI / 180.0f);
                p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth / 100.0f * M_PI / 180.0f);
                p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                cloud.push_back(p);
              }
        else {
          p.intensity = 0;
          p.elevation = laser_ids32[i];
          p.azimuth = azimuth >= 36000 ? 0 : azimuth;
          p.distance = distance * 0.001f;
          p.laser_id = laser_order32[i];
          p.valid = false;

          p.x = 0;
          p.y = 0;
          p.z = 0;

          cloud.push_back(p);
        }
            }
        }
    }

    return !first;
}

bool VelodyneStreamer::nextFrameHDL64(VCloud& cloud) {
    cloud.clear();

    Packet packet;
    VPoint p;

    bool first = true;
    float previous_azimuth = -1.0f;
    float PIper180 = M_PI / 180.0f;
    float cosVertAngle, sinVertAngle, RotCorrection, horizontalangle, sinRotAngle, cosRotAngle, hOffsetCorr, vOffsetCorr, xyDistance;
    int minIntensity, maxIntensity;
    float intensityScale, focaloffset, focalslope, intensityVal1, intensityColor;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        unsigned int timestamp;
        parseTimeStamp(payload + 1200, timestamp);

        int azimuth;
        int distance = 0.0f;

        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((int)(payload[1] << 8) + (int)payload[0]) == 0xEEFF;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {
                    p.azimuth = azimuth;
                    p.distanceINT = distance;
                    p.elevation = arr[i + !upper * 32][VERTCORRECTION];
                    p.laser_id = laser_order64[i + !upper * 32];
                    p.timestamp = timestamp;

                    distance = 0.1 * distance + arr[i + !upper * 32][DISTCORRECTION];

                    cosVertAngle = cosf(arr[i + !upper * 32][VERTCORRECTION] * PIper180);
                    sinVertAngle = sinf(arr[i + !upper * 32][VERTCORRECTION] * PIper180);

                    RotCorrection = arr[i + !upper * 32][ROTCORRECTION] * PIper180;

                    horizontalangle = azimuth * 0.01f * PIper180;
                    sinRotAngle = sinf(horizontalangle - RotCorrection);
                    cosRotAngle = cosf(horizontalangle - RotCorrection);

                    hOffsetCorr = arr[i + !upper * 32][HORIZOFFSETCORRECTION];
                    vOffsetCorr = arr[i + !upper * 32][VERTOFFSETCORRECTION];

                    xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

                    p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle) * 0.01f;
                    p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle) * 0.01f;
                    p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle) * 0.01f;

                    p.distance = distance * 0.01f;
                    p.intensity = r;

                    if (r != 0) {
                    	minIntensity = arr[i + !upper * 32][MININTENSITY];
                    	maxIntensity = arr[i + !upper * 32][MAXINTENSITY];

                    	intensityScale = (maxIntensity - minIntensity);

                    	distance *= 20000;

                    	focaloffset = 256 * (1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100)*(1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100);

                    	focalslope = arr[i + !upper * 32][FOCALSLOPE];

                    	intensityVal1 = r + focalslope*(abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
                    	if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
                    	if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;

                    	intensityColor = (intensityVal1 - minIntensity) / intensityScale;

                    	p.intensity = intensityColor;
                    }

                    cloud.push_back(p);
                }
            }
        }
    }

    return !first;
}

bool VelodyneStreamer::nextFrame(pcl::PointCloud<pcl::PointXYZI>& cloud) {
    switch (sensor) {
    case SensorType::VLP16: {
        if (dual_distance_return) return nextFrameVLP16DD(cloud);
        else return nextFrameVLP16(cloud);
        break;
    }
    case SensorType::HDL32: {
        return nextFrameHDL32(cloud);
        break;
    }
    case SensorType::HDL64: {
        return nextFrameHDL64(cloud);
        break;
    }
    default:
        throw "Not a valid sensor type!";
    }

    return false;
}
bool VelodyneStreamer::nextFrame(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    switch (sensor) {
    case SensorType::VLP16: {
        if (dual_distance_return) return nextFrameVLP16DD(cloud);
        else return nextFrameVLP16(cloud);
        break;
    }
    case SensorType::HDL32: {
        return nextFrameHDL32(cloud);
        break;
    }
    case SensorType::HDL64: {
        return nextFrameHDL64(cloud);
        break;
    }
    default:
        throw "Not a valid sensor type!";
    }

    return false;
}
bool VelodyneStreamer::nextFrame(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, std::vector<TData>& data) {
  switch (sensor) {
  case SensorType::VLP16: {
    if (dual_distance_return) return nextFrameVLP16DD(cloud, data);
    else return nextFrameVLP16(cloud, data);
    break;
  }
  case SensorType::HDL32: {
    return nextFrameHDL32(cloud, data);
    break;
  }
  case SensorType::HDL64: {
    return nextFrameHDL64(cloud, data);
    break;
  }
  default:
    throw "Not a valid sensor type!";
  }

  return false;
}

bool VelodyneStreamer::nextFrameVLP16(pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZI p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        p.intensity = r;

                        p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameVLP16(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZRGBNormal p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        r = r * 3;
                        p.r = r > 255 ? 255 : 0;
                        p.g = r > 255 ? ((256 * 2) - (r - 256)) / 2.0f : (255 - (255 - r));
                        p.b = r > 255 ? 0 : 255;

                        p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameVLP16(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, std::vector<TData>& data) {
  cloud.clear();
  data.clear();

  Packet packet;
  pcl::PointXYZRGBNormal p;

  bool first = true;
  float previous_azimuth = -1.0f;
  while (_reader.nextPacket(packet)) {
    if (packet.header().incl_len < 1248) continue;

    const unsigned char *payload = packet.data().payload();

    int azimuth[3];
    int average_azimuth_difference = 0.0f;
    int distance = 0.0f;

    int r;
    for (int n = 0; n < 12; n++) {
      payload = payload + 2; // 0xFFEE

      parseAzimuth(payload, azimuth[0]);
      if (first) {
        previous_azimuth = azimuth[0];
        first = false;
      }
      else {
        if (azimuth[0] < previous_azimuth) {
          return true;
        }
      }
      previous_azimuth = azimuth[0];
      if (n < 11) {
        parseAzimuth(payload + 100, azimuth[2]);
        azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
        average_azimuth_difference += azimuth[1] - azimuth[0];
      }
      else {
        azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
      }
      payload = payload + 2; // AZIMUTH

      for (int k = 0; k < 2; k++) {
        for (int i = 0; i < 16; i++) {
          parseDataBlock(payload, distance, r);
          payload = payload + 3; // DATABLOCK

          if (distance > 0) {
            TData tdata;
            tdata.intensity = r;
            tdata.azimuth = azimuth[k] * 0.01f;
            tdata.distance = distance * 0.01f;
            data.push_back(tdata);

            r = r * 2;
            p.r = r > 255 ? r - 256 : 0;
            p.g = r > 255 ? 255 - (r - 256) : r;
            p.b = r > 255 ? 0 : 255 - r;

            p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
            p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
            p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

            cloud.push_back(p);
          }
        }
      }
    }
  }

  return !first;
}

bool VelodyneStreamer::nextFrameVLP16DD(pcl::PointCloud<pcl::PointXYZI>& cloud) {

    cloud.clear();

    Packet packet;
    pcl::PointXYZI p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        p.intensity = r;

                        p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameVLP16DD(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZRGBNormal p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth[3];
        int average_azimuth_difference = 0.0f;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth[0]);
            if (first) {
                previous_azimuth = azimuth[0];
                first = false;
            }
            else {
                if (azimuth[0] < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth[0];
            if (n < 11) {
                parseAzimuth(payload + 100, azimuth[2]);
                azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
                average_azimuth_difference += azimuth[1] - azimuth[0];
            }
            else {
                azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
            }
            payload = payload + 2; // AZIMUTH

            for (int k = 0; k < 2; k++) {
                for (int i = 0; i < 16; i++) {
                    parseDataBlock(payload, distance, r);
                    payload = payload + 3; // DATABLOCK

                    if (distance > 0) {
                        p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
                        p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                        r = r * 3;
                        p.r = r > 255 ? 255 : 0;
                        p.g = r > 255 ? ((256 * 2) - (r - 256)) / 2.0f : (255 - (255 - r));
                        p.b = r > 255 ? 0 : 255;

                        cloud.push_back(p);
                    }
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameVLP16DD(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, std::vector<TData>& data) {
  cloud.clear();

  Packet packet;
  pcl::PointXYZRGBNormal p;

  bool first = true;
  float previous_azimuth = -1.0f;
  while (_reader.nextPacket(packet)) {
    if (packet.header().incl_len < 1248) continue;

    const unsigned char *payload = packet.data().payload();

    int azimuth[3];
    int average_azimuth_difference = 0.0f;
    int distance = 0.0f;

    int r;
    for (int n = 0; n < 12; n++) {
      payload = payload + 2; // 0xFFEE

      parseAzimuth(payload, azimuth[0]);
      if (first) {
        previous_azimuth = azimuth[0];
        first = false;
      }
      else {
        if (azimuth[0] < previous_azimuth) {
          return true;
        }
      }
      previous_azimuth = azimuth[0];
      if (n < 11) {
        parseAzimuth(payload + 100, azimuth[2]);
        azimuth[1] = interpolate_azimuth(azimuth[0], azimuth[2]);
        average_azimuth_difference += azimuth[1] - azimuth[0];
      }
      else {
        azimuth[1] = azimuth[0] + average_azimuth_difference / 11;
      }
      payload = payload + 2; // AZIMUTH

      for (int k = 0; k < 2; k++) {
        for (int i = 0; i < 16; i++) {
          parseDataBlock(payload, distance, r);
          payload = payload + 3; // DATABLOCK

          if (distance > 0) {
            p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth[k] / 100.0f * M_PI / 180.0f);
            p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth[k] / 100.0f * M_PI / 180.0f);
            p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

            r = r * 3;
            p.r = r > 255 ? 255 : 0;
            p.g = r > 255 ? ((256 * 2) - (r - 256)) / 2.0f : (255 - (255 - r));
            p.b = r > 255 ? 0 : 255;

            cloud.push_back(p);
          }
        }
      }
    }
  }

  return !first;
}

bool VelodyneStreamer::nextFrameHDL32(pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZI p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {
                    p.intensity = r;

                    p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth / 100.0f * M_PI / 180.0f);
                    p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth / 100.0f * M_PI / 180.0f);
                    p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                    cloud.push_back(p);
                }
        else {

        }
            }
        }
    }

    return !first;
}

bool VelodyneStreamer::nextFrameHDL32(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZRGBNormal p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {
                    r = r * 2;
                    p.r = r > 255 ? r - 256 : 0;
                    p.g = r > 255 ? 255 - (r - 256) : r;
                    p.b = r > 255 ? 0 : 255 - r;

                    p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth / 100.0f * M_PI / 180.0f);
                    p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth / 100.0f * M_PI / 180.0f);
                    p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                    cloud.push_back(p);
                }
        else {

        }
            }
        }
    }

    return !first;
}


bool VelodyneStreamer::nextFrameHDL32(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, std::vector<TData>& data) {
    cloud.clear();
        data.clear();

    Packet packet;
    pcl::PointXYZRGBNormal p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;

        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {
                                        TData tdata;
                                        tdata.intensity = r;
                                        tdata.azimuth = azimuth * 0.01f;
                                        tdata.distance = distance * 0.01f;
                                        data.push_back(tdata);

                    r = r * 2;
                    p.r = r > 255 ? r - 256 : 0;
                    p.g = r > 255 ? 255 - (r - 256) : r;
                    p.b = r > 255 ? 0 : 255 - r;
                                        
                    p.x = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * sinf(azimuth / 100.0f * M_PI / 180.0f);
                    p.y = distance / 1000.0f * cosf(laser_ids[i] * M_PI / 180.0f) * cosf(azimuth / 100.0f * M_PI / 180.0f);
                    p.z = distance / 1000.0f * sinf(laser_ids[i] * M_PI / 180.0f);

                    cloud.push_back(p);
                }
        else {

        }
            }
        }
    }

    return !first;
}


bool VelodyneStreamer::nextFrameHDL64(pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZI p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;
        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((int)(payload[1] << 8) + (int)payload[0]) == 0xEEFF;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {

                    distance = 0.1 * distance + arr[i + !upper * 32][DISTCORRECTION];

                    float cosVertAngle = cosf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);
                    float sinVertAngle = sinf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);

                    float RotCorrection = arr[i + !upper * 32][ROTCORRECTION] * M_PI / 180.0f;

                    float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                    float sinRotAngle = sinf(horizontalangle - RotCorrection);
                    float cosRotAngle = cosf(horizontalangle - RotCorrection);

                    float hOffsetCorr = arr[i + !upper * 32][HORIZOFFSETCORRECTION];
                    float vOffsetCorr = arr[i + !upper * 32][VERTOFFSETCORRECTION];

                    float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

                    p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle);
                    p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle);
                    p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle);

                    p.intensity = r;

                    if (r != 0) {
                        int minIntensity = 0;
                        int maxIntensity = 0;
                        float intensityScale = 0;
                        minIntensity = arr[i + !upper * 32][MININTENSITY];
                        maxIntensity = arr[i + !upper * 32][MAXINTENSITY];
                        //Get intensity scale
                        intensityScale = (maxIntensity - minIntensity);
                        // Get firing \93i\94  intensity
                        // Get firing \93i\94 distance, here unit is 2mm
                        distance *= 20000;
                        // Calculate offset according calibration
                        float focaloffset = 256 * (1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100)*(1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100);
                        // get slope from calibration
                        float focalslope = arr[i + !upper * 32][FOCALSLOPE];

                        // Calculate corrected intensity vs distance
                        float intensityVal1 = r + focalslope*(abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
                        if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
                        if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;
                        // Scale to new intensity scale
                        float intensityColor = (intensityVal1 - minIntensity) / intensityScale;

                        //(*(*Intensities)++) = intensityColor;
                        p.intensity = intensityColor;
                    }


                    cloud.push_back(p);
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameHDL64(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
    cloud.clear();

    Packet packet;
    pcl::PointXYZRGBNormal p;

    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;
        const unsigned char *payload = packet.data().payload();

        int azimuth;
        int distance = 0.0f;

        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((int)(payload[1] << 8) + (int)payload[0]) == 0xEEFF;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    return true;
                }
            }
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                if (distance > 0) {

                    distance = 0.1 * distance + arr[i + !upper * 32][DISTCORRECTION];

                    float cosVertAngle = cosf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);
                    float sinVertAngle = sinf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);

                    float RotCorrection = arr[i + !upper * 32][ROTCORRECTION] * M_PI / 180.0f;

                    float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                    float sinRotAngle = sinf(horizontalangle - RotCorrection);
                    float cosRotAngle = cosf(horizontalangle - RotCorrection);

                    float hOffsetCorr = arr[i + !upper * 32][HORIZOFFSETCORRECTION];
                    float vOffsetCorr = arr[i + !upper * 32][VERTOFFSETCORRECTION];

                    float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

                    p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle) / 100;
                    p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle) / 100;
                    p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle) / 100;

                    //if (r != 0) {
                    //	int minIntensity = 0;
                    //	int maxIntensity = 0;
                    //	float intensityScale = 0;
                    //	minIntensity = arr[i + !upper * 32][MININTENSITY];
                    //	maxIntensity = arr[i + !upper * 32][MAXINTENSITY];
                    //	intensityScale = (maxIntensity - minIntensity);
                    //	distance *= 20000;
                    //	float focaloffset = 256 * (1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100)*(1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100);
                    //	float focalslope = arr[i + !upper * 32][FOCALSLOPE];

                    //	float intensityVal1 = r + focalslope*(abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
                    //	if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
                    //	if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;
                    //	float intensityColor = (intensityVal1 - minIntensity) / intensityScale;

                    //	r = intensityColor;
                    //}

                    r = r * 2;
                    p.r = r > 255 ? r - 256 : 0;
                    p.g = r > 255 ? 255 - (r - 256) : r;
                    p.b = r > 255 ? 0 : 255 - r;

                    cloud.push_back(p);
                }
            }
        }
    }

    return !first;
}
bool VelodyneStreamer::nextFrameHDL64(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, std::vector<TData>& data) {
  cloud.clear();
  data.clear();

  Packet packet;
  pcl::PointXYZRGBNormal p;

  bool first = true;
  float previous_azimuth = -1.0f;
  while (_reader.nextPacket(packet)) {
    if (packet.header().incl_len < 1248) continue;
    const unsigned char *payload = packet.data().payload();

    int azimuth;
    int distance = 0.0f;

    int r;
    bool upper;

    for (int n = 0; n < 12; n++) {
      upper = ((int)(payload[1] << 8) + (int)payload[0]) == 0xEEFF;
      payload = payload + 2; // UPPER-LOWER LASER BLOCK

      parseAzimuth(payload, azimuth);
      if (first) {
        previous_azimuth = azimuth;
        first = false;
      }
      else {
        if (azimuth < previous_azimuth) {
          return true;
        }
      }
      payload = payload + 2; // AZIMUTH

      for (int i = 0; i < 32; i++) {
        parseDataBlock(payload, distance, r);
        payload = payload + 3; // DATABLOCK

        if (distance > 0) {

          distance = 0.1 * distance + arr[i + !upper * 32][DISTCORRECTION];

          float cosVertAngle = cosf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);
          float sinVertAngle = sinf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);

          float RotCorrection = arr[i + !upper * 32][ROTCORRECTION] * M_PI / 180.0f;

          float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
          float sinRotAngle = sinf(horizontalangle - RotCorrection);
          float cosRotAngle = cosf(horizontalangle - RotCorrection);

          float hOffsetCorr = arr[i + !upper * 32][HORIZOFFSETCORRECTION];
          float vOffsetCorr = arr[i + !upper * 32][VERTOFFSETCORRECTION];

          float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

          p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle) / 100;
          p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle) / 100;
          p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle) / 100;

          if (r != 0) {
            int minIntensity = 0;
            int maxIntensity = 0;
            float intensityScale = 0;
            minIntensity = arr[i + !upper * 32][MININTENSITY];
            maxIntensity = arr[i + !upper * 32][MAXINTENSITY];
            intensityScale = (maxIntensity - minIntensity);
            distance *= 20000;
            float focaloffset = 256 * (1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100)*(1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100);
            float focalslope = arr[i + !upper * 32][FOCALSLOPE];

            float intensityVal1 = r + focalslope*(abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
            if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
            if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;
            float intensityColor = (intensityVal1 - minIntensity) / intensityScale;

            r = intensityColor;
          }

          TData tdata;
          tdata.intensity = r;
          tdata.azimuth = azimuth * 0.01f;
          tdata.distance = distance * 0.01f;
          data.push_back(tdata);

          r = r * 2;
          p.r = r > 255 ? r - 256 : 0;
          p.g = r > 255 ? 255 - (r - 256) : r;
          p.b = r > 255 ? 0 : 255 - r;

          cloud.push_back(p);
        }
        else {
        }
      }
    }
  }

  return !first;
}


bool VelodyneStreamer::nextFrameParallel(VCloud& cloud) {
  return true;
}

bool VelodyneStreamer::open(std::string pcap) {
    _reader.release();
    if (!_reader.open(pcap)) return false;
    
    Packet packet;
    while (_reader.nextPacket(packet)) {
        if (packet.size() >= 1248) {
            if ((int)packet.data().payload()[1247 - 42] == 0x22) {
                sensor = SensorType::VLP16;
                dual_distance_return = (int)packet.data().payload()[1247 - 42] == 0x39;
            }
            else if ((int)packet.data().payload()[1247 - 42] == 0x21) {
                sensor = SensorType::HDL32;
            }
            else {
                sensor = SensorType::HDL64;
            }
            break;
        }
    }

    _reader.reset();

    return true;
}

bool VelodyneStreamer::nextFrameInOrder(std::vector<std::vector<VPoint>>& pointlist) {
  pointlist.clear();
  pointlist.resize(0);

  Packet packet;
  VPoint p;

  bool first = true;
  float previous_azimuth = -1.0f;
  while (_reader.nextPacket(packet)) {
    if (packet.header().incl_len < 1248) continue;

    const unsigned char *payload = packet.data().payload();

    int azimuth;
    int distance = 0.0f;

    int r;
    for (int n = 0; n < 12; n++) {
      payload = payload + 2; // 0xFFEE

      parseAzimuth(payload, azimuth);
      if (first) {
        previous_azimuth = azimuth;
        first = false;
      }
      else {
        if (azimuth < previous_azimuth) {
          return true;
        }
      }
      previous_azimuth = azimuth;
      payload = payload + 2; // AZIMUTH


      std::vector<VPoint> points;
      points.reserve(32);
      points.resize(32);
      for (int i = 0; i < 32; i++) {
        parseDataBlock(payload, distance, r);
        payload = payload + 3; // DATABLOCK

        if (distance > 0) {
          p.intensity = r;
          p.elevation = laser_ids32[i];
          p.azimuth = azimuth >= 36000 ? 0 : azimuth;
          p.distance = distance * 0.001f;
          p.laser_id = laser_order32[i];
          p.valid = true;

          p.x = distance / 1000.0f * cosf(laser_ids32[i] * M_PI / 180.0f) * sinf(azimuth / 100.0f * M_PI / 180.0f);
          p.y = distance / 1000.0f * cosf(laser_ids32[i] * M_PI / 180.0f) * cosf(azimuth / 100.0f * M_PI / 180.0f);
          p.z = distance / 1000.0f * sinf(laser_ids32[i] * M_PI / 180.0f);

          points[laser_order32[i]] = p;
        }
        else {
          p.intensity = 0;
          p.elevation = laser_ids32[i];
          p.azimuth = azimuth >= 36000 ? 0 : azimuth;
          p.distance = 0;
          p.laser_id = laser_order32[i];
          p.valid = false;

          p.x = 0;
          p.y = 0;
          p.z = 0;

          points[laser_order32[i]] = p;
        }
      }

      pointlist.push_back(points);
    }
  }

  return !first;
}


bool VelodyneStreamer::nextFrameInOrder64(std::vector<std::vector<VPoint>>& pointlist) {
  pointlist.clear();
  pointlist.resize(0);

  Packet packet;
  VPoint p;

  bool first = true;
  float previous_azimuth = -1.0f;
  while (_reader.nextPacket(packet)) {
    if (packet.header().incl_len < 1248) continue;
    const unsigned char *payload = packet.data().payload();

    int azimuth;
    int distance = 0.0f;

    int r;
    bool upper;

    std::vector<VPoint> points;
    for (int n = 0; n < 12; n++) {
      if (n % 2 == 0) {
        points.clear();
        points.resize(64);
      }
      upper = ((int)(payload[1] << 8) + (int)payload[0]) == 0xEEFF;
      payload = payload + 2; // UPPER-LOWER LASER BLOCK

      parseAzimuth(payload, azimuth);
      if (first) {
        previous_azimuth = azimuth;
        first = false;
      }
      else {
        if (azimuth < previous_azimuth) {
          return true;
        }
      }
      payload = payload + 2; // AZIMUTH

      for (int i = 0; i < 32; i++) {
        parseDataBlock(payload, distance, r);
        payload = payload + 3; // DATABLOCK

        if (distance > 0) {
          p.azimuth = azimuth;
          p.distanceINT = distance;
          p.elevation = arr[i + !upper * 32][VERTCORRECTION];
          p.laser_id = laser_order64[i + !upper * 32];

          distance = 0.1 * distance + arr[i + !upper * 32][DISTCORRECTION];

          float cosVertAngle = cosf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);
          float sinVertAngle = sinf(arr[i + !upper * 32][VERTCORRECTION] * M_PI / 180.0f);

          float RotCorrection = arr[i + !upper * 32][ROTCORRECTION] * M_PI / 180.0f;

          float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
          float sinRotAngle = sinf(horizontalangle - RotCorrection);
          float cosRotAngle = cosf(horizontalangle - RotCorrection);

          float hOffsetCorr = arr[i + !upper * 32][HORIZOFFSETCORRECTION];
          float vOffsetCorr = arr[i + !upper * 32][VERTOFFSETCORRECTION];

          float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

          p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle) / 100;
          p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle) / 100;
          p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle) / 100;

          p.distance = distance / 100;
          p.intensity = r;

          if (r != 0) {
            int minIntensity = 0;
            int maxIntensity = 0;
            float intensityScale = 0;
            minIntensity = arr[i + !upper * 32][MININTENSITY];
            maxIntensity = arr[i + !upper * 32][MAXINTENSITY];
            intensityScale = (maxIntensity - minIntensity);
            distance *= 20000;
            float focaloffset = 256 * (1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100)*(1 - arr[i + !upper * 32][FOCALDISTANCE] / 13100);
            float focalslope = arr[i + !upper * 32][FOCALSLOPE];

            float intensityVal1 = r + focalslope*(abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
            if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
            if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;
            float intensityColor = (intensityVal1 - minIntensity) / intensityScale;

            p.intensity = intensityColor;
          }

          p.valid = true;
          points[laser_order64[i + !upper * 32]] = p;
        }
        else {
          p.intensity = 0;
          p.elevation = laser_ids32[i];
          p.azimuth = azimuth >= 36000 ? 0 : azimuth;
          p.distance = 0;
          p.laser_id = laser_order32[i];
          p.valid = false;

          p.x = 0;
          p.y = 0;
          p.z = 0;

          p.valid = false;
          points[laser_order64[i + !upper * 32]] = p;
        }
      }
      if (n % 2 == 1) {
        pointlist.push_back(points);
      }
    }
  }

  return !first;
}