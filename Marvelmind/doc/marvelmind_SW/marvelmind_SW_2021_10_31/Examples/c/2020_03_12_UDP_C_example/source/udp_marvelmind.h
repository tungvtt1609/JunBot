#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifndef WIN32
#include <pthread.h>
#endif

#define DATA_INPUT_SEMAPHORE "/udp_data_input_semaphore"

#define REQUEST_MODE_POS 1
#define REQUEST_MODE_IMU_RAW 2
#define REQUEST_MODE_IMU_FUSION 4

typedef union {uint8_t b[2]; uint16_t w;int16_t wi;} uni_8x2_16;
typedef union {uint8_t b[4];float f;uint32_t dw;int32_t dwi;} uni_8x4_32;

struct PositionValue
{
    uint8_t address;
    uint32_t timestamp;
    int32_t x, y, z;

    bool ready;
};

typedef struct {
    uint8_t address;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t compass_x;
	int16_t compass_y;
	int16_t compass_z;

	uint32_t timestamp;

	bool updated;
} IMURawData;

typedef struct {
    uint8_t address;

	int32_t pos_x;
	int32_t pos_y;
	int32_t pos_z;

	int16_t quaternion_w;
	int16_t quaternion_x;
	int16_t quaternion_y;
	int16_t quaternion_z;

	int16_t velocity_x;
	int16_t velocity_y;
	int16_t velocity_z;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;

	uint32_t timestamp;

	bool updated;
} IMUFusionData;

struct MarvelmindUDP
{
// Server URL address
// default: "127.0.0.1"
    const char *serverAddress;

// Server UDP port
// default: 49200
    uint16_t serverPort;

// Address of hedgehog to request (0 = read streaming data for all mobile beacons)
    uint8_t beaconRequestAddress;

//  Rate of requests send
    uint8_t requestRateHz;

// maximum count of measurements of coordinates stored in buffer
// default: 3
    uint8_t maxBufferedPositions;

// buffer of measurements
    struct PositionValue * positionBuffer;

    IMURawData imuRawData;
    IMUFusionData imuFusionData;

// verbose flag which activate console output
//		default: False
    bool verbose;

//	pause flag. If True, class would not read data
    bool pause;

//  If True, thread would exit from main loop and stop
    bool terminationRequired;

//  receiveDataCallback is callback function to recieve data
    void (*receiveDataCallback)(struct PositionValue position);
    void (*anyInputPacketCallback)();

// private variables
    uint8_t lastValuesCount_;
    bool haveNewValues_;
    uint8_t lastValues_next;

    uint8_t request_mode;
#ifdef WIN32
    HANDLE thread_;
    CRITICAL_SECTION lock_;
#else
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif
};

struct MarvelmindUDP * createMarvelmindUDP ();
void destroyMarvelmindUDP (struct MarvelmindUDP * udp);
void startMarvelmindUDP (struct MarvelmindUDP * udp);
void printPositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                       bool onlyNew);
void printIMUFromMarvelmindUDP(struct MarvelmindUDP * udp);
bool getPositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                   uint8_t address,
                                   struct PositionValue * position);
void stopMarvelmindUDP (struct MarvelmindUDP * udp);

#define DEFAULT_UDP_SERVER_ADDRESS "127.0.0.1"
#define DEFAULT_UDP_SERVER_PORT 49100

