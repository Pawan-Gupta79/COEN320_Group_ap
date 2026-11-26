#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <cstddef>
#include <pthread.h>

#define RADAR_SHM_NAME        "/group_ap_radar_shm"
#define RADAR_CHANNEL_NAME    "group_ap"
#define PLANE_CHANNEL_PREFIX  "group_ap_plane_"
#define COMPUTER_CHANNEL_NAME "group_ap_computer"
#define COMMS_CHANNEL_NAME    "group_ap_comms"
#define DISPLAY_CHANNEL_NAME  "group_ap_display"

#define AIRSPACE_X_MAX 100000
#define AIRSPACE_Y_MAX 100000
#define AIRSPACE_Z_MIN 15000
#define AIRSPACE_Z_MAX 40000

#define MAX_PLANES 100

enum class MessageType {
    ENTER_AIRSPACE,
    EXIT_AIRSPACE,
    POSITION_UPDATE,
	REQUEST_POSITION,
	REQUEST_CHANGE_OF_HEADING,
	REQUEST_CHANGE_POSITION,
	REQUEST_CHANGE_ALTITUDE,
	REQUEST_AUGMENTED_INFO,
	CHANGE_TIME_CONSTRAINT_COLLISIONS,
	EXIT,
	COLLISION_DETECTED
};

struct Message {
    bool header; //intra process; 1: interprocess
	MessageType type;
    int planeID;
    void* data;  // Pointer to the message data
    size_t dataSize;  // Size of the serialized data
};

typedef struct {
	int id;
	double PositionX, PositionY, PositionZ, VelocityX, VelocityY, VelocityZ;
} msg_plane_info;

typedef struct {
	int ID;
	double VelocityX, VelocityY, VelocityZ;
	double altitude;
} msg_change_heading;

typedef struct {
	double x,y,z;
} msg_change_position;

typedef struct {
	double altitude;
} msg_change_altitude;

typedef struct {
	int seconds;
} msg_time_constraint;

typedef struct {
	int plane1;
	int plane2;
} msg_collision_pair;

// Shared memory structure
struct SharedMemory {
    msg_plane_info plane_data[MAX_PLANES];
    int count;  // Keep track of the number of planes in the buffer
    std::atomic<bool> is_empty;  // New flag to indicate if there are no planes in the buffer
    bool start;
    uint64_t timestamp;  // Timestamp of the last write
    bool simulation_over;
    pthread_mutex_t mutex;  // process-shared
};

struct Message_inter_process {
	bool header; //intra process; 1: interprocess
	MessageType type;
    int planeID;
    std::array<char, 256> data;  // Pointer to the message data
    size_t dataSize;  // Size of the serialized data
};

#define MAX_COLLISION_PAIRS (sizeof(std::array<char, 256>) / sizeof(msg_collision_pair))
