#pragma once
// ============================================================================
// Shared protocol for the whole ATC system.
// This file is copied into Lab4_ATC, Lab5_Computer and Lab5_Display.
// ALL COPIES MUST STAY IDENTICAL: the processes exchange these structs as
// raw bytes, so any difference in layout breaks the communication.
// ============================================================================
#include <atomic>
#include <array>
#include <cstdint>
#include <cstddef>
#include <pthread.h>

// ---------------------------------------------------------------------------
// Names of shared resources (prefixed with our group name so we never clash
// with other groups on the shared Raspberry Pi)
// ---------------------------------------------------------------------------
// Shared memory object, appears as /dev/shmem/group_ap_radar_shm
#define RADAR_SHM_NAME        "/group_ap_radar_shm"
// name_attach() channels
#define RADAR_CHANNEL_NAME    "group_ap"           // Aircraft -> Radar (enter/exit)
#define PLANE_CHANNEL_PREFIX  "group_ap_plane_"    // + plane id: Radar/Comms -> Aircraft
#define COMPUTER_CHANNEL_NAME "group_ap_computer"  // OperatorConsole -> ComputerSystem
#define COMMS_CHANNEL_NAME    "group_ap_comms"     // ComputerSystem -> CommunicationsSystem
#define DISPLAY_CHANNEL_NAME  "group_ap_display"   // ComputerSystem -> Display (collisions)

// Airspace limits (same values Aircraft uses)
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

// Intra-process message (Aircraft <-> Radar, same process, so a pointer is OK)
struct Message {
    bool header; //0: intra process; 1: interprocess
	MessageType type;
    int planeID;
    void* data;  // Pointer to the message data
    size_t dataSize;  // Size of the serialized data
};

typedef struct {
	int id;
	double PositionX, PositionY, PositionZ, VelocityX, VelocityY, VelocityZ;
} msg_plane_info;

// ---------------------------------------------------------------------------
// Payloads carried in Message_inter_process::data
// ---------------------------------------------------------------------------
// REQUEST_CHANGE_OF_HEADING
typedef struct {
	int ID;
	double VelocityX, VelocityY, VelocityZ;
	double altitude;
} msg_change_heading;

// REQUEST_CHANGE_POSITION
typedef struct {
	double x,y,z;
} msg_change_position;

// REQUEST_CHANGE_ALTITUDE
typedef struct {
	double altitude;
} msg_change_altitude;

// CHANGE_TIME_CONSTRAINT_COLLISIONS
typedef struct {
	int seconds;   // how far ahead (in seconds) the collision predictor looks
} msg_time_constraint;

// COLLISION_DETECTED: data holds an array of these
typedef struct {
	int plane1;
	int plane2;
} msg_collision_pair;

// Shared memory structure (written by Radar, read by ComputerSystem and Display)
struct SharedMemory {
    msg_plane_info plane_data[MAX_PLANES];
    int count;  // Keep track of the number of planes in the buffer
    std::atomic<bool> is_empty;  // True when there are no planes in the buffer
    bool start;                  // True once at least one plane has been seen this run
    uint64_t timestamp;          // Radar tick (seconds) of the last write
    bool simulation_over;        // Set by Radar just before it shuts down -> readers exit
    pthread_mutex_t mutex;       // PROCESS-SHARED mutex protecting all fields above
};

// Inter-process message: the payload is copied INTO the message (a pointer
// would be meaningless in another process's address space)
struct Message_inter_process {
	bool header; //0: intra process; 1: interprocess
	MessageType type;
    int planeID;
    std::array<char, 256> data;  // Serialized payload (one of the msg_* structs above)
    size_t dataSize;  // Size of the serialized data
};

#define MAX_COLLISION_PAIRS (sizeof(std::array<char, 256>) / sizeof(msg_collision_pair))
