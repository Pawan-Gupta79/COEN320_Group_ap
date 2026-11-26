#pragma once
// ============================================================================
// Small helpers shared by every name_attach() server / client in Lab5.
// (Copied into Lab5_Display as well; keep both copies identical.)
// ============================================================================
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>   // _IO_CONNECT, _IO_BASE, _IO_MAX
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include "Msg_structs.h"

// Receive buffer: large enough for any of our messages OR a QNX pulse.
union IpcBuffer {
	struct _pulse pulse;
	uint16_t type;                 // first 2 bytes: used to spot QNX system messages
	Message_inter_process msg;
};

// A name_attach() channel also receives SYSTEM traffic besides our messages:
//  - pulses (rcvid == 0), e.g. _PULSE_CODE_DISCONNECT when a client goes away
//  - _IO_CONNECT when a client calls name_open(): must be replied with EOK,
//    otherwise the client's name_open() stays blocked
// Returns true if 'buf' was such a system message (already handled -> skip it).
inline bool handleSystemMessage(int rcvid, IpcBuffer& buf) {
	if (rcvid == 0) {
		if (buf.pulse.code == _PULSE_CODE_DISCONNECT) {
			ConnectDetach(buf.pulse.scoid);
		}
		return true;
	}
	// All of OUR messages are Message_inter_process with header == 1, so their first
	// byte is 0x01. QNX's _IO_CONNECT (0x0100) has first byte 0x00, so no confusion.
	if (buf.msg.header) {
		return false;
	}
	if (buf.type == _IO_CONNECT) {
		MsgReply(rcvid, EOK, NULL, 0);
		return true;
	}
	if (buf.type >= _IO_BASE && buf.type <= _IO_MAX) {
		MsgError(rcvid, ENOSYS);   // some other I/O request we don't support
		return true;
	}
	MsgError(rcvid, EINVAL);       // not ours, not a known system message
	return true;
}

// Build a zero-initialised inter-process message with an optional payload.
// Zeroing matters: it guarantees no garbage in padding bytes or unused data.
inline void buildMessage(Message_inter_process& m, MessageType type, int planeID,
                         const void* payload = nullptr, size_t size = 0) {
	std::memset(&m, 0, sizeof(m));
	m.header = true;
	m.type = type;
	m.planeID = planeID;
	if (payload != nullptr && size > 0) {
		if (size > m.data.size()) size = m.data.size();
		std::memcpy(m.data.data(), payload, size);
	}
	m.dataSize = size;
}

// Send 'msg' to the server attached as 'name' and wait for its int status reply.
// Returns the server's status (0 = OK), or -1 if the server could not be reached.
inline int sendToServer(const char* name, const Message_inter_process& msg) {
	int coid = name_open(name, 0);
	if (coid == -1) {
		return -1;
	}
	int reply = -1;
	if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
		reply = -1;
	}
	name_close(coid);
	return reply;
}

// ---------------------------------------------------------------------------
// Reader side of the Radar's shared memory
// ---------------------------------------------------------------------------
// A consistent copy of the shared memory, taken while holding its mutex.
struct RadarSnapshot {
	std::vector<msg_plane_info> planes;
	uint64_t timestamp = 0;
	bool simulationOver = false;
};

// Open + map the Radar's shared memory. Returns the mapping, or nullptr if the
// Radar hasn't created it yet (caller retries). Mapped read/write because
// locking the process-shared mutex WRITES to it; we never modify the data.
inline SharedMemory* openRadarSharedMemory(int& fd) {
	fd = shm_open(RADAR_SHM_NAME, O_RDWR, 0666);
	if (fd == -1) {
		return nullptr;  // ENOENT: Radar not running yet
	}
	void* addr = mmap(nullptr, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (addr == MAP_FAILED) {
		perror("mmap of radar shared memory failed");
		close(fd);
		fd = -1;
		return nullptr;
	}
	return static_cast<SharedMemory*>(addr);
}

inline void closeRadarSharedMemory(SharedMemory*& shm, int& fd) {
	if (shm != nullptr) {
		munmap(shm, sizeof(SharedMemory));
		shm = nullptr;
	}
	if (fd != -1) {
		close(fd);
		fd = -1;
	}
	// No shm_unlink here: the Radar owns (created) the object and deletes it.
}

// Copy everything we need while holding the lock, then release it quickly so
// the Radar (the writer) is never blocked for long.
inline RadarSnapshot takeSnapshot(SharedMemory* shm) {
	RadarSnapshot s;
	pthread_mutex_lock(&shm->mutex);
	int n = shm->count;
	if (n < 0) n = 0;
	if (n > MAX_PLANES) n = MAX_PLANES;
	if (!shm->is_empty.load()) {
		s.planes.assign(shm->plane_data, shm->plane_data + n);
	}
	s.timestamp = shm->timestamp;
	s.simulationOver = shm->simulation_over;
	pthread_mutex_unlock(&shm->mutex);
	return s;
}
