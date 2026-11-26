#pragma once
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include "Msg_structs.h"

union IpcBuffer {
	struct _pulse pulse;
	uint16_t type;
	Message_inter_process msg;
};

// Handles pulses and the _IO_CONNECT sent by name_open(); returns true if buf was one of those
inline bool handleSystemMessage(int rcvid, IpcBuffer& buf) {
	if (rcvid == 0) {
		if (buf.pulse.code == _PULSE_CODE_DISCONNECT) {
			ConnectDetach(buf.pulse.scoid);
		}
		return true;
	}
	if (buf.msg.header) {
		return false;
	}
	if (buf.type == _IO_CONNECT) {
		MsgReply(rcvid, EOK, NULL, 0);
		return true;
	}
	if (buf.type >= _IO_BASE && buf.type <= _IO_MAX) {
		MsgError(rcvid, ENOSYS);
		return true;
	}
	MsgError(rcvid, EINVAL);
	return true;
}

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

// Returns the server's reply status, or -1 if it could not be reached
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

struct RadarSnapshot {
	std::vector<msg_plane_info> planes;
	uint64_t timestamp = 0;
	bool simulationOver = false;
};

// Mapped read/write because locking the process-shared mutex writes to it
inline SharedMemory* openRadarSharedMemory(int& fd) {
	fd = shm_open(RADAR_SHM_NAME, O_RDWR, 0666);
	if (fd == -1) {
		return nullptr;
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
}

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
