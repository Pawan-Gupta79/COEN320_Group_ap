#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <atomic>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sys/dispatch.h>
#include "Msg_structs.h"
#include "IpcHelpers.h"

// The Display runs as its own process, independent of Radar and ComputerSystem:
//  - Reader thread: reads the Radar's shared memory once per second and draws
//    a table plus a top-down grid of the airspace.
//  - Server thread: an IPC server (DISPLAY_CHANNEL_NAME) that receives
//    COLLISION_DETECTED messages from the ComputerSystem.
class Display {
public:
	explicit Display(bool useAnsi);
	~Display();

	bool start();   // attach channel, wait for radar, start threads
	void join();    // returns when the simulation is over

private:
	void readerLoop();
	void serverLoop();
	void stopServer();

	std::string render(const RadarSnapshot& snap);
	std::string renderTable(const RadarSnapshot& snap, const std::set<int>& flagged);
	std::string renderGrid(const RadarSnapshot& snap, const std::set<int>& flagged);

	bool useAnsi;                          // colours + clear screen (needs a real terminal)
	SharedMemory* shm = nullptr;
	int shm_fd = -1;
	name_attach_t* channel = nullptr;
	std::thread readerThread;
	std::thread serverThread;
	std::atomic<uint64_t> lastTick{0};

	// Shared between the server thread (writes) and the reader thread (draws)
	std::mutex alertMutex;
	std::vector<std::pair<int,int>> activePairs;   // current conflicts
	std::deque<std::string> alertHistory;          // last few alerts, newest first
};

#endif /* DISPLAY_H_ */
