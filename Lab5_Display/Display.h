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

class Display {
public:
	explicit Display(bool useAnsi);
	~Display();

	bool start();
	void join();

private:
	void readerLoop();
	void serverLoop();
	void stopServer();

	std::string render(const RadarSnapshot& snap);
	std::string renderTable(const RadarSnapshot& snap, const std::set<int>& flagged);
	std::string renderGrid(const RadarSnapshot& snap, const std::set<int>& flagged);

	bool useAnsi;
	SharedMemory* shm = nullptr;
	int shm_fd = -1;
	name_attach_t* channel = nullptr;
	std::thread readerThread;
	std::thread serverThread;
	std::atomic<uint64_t> lastTick{0};

	std::mutex alertMutex;
	std::vector<std::pair<int,int>> activePairs;
	std::deque<std::string> alertHistory;
};

#endif /* DISPLAY_H_ */
