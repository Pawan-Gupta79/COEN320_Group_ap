#include "Display.h"
#include "ATCTimer.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>

static const int GRID_COLS = 50;
static const int GRID_ROWS = 20;
static const size_t MAX_HISTORY = 5;

static const char* ANSI_CLEAR = "\033[2J\033[H";
static const char* ANSI_RED   = "\033[1;31m";
static const char* ANSI_BOLD  = "\033[1m";
static const char* ANSI_RESET = "\033[0m";

Display::Display(bool ansi) : useAnsi(ansi) {}

Display::~Display() {
	join();
	stopServer();
	closeRadarSharedMemory(shm, shm_fd);
}

bool Display::start() {
	channel = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
	if (channel == NULL) {
		perror("Display: name_attach failed");
		return false;
	}
	serverThread = std::thread(&Display::serverLoop, this);

	bool announced = false;
	while ((shm = openRadarSharedMemory(shm_fd)) == nullptr) {
		if (!announced) {
			std::cout << "Display: waiting for Radar (" << RADAR_SHM_NAME << ")..." << std::endl;
			announced = true;
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	readerThread = std::thread(&Display::readerLoop, this);
	return true;
}

void Display::join() {
	if (readerThread.joinable()) {
		readerThread.join();
	}
}

void Display::readerLoop() {
	ATCTimer timer(1, 0);
	while (true) {
		RadarSnapshot snap = takeSnapshot(shm);
		lastTick = snap.timestamp;

		std::string frame = render(snap);
		std::cout << frame << std::flush;

		if (snap.simulationOver) {
			std::cout << "\nRadar reports the simulation is over. Display exiting.\n";
			break;
		}
		timer.waitTimer();
	}
}

std::string Display::render(const RadarSnapshot& snap) {
	std::vector<std::pair<int,int>> pairs;
	std::deque<std::string> history;
	{
		std::lock_guard<std::mutex> lock(alertMutex);
		pairs = activePairs;
		history = alertHistory;
	}
	std::set<int> flagged;
	for (const auto& p : pairs) {
		flagged.insert(p.first);
		flagged.insert(p.second);
	}

	std::ostringstream out;
	if (useAnsi) out << ANSI_CLEAR;
	else         out << "\n\n";

	out << (useAnsi ? ANSI_BOLD : "")
	    << "==================== GROUP AP  -  ATC DISPLAY ====================" << (useAnsi ? ANSI_RESET : "") << "\n"
	    << " Radar time: " << std::setw(5) << snap.timestamp << " s"
	    << "     Aircraft in airspace: " << std::setw(3) << snap.planes.size()
	    << "     Active conflicts: " << pairs.size() << "\n";

	for (const auto& p : pairs) {
		out << (useAnsi ? ANSI_RED : "")
		    << " *** COLLISION WARNING: Aircraft " << p.first << " and " << p.second << " ***"
		    << (useAnsi ? ANSI_RESET : "") << "\n";
	}
	out << "------------------------------------------------------------------\n";

	if (snap.planes.empty()) {
		out << "\n   (no aircraft in the airspace)\n\n";
	} else {
		out << renderTable(snap, flagged) << "\n" << renderGrid(snap, flagged);
	}

	if (!history.empty()) {
		out << "------------------------------------------------------------------\n"
		    << " Recent alerts:\n";
		for (const auto& h : history) out << "   " << h << "\n";
	}
	return out.str();
}

std::string Display::renderTable(const RadarSnapshot& snap, const std::set<int>& flagged) {
	std::vector<msg_plane_info> planes = snap.planes;
	std::sort(planes.begin(), planes.end(),
	          [](const msg_plane_info& a, const msg_plane_info& b) { return a.id < b.id; });

	std::ostringstream t;
	t << std::fixed << std::setprecision(0);
	t << "  " << std::left << std::setw(5) << "ID" << std::right
	  << std::setw(9) << "X" << std::setw(9) << "Y" << std::setw(9) << "ALT"
	  << std::setw(7) << "VX" << std::setw(7) << "VY" << std::setw(7) << "VZ"
	  << "   STATUS\n";
	for (const auto& p : planes) {
		bool bad = flagged.count(p.id) > 0;
		if (bad && useAnsi) t << ANSI_RED;
		t << "  " << std::left << std::setw(5) << p.id << std::right
		  << std::setw(9) << p.PositionX << std::setw(9) << p.PositionY << std::setw(9) << p.PositionZ
		  << std::setw(7) << p.VelocityX << std::setw(7) << p.VelocityY << std::setw(7) << p.VelocityZ
		  << "   " << (bad ? "!! CONFLICT" : "ok");
		if (bad && useAnsi) t << ANSI_RESET;
		t << "\n";
	}
	return t.str();
}

std::string Display::renderGrid(const RadarSnapshot& snap, const std::set<int>& flagged) {
	std::vector<std::string> cells(GRID_ROWS, std::string(GRID_COLS, '.'));
	std::vector<std::vector<bool>> red(GRID_ROWS, std::vector<bool>(GRID_COLS, false));

	for (const auto& p : snap.planes) {
		int col = static_cast<int>(p.PositionX / AIRSPACE_X_MAX * GRID_COLS);
		int row = GRID_ROWS - 1 - static_cast<int>(p.PositionY / AIRSPACE_Y_MAX * GRID_ROWS);
		col = std::max(0, std::min(GRID_COLS - 1, col));
		row = std::max(0, std::min(GRID_ROWS - 1, row));
		char mark = static_cast<char>('0' + (p.id % 10));
		cells[row][col] = (cells[row][col] == '.') ? mark : '+';
		if (flagged.count(p.id)) red[row][col] = true;
	}

	std::ostringstream g;
	g << "  Y ^  top-down view (each cell = " << AIRSPACE_X_MAX / GRID_COLS << " x "
	  << AIRSPACE_Y_MAX / GRID_ROWS << ")\n";
	for (int r = 0; r < GRID_ROWS; ++r) {
		g << "    |";
		for (int c = 0; c < GRID_COLS; ++c) {
			if (red[r][c] && useAnsi) g << ANSI_RED << cells[r][c] << ANSI_RESET;
			else                      g << cells[r][c];
		}
		g << "|\n";
	}
	g << "    +" << std::string(GRID_COLS, '-') << "+-> X\n";
	return g.str();
}

void Display::serverLoop() {
	while (true) {
		IpcBuffer buf;
		int rcvid = MsgReceive(channel->chid, &buf, sizeof(buf), NULL);
		if (rcvid == -1) {
			perror("Display: MsgReceive failed");
			break;
		}
		if (handleSystemMessage(rcvid, buf)) continue;

		int status = 0;
		const Message_inter_process& msg = buf.msg;
		if (msg.type == MessageType::EXIT) {
			MsgReply(rcvid, EOK, &status, sizeof(status));
			break;
		}
		if (msg.type != MessageType::COLLISION_DETECTED) {
			status = -1;
			MsgReply(rcvid, EOK, &status, sizeof(status));
			continue;
		}

		MsgReply(rcvid, EOK, &status, sizeof(status));

		size_t n = msg.dataSize / sizeof(msg_collision_pair);
		if (n > MAX_COLLISION_PAIRS) n = MAX_COLLISION_PAIRS;
		std::vector<std::pair<int,int>> pairs;
		for (size_t i = 0; i < n; ++i) {
			msg_collision_pair cp;
			std::memcpy(&cp, msg.data.data() + i * sizeof(cp), sizeof(cp));
			pairs.push_back({cp.plane1, cp.plane2});
		}

		std::lock_guard<std::mutex> lock(alertMutex);
		for (const auto& p : pairs) {
			bool isNew = std::find(activePairs.begin(), activePairs.end(), p) == activePairs.end();
			if (isNew) {
				std::ostringstream line;
				line << "[t=" << lastTick.load() << "s] *** COLLISION WARNING: Aircraft "
				     << p.first << " and " << p.second << " ***";
				std::cout << (useAnsi ? ANSI_RED : "") << line.str()
				          << (useAnsi ? ANSI_RESET : "") << std::endl;
				alertHistory.push_front(line.str());
				if (alertHistory.size() > MAX_HISTORY) alertHistory.pop_back();
			}
		}
		activePairs = pairs;
	}
}

void Display::stopServer() {
	if (!serverThread.joinable()) return;
	Message_inter_process exitMsg;
	buildMessage(exitMsg, MessageType::EXIT, -1);
	sendToServer(DISPLAY_CHANNEL_NAME, exitMsg);
	serverThread.join();
	if (channel != nullptr) {
		name_detach(channel, 0);
		channel = nullptr;
	}
}
