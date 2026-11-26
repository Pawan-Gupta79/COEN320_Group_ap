#include "Display.h"
#include "ATCTimer.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>

// Grid size: the 100 000 x 100 000 airspace drawn as 50 columns x 20 rows
static const int GRID_COLS = 50;
static const int GRID_ROWS = 20;
static const size_t MAX_HISTORY = 5;

// ANSI escape codes (only used when useAnsi is true)
static const char* ANSI_CLEAR = "\033[2J\033[H";
static const char* ANSI_RED   = "\033[1;31m";
static const char* ANSI_BOLD  = "\033[1m";
static const char* ANSI_RESET = "\033[0m";

Display::Display(bool ansi) : useAnsi(ansi) {}

Display::~Display() {
	join();
	stopServer();
	closeRadarSharedMemory(shm, shm_fd);   // munmap + close (Radar unlinks)
}

bool Display::start() {
	// 1. IPC server channel first, so collision alerts can arrive at any time
	channel = name_attach(NULL, DISPLAY_CHANNEL_NAME, 0);
	if (channel == NULL) {
		perror("Display: name_attach failed");
		return false;
	}
	serverThread = std::thread(&Display::serverLoop, this);

	// 2. Wait for the Radar's shared memory
	bool announced = false;
	while ((shm = openRadarSharedMemory(shm_fd)) == nullptr) {
		if (!announced) {
			std::cout << "Display: waiting for Radar (" << RADAR_SHM_NAME << ")..." << std::endl;
			announced = true;
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// 3. Periodic reader/renderer
	readerThread = std::thread(&Display::readerLoop, this);
	return true;
}

void Display::join() {
	if (readerThread.joinable()) {
		readerThread.join();
	}
}

// ---------------------------------------------------------------------------
// Reader: one refresh per second, driven by a QNX periodic timer
// ---------------------------------------------------------------------------
void Display::readerLoop() {
	ATCTimer timer(1, 0);
	while (true) {
		RadarSnapshot snap = takeSnapshot(shm);   // copy under the process-shared mutex
		lastTick = snap.timestamp;

		// Build the whole frame in memory, then write it in ONE call:
		// no flicker and no half-drawn frames.
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
	// Copy the alert state under its lock, then draw without holding it
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

	// Collision banner: shown on every refresh while the conflict lasts
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

// Top-down view (X to the right, Y upwards). Each aircraft is drawn with the
// last digit of its ID; '+' means several aircraft share a cell.
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

// ---------------------------------------------------------------------------
// IPC server: collision alerts from the ComputerSystem
// ---------------------------------------------------------------------------
void Display::serverLoop() {
	while (true) {
		IpcBuffer buf;
		int rcvid = MsgReceive(channel->chid, &buf, sizeof(buf), NULL);
		if (rcvid == -1) {
			perror("Display: MsgReceive failed");
			break;
		}
		if (handleSystemMessage(rcvid, buf)) continue;   // pulse / name_open connect

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

		// Reply FIRST: the ComputerSystem's monitor thread is blocked in MsgSend
		// until we reply, and it shouldn't wait while we update the screen.
		MsgReply(rcvid, EOK, &status, sizeof(status));

		// Decode the list of pairs (the message is always the COMPLETE current list)
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
				// Print immediately (prompt warning), and keep it in the history panel
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
	sendToServer(DISPLAY_CHANNEL_NAME, exitMsg);   // unblock our own MsgReceive
	serverThread.join();
	if (channel != nullptr) {
		name_detach(channel, 0);
		channel = nullptr;
	}
}
