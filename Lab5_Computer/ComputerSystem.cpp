#include "ComputerSystem.h"
#include "ATCTimer.h"
#include "IpcHelpers.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <limits>
#include <sched.h>
#include <sys/dispatch.h>
#include <cstring> // For memcpy

// COEN320 Task 3.1, set the display channel name (it should be the same as the Coen320_Lab (Task0) Display channel name)
#define display_channel_name DISPLAY_CHANNEL_NAME   // "group_ap_display", see Msg_structs.h

// Priority for the collision-monitor thread. QNX's default is 10; the
// safety-critical task (collision detection) runs above the operator console
// and comms threads so a busy console can never delay a collision warning.
static const int MONITOR_PRIORITY = 20;


ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    stopOperatorListener();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
	// COEN320 Task 3.2 / 3.3
	// Open the Radar's shared memory and map it into "shared_mem".
	// The Radar may not be running yet, so retry once per second until it is.
	bool announced = false;
	while (true) {
		shared_mem = openRadarSharedMemory(shm_fd);   // shm_open + mmap (IpcHelpers.h)
		if (shared_mem != nullptr) {
			std::cout << "Shared memory " << RADAR_SHM_NAME << " initialized successfully." << std::endl;
			return true;
		}
		if (errno != ENOENT) {
			perror("ComputerSystem: shm_open failed");
		}
		if (!announced) {
			std::cout << "Waiting for Radar to create " << RADAR_SHM_NAME << "..." << std::endl;
			announced = true;
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void ComputerSystem::cleanupSharedMemory() {
	closeRadarSharedMemory(shared_mem, shm_fd);  // munmap + close (Radar does the unlink)
}

bool ComputerSystem::startMonitoring() {
	// 1. Open our own channel first so the OperatorConsole can reach us
	//    even while we wait for the Radar.
	operatorChannel = name_attach(NULL, COMPUTER_CHANNEL_NAME, 0);
	if (operatorChannel == NULL) {
		perror("ComputerSystem: name_attach failed");
		return false;
	}
	monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);

	// 2. Wait for the Radar's shared memory, then start the monitor thread.
    if (initializeSharedMemory()) {
        running = true; // will be used in monitorAirspace
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);

        // Give the safety-critical thread a higher real-time priority
        sched_param param;
        param.sched_priority = MONITOR_PRIORITY;
        int rc = pthread_setschedparam(monitorThread.native_handle(), SCHED_RR, &param);
        if (rc != 0) {
        	std::cerr << "ComputerSystem: could not raise monitor priority: " << strerror(rc) << "\n";
        }
        return true;
    } else {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
}

void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void ComputerSystem::monitorAirspace() {
	ATCTimer timer(1,0);   // periodic: one radar scan per second
	bool wasEmpty = false;
	bool firstPass = true;

	while (running) {
		// Copy the shared data under the process-shared mutex (see takeSnapshot)
		RadarSnapshot snap = takeSnapshot(shared_mem);

		if (snap.simulationOver) {
			std::cout << "Radar reports the simulation is over. Stopping monitoring.\n";
			break;
		}

		bool isEmpty = snap.planes.empty();
		if (isEmpty && (firstPass || !wasEmpty)) {
			std::cout << "No planes in airspace (t=" << snap.timestamp << "s). Waiting...\n";
		}
		wasEmpty = isEmpty;
		firstPass = false;

		//**************Call Collision Detector*********************
		// Called even with 0 or 1 planes: that clears any old warnings
		checkCollision(snap.timestamp, snap.planes);

        // Wait for the next period
		timer.waitTimer();
    }
	running = false;
	std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    // COEN320 Task 3.4
    // Check every PAIR of planes once (i < j): n*(n-1)/2 checks.
    std::set<std::pair<int,int>> current;
    for (size_t i = 0; i < planes.size(); ++i) {
    	for (size_t j = i + 1; j < planes.size(); ++j) {
    		if (checkAxes(planes[i], planes[j])) {
    			int a = std::min(planes[i].id, planes[j].id);
    			int b = std::max(planes[i].id, planes[j].id);
    			current.insert({a, b});
    		}
    	}
    }

    // Console warning only for NEW pairs, so the console isn't flooded every second
    for (const auto& p : current) {
    	if (lastCollisions.find(p) == lastCollisions.end()) {
    		std::cout << "[t=" << currentTime << "s] *** COLLISION WARNING: Aircraft "
    		          << p.first << " and " << p.second << " within "
    		          << timeConstraintCollisionFreq.load() << "s ***" << std::endl;
    	}
    }
    for (const auto& p : lastCollisions) {
    	if (current.find(p) == current.end()) {
    		std::cout << "[t=" << currentTime << "s] Conflict cleared: Aircraft "
    		          << p.first << " and " << p.second << std::endl;
    	}
    }

    // COEN320 Task 3.5
    // Tell the Display whenever the set of conflicts CHANGES (including when it
    // becomes empty, so the Display can remove the highlight). The Display
    // treats each message as "the complete current list".
    if (current != lastCollisions || !displayUpToDate) {
    	std::vector<msg_collision_pair> pairs;
    	for (const auto& p : current) {
    		if (pairs.size() >= MAX_COLLISION_PAIRS) break;  // 256-byte payload limit
    		pairs.push_back({p.first, p.second});
    	}
    	Message_inter_process msg_to_send;
    	buildMessage(msg_to_send, MessageType::COLLISION_DETECTED, -1,
    	             pairs.data(), pairs.size() * sizeof(msg_collision_pair));
    	bool ok = sendCollisionToDisplay(msg_to_send);
    	if (!ok && displayUpToDate) {
    		std::cerr << "Display (" << display_channel_name << ") not reachable; will retry.\n";
    	}
    	displayUpToDate = ok;
    }
    lastCollisions = current;
}

// Solve, for ONE axis, when |d + v*t| < C, where d = position difference and
// v = velocity difference. Returns the time interval [lo, hi] (may be empty).
static void axisConflictInterval(double d, double v, double C, double& lo, double& hi) {
	const double INF = std::numeric_limits<double>::infinity();
	if (std::fabs(v) < 1e-9) {
		// Same speed on this axis: the gap never changes
		if (std::fabs(d) < C) { lo = -INF; hi = INF; }   // always too close
		else                  { lo = INF;  hi = -INF; }  // never too close (empty)
		return;
	}
	double t1 = (-C - d) / v;
	double t2 = ( C - d) / v;
	lo = std::min(t1, t2);
	hi = std::max(t1, t2);
}

bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // COEN320 Task 3.4
    // Planes move in straight lines: p(t) = p0 + v*t. They are in conflict at time t
    // if they are too close on X AND Y AND Z at that SAME t. For each axis we solve
    // exactly for the time interval where that axis is too close, then intersect the
    // three intervals with the look-ahead window [0, T]. If anything is left, the
    // planes will violate separation within T seconds.
    //   (Exact, and cheaper than simulating t = 0,1,2,...,T step by step.)
    double lo = 0.0;
    double hi = static_cast<double>(timeConstraintCollisionFreq.load());

    double axLo, axHi;
    axisConflictInterval(plane2.PositionX - plane1.PositionX, plane2.VelocityX - plane1.VelocityX, CONSTRAINT_X, axLo, axHi);
    lo = std::max(lo, axLo); hi = std::min(hi, axHi);
    axisConflictInterval(plane2.PositionY - plane1.PositionY, plane2.VelocityY - plane1.VelocityY, CONSTRAINT_Y, axLo, axHi);
    lo = std::max(lo, axLo); hi = std::min(hi, axHi);
    axisConflictInterval(plane2.PositionZ - plane1.PositionZ, plane2.VelocityZ - plane1.VelocityZ, CONSTRAINT_Z, axLo, axHi);
    lo = std::max(lo, axLo); hi = std::min(hi, axHi);

    return lo <= hi;
}


bool ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg){
	// name_open + MsgSend + name_close (the template never closed the connection,
	// which leaked one connection per message). Returns false if the Display isn't running.
	return sendToServer(display_channel_name, msg) != -1;
}

// ---------------------------------------------------------------------------
// Task 4: commands from the OperatorConsole arrive on COMPUTER_CHANNEL_NAME
// ---------------------------------------------------------------------------
void ComputerSystem::processMessage() {
	while (true) {
		IpcBuffer buf;
		int rcvid = MsgReceive(operatorChannel->chid, &buf, sizeof(buf), NULL);
		if (rcvid == -1) {
			if (!listen) break;
			perror("ComputerSystem: MsgReceive failed");
			continue;
		}
		if (handleSystemMessage(rcvid, buf)) continue;  // pulse / name_open connect

		const Message_inter_process& msg = buf.msg;
		int status = 0;
		switch (msg.type) {
		case MessageType::EXIT:            // sent by stopOperatorListener() at shutdown
			MsgReply(rcvid, EOK, &status, sizeof(status));
			return;
		case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS:
			handleTimeConstraintChange(msg);
			break;
		case MessageType::REQUEST_AUGMENTED_INFO:
			printAugmentedInfo(msg.planeID);
			break;
		case MessageType::REQUEST_CHANGE_OF_HEADING:
		case MessageType::REQUEST_CHANGE_POSITION:
		case MessageType::REQUEST_CHANGE_ALTITUDE:
			status = sendMessagesToComms(msg);   // forwarded to the aircraft
			break;
		default:
			std::cerr << "ComputerSystem: unexpected message type " << static_cast<int>(msg.type) << "\n";
			status = -1;
			break;
		}
		MsgReply(rcvid, EOK, &status, sizeof(status));   // unblock the console
	}
}

int ComputerSystem::sendMessagesToComms(const Message_inter_process& msg) {
	int status = sendToServer(COMMS_CHANNEL_NAME, msg);
	if (status == -1) {
		std::cerr << "ComputerSystem: command for aircraft " << msg.planeID << " failed\n";
	}
	return status;
}

void ComputerSystem::handleTimeConstraintChange(const Message_inter_process& msg) {
	msg_time_constraint tc;
	std::memcpy(&tc, msg.data.data(), sizeof(tc));
	if (tc.seconds <= 0) {
		std::cerr << "ComputerSystem: ignoring invalid look-ahead " << tc.seconds << "s\n";
		return;
	}
	timeConstraintCollisionFreq = tc.seconds;
	std::cout << "ComputerSystem: collision look-ahead is now " << tc.seconds << "s\n";
}

void ComputerSystem::printAugmentedInfo(int planeID) {
	if (shared_mem == nullptr) {
		std::cout << "No radar data yet.\n";
		return;
	}
	RadarSnapshot snap = takeSnapshot(shared_mem);
	for (const auto& p : snap.planes) {
		if (p.id == planeID) {
			double speed = std::sqrt(p.VelocityX * p.VelocityX + p.VelocityY * p.VelocityY + p.VelocityZ * p.VelocityZ);
			std::cout << std::fixed << std::setprecision(0)
			          << "Aircraft " << p.id << " @ t=" << snap.timestamp << "s\n"
			          << "  position  (" << p.PositionX << ", " << p.PositionY << ", " << p.PositionZ << ")\n"
			          << "  velocity  (" << p.VelocityX << ", " << p.VelocityY << ", " << p.VelocityZ << ")"
			          << "  |v| = " << speed << " per s\n";
			std::cout.unsetf(std::ios::floatfield);
			return;
		}
	}
	std::cout << "Aircraft " << planeID << " is not in the airspace.\n";
}

void ComputerSystem::stopOperatorListener() {
	if (!monitorOperatorInput.joinable()) return;
	// The listener is blocked in MsgReceive. Unblock it cleanly by sending it an EXIT message.
	listen = false;
	Message_inter_process exitMsg;
	buildMessage(exitMsg, MessageType::EXIT, -1);
	sendToServer(COMPUTER_CHANNEL_NAME, exitMsg);
	monitorOperatorInput.join();
	if (operatorChannel != nullptr) {
		name_detach(operatorChannel, 0);
		operatorChannel = nullptr;
	}
}
