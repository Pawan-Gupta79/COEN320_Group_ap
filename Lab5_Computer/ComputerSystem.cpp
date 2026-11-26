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
#define display_channel_name DISPLAY_CHANNEL_NAME

static const int MONITOR_PRIORITY = 20;


ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    stopOperatorListener();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
	// Open the shared memory object
	bool announced = false;
	while (true) {
        // COEN320 Task 3.2
		// Attempt to open the shared memory object (You need to use the same name as Task 2 in Radar)
        // In case of error, retry until successful
        // COEN320 Task 3.3
		// Map the shared memory object into the process's address space
        // The shared memory should be mapped to "shared_mem" (check for errors)
		shared_mem = openRadarSharedMemory(shm_fd);
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
	closeRadarSharedMemory(shared_mem, shm_fd);
}

bool ComputerSystem::startMonitoring() {
	operatorChannel = name_attach(NULL, COMPUTER_CHANNEL_NAME, 0);
	if (operatorChannel == NULL) {
		perror("ComputerSystem: name_attach failed");
		return false;
	}
	monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);

    if (initializeSharedMemory()) {
        running = true; // will be used in monitorAirspace
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);

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
	ATCTimer timer(1,0);
	bool wasEmpty = false;
	bool firstPass = true;

	while (running) {
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
		checkCollision(snap.timestamp, snap.planes);

        // Sleep for a short interval before the next poll
		timer.waitTimer();
    }
	running = false;
	std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    // COEN320 Task 3.4
    // detect collisions between planes in the airspace within the time constraint
    // You need to Iterate through each pair of planes and in case of collision,
    // store the pair of plane IDs that are predicted to collide
    // You can use the function checkAxes provided below to check if two planes will collide
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
    // in case of collision, send message to Display system
    /*
    HINT:
    In case of collision a Message (type Message_inter_process) should be sent to the Display system
    The data field of the message should contain the list of pairs of plane IDs that are predicted to collide
    Make sure to fill dataSize field of the message appropriately
    */
    if (current != lastCollisions || !displayUpToDate) {
    	std::vector<msg_collision_pair> pairs;
    	for (const auto& p : current) {
    		if (pairs.size() >= MAX_COLLISION_PAIRS) break;
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

// Time interval in which |d + v*t| < C on one axis
static void axisConflictInterval(double d, double v, double C, double& lo, double& hi) {
	const double INF = std::numeric_limits<double>::infinity();
	if (std::fabs(v) < 1e-9) {
		if (std::fabs(d) < C) { lo = -INF; hi = INF; }
		else                  { lo = INF;  hi = -INF; }
		return;
	}
	double t1 = (-C - d) / v;
	double t2 = ( C - d) / v;
	lo = std::min(t1, t2);
	hi = std::max(t1, t2);
}

bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // COEN320 Task 3.4
    // A collision is defined as two planes entering the defined airspace constraints within the time constraint
    // You need to implement the logic to check if plane1 and plane2 will collide within the time constraint
    // Return true if they will collide, false otherwise
    // A simple approach is to just check if their positions will be within the defined constraints (e.g., CONSTRAINT_X, CONSTRAINT_Y, CONSTRAINT_Z)
    // A more accurate approach would involve calculating their future positions based on their velocities
    // and checking if those future positions will be within the defined constraints within the time constraint
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
	return sendToServer(display_channel_name, msg) != -1;
}

void ComputerSystem::processMessage() {
	while (true) {
		IpcBuffer buf;
		int rcvid = MsgReceive(operatorChannel->chid, &buf, sizeof(buf), NULL);
		if (rcvid == -1) {
			if (!listen) break;
			perror("ComputerSystem: MsgReceive failed");
			continue;
		}
		if (handleSystemMessage(rcvid, buf)) continue;

		const Message_inter_process& msg = buf.msg;
		int status = 0;
		switch (msg.type) {
		case MessageType::EXIT:
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
			status = sendMessagesToComms(msg);
			break;
		default:
			std::cerr << "ComputerSystem: unexpected message type " << static_cast<int>(msg.type) << "\n";
			status = -1;
			break;
		}
		MsgReply(rcvid, EOK, &status, sizeof(status));
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
