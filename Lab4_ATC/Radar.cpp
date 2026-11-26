#include "Radar.h"
#include <sys/dispatch.h>


Radar::Radar(uint64_t& tick_counter) : tick_counter_ref(tick_counter), activeBufferIndex(0), timer(1,0), stopThreads(false) {
	if (!initSharedMemory()) {
		std::cerr << "Radar: could not set up shared memory, exiting" << std::endl;
		exit(EXIT_FAILURE);
	}
	clearSharedMemory(); //For future Use
	// Start threads for listening to airspace events
    Arrival_Departure = std::thread(&Radar::ListenAirspaceArrivalAndDeparture, this);
    UpdatePosition = std::thread(&Radar::ListenUpdatePosition, this);
}

Radar::~Radar() {
    // Join threads to ensure proper cleanup
    shutdown();
    clearSharedMemory();//For future Use

    if (sharedMemPtr != nullptr) {
    	pthread_mutex_lock(&sharedMemPtr->mutex);
    	sharedMemPtr->simulation_over = true;
    	pthread_mutex_unlock(&sharedMemPtr->mutex);
    	std::cout << "Radar: simulation over, notifying readers..." << std::endl;
    	// give the readers time to see the flag before the mutex is destroyed
    	std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    cleanupSharedMemory();
}

bool Radar::initSharedMemory() {
	shm_fd = shm_open(RADAR_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("Radar: shm_open failed");
		return false;
	}

	if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
		perror("Radar: ftruncate failed");
		close(shm_fd);
		shm_unlink(RADAR_SHM_NAME);
		shm_fd = -1;
		return false;
	}

	void* addr = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (addr == MAP_FAILED) {
		perror("Radar: mmap failed");
		close(shm_fd);
		shm_unlink(RADAR_SHM_NAME);
		shm_fd = -1;
		return false;
	}

	sharedMemPtr = static_cast<SharedMemory*>(addr);

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	int rc = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
	if (rc == 0) rc = pthread_mutex_init(&sharedMemPtr->mutex, &attr);
	pthread_mutexattr_destroy(&attr);
	if (rc != 0) {
		std::cerr << "Radar: pthread_mutex_init failed: " << strerror(rc) << std::endl;
		munmap(sharedMemPtr, SHARED_MEMORY_SIZE);
		sharedMemPtr = nullptr;
		close(shm_fd);
		shm_unlink(RADAR_SHM_NAME);
		shm_fd = -1;
		return false;
	}
	sharedMemPtr->simulation_over = false;

	std::cout << "Radar: shared memory " << RADAR_SHM_NAME << " created ("
	          << SHARED_MEMORY_SIZE << " bytes)" << std::endl;
	return true;
}

void Radar::cleanupSharedMemory() {
	if (sharedMemPtr != nullptr) {
		pthread_mutex_destroy(&sharedMemPtr->mutex);
		if (munmap(sharedMemPtr, SHARED_MEMORY_SIZE) == -1) {
			perror("Radar: munmap failed");
		}
		sharedMemPtr = nullptr;
	}
	if (shm_fd != -1) {
		close(shm_fd);
		shm_fd = -1;
	}
	if (shm_unlink(RADAR_SHM_NAME) == -1) {
		perror("Radar: shm_unlink failed");
	}
}

void Radar::shutdown() {
    // Set stop flag and wait for threads to complete
    stopThreads.store(true);

    // If the channel exists, close it properly
    if (Radar_channel) {
        name_detach(Radar_channel, 0);
    }

    if (Arrival_Departure.joinable()) {
        Arrival_Departure.join();
    }
    if (UpdatePosition.joinable()) {
        UpdatePosition.join();
    }
}


// Method to get the current active buffer
std::vector<msg_plane_info>& Radar::getActiveBuffer() {
    return planesInAirspaceData[activeBufferIndex];
}
//Coen320_Lab (Task0): Create channel to be reachable by radar that wants to poll the Airplane
//Radar Channel name should contain your group name
//To choose the channel with concatenating your group name with "Radar"
//Note: It is critical to not interfere with other groups
void Radar::ListenAirspaceArrivalAndDeparture() {
	Radar_channel = name_attach(NULL, RADAR_CHANNEL_NAME, 0);
	if (Radar_channel == NULL) {
		std::cerr << "Failed to create channel for Radar" << std::endl;
		exit(EXIT_FAILURE);
	}
	// Simulated listening for aircraft arrivals and departures
    while (!stopThreads.load()) {
        // Replace with IPC
        Message msg;
        int rcvid = MsgReceive(Radar_channel->chid, &msg, sizeof(msg), nullptr); // Replace with actual channel ID
        if (rcvid == -1) {
        	// Silently skip if MsgReceive fails, but no crash happens
        	// std::cerr << "Error receiving airspace message:" << strerror(errno) << std::endl;
        	continue;
        }
        if (rcvid == 0) {  // pulse
        	continue;
        }

        // Reply back to the client
        int msg_ret = msg.planeID;
        MsgReply(rcvid, 0, &msg_ret, sizeof(msg_ret)); // Send plane's ID back to airplane

        switch (msg.type) {
        case MessageType::ENTER_AIRSPACE:
            addPlaneToAirspace(msg);
            break;
        case MessageType::EXIT_AIRSPACE:
            removePlaneFromAirspace(msg.planeID);
            break;
        default:
        	//All other messages dropped
            //std::cerr << "Unknown airspace message type" << std::endl;
        	break;
        }

    }
}

void Radar::ListenUpdatePosition() {

    while (!stopThreads.load()) {
    	timer.waitTimer(); // Wait for the next timer interval before polling again
    	// Only poll airspace if there are planes
    	bool hasPlanes;
    	{
    		std::lock_guard<std::mutex> lock(airspaceMutex);
    		hasPlanes = !planesInAirspace.empty();
    	}
        if (hasPlanes) {
            pollAirspace();  // Call pollAirspace() to gather position data
            writeToSharedMemory();  // Write active buffer to shared memory //For future Use
            wasAirspaceEmpty = false;
        } else if (!wasAirspaceEmpty){
        	// Only write empty buffer once after transition to empty
        	writeToSharedMemory();  // Write to shared mem when all planes have left the airspace //For future Use
        	wasAirspaceEmpty = true;  // Set flag to indicate airspace is empty
        } else{
        	//std::cout << "Airspace is empty\n";
        }

    }
}

void Radar::pollAirspace(){

	airspaceMutex.lock();
	// Make a copy of the current planes in airspace to avoid modification during iteration
	std::unordered_set<int> planesToPoll = planesInAirspace;
	airspaceMutex.unlock();

	int inactiveBufferIndex = (activeBufferIndex + 1) % 2;
	std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[inactiveBufferIndex];
	inactiveBuffer.clear();


	//make channel to aircraft
	for (int planeID: planesToPoll){

		airspaceMutex.lock();
		bool isPlaneInAirspace = planesInAirspace.find(planeID) != planesInAirspace.end();
		airspaceMutex.unlock();
		if (isPlaneInAirspace){
			try {
			// Confirm that the plane is still in airspace
				msg_plane_info plane_info = getAircraftData(planeID);
				inactiveBuffer.emplace_back(plane_info);
			} catch (const std::exception& e) {
				// if error to process plane get next id and exception description
				//std::cerr << "Radar: Failed to get plane data " << planeID << ": " << e.what() << "\n";
				continue;
			}
		}
	}

	{
		std::lock_guard<std::mutex> lock(bufferSwitchMutex);
	    activeBufferIndex = inactiveBufferIndex;
	}
}

msg_plane_info Radar::getAircraftData(int id) {
	//Coen320_Lab (Task0): You need to correct the channel name
	//It is your group name + plane id

	std::string id_str = PLANE_CHANNEL_PREFIX + std::to_string(id);  // Convert integer id to string
	const char* ID = id_str.c_str();         // Convert string to const char*
	int plane_channel = name_open(ID, 0);

	if (plane_channel == -1) {
		throw std::runtime_error("Radar: Error occurred while attaching to channel");
	}

	// Prepare a message to request position data
	Message requestMsg;
	std::memset(&requestMsg, 0, sizeof(requestMsg));
	requestMsg.header = false;
	requestMsg.type = MessageType::REQUEST_POSITION;
	requestMsg.planeID = id;
	requestMsg.data = NULL;

	// Structure to hold the received position data
	Message receiveMessage;

	// Send the position request to the aircraft and receive the response
	if (MsgSend(plane_channel, &requestMsg, sizeof(requestMsg), &receiveMessage, sizeof(receiveMessage)) == -1) {
		name_close(plane_channel);
		throw std::runtime_error("Radar: Error occurred while sending request message to aircraft");
	}

	msg_plane_info received_info = *static_cast<msg_plane_info*>(receiveMessage.data);

	// Close the communication channel with the aircraft
	name_close(plane_channel);

	return received_info;
}

void Radar::addPlaneToAirspace(Message msg) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	int plane_data = msg.planeID;
    planesInAirspace.insert(plane_data);
    std::cout << "Plane " << msg.planeID << " added to airspace" << std::endl;
}

void Radar::removePlaneFromAirspace(int planeID) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	planesInAirspace.erase(planeID);  // Directly remove the integer from the list
	std::cout << "Plane " << planeID << " removed from airspace" << std::endl;
}


void Radar::writeToSharedMemory() {
	// COEN320 Lab4_5 Task 2.1
	/*
	You need to implement the shared memory writing process here
	Save SharedMemory structure to shared memory
	Make sure to use mutex to protect the buffer switching process (e.g., bufferSwitchMutex)
	To store aircraft data, use inactiveBuffer which is obtained from getActiveBuffer() method
	Check the code snippet below for reference
	at the end you need to unmap the shared memory and close the file descriptor
	e.g., munmap(<SharedMemory object, shared_mem>, SHARED_MEMORY_SIZE) and close(shm_fd)
	*/
	if (sharedMemPtr == nullptr) return;

	std::lock_guard<std::mutex> lock(bufferSwitchMutex);
	pthread_mutex_lock(&sharedMemPtr->mutex);

	// Get the active buffer based on the current active index
	std::vector<msg_plane_info>& activeBuffer = getActiveBuffer();

	size_t n = activeBuffer.size();
	if (n > MAX_PLANES) n = MAX_PLANES;

	if (n > 0) {
		std::memcpy(sharedMemPtr->plane_data, activeBuffer.data(), n * sizeof(msg_plane_info));
		sharedMemPtr->start = true;
	}
	sharedMemPtr->count = static_cast<int>(n);
	// Get the current timestamp
	sharedMemPtr->timestamp = tick_counter_ref;
	// Check if activeBuffer is empty and set the flag accordingly
	sharedMemPtr->is_empty.store(n == 0);
	pthread_mutex_unlock(&sharedMemPtr->mutex);

	// clean up
	activeBuffer.clear();
}


void Radar::clearSharedMemory() {
	// COEN320 Lab4_5 Task 2.2
	// You need to implement the shared memory clearing process here
	// Initialize the shared memory structure to an empty state
	// You need to clear plane_data, count, etc.
	if (sharedMemPtr == nullptr) return;

	std::lock_guard<std::mutex> lock(bufferSwitchMutex);
	pthread_mutex_lock(&sharedMemPtr->mutex);
	std::memset(sharedMemPtr->plane_data, 0, sizeof(sharedMemPtr->plane_data));  // Clear plane data
	sharedMemPtr->count = 0;  // No planes initially
	sharedMemPtr->start = false;
	sharedMemPtr->timestamp = tick_counter_ref;
	sharedMemPtr->is_empty.store(true);  // Set the is_empty flag to false to not block reader
	pthread_mutex_unlock(&sharedMemPtr->mutex);
	// Finally unmap the shared memory and close the file descriptor (done in cleanupSharedMemory)
}
