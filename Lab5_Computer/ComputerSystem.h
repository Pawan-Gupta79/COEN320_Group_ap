// Coen320 Task: Read this file and make the ComputerSystem.cpp file with the
// implementation of the functions declared in this header file
#ifndef COMPUTER_SYSTEM_H
#define COMPUTER_SYSTEM_H

#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <vector>
#include <set>
#include <utility>
#include <sys/dispatch.h>

const double CONSTRAINT_X = 3000;
const double CONSTRAINT_Y = 3000;
const double CONSTRAINT_Z = 1000;

#include "Msg_structs.h"  // Include the structure definition for msg_plane_info

class ComputerSystem {
public:
    ComputerSystem();
    ~ComputerSystem();

    bool startMonitoring();
    void joinThread();

private:
    void monitorAirspace();
    bool initializeSharedMemory();
    void cleanupSharedMemory();

    //Collsion detection
    void checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes);
    bool checkAxes(msg_plane_info plane1, msg_plane_info plane2);

    //Handle messages from operator
    void processMessage();
    int sendMessagesToComms(const Message_inter_process& msg);
    void handleTimeConstraintChange(const Message_inter_process& msg);
    void printAugmentedInfo(int planeID);
    bool sendCollisionToDisplay(const Message_inter_process& msg);
    void stopOperatorListener();

    std::atomic<int> timeConstraintCollisionFreq{180};

    std::set<std::pair<int,int>> lastCollisions;
    bool displayUpToDate = true;

    int shm_fd;
    SharedMemory* shared_mem;
    std::thread monitorThread;
    std::thread monitorOperatorInput;
    std::atomic<bool> running;

    name_attach_t* operatorChannel = nullptr;
    std::atomic<bool> listen{true};
};

#endif // COMPUTER_SYSTEM_H
