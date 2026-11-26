#include "ComputerSystem.h"
#include "OperatorConsole.h"
#include "CommunicationsSystem.h"

int main() {
    // Task 4 (You need to first implement Task 3)
    /*
    You need to implement OperatorConsole to send commands to Aircraft
    You may make another class and read user commands to adjust the aircraft data in case of collision.
    You may use Message_inter_process with MessageType to communicate with Aircrafts:
    MessageType::REQUEST_CHANGE_OF_HEADING, MessageType::REQUEST_CHANGE_POSITION, MessageType::REQUEST_CHANGE_ALTITUDE
    // check OperatorConsole.h and CommunicationsSystem.h for a template
    */
    CommunicationsSystem comms;
    ComputerSystem computerSystem;

    if (computerSystem.startMonitoring()) {
        OperatorConsole console;
        computerSystem.joinThread();
    } else {
            std::cerr << "Failed to start monitoring." << std::endl;
        }

    std::cout << "Monitoring stopped. Exiting main." << std::endl;

    return 0;
}
