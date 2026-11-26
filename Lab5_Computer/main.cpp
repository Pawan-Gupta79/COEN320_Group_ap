#include "ComputerSystem.h"
#include "OperatorConsole.h"
#include "CommunicationsSystem.h"

int main() {
    // Task 4: three cooperating parts in this process, each with its own thread(s):
    //   OperatorConsole --IPC--> ComputerSystem --IPC--> CommunicationsSystem --IPC--> Aircraft
    // Construction order matters: each part's channel must exist before the part
    // that sends to it starts. C++ destroys locals in REVERSE order, so shutdown
    // goes console -> computer -> comms, i.e. senders stop before receivers.
    CommunicationsSystem comms;
    ComputerSystem computerSystem;

    if (computerSystem.startMonitoring()) {
        OperatorConsole console;      // starts reading commands once monitoring is up
        computerSystem.joinThread();  // returns when the Radar says the simulation is over
    } else {
        std::cerr << "Failed to start monitoring." << std::endl;
    }

    std::cout << "Monitoring stopped. Exiting main." << std::endl;

    return 0;
}
