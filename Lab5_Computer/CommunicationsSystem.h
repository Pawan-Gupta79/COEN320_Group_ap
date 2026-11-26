#ifndef SRC_COMMUNICATIONSSYSTEM_H_
#define SRC_COMMUNICATIONSSYSTEM_H_

#include <iostream>
#include <thread>
#include <sys/dispatch.h>
#include "Msg_structs.h"

// Relays operator commands from the ComputerSystem to the right aircraft.
//   ComputerSystem --(COMMS_CHANNEL_NAME)--> CommunicationsSystem
//   CommunicationsSystem --("group_ap_plane_<id>")--> Aircraft
class CommunicationsSystem {
public:
	CommunicationsSystem();
	~CommunicationsSystem();
private:
    void HandleCommunications();
    int messageAircraft(const Message_inter_process& msg);
    std::thread Communications_System;
    name_attach_t* commsChannel = nullptr;
};


#endif /* SRC_COMMUNICATIONSSYSTEM_H_ */
