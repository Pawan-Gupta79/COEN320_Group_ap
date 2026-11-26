#include "CommunicationsSystem.h"
#include "IpcHelpers.h"
#include <string>

CommunicationsSystem::CommunicationsSystem() {
	// Attach BEFORE starting the thread so the channel exists as soon as the
	// object is constructed (ComputerSystem may send right away).
	commsChannel = name_attach(NULL, COMMS_CHANNEL_NAME, 0);
	if (commsChannel == NULL) {
		perror("CommunicationsSystem: name_attach failed");
		return;
	}
	Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
	if (Communications_System.joinable()) {
		// Unblock our own MsgReceive with an EXIT message, then wait for the thread
		Message_inter_process exitMsg;
		buildMessage(exitMsg, MessageType::EXIT, -1);
		sendToServer(COMMS_CHANNEL_NAME, exitMsg);
		Communications_System.join();
	}
	if (commsChannel != nullptr) {
		name_detach(commsChannel, 0);
	}
}

void CommunicationsSystem::HandleCommunications() {
	while (true) {
		IpcBuffer buf;
		int rcvid = MsgReceive(commsChannel->chid, &buf, sizeof(buf), NULL);
		if (rcvid == -1) {
			perror("CommunicationsSystem: MsgReceive failed");
			break;
		}
		if (handleSystemMessage(rcvid, buf)) continue;

		int status;
		if (buf.msg.type == MessageType::EXIT) {
			status = 0;
			MsgReply(rcvid, EOK, &status, sizeof(status));
			break;
		}
		status = messageAircraft(buf.msg);
		MsgReply(rcvid, EOK, &status, sizeof(status));  // pass the aircraft's answer back
	}
}

// Forward the command unchanged (header == 1 tells the aircraft it is a command,
// not the Radar's periodic position request).
int CommunicationsSystem::messageAircraft(const Message_inter_process& msg) {
	std::string planeChannel = PLANE_CHANNEL_PREFIX + std::to_string(msg.planeID);
	int status = sendToServer(planeChannel.c_str(), msg);
	if (status == -1) {
		std::cerr << "CommunicationsSystem: aircraft " << msg.planeID
		          << " not reachable (not in airspace?)\n";
	} else {
		std::cout << "CommunicationsSystem: command delivered to aircraft " << msg.planeID << "\n";
	}
	return status;
}
