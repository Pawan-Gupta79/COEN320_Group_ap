#include <iostream>
#include <iomanip>
#include <memory>
#include <pthread.h>
#include <cstring>
#include <sys/iomsg.h>
#include "Aircraft.h"
#include "ATCTimer.h"


//Coen320_Lab (Task0): Radar Channel name should contain your group name
#define Radar RADAR_CHANNEL_NAME //attach point for AirTrafficControl
/*#define Display_ID "display" //attach point for AirTrafficControl // You need to set this for Part 2 (Display)*/

void* updatePositionThread(void* arg) {
    Aircraft* aircraft = static_cast<Aircraft*>(arg);
    return reinterpret_cast<void*>(aircraft->updatePosition());
}

// Constructor definition
Aircraft::Aircraft(int id, double x, double y, double z, double sx, double sy, double sz, int t)
    : id(id), posX(x), posY(y), posZ(z), speedX(sx), speedY(sy), speedZ(sz), arrivalTime(t), inAirspace(true) {
	message_id = -1;
	Radar_id = -1;
	airspace = {0, 100000, 0, 100000, 15000, 40000};
	// Coen320_lab3(Task1): You need to create a thread worker
	// Worker function: updatePositionThread
	// Worker function parameters: (void*)this
	// Thread handler id: thread_id
	//Note: you need to verify if the thread creation is successful, otherwise print the error
	if (pthread_create(&thread_id, NULL, updatePositionThread, (void*)this) !=0)
	{
		perror("Failed to create aircraft thread");
	}

}

Aircraft::~Aircraft(){};

//Print current Aircraft data
void Aircraft::printInitialAircraftData() const {
    std::cout << std::left 	<< std::setw(5) << id
    						<< std::setw(5) << arrivalTime
							<< std::setw(5) << posX
							<< std::setw(5) << posY
							<< std::setw(5) << posZ
							<< std::setw(5) << speedX
							<< std::setw(5) << speedY
							<< std::setw(5) << speedZ
							<< "\n";
}


void Aircraft::changeHeading(double Vx, double Vy, double Vz){
	speedX = Vx;
	speedY = Vy;
	speedZ = Vz;
}

void Aircraft::handleCommand(int rcvid, const Message_inter_process& msg) {
	int status = 0;
	switch (msg.type) {
	case MessageType::REQUEST_CHANGE_OF_HEADING: {
		msg_change_heading h;
		std::memcpy(&h, msg.data.data(), sizeof(h));
		changeHeading(h.VelocityX, h.VelocityY, h.VelocityZ);
		std::cout << "Aircraft " << id << ": new velocity (" << speedX << ", "
		          << speedY << ", " << speedZ << ")\n";
		break;
	}
	case MessageType::REQUEST_CHANGE_POSITION: {
		msg_change_position p;
		std::memcpy(&p, msg.data.data(), sizeof(p));
		posX = p.x;
		posY = p.y;
		posZ = p.z;
		std::cout << "Aircraft " << id << ": moved to (" << posX << ", "
		          << posY << ", " << posZ << ")\n";
		break;
	}
	case MessageType::REQUEST_CHANGE_ALTITUDE: {
		msg_change_altitude a;
		std::memcpy(&a, msg.data.data(), sizeof(a));
		posZ = a.altitude;
		std::cout << "Aircraft " << id << ": new altitude " << posZ << "\n";
		break;
	}
	default:
		std::cerr << "Aircraft " << id << ": unknown command type "
		          << static_cast<int>(msg.type) << "\n";
		status = -1;
		break;
	}
	MsgReply(rcvid, EOK, &status, sizeof(status));
}


int Aircraft::updatePosition() {
    ATCTimer timer(1, 0);
    int currentTime = 0;  // Variable to track current time

    // Wait until the arrival time has passed
    while (currentTime < arrivalTime) {
        timer.waitTimer();  // Wait for 1 second
        ++currentTime;
    }

    //********SEND ENTER AIRSPACE TO RADAR**************
    //Coen320_Lab3(Task5): We are using message passing. Learn how to open a channel with Radar module
    // Open channel with radar and verify if the channel opened successfully
    //Use the function name_open with the radar channel name and parameter 0

    if ((Radar_id = name_open(Radar, 0)) == -1) {
		perror("Error occurred while creating the channel with Radar");
		return EXIT_FAILURE;
	}

    //Coen320_Lab3(Task6): Once the arrival time is reached, send the ENTER_AIRSPACE message
    //Read and learn how we create a message
    Message enterAirspaceMessage = createEnterAirspaceMessage(id);

    // Send message
    //Coen320_Lab3(Task7): send the message Using the MsgSend function to Radar_id channel
    //Parameters:
    //Channel ID (e.g. Radar_id)
    //Message address
    // size of the message
    //Respond Message address (put zero)
    //Respond Message size (put zero)
    //For more information use ctrl+space
    //answer: MsgSend(Radar_id, &createEnterAirspaceMessage, sizeof(createEnterAirspaceMessage),0,0)

    if (MsgSend(Radar_id, &enterAirspaceMessage, sizeof(enterAirspaceMessage),0,0) == -1) {
            std::cout << "Failed to send enter message to Radar!\n";
            return EXIT_FAILURE;
	}

    //********SEND UPDATE POSITION TO RADAR**************
    //Coen320_Lab (Task0): Create channel to be reachable by radar that wants to poll the Airplane
    //To chose the polling channel concatenate your group name with the plane id
    //Note: It is critical to not interfere other groups
    std::string id_str = PLANE_CHANNEL_PREFIX + std::to_string(id);  // Convert integer id to string
    const char* ID = id_str.c_str();         // Convert string to const char*
    name_attach_t* Plane_channel = name_attach(NULL, ID, 0); // For server

    if (Plane_channel == NULL) {
        std::cerr << "Could not attach plane ID: " << ID << " to channel\n";
        return EXIT_FAILURE;
    }

    // Start the position update loop
    while (true) {
        // Update position based on velocity
        posX += speedX;
        posY += speedY;
        posZ += speedZ;

        // Debug: Print the new position
        //std::cout << "Updated Position: (" << posX << ", " << posY << ", " << posZ << ")\n";

        // Check if the plane is still within airspace boundaries
        if (posX < airspace.lower_x_boundary || posX > airspace.upper_x_boundary ||
            posY < airspace.lower_y_boundary || posY > airspace.upper_y_boundary ||
            posZ < airspace.lower_z_boundary || posZ > airspace.upper_z_boundary) {
            // Send exit airspace message and exit loop if out of bounds
            Message exitAirspaceMessage = createExitAirspaceMessage(id);
            if (MsgSend(Radar_id, &exitAirspaceMessage, sizeof(exitAirspaceMessage), 0, 0) == -1) {
                std::cout << "Failed to send exit message to Radar!\n";
                return EXIT_FAILURE;
            }
            break;  // Exit the loop if out of bounds
        }

        // Check for incoming position update requests from Radar
        bool gotOurMessage = false;
        while (!gotOurMessage) {
            alignas(8) char buffer[sizeof(Message_inter_process)];  // Buffer to handle largest message size
            int rcvid = MsgReceive(Plane_channel->chid, buffer, sizeof(buffer), NULL);

            if (rcvid == -1) {
                perror("Aircraft: MsgReceive failed");
                break;
            }
            if (rcvid == 0) {  // pulse
                struct _pulse* pulse = reinterpret_cast<struct _pulse*>(buffer);
                if (pulse->code == _PULSE_CODE_DISCONNECT) {
                    ConnectDetach(pulse->scoid);
                }
                continue;
            }

            bool isInterProcess = buffer[0] & 0x01;
            uint16_t systemType;
            std::memcpy(&systemType, buffer, sizeof(systemType));
            if (!isInterProcess && systemType == _IO_CONNECT) {  // name_open() from a client
                MsgReply(rcvid, EOK, NULL, 0);
                continue;
            }
            gotOurMessage = true;

            // Radar requested position data
            if (isInterProcess){  //sporadic
            	// Message is of type Message_inter_process
            	// Handle different message types using switch
                // COEN320 Lab 4_5: You need to handle different message types here
                // These commands come from Communication System
            	Message_inter_process receivedMsg;
            	std::memcpy(&receivedMsg, buffer, sizeof(receivedMsg));
            	handleCommand(rcvid, receivedMsg);
            } else {  //periodic
            	// Message is of type Message
            	Message* receivedMsg = reinterpret_cast<Message*>(buffer);

            	if (receivedMsg->type == MessageType::REQUEST_POSITION) {
            		msg_plane_info positionData = {id, posX, posY, posZ, speedX, speedY, speedZ};
            	    Message posUpdateMessage = createPositionUpdateMessage(id, positionData);

            	    MsgReply(rcvid, 0, &posUpdateMessage, sizeof(posUpdateMessage)); // Send reply with position
            	} else {
            		MsgError(rcvid, ENOSYS);
            	}
            }
        }

        // Wait for the next time step
        timer.waitTimer();
    }

    name_detach(Plane_channel, 0);
    name_close(Radar_id);
    pthread_exit(NULL);

    return 0;
}


int Aircraft::getArrivalTime() {
	return arrivalTime;
}

int Aircraft::getID(){
	return id;
}

//Coen320_Lab3 (Task2): look at the message creation example here
Message Aircraft::createEnterAirspaceMessage(int planeID){
	Message msg;
	msg.header = false;
	msg.type = MessageType::ENTER_AIRSPACE;
	msg.planeID = planeID;
	msg.data = 	NULL;  // Allocate dynamically and copy info data
	return msg;
}

//Coen320_Lab3(Task3): complete the createPositionUpdateMessage function with what you learned above
Message Aircraft::createExitAirspaceMessage(int planeID){

	Message msg;
	msg.header = false;
	msg.type = MessageType::EXIT_AIRSPACE;// Use the correct Message type
	msg.planeID = planeID;// Use the passed Plane ID
	msg.data = NULL;

	return msg;
}
//Coen320_Lab3(Task4): complete the createPositionUpdateMessage function with what you learned above
Message Aircraft::createPositionUpdateMessage(int planeID, const msg_plane_info& info) {

    Message msg;
    msg.header = false;
    msg.type = MessageType::POSITION_UPDATE; // Use the correct Message type
    msg.planeID = planeID;// Use the passed Plane ID
    msg.data = (void*)&info;  // Allocate and copy info data

    return msg;

}


