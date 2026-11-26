#include "OperatorConsole.h"
#include "IpcHelpers.h"
#include <sstream>
#include <fstream>
#include <ctime>
#include <poll.h>
#include <unistd.h>

OperatorConsole::OperatorConsole() {
	printHelp();
	Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
	exit = true;   // the input loop checks this at least every 500 ms
	if (Operator_Console.joinable()) {
		Operator_Console.join();
	}
}

void OperatorConsole::printHelp() {
	std::cout <<
		"\n================ OPERATOR CONSOLE ================\n"
		" speed <id> <vx> <vy> <vz>   change velocity (heading)\n"
		" alt   <id> <z>              change altitude\n"
		" pos   <id> <x> <y> <z>      move aircraft\n"
		" info  <id>                  show aircraft details\n"
		" lookahead <seconds>         collision prediction window\n"
		" help                        show this list\n"
		" exit                        close the console\n"
		"==================================================\n" << std::endl;
}

void OperatorConsole::HandleConsoleInputs() {
	std::string line;
	while (!exit) {
		// Wait up to 500 ms for keyboard input instead of blocking forever in
		// getline(). That way the thread can notice 'exit' and shut down cleanly.
		struct pollfd pfd;
		pfd.fd = STDIN_FILENO;
		pfd.events = POLLIN;
		pfd.revents = 0;
		int ready = poll(&pfd, 1, 500);
		if (ready <= 0) continue;   // timeout (or signal): re-check 'exit'

		if (!std::getline(std::cin, line)) {   // EOF: stdin closed
			break;
		}
		if (!line.empty()) {
			handleLine(line);
		}
	}
}

void OperatorConsole::handleLine(const std::string& line) {
	std::istringstream in(line);
	std::string cmd;
	in >> cmd;

	Message_inter_process msg;
	bool parsed = false;

	if (cmd == "speed") {
		msg_change_heading h{};
		if (in >> h.ID >> h.VelocityX >> h.VelocityY >> h.VelocityZ) {
			buildMessage(msg, MessageType::REQUEST_CHANGE_OF_HEADING, h.ID, &h, sizeof(h));
			parsed = true;
		}
	} else if (cmd == "alt") {
		int id; msg_change_altitude a{};
		if (in >> id >> a.altitude) {
			if (a.altitude < AIRSPACE_Z_MIN || a.altitude > AIRSPACE_Z_MAX) {
				std::cout << "Note: altitude outside " << AIRSPACE_Z_MIN << "-" << AIRSPACE_Z_MAX
				          << "; the aircraft will leave the airspace.\n";
			}
			buildMessage(msg, MessageType::REQUEST_CHANGE_ALTITUDE, id, &a, sizeof(a));
			parsed = true;
		}
	} else if (cmd == "pos") {
		int id; msg_change_position p{};
		if (in >> id >> p.x >> p.y >> p.z) {
			buildMessage(msg, MessageType::REQUEST_CHANGE_POSITION, id, &p, sizeof(p));
			parsed = true;
		}
	} else if (cmd == "info") {
		int id;
		if (in >> id) {
			buildMessage(msg, MessageType::REQUEST_AUGMENTED_INFO, id);
			parsed = true;
		}
	} else if (cmd == "lookahead") {
		msg_time_constraint tc{};
		if (in >> tc.seconds) {
			buildMessage(msg, MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS, -1, &tc, sizeof(tc));
			parsed = true;
		}
	} else if (cmd == "help") {
		printHelp();
		return;
	} else if (cmd == "exit") {
		logCommand(line);
		std::cout << "Operator console closed (monitoring continues).\n";
		exit = true;
		return;
	} else {
		std::cout << "Unknown command '" << cmd << "'. Type 'help'.\n";
		return;
	}

	if (!parsed) {
		std::cout << "Bad arguments for '" << cmd << "'. Type 'help'.\n";
		return;
	}

	logCommand(line);
	int status = sendToServer(COMPUTER_CHANNEL_NAME, msg);   // blocks until ComputerSystem replies
	if (status == 0) {
		std::cout << "OK\n";
	} else {
		std::cout << "Command failed (ComputerSystem or aircraft not reachable).\n";
	}
}

void OperatorConsole::logCommand(const std::string& command) {
	std::ofstream log(OPERATOR_LOG_FILE, std::ios::app);
	if (!log) {
		perror("OperatorConsole: cannot open log file");
		return;
	}
	std::time_t now = std::time(nullptr);
	char stamp[32];
	std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
	log << "[" << stamp << "] " << command << "\n";
}
