#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include <atomic>
#include <string>
#include "Msg_structs.h"

#define OPERATOR_LOG_FILE "group_ap_operator_log.txt"

class OperatorConsole {
public:
	OperatorConsole();
    ~OperatorConsole();

private:
    void HandleConsoleInputs();
    void handleLine(const std::string& line);
    void printHelp();
    void logCommand(const std::string& command);
    std::thread Operator_Console;
    std::atomic<bool> exit{false};
};



#endif /* OPERATORCONSOLE_H_ */
