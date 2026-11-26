#include "Display.h"
#include <cstring>
#include <iostream>

// Usage:  Display            (colours + clear-screen; run in a real terminal, e.g. ssh/telnet)
//         Display --plain    (no escape codes; use this in the Momentics console)
int main(int argc, char* argv[]) {
	bool useAnsi = true;
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--plain") == 0) useAnsi = false;
	}

	Display display(useAnsi);
	if (!display.start()) {
		std::cerr << "Display failed to start." << std::endl;
		return 1;
	}
	display.join();   // returns when the Radar reports the simulation is over
	std::cout << "Display stopped." << std::endl;
	return 0;
}
