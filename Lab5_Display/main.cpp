#include "Display.h"
#include <cstring>
#include <iostream>

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
	display.join();
	std::cout << "Display stopped." << std::endl;
	return 0;
}
