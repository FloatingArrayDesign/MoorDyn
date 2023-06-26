#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>

int
main(int, char**)
{
	// Path for input file:
	MoorDyn system = MoorDyn_Create("./Mooring/tripod.txt");

	if (!system) {
		std::cerr << "Failure Creating the mooring system\n";
		return 1;
	}

	int err;

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring initialization: " << err
		          << "\n";
		MoorDyn_Close(system);
		return 3;
	}

	double dt = 0.1;                 // Set time increment
	const unsigned int nts = 3 / dt; // Number of time steps

	for (unsigned int i = 0; i < nts; i++) {
		// This deals with the type mismatch between nts (int) and time
		// (double): May need to be cautious of rounding errors:
		double t = i * dt;
		std::cout << "Time: " << t << '\n';

		// At each time step, advance the system:
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure during the mooring step " << i << ": " << err
			          << "\n";
			MoorDyn_Close(system);
			return 4;
		}
	}

	std::cout << "Closing down system.\n";

	// Close the system and deallocate memory:
	err = MoorDyn_Close(system);
	return 0;
}
