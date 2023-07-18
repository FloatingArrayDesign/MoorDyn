#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>

/** @brief Check that we can initialize a mooring system with Beta_n
 * and Beta_t defined in the input file and run a step.
 * @return true if the test passed
 */
bool
test_4d_currents()
{
	MoorDyn system = MoorDyn_Create("Mooring/wavekin_4/hanging_lines.txt");
	if (!system) {
		std::cerr << "Failure Creating the Mooring system" << std::endl;
		return false;
	}

	int err;
	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		std::cerr << "Failure during the mooring initialization: " << err
		          << std::endl;
		return false;
	}

	// Integrate in time
	const double t_max = 15.0;
	double t = 0.0, dt = 0.1;
	while (t < t_max) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			std::cerr << "Failure during the mooring step: " << err
			          << std::endl;
			return false;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure closing Moordyn: " << err << std::endl;
		return false;
	}

	return true;
}

/** @brief Run all tests
 * @return 0 if the tests ran fine. Index of failing test otherwise.
 */
int
main(int, char**)
{
	if (!test_4d_currents())
		return 1;

	return 0;
}
