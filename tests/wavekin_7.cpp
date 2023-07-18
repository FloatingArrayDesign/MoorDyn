/** @file wavekin_7.cpp
 * A test case with component summing wave generation based on a
 * wave_frequencies.txt file
 */
#include "MoorDyn2.hpp"
#include "MoorDynAPI.h"
#include <iostream>
#include <algorithm>
#include "util.h"
using namespace std;

/** @brief Runs a simulation
 *
 * @return true if the test is passed, false if problems are detected
 */
bool
api()
{
	moordyn::MoorDyn system("Mooring/wavekin_7/wavekin_7.txt");

	unsigned int n_dof;
	n_dof = system.NCoupledDOF();
	if (n_dof != 3) {
		cerr << "3x1 = 3 DOFs were expected, but " << n_dof << "were reported"
		     << endl;

		// MoorDyn_Close(system);
		return false;
	}

	// int err;
	double x[3], dx[3];
	// Set the fairlead points, as they are in the config file
	std::fill(x, x + 3, 0.0);
	std::fill(dx, dx + 3, 0.0);
	auto err = system.Init(x, dx);
	if (err != MOORDYN_SUCCESS) {
		cout << "failed to init fft_waves" << endl;
		return false;
	}

	// Integrate in time
	const double t_max = 35;
	double t = 0.0, dt = 0.1;
	double f[3];

	while (t < t_max) {
		system.Step(x, dx, f, t, dt);

		std::cout << "Time: " << t << std::endl;
	}

	return true;
}

/** @brief Runs all the test
 * @return 0 if the tests have ran just fine, 1 otherwise
 */
int
main(int, char**)
{
	if (!api())
		return 1;

	return 0;
}
