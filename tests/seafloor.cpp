/** @file seafloor.cpp
 * Pendulum swining into the seafloor to validate 3D seafloor
 */

// Visual studio still uses this
#define _USE_MATH_DEFINES

#include "MoorDyn2.h"
#include "Seafloor.h"
#include "Waves.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <array>

#include "util.h"
#define TOL 1.0e-1

bool
compare(double v1, double v2, double tol)
{
	return fabs(v1 - v2) <= tol;
}

#define CHECK_VALUE(name, v1, v2, tol, t)                                      \
	if (!compare(v1, v2, tol)) {                                               \
		cerr << setprecision(8) << "Checking " << name                         \
		     << " failed at t = " << t << " s. " << v1 << " was expected"      \
		     << " but " << v2 << " was computed" << endl;                      \
		MoorDyn_Close(system);                                                 \
		return false;                                                          \
	}

using namespace std;

/** @brief Pendulum motion into the seafloor
 * @return true if the test worked, false otherwise
 */
bool
pendulum()
{
	MoorDyn system = MoorDyn_Create("Mooring/3D_seafloor/hanging_lines.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 0) {
		cerr << "0 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double pos[3];
	const auto point = MoorDyn_GetPoint(system, 2);
	err = MoorDyn_GetPointPos(point, pos);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting the mass position: " << err << endl;
		return false;
	}
	const double x0 = pos[0];
	const double z0 = pos[2];
	const double l0 = sqrt(x0 * x0 + z0 * z0);
	cout << l0 << endl;

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	MoorDynSeafloor seafloor = MoorDyn_GetSeafloor(system);
	if (!seafloor) {
		cerr << "Could not get seafloor instance" << endl;
		return false;
	}

	double avgDepth, minDepth;
	err = MoorDyn_GetMinDepth(seafloor, &minDepth);
	if (err != MOORDYN_SUCCESS) {
		cerr << "couldn't get min depth" << endl;
		return false;
	}

	err = MoorDyn_GetAverageDepth(seafloor, &avgDepth);
	if (err != MOORDYN_SUCCESS) {
		cerr << "couldn't get min depth" << endl;
		return false;
	}

	cout << "Average depth is " << avgDepth << ", Minimum depth is " << minDepth
	     << endl;

	MoorDynWaves waves = MoorDyn_GetWaves(system);
	double x_min = -50.0;
	double x_max = 50;
	double x_step = 1.0;

	double z_min = -50.0;
	double z_max = 5.0;
	double z_step = 1.0;

	const double w = sqrt(9.80665 / l0);
	const double T = 2.0 * M_PI / w;
	double dt = T / 20.0;
	double t = 0.0;
	std::vector<double> x_peaks = { x0 };
	std::vector<double> t_peaks = { 0.0 };
	double d_peak = 0.0;
	while (t < 1.0 * T) {
		cout << t << " / " << 1.0 * T << '\n';
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
		}
		err = MoorDyn_GetPointPos(point, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the mass position: " << err << endl;
			return false;
		}
		const double x = pos[0];
		const double z = pos[2];
		const double l = sqrt(x * x + z * z);
		// CHECK_VALUE("L", l0, l, 0.01 * l0, t);
		const double d = fabs(x - x_peaks.back());
		if (d > d_peak)
			d_peak = d;
		else {
			x_peaks.push_back(x);
			t_peaks.push_back(t);
			d_peak = 0.0;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

/** @brief Runs all the test
 * @return 0 if the tests have ran just fine. The index of the failing test
 * otherwise
 */
int
main(int, char**)
{
	if (!pendulum()) {
		cout << "seafloor test failed" << endl;
		return 1;
	}
	return 0;
}
