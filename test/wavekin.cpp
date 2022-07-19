/*
 * Copyright (c) 2022 Matt Hall <mtjhall@alumni.uvic.ca> and Jose Luis
 * Cercos-Pita <jlc@core-marine.com>
 *
 * This file is part of MoorDyn.  MoorDyn is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * MoorDyn is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MoorDyn.  If not, see <http://www.gnu.org/licenses/>.
 */

/** @file wave_kin.cpp
 * A simple driver program that will run MoorDyn VERSION 2 testing several
 * wave kinematics
 */
#include "MoorDyn2.h"
#include <string.h>
#include <math.h>
#include <iostream>
#include <algorithm>

using namespace std;

/** Constant underwater current
 * @param t Simulation time
 * @param r Point where the kinematics shall be evaluated
 * @param u Velocity
 * @param du Acceleration
 */
void
current(double PARAM_UNUSED t,
        const double PARAM_UNUSED* r,
        double* u,
        double* du)
{
	memset(u, 0.0, 3 * sizeof(double));
	memset(du, 0.0, 3 * sizeof(double));
	u[0] = 1.0;
}

/** Regular wave
 * @param t Simulation time
 * @param r Point where the kinematics shall be evaluated
 * @param u Velocity
 * @param du Acceleration
 */
void
wave(double t, const double* r, double* u, double* du)
{
	const double pi = 3.1416, g = 9.81, A = 1.5, L = 10.0, T = 15.0, H = 50.0;
	memset(u, 0.0, 3 * sizeof(double));
	memset(du, 0.0, 3 * sizeof(double));
	const double k = 2.0 * L / pi, w = 2.0 * T / pi, zf = (1.0 + r[2] / H);
	u[0] = zf * A * g * k / w * cos(k * r[0] - w * t);
	u[1] = zf * A * w * sin(k * r[0] - w * t);
	du[0] = zf * A * g * k * sin(k * r[0] - w * t);
	du[1] = -zf * A * w * w * cos(k * r[0] - w * t);
}

/** @brief Runs a simulation
 *
 * The water kinematics is set using the API, i.e. MoorDyn_InitExtWaves(),
 * MoorDyn_GetWavesCoords() and MoorDyn_SetWaves()
 * @param cb The callback function called to get the wave kinematics at a
 *           certain position and time
 * @return true if the test is passed, false if problems are detected
 */
bool
api(void (*cb)(double, const double*, double*, double*))
{
	MoorDyn system = MoorDyn_Create("Mooring/wavekin_1.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	const unsigned int n_dof = MoorDyn_NCoupledDOF(system);
	if (n_dof != 3) {
		cerr << "3x1 = 3 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[3], dx[3];
	// Set the fairlead connections, as they are in the config file
	std::fill(x, x + 3, 0.0);
	std::fill(dx, dx + 3, 0.0);
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	unsigned int nwp;
	err = MoorDyn_ExternalWaveKinInit(system, &nwp);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure during the wave kinematics initialization: " << err
		     << endl;
		return false;
	}

	double* r = new double[3 * nwp];
	double* u = new double[3 * nwp];
	double* du = new double[3 * nwp];
	if (!r || !u || !du) {
		MoorDyn_Close(system);
		cerr << "Failure allocating " << 3 * 3 * nwp * sizeof(double)
		     << " bytes" << endl;
		return false;
	}

	// Integrate in time
	const double t_max = 30.0;
	double t = 0.0, dt = 0.1;
	double f[3];
	while (t < t_max) {
		err = MoorDyn_GetWaveKinCoordinates(system, r);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure getting the wave kinematics nodes: " << err
			     << endl;
			return false;
		}

		for (unsigned int i = 0; i < nwp; i++) {
			(*cb)(t, r + 3 * i, u + 3 * i, du + 3 * i);
		}
		err = MoorDyn_SetWaveKin(system, u, du, t);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure setting the wave kinematics: " << err << endl;
			return false;
		}

		err = MoorDyn_Step(system, x, dx, f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
		}
	}

	delete[] r;
	delete[] u;
	delete[] du;

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

/** @brief Runs a simulation
 *
 * The water kinematics is set using the Waves instance, i.e. moordyn::Waves
 * @return true if the test is passed, false if problems are detected
 */
bool
grid()
{
	MoorDyn system = MoorDyn_Create("Mooring/wavekin_2/wavekin_2.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	const unsigned int n_dof = MoorDyn_NCoupledDOF(system);
	if (n_dof != 3) {
		cerr << "3x1 = 3 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[3], dx[3];
	// Set the fairlead connections, as they are in the config file
	std::fill(x, x + 3, 0.0);
	std::fill(dx, dx + 3, 0.0);
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	// Integrate in time
	const double t_max = 30.0;
	double t = 0.0, dt = 0.1;
	double f[3];
	while (t < t_max) {
		err = MoorDyn_Step(system, x, dx, f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
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
 * @return 0 if the tests have ran just fine, 1 otherwise
 */
int
main(int, char**)
{
	if (!api(&current))
		return 1;
	if (!api(&wave))
		return 1;
	if (!grid())
		return 2;

	return 0;
}
