/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file wave_kin.cpp
 * A simple driver program that will run MoorDyn VERSION 2 testing several
 * wave kinematics
 */
#include "MoorDyn2.h"
#include "MoorDyn2.hpp"
#include "Waves.hpp"
#include "util.h"
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
	const double pi = 3.1416, g = 9.80665, A = 1.5, L = 10.0, T = 15.0,
	             H = 50.0;
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
api(const char* input_file, void (*cb)(double, const double*, double*, double*))
{
	MoorDyn system = MoorDyn_Create(input_file);
	// MoorDyn system = MoorDyn_Create("Mooring/wavekin_1.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 3) {
		cerr << "3x1 = 3 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[3], dx[3];
	// Set the fairlead points, as they are in the config file
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
		err = MoorDyn_ExternalWaveKinGetCoordinates(system, r);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure getting the wave kinematics nodes: " << err
			     << endl;
			return false;
		}

		for (unsigned int i = 0; i < nwp; i++) {
			(*cb)(t, r + 3 * i, u + 3 * i, du + 3 * i);
		}
		err = MoorDyn_ExternalWaveKinSet(system, u, du, t);
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
tabulated(const char* input_file)
{
	MoorDyn system = MoorDyn_Create(input_file);
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	double *x = NULL, *dx = NULL;
	if (n_dof) {
		x = new double[n_dof];
		std::fill(x, x + n_dof, 0.0);
		dx = new double[n_dof];
		std::fill(dx, dx + n_dof, 0.0);
	}

	int err;
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	// Integrate in time
	const double t_max = 10.0;
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

	cout << "Finished stepping forward to time " << t << endl;

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
	if (!api("Mooring/wavekin_current_1.txt", &current))
		return 1;
	if (!api("Mooring/wavekin_wave_1.txt", &wave))
		return 1;

	/**
	 * Some of these tests are commented out mainly because it's a lot to run
	 * and they're just for testing that waves and currents combine correctly
	 */
	// WAVE_GRID + Steady Currents
	if (!tabulated("Mooring/wavekin_2/wavekin_3.txt"))
		return 2;
	// WAVE_GRID + Dynamic Currents
	// if (!tabulated("Mooring/wavekin_2/wavekin_3_curr2.txt"))
	// 	return 2;
	// WAVE_GRID + 4D Current Grid
	// if (!tabulated("Mooring/wavekin_2/wavekin_3_curr5.txt"))
	// 	return 2;
	// WAVE_FFT_GRID + Steady Currents
	if (!tabulated("Mooring/wavekin_2/wavekin_2.txt"))
		return 2;
	// WAVE_FFT_GRID + Dynamic Currents
	// if (!tabulated("Mooring/wavekin_2/wavekin_2_curr2.txt"))
	// 	return 2;
	// WAVE_FFT_GRID + 4D Current Grid
	// if (!tabulated("Mooring/wavekin_2/wavekin_2_curr5.txt"))
	// 	return 2;

	if (!tabulated("Mooring/wavekin_3/test_dynamic_currents.txt"))
		return 2;

	return 0;
}
