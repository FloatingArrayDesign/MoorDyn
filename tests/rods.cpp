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

/** @file rods.cpp
 * Minimal tests for rods in different situations
 */

#define _USE_MATH_DEFINES

#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include "util.h"

#define TOL 1e-2

using namespace std;

bool
added_mass()
{
	// Do the math for our expected acceleration given the mass and that
	// Ca = 1.0
	double length = 2.0;
	double mass = 100 * length;
	double r = 0.25;
	double V = length * M_PI * r * r;
	double rho_w = 1025;
	double g = 9.80665;
	double buoyancy = rho_w * V * g;
	double weight = mass * g;
	double Fnet = buoyancy - weight;
	double added_mass = 1.0 * V * rho_w;
	double acceleration = Fnet / (mass + added_mass);

	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/rod_tests/AddedMass.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, Tmax = 2.5, dt = 0.1;

	while (t < Tmax) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}

		const auto rod = MoorDyn_GetRod(system, 1);

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the number of nodes for rod " << 1 << ": "
			     << err << endl;
			return false;
		}

		double expected_z = -10 + 0.5 * acceleration * t * t;
		double pos[3];
		for (unsigned int i = 0; i < n_nodes; i++) {

			err = MoorDyn_GetRodNodePos(rod, i, pos);
			if (err != MOORDYN_SUCCESS) {
				cerr << "Failure getting the position of nodes " << i
				     << " for rod 1"
				     << ": " << err << endl;
				return false;
			}
			if (!isclose(pos[2], expected_z, 1e-8, 1e-10)) {
				cerr << "Node " << i << " of Rod 1 should have a z position of "
				     << expected_z << " but has a z pos of " << pos[2]
				     << " at t=" << t << endl;
				return false;
			}
		}
		// when the rod get higher than z = -1, stop simulating
		if (expected_z > -1.0) {
			break;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

/** @brief Pinned rod horizontally lying, which should float towards the
 * vertical direction.
 *
 * Since there is no damping forces, the rods will be oscillating from one side
 * to the other
 */
bool
pinned_floating()
{
	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/RodPinnedFloating.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	unsigned int n_rods;
	err = MoorDyn_GetNumberRods(system, &n_rods);
	for (unsigned int i_rod = 1; i_rod <= n_rods; i_rod++) {
		const auto rod = MoorDyn_GetRod(system, i_rod);
		if (!rod) {
			cerr << "Failure getting the rod " << i_rod << endl;
			return false;
		}

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the number of nodes for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		// Check that the rod is floating upwards
		double pos[3];
		err = MoorDyn_GetRodNodePos(rod, 0, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting first node position for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		cout << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
		const double z0 = pos[2];
		err = MoorDyn_GetRodNodePos(rod, n_nodes, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting last node position for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		const double z1 = pos[2];
		cout << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;

		if (z1 < z0) {
			cerr << "The last node is below the first one for rod " << i_rod
			     << ": " << z1 << " vs. " << z0 << endl;
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

/** @brief Rod hanging from 2 identical ropes, which should move horizontally
 */
bool
hanging()
{
	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/RodHanging.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	const auto rod = MoorDyn_GetRod(system, 1);
	if (!rod) {
		cerr << "Failure getting the rod" << endl;
		return false;
	}

	unsigned int n_nodes;
	err = MoorDyn_GetRodN(rod, &n_nodes);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting the number of nodes: " << err << endl;
		return false;
	}
	// Check that the rod is is not rotating
	const double l = 20.0;
	double posa[3], posb[3];
	err = MoorDyn_GetRodNodePos(rod, 0, posa);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting first node position: " << err << endl;
		return false;
	}
	cout << posa[0] << ", " << posa[1] << ", " << posa[2] << endl;
	err = MoorDyn_GetRodNodePos(rod, n_nodes, posb);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting last node position: " << err << endl;
		return false;
	}
	cout << posb[0] << ", " << posb[1] << ", " << posb[2] << endl;

	if (((posa[0] + 0.5 * l) / l > TOL) || ((posb[0] - 0.5 * l) / l > TOL) ||
	    (fabs(posa[1]) / l > TOL) || (fabs(posb[1]) / l > TOL) ||
	    (fabs(posa[2] - posb[2]) / l > TOL)) {
		cerr << "The rod is rotating" << endl;
		return false;
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
	if (!pinned_floating())
		return 1;
	if (!hanging())
		return 2;
	if (!added_mass())
		return 3;

	cout << "rods.cpp passed successfully" << endl;
	return 0;
}
