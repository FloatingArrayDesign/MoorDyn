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

TEST_CASE("Added mass")
{
	// Do the math for our expected acceleration given the mass and that
	// Ca = 1.0
	double length = 2.0;
	double mass = 100 * length;
	double r = 0.25;
	double V = length * M_PI * r * r;
	double rho_w = 1025;
	double g = 9.8;
	double buoyancy = rho_w * V * g;
	double weight = mass * g;
	double Fnet = buoyancy - weight;
	double added_mass = 1.0 * V * rho_w;
	double acceleration = Fnet / (mass + added_mass);

	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/rod_tests/AddedMass.txt");
	REQUIRE(system);

	err = MoorDyn_Init(system, NULL, NULL);
	REQUIRE(err == MOORDYN_SUCCESS);

	double t = 0, Tmax = 2.5, dt = 0.1;

	while (t < Tmax) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		REQUIRE(err == MOORDYN_SUCCESS);

		const auto rod = MoorDyn_GetRod(system, 1);

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		REQUIRE(err == MOORDYN_SUCCESS);

		double expected_z = -10 + 0.5 * acceleration * t * t;
		double pos[3];
		for (unsigned int i = 0; i < n_nodes; i++) {

			err = MoorDyn_GetRodNodePos(rod, i, pos);
			REQUIRE(err == MOORDYN_SUCCESS);
			REQUIRE(isclose(pos[2], expected_z, 1e-8, 1e-10));
		}
		// when the rod get higher than z = -1, stop simulating
		if (expected_z > -1.0) {
			break;
		}
	}

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}

/** @brief Pinned rod horizontally lying, which should float towards the
 * vertical direction.
 *
 * Since there is no damping forces, the rods will be oscillating from one side
 * to the other
 */
TEST_CASE("Pinned floating rod")
{
	int err;
	MoorDyn system = MoorDyn_Create("Mooring/RodPinnedFloating.txt");
	REQUIRE(system);

	err = MoorDyn_Init(system, NULL, NULL);
	REQUIRE(err == MOORDYN_SUCCESS);

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	REQUIRE(err == MOORDYN_SUCCESS);

	unsigned int n_rods;
	err = MoorDyn_GetNumberRods(system, &n_rods);
	for (unsigned int i_rod = 1; i_rod <= n_rods; i_rod++) {
		const auto rod = MoorDyn_GetRod(system, i_rod);
		REQUIRE(rod);

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		REQUIRE(err == MOORDYN_SUCCESS);
		// Check that the rod is floating upwards
		double pos[3];
		err = MoorDyn_GetRodNodePos(rod, 0, pos);
		REQUIRE(err == MOORDYN_SUCCESS);
		const double z0 = pos[2];
		err = MoorDyn_GetRodNodePos(rod, n_nodes, pos);
		REQUIRE(err == MOORDYN_SUCCESS);
		const double z1 = pos[2];
		REQUIRE(z1 >= z0);
	}

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}

TEST_CASE("Hanging rod")
{
	int err;
	MoorDyn system = MoorDyn_Create("Mooring/RodHanging.txt");
	REQUIRE(system);

	err = MoorDyn_Init(system, NULL, NULL);
	REQUIRE(err == MOORDYN_SUCCESS);

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	REQUIRE(err == MOORDYN_SUCCESS);

	const auto rod = MoorDyn_GetRod(system, 1);
	REQUIRE(rod);

	unsigned int n_nodes;
	err = MoorDyn_GetRodN(rod, &n_nodes);
	REQUIRE(err == MOORDYN_SUCCESS);

	// Check that the rod is is not rotating
	const double l = 20.0;
	double posa[3], posb[3];
	err = MoorDyn_GetRodNodePos(rod, 0, posa);
	REQUIRE(err == MOORDYN_SUCCESS);
	err = MoorDyn_GetRodNodePos(rod, n_nodes, posb);
	REQUIRE(err == MOORDYN_SUCCESS);

	REQUIRE((posa[0] + 0.5 * l) / l < TOL);
	REQUIRE((posb[0] - 0.5 * l) / l < TOL);

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}
