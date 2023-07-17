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

/** @file minimal.cpp
 * Minimal tests that only checks the library is correctly initialized,
 * running and closing
 */

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include "util.h"

#define TOL 1.0e-6
#define COG_Z -35.0
#define VX 0.01
#define VRX 0.01 * M_PI / 180.0

using namespace std;

TEST_CASE("Minimal body with attached entities")
{
	MoorDyn system = MoorDyn_Create("Mooring/lines_body.txt");
	REQUIRE(system);

	int err;
	unsigned int n_dof;
	err = MoorDyn_NCoupledDOF(system, &n_dof);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 6);

	double x[6], dx[6];
	// Get the initial position of the body
	auto body = MoorDyn_GetBody(system, 1);
	err = MoorDyn_GetBodyState(body, x, dx);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(isclose(x[0], 0));
	REQUIRE(isclose(x[1], 0));
	REQUIRE(isclose(x[2], COG_Z));
	REQUIRE(isclose(x[3], 0));
	REQUIRE(isclose(x[4], 0));
	REQUIRE(isclose(x[5], 0));

	// Get the initial position of the fairleads
	double xf_ref[9];
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf_ref + 3 * i);
		REQUIRE(err == MOORDYN_SUCCESS);
	}
	err = MoorDyn_Init(system, x, dx);
	REQUIRE(err == MOORDYN_SUCCESS);

	// Check that the fairleads have not moved
	double xf[9];
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		REQUIRE(err == MOORDYN_SUCCESS);
		REQUIRE(isclose(xf[3 * i], xf_ref[3 * i]));
		REQUIRE(isclose(xf[3 * i + 1], xf_ref[3 * i + 1]));
		REQUIRE(isclose(xf[3 * i + 2], xf_ref[3 * i + 2]));
	}

	// Let it run for a little while without moving
	double f[9];
	double t = 0.0, dt = 0.5;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	REQUIRE(err == MOORDYN_SUCCESS);

	// Check that the fairleads have not moved again
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		REQUIRE(err == MOORDYN_SUCCESS);
		REQUIRE(isclose(xf[3 * i], xf_ref[3 * i]));
		REQUIRE(isclose(xf[3 * i + 1], xf_ref[3 * i + 1]));
		REQUIRE(isclose(xf[3 * i + 2], xf_ref[3 * i + 2]));
	}

	// Let move it forward a little
	t += dt;
	dx[0] = VX;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	REQUIRE(err == MOORDYN_SUCCESS);

	// Check the new position and velocity of the body
	double x_new[6], dx_new[6];
	err = MoorDyn_GetBodyState(body, x_new, dx_new);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(isclose(x_new[0], VX * dt));
	REQUIRE(isclose(x_new[1], 0));
	REQUIRE(isclose(x_new[2], COG_Z));
	REQUIRE(isclose(x_new[3], 0));
	REQUIRE(isclose(x_new[4], 0));
	REQUIRE(isclose(x_new[5], 0));
	REQUIRE(isclose(dx_new[0], VX));
	REQUIRE(isclose(dx_new[1], 0));
	REQUIRE(isclose(dx_new[2], 0));
	REQUIRE(isclose(dx_new[3], 0));
	REQUIRE(isclose(dx_new[4], 0));
	REQUIRE(isclose(dx_new[5], 0));

	// Check that the fairleads have correctly moved
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		REQUIRE(err == MOORDYN_SUCCESS);
		REQUIRE(isclose(xf[3 * i], xf_ref[3 * i] + VX * dt));
		REQUIRE(isclose(xf[3 * i + 1], xf_ref[3 * i + 1]));
		REQUIRE(isclose(xf[3 * i + 2], xf_ref[3 * i + 2]));
	}

	// Let rotate it a little
	t += dt;
	x[0] = VX * dt;
	dx[0] = 0.0;
	dx[3] = VRX;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	REQUIRE(err == MOORDYN_SUCCESS);

	// Check the new position and velocity of the body
	err = MoorDyn_GetBodyState(body, x_new, dx_new);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(isclose(x_new[0], VX * dt));
	REQUIRE(isclose(x_new[1], 0));
	REQUIRE(isclose(x_new[2], COG_Z));
	REQUIRE(isclose(x_new[3], 0));
	REQUIRE(isclose(x_new[4], 0));
	REQUIRE(isclose(x_new[5], 0));
	REQUIRE(isclose(dx_new[0], 0));
	REQUIRE(isclose(dx_new[1], 0));
	REQUIRE(isclose(dx_new[2], 0));
	REQUIRE(isclose(dx_new[3], VRX));
	REQUIRE(isclose(dx_new[4], 0));
	REQUIRE(isclose(dx_new[5], 0));

	// Check that the fairleads have correctly moved
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		REQUIRE(err == MOORDYN_SUCCESS);
		const double rx = 0.01 * M_PI / 180.0 * dt;
		const double x_ref = xf_ref[3 * i] + 0.01 * dt;
		const double y_ref = xf_ref[3 * i + 1] * cos(rx) -
			(xf_ref[3 * i + 2] - COG_Z) * sin(rx);
		const double z_ref = xf_ref[3 * i + 1] * sin(rx) +
			(xf_ref[3 * i + 2] - COG_Z) * cos(rx) + COG_Z;
		REQUIRE(isclose(xf[3 * i], x_ref));
		REQUIRE(isclose(xf[3 * i + 1], y_ref));
		REQUIRE(isclose(xf[3 * i + 2], z_ref));
	}

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}
