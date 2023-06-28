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

#define TOL 1.0e-6
#define COG_Z 0.0
#define VX 0.01
#define VRX 0.01 * M_PI / 180.0

bool
compare(double v1, double v2)
{
	return fabs(v1 - v2) <= TOL;
}

#define CHECK_VALUE(name, v1, v2)                                              \
	if (!compare(v1, v2)) {                                                    \
		cerr << setprecision(8) << "Checking " << name << " failed. " << v1    \
		     << " was expected"                                                \
		     << " but " << v2 << " was computed" << endl;                      \
		MoorDyn_Close(system);                                                 \
		return false;                                                          \
	}

using namespace std;

/** @brief Check that fairleads connected to a body move the correct way
 * @return true if the test worked, false otherwise
 */
bool
minimal()
{
	MoorDyn system = MoorDyn_Create("Mooring/lines_body.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 6) {
		cerr << "6 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[6], dx[6];
	// Get the initial position of the body
	auto body = MoorDyn_GetBody(system, 1);
	err = MoorDyn_GetBodyState(body, x, dx);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure retrieving the body position: " << err << endl;
		MoorDyn_Close(system);
		return false;
	}
	CHECK_VALUE("body x", x[0], 0);
	CHECK_VALUE("body y", x[1], 0);
	CHECK_VALUE("body z", x[2], COG_Z);
	CHECK_VALUE("body Rx", x[3], 0);
	CHECK_VALUE("body Ry", x[4], 0);
	CHECK_VALUE("body Rz", x[5], 0);

	// Get the initial position of the fairleads
	double xf_ref[9];
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf_ref + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
		cout << xf_ref[3 * i] << ", " << xf_ref[3 * i + 1] << ", " << xf_ref[3 * i + 2] << endl;
	}
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	// Check that the fairleads have not moved
	double xf[9];
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
		CHECK_VALUE("x0", xf_ref[3 * i], xf[3 * i]);
		CHECK_VALUE("y0", xf_ref[3 * i + 1], xf[3 * i + 1]);
		CHECK_VALUE("z0", xf_ref[3 * i + 2], xf[3 * i + 2]);
	}

	// Let it run for a little while without moving
	double f[9];
	double t = 0.0, dt = 0.5;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring step: " << err << endl;
		return false;
	}

	// Check that the fairleads have not moved again
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
		CHECK_VALUE("x1", xf_ref[3 * i], xf[3 * i]);
		CHECK_VALUE("y1", xf_ref[3 * i + 1], xf[3 * i + 1]);
		CHECK_VALUE("z1", xf_ref[3 * i + 2], xf[3 * i + 2]);
	}

	// Let move it forward a little
	t += dt;
	dx[0] = VX;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring step: " << err << endl;
		return false;
	}

	// Check the new position and velocity of the body
	double x_new[6], dx_new[6];
	err = MoorDyn_GetBodyState(body, x_new, dx_new);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure retrieving the body position: " << err << endl;
		MoorDyn_Close(system);
		return false;
	}
	CHECK_VALUE("body x2", VX * dt, x_new[0]);
	CHECK_VALUE("body y2", 0, x_new[1]);
	CHECK_VALUE("body z2", COG_Z, x_new[2]);
	CHECK_VALUE("body Rx2", 0, x_new[3]);
	CHECK_VALUE("body Ry2", 0, x_new[4]);
	CHECK_VALUE("body Rz2", 0, x_new[5]);
	CHECK_VALUE("body dx2", VX, dx_new[0]);
	CHECK_VALUE("body dy2", 0, dx_new[1]);
	CHECK_VALUE("body dz2", 0, dx_new[2]);
	CHECK_VALUE("body dRx2", 0, dx_new[3]);
	CHECK_VALUE("body dRy2", 0, dx_new[4]);
	CHECK_VALUE("body dRz2", 0, dx_new[5]);

	// Check that the fairleads have correctly moved
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
		CHECK_VALUE("x2", xf_ref[3 * i] + 0.01 * dt, xf[3 * i]);
		CHECK_VALUE("y2", xf_ref[3 * i + 1], xf[3 * i + 1]);
		CHECK_VALUE("z2", xf_ref[3 * i + 2], xf[3 * i + 2]);
	}

	// Let rotate it a little
	t += dt;
	x[0] = VX * dt;
	dx[0] = 0.0;
	dx[3] = VRX;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring step: " << err << endl;
		return false;
	}

	// Check the new position and velocity of the body
	err = MoorDyn_GetBodyState(body, x_new, dx_new);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure retrieving the body position: " << err << endl;
		MoorDyn_Close(system);
		return false;
	}
	CHECK_VALUE("body x3", VX * dt, x_new[0]);
	CHECK_VALUE("body y3", 0, x_new[1]);
	CHECK_VALUE("body z3", COG_Z, x_new[2]);
	CHECK_VALUE("body Rx3", VRX * dt, x_new[3]);
	CHECK_VALUE("body Ry3", 0, x_new[4]);
	CHECK_VALUE("body Rz3", 0, x_new[5]);
	CHECK_VALUE("body dx3", 0, dx_new[0]);
	CHECK_VALUE("body dy3", 0, dx_new[1]);
	CHECK_VALUE("body dz3", 0, dx_new[2]);
	CHECK_VALUE("body dRx3", VRX, dx_new[3]);
	CHECK_VALUE("body dRy3", 0, dx_new[4]);
	CHECK_VALUE("body dRz3", 0, dx_new[5]);

	// Check that the fairleads have correctly moved
	for (unsigned int i = 0; i < 3; i++) {
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, xf + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
		const double rx = 0.01 * M_PI / 180.0 * dt;
		const double x_ref = xf_ref[3 * i] + 0.01 * dt;
		const double y_ref = xf_ref[3 * i + 1] * cos(rx) -
			xf_ref[3 * i + 2] * sin(rx);
		const double z_ref = xf_ref[3 * i + 1] * sin(rx) +
			xf_ref[3 * i + 2] * cos(rx);
		CHECK_VALUE("x3", x_ref, xf[3 * i]);
		CHECK_VALUE("y3", y_ref, xf[3 * i + 1]);
		CHECK_VALUE("z3", z_ref, xf[3 * i + 2]);
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
	if (!minimal())
		return 1;
	return 0;
}
