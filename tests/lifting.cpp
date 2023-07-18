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

/** @file pendulum.cpp
 * Simple case of a pendulum motion
 */

// Visual studio still uses this
#define _USE_MATH_DEFINES

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#define TOL 0.5

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

/** @brief Pendulum motion
 * @return true if the test worked, false otherwise
 */
bool
lifting()
{
	MoorDyn system = MoorDyn_Create("Mooring/lifting.txt");
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
	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	auto line = MoorDyn_GetLine(system, 1);
	if (!line) {
		cerr << "Failure getting the line" << endl;
		return false;
	}
	const auto point = MoorDyn_GetPoint(system, 2);
	if (!point) {
		cerr << "Failure getting the point" << endl;
		return false;
	}

	const double lift_vel = 1.0;
	err = MoorDyn_SetLineUnstretchedLengthVel(line, -lift_vel);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure setting the line length rate of change: " << err
		     << endl;
		return false;
	}
	double line_len0;
	err = MoorDyn_GetLineUnstretchedLength(line, &line_len0);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting the starting line length: " << err << endl;
		return false;
	}

	double dt = 1.0;
	double t = 0.0;
	while (t < 10.0) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
		}
		double line_len;
		err = MoorDyn_GetLineUnstretchedLength(line, &line_len);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the line length: " << err << endl;
			return false;
		}
		if (fabs(line_len - (line_len0 - lift_vel * t)) > TOL) {
			cerr << "Line unstretched length error (t = " << t
			     << "): " << line_len << " vs. " << line_len0 - lift_vel * t
			     << endl;
			return false;
		}

		double pos[3];
		err = MoorDyn_GetPointPos(point, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the mass position: " << err << endl;
			return false;
		}

		if (fabs(line_len + pos[2]) > TOL) {
			cerr << "Line length error (t = " << t << "): " << line_len
			     << " vs. " << -pos[2] << endl;
			return false;
		}

		t += dt;
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
	if (!lifting())
		return 1;
	return 0;
}
