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

#define TOL 1.0e-2
#define POSX -203.22
#define POSY 351.98
#define POSZ -273.62
#define TENX 367382.06
#define TENY -636317.09
#define TENZ 220304.04
#define CURV 0.0

bool
compare(double v1, double v2)
{
	auto err = fabs(v1 - v2);
	if (!err)
		return true;
	auto renorm = v1 ? fabs(v1) : 1.0;
	return err / renorm <= TOL;
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

namespace old_api {

/** @brief Check that bad input files are correctly handled
 * @return true if the test worked, false otherwise
 */
bool
bad_input_file()
{
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	int err;
	double x[6], dx[6];
	std::fill(x, x + 6, 0.0);
	std::fill(dx, dx + 6, 0.0);
	err = MoorDynInit(x, dx, "badfile.txt");
	if (err != MOORDYN_UNHANDLED_ERROR) {
		cerr << "The error code " << MOORDYN_UNHANDLED_ERROR
		     << " was expected, but " << err << " was received" << endl;
		return false;
	}

	err = MoorDynClose();
	if (err != MOORDYN_INVALID_VALUE) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

/** @brief Check that a mooring system can be initialized, that a step can be
 * ran and that everyting can be closed
 * @return true if the test worked, false otherwise
 */
bool
minimal()
{
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	int err;
	double x[9], dx[9];
	// Set the fairlead points, as they are in the config file
	x[0] = 5.2;
	x[1] = 0.0;
	x[2] = -70.0;
	x[3] = -2.6;
	x[4] = 4.5;
	x[5] = -70.0;
	x[6] = -2.6;
	x[7] = -4.5;
	x[8] = -70.0;
	std::fill(dx, dx + 9, 0.0);
	err = MoorDynInit(x, dx, "Mooring/lines.txt");
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double f[6 * 3 + 3 * 3]; // 6x3 for the fairleads, 3x3 for the anchors
	double t = 0.0, dt = 0.5;
	err = MoorDynStep(x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring step: " << err << endl;
		return false;
	}

	err = MoorDynClose();
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

} // ::old_api

/** @brief Check that bad input files are correctly handled
 * @return true if the test worked, false otherwise
 */
bool
bad_input_file()
{
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("badfile.txt");
	if (system) {
		cerr << "The system is not Null when a bad file is provided" << endl;
		MoorDyn_Close(system);
		return false;
	}

	return true;
}

/** @brief Check that a mooring system can be initialized, that a step can be
 * ran and that everyting can be closed
 * @return true if the test worked, false otherwise
 */
bool
minimal()
{
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 9) {
		cerr << "3x3 = 9 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[9], dx[9];
	// Get the initial positions from the config file
	for (unsigned int i = 0; i < 3; i++) {
		// 4 = first fairlead id
		auto point = MoorDyn_GetPoint(system, i + 4);
		err = MoorDyn_GetPointPos(point, x + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure retrieving the fairlead " << i + 4
			     << " position: " << err << endl;
			MoorDyn_Close(system);
			return false;
		}
	}
	std::fill(dx, dx + 9, 0.0);
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double f[9];
	double t = 0.0, dt = 0.5;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring step: " << err << endl;
		return false;
	}

	// Check a random node position, tension and curvature
	auto line = MoorDyn_GetLine(system, 2);
	unsigned int node;
	err = MoorDyn_GetLineNumberNodes(line, &node);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting the number of nodes: " << err << endl;
		MoorDyn_Close(system);
		return false;
	}
	node /= 2;
	double pos[3], ten[3], curv;
	err = MoorDyn_GetLineNodePos(line, node, pos);
	CHECK_VALUE("POSX", POSX, pos[0]);
	CHECK_VALUE("POSY", POSY, pos[1]);
	CHECK_VALUE("POSZ", POSZ, pos[2]);
	err = MoorDyn_GetLineNodeTen(line, node, ten);
	CHECK_VALUE("TENX", TENX, ten[0]);
	CHECK_VALUE("TENY", TENY, ten[1]);
	CHECK_VALUE("TENZ", TENZ, ten[2]);
	err = MoorDyn_GetLineNodeCurv(line, node, &curv);
	CHECK_VALUE("CURV", CURV, curv);

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
	if (!old_api::bad_input_file())
		return 1;
	if (!old_api::minimal())
		return 2;
	if (!bad_input_file())
		return 3;
	if (!minimal())
		return 4;
	return 0;
}
