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

/** @file stability.cpp
 * Simple cases with analytical solutions to test the model stability
 */

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include "util.h"

using namespace std;


TEST_CASE("Single mass hanging from a spring")
{
	MoorDyn system = MoorDyn_Create("Mooring/stability/spring.txt");
	REQUIRE(system);

	int err;
	unsigned int n_dof;
	err = MoorDyn_NCoupledDOF(system, &n_dof);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 0);

	const auto point = MoorDyn_GetPoint(system, 2);
	REQUIRE(err == MOORDYN_SUCCESS);

	double pos[3];
	err = MoorDyn_Init(system, NULL, NULL);
	REQUIRE(err == MOORDYN_SUCCESS);
	err = MoorDyn_GetPointPos(point, pos);
	REQUIRE(err == MOORDYN_SUCCESS);
	const double z0 = pos[2];
	cout << "z0 = " << z0 << endl;
	REQUIRE(isclose(pos[0], 0.0));
	REQUIRE(isclose(pos[1], 0.0));
	REQUIRE(isclose(z0, -1.0));

	const double g = 9.8;
	const double m = 1.0;
	const double EA = 1.0;
	const double l0 = 1.0;

	double dt = 0.1;
	double t = 0.0;
	const double T = 10.0;
	while (t < T) {
		cout << t << " / " << T << '\n';
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		REQUIRE(err == MOORDYN_SUCCESS);
		err = MoorDyn_GetPointPos(point, pos);
		REQUIRE(err == MOORDYN_SUCCESS);

		REQUIRE(isclose(pos[0], 0.0));
		REQUIRE(isclose(pos[1], 0.0));
		const double gm_EA = g * m / EA;
		const double z = z0 - gm_EA + gm_EA * cos(sqrt(EA / (m * l0)) * t);
		cout << "z = " << pos[2] << " vs. " << z << endl;
		REQUIRE(isclose(pos[2], z, 0.0, 1e-5));
	}

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}


TEST_CASE("Single mass hanging from a set of springs")
{
	MoorDyn system = MoorDyn_Create("Mooring/stability/springs.txt");
	REQUIRE(system);

	int err;
	unsigned int n_dof;
	err = MoorDyn_NCoupledDOF(system, &n_dof);
	REQUIRE(err == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 0);

	const auto point = MoorDyn_GetPoint(system, 2);
	REQUIRE(err == MOORDYN_SUCCESS);

	double pos[3];
	err = MoorDyn_Init(system, NULL, NULL);
	REQUIRE(err == MOORDYN_SUCCESS);
	err = MoorDyn_GetPointPos(point, pos);
	REQUIRE(err == MOORDYN_SUCCESS);
	const double z0 = pos[2];
	cout << "z0 = " << z0 << endl;
	REQUIRE(isclose(pos[0], 0.0));
	REQUIRE(isclose(pos[1], 0.0));
	REQUIRE(isclose(z0, -1.0));

	const double g = 9.8;
	const double m = 1.0;
	const double EA = 1.0;
	const double l0 = 1.0;

	double dt = 0.1;
	double t = 0.0;
	const double T = 10.0;
	while (t < T) {
		cout << t << " / " << T << '\n';
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		REQUIRE(err == MOORDYN_SUCCESS);
		err = MoorDyn_GetPointPos(point, pos);
		REQUIRE(err == MOORDYN_SUCCESS);

		REQUIRE(isclose(pos[0], 0.0));
		REQUIRE(isclose(pos[1], 0.0));
		const double gm_EA = g * m / EA;
		const double z = z0 - gm_EA + gm_EA * cos(sqrt(EA / (m * l0)) * t);
		cout << "z = " << pos[2] << " vs. " << z << endl;
		REQUIRE(isclose(pos[2], z, 0.0, 1e-5));
	}

	err = MoorDyn_Close(system);
	REQUIRE(err == MOORDYN_SUCCESS);
}
