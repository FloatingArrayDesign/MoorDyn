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

#include "MoorDyn2.h"
#include <algorithm>
#include <cmath>
#include <catch2/catch_test_macros.hpp>

// #define SAVE_VTK

using namespace std;

TEST_CASE("Breaker")
{
	MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 9);

	int err;
	double x[9], dx[9];
	// Get the initial positions from the config file
	for (unsigned int i = 0; i < 3; i++) {
		// 4 = first fairlead id
		auto point = MoorDyn_GetPoint(system, i + 4);
		REQUIRE(point);
		REQUIRE(MoorDyn_GetPointPos(point, x + 3 * i) == MOORDYN_SUCCESS);
	}

	auto point = MoorDyn_GetPoint(system, 4);
	REQUIRE(point);
	auto line = MoorDyn_GetLine(system, 1);
	REQUIRE(line);

	std::fill(dx, dx + 9, 0.0);
	REQUIRE(MoorDyn_Init(system, x, dx) == MOORDYN_SUCCESS);
#ifdef SAVE_VTK
	REQUIRE(MoorDyn_SaveVTK(system, "line_break.000.vtm") == MOORDYN_SUCCESS);
#endif
	// Let's move the system at a 10.0m/s speed during 0.5 seconds
	double f[9];
	dx[0] = 10.0;
	double t = 0.0, dt = 0.5;
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);
#ifdef SAVE_VTK
	REQUIRE(MoorDyn_SaveVTK(system, "line_break.001.vtm") == MOORDYN_SUCCESS);
#endif
	x[0] += dx[0] * dt;

	// Break a line and repeat 3 times
	REQUIRE(MoorDyn_BreakLine(system, point, line) == MOORDYN_SUCCESS);
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);
#ifdef SAVE_VTK
	REQUIRE(MoorDyn_SaveVTK(system, "line_break.002.vtm") == MOORDYN_SUCCESS);
#endif
	x[0] += dx[0] * dt;
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);
#ifdef SAVE_VTK
	REQUIRE(MoorDyn_SaveVTK(system, "line_break.003.vtm") == MOORDYN_SUCCESS);
#endif
	x[0] += dx[0] * dt;
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);
#ifdef SAVE_VTK
	REQUIRE(MoorDyn_SaveVTK(system, "line_break.004.vtm") == MOORDYN_SUCCESS);
#endif
	x[0] += dx[0] * dt;

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}
