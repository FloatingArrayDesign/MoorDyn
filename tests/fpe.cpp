/*
 * Copyright (c) 2026 Harish Gopalan
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

/** @file fpe.cpp
 * Check that no floating point exceptions are raised while creating,
 * initializing and integrating some systems.
 *
 * Programs that trap the floating point exceptions (e.g. with feenableexcept())
 * are killed by FE_INVALID, FE_DIVBYZERO or FE_OVERFLOW, even if MoorDyn
 * discards the resulting value afterwards
 */

#include "MoorDyn2.h"
#include <cfenv>
#include <string>
#include <vector>
#include <catch2/catch_test_macros.hpp>

#define FPE_FLAGS (FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW)

/** @brief Create, initialize and integrate a system for some time steps
 * @param path The input file
 * @param cpld_point The coupled point index, 0 if there are no coupled DOFs
 * @return The floating point exception flags raised
 */
int
raised_fpe(const std::string& path, unsigned int cpld_point = 0)
{
	std::feclearexcept(FE_ALL_EXCEPT);

	MoorDyn system = MoorDyn_Create(path.c_str());
	REQUIRE(system);
	REQUIRE(MoorDyn_SetVerbosity(system, MOORDYN_ERR_LEVEL) == MOORDYN_SUCCESS);
	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == (cpld_point ? 3 : 0));
	std::vector<double> x(n_dof, 0.0), xd(n_dof, 0.0), f(n_dof, 0.0);
	if (cpld_point) {
		auto point = MoorDyn_GetPoint(system, cpld_point);
		REQUIRE(point);
		REQUIRE(MoorDyn_GetPointPos(point, x.data()) == MOORDYN_SUCCESS);
	}
	REQUIRE(MoorDyn_Init(system, x.data(), xd.data()) == MOORDYN_SUCCESS);

	double t = 0.0, dt = 0.01;
	for (unsigned int i = 0; i < 10; i++) {
		REQUIRE(MoorDyn_Step(system, x.data(), xd.data(), f.data(), &t, &dt) ==
		        MOORDYN_SUCCESS);
	}
	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);

	return std::fetestexcept(FPE_FLAGS);
}

TEST_CASE("Line between fixed points, stationary IC")
{
	// Points, rods and bodies have no characteristic length for the CFL
	REQUIRE(raised_fpe("Mooring/fpe/span.txt") == 0);
}

TEST_CASE("Weightless line catenary IC")
{
	// The catenary solver cannot find a solution, so it is discarded
	REQUIRE(raised_fpe("Mooring/pendulum.txt") == 0);
}

TEST_CASE("Body with a soft line and a prescribed time step")
{
	// cfl = max() if dtM is prescribed, and the line natural period > 1 s
	REQUIRE(raised_fpe("Mooring/body_tests/bodyDrag.txt") == 0);
}

TEST_CASE("Zero-length rods")
{
	REQUIRE(raised_fpe("Mooring/local_euler/complex_system.txt", 4) == 0);
}

TEST_CASE("Coupled fairlead, stationary IC")
{
	REQUIRE(raised_fpe("Mooring/polyester/simple.txt", 2) == 0);
}
