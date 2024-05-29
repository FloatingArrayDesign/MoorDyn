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

/** @file beam.cpp
 * Simply supported and cantilevered beams (Cables modelled by Euler-Bernoulli
 * equations).
 */

#include "MoorDyn2.h"
#include <cmath>
#include <catch2/catch_test_macros.hpp>

#define TOL 0.5
#define Z0 10.0
#define L 10.0
#define D 0.05
#define W 100.0
#define EA 1.5e9
#define EI 1.0e7
#define G 9.80665

bool
compare(double v1, double v2)
{
	auto err = std::abs(v1 - v2);
	if (!err)
		return true;
	auto renorm = v1 ? std::abs(v1) : 1.0;
	return err / renorm <= TOL;
}

using namespace std;

/** @brief Simply supported beam analytic solution
 * @param x x position of the node
 * @return z position of the node
 * @see
 * https://www.efunda.com/formulae/solid_mechanics/beams/casestudy_display.cfm?case=simple_uniformload
 */
double
simply_supported_solution(double x)
{
	const double p = W * G;
	return -p * x * (L * L * L - 2. * x * x * L + x * x * x) / (24. * EI);
}

TEST_CASE("Simply supported beam")
{
	MoorDyn system = MoorDyn_Create("Mooring/BeamSimplySupported.txt");
	REQUIRE(system);

	REQUIRE(MoorDyn_Init(system, NULL, NULL) == MOORDYN_SUCCESS);

	// Compare the node positions
	auto line = MoorDyn_GetLine(system, 1);
	REQUIRE(line);
	unsigned int n_nodes;
	REQUIRE(MoorDyn_GetLineNumberNodes(line, &n_nodes) == MOORDYN_SUCCESS);
	for (unsigned int i = 0; i < n_nodes; i++) {
		double pos[3];
		REQUIRE(MoorDyn_GetLineNodePos(line, i, pos) == MOORDYN_SUCCESS);
		const double z = simply_supported_solution(pos[0]);
		REQUIRE(compare(z, pos[2] - Z0));
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

/** @brief Cantilevered beam analytic solution
 * @param x x position of the node
 * @return z position of the node
 * @see
 * https://www.efunda.com/formulae/solid_mechanics/beams/casestudy_display.cfm?case=cantilever_uniformload
 */
double
cantilevered_solution(double x)
{
	const double p = W * G;
	return -p * x * x * (6. * L * L - 4. * x * L + x * x) / (24. * EI);
}

TEST_CASE("Cantilevered beam")
{
	MoorDyn system = MoorDyn_Create("Mooring/BeamCantilevered.txt");
	REQUIRE(system);

	REQUIRE(MoorDyn_Init(system, NULL, NULL) == MOORDYN_SUCCESS);

	// Compare the node positions
	auto line = MoorDyn_GetLine(system, 1);
	REQUIRE(line);
	unsigned int n_nodes;
	REQUIRE(MoorDyn_GetLineNumberNodes(line, &n_nodes) == MOORDYN_SUCCESS);
	for (unsigned int i = 0; i < n_nodes; i++) {
		double pos[3];
		REQUIRE(MoorDyn_GetLineNodePos(line, i, pos) == MOORDYN_SUCCESS);
		const double z = cantilevered_solution(pos[0]);
		REQUIRE(compare(z, pos[2] - Z0));
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}
