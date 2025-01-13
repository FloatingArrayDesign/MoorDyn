/*
 * Copyright (c) 2024 Jose Luis Cercos-Pita <jlc@core-marine.com> and Matt Hall
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

/** @file polyester.cpp
 * Minimal tests that only checks the library is correctly initialized,
 * running and closing
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <deque>
#include "MoorDyn2.h"
#include "csv_parser.h"
#include <catch2/catch_test_macros.hpp>


#define MBL 25.3e6
#define KRS 13.4
#define KRD1 16.0
#define KRD2 0.35
#define TC 200.0
// The tensions expected, on inverse order
#define TTIMES { 320.0, 640.0, 960.0, 1280.0, 1599.0 }
#define TMEANS { 0.1, 0.2, 0.3, 0.4, 0.5 }


double vec_norm(const double v[3])
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double get_average_tension(MoorDynLine line,
                           MoorDynPoint anchor,
                           MoorDynPoint fairlead)
{
	unsigned int n;
	REQUIRE(MoorDyn_GetLineN(line, &n) == MOORDYN_SUCCESS);
	// To average the tensions along the line, we are considering the
	// connections instead of the line end up nodes, as long as the latter are
	// not that accurate
	double tension = 0.0;
	double force[3];
	// REQUIRE(MoorDyn_GetPointForce(anchor, force) == MOORDYN_SUCCESS);
	// tension += vec_norm(force);
	REQUIRE(MoorDyn_GetPointForce(fairlead, force) == MOORDYN_SUCCESS);
	tension += vec_norm(force);
	for (unsigned int i = 0; i < n; i++) {
		REQUIRE(MoorDyn_GetLineNodeTen(line, i, force) == MOORDYN_SUCCESS);
		tension += vec_norm(force);
	}
	return tension / (n + 1);
}

TEST_CASE("Ramp up, stabilization and cycles")
{
	MoorDyn system = MoorDyn_Create("Mooring/polyester/simple.txt");
	REQUIRE(system);
	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	double r[3], dr[3];
	auto anchor = MoorDyn_GetPoint(system, 1);
	REQUIRE(anchor);
	auto fairlead = MoorDyn_GetPoint(system, 2);
	REQUIRE(fairlead);
	auto line = MoorDyn_GetLine(system, 1);
	REQUIRE(line);

	double l0;
	REQUIRE(MoorDyn_GetLineUnstretchedLength(line, &l0) == MOORDYN_SUCCESS);

	REQUIRE(MoorDyn_GetPointPos(fairlead, r) == MOORDYN_SUCCESS);
	std::fill(dr, dr + 3, 0.0);
	REQUIRE(MoorDyn_Init(system, r, dr) == MOORDYN_SUCCESS);

	// Read the csv with the motions
	std::ifstream csv("Mooring/polyester/motion.csv");
	aria::csv::CsvParser csv_parser(csv);

	std::vector<double> tdata, xdata;
	unsigned int skip = 2;
	for (auto& row : csv_parser) {
		if (skip > 0) {
			skip--;
			continue;
		}
		if (row.size() < 2)
			continue;
		tdata.push_back(std::stod(row.at(0)));
		xdata.push_back(std::stod(row.at(1)));
	}

	// Time to move
	double t = 0.0, dt;
	// Data about the lenght and stiffness recomputation as a function of time
	std::deque<double> times;  
	std::deque<double> tensions;  
	std::deque<double> ttimes(TTIMES);
	std::deque<double> tmeans(TMEANS);
	for (unsigned int i = 0; i < tdata.size(); i++) {
		REQUIRE(MoorDyn_GetPointPos(fairlead, r) == MOORDYN_SUCCESS);
		double t_dst = tdata.at(i);
		double dt = t_dst - t;
		double x_dst = xdata.at(i);
		dr[0] = (x_dst - r[0]) / dt;
		double f[3];
		REQUIRE(MoorDyn_Step(system, r, dr, f, &t, &dt) == MOORDYN_SUCCESS);
		times.push_back(t);
		tensions.push_back(get_average_tension(line, anchor, fairlead));

		// Let's check and tweak the line if there is info enough
		if (times.back() - times.front() < TC)
			continue;

		double tension = 0.0;
		for (auto f : tensions)
			tension += f;
		tension /= tensions.size();
		times.pop_front();
		tensions.pop_front();

		double ks = KRS * MBL;
		double kd = (KRD1 + KRD2 * tension / MBL * 100) * MBL;
		double l = l0 * (1.0 + tension / ks) / (1.0 + tension / kd);
		REQUIRE(
			MoorDyn_SetLineConstantEA(line, kd) == MOORDYN_SUCCESS);
		REQUIRE(
			MoorDyn_SetLineUnstretchedLength(line, l) == MOORDYN_SUCCESS);

		if (t >= ttimes.front()) {
			const double tmean = tmeans.front();
			ttimes.pop_front();
			tmeans.pop_front();
			REQUIRE(
				fabs(tension / MBL - tmean) < 0.025);
		}
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}
