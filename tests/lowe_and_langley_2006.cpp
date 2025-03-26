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

/** @file lowe_and_langley_2006.cpp
 * Validation against the Lowe and Langley OMAE 2006 paper. Their results have
 * been digitalized with https://github.com/automeris-io/WebPlotDigitizer
 */

#define _USE_MATH_DEFINES
#include "MoorDyn2.h"
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <tuple>
#include <cmath>
#include <catch2/catch_test_macros.hpp>

using namespace std;

/// Time step in the moton files
const double DT = 0.001;

/** @brief Parse a line of a tabulated file
 * @param line The line of text
 * @return The vector of values
 */
vector<double>
parse_tab_line(const char* line)
{
	vector<double> fields;
	const char del = ',';
	stringstream sstream(line);
	string word;
	while (std::getline(sstream, word, del)) {
		fields.push_back(stod(word.c_str()));
	}
	return fields;
}

/** @brief Read a tabulated file
 * @param filepath The tabulated file path
 * @return 2D array
 */
vector<vector<double>>
read_tab_file(const char* filepath)
{
	vector<vector<double>> data;
	fstream f;
	f.open(filepath, ios::in);
	if (!f.is_open())
		return data;
	string line;
	while (getline(f, line)) {
		data.push_back(parse_tab_line(line.c_str()));
	}
	f.close();

	return data;
}

/** @brief Initialize the jumper
 * @return The MoorDyn system, the line and the failead
 */
std::tuple<MoorDyn, MoorDynLine, MoorDynPoint>
init()
{
	MoorDyn system = MoorDyn_Create("Mooring/lowe_and_langley_2006/line.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	auto line = MoorDyn_GetLine(system, 1);
	REQUIRE(line);
	auto point = MoorDyn_GetPoint(system, 2);
	REQUIRE(point);

	// Set the fairlead points, as they are in the config file
	double r[3], u[3];
	REQUIRE(MoorDyn_GetPointPos(point, r) == MOORDYN_SUCCESS);
	std::fill(u, u + 3, 0.0);
	REQUIRE(MoorDyn_Init(system, r, u) == MOORDYN_SUCCESS);

	return { system, line, point };
}

TEST_CASE("Stationary")
{
	const double tol = 0.2;
	const double top_ten_ref[3] = { 11.40, 0.0, 45.71 };
	const double bottom_ten_ref[3] = { -11.40, 0.0, 24.04 };
	auto [system, line, point] = init();
	unsigned int n_segments;
	REQUIRE(MoorDyn_GetLineN(line, &n_segments) == MOORDYN_SUCCESS);

	double top_ten[3], bottom_ten[3];
	REQUIRE(MoorDyn_GetPointForce(point, top_ten) == MOORDYN_SUCCESS);
	REQUIRE(MoorDyn_GetLineNodeForce(line, 0, bottom_ten) == MOORDYN_SUCCESS);
	for (unsigned int i = 0; i < 3; i++) {
		REQUIRE(std::abs(top_ten_ref[i] + 1.0e-3 * top_ten[i]) < tol);
		REQUIRE(std::abs(bottom_ten_ref[i] + 1.0e-3 * bottom_ten[i]) < tol);
	}
}

/** @brief Run a linear motion case
 * @param case_id Either 1, 2 or 3
 */
void
case123(const unsigned int case_id)
{
	REQUIRE(((case_id >= 1) && (case_id <= 3)));
	const double tol = 1.0;
	const double A = 10.0;
	const double T = 27.0;
	double dt = 0.005;

	stringstream ref_file;
	ref_file << "Mooring/lowe_and_langley_2006/Case " << case_id << ".csv";
	auto ref_data = read_tab_file(ref_file.str().c_str());
	unsigned int ref_data_index = 0;

	auto [system, line, point] = init();
	unsigned int n_segments;
	REQUIRE(MoorDyn_GetLineN(line, &n_segments) == MOORDYN_SUCCESS);

	double r0[3], u[3];
	REQUIRE(MoorDyn_GetPointPos(point, r0) == MOORDYN_SUCCESS);
	std::fill(u, u + 3, 0.0);

	double t = 0.0;
	const double t_ramp = 1.0 * T;
	while (t < 80.0 + t_ramp) {
		double f = t >= t_ramp ? 1.0 : t / t_ramp;
		f = f * f * f * (6 * f * f - 15 * f + 10);

		double r[3], ten[3];
		REQUIRE(MoorDyn_GetPointPos(point, r) == MOORDYN_SUCCESS);
		double x = r0[case_id - 1] + A * sin(2.0 * M_PI / T * (t + dt));
		x = r0[case_id - 1] + f * (x - r0[case_id - 1]);
		u[case_id - 1] = (x - r[case_id - 1]) / dt;
		REQUIRE(MoorDyn_Step(system, r, u, ten, &t, &dt) == MOORDYN_SUCCESS);

		const double t_ref = t - t_ramp;
		if (t_ref >= ref_data[ref_data_index][0]) {
			REQUIRE(MoorDyn_GetLineNodeForce(line, n_segments, ten) ==
			        MOORDYN_SUCCESS);
			const double ten_kn =
			    1.0e-3 *
			    sqrt(ten[0] * ten[0] + ten[1] * ten[1] + ten[2] * ten[2]);
			REQUIRE(std::abs(ten_kn - ref_data[ref_data_index][1]) < tol);
			ref_data_index++;
			if (ref_data_index >= ref_data.size())
				break;
		}
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

TEST_CASE("Case 1")
{
	case123(1);
}

TEST_CASE("Case 2")
{
	case123(2);
}

TEST_CASE("Case 3")
{
	case123(3);
}

#define Z 100.0

/** @brief Solve the linear wave theory on a point
 * @param A Wave amplitude
 * @param T Wave period
 * @param phi Wave phase
 * @param d Wave direction angle
 * @param t Time
 * @param r Point where wave shall be solved
 * @param u Output velocity
 * @param a Output acceleration
 */
void
wave(double A,
     double T,
     double phi,
     double d,
     double t,
     const double* r,
     double* u,
     double* a)
{
	const double omega = 2.0 * M_PI / T;
	const double k = omega * omega / 9.81;

	const double x = r[0] * cos(d) + r[1] * sin(d);
	const double cs = cos(k * x - omega * t + phi);
	const double ss = sin(k * x - omega * t + phi);

	const double ch = cosh(k * (r[2] + Z)) / cosh(k * Z);
	const double sh = sinh(k * (r[2] + Z)) / cosh(k * Z);

	const double uA = omega * A;
	const double aA = omega * uA;

	u[0] = uA * cs * ch * cos(d);
	u[1] = uA * cs * ch * sin(d);
	u[2] = uA * ss * sh;
	a[0] = aA * ss * ch * cos(d);
	a[1] = aA * ss * ch * sin(d);
	a[2] = -aA * cs * sh;
}

/** @brief Run a wave driven case
 * @param case_id Either 4, 5 or 6
 */
void
case456(const unsigned int case_id, double phi)
{
	REQUIRE(((case_id >= 4) && (case_id <= 6)));
	const double tol = 1.0;
	const double A = 5.0;
	const double T = 10.0;
	double dt = 0.005;

	stringstream ref_file;
	ref_file << "Mooring/lowe_and_langley_2006/Case " << case_id << ".csv";
	auto ref_data = read_tab_file(ref_file.str().c_str());
	unsigned int ref_data_index = 0;

	auto [system, line, point] = init();
	unsigned int nwp;
	REQUIRE(MoorDyn_ExternalWaveKinInit(system, &nwp) == MOORDYN_SUCCESS);
	double* rwp = (double*)malloc(3 * nwp * sizeof(double));
	double* uwp = (double*)malloc(3 * nwp * sizeof(double));
	double* awp = (double*)malloc(3 * nwp * sizeof(double));
	REQUIRE((rwp && uwp && awp));

	unsigned int n_segments;
	REQUIRE(MoorDyn_GetLineN(line, &n_segments) == MOORDYN_SUCCESS);

	double r[3], u[3];
	REQUIRE(MoorDyn_GetPointPos(point, r) == MOORDYN_SUCCESS);
	std::fill(u, u + 3, 0.0);

	double t = 0.0;
	const double t_ramp = 2.0 * T;
	while (t < 30.0 + t_ramp) {
		double ten[3];

		REQUIRE(MoorDyn_ExternalWaveKinGetCoordinates(system, rwp) ==
		        MOORDYN_SUCCESS);
		for (unsigned int i = 0; i < nwp; i++) {
			wave(A,
			     T,
			     phi,
			     (case_id - 4) * 0.25 * M_PI,
			     t + 0.5 * dt,
			     rwp + 3 * i,
			     uwp + 3 * i,
			     awp + 3 * i);
		}
		REQUIRE(MoorDyn_ExternalWaveKinSet(system, uwp, awp, t + 0.5 * dt) ==
		        MOORDYN_SUCCESS);
		REQUIRE(MoorDyn_Step(system, r, u, ten, &t, &dt) == MOORDYN_SUCCESS);

		const double t_ref = t - t_ramp;
		if (t_ref >= ref_data[ref_data_index][0]) {
			REQUIRE(MoorDyn_GetLineNodeForce(line, n_segments, ten) ==
			        MOORDYN_SUCCESS);
			const double ten_kn =
			    1.0e-3 *
			    sqrt(ten[0] * ten[0] + ten[1] * ten[1] + ten[2] * ten[2]);
			REQUIRE(std::abs(ten_kn - ref_data[ref_data_index][1]) < tol);
			ref_data_index++;
			if (ref_data_index >= ref_data.size())
				break;
		}
	}

	free(rwp);
	free(uwp);
	free(awp);
	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

TEST_CASE("Case 4")
{
	case456(4, 230.0 * M_PI / 180.0);
}

TEST_CASE("Case 5")
{
	case456(5, 165.0 * M_PI / 180.0);
}

TEST_CASE("Case 6")
{
	case456(6, 0.0 * M_PI / 180.0);
}
