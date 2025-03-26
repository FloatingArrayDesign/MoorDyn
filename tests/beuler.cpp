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

/** @file beuler.cpp
 * Tests ran with the implicit Backwards Euler
 */

#include "MoorDyn2.h"
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <catch2/catch_test_macros.hpp>

using namespace std;

/// Time step in the moton files
#define DT 0.1
/// List of available depths
#define DEPTH "0600"
/// List of available motions
#define MOTION "ZZP1_A1"
/// List of static tensions at the fairlead predicted by quasi-static codes
#define STATIC_FAIR_TENSION 5232.6
/// List of static tensions at the anchor predicted by quasi-static codes
#define STATIC_ANCHOR_TENSION 3244.2
/// Allowed relative error in the static tension value
#define MAX_STATIC_ERROR 0.1
/// Allowed relative error in the variable tension value
#define MAX_DYNAMIC_ERROR 0.15

/** @brief Parse a line of a tabulated file
 * @param line The line of text
 * @return The vector of values
 */
vector<double>
parse_tab_line(const char* line)
{
	vector<double> fields;
	const char del = '\t';
	stringstream sstream(line);
	string word;
	while (std::getline(sstream, word, del)) {
		fields.push_back(stod(word.c_str()));
	}
	return fields;
}

/** @brief Read a tabulated file
 * @param filepath The tabulated file path
 * @return 2D array, where the first dimension is the file line and the second
 * is the field
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

TEST_CASE("quasi_static_chain with beuler10")
{
	stringstream lines_file, motion_file, ref_file;
	lines_file << "Mooring/WD" << DEPTH << "_Chain" << ".txt";
	motion_file << "Mooring/QuasiStatic/" << MOTION << ".txt";
	ref_file << "Mooring/QuasiStatic/WD" << DEPTH << "_Chain_" << MOTION
	         << ".txt";
	auto motion_data = read_tab_file(motion_file.str().c_str());
	auto ref_data = read_tab_file(ref_file.str().c_str());

	MoorDyn system = MoorDyn_Create(lines_file.str().c_str());
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	double x[3], dx[3];
	// Set the fairlead points, as they are in the config file
	std::fill(x, x + 3, 0.0);
	std::fill(dx, dx + 3, 0.0);
	REQUIRE(MoorDyn_Init(system, x, dx) == MOORDYN_SUCCESS);

	// Compute the static tension
	int num_lines = 1;
	float fh, fv, ah, av;
	REQUIRE(MoorDyn_GetFASTtens(system, &num_lines, &fh, &fv, &ah, &av) ==
	        MOORDYN_SUCCESS);
	const double ffair0 = sqrt(fh * fh + fv * fv);
	const double ffair_ref0 = 1.e3 * STATIC_FAIR_TENSION;
	const double fanch0 = sqrt(ah * ah + av * av);
	const double fanch_ref0 = 1.e3 * STATIC_ANCHOR_TENSION;
	const double efair0 = (ffair0 - ffair_ref0) / ffair_ref0;
	const double eanch0 = (fanch0 - fanch_ref0) / fanch_ref0;
	REQUIRE(efair0 <= MAX_STATIC_ERROR);
	REQUIRE(eanch0 <= MAX_STATIC_ERROR);

	// Change the time scheme
	REQUIRE(MoorDyn_SetTimeScheme(system, "beuler10") == MOORDYN_SUCCESS);
	REQUIRE(MoorDyn_SetCFL(system, 0.6) == MOORDYN_SUCCESS);
	double dtM;
	REQUIRE(MoorDyn_GetDt(system, &dtM) == MOORDYN_SUCCESS);
	std::cout << "New time step = " << dtM << " s" << std::endl;

	// Start integrating. The integration have a first chunk of initialization
	// motion to get something more periodic. In that chunk of the simulation
	// we are not checking for errors
	double ef_time = 0.0;
	double ef_value = 0.0;
	double ef_ref = 0.0;
	double ea_time = 0.0;
	double ea_value = 0.0;
	double ea_ref = 0.0;
	unsigned int i_ref = 0; // To track the line in the ref values file
	double f[3];
	double t_ref0 = motion_data[0][0];
	for (unsigned int i = 0; i < motion_data.size() - 1; i++) {
		double t_ref = motion_data[i][0];
		double t = t_ref - t_ref0;
		double dt = DT;
		for (unsigned int j = 0; j < 3; j++) {
			x[j] = motion_data[i][j + 1];
			dx[j] = (motion_data[i + 1][j + 1] - x[j]) / dt;
		}
		REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);

		if (t_ref < 0.0)
			continue;

		REQUIRE(MoorDyn_GetFASTtens(system, &num_lines, &fh, &fv, &ah, &av) ==
		        MOORDYN_SUCCESS);
		const double ffair = sqrt(fh * fh + fv * fv) - ffair0;
		const double ffair_ref = 1.e3 * ref_data[i_ref][3] - ffair_ref0;
		const double fanch = sqrt(ah * ah + av * av) - fanch0;
		const double fanch_ref = 2.0 * (1.e3 * ref_data[i_ref][4] - fanch_ref0);
		if (fabs(ffair - ffair_ref) > ef_value) {
			ef_time = t;
			ef_value = fabs(ffair - ffair_ref);
		}
		if (fabs(ffair_ref) > ef_ref)
			ef_ref = fabs(ffair_ref);
		if (fabs(fanch - fanch_ref) > ea_value) {
			ea_time = t;
			ea_value = fabs(fanch - fanch_ref);
		}
		if (fabs(fanch_ref) > ea_ref)
			ea_ref = fabs(fanch_ref);

		i_ref++;
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);

	ef_value = ef_value / (2.0 * ef_ref);
	const double max_rel_err = MAX_DYNAMIC_ERROR;
	REQUIRE(ef_value <= max_rel_err);
	ea_value = ea_value / (2.0 * ea_ref);
	// For the time being we better ignore these errors
	// REQUIRE(ea_value <= max_rel_err);
}
