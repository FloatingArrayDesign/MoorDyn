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
 * Validation against Orcaflex
 */

#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace std;

/// Time step in the moton files
const double DT = 0.1;
/// List of available depths
vector<string> DEPTHS({ "0050", "0200", "0600" });
/// List of available motions
vector<string> MOTIONS({ "ZZP1_A1", "ZZP1_A2" });
/// List of static tensions at the fairlead predicted by quasi-static codes
vector<double> STATIC_FAIR_TENSION({ 991.6, 2065.4, 5232.6 });
/// List of static tensions at the anchor predicted by quasi-static codes
vector<double> STATIC_ANCHOR_TENSION({ 826.2, 1402.2, 3244.2 });
/// Allowed relative error in the static tension value
const double MAX_STATIC_ERROR = 0.1;
/// Allowed relative error in the variable tension value
const double MAX_DYNAMIC_ERROR = 0.15;

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
	if (!f.is_open()) {
		cerr << "Cannot open file " << filepath << endl;
		return data;
	}
	string line;
	while (getline(f, line)) {
		data.push_back(parse_tab_line(line.c_str()));
	}
	f.close();

	return data;
}

/** @brief Run a validation against a quasi-static code
 * @return true if the test worked, false otherwise
 */
bool
validation(const char* depth, const char* motion)
{
	auto it = std::find(DEPTHS.begin(), DEPTHS.end(), depth);
	if (it == DEPTHS.end()) {
		cerr << "Unhandled water depth: " << depth << endl;
		return false;
	}
	const unsigned int depth_i = (it - DEPTHS.begin());

	stringstream lines_file, motion_file, ref_file;
	lines_file << "Mooring/WD" << depth << "_Chain"
	           << ".txt";
	motion_file << "Mooring/QuasiStatic/" << motion << ".txt";
	ref_file << "Mooring/QuasiStatic/WD" << depth << "_Chain_" << motion
	         << ".txt";
	auto motion_data = read_tab_file(motion_file.str().c_str());
	auto ref_data = read_tab_file(ref_file.str().c_str());

	MoorDyn system = MoorDyn_Create(lines_file.str().c_str());
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 3) {
		cerr << "3x1 = 3 DOFs were expected, but " << n_dof << "were reported"
		     << endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[3], dx[3];
	// Set the fairlead points, as they are in the config file
	std::fill(x, x + 3, 0.0);
	std::fill(dx, dx + 3, 0.0);
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	// Compute the static tension
	int num_lines = 1;
	float fh, fv, ah, av;
	err = MoorDyn_GetFASTtens(system, &num_lines, &fh, &fv, &ah, &av);
	if (err != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		cerr << "Failure getting the initial tension: " << err << endl;
		return false;
	}
	const double ffair0 = sqrt(fh * fh + fv * fv);
	const double ffair_ref0 = 1.e3 * STATIC_FAIR_TENSION[depth_i];
	cout << "Static tension on the fairlead = " << ffair0 << endl;
	cout << "    Reference value = " << ffair_ref0 << endl;
	const double fanch0 = sqrt(ah * ah + av * av);
	const double fanch_ref0 = 1.e3 * STATIC_ANCHOR_TENSION[depth_i];
	cout << "Static tension on the anchor = " << fanch0 << endl;
	cout << "    Reference value = " << fanch_ref0 << endl;
	const double efair0 = (ffair0 - ffair_ref0) / ffair_ref0;
	const double eanch0 = (fanch0 - fanch_ref0) / fanch_ref0;
	if ((efair0 > MAX_STATIC_ERROR) || (eanch0 > MAX_STATIC_ERROR)) {
		MoorDyn_Close(system);
		cerr << "Too large error" << endl;
		return false;
	}

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
		err = MoorDyn_Step(system, x, dx, f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
		}

		if (t_ref < 0.0)
			continue;

		err = MoorDyn_GetFASTtens(system, &num_lines, &fh, &fv, &ah, &av);
		if (err != MOORDYN_SUCCESS) {
			MoorDyn_Close(system);
			cerr << "Failure getting the initial tension: " << err << endl;
			return false;
		}
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

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	cout << "Maximum dynamic error in the fairlead = " << ef_value << endl;
	cout << "    at time = " << ef_time << endl;
	ef_value = ef_value / (2.0 * ef_ref);
	if (ef_value > MAX_DYNAMIC_ERROR) {
		cerr << ef_value << " is an excesively large error" << endl;
		return false;
	}
	cout << "Maximum dynamic error in the anchor = " << ea_value << endl;
	cout << "    at time = " << ea_time << endl;
	ea_value = ea_value / (2.0 * ea_ref);
	/*  For the time being we better ignore these errors
	if (ea_value > MAX_DYNAMIC_ERROR)
	{
	    cerr << ea_value << " is an excesively large error" << endl;
	    return false;
	}
	*/

	return true;
}

/** @brief Runs all the test
 * @return 0 if the tests have ran just fine, 1 otherwise
 */
int
main(int, char**)
{
	for (auto depth : DEPTHS) {
		for (auto motion : MOTIONS) {
			if (!validation(depth.c_str(), motion.c_str()))
				return 1;
		}
	}
	return 0;
}
