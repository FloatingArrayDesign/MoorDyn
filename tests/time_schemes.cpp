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

/** @file time_schemes.cpp
 * Use the same example than the pendulum, but with different time schemes,
 * using the maximum time step which brings them to the brink of stability
 */

// Visual studio still uses this
#define _USE_MATH_DEFINES

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <string>
#include <regex>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

static std::vector<std::string> schemes({ "Euler",
                                          "Heun",
                                          "RK2",
                                          "RK4",
                                          // "AB2",
                                          // "AB3",
                                          // "AB4",
                                          "BEuler5",
                                          "BEuler10",
                                          "BEuler15",
                                          "BEuler20",
                                          "Midpoint5",
                                          "Midpoint10",
                                          "Midpoint15",
                                          "Midpoint20",
                                          "ACA5",
                                          "ACA10",
                                          "ACA15",
                                          "ACA20",
                                          "Wilson20" });
static std::vector<std::string> dts({ "1.5E-4", // Euler
                                      "1.8E-4", // Heun
                                      "2.6E-4", // RK2
                                      "4.9E-4", // RK4
                                      // "1.4E-4",   // AB2
                                      // "1.1E-4",   // AB3
                                      // "1.0E-4",   // AB5
                                      "6.0E-4",    // BEuler5
                                      "1.0E-3",    // BEuler10
                                      "1.2E-3",    // BEuler15
                                      "1.5E-3",    // BEuler20
                                      "1.0E-3",    // Midpoint5
                                      "1.9E-3",    // Midpoint10
                                      "2.5E-3",    // Midpoint15
                                      "3.0E-3",    // Midpoint20
                                      "9.4E-4",    // ACA5
                                      "1.5E-3",    // ACA10
                                      "1.5E-3",    // ACA15
                                      "1.6E-3",    // ACA20
                                      "2.4E-3" }); // Wilson20

using namespace std;

/** @brief Read a whole file
 * @param in The opened file
 * @return The file content
 */
std::string
slurp(std::ifstream& in)
{
	std::ostringstream sstr;
	sstr << in.rdbuf();
	return sstr.str();
}

/** @brief Replaces a substring
 * @param subject The string to manipulate
 * @param search The term to replace
 * @param replace The replacement
 * @return The manipulated string
 */
std::string
ReplaceString(std::string subject,
              const std::string& search,
              const std::string& replace)
{
	size_t pos = 0;
	while ((pos = subject.find(search, pos)) != std::string::npos) {
		subject.replace(pos, search.length(), replace);
		pos += replace.length();
	}
	return subject;
}

/** @brief Pendulum motion
 * @return true if the test worked, false otherwise
 */
bool
pendulum(std::string tscheme, std::string dt)
{
	std::cout << tscheme << ", dt = " << dt << " s" << std::endl;
	std::cout << "================" << std::endl << std::endl;
	// Read the input generic file
	std::ifstream ifile("Mooring/time_schemes.txt");
	std::string text = slurp(ifile);
	ifile.close();
	// Replace the scheme and the dt
	text = ReplaceString(text, "@TSCHEME@", tscheme);
	text = ReplaceString(text, "@DT@", dt);
	// Save it somewhere
	std::stringstream filepath;
	filepath << fs::temp_directory_path().string() << "/" << tscheme << ".txt";
	std::ofstream ofile(filepath.str());
	ofile << text;
	ofile.close();

	MoorDyn system = MoorDyn_Create(filepath.str().c_str());
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	int err;
	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0.0, dt_val = std::stod(dt);
	while (t < 10.0) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt_val);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring step: " << err << endl;
			return false;
		}
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
	for (unsigned int i = 0; i < schemes.size(); i++) {
		const auto before = chrono::system_clock::now();
		if (!pendulum(schemes[i], dts[i]))
			return 1;
		const chrono::duration<double> duration =
		    chrono::system_clock::now() - before;
		cout << endl
		     << "*** " << schemes[i] << " = " << duration.count() << "s" << endl
		     << endl;
	}
	return 0;
}
