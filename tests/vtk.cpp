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

/** @file vtk.cpp
 * Tests on the VTK output files. This test is only compiled and executed if
 * USE_VTK=ON on the CMake configuration
 */

#include "MoorDyn2.h"
#include <iostream>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

bool
write_vtk_lines()
{
	std::cout << "*** Writing VTK files..." << std::endl;
	MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
	if (!system) {
		std::cerr << "Failure Creating the Mooring system" << std::endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof != 9) {
		std::cerr << "3x3 = 9 DOFs were expected, but " << n_dof
		          << "were reported" << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	double x[9], dx[9];
	// Get the initial positions from the config file
	for (unsigned int i = 0; i < 3; i++) {
		// 4 = first fairlead id
		auto conn = MoorDyn_GetConnection(system, i + 4);
		err = MoorDyn_GetConnectPos(conn, x + 3 * i);
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure retrieving the fairlead " << i + 4
			          << " position: " << err << std::endl;
			MoorDyn_Close(system);
			return false;
		}
	}
	std::fill(dx, dx + 9, 0.0);
	err = MoorDyn_Init(system, x, dx);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring initialization: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	unsigned int n_lines;
	err = MoorDyn_GetNumberLines(system, &n_lines);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the number of lines: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	for (unsigned int line_i = 1; line_i <= n_lines; line_i++) {
		auto line = MoorDyn_GetLine(system, line_i);
		if (!line) {
			std::cerr << "Failure getting the line " << line_i << std::endl;
			MoorDyn_Close(system);
			return false;
		}
		std::stringstream filepath;
		filepath << fs::temp_directory_path().string() << "/"
		         << "vtk_line_" << line_i << ".vtp";
		std::cout << "***     Saving on '" << filepath.str().c_str() << "'..."
		          << std::endl;

		err = MoorDyn_SaveLineVTK(line, filepath.str().c_str());
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure saving the line file '"
			          << filepath.str().c_str() << "':" << err << std::endl;
			MoorDyn_Close(system);
			return false;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure closing Moordyn: " << err << std::endl;
		return false;
	}
	std::cout << "***  OK!" << std::endl;

	return true;
}

int
main(int, char**)
{
	if (!write_vtk_lines())
		return 1;
	return 0;
}
