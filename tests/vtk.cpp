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
 * Tests on the VTK output files
 */

#include "MoorDyn2.h"
#include <iostream>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

bool
write_vtk_by_instances()
{
	std::cout << "*** Writing VTK files..." << std::endl;
	MoorDyn system = MoorDyn_Create("Mooring/BodiesAndRods.dat");
	if (!system) {
		std::cerr << "Failure Creating the Mooring system" << std::endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof) {
		std::cerr << "No DOFs were expected, but " << n_dof << " were reported"
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring initialization: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	unsigned int n_bodies;
	err = MoorDyn_GetNumberBodies(system, &n_bodies);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the number of bodies: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	for (unsigned int body_i = 1; body_i <= n_bodies; body_i++) {
		auto body = MoorDyn_GetBody(system, body_i);
		if (!body) {
			std::cerr << "Failure getting the body " << body_i << std::endl;
			MoorDyn_Close(system);
			return false;
		}
		std::stringstream filepath;
		filepath << fs::temp_directory_path().string() << "/"
		         << "vtk_body_" << body_i << ".00000.vtp";
		std::cout << "***     Saving on '" << filepath.str().c_str() << "'..."
		          << std::endl;

		err = MoorDyn_SaveBodyVTK(body, filepath.str().c_str());
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure saving the body file '"
			          << filepath.str().c_str() << "':" << err << std::endl;
			MoorDyn_Close(system);
			return false;
		}
	}

	unsigned int n_rods;
	err = MoorDyn_GetNumberRods(system, &n_rods);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the number of rods: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	for (unsigned int rod_i = 1; rod_i <= n_rods; rod_i++) {
		auto rod = MoorDyn_GetRod(system, rod_i);
		if (!rod) {
			std::cerr << "Failure getting the rod " << rod_i << std::endl;
			MoorDyn_Close(system);
			return false;
		}
		std::stringstream filepath;
		filepath << fs::temp_directory_path().string() << "/"
		         << "vtk_rod_" << rod_i << ".00000.vtp";
		std::cout << "***     Saving on '" << filepath.str().c_str() << "'..."
		          << std::endl;

		err = MoorDyn_SaveRodVTK(rod, filepath.str().c_str());
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure saving the rod file '"
			          << filepath.str().c_str() << "':" << err << std::endl;
			MoorDyn_Close(system);
			return false;
		}
	}

	unsigned int n_points;
	err = MoorDyn_GetNumberPoints(system, &n_points);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the number of points: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	for (unsigned int point_i = 1; point_i <= n_points; point_i++) {
		auto point = MoorDyn_GetPoint(system, point_i);
		if (!point) {
			std::cerr << "Failure getting the point " << point_i << std::endl;
			MoorDyn_Close(system);
			return false;
		}
		std::stringstream filepath;
		filepath << fs::temp_directory_path().string() << "/"
		         << "vtk_point_" << point_i << ".00000.vtp";
		std::cout << "***     Saving on '" << filepath.str().c_str() << "'..."
		          << std::endl;

		err = MoorDyn_SavePointVTK(point, filepath.str().c_str());
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure saving the point file '"
			          << filepath.str().c_str() << "':" << err << std::endl;
			MoorDyn_Close(system);
			return false;
		}
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
		         << "vtk_line_" << line_i << ".00000.vtp";
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

bool
write_vtk_system()
{
	std::cout << "*** Writing VTK file..." << std::endl;
	MoorDyn system = MoorDyn_Create("Mooring/BodiesAndRods.dat");
	if (!system) {
		std::cerr << "Failure Creating the Mooring system" << std::endl;
		return false;
	}

	unsigned int n_dof;
	if (MoorDyn_NCoupledDOF(system, &n_dof) != MOORDYN_SUCCESS) {
		MoorDyn_Close(system);
		return false;
	}
	if (n_dof) {
		std::cerr << "No DOFs were expected, but " << n_dof << " were reported"
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	int err;
	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring initialization: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	std::stringstream filepath;
	filepath << fs::temp_directory_path().string() << "/"
	         << "vtk_system.00000.vtm";
	std::cout << "***     Saving on '" << filepath.str().c_str() << "'..."
	          << std::endl;
	err = MoorDyn_SaveVTK(system, filepath.str().c_str());
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure saving the file '" << filepath.str().c_str()
		          << "':" << err << std::endl;
		MoorDyn_Close(system);
		return false;
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
	if (!write_vtk_by_instances())
		return 1;
	if (!write_vtk_system())
		return 2;
	return 0;
}
