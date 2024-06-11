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

/** @file io.cpp
 * Tests on the input/output interface, moordyn::io
 */

#ifndef WIN32
// Until we check the C++ API in Windows
#include "IO.hpp"
#endif
#include "MoorDyn2.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;
#ifndef WIN32
// Until we check the C++ API in Windows
using namespace moordyn;
#endif

#define LISTS_LENGTH 32
#define COMPARE_LISTS(v1, v2)                                                  \
	if (v1.size() != v2.size())                                                \
		return false;                                                          \
	for (unsigned int i = 0; i < v1.size(); i++)                               \
		if (v1[i] != v2[i])                                                    \
			return false;

#ifdef WIN32
// Until we check the C++ API in Windows
bool
io_class()
{
	return true;
}
#else
class IOTester : public io::IO
{
  public:
	IOTester(Log* log)
	  : IO(log)
	  , i(-3)
	  , ui(1024)
	  , r(37.431)
	  , v({ 0.0, 1.0, 2.0 })
	  , v6({ 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 })
	  , m({ { 3.0, 4.5, -8.0 },
	        { 10.0, -256.0, -1024.341 },
	        { 32.4, 55.7, 812309765.2 } })
	  , m6({ { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 } })
	{
		for (unsigned int i = 0; i < LISTS_LENGTH; i++) {
			lv.push_back(v);
			lv6.push_back(v6);
			lm.push_back(m);
			lm6.push_back(m6);
		}
	}

	~IOTester() {}

	void clear()
	{
		i = 0;
		ui = 0;
		r = 0.0;
		v = vec::Zero();
		v6 = vec6::Zero();
		m = mat::Zero();
		m6 = mat6::Zero();
		lv.clear();
		lv6.clear();
		lm.clear();
		lm6.clear();
	}

	virtual std::vector<uint64_t> Serialize(void)
	{
		std::vector<uint64_t> data, subdata;
		data.push_back(io::IO::Serialize((int64_t)i));
		data.push_back(io::IO::Serialize((uint64_t)ui));
		data.push_back(io::IO::Serialize(r));
		subdata = io::IO::Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(v6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(m);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(m6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lv);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lv6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lm);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lm6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		return data;
	}

	virtual uint64_t* Deserialize(const uint64_t* data)
	{
		uint64_t* ptr = (uint64_t*)data;
		int64_t ii;
		ptr = io::IO::Deserialize(ptr, ii);
		i = (int)ii;
		int64_t uii;
		ptr = io::IO::Deserialize(ptr, uii);
		ui = (unsigned int)ii;
		ptr = io::IO::Deserialize(ptr, r);
		ptr = io::IO::Deserialize(ptr, v);
		ptr = io::IO::Deserialize(ptr, v6);
		ptr = io::IO::Deserialize(ptr, m);
		ptr = io::IO::Deserialize(ptr, m6);
		ptr = io::IO::Deserialize(ptr, lv);
		ptr = io::IO::Deserialize(ptr, lv6);
		ptr = io::IO::Deserialize(ptr, lm);
		ptr = io::IO::Deserialize(ptr, lm6);
		return ptr;
	}

	IOTester& operator=(const IOTester& visitor)
	{
		i = visitor.i;
		ui = visitor.ui;
		r = visitor.r;
		v = visitor.v;
		v6 = visitor.v6;
		m = visitor.m;
		m6 = visitor.m6;
		lv = visitor.lv;
		lv6 = visitor.lv6;
		lm = visitor.lm;
		lm6 = visitor.lm6;
		return *this;
	}

	bool operator==(const IOTester& visitor)
	{
		if (i != visitor.i)
			return false;
		if (ui != visitor.ui)
			return false;
		if (r != visitor.r)
			return false;
		if (v != visitor.v)
			return false;
		if (v6 != visitor.v6)
			return false;
		if (m != visitor.m)
			return false;
		if (m6 != visitor.m6)
			return false;
		COMPARE_LISTS(lv, visitor.lv);
		COMPARE_LISTS(lv6, visitor.lv6);
		COMPARE_LISTS(lm, visitor.lm);
		COMPARE_LISTS(lm6, visitor.lm6);
		return true;
	}

  private:
	int i;
	unsigned int ui;
	moordyn::real r;
	moordyn::vec v;
	moordyn::vec6 v6;
	moordyn::mat m;
	moordyn::mat6 m6;
	std::vector<moordyn::vec> lv;
	std::vector<moordyn::vec6> lv6;
	std::vector<moordyn::mat> lm;
	std::vector<moordyn::mat6> lm6;
};

bool
io_class()
{
	// First basic test, check that we can serialize and deserialize
	std::cout << "*** Serialize -> Deserialize..." << std::endl;
	Log dummy_log;
	IOTester src(&dummy_log), dst(&dummy_log);
	dst.clear();
	auto data = src.Serialize();
	dst.Deserialize(data.data());
	if (src == dst) {
		cerr << "The deserialized data does not match the original"
		     << std::endl;
		return false;
	}
	std::cout << "***  OK!" << std::endl;

	// Now try saving and loading
	std::cout << "*** Save -> Load..." << std::endl;
	std::stringstream filepath;
	filepath << fs::temp_directory_path().string() << "/"
	         << "test.moordyn";
	dst.clear();
	src.Save(filepath.str());
	dst.Load(filepath.str());
	if (src == dst) {
		std::cerr << "The loaded data does not match the original" << std::endl;
		return false;
	}
	std::cout << "***  OK!" << std::endl;

	return true;
}
#endif

bool
skip_ic()
{
	std::cout << "*** Skip initial condition..." << std::endl;
	// We first run the system in the regular way, but saving the state after
	// calling MoorDyn_Init()

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
		auto point = MoorDyn_GetPoint(system, i + 4);
		err = MoorDyn_GetPointPos(point, x + 3 * i);
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

	std::stringstream filepath;
	filepath << fs::temp_directory_path().string() << "/"
	         << "minimal.moordyn";
	err = MoorDyn_Save(system, filepath.str().c_str());
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure saving the mooring system: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	double f[9];
	double t = 0.0, dt = 0.5;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring step: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	// Now we create and run a second system, this time loading it from the
	// saved snapshot above
	MoorDyn system2 = MoorDyn_Create("Mooring/lines.txt");
	if (!system2) {
		std::cerr << "Failure Creating the second Mooring system" << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	err = MoorDyn_Init_NoIC(system2, x, dx);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the second mooring initialization: " << err
		          << std::endl;
		MoorDyn_Close(system);
		MoorDyn_Close(system2);
		return false;
	}

	err = MoorDyn_Load(system2, filepath.str().c_str());
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure loading the mooring system: " << err << std::endl;
		MoorDyn_Close(system);
		MoorDyn_Close(system2);
		return false;
	}

	t = 0.0;
	err = MoorDyn_Step(system2, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the second mooring step: " << err
		          << std::endl;
		MoorDyn_Close(system);
		MoorDyn_Close(system2);
		return false;
	}

	// Check that the nodes positions match between both systems
	unsigned int n_lines;
	err = MoorDyn_GetNumberLines(system, &n_lines);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the number of lines: " << err
		          << std::endl;
		MoorDyn_Close(system);
		MoorDyn_Close(system2);
		return false;
	}
	for (unsigned int line_i = 1; line_i <= n_lines; line_i++) {
		auto line = MoorDyn_GetLine(system, line_i);
		auto line2 = MoorDyn_GetLine(system2, line_i);
		unsigned int n_nodes;
		err = MoorDyn_GetLineNumberNodes(line, &n_nodes);
		if (err != MOORDYN_SUCCESS) {
			std::cerr << "Failure getting the number of nodes: " << err
			          << std::endl;
			MoorDyn_Close(system);
			MoorDyn_Close(system2);
			return false;
		}
		for (unsigned int node_i = 0; node_i < n_nodes; node_i++) {
			double pos[3], pos2[3];
			err = MoorDyn_GetLineNodePos(line, node_i, pos);
			if (err != MOORDYN_SUCCESS) {
				std::cerr << "Failure getting the node position: " << err
				          << std::endl;
				MoorDyn_Close(system);
				MoorDyn_Close(system2);
				return false;
			}
			err = MoorDyn_GetLineNodePos(line2, node_i, pos2);
			if (err != MOORDYN_SUCCESS) {
				std::cerr << "Failure getting the node position: " << err
				          << std::endl;
				MoorDyn_Close(system);
				MoorDyn_Close(system2);
				return false;
			}
			for (unsigned int i = 0; i < 3; i++) {
				if (pos[i] != pos2[i]) {
					std::cerr << "Line " << line_i << ", node " << node_i
					          << ", coord " << i << ": " << pos[i]
					          << " != " << pos2[i] << std::endl;
					MoorDyn_Close(system);
					MoorDyn_Close(system2);
					return false;
				}
			}
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure closing Moordyn: " << err << std::endl;
		MoorDyn_Close(system2);
		return false;
	}
	err = MoorDyn_Close(system2);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure closing second Moordyn: " << err << std::endl;
		return false;
	}
	std::cout << "***  OK!" << std::endl;

	return true;
}

bool
restore()
{
	std::cout << "*** Restore to a previous state..." << std::endl;
	// We first run the system in the regular way, then we restore it and run
	// again

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
		auto point = MoorDyn_GetPoint(system, i + 4);
		err = MoorDyn_GetPointPos(point, x + 3 * i);
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

	// We backup the system now
	size_t backup_size = 0;
	uint64_t* backup = NULL;
	err = MoorDyn_Serialize(system, &backup_size, NULL);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure getting the serialization size: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	backup = (uint64_t*)malloc(backup_size);
	if (!backup) {
		std::cerr << "Failure allocating " << backup_size << "bytes"
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	err = MoorDyn_Serialize(system, NULL, backup);

	// First run
	double f[9];
	double t = 0.0, dt = 0.5;
	err = MoorDyn_Step(system, x, dx, f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the mooring step: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}

	// Now we restore the system and run again
	err = MoorDyn_Deserialize(system, backup);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure while restoring the system: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	free(backup);
	double f2[9];
	t = 0.0;
	err = MoorDyn_Step(system, x, dx, f2, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure during the repeated mooring step: " << err
		          << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	for (unsigned int i = 0; i < 9; i++) {
		if (f[i] != f2[i]) {
			std::cerr << "Force missmatch at component " << i << ": " << f[i]
			          << " != " << f2[i] << std::endl;
			MoorDyn_Close(system);
			return false;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "Failure closing Moordyn: " << err << std::endl;
		MoorDyn_Close(system);
		return false;
	}
	std::cout << "***  OK!" << std::endl;

	return true;
}

int
main(int, char**)
{
	if (!io_class())
		return 1;
	if (!skip_ic())
		return 1;
	if (!restore())
		return 1;
	return 0;
}
