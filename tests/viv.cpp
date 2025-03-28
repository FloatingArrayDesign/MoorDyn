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

/** @file pendulum.cpp
 * Simple case of a pendulum motion
 */

// Visual studio still uses this
#define _USE_MATH_DEFINES

#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <catch2/catch_test_macros.hpp>

// NOTE: this is largely built on the pendulum test framework

#define TOL                                                                    \
	1.0e-2 // absolute tolerance. In setting this up, max error was 0.008069,
	       // but this can vary slightly with every simulation

bool
compare(double v1, double v2, double tol)
{
	return fabs(v1 - v2) <= tol;
}

void
check_value(const char* name, double v1, double v2, double tol, double t)
{
	if (!compare(v1, v2, tol)) {
		std::cerr << std::setprecision(8) << "Checking " << name
		          << " failed at t = " << t << " s. " << v1 << " was expected"
		          << " but " << v2 << " was computed" << std::endl;
		throw std::runtime_error("Missmatching values");
	}
}

using namespace std;

TEST_CASE("VIV frequency simulation and check")
{
	MoorDyn system = MoorDyn_Create("Mooring/viv/viv.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 0);

	REQUIRE(MoorDyn_Init(system, NULL, NULL) == MOORDYN_SUCCESS);

	// get the moordyn line object
	const MoorDynLine line1 = MoorDyn_GetLine(system, 1);
	REQUIRE(line1);

	const double T =
	    1 / (2 * 1.04); // Target VIV period, fairten frequency is 2x target CF
	                    // strain frequency of 1.04 Hz.
	double dt = 0.0001; // time step (same as dtM)
	double t = 0.0;     // time
	double ten = 0.0;   // tension (initialize as zero)
	std::vector<double> ten_peaks = { 0.0 }; // tension peaks?
	std::vector<double>
	    peak_times = {}; // tracking the time of peaks (empty to start, will be
	                     // filled in for every peak)
	double d_peak =
	    0.0; // last peak tracker. For checking if there is a change in the peak

	while (t < 15) { // run for 15 seconds
		REQUIRE(MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt) ==
		        MOORDYN_SUCCESS);

		REQUIRE(MoorDyn_GetLineFairTen(line1, &ten) == MOORDYN_SUCCESS);

		if (t > 10) { // only check period after 10 seconds. Before then, there
			          // isn't complete lock in
			// check for peaks in tension. These are used to determine the
			// period
			const double d = fabs(ten - ten_peaks.back()); // tension delta
			if (d > d_peak)
				d_peak = d;
			else {
				ten_peaks.push_back(ten);
				peak_times.push_back(t);
				d_peak = 0.0;
			}
		}
	}

	REQUIRE(peak_times.size() != 0);

	// Check period is correct
	for (unsigned int i = 2; i < peak_times.size(); i++) {
		INFO("Target Period: " << T << " s. Calculated period: "
		                       << peak_times[i] - peak_times[i - 2] << " s");
		// check that peak_times[i]-peak_times[i-2] is within TOL of T
		// note that peak_times[i]-peak_times[i-1] would be a half period
		REQUIRE_NOTHROW(check_value("Period",
		                            T,
		                            peak_times[i] - peak_times[i - 2],
		                            TOL,
		                            peak_times[i]));
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}
