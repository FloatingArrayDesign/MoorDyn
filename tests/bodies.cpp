/*
 * Copyright (c) 2022 Alex Kinley, Jose Luis Cercos-Pita <jlc@core-marine.com>
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

/** @file rods.cpp
 * Minimal tests for rods in different situations
 */

#define _USE_MATH_DEFINES

#include "MoorDyn2.hpp"
#include "MoorDyn2.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>

#include "util.h"

using namespace std;

#ifdef USE_VTK

bool
write_system_vtk(MoorDyn in_system, double time, SeriesWriter* series_writer)
{
	// if no series writer, we do nothing but pretend to work
	if (series_writer == NULL) {
		return true;
	}

	moordyn::MoorDyn* system = (moordyn::MoorDyn*)(in_system);
	std::stringstream filename;
	std::stringstream element_name;
	element_name << "vtk_system";
	auto& vtp_series = series_writer->getSeries(element_name.str());
	auto step_num = vtp_series.time_steps.size();

	filename << element_name.str() << "." << step_num << ".vtm";
	std::string full_path = "../../vtk_out/" + filename.str();
	// std::cout << "***     Saving on '" << full_path << "'..." <<
	// std::endl;
	vtp_series.time_steps.push_back({ filename.str(), time });
	system->saveVTK(full_path.c_str());
	return true;
}

#else
class SeriesWriter;

bool
write_system_vtk(MoorDyn in_system, double time, SeriesWriter* series_writer)
{
	return true;
}

#endif

/**
 * @brief Represents a number of times and data for coupled DOFs
 *
 * Along with followTrajectory() makes it easier to move coupled DOFs along
 * predetermined trajectories
 *
 * @tparam T DOF data type (Probably an Eigen Array/Vector type)
 */
template<typename T>
struct Trajectory
{
	std::vector<double> times;
	std::vector<T> positions;

	/**
	 * @brief Calls the given function for each time between startT and endT
	 * (spaced dt apart), and stores the result in positions
	 *
	 * If you can write the coupled DOFs as a function of time, this allows you
	 * to make a trajectory out of them
	 * @tparam F
	 * @param startT First time
	 * @param endT Last time (included in the trajectory)
	 * @param dt Spacing between time steps
	 * @param f Callback
	 * @return Trajectory<T>
	 */
	template<class F>
	static Trajectory<T> fromLambda(double startT, double endT, double dt, F f)
	{
		double timeRange = endT - startT;
		// +1 to include endT
		double numTimes = 1 + round(timeRange / dt);
		std::vector<double> times(numTimes);
		std::vector<T> pos(numTimes);
		for (int i = 0; i < numTimes; i++) {
			double t = startT + (i * dt);
			times[i] = t;
			pos[i] = f(t);
		}
		return Trajectory<T>{ times, pos };
	}
};

/**
 * @brief Run the simulation with the given trajectory data for the coupled DOFs
 *
 * @tparam T
 * @param system MoorDyn system
 * @param trajectory The trajectory to use for the coupled DOFs
 * @param t Reference to the time, makes it easier to integrate into program
 * flow
 * @param series_writer Used to write out vtk series
 * @return true Success
 * @return false Failure
 */
template<typename T>
bool
followTrajectory(MoorDyn& system,
                 const Trajectory<T>& trajectory,
                 double& t,
                 SeriesWriter* series_writer)
{

	int err;
	int num_steps = static_cast<int>(trajectory.times.size());

	t = trajectory.times[0];

	for (int i = 0; i < (num_steps - 1); i++) {
		T x = trajectory.positions[i];
		T dx = trajectory.positions[i + 1] - x;

		std::vector<double> f(x.size());
		double dt = trajectory.times[i + 1] - trajectory.times[i];
		err = MoorDyn_Step(system, x.data(), dx.data(), f.data(), &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}

		if (!write_system_vtk(system, t, series_writer)) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Uses a line between a point on the body and a coupled point to rotate
 * the body around
 *
 *
 * The point on the body starts at (1, 0, 0) relative to the body.
 * Then we drag the point to (0, 0, 1) to rotate the body -90 degrees around the
 * y-axis Then we drag the point to (0, 1, 0) to rotate the body -90 degrees
 * around the x-axis Then we drag the point to back to (1, 0, 0) to rotate the
 * body 90 degrees around the z-axis
 *
 * The final result of this should be that the body gets rotated -90 degrees
 * around the x-axis.
 *
 *
 * @param series_writer
 * @return true
 * @return false
 */
bool
rotatingBody(SeriesWriter* series_writer)
{
	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/body_tests/rotatingBody.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	unsigned int n_dof;
	err = MoorDyn_NCoupledDOF(system, &n_dof);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting NCoupledDOF: " << err << endl;
		return false;
	}
	if (n_dof != 3) {
		cerr << "Expected 3 DOFs but got " << n_dof << endl;
		return false;
	}

	const moordyn::vec3 body_center{ 0, 0, 5 };
	const moordyn::real radius = 2.0;
	moordyn::vec3 x = (radius * moordyn::vec3{ 1, 0, 0 }) + body_center;
	moordyn::vec3 dx{ 0, 0, 0 };
	double f[3];

	err = MoorDyn_Init(system, x.data(), dx.data());
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	MoorDynPoint point = MoorDyn_GetPoint(system, 8);

	if (!write_system_vtk(system, 0, series_writer)) {
		return false;
	}

	double t = 0, dt = 0.1;
	// do one outer time step just to make sure everything is settled
	err = MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	if (!write_system_vtk(system, t, series_writer)) {
		return false;
	}
	double start_t = t;

	// goes from (2, 0, 0) to (0, 0, 2) in 2 seconds
	auto trajectory = Trajectory<
	    moordyn::
	        vec3>::fromLambda(start_t, start_t + 2.0, dt, [&](double time) {
		const double elapsed_time = time - start_t;
		const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		x = body_center + (radius * moordyn::vec3{ cos(angle), 0, sin(angle) });
		return x;
	});
	if (!followTrajectory(system, trajectory, t, series_writer)) {
		return false;
	}

	start_t = t;
	x = body_center + moordyn::vec3(0.0, 0.0, 0.05 + radius);
	dx = moordyn::vec3(0, 0, 0.0);
	// give 0.5 seconds to settle at the top
	while (t < start_t + 0.5) {
		err = MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}
		if (!write_system_vtk(system, t, series_writer)) {
			return false;
		}
	}

	moordyn::vec3 point_pos;
	MoorDyn_GetPointPos(point, point_pos.data());
	{
		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 0, 0, 1.0 };
		if (!allclose(rel_pos, expected_pos, 0.1, 0.15)) {
			cerr << "Point 8 relative to body center should be "
			     << expected_pos.transpose() << " but was "
			     << rel_pos.transpose() << " at t = " << t << endl;
		}
	}

	start_t = t;

	// goes from (0, 0, 2) to (0, 2, 0) in 2 seconds
	auto trajectory2 = Trajectory<
	    moordyn::
	        vec3>::fromLambda(start_t, start_t + 2.0, dt, [&](double time) {
		const double elapsed_time = time - start_t;
		const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		x = body_center + (radius * moordyn::vec3{ 0, sin(angle), cos(angle) });
		return x;
	});
	if (!followTrajectory(system, trajectory2, t, series_writer)) {
		return false;
	}

	start_t = t;
	x = body_center + moordyn::vec3(0.0, radius + 0.05, 0);
	dx = moordyn::vec3(0, 0.0, 0.0);
	// give 0.5 seconds to settle at the top
	while (t < start_t + 0.5) {
		err = MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}
		if (!write_system_vtk(system, t, series_writer)) {
			return false;
		}
	}

	MoorDyn_GetPointPos(point, point_pos.data());
	{

		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 0, 1.0, 0.0 };
		if (!allclose(rel_pos, expected_pos, 0.1, 0.15)) {
			cerr << "Point 8 relative to body center should be "
			     << expected_pos.transpose() << " but was "
			     << rel_pos.transpose() << " at t = " << t << endl;
		}
	}

	start_t = t;

	// goes from (0, 2, 0) to (2, 0, 0) in 2 seconds
	auto trajectory3 = Trajectory<
	    moordyn::
	        vec3>::fromLambda(start_t, start_t + 2.0, dt, [&](double time) {
		const double elapsed_time = time - start_t;
		const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		x = body_center + (radius * moordyn::vec3{ sin(angle), cos(angle), 0 });
		return x;
	});
	if (!followTrajectory(system, trajectory3, t, series_writer)) {
		return false;
	}

	start_t = t;
	x = body_center + moordyn::vec3(radius + 0.05, 0, 0);
	dx = moordyn::vec3(0, 0.0, 0.0);
	// give 0.1 seconds to settle at the top
	while (t < start_t + 0.5) {
		err = MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}
		if (!write_system_vtk(system, t, series_writer)) {
			return false;
		}
	}

	MoorDyn_GetPointPos(point, point_pos.data());
	{

		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 1.0, 0.0, 0.0 };
		if (!allclose(rel_pos, expected_pos, 0.1, 0.15)) {
			cerr << "Point 8 relative to body center should be "
			     << expected_pos.transpose() << "but was "
			     << rel_pos.transpose() << " at t=" << t << endl;
		}
	}

	auto body = MoorDyn_GetBody(system, 1);
	moordyn::vec6 r, rd;
	MoorDyn_GetBodyState(body, r.data(), rd.data());
	// We want axis-angle representation, and it's easier to compute that from
	// quaternion so we convert back to quaternion
	auto xyz_quat = moordyn::XYZQuat::fromVec6(r);
	auto q = xyz_quat.quat;
	// convert body orientation to axis-angle
	double angle = moordyn::rad2deg * 2 * acos(q.w());
	double denom = (sqrt(1 - q.w() * q.w()));
	moordyn::vec3 axis{ q.x() / denom, q.y() / denom, q.z() / denom };
	if (!(axis.x() > 0.85 && axis.y() < 0.2 && axis.z() < 0.2)) {
		cerr << "The final rotation of the body in angle axis form should "
		        "have an axis in the x direction, but axis is "
		     << axis.transpose() << endl;
		return false;
	}
	// normalize angle between +180 and -180
	while (angle > 180.) {
		angle -= 360;
	}
	while (angle < -180) {
		angle += 360;
	}
	if (!(abs(angle - (-90)) < 10)) {
		cerr << "The final rotation of the body in angle-axis form should "
		        "have a angle near 90 degrees but angle is "
		     << angle << endl;
		return false;
	}

	cout << "Body r = " << r.transpose() << endl;
	cout << "Axis-Angle rotation: axis = " << axis.transpose()
	     << ", angle = " << angle << " degrees" << endl;
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
	try {
		// SeriesWriter series_writer;
		if (!rotatingBody(NULL)) {
			// series_writer.writeJson("../../vtk_out/");
			return 3;
		}
		// series_writer.writeJson("../../vtk_out/");

	} catch (std::exception& e) {
		cerr << "rotatingBody failed with exception " << e.what() << endl;
		return 3;
	}

	cout << "bodies.cpp passed successfully" << endl;
	return 0;
}
