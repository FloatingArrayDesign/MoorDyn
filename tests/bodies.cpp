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
#include <vector>
#include <numeric>
#include <catch2/catch_test_macros.hpp>
#include "util.h"

using namespace std;

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
 * @return true Success
 * @return false Failure
 */
template<typename T>
bool
followTrajectory(MoorDyn& system, const Trajectory<T>& trajectory, double& t)
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
		if (err != MOORDYN_SUCCESS)
			return false;
	}
	return true;
}

/**
 * @brief Uses a line between a point on the body and a coupled point to rotate
 * the body around
 *
 * The point on the body starts at (1, 0, 0) relative to the body.
 * Then we drag the point to (0, 0, 1) to rotate the body -90 degrees around the
 * y-axis Then we drag the point to (0, 1, 0) to rotate the body -90 degrees
 * around the x-axis Then we drag the point to back to (1, 0, 0) to rotate the
 * body 90 degrees around the z-axis
 *
 * The final result of this should be that the body gets rotated -90 degrees
 * around the x-axis.
 */
TEST_CASE("Rotating body")
{
	MoorDyn system = MoorDyn_Create("Mooring/body_tests/rotatingBody.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	const moordyn::vec3 body_center{ 0, 0, 5 };
	const moordyn::real radius = 2.0;
	moordyn::vec3 x = (radius * moordyn::vec3{ 1, 0, 0 }) + body_center;
	moordyn::vec3 dx{ 0, 0, 0 };
	double f[3];

	REQUIRE(MoorDyn_Init(system, x.data(), dx.data()) == MOORDYN_SUCCESS);

	MoorDynPoint point = MoorDyn_GetPoint(system, 8);
	REQUIRE(point);

	double t = 0, dt = 0.1;
	// do one outer time step just to make sure everything is settled
	REQUIRE(MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt) ==
	        MOORDYN_SUCCESS);
	double start_t = t;

	// goes from (2, 0, 0) to (0, 0, 2) in 2 seconds
	auto trajectory = Trajectory<moordyn::vec3>::fromLambda(
	    start_t, start_t + 2.0, dt, [&](double time) {
		    const double elapsed_time = time - start_t;
		    const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		    x = body_center +
		        (radius * moordyn::vec3{ cos(angle), 0, sin(angle) });
		    return x;
	    });
	REQUIRE(followTrajectory(system, trajectory, t));

	start_t = t;
	x = body_center + moordyn::vec3(0.0, 0.0, 0.05 + radius);
	dx = moordyn::vec3(0, 0, 0.0);
	// give 0.5 seconds to settle at the top
	while (t < start_t + 0.5) {
		REQUIRE(MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt) ==
		        MOORDYN_SUCCESS);
	}

	moordyn::vec3 point_pos;
	REQUIRE(MoorDyn_GetPointPos(point, point_pos.data()) == MOORDYN_SUCCESS);
	{
		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 0, 0, 1.0 };
		REQUIRE(allclose(rel_pos, expected_pos, 0.1, 0.15));
	}

	start_t = t;

	// goes from (0, 0, 2) to (0, 2, 0) in 2 seconds
	auto trajectory2 = Trajectory<moordyn::vec3>::fromLambda(
	    start_t, start_t + 2.0, dt, [&](double time) {
		    const double elapsed_time = time - start_t;
		    const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		    x = body_center +
		        (radius * moordyn::vec3{ 0, sin(angle), cos(angle) });
		    return x;
	    });
	REQUIRE(followTrajectory(system, trajectory2, t));

	start_t = t;
	x = body_center + moordyn::vec3(0.0, radius + 0.05, 0);
	dx = moordyn::vec3(0, 0.0, 0.0);
	// give 0.5 seconds to settle at the top
	while (t < start_t + 0.5) {
		REQUIRE(MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt) ==
		        MOORDYN_SUCCESS);
	}

	REQUIRE(MoorDyn_GetPointPos(point, point_pos.data()) == MOORDYN_SUCCESS);
	{

		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 0, 1.0, 0.0 };
		REQUIRE(allclose(rel_pos, expected_pos, 0.1, 0.15));
	}

	start_t = t;

	// goes from (0, 2, 0) to (2, 0, 0) in 2 seconds
	auto trajectory3 = Trajectory<moordyn::vec3>::fromLambda(
	    start_t, start_t + 2.0, dt, [&](double time) {
		    const double elapsed_time = time - start_t;
		    const double angle = 0.5 * (M_PI / 2.0) * elapsed_time;
		    x = body_center +
		        (radius * moordyn::vec3{ sin(angle), cos(angle), 0 });
		    return x;
	    });
	REQUIRE(followTrajectory(system, trajectory3, t));

	start_t = t;
	x = body_center + moordyn::vec3(radius + 0.05, 0, 0);
	dx = moordyn::vec3(0, 0.0, 0.0);
	// give 0.1 seconds to settle at the top
	while (t < start_t + 0.5) {
		REQUIRE(MoorDyn_Step(system, x.data(), dx.data(), f, &t, &dt) ==
		        MOORDYN_SUCCESS);
	}

	REQUIRE(MoorDyn_GetPointPos(point, point_pos.data()) == MOORDYN_SUCCESS);
	{

		moordyn::vec3 rel_pos = point_pos - body_center;
		moordyn::vec3 expected_pos{ 1.0, 0.0, 0.0 };
		REQUIRE(allclose(rel_pos, expected_pos, 0.1, 0.15));
	}

	auto body = MoorDyn_GetBody(system, 1);
	REQUIRE(body);
	moordyn::vec6 r, rd;
	REQUIRE(MoorDyn_GetBodyState(body, r.data(), rd.data()) == MOORDYN_SUCCESS);
	// We want axis-angle representation, and it's easier to compute that from
	// quaternion so we convert back to quaternion
	auto xyz_quat = moordyn::XYZQuat::fromVec6(r);
	auto q = xyz_quat.quat;
	// convert body orientation to axis-angle
	double angle = moordyn::rad2deg * 2 * acos(q.w());
	double denom = (sqrt(1 - q.w() * q.w()));
	moordyn::vec3 axis{ q.x() / denom, q.y() / denom, q.z() / denom };
	REQUIRE(
	    (abs(axis.x()) > 0.85 && abs(axis.y()) < 0.2 && abs(axis.z()) < 0.2));
	// normalize angle between +180 and -180
	while (angle > 180.) {
		angle -= 360;
	}
	while (angle < -180) {
		angle += 360;
	}
	REQUIRE(abs(angle - 90) < 10);

	cout << "Body r = " << r.transpose() << endl;
	cout << "Axis-Angle rotation: axis = " << axis.transpose()
	     << ", angle = " << angle << " degrees" << endl;
	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

/**
 * @brief Compares inertial deflection of a pinned body to analytical solution
 *
 * The coupled pinned body that is massless and volumeless with a rod fixed to
 * it is moved with constant acceleration in a vaccum (0 water density). The
 * resulting avg inertial deflection should match an analytical solution of
 * theta = arctan(-accel/gravity)
 *
 * This solution was derived both with Newtonian and LaGrangian mechanics.
 * It is the same as the pendulum on an accelerating cart problem.
 *
 * This only tests the inertial properties of pinned bodies, other tests deal
 * with hydrodynamics and general body properties
 */
TEST_CASE("Pinned body")
{
	MoorDyn system = MoorDyn_Create("Mooring/body_tests/pinnedBody.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 6);

	moordyn::vec6 x{ 0, 0, -5, 0, 0, 0 };
	moordyn::vec6 xd{ 0, 0, 0, 0, 0, 0 };
	double f[6];

	REQUIRE(MoorDyn_Init(system, x.data(), xd.data()) == MOORDYN_SUCCESS);

	auto body = MoorDyn_GetBody(system, 1);
	REQUIRE(body);
	moordyn::vec6 r, rd;
	vector<double> roll;
	int i = 0, j = 0;
	double t = 0.0, dt = 0.01, accel = 0.5;
	bool local_min_max;

	while (t < 50.0) {

		x[1] = 0.5 * accel * pow(t, 2);
		xd[1] = accel * t;
		REQUIRE(MoorDyn_Step(system, x.data(), xd.data(), f, &t, &dt) ==
		        MOORDYN_SUCCESS);

		REQUIRE(MoorDyn_GetBodyState(body, r.data(), rd.data()) ==
		        MOORDYN_SUCCESS);
		roll.push_back(r[3]);

		if (i >= 30) { // after the simulation has run for a few time steps
			// When local min or max of oscillation, indicates half of an
			// oscialltion has occured
			local_min_max = (((roll[i] - roll[i - 1]) / dt) *
			                 ((roll[i - 1] - roll[i - 2]) / dt)) < 0;
			if (local_min_max)
				j++;
		}
		if (j > 3)
			break; // after 2 full oscillations

		t = t + dt;
		i++;
	}
	double theta = atan(-accel / 9.80665);
	double average = reduce(roll.begin(), roll.end()) / roll.size();
	REQUIRE(abs(average - theta) <= 0.001);

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);

	// NOTE: jlcercos has dropped setprecision since it was not restored to the
	// default value afterwards, which might impair the subsequent exexutions
	cout << "Average roll is " << average << endl;
	cout << "Theoretical roll is " << theta << endl;
}

TEST_CASE("Body drag")
{
	MoorDyn system = MoorDyn_Create("Mooring/body_tests/bodyDrag.txt");
	REQUIRE(system);

	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 0);

	double f[3];

	REQUIRE(MoorDyn_Init(system, nullptr, nullptr) == MOORDYN_SUCCESS);

	double t = 0, dt = 0.1;
	double max_t = 5;
	while (t < max_t) {
		// do one outer time step just to make sure everything is settled
		REQUIRE(MoorDyn_Step(system, nullptr, nullptr, f, &t, &dt) ==
		        MOORDYN_SUCCESS);
	}

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

TEST_CASE("Excentric body")
{
	moordyn::MoorDyn system{ "Mooring/body_tests/orbitalBody.txt" };

	double f[3];
	const double radius = 10.0;
	const double omega = 0.2;
	const moordyn::vec r0 = moordyn::vec(0, 0, 20.0);
	auto bodies = system.GetBodies();
	for (auto body : bodies) {
		const auto [pos, vel] = body->getState();
		const moordyn::vec6 new_vel =
		    moordyn::vec6(0.0, 0.0, radius * omega, omega, 0.0, 0.0);
		body->r7 = pos;
		body->v6 = new_vel;
	}

	REQUIRE(system.Init(nullptr, nullptr) == MOORDYN_SUCCESS);

	double t = 0, dt = 0.25;
	double t_max = 10.0;

	// I do this just so that my first vtk output includes correct forces and
	// stuff
	double small_dt = 1e-6;
	REQUIRE(system.Step(NULL, NULL, f, t, small_dt) == MOORDYN_SUCCESS);
	while ((t_max - t) > (0.1 * dt)) {
		REQUIRE(system.Step(NULL, NULL, f, t, dt) == MOORDYN_SUCCESS);
		for (auto body : bodies) {
			const auto [pos, vel] = body->getState();
			const moordyn::vec3 global_pos = pos.pos;
			const moordyn::vec q = (global_pos - r0) / radius;
			const moordyn::real w = atan2(q.z(), q.y()) / t;
			const auto err_r = (global_pos - r0).norm() - radius;
			const auto err_w = atan2(q.z(), q.y()) / t - omega;

			// Check that we are orbiting at the right radius
			REQUIRE(abs(err_r) <= 1e-6 * radius);
			// Check that we are orbiting at the right velocity
			// NOTE: While orbiting around the rod the angular speed is really
			// stable. The same cannot be said of the orbital kinematics around
			// the point, where the velocity can be almost 0.3 rad/s
			REQUIRE(abs(err_w) <= 1e-6 * omega);
		}
	}
}
