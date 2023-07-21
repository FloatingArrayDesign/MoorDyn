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

/** @file rods.cpp
 * Minimal tests for rods in different situations
 */

#define _USE_MATH_DEFINES

#include "MoorDyn2.h"
#include "MoorDyn2.hpp"
#include "Misc.hpp"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include "util.h"

#define TOL 1e-2

using namespace std;

bool
added_mass()
{
	// Do the math for our expected acceleration given the mass and that
	// Ca = 1.0
	double length = 2.0;
	double mass = 100 * length;
	double r = 0.25;
	double V = length * M_PI * r * r;
	double rho_w = 1025;
	double g = 9.8;
	double buoyancy = rho_w * V * g;
	double weight = mass * g;
	double Fnet = buoyancy - weight;
	double added_mass = 1.0 * V * rho_w;
	double acceleration = Fnet / (mass + added_mass);

	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/rod_tests/AddedMass.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, Tmax = 2.5, dt = 0.1;

	while (t < Tmax) {
		err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure during the mooring initialization: " << err
			     << endl;
			return false;
		}

		const auto rod = MoorDyn_GetRod(system, 1);

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the number of nodes for rod " << 1 << ": "
			     << err << endl;
			return false;
		}

		double expected_z = -10 + 0.5 * acceleration * t * t;
		double pos[3];
		for (unsigned int i = 0; i < n_nodes; i++) {

			err = MoorDyn_GetRodNodePos(rod, i, pos);
			if (err != MOORDYN_SUCCESS) {
				cerr << "Failure getting the position of nodes " << i
				     << " for rod 1"
				     << ": " << err << endl;
				return false;
			}
			if (!isclose(pos[2], expected_z, 1e-8, 1e-10)) {
				cerr << "Node " << i << " of Rod 1 should have a z position of "
				     << expected_z << " but has a z pos of " << pos[2]
				     << " at t=" << t << endl;
				return false;
			}
		}
		// when the rod get higher than z = -1, stop simulating
		if (expected_z > -1.0) {
			break;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return true;
}

moordyn::mat
fastRodOrMat(moordyn::vec3 q)
{

	// this code computes the rotation matrix between the X axis and rod
	// axis vector q. See the function exactMassMatrix to see
	// the un-simplified version of this math that computes the rotation matrix
	// between the x unit vector and q. This version makes use of the knowledge
	// that the first vector is {1, 0, 0} to simplify and reduce the math
	// significantly.
	// This could potentially be used inside of Rod.cpp, but what is currently
	// there works fine and while it is slower than this, it's not an actual
	// performance bottleneck.
	moordyn::mat OrMat;
	if (abs(q.y()) > 1e-8 || abs(q.z()) > 1e-8) {
		moordyn::real scale = 1 / (1 + q.x());
		moordyn::real ny = 1 - (q.y() * q.y() * scale);
		moordyn::real nz = 1 - (q.z() * q.z() * scale);
		moordyn::real b = -(q.y() * q.z() * scale);
		// clang-format off
		OrMat << q.x(), -q.y(), -q.z(),
				 q.y(),     ny,      b,
				 q.z(),      b,     nz;
		// clang-format on
	} else {
		OrMat = moordyn::mat::Identity();
	}
	return OrMat;
}
moordyn::mat6
exactMassMatrix(moordyn::vec3 r0, moordyn::vec3 rN, double w, double d)
{
	moordyn::vec3 q = rN - r0;

	moordyn::real length = q.norm();
	q = q.normalized();
	double mass = w * length;
	moordyn::vec3 cg = 0.5 * length * q;
	moordyn::mat H = moordyn::getH(cg);
	moordyn::mat massMat = mass * moordyn::mat::Identity();
	moordyn::mat6 M6net = moordyn::mat6::Zero();
	M6net.topLeftCorner<3, 3>() += massMat;
	const moordyn::mat tempM1 = massMat * H;
	M6net.bottomLeftCorner<3, 3>() += tempM1;
	M6net.topRightCorner<3, 3>() += tempM1.transpose();
	// this mass doesn't affect the inertia matrix because
	// that gets handled below

	// add inertia terms for the Rod assuming it is uniform density (radial
	// terms add to existing matrix which contains parallel-axis-theorem
	// components only)
	moordyn::mat Imat_l = moordyn::mat::Zero();
	// axial moment of inertia
	moordyn::real I_l = mass / 8.0 * d * d;
	// this is just the analytical equation for moment of inertia of
	// a uniform cylinder around its end.
	auto r = d / 2.0;
	moordyn::real I_r =
	    0.25 * mass * r * r + (1.0 / 3.0) * mass * length * length;

	Imat_l(0, 0) = I_l;
	Imat_l(1, 1) = I_r;
	Imat_l(2, 2) = I_r;

	// get rotation matrix to put things in global rather than rod-axis
	// orientations
	moordyn::mat OrMat = moordyn::mat::Identity();
	const moordyn::vec v = moordyn::vec::UnitX().cross(q);
	// std::cout << "v = " << v << endl;
	if (v.any()) {
		const moordyn::real c = moordyn::vec::UnitX().dot(q);
		const moordyn::real s = v.norm();
		const moordyn::mat kmat = moordyn::getH(v);
		OrMat += kmat + kmat * kmat * ((1 - c) / (s * s));
	}
	// cout << "exactOrMat =\n" << OrMat << endl;
	const moordyn::mat Imat = moordyn::rotateMass(OrMat, Imat_l);
	M6net.bottomRightCorner<3, 3>() += Imat;
	return M6net;
}
bool
rod_inertia()
{
	// Do the math for our expected acceleration given the mass and that
	// Ca = 1.0
	double rod_w = 100;
	double r = 0.25;
	bool didPass = true;

	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/rod_tests/RodInertia.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	for (int rodNum = 1; rodNum <= 8; rodNum++) {
		const auto rod = MoorDyn_GetRod(system, rodNum);

		unsigned int n_segs;
		err = MoorDyn_GetRodN(rod, &n_segs);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the number of nodes for rod " << 1 << ": "
			     << err << endl;
			return false;
		}

		moordyn::vec3 r0, rN;
		err = MoorDyn_GetRodNodePos(rod, 0, r0.data());
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the position of node 0"
			     << " for rod " << rodNum << ": " << err << endl;
			return false;
		}

		err = MoorDyn_GetRodNodePos(rod, n_segs, rN.data());
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the position of node N"
			     << " for rod " << rodNum << ": " << err << endl;
			return false;
		}
		// cout << "r0 = " << r0.transpose() << endl;
		// cout << "rN = " << rN.transpose() << endl;

		moordyn::mat6 trueMassMat = exactMassMatrix(r0, rN, rod_w, r * 2);
		// std::cout << "trueMass mat = \n" << trueMassMat << endl;
		moordyn::vec6 fnet;
		moordyn::mat6 M6net;
		((moordyn::Rod*)(rod))->getNetForceAndMass(fnet, M6net);

		// std::cout << "M6net = \n" << M6net << endl;
		if (!allclose(trueMassMat, M6net, 1e-8, 1e-10)) {
			cout << "Rod #" << rodNum << " had an incorrect mass matrix"
			     << endl;
			cout << "Should have been \n" << trueMassMat << endl;
			cout << "But was \n" << M6net << endl;
			didPass = false;
			// return false;
		}
	}

	err = MoorDyn_Close(system);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure closing Moordyn: " << err << endl;
		return false;
	}

	return didPass;
}

/** @brief Pinned rod horizontally lying, which should float towards the
 * vertical direction.
 *
 * Since there is no damping forces, the rods will be oscillating from one side
 * to the other
 */
bool
pinned_floating()
{
	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/RodPinnedFloating.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	unsigned int n_rods;
	err = MoorDyn_GetNumberRods(system, &n_rods);
	for (unsigned int i_rod = 1; i_rod <= n_rods; i_rod++) {
		const auto rod = MoorDyn_GetRod(system, i_rod);
		if (!rod) {
			cerr << "Failure getting the rod " << i_rod << endl;
			return false;
		}

		unsigned int n_nodes;
		err = MoorDyn_GetRodN(rod, &n_nodes);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting the number of nodes for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		// Check that the rod is floating upwards
		double pos[3];
		err = MoorDyn_GetRodNodePos(rod, 0, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting first node position for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		cout << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
		const double z0 = pos[2];
		err = MoorDyn_GetRodNodePos(rod, n_nodes, pos);
		if (err != MOORDYN_SUCCESS) {
			cerr << "Failure getting last node position for rod " << i_rod
			     << ": " << err << endl;
			return false;
		}
		const double z1 = pos[2];
		cout << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;

		if (z1 < z0) {
			cerr << "The last node is below the first one for rod " << i_rod
			     << ": " << z1 << " vs. " << z0 << endl;
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

/** @brief Rod hanging from 2 identical ropes, which should move horizontally
 */
bool
hanging()
{
	int err;
	cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

	MoorDyn system = MoorDyn_Create("Mooring/RodHanging.txt");
	if (!system) {
		cerr << "Failure Creating the Mooring system" << endl;
		return false;
	}

	err = MoorDyn_Init(system, NULL, NULL);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	double t = 0, T = 10.0;
	err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &T);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure during the mooring initialization: " << err << endl;
		return false;
	}

	const auto rod = MoorDyn_GetRod(system, 1);
	if (!rod) {
		cerr << "Failure getting the rod" << endl;
		return false;
	}

	unsigned int n_nodes;
	err = MoorDyn_GetRodN(rod, &n_nodes);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting the number of nodes: " << err << endl;
		return false;
	}
	// Check that the rod is is not rotating
	const double l = 20.0;
	double posa[3], posb[3];
	err = MoorDyn_GetRodNodePos(rod, 0, posa);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting first node position: " << err << endl;
		return false;
	}
	cout << posa[0] << ", " << posa[1] << ", " << posa[2] << endl;
	err = MoorDyn_GetRodNodePos(rod, n_nodes, posb);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Failure getting last node position: " << err << endl;
		return false;
	}
	cout << posb[0] << ", " << posb[1] << ", " << posb[2] << endl;

	if (((posa[0] + 0.5 * l) / l > TOL) || ((posb[0] - 0.5 * l) / l > TOL) ||
	    (fabs(posa[1]) / l > TOL) || (fabs(posb[1]) / l > TOL) ||
	    (fabs(posa[2] - posb[2]) / l > TOL)) {
		cerr << "The rod is rotating" << endl;
		return false;
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
	if (!pinned_floating())
		return 1;
	if (!hanging())
		return 2;
	if (!added_mass())
		return 3;
	if (!rod_inertia())
		return 4;

	cout << "rods.cpp passed successfully" << endl;
	return 0;
}
