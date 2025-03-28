#include <cmath>
#include <iostream>
#include <chrono>
#include "MoorDyn2.h"
#include <catch2/catch_test_macros.hpp>

#define DUPLICATED_TOL 1e-4
#define HANGING_TOL 1e-1

double
compare_lines(MoorDynLine line1, MoorDynLine line2)
{
	unsigned int n1, n2;
	REQUIRE(MoorDyn_GetLineN(line1, &n1) == MOORDYN_SUCCESS);
	REQUIRE(MoorDyn_GetLineN(line2, &n2) == MOORDYN_SUCCESS);
	REQUIRE(((n1 != n2) && (n2 % n1 == 0)));
	unsigned int m = n2 / n1;
	double error = 0.0;
	for (unsigned int i = 0; i < n1; i++) {
		double pos1[3], pos2[3];
		REQUIRE(MoorDyn_GetLineNodePos(line1, i, pos1) == MOORDYN_SUCCESS);
		REQUIRE(MoorDyn_GetLineNodePos(line2, i * m, pos2) == MOORDYN_SUCCESS);
		const double e_x = pos2[0] - pos1[0];
		const double e_z = pos2[2] - pos1[2];
		const double e = sqrt(e_x * e_x + e_z * e_z);
		if (error < e)
			error = e;
	}
	return error;
}

TEST_CASE("Duplicated line with different resolutions")
{
	MoorDyn system = MoorDyn_Create("Mooring/local_euler/duplicated.txt");
	REQUIRE(system);
	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	double x[3], dx[3];
	auto point = MoorDyn_GetPoint(system, 2);
	REQUIRE(point);
	REQUIRE(MoorDyn_GetPointPos(point, x) == MOORDYN_SUCCESS);
	std::fill(dx, dx + 3, 0.0);
	REQUIRE(MoorDyn_Init(system, x, dx) == MOORDYN_SUCCESS);

	auto line1 = MoorDyn_GetLine(system, 1);
	REQUIRE(line1);
	double l;
	REQUIRE(MoorDyn_GetLineUnstretchedLength(line1, &l) == MOORDYN_SUCCESS);
	auto line2 = MoorDyn_GetLine(system, 2);
	REQUIRE(line2);
	auto e0 = compare_lines(line1, line2);
	REQUIRE(e0 / l <= DUPLICATED_TOL);

	double f[3];
	double t = 0.0, dt = 1.0;
	dx[0] = -1.0;
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);

	auto e = compare_lines(line1, line2);
	REQUIRE(e / l <= DUPLICATED_TOL);

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

TEST_CASE("Hanging split line")
{
	MoorDyn system = MoorDyn_Create("Mooring/local_euler/hanging.txt");
	REQUIRE(system);
	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 6);

	double x[6], dx[6];
	for (unsigned int i = 0; i < 2; i++) {
		auto point = MoorDyn_GetPoint(system, i + 1);
		REQUIRE(point);
		REQUIRE(MoorDyn_GetPointPos(point, x + 3 * i) == MOORDYN_SUCCESS);
	}
	std::fill(dx, dx + 6, 0.0);
	REQUIRE(MoorDyn_Init(system, x, dx) == MOORDYN_SUCCESS);

	double f[6];
	double t = 0.0, dt = 1.0;
	dx[0] = 1.0;
	dx[3] = -1.0;
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);

	auto joint = MoorDyn_GetPoint(system, 3);
	REQUIRE(joint);
	REQUIRE(MoorDyn_GetPointPos(joint, x) == MOORDYN_SUCCESS);
	REQUIRE(std::abs(x[0]) < HANGING_TOL);

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}

TEST_CASE("Complex system performance test")
{
	MoorDyn system = MoorDyn_Create("Mooring/local_euler/complex_system.txt");
	REQUIRE(system);
	unsigned int n_dof;
	REQUIRE(MoorDyn_NCoupledDOF(system, &n_dof) == MOORDYN_SUCCESS);
	REQUIRE(n_dof == 3);

	double x[3], dx[3];
	auto point = MoorDyn_GetPoint(system, 4);
	REQUIRE(point);
	REQUIRE(MoorDyn_GetPointPos(point, x) == MOORDYN_SUCCESS);
	std::fill(dx, dx + 3, 0.0);
	auto in_init = std::chrono::steady_clock::now();
	REQUIRE(MoorDyn_Init(system, x, dx) == MOORDYN_SUCCESS);
	auto out_init = std::chrono::steady_clock::now();
	auto dt_init =
	    std::chrono::duration_cast<
	        std::chrono::duration<double, std::ratio<1>>>(out_init - in_init)
	        .count();

	double f[6];
	double t = 0.0, dt = 50.0;
	auto in_step = std::chrono::steady_clock::now();
	REQUIRE(MoorDyn_Step(system, x, dx, f, &t, &dt) == MOORDYN_SUCCESS);
	auto out_step = std::chrono::steady_clock::now();
	auto dt_step =
	    std::chrono::duration_cast<
	        std::chrono::duration<double, std::ratio<1>>>(out_step - in_step)
	        .count();
	REQUIRE(dt_step < dt_init);

	REQUIRE(MoorDyn_Close(system) == MOORDYN_SUCCESS);
}
