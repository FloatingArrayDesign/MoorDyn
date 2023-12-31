#include <cmath>
#include "MoorDyn2.h"
#include <catch2/catch_test_macros.hpp>

#define DUPLICATED_TOL 1e-4

double compare_lines(MoorDynLine line1, MoorDynLine line2)
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
