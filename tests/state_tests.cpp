#include <iostream>
#include <sstream>

#include "Misc.hpp"
#include "Log.hpp"
#include "Point.hpp"
#include "Line.hpp"
#include "State.hpp"
#include <catch2/catch_test_macros.hpp>
#include "catch2/matchers/catch_matchers_templated.hpp"
#include "util.h"

// md::real is faster to type
namespace md = moordyn;

md::Point*
CreatePoint(md::Log* log, EnvCondRef env)
{
	md::Point* obj = new md::Point(log, 0);
	obj->setup(0,
	           md::Point::FIXED,
	           md::vec::Zero(),
	           0,
	           0,
	           md::vec::Zero(),
	           0.0,
	           0.0,
	           env);
	return obj;
}

md::Line*
CreateLine(md::Log* log,
           EnvCondRef env,
           unsigned int n,
           shared_ptr<ofstream> outfile)
{
	md::Line* obj = new md::Line(log, 0);
	LineProps props;
	props.type = "main";
	props.d = 0.1;
	props.w = 100.;
	props.ElasticMod = 1;
	props.EA = 3.e8;
	props.BA = -1.0;
	props.EI = 0.0;
	props.Cdn = 1.0;
	props.Can = 1.0;
	props.Cdt = 0.0;
	props.Cat = 0.0;
	props.Cl = 0.0;
	props.nEApoints = 0;
	props.nBApoints = 0;
	props.nEIpoints = 0;
	obj->setup(0, &props, 1.e3, n, env, outfile, "", 0);
	return obj;
}

#define SYS_STARTER                                                            \
	md::Log* log = new md::Log(MOORDYN_DBG_LEVEL);                             \
	EnvCondRef env = std::make_shared<EnvCond>();                              \
	env->g = 9.81;                                                             \
	env->WtrDpth = 0.;                                                         \
	env->rho_w = 1025.;                                                        \
	env->kb = 3.0e6;                                                           \
	env->cb = 3.0e5;                                                           \
	env->WriteUnits = 1;                                                       \
	env->writeLog = 0;                                                         \
	env->FrictionCoefficient = 0.0;                                            \
	env->FricDamp = 200.0;                                                     \
	env->StatDynFricScale = 1.0;                                               \
	shared_ptr<ofstream> outfile = make_shared<ofstream>("state_tests.out");

#define SYS_KILLER                                                             \
	outfile->close();                                                          \
	delete log;

TEST_CASE("State generation")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state(log);
	state.addInstance(p1);
	state.addInstance(p2);
	state.addInstance(l);

	REQUIRE(state.get().rows() == 3);
	REQUIRE(state.get()(0).rows() == 1);
	REQUIRE(state.get()(0).cols() == 6);
	REQUIRE(state.get()(1).rows() == 1);
	REQUIRE(state.get()(1).cols() == 6);
	REQUIRE(state.get()(2).rows() == 19);
	REQUIRE(state.get()(2).cols() == 6);
	REQUIRE(state.get(p1).rows() == 1);
	REQUIRE(state.get(p1).cols() == 6);
	REQUIRE(state.get(p2).rows() == 1);
	REQUIRE(state.get(p2).cols() == 6);
	REQUIRE(state.get(l).rows() == 19);
	REQUIRE(state.get(l).cols() == 6);

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("State initializing")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State r(log);
	r.addInstance(p1);
	r.addInstance(p2);
	r.addInstance(l);

	// Set the data
	r.get(p1).row(0).head<3>() = md::vec::Zero();
	r.get(p1).row(0).tail<3>() = md::vec::Zero();
	r.get(p2).row(0).head<3>() = md::vec(1.0, 2.0, 3.0);
	r.get(p2).row(0).tail<3>() = md::vec(4.0, 5.0, 6.0);
	md::real norm = 0.0;
	for (unsigned int i = 0; i < 19; i++) {
		norm += md::vec6(i, 2 * i, 4 * i, 5 * i, 6 * i, 7 * i).squaredNorm();
		r.get(l).row(i).head<3>() = md::vec(i, 2 * i, 4 * i);
		r.get(l).row(i).tail<3>() = md::vec(5 * i, 6 * i, 7 * i);
	}
	norm = sqrt(norm);

	REQUIRE(r.get(p1).norm() == 0.0);
	REQUIRE(isclose(r.get(p2).norm(),
	                md::vec6(1.0, 2.0, 3.0, 4.0, 5.0, 6.0).norm()));
	REQUIRE(isclose(r.get(l).norm(), norm));

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("Operating states")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State r(log), r2(log), r3(log);
	r.addInstance(p1);
	r.addInstance(p2);
	r.addInstance(l);
	r2.addInstance(p1);
	r2.addInstance(p2);
	r2.addInstance(l);
	r3.addInstance(p1);
	r3.addInstance(p2);
	r3.addInstance(l);

	// Set the data
	r.get(p1).row(0).head<3>() = md::vec::Zero();
	r.get(p1).row(0).tail<3>() = md::vec::Zero();
	r.get(p2).row(0).head<3>() = md::vec(1.0, 2.0, 3.0);
	r.get(p2).row(0).tail<3>() = md::vec(4.0, 5.0, 6.0);
	for (unsigned int i = 0; i < 19; i++) {
		r.get(l).row(i).head<3>() = md::vec(i, 2 * i, 4 * i);
		r.get(l).row(i).tail<3>() = md::vec(5 * i, 6 * i, 7 * i);
	}

	r2.get() = r.get() * 2.0;
	r3.get() = (r.get() - r2.get());
	r.get() += r3.get();

	REQUIRE(r.get(p1).norm() == 0.0);
	REQUIRE(r.get(p2).norm() == 0.0);
	REQUIRE(r.get(l).norm() == 0.0);

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}
