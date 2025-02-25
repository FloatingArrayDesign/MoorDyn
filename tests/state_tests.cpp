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
CreateLine(md::Log* log, EnvCondRef env, unsigned int n,
           shared_ptr<ofstream> outfile)
{
	md::Line* obj = new md::Line(log, 0);
	LineProps props;
	props.type = "main";
	props.d = 0.1;
	props.w = 100.;
	props.EA = 3.e8;
	props.EI = 0.0;
	props.c = -1.0;
	props.Cdn = 1.0;
	props.Can = 1.0;
	props.Cdt = 0.0;
	props.Cat = 0.0;
	props.nEApoints = 0;
	props.nCpoints = 0;
	props.nEIpoints = 0;
	obj->setup(0,
			   &props,
			   1.e3,
			   n,
			   env,
			   outfile,
			   "");
	return obj;
}

#define SYS_STARTER                                                            \
	md::Log *log = new md::Log(MOORDYN_DBG_LEVEL);                             \
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

TEST_CASE("Scalar state var generation")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state(log);
	state.addPoint(p1);
	state.addPoint(p2);
	state.addLine(l);
	state.addVar<md::real>("scalar_by_type");
	state.addVar("scalar_by_enum", md::state::VarBase::types::REAL);

	REQUIRE(state.get<md::real>("scalar_by_type").rows() == 21);
	REQUIRE(state.get<md::real>("scalar_by_enum").rows() == 21);

	bool catched = false;
	try {
		state.get<md::real>("invalid_name");
	} catch (const md::invalid_value_error& e) {
		catched = true;
	}
	REQUIRE(catched);
	catched = false;
	try {
		state.get<md::vec>("scalar_by_type");
	} catch (const md::invalid_type_error& e) {
		catched = true;
	}
	REQUIRE(catched);

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("State var setting")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state(log);
	state.addPoint(p1);
	state.addPoint(p2);
	state.addLine(l);
	state.addVar<md::vec>("a");
	state.addVar<md::vec>("b");

	// Set the full state vars
	Eigen::Matrix<md::vec, 21, 1> ma;
	Eigen::Matrix<md::vec, 21, 1> mb;
	for (unsigned int i = 0; i < 21; i++) {
		ma(i) = md::vec::Zero();
		mb(i) = md::vec(i, 2*i, 4*i);
	}
	// They can be set with the set method:
	state.set<md::vec>("a", mb);
	state.set<md::vec>("b", ma);
	REQUIRE(state.get<md::vec>("a").rows() == mb.rows());
	REQUIRE(state.get<md::vec>("b").rows() == ma.rows());
	// Comparisons should be made element by element
	for (unsigned int i = 0; i < mb.rows(); i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), mb(i)));
	}
	for (unsigned int i = 0; i < ma.rows(); i++) {
		REQUIRE(allclose(state.get<md::vec>("b")(i), ma(i)));
	}
	// Or they can be set with the = operator
	state.get<md::vec>("a") = ma;
	state.get<md::vec>("b") = mb;
	REQUIRE(state.get<md::vec>("a").rows() == ma.rows());
	REQUIRE(state.get<md::vec>("b").rows() == mb.rows());
	for (unsigned int i = 0; i < ma.rows(); i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), ma(i)));
	}
	for (unsigned int i = 0; i < mb.rows(); i++) {
		REQUIRE(allclose(state.get<md::vec>("b")(i), mb(i)));
	}

	// The same can be done with just a subpart of the state var:
	state.set<md::vec>("a", l, state.get<md::vec>("b", l));
	// Internally, the state var is storing first the lines and then the points
	for (unsigned int i = 0; i < 19; i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), mb(i)));
	}
	for (unsigned int i = 19; i < 21; i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), ma(i)));
	}
	// If we modify a, b should remain the same
	state.get<md::vec>("a")(1) = md::vec::Zero();
	REQUIRE(!allclose(state.get<md::vec>("a")(1), state.get<md::vec>("b")(1)));
	// And again, we can use the = operator to set state var subparts:
	state.get<md::vec>("a", l) = state.get<md::vec>("b", l);
	for (unsigned int i = 0; i < 19; i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), mb(i)));
	}
	for (unsigned int i = 19; i < 21; i++) {
		REQUIRE(allclose(state.get<md::vec>("a")(i), ma(i)));
	}

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("State var operations")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state(log);
	state.addPoint(p1);
	state.addPoint(p2);
	state.addLine(l);
	state.addVar<md::vec>("a");
	state.addVar<md::vec>("b");
	state.addVar<md::vec>("c");

	Eigen::Matrix<md::vec, 21, 1> ma;
	Eigen::Matrix<md::vec, 21, 1> mb;
	for (unsigned int i = 0; i < 21; i++) {
		ma(i) = md::vec(0.5 * i, i, 2 * i);
		mb(i) = md::vec(i, 2 * i, 4 * i);
	}
	state.set<md::vec>("a", ma);
	state.get<md::vec>("b") = mb;

	REQUIRE(state.get<md::vec>("a").rows() == ma.rows());
	REQUIRE(state.get<md::vec>("b").rows() == mb.rows());

	state.set<md::vec>("c", state.get<md::vec>("a") + state.get<md::vec>("b"));
	for (unsigned int i = 0; i < ma.rows(); i++) {
		REQUIRE(allclose(state.get<md::vec>("c")(i), ma(i) + mb(i)));
	}

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("List var")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state(log);
	state.addPoint(p1);
	state.addPoint(p2);
	state.addLine(l);
	state.addVar<md::list>("a");
	state.addVar<md::list>("b");
	state.addVar<md::list>("c");
 
	// We just fill the lists for the line, with 4 elements each
	Eigen::Matrix<md::list, 19, 1> ma;
	Eigen::Matrix<md::list, 19, 1> mb;
	for (unsigned int i = 0; i < 19; i++) {
		ma(i).resize(4);
		ma(i) = Eigen::Matrix<md::real, 4, 1>(0.5 * i, i, 2 * i, 4 * i);
		mb(i).resize(4);
		mb(i) = Eigen::Matrix<md::real, 4, 1>(i, 2 * i, 4 * i, 8 * i);
	}
 
	state.setListLength("a", 4, l);
	state.setListLength("b", 4, l);
	state.setListLength("c", 4, l);
	state.get<md::list>("a", l) = ma;
	state.get<md::list>("b", l) = mb;

	REQUIRE(state.get<md::list>("a").rows() == state.get<md::list>("b").rows());
 
	state.get<md::list>("c") =
		state.get<md::list>("a") + state.get<md::list>("b");
	for (unsigned int i = 0; i < 19; i++) {
		REQUIRE(allclose(state.get<md::list>("c")(i), ma(i) + mb(i)));
	}

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}

TEST_CASE("Serialization")
{
	SYS_STARTER

	auto p1 = CreatePoint(log, env);
	auto p2 = CreatePoint(log, env);
	auto l = CreateLine(log, env, 20, outfile);

	md::state::State state_in(log), state_out(log);
	state_in.addPoint(p1);
	state_in.addPoint(p2);
	state_in.addLine(l);
	state_out.addPoint(p1);
	state_out.addPoint(p2);
	state_out.addLine(l);

	state_in.addVar<md::list>("a");
	state_in.setListLength("a", 4, l);
	state_in.addVar<md::vec6>("b");
	state_out.addVar<md::list>("a");
	state_out.setListLength("a", 4, l);
	state_out.addVar<md::vec6>("b");
 
	// We just fill the lists for the line, with 4 elements each
	Eigen::Matrix<md::list, 19, 1> va_in;
	Eigen::Matrix<md::list, 19, 1> va_out;
	for (unsigned int i = 0; i < 19; i++) {
		va_in(i).resize(4);
		va_in(i) = Eigen::Matrix<md::real, 4, 1>(0.5 * i, i, 2 * i, 4 * i);
		va_out(i).resize(4);
		va_out(i) = Eigen::Matrix<md::real, 4, 1>::Zero();
	}
	state_in.get<md::list>("a", l) = va_in;
	state_out.get<md::list>("a", l) = va_out;

	for (unsigned int i = 0; i < 21; i++) {
		state_in.get<md::vec6>("b")(i) = md::vec6(
			i, 2 * i, 4 * i, 8 * i, 16 * i, 32 * i);
		state_out.get<md::vec6>("b")(i) = md::vec6::Zero();
	}

	auto data_saved = state_in.Serialize();
	state_out.Deserialize(data_saved.data());

	REQUIRE(state_out.get<md::list>("a").rows() ==
		state_in.get<md::list>("a").rows());
	REQUIRE(state_out.get<md::vec6>("b").rows() ==
		state_in.get<md::vec6>("b").rows());
 
	for (unsigned int i = 0; i < 21; i++) {
		REQUIRE(allclose(state_out.get<md::list>("a")(i),
		                 state_in.get<md::list>("a")(i)));
	}
	for (unsigned int i = 0; i < 21; i++) {
		REQUIRE(allclose(state_out.get<md::vec6>("b")(i),
		                 state_in.get<md::vec6>("b")(i)));
	}

	delete p1;
	delete p2;
	delete l;
	SYS_KILLER
}
