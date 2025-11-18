#include <iostream>
#include <sstream>

#include "Misc.hpp"
#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_tostring.hpp"
#include "catch2/matchers/catch_matchers_templated.hpp"
#include "util.h"

namespace Catch {
template<typename T, int N>
struct StringMaker<Eigen::Vector<T, N>>
{
	static std::string convert(const Eigen::Vector<T, N>& value)
	{
		Eigen::IOFormat testFmt(4, Eigen::DontAlignCols, ", ", "\n", "[", "]");
		std::stringstream ss;
		ss << (value.transpose()).format(testFmt);
		return ss.str();
	}
};
}

template<typename DerivedA>
struct IsCloseMatcher : Catch::Matchers::MatcherGenericBase
{
	IsCloseMatcher(
	    const DerivedA& a,
	    const typename DerivedA::RealScalar rtol =
	        Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
	    const typename DerivedA::RealScalar atol =
	        Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
	  : _a(a)
	  , _rtol(rtol)
	  , _atol(atol)
	{
	}

	template<typename DerivedB>
	bool match(const DerivedB& b) const
	{
		return ((_a.derived() - b.derived()).array().abs() <=
		        (_atol + _rtol * b.derived().array().abs()))
		    .all();
	}

	std::string describe() const override
	{
		std::stringstream ss;
		ss << "Is close to: " << Catch::StringMaker<DerivedA>::convert(_a)
		   << "\nrtol = " << _rtol << ", atol = " << _atol;
		return ss.str();
	}

  private:
	const DerivedA _a;
	const typename DerivedA::RealScalar _rtol;
	const typename DerivedA::RealScalar _atol;
};

template<typename T>
IsCloseMatcher<T>
IsClose(T value)
{
	return IsCloseMatcher<T>(value, 1e-10, 1e-12);
}

// md::real is faster to type
namespace md = moordyn;
using namespace md;

TEST_CASE("getH gives the cross product matrix")
{

	vec testVec{ 1.0, 2.0, 3.0 };
	vec v{ 0.3, 0.2, 0.1 };
	// getH() should create a matrix that replicates the behavior of the cross
	// product such that getH(v) * a == -v.cross(a)
	vec ref = v.cross(-testVec);
	REQUIRE_THAT(getH(v) * testVec, IsClose(ref));
}

TEST_CASE("translateMass linear acceleration")
{
	/**
	 * This test imagines that we have some point whose center of mass
	 * is 1 meter in the x direction away from our reference point.
	 *
	 * A force applied in the y direction through this center of mass should
	 * result in no rotation and a acceleration in the y direction according to
	 * F = ma
	 *
	 * In our local coordinate system, this force will result in a torque.
	 *
	 * The mass matrix produced by translateMass should be such that we can
	 * correctly predict the acceleration in this situation
	 *
	 */

	md::real m = 1.0;
	mat mass = m * mat::Identity();
	mat sphereI = ((2.0 / 5.0) * m) * mat::Identity();

	vec offset{ 1.0, 1.0, 1.0 };

	mat6 M6 = translateMass(offset, mass);
	mat sphereIRef = sphereI - mass * getH(offset) * getH(offset);
	M6.bottomRightCorner<3, 3>() += sphereIRef;

	vec6 F = vec6::Zero();

	vec f3{ 0, 10, 0 };
	F.head<3>() = f3;
	F.tail<3>() = offset.cross(f3);

	// std::cout << "F = " << F.transpose() << std::endl;
	// std::cout << "M6 = \n" << M6 << std::endl;

	vec6 acc = solveMat6(M6, F);

	// linear acceleration by F = ma
	// no angular acceleration
	vec6 expectedAcc = vec6::Zero();
	expectedAcc.head<3>() = f3 / m;
	REQUIRE_THAT(acc, IsClose(expectedAcc));
}

TEST_CASE("translateMass6 linear acceleration")
{
	/**
	 * Like the testTranslateMass test except we do a series of two offsets
	 * We verify both that the acceleration is computed correctly,
	 * and that the mass matrix we get matched what we get by translating
	 * the mass by both offsets simultaneously.
	 */
	md::real m = 1.0;
	mat mass = m * mat::Identity();

	mat6 mass6 = mat6::Zero();
	mass6.topLeftCorner<3, 3>() = mass;

	vec offset{ 1.0, 0.0, 0.0 };
	vec offset2{ 2, 1, 0.2 };

	mat6 M6 = translateMass(offset, mass);

	M6 = translateMass6(offset2, M6);

	REQUIRE_THAT(translateMass((offset + offset2), mass), IsClose(M6));

	// we add some moment of inertia to prevent a singular matrix.
	// presume it's a sphere with radius 1
	mat sphereI = ((2.0 / 5.0) * m) * mat::Identity();
	mat sphereIRef =
	    sphereI - mass * getH(offset + offset2) * getH(offset + offset2);
	M6.bottomRightCorner<3, 3>() += sphereIRef;
	vec6 F = vec6::Zero();

	vec f3{ 0, 10, 0 };
	F.head<3>() = f3;
	F.tail<3>() = (offset + offset2).cross(f3);
	// std::cout << "F = " << F.transpose() << std::endl;
	// std::cout << "M6 = \n" << M6 << std::endl;
	// std::cout << "det(M6) = " << M6.determinant() << std::endl;
	vec6 acc = solveMat6(M6, F);

	// linear acceleration by F = ma
	// no angular acceleration
	vec6 expectedAcc = vec6::Zero();
	expectedAcc.head<3>() = f3 / m;

	REQUIRE_THAT(acc, IsClose(expectedAcc));
}

TEST_CASE("rotateMass simple")
{
	md::real m = 1.0;
	mat mass = m * mat::Identity();
	mat sphereI = ((2.0 / 5.0) * m) * mat::Identity();

	vec offset{ 1.0, 1.0, 0.0 };

	mat6 M6 = translateMass(offset, mass);

	vec3 axis{ 0, 0, 1 };
	axis.normalize();
	// rotate -90 degrees around the z axis
	Eigen::AngleAxisd rot(-pi / 2, axis);
	mat6 rotatedMass = rotateMass6(rot.toRotationMatrix(), M6);

	// this offset represents the offset after rotation
	vec newOffset{ 1, -1, 0.0 };

	REQUIRE_THAT(rot.toRotationMatrix() * offset, IsClose(newOffset));
	REQUIRE_THAT(translateMass(newOffset, mass), IsClose(rotatedMass));

	// add some moment of inertia to prevent singular mass matrix
	mat sphereIRef = sphereI - mass * getH(newOffset) * getH(newOffset);
	rotatedMass.bottomRightCorner<3, 3>() += sphereIRef;
	vec6 F = vec6::Zero();

	vec f3{ 0, 10, 0 };
	F.head<3>() = f3;
	F.tail<3>() = newOffset.cross(f3);

	vec6 acc = solveMat6(rotatedMass, F);

	vec6 expectedAcc = vec6::Zero();
	expectedAcc.head<3>() = f3 / m;

	REQUIRE_THAT(acc, IsClose(expectedAcc));
}
