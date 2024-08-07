/*
 * Copyright (c) 2022, Matt Hall
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

#pragma once

// Visual studio still uses this
#define _USE_MATH_DEFINES

#include "MoorDynAPI.h"
#include "Eigen/Dense"

#include "Waves/WaveOptions.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <complex>
#include <utility>
#include <initializer_list>
#include <filesystem>

#include <memory>
#include <limits>

#ifdef OSX
#include <sys/uio.h>
#elif defined WIN32
#include <windows.h> // these are for guicon function RedirectIOToConsole
#include <io.h>
#endif

// note: this file contains the struct definitions for environmental and
// line/point properties

// from Iñaki Zabala
#ifdef _MSC_VER
template<typename T>
static inline T
round(T val)
{
	return floor(val + 0.5);
}
#endif

using namespace std;

namespace Eigen {
// Eigen does not provide 6 components objects out of the box
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6, 6> Matrix6f;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<int, 6, 1> Vector6i;
typedef Matrix<int, 6, 6> Matrix6i;
// It is also convenient for us to define a generic Eigen dynamic matrix class
#ifdef MOORDYN_SINGLEPRECISSION
typedef MatrixXf MatrixXr;
#else
typedef MatrixXd MatrixXr;
#endif
}

/** @brief MoorDyn2 C++ API namespace
 */
namespace moordyn {

#ifdef MOORDYN_SINGLEPRECISSION
typedef float real;
typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef Eigen::Vector6f vec6;
typedef vec3 vec;
typedef Eigen::Matrix2f mat2;
typedef Eigen::Matrix3f mat3;
typedef Eigen::Matrix4f mat4;
typedef Eigen::Matrix6f mat6;
typedef mat3 mat;
typedef Eigen::Quaternionf quaternion;
#else
/// Real numbers wrapper. It is either double or float
typedef double real;
/// 2-D vector of real numbers
typedef Eigen::Vector2d vec2;
/// 3-D vector of real numbers
typedef Eigen::Vector3d vec3;
/// 4-D vector of real numbers
typedef Eigen::Vector4d vec4;
/// 6-D vector of real numbers
typedef Eigen::Vector6d vec6;
/// vec3 renaming
typedef vec3 vec;
/// 2x2 matrix of real numbers
typedef Eigen::Matrix2d mat2;
/// 3x3 matrix of real numbers
typedef Eigen::Matrix3d mat3;
/// 4x4 matrix of real numbers
typedef Eigen::Matrix4d mat4;
/// 6x6 matrix of real numbers
typedef Eigen::Matrix6d mat6;
/// mat3 renaming
typedef mat3 mat;
/// Quaternion of real numbers
typedef Eigen::Quaterniond quaternion;
#endif
/// 2-D vector of integers
typedef Eigen::Vector2i ivec2;
/// 3-D vector of integers
typedef Eigen::Vector3i ivec3;
/// 4-D vector of integers
typedef Eigen::Vector4i ivec4;
/// 6-D vector of integers
typedef Eigen::Vector6i ivec6;
/// Renaming of ivec3
typedef ivec3 ivec;

/// Complex numbers
typedef std::complex<real> complex;

/** @brief This function compares two real numbers and determines if they are
 * "almost" equal
 *
 * "almost" equal means equal within some relative tolerance (basically
 * ignoring the last 2 significant digits) (see "Safe Comparisons" suggestion
 * from http://www.lahey.com/float.htm)
 * @param a1 The first real number to compare
 * @param a2 The second real number to compare
 * @return true if and only if the numbers are almost equal, false otherwise
 * @note The numbers are added together in this routine, so overflow can result
 * if comparing two "huge" numbers.
 * @note Use this function instead of directly calling a specific routine in
 * the generic interface.
 */
inline bool
EqualRealNos(const real a1, const real a2)
{
	constexpr real eps = std::numeric_limits<moordyn::real>::epsilon();
	constexpr real tol = ((real)100.0) * eps / ((real)2.0);

	const real fraction = (std::max)(std::abs(a1 + a2), ((real)1.0));
	return std::abs(a1 - a2) <= fraction * tol;
}

inline vec3
canonicalEulerAngles(const quaternion& quat, int a0, int a1, int a2)
{
	// From issue #163:
	// https://github.com/FloatingArrayDesign/MoorDyn/issues/163
	mat3 coeff = quat.normalized().toRotationMatrix();
	vec3 res{};
	using Index = int;
	using Scalar = real;
	const Index odd = ((a0 + 1) % 3 == a1) ? 0 : 1;
	const Index i = a0;
	const Index j = (a0 + 1 + odd) % 3;
	const Index k = (a0 + 2 - odd) % 3;
	if (a0 == a2) {
		// Proper Euler angles (same first and last axis).
		// The i, j, k indices enable addressing the input matrix as the XYX
		// archetype matrix (see Graphics Gems IV), where e.g. coeff(k, i) means
		// third column, first row in the XYX archetype matrix:
		//  c2      s2s1              s2c1
		//  s2s3   -c2s1s3 + c1c3    -c2c1s3 - s1c3
		// -s2c3    c2s1c3 + c1s3     c2c1c3 - s1s3
		// Note: s2 is always positive.
		Scalar s2 = Eigen::numext::hypot(coeff(j, i), coeff(k, i));
		if (odd) {
			res[0] = atan2(coeff(j, i), coeff(k, i));
			// s2 is always positive, so res[1] will be within the canonical [0,
			// pi] range
			res[1] = atan2(s2, coeff(i, i));
		} else {
			// In the !odd case, signs of all three angles are flipped at the
			// very end. To keep the solution within the canonical range, we
			// flip the solution and make res[1] always negative here (since s2
			// is always positive, -atan2(s2, c2) will always be negative). The
			// final flip at the end due to !odd will thus make res[1] positive
			// and canonical. NB: in the general case, there are two correct
			// solutions, but only one is canonical. For proper Euler angles,
			// flipping from one solution to the other involves flipping the
			// sign of the second angle res[1] and adding/subtracting pi to the
			// first and third angles. The addition/subtraction of pi to the
			// first angle res[0] is handled here by flipping the signs of
			// arguments to atan2, while the calculation of the third angle does
			// not need special adjustment since it uses the adjusted res[0] as
			// the input and produces a correct result.
			res[0] = atan2(-coeff(j, i), -coeff(k, i));
			res[1] = -atan2(s2, coeff(i, i));
		}
		// With a=(0,1,0), we have i=0; j=1; k=2, and after computing the first
		// two angles, we can compute their respective rotation, and apply its
		// inverse to M. Since the result must be a rotation around x, we have:
		//
		//  c2  s1.s2 c1.s2                   1  0   0
		//  0   c1    -s1       *    M    =   0  c3  s3
		//  -s2 s1.c2 c1.c2                   0 -s3  c3
		//
		//  Thus:  m11.c1 - m21.s1 = c3  &   m12.c1 - m22.s1 = s3
		Scalar s1 = sin(res[0]);
		Scalar c1 = cos(res[0]);
		res[2] = atan2(c1 * coeff(j, k) - s1 * coeff(k, k),
		               c1 * coeff(j, j) - s1 * coeff(k, j));
	} else {
		// Tait-Bryan angles (all three axes are different; typically used for
		// yaw-pitch-roll calculations). The i, j, k indices enable addressing
		// the input matrix as the XYZ archetype matrix (see Graphics Gems IV),
		// where e.g. coeff(k, i) means third column, first row in the XYZ
		// archetype matrix:
		//  c2c3    s2s1c3 - c1s3     s2c1c3 + s1s3
		//  c2s3    s2s1s3 + c1c3     s2c1s3 - s1c3
		// -s2      c2s1              c2c1
		res[0] = atan2(coeff(j, k), coeff(k, k));
		Scalar c2 = Eigen::numext::hypot(coeff(i, i), coeff(i, j));
		// c2 is always positive, so the following atan2 will always return a
		// result in the correct canonical middle angle range [-pi/2, pi/2]
		res[1] = atan2(-coeff(i, k), c2);
		Scalar s1 = sin(res[0]);
		Scalar c1 = cos(res[0]);
		res[2] = atan2(s1 * coeff(k, i) - c1 * coeff(j, i),
		               c1 * coeff(j, j) - s1 * coeff(k, j));
	}
	if (!odd) {
		res = -res;
	}
	return res;
}

inline vec3
Quat2Euler(const quaternion& q)
{
	// 0, 1, 2 correspond to axes leading to XYZ rotation
	return canonicalEulerAngles(q, 0, 1, 2);
}

inline quaternion
Euler2Quat(const vec3& angles)
{
	using AngleAxis = Eigen::AngleAxis<real>;
	quaternion q = AngleAxis(angles.x(), vec3::UnitX()) *
	               AngleAxis(angles.y(), vec3::UnitY()) *
	               AngleAxis(angles.z(), vec3::UnitZ());
	return q;
}

/** @brief Joint of a point and a quaternion
 *
 * Some entities are defined just by its position and orientation, so this class
 * makes it happen on a simpler way
 */
struct XYZQuat
{
	vec3 pos;
	quaternion quat;

	XYZQuat() {}

	XYZQuat(vec3 pos, quaternion quat)
	  : pos(pos)
	  , quat(quat)
	{
	}

	static XYZQuat Zero()
	{
		return XYZQuat{ vec3::Zero(), quaternion::Identity() };
	}
	static XYZQuat fromVec6(const vec6& vec)
	{
		return XYZQuat{ vec.head<3>(), Euler2Quat(vec.tail<3>()) };
	}
	vec6 toVec6() const
	{
		vec6 out;
		out.head<3>() = this->pos;
		out.tail<3>() = Quat2Euler(this->quat);
		return out;
	}
	Eigen::Vector<real, 7> toVec7() const
	{
		Eigen::Vector<real, 7> out;
		out.head<3>() = pos;
		out.tail<4>() = quat.coeffs();
		return out;
	}

	XYZQuat operator+(const XYZQuat& visitor) const;
	XYZQuat operator-(const XYZQuat& visitor) const;
	XYZQuat operator*(const real& visitor) const;
};

/// The imaginary unit
const complex i1(0., 1.);

/** @brief Convert a vector to a C-ish array
 * @param v The input vector
 * @param a The output array
 */
template<typename T>
inline void
vec2array(const vec& v, T* a)
{
	a[0] = (T)v[0];
	a[1] = (T)v[1];
	a[2] = (T)v[2];
}

/** @brief Convert a C-ish array to a vector
 * @param a The input array
 * @param v The output vector
 */
template<typename T>
inline void
array2vec(const T* a, vec& v)
{
	v[0] = (moordyn::real)a[0];
	v[1] = (moordyn::real)a[1];
	v[2] = (moordyn::real)a[2];
}

/** @brief Convert a vector to a C-ish array
 * @param v The input vector
 * @param a The output array
 */
template<typename T>
inline void
vec62array(const vec6& v, T* a)
{
	a[0] = (T)v[0];
	a[1] = (T)v[1];
	a[2] = (T)v[2];
	a[3] = (T)v[3];
	a[4] = (T)v[4];
	a[5] = (T)v[5];
}

/** @brief Convert a C-ish array to a vector
 * @param a The input array
 * @param v The output vector
 */
template<typename T>
inline void
array2vec6(const T* a, vec6& v)
{
	v[0] = (moordyn::real)a[0];
	v[1] = (moordyn::real)a[1];
	v[2] = (moordyn::real)a[2];
	v[3] = (moordyn::real)a[3];
	v[4] = (moordyn::real)a[4];
	v[5] = (moordyn::real)a[5];
}

/** @brief Convert a matrix to a C-ish array
 * @param v The input matrix
 * @param a The output array
 */
template<typename T>
inline void
mat2array(const mat& v, T a[3][3])
{
	a[0][0] = (T)v(0, 0);
	a[0][1] = (T)v(0, 1);
	a[0][2] = (T)v(0, 2);
	a[1][0] = (T)v(1, 0);
	a[1][1] = (T)v(1, 1);
	a[1][2] = (T)v(1, 2);
	a[2][0] = (T)v(2, 0);
	a[2][1] = (T)v(2, 1);
	a[2][2] = (T)v(2, 2);
}

/** @brief Convert a C-ish array to a matrix
 * @param a The input array
 * @param v The output matrix
 */
template<typename T>
inline void
array2mat(const T a[3][3], mat& v)
{
	v(0, 0) = (moordyn::real)a[0][0];
	v(0, 1) = (moordyn::real)a[0][1];
	v(0, 2) = (moordyn::real)a[0][2];
	v(1, 0) = (moordyn::real)a[1][0];
	v(1, 1) = (moordyn::real)a[1][1];
	v(1, 2) = (moordyn::real)a[1][2];
	v(2, 0) = (moordyn::real)a[2][0];
	v(2, 1) = (moordyn::real)a[2][1];
	v(2, 2) = (moordyn::real)a[2][2];
}

/** @brief Convert a matrix to a C-ish array
 * @param v The input matrix
 * @param a The output array
 */
template<typename T>
inline void
mat62array(const mat6& v, T a[6][6])
{
	a[0][0] = (T)v(0, 0);
	a[0][1] = (T)v(0, 1);
	a[0][2] = (T)v(0, 2);
	a[0][3] = (T)v(0, 3);
	a[0][4] = (T)v(0, 4);
	a[0][5] = (T)v(0, 5);
	a[1][0] = (T)v(1, 0);
	a[1][1] = (T)v(1, 1);
	a[1][2] = (T)v(1, 2);
	a[1][3] = (T)v(1, 3);
	a[1][4] = (T)v(1, 4);
	a[1][5] = (T)v(1, 5);
	a[2][0] = (T)v(2, 0);
	a[2][1] = (T)v(2, 1);
	a[2][2] = (T)v(2, 2);
	a[2][3] = (T)v(2, 3);
	a[2][4] = (T)v(2, 4);
	a[2][5] = (T)v(2, 5);
}

/** @brief Convert a C-ish array to a matrix
 * @param a The input array
 * @param v The output matrix
 */
template<typename T>
inline void
array2mat6(const T a[6][6], mat6& v)
{
	v(0, 0) = (moordyn::real)a[0][0];
	v(0, 1) = (moordyn::real)a[0][1];
	v(0, 2) = (moordyn::real)a[0][2];
	v(0, 3) = (moordyn::real)a[0][3];
	v(0, 4) = (moordyn::real)a[0][4];
	v(0, 5) = (moordyn::real)a[0][5];
	v(1, 0) = (moordyn::real)a[1][0];
	v(1, 1) = (moordyn::real)a[1][1];
	v(1, 2) = (moordyn::real)a[1][2];
	v(1, 3) = (moordyn::real)a[1][3];
	v(1, 4) = (moordyn::real)a[1][4];
	v(1, 5) = (moordyn::real)a[1][5];
	v(2, 0) = (moordyn::real)a[2][0];
	v(2, 1) = (moordyn::real)a[2][1];
	v(2, 2) = (moordyn::real)a[2][2];
	v(2, 3) = (moordyn::real)a[2][3];
	v(2, 4) = (moordyn::real)a[2][4];
	v(2, 5) = (moordyn::real)a[2][5];
}

/** Slice a C++ vector
 * @param v The vector to slice
 * @param m The first element to consider
 * @param n The number of elements
 * @return The sliced vector
 */
template<typename T>
std::vector<T>
vector_slice(std::vector<T> const& v, unsigned int m, unsigned int n)
{
	auto first = v.begin() + m;
	auto last = first + n;
	std::vector<T> v2(first, last);
	return v2;
}

/** Slice a C++ vector
 * @param v The vector to slice
 * @param n The number of elements
 * @return The sliced vector
 */
template<typename T>
std::vector<T>
vector_slice(std::vector<T> const& v, unsigned int n)
{
	return vector_slice(v, 0, n);
}

/** Extend a C++ vector
 * @param v The vector to slice
 * @param v_prime The vector to be concatenated
 */
template<typename T>
void
vector_extend(std::vector<T>& v, std::vector<T> const& v_prime)
{
	v.reserve(v.size() + distance(v_prime.begin(), v_prime.end()));
	v.insert(v.end(), v_prime.begin(), v_prime.end());
}

/** @brief End point qualifiers
 *
 * Used for both lines and rods
 */
typedef enum
{
	/// Bottom of the line
	ENDPOINT_A = 0,
	/// Top of the line
	ENDPOINT_B = 1,
	// Some aliases
	ENDPOINT_BOTTOM = ENDPOINT_A,
	ENDPOINT_TOP = ENDPOINT_B,
} EndPoints;

/** @brief Gives an character representation of the end point
 * @return The endpoint char
 */
inline char
end_point_name(EndPoints p)
{
	return char('A' + (int)p);
}

/** \addtogroup moordyn_errors
 *  @{
 */

/// Error identifier
typedef int error_id;

/// Simple macro to define custom exceptions
#define MAKE_EXCEPTION(name)                                                   \
	class name : public std::runtime_error                                     \
	{                                                                          \
	  public:                                                                  \
		name(const char* msg)                                                  \
		  : std::runtime_error(msg)                                            \
		{                                                                      \
		}                                                                      \
	};

/// Exception thrown for invalid input files
MAKE_EXCEPTION(input_file_error)
/// Exception thrown for invalid output files
MAKE_EXCEPTION(output_file_error)
/// Exception thrown for invalid input values
MAKE_EXCEPTION(input_error)
/// Exception thrown when NaN values are encountered
MAKE_EXCEPTION(nan_error)
/// Exception thrown when memory errors are triggered
MAKE_EXCEPTION(mem_error)
/// Exception thrown when invalid values are found
MAKE_EXCEPTION(invalid_value_error)
/// Exception thrown when invalid values are found
MAKE_EXCEPTION(non_implemented_error)
/// Exception thrown for other uhandled errors
MAKE_EXCEPTION(unhandled_error)

/// Throw the exception associated with the provided error. Do nothing if
/// MOORDYN_SUCCESS is passed
#define MOORDYN_THROW(err, msg)                                                \
	switch (err) {                                                             \
		case MOORDYN_SUCCESS:                                                  \
			break;                                                             \
		case MOORDYN_INVALID_INPUT_FILE:                                       \
			throw moordyn::input_file_error(msg);                              \
			break;                                                             \
		case MOORDYN_INVALID_OUTPUT_FILE:                                      \
			throw moordyn::output_file_error(msg);                             \
			break;                                                             \
		case MOORDYN_INVALID_INPUT:                                            \
			throw moordyn::input_error(msg);                                   \
			break;                                                             \
		case MOORDYN_NAN_ERROR:                                                \
			throw moordyn::nan_error(msg);                                     \
			break;                                                             \
		case MOORDYN_MEM_ERROR:                                                \
			throw moordyn::mem_error(msg);                                     \
			break;                                                             \
		case MOORDYN_INVALID_VALUE:                                            \
			throw moordyn::invalid_value_error(msg);                           \
			break;                                                             \
		case MOORDYN_NON_IMPLEMENTED:                                          \
			throw moordyn::non_implemented_error(msg);                         \
			break;                                                             \
		default:                                                               \
			throw moordyn::unhandled_error(msg);                               \
			break;                                                             \
	}

/// Catch thrown exceptions and convert them in an error_id. It also gives the
/// message on the Exception. This macro will only handle known exceptions, i.e.
/// the ones declared in moordyn_errors. You can add more catch() instances
/// afterwards
#define MOORDYN_CATCHER(err, msg)                                              \
	catch (moordyn::input_file_error const& e)                                 \
	{                                                                          \
		err = MOORDYN_INVALID_INPUT_FILE;                                      \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::output_file_error const& e)                                \
	{                                                                          \
		err = MOORDYN_INVALID_OUTPUT_FILE;                                     \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::input_error const& e)                                      \
	{                                                                          \
		err = MOORDYN_INVALID_INPUT;                                           \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::nan_error const& e)                                        \
	{                                                                          \
		err = MOORDYN_NAN_ERROR;                                               \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::mem_error const& e)                                        \
	{                                                                          \
		err = MOORDYN_MEM_ERROR;                                               \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::invalid_value_error const& e)                              \
	{                                                                          \
		err = MOORDYN_INVALID_VALUE;                                           \
		msg = e.what();                                                        \
	}                                                                          \
	catch (moordyn::unhandled_error const& e)                                  \
	{                                                                          \
		err = MOORDYN_UNHANDLED_ERROR;                                         \
		msg = e.what();                                                        \
	}

/**
 * @}
 */

/** \addtogroup string_tools
 *  @{
 */

namespace str {

/** @brief Convert a string to lower case
 * @param str String to check
 * @return A lower case copy of the string
 */
string
lower(const string& str);

/** @brief Convert a string to lower case
 * @param str String to check
 * @return A lower case copy of the string
 */
string
upper(const string& str);

/** @brief Check if a string starts with the provided prefix
 * @param str String to check
 * @param prefix The prefix to look for
 * @return true if the string starts with the prefix, false otherwise
 */
bool
startswith(const string& str, const string& prefix);

/** @brief Check if a string contains one of the provided terms
 * @param str String to check
 * @param terms List of terms to look for
 * @return true if the string contains one or more of the terms, false
 * otherwise
 */
bool
has(const string& str, const vector<string> terms);

/** @brief Split a string in a list of substrings
 * @param str String to split
 * @param sep Separator
 * @return The list of substrings
 */
vector<string>
split(const string& str, const char sep);

/** @brief Split a string in a list of substrings
 *
 * The space is used as separator
 * @param s String to split
 * @return The list of substrings
 */
inline vector<string>
split(const string& s)
{
	vector<string> sout = split(s, ' ');
	if (sout.size() == 1) return split(sout[0], '	');
	else return sout;
}

/**
 * @brief Removes trailing whitespace from the passed in string
 *
 * @param s String to trim
 */
void
rtrim(string& s);

/** @brief Split a string into separate letter strings and integers
 */
int DECLDIR
decomposeString(const std::string& outWord,
                std::string& let1,
                std::string& num1,
                std::string& let2,
                std::string& num2,
                std::string& let3);

bool
isOneOf(const std::string& str,
        const std::initializer_list<const std::string> values);

} // ::moordyn::str

namespace fileIO {

/**
 * Read in all of the lines of a file.
 * Throws an error if file is not found or other error occurs.
 */
std::vector<std::string>
fileToLines(const std::filesystem::path& path);

}

/**
 * @}
 */

/** \defgroup environment Environmental variables
 *  @{
 */

/** Bathymetry specification alternatives
 */
typedef enum
{
	/// Flat seafloor:
	SEAFLOOR_FLAT = 0,
	/// 3D seafloor
	SEAFLOOR_3D = 1,
} seafloor_settings;

/**
 * @}
 */

/**
 * @brief Solves a 6x6 system of equations M * a = b
 *
 * Uses Eigen::ColPivHouseholderQR which means it doesn't
 * have any particular constraints on the matrix properties.
 * Has high accuracy and good speed.
 *
 * @param mat 6x6 Matrix (M in M * a = b)
 * @param vec 6x1 Vector (b in M * a = b)
 * @return vec6 Resulting solution (a in M * a = b)
 */
vec6 DECLDIR
solveMat6(const mat6& mat, const vec6& vec);

/** \defgroup transformations 3D transformations
 *  @{
 */

/** @brief Normalized direction vector
 * @param u The output normalized direction vector
 * @param r1 The orig point
 * @param r2 The dest point
 * @return The distance between the points
 */
inline moordyn::real
unitvector(vec& u, const vec& r1, const vec& r2)
{
	u = r2 - r1;
	const double l = u.norm();
	if (!EqualRealNos(l, 0.0))
		u /= l;
	return l;
}

/** @brief Compute a vector with the same direction but different length
 * @param u The input vector
 * @param newlength The new length of the output vector
 * @param y The output vector
 * @note If the input vector has null length, the output vector will as well,
 * no matter @p newlength is provided
 */
template<typename T>
inline void
scalevector(const vec& u, T newlength, vec& y)
{
	const moordyn::real l2 = u.squaredNorm();
	if (EqualRealNos(l2, 0.0)) {
		y = u;
		return;
	}
	const moordyn::real scaler = (moordyn::real)newlength / sqrt(l2);
	y = scaler * u;
}

/** @brief Produce alternator matrix
 *
 * This is the cross product represented as a matrix
 * r x a = getH(r) * a
 *
 * See "anti-symmetric tensor components" from Sadeghi and Incecik
 * @param r Offset vector
 * @return Alternator matrix
 */
inline mat
getH(vec r)
{
	mat H;
	// clang-format off
	H << 0, r[2], -r[1],
		-r[2], 0, r[0],
		r[1], -r[0], 0;
	// clang-format on
	return H;
}

/** @brief Compute the mass matrix on an offset point
 * @param r Offset
 * @param M Mass matrix
 * @return Translated Mass & Inertia matrix
 */
mat6 DECLDIR
translateMass(vec r, mat M);

/** @brief Compute the mass matrix on an offset point
 * @param r Offset
 * @param M Mass & Inertia matrix
 * @return Translated Mass & Inertia matrix
 */
mat6 DECLDIR
translateMass6(vec r, mat6 M);

/** @brief rotation to a 3x3 mass matrix or any other second order tensor
 *
 * \f$ M^{*} = R \cdot M \cdot R^T \f$
 * @param R Rotation matrix
 * @param M Mass matrix
 * @return Rotated mass
 */
inline mat
rotateMass(mat R, mat M)
{
	return R * M * R.transpose();
}

/** @brief rotation to a 6x6 mass/inertia tensor
 *
 * see Sadeghi and Incecik 2005 for theory
 * @param R Rotation matrix
 * @param M Mass & Inertia matrix
 * @return Rotated mass
 */
mat6 DECLDIR
rotateMass6(mat R, mat6 M);

/** @brief calculate position and velocity of point based on its position
 * relative to moving 6DOF body
 * @param rRelBody The point in the body local system of reference
 * @param M The rotation matrix
 * @param r The position of the local system origin
 * @param rd The velocity of the local system origin
 * @param rOut The output position with respect the global system of reference
 * @param rdOut The output velocity with respect the global system of reference
 */
void
transformKinematics(const vec& rRelBody,
                    const mat& M,
                    const vec& r,
                    const vec6& rd,
                    vec& rOut,
                    vec& rdOut);

/** @brief Rotation matrix around x axis
 * @param rads The angle in radians
 * @return The rotation matrix
 */
inline mat
RotX(real rads)
{
	const real s = sin(rads);
	const real c = cos(rads);
	mat R;
	R << 1., 0., 0., 0., c, -s, 0., s, c;
	return R;
}

/** @brief Rotation matrix around y axis
 * @param rads The angle in radians
 * @return The rotation matrix
 */
inline mat
RotY(real rads)
{
	const real s = sin(rads);
	const real c = cos(rads);
	mat R;
	R << c, 0., s, 0., 1., 0., -s, 0., c;
	return R;
}

/** @brief Rotation matrix around z axis
 * @param rads The angle in radians
 * @return The rotation matrix
 */
inline mat
RotZ(real rads)
{
	const real s = sin(rads);
	const real c = cos(rads);
	mat R;
	R << c, -s, 0., s, c, 0., 0., 0., 1.;
	return R;
}

// clang-format off
/// Create the Euler rotations, like RotXYZ, RotXZX, RotZYX...
#define MAKE_EULER_ROT(a,b,c)                                                  \
inline mat Rot ## a ## b ## c(real a1, real a2, real a3)                       \
{                                                                              \
	return Rot ## a (a1) * Rot ## b (a2) * Rot ## c (a3);                      \
}                                                                              \
inline mat Rot ## a ## b ## c(vec rads)                                        \
{                                                                              \
	return Rot ## a ## b ## c (rads[0], rads[1], rads[2]);                     \
}

// clang-format on

MAKE_EULER_ROT(X, Y, X)
MAKE_EULER_ROT(X, Y, Z)
MAKE_EULER_ROT(X, Z, X)
MAKE_EULER_ROT(X, Z, Y)
MAKE_EULER_ROT(Y, X, Y)
MAKE_EULER_ROT(Y, X, Z)
MAKE_EULER_ROT(Y, Z, X)
MAKE_EULER_ROT(Y, Z, Y)
MAKE_EULER_ROT(Z, X, Y)
MAKE_EULER_ROT(Z, X, Z)
MAKE_EULER_ROT(Z, Y, X)
MAKE_EULER_ROT(Z, Y, Z)

/** @brief Get the spherical angles for a vector
 * @param q The vector
 * @return The orientation angles, i.e. inclination (from the horizontal plane)
 * and heading (in radians)
 * @throws nan_error If the provided vector is too small or null
 */
std::pair<real, real>
orientationAngles(vec q);

/** @brief Convenience function to calculate curvature based on adjacent
 * segments' direction vectors and their combined length
 * @param length The length of the 2 segments
 * @param q1 First direction vector
 * @param q2 Second direction vector
 * @return The curvature
 */
moordyn::real
GetCurvature(moordyn::real length, const vec& q1, const vec& q2);

/**
 * @}
 */

#if !defined(MOORDYN_SINGLEPRECISSION) && defined(M_PIl)
/// Pi constant
const real pi = M_PIl;
#else
/// Pi constant
// const real pi = M_PI;
const real pi = 3.141592653589793238462643383279502884197169399375105820974944;
#endif
/// Constant to convert radians into degrees
const real rad2deg = 180.0 / pi;

/// Constant to convert degrees into radians
const real deg2rad = pi / 180.0;

/** \defgroup entities_properties Properties readed from the input file, used
 * to define the entities handled during the simulation
 *  @{
 */

class Rod;
class Point;
class Line;

/** @brief Failure conditions
 */
typedef struct _FailProps
{
	/// The rod the lines are attached to, if any. Otherwise it is NULL
	Rod* rod;
	/// The rod attachment end point, useless if rod is NULL
	EndPoints rod_end_point;
	/// The point the lines are attached to, if any. Otherwise it is NULL
	Point* point;
	/// The attached lines
	std::vector<Line*> lines;
	/// The attached line end points. This is actually an array to be filled
	/// with the end points where each line is detached
	std::vector<EndPoints> line_end_points;
	/// Failure criteria based on simulation time (s)
	real time;
	/// Failure criteria based on tension (N)
	real ten;
	/// false until the line fails
	bool status;
} FailProps;

/**
 * @}
 */

} // ::moordyn

const int nCoef = 30; // maximum number of entries to allow in nonlinear
                      // coefficient lookup tables

/** \ingroup environment
 *  @{
 */

struct EnvCond
{
	/// Gavity acceleration (m/s^2)
	double g;
	/// Water depth (m)
	double WtrDpth;
	/// Water density (kg/m^3)
	double rho_w;

	/// bottom stiffness (Pa/m)
	double kb;
	/// bottom damping   (Pa/m/s)
	double cb;
	/// Bottom modeling mode (0=flat, 1=3d...)<<<
	moordyn::seafloor_settings SeafloorMode;

	/// Water Kinematics Options
	moordyn::waves::WaterKinOptions waterKinOptions;

	/// general bottom friction coefficient, as a start
	double FrictionCoefficient;
	/// a damping coefficient used to model the friction at speeds near zero
	double FricDamp;
	/// a ratio of static to dynamic friction ( = mu_static/mu_dynamic)
	double StatDynFricScale;

	/// a global switch for whether to show the units line in the output files
	/// (1, default), or skip it (0)
	int WriteUnits;
	/// whether to write a log file. (0=no, 1=basic, 2=full description,
	/// 3=ongoing output
	int writeLog;
};

typedef std::shared_ptr<EnvCond> EnvCondRef;

/// @}

typedef struct _LineProps // (matching Line Dictionary inputs)
{
	string type;
	double d;
	double w; // linear weight in air
	double EA;
	double EI;
	double c; // internal damping
	double cI;
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	int nEApoints; // number of values in stress-strain lookup table (0 means
	               // using constant E)
	double
	    stiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double stiffYs[nCoef]; // y array for stress-strain lookup table
	int nCpoints; // number of values in stress-strainrate lookup table (0 means
	              // using constant c)
	double dampXs[nCoef]; // x array for stress-strainrate lookup table (up to
	                      // nCoef)
	double dampYs[nCoef]; // y array for stress-strainrate lookup table
	int nEIpoints = 0; // number of values in bending stress-strain lookup table
	                   // (0 means using constant E)
	double
	    bstiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double bstiffYs[nCoef]; // y array for stress-strain lookup table
} LineProps;

typedef struct _RodProps // (matching Rod Dictionary inputs)
{
	string type;
	double d;
	double w; // linear weight in air
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	double CaEnd;
	double CdEnd;
} RodProps;

typedef struct _PointProps // matching node input stuff
{
	int number;
	string type;
	double X;
	double Y;
	double Z;
	double M;
	double V;
	double FX;
	double FY;
	double FZ;
	double
	    CdA;   // added 2015/1/15 - product of drag coefficient and frontal area
	double Ca; // added 2015/1/15  - added mass coefficient
} PointProps;

typedef struct _BodyProps // matching body input stuff
{
	int number;
	string type;
	double X0; // constants set at startup from input file
	double Y0;
	double Z0;
	double Xcg;
	double Ycg;
	double Zcg;
	double M;
	double V;
	double IX;
	double IY;
	double IZ;
	double CdA;
	double Ca;
} BodyProps;

// ------------ Output definitions -------------
enum QTypeEnum : int
{

	Time = 0,
	PosX = 1,
	PosY = 2,
	PosZ = 3,
	RX = 4,
	RY = 5,
	RZ = 6,
	VelX = 7,
	VelY = 8,
	VelZ = 9,
	RVelX = 10,
	RVelY = 11,
	RVelZ = 12,
	AccX = 13,
	AccY = 14,
	AccZ = 15,
	RAccX = 16,
	RAccY = 17,
	RAccZ = 18,
	Ten = 19,
	FX = 20,
	FY = 21,
	FZ = 22,
	MX = 23,
	MY = 24,
	MZ = 25,
	Sub = 26,
	TenA = 27,
	TenB = 28
};

// The following are some definitions for use with the output options in
// MoorDyn. These are for the global output quantities specified by OutList, not
// line-specific outputs. Output definitions follow the structure described by
// the MD_OutParmType . Each output channel is described by the following
// fields:
//  Name   - (string) what appears at the top of the output column
//  Units  - (string) selected from UnitList (see below) based on index QType
//  OType  - (int) the type of object the output is from. 1=line, 2=point
//  (0=invalid) ObjID  - (int) the ID number of the line or point QType  -
//  (int) the type of quantity to output.  0=tension, 1=x pos, etc.  see the
//  parameters below NodeID - (int) the ID number of the node of the output
//  quantity
//
// These are the "OTypes": 0=Point object, 1=Line Object
// (will just use 0 and 1 rather than parameter names)
//
// Indices for computing output channels:  - customized for the MD_OutParmType
// approach these are the "QTypes"
//
// UnitList is in MoorDyn.cpp
typedef struct _OutChanProps
{ // this is C version of MDOutParmType - a less literal alternative of the NWTC
  // OutParmType for MoorDyn (to avoid huge lists of possible output channel
  // permutations)
	string Name;     // "name of output channel"
	string Units;    // "units string"
	QTypeEnum QType; // "type of quantity - 0=tension, 1=x, 2=y, 3=z..."
	int OType;       // "type of object - 1=line, 2=point, 3=rod, 4=body"
	int NodeID; // "node number if OType = 1 or 3. -1 indicated whole rod or
	            // fairlead for line"
	int ObjID;  // "number of Point or Line object", subtract 1 to get the
	            // index in the LineList or PointList
} OutChanProps;
