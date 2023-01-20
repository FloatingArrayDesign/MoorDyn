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

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <complex>
#include <utility>

#include <fstream>
#include <sstream>
#include <cstring>

#include <memory>

// #ifdef USEGL
//  #include <GL/gl.h>  // for openGL drawing option
//  #include <GL/glu.h> // used in arrow function
// #endif

#ifdef OSX
#include <sys/uio.h>
#elif defined WIN32
#include <windows.h> // these are for guicon function RedirectIOToConsole
#include <io.h>
#endif

#include <stdio.h>
#include <fcntl.h>
#include <iostream>

// note: this file contains the struct definitions for environmental and
// line/connect properties

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
#else
typedef double real;
typedef Eigen::Vector2d vec2;
typedef Eigen::Vector3d vec3;
typedef Eigen::Vector4d vec4;
typedef Eigen::Vector6d vec6;
typedef vec3 vec;
typedef Eigen::Matrix2d mat2;
typedef Eigen::Matrix3d mat3;
typedef Eigen::Matrix4d mat4;
typedef Eigen::Matrix6d mat6;
typedef mat3 mat;
#endif
typedef Eigen::Vector2i ivec2;
typedef Eigen::Vector3i ivec3;
typedef Eigen::Vector4i ivec4;
typedef Eigen::Vector6i ivec6;
typedef ivec3 ivec;

typedef std::complex<real> complex;

/// The imaginary unit
const complex i1(0., 1.);

/** @brief Convert a vector to a C-ish array
 * @param v The input vector
 * @param a The output array
 */
template<typename T>
void
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
void
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

#ifndef ASCII_A
/// The ASCII value of the A character
#define ASCII_A 10
#endif

/** @brief Gives an character representation of the end point
 * @return The endpoint char
 */
inline char
end_point_name(EndPoints p)
{
	return char(ASCII_A + (int)p);
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
	return split(s, ' ');
}

/** @brief Split a string into separate letter strings and integers
 */
int
decomposeString(char outWord[10],
                char let1[10],
                char num1[10],
                char let2[10],
                char num2[10],
                char let3[10]);

} // ::moordyn::str

/**
 * @}
 */

/** \defgroup environment Environmental variables
 *  @{
 */

/** @brief Available settings for waves
 */
typedef enum
{
	/// No waves
	WAVES_NONE = 0,
	/// Waves externally provided
	WAVES_EXTERNAL = 1,
	/// Wave elevation FFT, grid approach
	WAVES_FFT_GRID = 2,
	/// Wave elevation time series, grid approach
	WAVES_GRID = 3,
	/// Wave elevation FFT, node approach
	WAVES_FFT_NODE = 4,
	/// Wave elevation time series, node approach
	WAVES_NODE = 5,
	/// velocity, acceleration, and wave elevation grid data
	WAVES_KIN = 6,
} waves_settings;

// Current options: 0 - no currents or set externally (as part of WaveKin =0 or
// 1 approach) [default]
//                  1 - read in steady current profile, grid approach
//                  (current_profile.txt)** 2 - read in dynamic current profile,
//                  grid approach (current_profile_dynamic.txt)** 3 - read in
//                  steady current profile, node approach (current_profile.txt)
//                  4 - read in dynamic current profile, node approach
//                  (current_profile_dynamic.txt)

/** @brief Available settings for currents
 */
typedef enum
{
	/// No currents
	CURRENTS_NONE = 0,
	/// steady current profile, grid approach
	CURRENTS_STEADY_GRID = 1,
	/// dynamic current profile, grid approach
	CURRENTS_DYNAMIC_GRID = 2,
	/// steady current profile, node approach
	CURRENTS_STEADY_NODE = 3,
	/// dynamic current profile, node approach
	CURRENTS_DYNAMIC_NODE = 4,
	/// 4D current profile
	CURRENTS_4D = 5;
} currents_settings;

/** @brief Are the waves settings grid based?
 * @param opt Waves settings
 * @return true if the waves are provided in a grid, false otherwise
 */
inline bool
is_waves_grid(waves_settings opt)
{
	if (opt == WAVES_FFT_GRID)
		return true;
	if (opt == WAVES_GRID)
		return true;
	return false;
}

/** @brief Are the waves settings node based?
 * @param opt Waves settings
 * @return true if the waves are provided in the nodes, false otherwise
 */
inline bool
is_waves_node(waves_settings opt)
{
	if (opt == WAVES_FFT_NODE)
		return true;
	if (opt == WAVES_NODE)
		return true;
	return false;
}

/** @brief Are the currents settings grid based?
 * @param opt Currents settings
 * @return true if the currents are provided in a grid, false otherwise
 */
inline bool
is_currents_grid(currents_settings opt)
{
	if (opt == CURRENTS_STEADY_GRID)
		return true;
	if (opt == CURRENTS_DYNAMIC_GRID)
		return true;
	return false;
}

/** @brief Are the currents settings node based?
 * @param opt Currents settings
 * @return true if the currents are provided in the nodes, false otherwise
 */
inline bool
is_currents_node(currents_settings opt)
{
	if (opt == CURRENTS_STEADY_NODE)
		return true;
	if (opt == CURRENTS_DYNAMIC_NODE)
		return true;
	return false;
}

/**
 * @}
 */

/** \defgroup interpolation Interpolation utilities
 *  @{
 */

/** @brief One-dimensional linear interpolation factor
 * @param xp The points where data is available
 * @param i0 The starting index to look for the upper bound
 * @param x The evaluation point
 * @param f The interpolation factor
 * @return The index of the upper bound
 */
template<typename T>
inline unsigned int
interp_factor(const vector<T>& xp, unsigned int i0, const T& x, T& f)
{
	if (xp.size() == 1) {
		f = 0.0;
		return 0;
	}

	if (i0 == 0)
		i0++;
	if (i0 > xp.size() - 1)
		i0 = xp.size() - 1;

	if (x <= xp[i0 - 1]) {
		f = 0.0;
		return i0;
	}
	if (x >= xp.back()) {
		f = 1.0;
		return xp.size() - 1;
	}

	for (unsigned i = i0; i < xp.size() - 1; i++) {
		if (x <= xp[i]) {
			f = (x - xp[i - 1]) / (xp[i] - xp[i - 1]);
			return i;
		}
	}

	// Just to avoid the compiler warnings. This point is actually never reached
	f = 1.0;
	return xp.size() - 1;
}

/** @brief One-dimensional linear interpolation factor
 *
 * This function is equivalent to calling interp_factor(xp, 1, x, f)
 * @param xp The points where data is available
 * @param x The evaluation point
 * @param f The interpolation factor
 * @return The index of the upper bound
 */
template<typename T>
inline unsigned int
interp_factor(const vector<T>& xp, const T& x, T& f)
{
	return interp_factor(xp, 1, x, f);
}

/** @brief One-dimensional linear interpolation
 *
 * For monotonically increasing sample points.
 * @param xp The points where data is available
 * @param yp The data values
 * @param x The evaluation points
 * @param y The interpolated values. The vector shall be already initialized
 * with at least the same number of components than \p x
 */
template<typename Tx, typename Ty>
inline void
interp(const vector<Tx>& xp,
       const vector<Ty>& yp,
       const vector<Tx>& x,
       vector<Ty>& y)
{
	if (yp.size() == 1) {
		y[0] = yp[0];
		return;
	}

	real f;
	unsigned int j = 1;
	for (unsigned int i = 0; i < x.size(); i++) {
		j = interp_factor(xp, j, x[i], f);
		y[i] = yp[j - 1] + f * (yp[j] - yp[j - 1]);
	}
}

/** @brief One-dimensional linear interpolation
 * @param xp The points where data is available
 * @param yp The data values
 * @param x The evaluation point
 * @return The interpolated value
 */
template<typename Tx, typename Ty>
inline Ty
interp(const vector<Tx>& xp, const vector<Ty>& yp, Tx x)
{
	if (yp.size() == 1) {
		return yp[0];
	}

	real f;
	const auto j = interp_factor(xp, 1, x, f);
	return yp[j - 1] + f * (yp[j] - yp[j - 1]);
}

/** @brief Bilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp2(const vector<vector<T>>& values,
        unsigned int i,
        unsigned int j,
        T fx,
        T fy)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;

	T c00 = values[i0][j0];
	T c01 = values[i0][j];
	T c10 = values[i][j0];
	T c11 = values[i][j];

	T c0 = c00 * (1. - fx) + c10 * fx;
	T c1 = c01 * (1. - fx) + c11 * fx;

	return c0 * (1 - fy) + c1 * fy;
}

/** @brief Trilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param k The upper bound index in the z direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @param fz The linear interplation factor in the z direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp3(const vector<vector<vector<T>>>& values,
        unsigned int i,
        unsigned int j,
        unsigned int k,
        T fx,
        T fy,
        T fz)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;
	unsigned int k0 = k > 0 ? k - 1 : 0;

	T c000 = values[i0][j0][k0];
	T c001 = values[i0][j0][k];
	T c010 = values[i0][j][k0];
	T c011 = values[i0][j][k];
	T c100 = values[i][j0][k0];
	T c101 = values[i][j0][k];
	T c110 = values[i][j][k0];
	T c111 = values[i][j][k];

	T c00 = c000 * (1. - fx) + c100 * fx;
	T c01 = c001 * (1. - fx) + c101 * fx;
	T c10 = c010 * (1. - fx) + c110 * fx;
	T c11 = c011 * (1. - fx) + c111 * fx;

	T c0 = c00 * (1. - fy) + c10 * fy;
	T c1 = c01 * (1. - fy) + c11 * fy;

	return c0 * (1 - fz) + c1 * fz;
}

/** @brief Quadrilinear filter
 * @param values The available data
 * @param i The upper bound index in the x direction
 * @param j The upper bound index in the y direction
 * @param k The upper bound index in the z direction
 * @param w The upper bound index in the w direction
 * @param fx The linear interplation factor in the x direction
 * @param fy The linear interplation factor in the y direction
 * @param fz The linear interplation factor in the z direction
 * @param fw The linear interplation factor in the w direction
 * @return The linearly interpolated value
 * @see interp_factor
 */
template<typename T>
inline T
interp4(const vector<vector<vector<vector<T>>>>& values,
        unsigned int i,
        unsigned int j,
        unsigned int k,
        unsigned int w,
        T fx,
        T fy,
        T fz,
        T fw)
{
	unsigned int i0 = i > 0 ? i - 1 : 0;
	unsigned int j0 = j > 0 ? j - 1 : 0;
	unsigned int k0 = k > 0 ? k - 1 : 0;
	unsigned int w0 = w > 0 ? w - 1 : 0;

	T c000 = values[i0][j0][k0][w0] * fw + values[i0][j0][k0][w] * (1. - fw);
	T c001 = values[i0][j0][k][w0] * fw + values[i0][j0][k][w] * (1. - fw);
	T c010 = values[i0][j][k0][w0] * fw + values[i0][j][k0][w] * (1. - fw);
	T c011 = values[i0][j][k][w0] * fw + values[i0][j][k][w] * (1. - fw);
	T c100 = values[i][j0][k0][w0] * fw + values[i0][j0][k0][w] * (1. - fw);
	T c101 = values[i][j0][k][w0] * fw + values[i0][j0][k][w] * (1. - fw);
	T c110 = values[i][j][k0][w0] * fw + values[i0][j][k0][w] * (1. - fw);
	T c111 = values[i][j][k][w0] * fw + values[i0][j0][k0][w] * (1. - fw);

	T c00 = c000 * (1. - fx) + c100 * fx;
	T c01 = c001 * (1. - fx) + c101 * fx;
	T c10 = c010 * (1. - fx) + c110 * fx;
	T c11 = c011 * (1. - fx) + c111 * fx;

	T c0 = c00 * (1. - fy) + c10 * fy;
	T c1 = c01 * (1. - fy) + c11 * fy;

	return c0 * (1 - fz) + c1 * fz;
}

/**
 * @}
 */

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
	vec v = r2 - r1;
	const double l = v.norm();
	u = v / l;
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
	if (l2 == 0.0) {
		y = u;
		return;
	}
	const moordyn::real scaler = (moordyn::real)newlength / sqrt(l2);
	y = scaler * u;
}

/** @brief Produce alternator matrix
 *
 * See "anti-symmetric tensor components" from Sadeghi and Incecik
 * @param r Offset vector
 * @return Alternator matrix
 */
inline mat
getH(vec r)
{
	mat H;
	H << 0, r[2], r[1], r[2], 0, r[0], r[1], r[0], 0;
	return H;
}

/** @brief Compute the mass matrix on an offset point
 * @param r Offset
 * @param M Mass matrix
 * @return Translated Mass & Inertia matrix
 */
mat6
translateMass(vec r, mat M);

/** @brief Compute the mass matrix on an offset point
 * @param r Offset
 * @param M Mass & Inertia matrix
 * @return Translated Mass & Inertia matrix
 */
mat6
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
mat6
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

/** @brief Rotation matrix around z axis
 * @param x The angle around x axis in radians
 * @param y The angle around y axis in radians
 * @param z The angle around z axis in radians
 * @return The rotation matrix
 */
inline mat
RotXYZ(real x, real y, real z)
{
	return RotX(x) * RotY(y) * RotZ(z);
}

/** @brief Euler XYZ rotation matrix
 * @param rads The angles in radians
 * @return The rotation matrix
 */
inline mat
RotXYZ(vec rads)
{
	return RotXYZ(rads[0], rads[1], rads[2]);
}

/** @brief Get the spherical angles for a vector
 * @param q The vector
 * @return The orientation angles, i.e. inclination and heading (in radians)
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
const real pi = M_PI;
#endif
/// Constant to convert radians into degrees
const real rad2deg = 180.0 / pi;

/** \defgroup entities_properties Properties readed from the input file, used
 * to define the entities handled during the simulation
 *  @{
 */

class Rod;
class Connection;
class Line;

/** @brief Failure conditions
 */
typedef struct _FailProps
{
	/// The rod the lines are attached to, if any. Otherwise it is NULL
	Rod* rod;
	/// The rod attachment end point, useless if rod is NULL
	EndPoints rod_end_point;
	/// The connection the lines are attached to, if any. Otherwise it is NULL
	Connection* conn;
	/// The attached lines
	std::vector<Line*> lines;
	/// The attached line end points. This is actually an array to be filled
	/// with the end points where each line is dettached
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

typedef struct
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
	/// wave kinematics flag (0=off, >0=on...)<<<
	moordyn::waves_settings WaveKin;
	/// current flag (0=off, >0=on...)<<<
	moordyn::currents_settings Current;
	/// time step used to downsample wave elevation data with
	double dtWave;

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
} EnvCond;

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
} RodProps;

typedef struct _ConnectProps // matching node input stuff
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
} ConnectProps;

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

typedef struct _OutChanProps
{ // this is C version of MDOutParmType - a less literal alternative of the NWTC
  // OutParmType for MoorDyn (to avoid huge lists of possible output channel
  // permutations)
	char Name[10];  // "name of output channel"
	char Units[10]; // "units string"     - should match UnitsSize in
	                // MoorDyn.cpp (should turn into def)
	int QType;      // "type of quantity - 0=tension, 1=x, 2=y, 3=z..."
	int OType;      // "type of object - 1=line, 2=connect"
	int NodeID;     // "node number if OType=1.  0=anchor, -1=N=Fairlead"
	int ObjID;      // "number of Connect or Line object", subtract 1 to get the
	                // index in the LineList or ConnectList
} OutChanProps;

// --------------------------- Output definitions
// -----------------------------------------

// The following are some definitions for use with the output options in
// MoorDyn. These are for the global output quantities specified by OutList, not
// line-specific outputs. Output definitions follow the structure described by
// the MD_OutParmType . Each output channel is described by the following
// fields:
//  Name   - (string) what appears at the top of the output column
//  Units  - (string) selected from UnitList (see below) based on index QType
//  OType  - (int) the type of object the output is from. 1=line, 2=connect
//  (0=invalid) ObjID  - (int) the ID number of the line or connect QType  -
//  (int) the type of quantity to output.  0=tension, 1=x pos, etc.  see the
//  parameters below NodeID - (int) the ID number of the node of the output
//  quantity
//
// These are the "OTypes": 0=Connect object, 1=Line Object
// (will just use 0 and 1 rather than parameter names)
//
// Indices for computing output channels:  - customized for the MD_OutParmType
// approach these are the "QTypes"

const int Time = 0;
const int PosX = 1;
const int PosY = 2;
const int PosZ = 3;
const int VelX = 4;
const int VelY = 5;
const int VelZ = 6;
const int AccX = 7;
const int AccY = 8;
const int AccZ = 9;
const int Ten = 10;
const int FX = 11;
const int FY = 12;
const int FZ = 13;

// UnitList is in MoorDyn.cpp

// vector<string> strvector(strarray, strarray + 3);

// // List of units corresponding to the quantities parameters for QTypes
//  struct Units
// {
//	  char Time[10]    = "(s)      ";
//	  char PosX[10]    = "(m)      ";
//	  char PosY[10]    = "(m)      ";
//	  char PosZ[10]    = "(m)      ";
//	  char VelX[10]    = "(m/s)    ";
//	  char VelY[10]    = "(m/s)    ";
//	  char VelZ[10]    = "(m/s)    ";
//	  char AccX[10]    = "(m/s2)   ";
//	  char AccY[10]    = "(m/s2)   ";
//	  char AccZ[10]    = "(m/s2)   ";
//	  char Ten [10]    = "(N)      ";
//	  char FX  [10]    = "(N)      ";
//	  char FY  [10]    = "(N)      ";
//	  char FZ  [10]    = "(N)      ";
// };
