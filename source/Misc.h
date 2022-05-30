/*
 * Copyright (c) 2014 Matt Hall <mtjhall@alumni.uvic.ca>
 * 
 * This file is part of MoorDyn.  MoorDyn is free software: you can redistribute 
 * it and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 * 
 * MoorDyn is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MoorDyn.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MISC_H
#define MISC_H

#include "MoorDynAPI.h"
#include "Eigen/Dense"

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <complex>


#include <fstream>
#include <sstream>
#include <cstring>

#include <memory>

//#ifdef USEGL
// #include <GL/gl.h>  // for openGL drawing option
// #include <GL/glu.h> // used in arrow function
//#endif

#include "kiss_fftr.h"  // used for any wave kinematics functions

#ifdef OSX
 #include <sys/uio.h>
#elif defined WIN32
 #include <windows.h>  // these are for guicon function RedirectIOToConsole
 #include <io.h>
#endif

#include <stdio.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

// note: this file contains the struct definitions for environmental and line/connect properties


// from Iñaki Zabala
#ifdef _MSC_VER
template<typename T> static inline T round(T val) {return floor(val + 0.5);}
#endif

using namespace std;

#ifdef MOORDYN_SINGLEPRECISSION
typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef vec3 vec;
typedef Eigen::Matrix2f mat2;
typedef Eigen::Matrix3f mat3;
typedef Eigen::Matrix4f mat4;
typedef mat3 mat;
#else
typedef Eigen::Vector2d vec2;
typedef Eigen::Vector3d vec3;
typedef Eigen::Vector4d vec4;
typedef vec3 vec;
typedef Eigen::Matrix2d mat2;
typedef Eigen::Matrix3d mat3;
typedef Eigen::Matrix4d mat4;
typedef mat3 mat;
#endif
typedef Eigen::Vector2i ivec2;
typedef Eigen::Vector3i ivec3;
typedef Eigen::Vector4i ivec4;
typedef ivec3 ivec;

namespace moordyn
{

#ifdef MOORDYN_SINGLEPRECISSION
typedef float real;
#else
typedef double real;
#endif

typedef std::complex<real> complex;

/// The imaginary unit
const complex i1(0., 1.);

/** @brief Convert a vector to a C-ish array
 * @param v The input vector
 * @param a The output array
 */
template <typename T>
inline void vec2array(const vec &v, T *a)
{
	a[0] = (T)v[0];
	a[1] = (T)v[1];
	a[2] = (T)v[2];
}

/** @brief Convert a C-ish array to a vector
 * @param a The input array
 * @param v The output vector
 */
template <typename T>
inline void array2vec(const T *a, vec &v)
{
	v[0] = (moordyn::real)a[0];
	v[1] = (moordyn::real)a[1];
	v[2] = (moordyn::real)a[2];
}

/** @brief Convert a matrix to a C-ish array
 * @param v The input matrix
 * @param a The output array
 */
template <typename T>
inline void mat2array(const mat &v, T a[3][3])
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
template <typename T>
inline void array2mat(const T a[3][3], mat &v)
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


/** \addtogroup moordyn_errors
 *  @{
 */

/// Error identifier
typedef int error_id;

/// Simple macro to define custom exceptions
#define MAKE_EXCEPTION(name)                                                    \
	class name : public std::runtime_error                                      \
	{                                                                           \
	public:                                                                     \
		name(const char* msg) : std::runtime_error(msg) {}                      \
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
#define MOORDYN_THROW(err, msg)                                                 \
	switch(err) {                                                               \
		case MOORDYN_SUCCESS:                                                   \
			break;                                                              \
		case MOORDYN_INVALID_INPUT_FILE:                                        \
			throw moordyn::input_file_error(msg);                               \
			break;                                                              \
		case MOORDYN_INVALID_OUTPUT_FILE:                                       \
			throw moordyn::output_file_error(msg);                              \
			break;                                                              \
		case MOORDYN_INVALID_INPUT:                                             \
			throw moordyn::input_error(msg);                                    \
			break;                                                              \
		case MOORDYN_NAN_ERROR:                                                 \
			throw moordyn::nan_error(msg);                                      \
			break;                                                              \
		case MOORDYN_MEM_ERROR:                                                 \
			throw moordyn::mem_error(msg);                                      \
			break;                                                              \
		case MOORDYN_INVALID_VALUE:                                             \
			throw moordyn::invalid_value_error(msg);                            \
			break;                                                              \
		case MOORDYN_NON_IMPLEMENTED:                                           \
			throw moordyn::non_implemented_error(msg);                          \
			break;                                                              \
		default:                                                                \
			throw moordyn::unhandled_error(msg);                                \
			break;                                                              \
	}

/// Catch thrown exceptions and convert them in an error_id. It also gives the
/// message on the Exception. This macro will only handle known exceptions, i.e.
/// the ones declared in moordyn_errors. You can add more catch() instances
/// afterwards
#define MOORDYN_CATCHER(err, msg)                                               \
	catch(moordyn::input_file_error const& e) {                                 \
		err = MOORDYN_INVALID_INPUT_FILE;                                       \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::output_file_error const& e) {                                \
		err = MOORDYN_INVALID_OUTPUT_FILE;                                      \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::input_error const& e) {                                      \
		err = MOORDYN_INVALID_INPUT;                                            \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::nan_error const& e) {                                        \
		err = MOORDYN_NAN_ERROR;                                                \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::mem_error const& e) {                                        \
		err = MOORDYN_MEM_ERROR;                                                \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::invalid_value_error const& e) {                              \
		err = MOORDYN_INVALID_VALUE;                                            \
		msg = e.what();                                                         \
	}                                                                           \
	catch(moordyn::unhandled_error const& e) {                                  \
		err = MOORDYN_UNHANDLED_ERROR;                                          \
		msg = e.what();                                                         \
	}

/**
 * @}
 */

/** \addtogroup string_tools
 *  @{
 */

namespace str
{

/** @brief Convert a string to lower case
 * @param str String to check
 * @return A lower case copy of the string
 */
string lower(const string &str);

/** @brief Convert a string to lower case
 * @param str String to check
 * @return A lower case copy of the string
 */
string upper(const string &str);

/** @brief Check if a string contains one of the provided terms
 * @param str String to check
 * @param terms List of terms to look for
 * @return true if the string contains one or more of the terms, false
 * otherwise
 */
bool has(const string &str, const vector<string> terms);

/** @brief Split a string in a list of substrings
 * @param str String to split
 * @param sep Separator
 */
vector<string> split(const string &str, const char sep);

}  // ::moordyn::str

/**
 * @}
 */

/** \defgroup environment Environmental variables
 *  @{
 */

/** @brief Available settings for waves
 */
typedef enum {
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

	// Current options: 0 - no currents or set externally (as part of WaveKin =0 or 1 approach) [default]
    //                  1 - read in steady current profile, grid approach (current_profile.txt)**
    //                  2 - read in dynamic current profile, grid approach (current_profile_dynamic.txt)**
    //                  3 - read in steady current profile, node approach (current_profile.txt)
    //                  4 - read in dynamic current profile, node approach (current_profile_dynamic.txt)


/** @brief Available settings for currents
 */
typedef enum {
	/// No currents
	CURRENTS_NONE = 0,
	/// steady current profile, grid approach
	CURRENTS_STEADY_GRID = 1,
	/// dynamic current profile, grid approach
	CURRENTS_DYNAMIC_GRID = 2,
	/// steady current profile, node approach
	CURRENTS_STEADY_NODE = 1,
	/// dynamic current profile, node approach
	CURRENTS_DYNAMIC_NODE = 2,
} currents_settings;

/** @brief Are the waves settings grid based?
 * @param opt Waves settings
 * @return true if the waves are provided in a grid, false otherwise
 */
inline bool is_waves_grid(waves_settings opt)
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
inline bool is_waves_node(waves_settings opt)
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
inline bool is_currents_grid(currents_settings opt)
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
inline bool is_currents_node(currents_settings opt)
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
template <typename T>
inline unsigned int interp_factor(const vector<T> &xp,
                                  unsigned int i0,
                                  const T &x,
                                  T &f)
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

	for (unsigned i = i0; i < xp.size() - 1; i++)
	{
		if (x <= xp[i])
		{
			f = (x - xp[i - 1]) / (xp[i] - xp[i - 1] );
			return i;
		}
	}

	// Justto avoid the compiler warnings. This point is actually never reached
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
template <typename T>
inline unsigned int interp_factor(const vector<T> &xp,
                                  const T &x,
                                  T &f)
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
template <typename T>
inline void interp(const vector<T> &xp,
                   const vector<T> &yp,
                   const vector<T> &x,
                   vector<T> &y)
{
	real f;
	unsigned int j = 1;
	for (unsigned int i = 0; i < x.size(); i++)
	{
		j = interp_factor(xp, j, x[i], f);
		y[i] = yp[j - 1] + f * (yp[j] - yp[j - 1]);
	}
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
template <typename T>
inline T interp2(const vector<vector<T>> &values,
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
template <typename T>
inline T interp3(const vector<vector<vector<T>>> &values,
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
template <typename T>
inline T interp4(const vector<vector<vector<vector<T>>>> &values,
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

}  // ::moordyn


const double pi = 3.14159265;
const double rad2deg = 57.29577951;

const int wordy = 1;   			// flag to enable excessive output (if > 0) for troubleshooting

const int nCoef = 30;   // maximum number of entries to allow in nonlinear coefficient lookup tables

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
	/// The log file stream
	ofstream *outfileLogPtr;
} EnvCond;



typedef struct _LineProps // (matching Line Dictionary inputs)
{
	string type; 
	double d;
	double w;		// linear weight in air
	double EA;
	double EI;
	double c;    	// internal damping
	double cI;
	double Can;
	double Cat;
	double Cdn;
	double Cdt;	
	int nEApoints;        // number of values in stress-strain lookup table (0 means using constant E)
	double stiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double stiffYs[nCoef]; // y array for stress-strain lookup table
	int nCpoints;        // number of values in stress-strainrate lookup table (0 means using constant c)
	double dampXs[nCoef]; // x array for stress-strainrate lookup table (up to nCoef)
	double dampYs[nCoef]; // y array for stress-strainrate lookup table	
	int nEIpoints = 0; // number of values in bending stress-strain lookup table (0 means using constant E)
	double bstiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double bstiffYs[nCoef]; // y array for stress-strain lookup table
} LineProps;

typedef struct _RodProps // (matching Rod Dictionary inputs)
{
	string type; 
	double d;
	double w;		// linear weight in air
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
	double CdA;  // added 2015/1/15 - product of drag coefficient and frontal area
	double Ca;  // added 2015/1/15  - added mass coefficient
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

typedef struct _FailProps    // for failure conditions
{
	int attachID;         // ID of connection or Rod the lines are attached to (index is -1 this value)
	int isRod;             // 1 Rod end A, 2 Rod end B, 0 if connection
	int lineIDs[30];        // array of one or more lines to detach (starting from 1...)
	int lineTops[30];       // an array that will be FILLED IN to return which end of each line was disconnected ... 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	int nLinesToDetach;  // how many lines to dettach
	double failTime;
	double failTen;        // N
	int failStatus;    // 0 not failed yet, 1 failed
} FailProps;

typedef struct _OutChanProps
{  // this is C version of MDOutParmType - a less literal alternative of the NWTC OutParmType for MoorDyn (to avoid huge lists of possible output channel permutations)                                                                         
	char Name[10]; 		// "name of output channel"   
	char Units[10];			// "units string"     - should match UnitsSize in MoorDyn.cpp (should turn into def)                                                                            
	int QType;     		// "type of quantity - 0=tension, 1=x, 2=y, 3=z..."                                         
	int OType;     		// "type of object - 1=line, 2=connect"                                                                              
	int NodeID;    		// "node number if OType=1.  0=anchor, -1=N=Fairlead"                                                              
	int ObjID;     		// "number of Connect or Line object", subtract 1 to get the index in the LineList or ConnectList
} OutChanProps;


  // --------------------------- Output definitions -----------------------------------------

  // The following are some definitions for use with the output options in MoorDyn.
  // These are for the global output quantities specified by OutList, not line-specific outputs.
  // Output definitions follow the structure described by the MD_OutParmType .
  // Each output channel is described by the following fields:
  //  Name   - (string) what appears at the top of the output column
  //  Units  - (string) selected from UnitList (see below) based on index QType
  //  OType  - (int) the type of object the output is from. 1=line, 2=connect (0=invalid)
  //  ObjID  - (int) the ID number of the line or connect
  //  QType  - (int) the type of quantity to output.  0=tension, 1=x pos, etc.  see the parameters below
  //  NodeID - (int) the ID number of the node of the output quantity
  // 
  // These are the "OTypes": 0=Connect object, 1=Line Object
  // (will just use 0 and 1 rather than parameter names)
  // 
  // Indices for computing output channels:  - customized for the MD_OutParmType approach
  // these are the "QTypes"
 
	  const int Time    =    0 ;
	  const int PosX    =    1 ;
	  const int PosY    =    2 ;
	  const int PosZ    =    3 ;
	  const int VelX    =    4 ;
	  const int VelY    =    5 ;
	  const int VelZ    =    6 ;
	  const int AccX    =    7 ;
	  const int AccY    =    8 ;
	  const int AccZ    =    9 ;
	  const int Ten     =    10;
	  const int FX      =    11;
	  const int FY      =    12;
	  const int FZ      =    13;

	  // UnitList is in MoorDyn.cpp
	  
	  
	  
	  //vector<string> strvector(strarray, strarray + 3);
	  
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




// below are function prototypes for misc functions

// TODO: replace duplicates for different data types with templated functions <<< (or remove vector types)

void interpArray(int ndata, int n, double *xdata, double *ydata, double *xin, double *yout);
void interpArray(int ndata, int n, vector<double> xdata, vector<double> ydata, vector<double> xin, vector<double> &yout);

double calculate4Dinterpolation(double**** f, int ix0, int iy0, int iz0, int it0, double fx, double fy, double fz, double ft);
double calculate3Dinterpolation(double*** f, int ix0, int iy0, int iz0, double fx, double fy, double fz);
double calculate2Dinterpolation(double** f, int ix0, int iy0, double fx, double fy);
int getInterpNums(double *xlist, int nx, double xin, double *fout);
void getInterpNums(double *xlist, int nx, double xin, double fout[2], int iout[2]);

moordyn::real GetCurvature(moordyn::real length, const vec& q1, const vec& q2);

void GetOrientationAngles(double q[3], double* phi, double* sinPhi, double* cosPhi, 
                          double* tanPhi, double* beta, double* sinBeta, double* cosBeta);

int decomposeString(char outWord[10], char let1[10], 
     char num1[10], char let2[10], char num2[10], char let3[10]);

/** @brief Eye matrix component
 *
 * Simply returns 1.0 if both component indexes matches, 0 otherwise
 * @param I Row
 * @param J Column
 * @return The eye matrix component
 */
inline double eye(int I, int J)
{
	return (double)(I == J);
}

void getH(double r[3], double H[3][3]);
void getH(double r[3], double H[9]);

/** @brief Get the length of a vector
 * @param v The vector
 * @return The vector length
 */
template <typename T>
inline double vectorLength(T *v)
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * @{
 */

/** @brief Normalized direction vector
 * @param u The output normalized direction vector
 * @param r1 The orig point
 * @param r2 The dest point
 * @return The distance between the points
 */
template <typename T>
inline double unitvector(T *u, const T *r1, const T *r2)
{
	const T v[3] = {r2[0] - r1[0], r2[1] - r1[1], r2[2] - r1[2]};
	const double l = vectorLength(v);
	u[0] = v[0] / l;
	u[1] = v[1] / l;
	u[2] = v[2] / l;
	return l;
}

inline double unitvector(vec &u, const vec &r1, const vec &r2)
{
	vec v = r2 - r1;
	const double l = v.norm();
	u = v / l;
	return l;
}

inline double unitvector(double *u, const vec &r1, const vec &r2)
{
	vec v = r2 - r1;
	const double l = v.norm();
	moordyn::vec2array(v / l, u);
	return l;
}

template <typename T>
inline double unitvector(vector<T> &u, vector<T> & r1, vector<T> & r2)
{
	return unitvector(u.data(), r1.data(), r2.data());
}

template <typename T>
inline double unitvector(T *u, vector<T> & r1, vector<T> & r2)
{
	return unitvector(u, r1.data(), r2.data());
}

/**
 * @}
 */

void transposeM3(double A[3][3], double Atrans[3][3]);
void transposeM3(double A[9], double Atrans[9]);

void addM6(double Min1[6][6], double Min2[6][6], double Mout[6][6]);

void multiplyM3(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3(double A[9], double B[9], double C[9]);

void multiplyM3AtransB(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3AtransB(double A[9], double B[9], double C[9]);

void multiplyM3ABtrans(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3ABtrans(double A[9], double B[9], double C[9]);

double distance3d( double* r1, double* r2);

double dotProd( vector<double>& A, vector<double>& B);
double dotProd( double A[], vector<double>& B);
double dotProd( double A[], double B[]);

/**
 * @{
 */

/** @brief Inner product of 3D vectors
 * @param a First vector
 * @param b Second vector
 * @return the inner product result
 * @note This function is way faster than any other general solution
 */
template <typename T>
inline T dotProd3(const T *a, const T *b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <typename T>
inline moordyn::real __attribute__((deprecated)) dotProd3(const vec& a, const T *b)
{
	vec bb;
	moordyn::array2vec(b, bb);
	return a.dot(bb);
}

template <typename T>
inline moordyn::real __attribute__((deprecated)) dotProd3(const T *a, const vec& b)
{
	vec aa;
	moordyn::array2vec(a, aa);
	return aa.dot(b);
}

/**
 * @}
 */

/**
 * @{
 */

/** @brief Compute a nw vector with the same direction of the provided one, but
 * a new length
 * @param u The vector to set the direction
 * @param newlength The new length
 * @param y The output vector
 * @not If the input vector has null length, the output vector will as well,
 * no matter @p newlength is provided
 */
template <typename T>
inline void scalevector(const T *u, T newlength, T *y)
{
	const double l2 = dotProd3(u, u);
	if (l2 == 0.0)
	{
		memcpy(y, u, 3 * sizeof(double));
		return;
	}
	const double scaler = newlength / sqrt(l2);
	y[0] = scaler * u[0];
	y[1] = scaler * u[1];
	y[2] = scaler * u[2];
}

template <typename T>
inline void scalevector(const vec &u, T newlength, vec &y)
{
	const moordyn::real l2 = u.squaredNorm();
	if (l2 == 0.0)
	{
		y = u;
		return;
	}
	const moordyn::real scaler = (moordyn::real)newlength / sqrt(l2);
	y = scaler * u;
}

template <typename T>
inline void scalevector(vector<T> &u, T newlength, vector<T> &y)
{
	scalevector(u.data(), newlength, y.data());
}

/**
 * @}
 */

/** @brief 3D cross vector product
 * @param u First vector
 * @param v Second vector
 * @param out The result
 */
template <typename T>
inline void crossProd(const T *u, const T *v, T *out)
{
	out[0] = u[1] * v[2] - u[2] * v[1];
	out[1] = u[2] * v[0] - u[0] * v[2];
	out[2] = u[0] * v[1] - u[1] * v[0];
}

template <typename T>
inline double crossProd(vector<T> &u, vector<T> &v, T *out)
{
	return crossProd(u.data(), v.data(), out);
}

template <typename T>
inline double crossProd(vector<T> &u, const T* v, T *out)
{
	return crossProd(u.data(), v, out);
}

void inverse3by3( vector< vector< double > > & minv, vector< vector< double > > & m);

void Crout(int d,double*S,double*D);
void __attribute__((deprecated)) solveCrout(int d,double*LU,double*b,double*x);

template <typename TwoD1, typename TwoD2>
void __attribute__((deprecated)) LUsolve(int n, TwoD1& A, TwoD2& LU, double*b, double *y, double*x)
{
	// Solves Ax=b for x
	// LU contains LU matrices, y is a temporary vector
	// all dimensions are n

	for(int k=0; k<n; ++k)
	{
		for(int i=k; i<n; ++i)
		{
			double sum=0.;

			for(int p=0; p<k; ++p)
				sum += LU[i][p] * LU[p][k];
			LU[i][k] = A[i][k] - sum; // not dividing by diagonals
		}
		for(int j=k+1;j<n;++j)
		{
			double sum=0.;
			for(int p=0;p<k;++p)
				sum += LU[k][p] * LU[p][j];
			LU[k][j] = (A[k][j] - sum) / LU[k][k];
		}
	}

	for(int i=0; i<n; ++i)
	{
		double sum=0.;
		for(int k=0; k<i; ++k)
			sum += LU[i][k] * y[k];
		y[i] = (b[i] - sum) / LU[i][i];
	}
	for(int i=n-1; i>=0; --i)
	{
		double sum=0.;
		for(int k=i+1; k<n; ++k)
			sum += LU[i][k] * x[k];
		x[i] = (y[i] - sum);  // not dividing by diagonals
   }
}

// void LUsolve(int n, double **A,double **LU, double*b, double *y, double*x);
void __attribute__((deprecated)) LUsolve3(double A[3][3], double x[3], double b[3]);
void __attribute__((deprecated)) LUsolve6(const double A[6][6], double x[6], const double b[6]);

/** @brief Compute 3x3 matrices determinant
 * @param m The matrix
 */
template <typename T>
inline double DetM3(const T **m)
{
	return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
	       m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
	       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/** @brief Invert a 3x3 matrix
 * @param m The matrix to invert
 * @return The matrix determinant
 * @warning This function is overwriting the matrix
 * @note This function computes the invert without checking for non-null
 * determinants
 * @note This function is way faster than any LU decomposition for 3x3 matrices
 */
template <typename T>
inline double InvM3(T **m)
{
	T minv[3][3];
	const double det = DetM3((const T**)m);
	const double idet = 1.0 / det;

	minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * idet;
	minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * idet;
	minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * idet;
	minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * idet;
	minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * idet;
	minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * idet;
	minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * idet;
	minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * idet;
	minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * idet;

	memcpy(m[0], minv[0], 3 * sizeof(T));
	memcpy(m[1], minv[1], 3 * sizeof(T));
	memcpy(m[2], minv[2], 3 * sizeof(T));
	return det;
}

/** @brief Solve a 3x3 linear system
 * @param m The matrix to invert
 * @return The matrix determinant
 * @warning This function is overwriting the matrix
 * @note This function does not check if the matrix is actually invertible
 * @note This function is way faster than any LU decomposition for 3x3 matrices
 */
template <typename T>
inline double __attribute__((deprecated)) Solve3(T **m, T *x, const T *b)
{
	const double det = InvM3(m);
	x[0] = dotProd3((const T*)m[0], b);
	x[1] = dotProd3((const T*)m[1], b);
	x[2] = dotProd3((const T*)m[2], b);
	return det;
}

/** @brief Duplicate a matrix
 * @param src The original matrix
 * @param dst The duplicate
 */
template <typename T>
inline void CopyMat(int n, const T **src, T **dst)
{
	for(int i=0; i<n; ++i)
	{
		memcpy(dst[i], src[i], n * sizeof(T));
	}
}

void RotMat( double x1, double x2, double x3, double TransMat[]);

void QuaternionToDCM(double q[4], double outMat[3][3]);

void rotateM3(double Min[3][3], double rotMat[3][3], double outMat[3][3]);
void rotateM3(double Min[9], double rotMat[9], double outMat[9]);

void rotateM6(double Min[6][6], double rotMat[3][3], double outMat[6][6]);
void rotateM6(double Min[36], double rotMat[9], double outMat[36]);

void rotateVector3(double inVec[3], double rotMat[9], double outVec[3]);
void rotateVector6(double inVec[6], double rotMat[9], double outVec[6]);

void transformKinematics(const double rRelBody[3], const double r_in[3], const double TransMat[9], const double rd_in[6], double rOut[3], double rdOut[3]);
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], vector< double > &rOut, vector< double > &rdOut);
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], double rOut[3], double rdOut[3]);

void translateForce6DOF(double dx[3], double F[6], double Fout[6]);

void translateForce3to6DOF(double dx[3], double F[3], double Fout[6]);

void transformMass3to6DOF(double r[3], double TransMat[9], double Min[3][3], double Iin[3][3], double Mout[6][6]);

void transformMass3to6DOF(double r[3], double TransMat[9], double Min[3][3], double Iin[3][3], double Mout[6][6]);

void translateMassInertia3to6DOF(double r[3], double Min[3][3], double Iin[3][3], double Mout[6][6]);

//void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6]);
void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6]);

void translateMass3to6DOF(double r[3], mat &Min, double Mout[6][6]);

void translateMass6to6DOF(double r[3], double Min[6][6], double Mout[6][6]);
void translateMass6to6DOF(double r[3], double Min[36], double Mout[36]);

vector<string> split(const string &s);
vector<string> splitBar(const string &s);
vector<string> splitComma(const string &s);

void reverse(double* data, int datasize);
void doIIR(double* in, double* out, int dataSize, double* a, double* b, int kernelSize);
void doSSfilter(double* in, double* out, int dataSize, double* a, double* beta, double b0, int kernelSize);

void free2Dmem(void** theArray, int n1);

double*    make1Darray(int n1);
double**   make2Darray(int n1, int n2);
double***  make3Darray(int n1, int n2, int n3);
double**** make4Darray(int n1, int n2, int n3, int n4);
void free2Darray(double**   theArray, int n1);
void free3Darray(double***  theArray, int n1, int n2);
void free4Darray(double**** theArray, int n1, int n2, int n3);


#endif
