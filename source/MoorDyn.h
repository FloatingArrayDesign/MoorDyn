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

/** @file MoorDyn.h
 * Old API for compatibility with the previous versions of MoorDyn
 */

#ifndef __MOORDYN_H__
#define __MOORDYN_H__

#include "MoorDynAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @defgroup old_c_api The old API
 *
 * The old API is based on a singleton, i.e. just one MoorDyn instance can be
 * hold per process
 *  @{
 */

/** @brief initializes MoorDyn
 * 
 * Including reading the input file, creating the mooring system data
 * structures, and calculating the initial conditions
 * 
 * @param x Position vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @param xd Velocity vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @param infilename The input file, if either NULL or "", then
 * "Mooring/lines.txt" will be considered
 * @return 0 If the mooring system is correctly initialized, an error code
 * otherwise (see @ref moordyn_errors)
 * @warning Just one instance of MoorDyn per process is allowed. Thus, if
 * several mooring systems shall be handled, please spawn additional processes
 */
int DECLDIR MoorDynInit(const double x[], const double xd[], const char *infilename);

/** @brief Runs a time step of the MoorDyn system
 * @param system The Moordyn system
 * @param x Position vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @param xd Velocity vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @param f Output forces (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @return 0 if the mooring system has correctly evolved, an error code
 * otherwise (see @ref moordyn_errors)
 * @see MoorDynInit
 */
int DECLDIR MoorDynStep(const double x[], const double xd[], double f[], double* t, double* dt);

/** @brief Releases MoorDyn allocated resources
 * @return 0 If the mooring system is correctly destroyed, an error code
 * otherwise (see @ref moordyn_errors)
 * @note Call this function even if the initialization failed
 */
int DECLDIR MoorDynClose(void);

/** @brief Initializes the external Wave kinetics
 *
 * This is useless unless WaveKin option is set in the input file. If that is
 * the case, remember calling this function after MoorDyn_Init()
 * @return The number of points where the wave kinematics shall be provided. 0
 * if errors are detected
 */
int DECLDIR externalWaveKinInit();

/** @brief Get the points where the waves kinematics shall be provided
 * @param r The output coordinates (3 components per point)
 * @see externalWaveKinInit()
 */
void DECLDIR getWaveKinCoordinates(double r_out[]);

/** @brief Set the kinematics of the waves
 *
 * Use this function if WaveKin option is set in the input file
 * @param U The velocities at the points (3 components per point)
 * @param Ud The accelerations at the points (3 components per point)
 * @see externalWaveKinInit()
 * @see getWaveKinCoordinates()
 */
void DECLDIR setWaveKin(const double U_in[], const double Ud_in[], double t_in);

double DECLDIR GetFairTen(int);

int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] );

int DECLDIR GetConnectPos(int l, double pos[3]);
int DECLDIR GetConnectForce(int l, double force[3]);
int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3]);

int DECLDIR DrawWithGL(void);

int AllOutput(double, double);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
