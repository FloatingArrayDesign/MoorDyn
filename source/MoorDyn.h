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

/** @file MoorDyn.h
 * Old API for compatibility with the previous versions of MoorDyn
 *
 * Both the input files and some function names have been redesigned, so the
 * code will not be totally backward compatible.
 *
 * The usage of this API is anyway strongly discouraged, please consider
 * migrating to the v2 API in MoorDyn2.h
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
	 * The old API is based on a singleton, i.e. just one MoorDyn instance can
	 * be hold per process
	 *  @{
	 */

	/** @brief initializes MoorDyn
	 *
	 * Including reading the input file, creating the mooring system data
	 * structures, and calculating the initial conditions
	 *
	 * @param x Position vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param xd Velocity vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param infilename The input file, if either NULL or "", then
	 * "Mooring/lines.txt" will be considered
	 * @return 0 If the mooring system is correctly initialized, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @warning Just one instance of MoorDyn per process is allowed. Thus, if
	 * several mooring systems shall be handled, please spawn additional
	 * processes
	 */
	int DECLDIR MoorDynInit(const double x[],
	                        const double xd[],
	                        const char* infilename);

	/** @brief Runs a time step of the MoorDyn system
	 * @param x Position vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param xd Velocity vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param f Output forces (6 components per coupled body or cantilevered rod
	 * and 3 components per pinned rod or coupled point)
	 * @param t Simulation time
	 * @param dt Time step
	 * @return 0 if the mooring system has correctly evolved, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDynInit
	 */
	int DECLDIR MoorDynStep(const double x[],
	                        const double xd[],
	                        double f[],
	                        double* t,
	                        double* dt);

	/** @brief Releases MoorDyn allocated resources
	 * @return 0 If the mooring system is correctly destroyed, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @note Call this function even if the initialization failed
	 */
	int DECLDIR MoorDynClose(void);

	/** @brief Initializes the external Wave kinetics
	 *
	 * This is useless unless WaveKin option is set in the input file. If that
	 * is the case, remember calling this function after MoorDyn_Init()
	 * @return The number of points where the wave kinematics shall be provided.
	 * 0 if errors are detected
	 */
	int DECLDIR externalWaveKinInit();

	/** @brief Get the points where the waves kinematics shall be provided
	 * @param r_out The output coordinates (3 components per point)
	 * @see externalWaveKinInit()
	 */
	void DECLDIR getWaveKinCoordinates(double r_out[]);

	/** @brief Set the kinematics of the waves
	 *
	 * Use this function if WaveKin option is set in the input file
	 * @param U_in The velocities at the points (3 components per point)
	 * @param Ud_in The accelerations at the points (3 components per point)
	 * @param t_in Simulation time
	 * @see externalWaveKinInit()
	 * @see getWaveKinCoordinates()
	 */
	void DECLDIR setWaveKin(const double U_in[],
	                        const double Ud_in[],
	                        double t_in);

	double DECLDIR GetFairTen(int);

	int DECLDIR GetFASTtens(int* numLines,
	                        float FairHTen[],
	                        float FairVTen[],
	                        float AnchHTen[],
	                        float AnchVTen[]);

	int DECLDIR GetPointPos(int l, double pos[3]);
	int DECLDIR GetPointForce(int l, double force[3]);
	int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3]);

	int AllOutput(double, double);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
