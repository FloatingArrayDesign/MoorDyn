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

/** @file Moordyn2.h
 * C API for the moordyn::MoorDyn object, which is the main simulation handler
 */

#ifndef __MOORDYN2_H__
#define __MOORDYN2_H__

#include "MoorDynAPI.h"
#include "Waves.h"
#include "Seafloor.h"
#include "Point.h"
#include "Rod.h"
#include "Line.h"
#include "Body.h"
#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef WIN32
#include <windows.h>
#endif

	/** @defgroup new_c_api The C API
	 *  @{
	 */

	/// A mooring system instance
	typedef struct __MoorDyn* MoorDyn;

	/** @brief Creates a MoorDyn instance
	 *
	 * At the time of creating a new MoorDyn instance, the input file is read
	 * and all the objects and structures are created. You must call afterwards
	 * MoorDyn_Init() to compute the initial conditions
	 *
	 * @param infilename The input file, if either NULL or "", then
	 * "Mooring/lines.txt" will be considered
	 * @return The mooring instance, NULL if errors happened
	 */
	MoorDyn DECLDIR MoorDyn_Create(const char* infilename);

	/** @brief Get the number of coupled Degrees Of Freedom (DOFs)
	 *
	 * The number of components for some parameters in MoorDyn_Init() and
	 * MoorDyn_Step() can be known using this function
	 * @return MOORDYN_INVALID_VALUE if @p system is NULL, MOORDYN_SUCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_NCoupledDOF(MoorDyn system, unsigned int* n);

	/** @brief Set the instance verbosity level
	 * @param system The Moordyn system
	 * @param verbosity The verbosity level. It can take the following values
	 *  - MOORDYN_DBG_LEVEL Every single message will be printed
	 *  - MOORDYN_MSG_LEVEL Messages specially designed to help debugging the
	 * code will be omitted
	 *  - MOORDYN_WRN_LEVEL Just errors and warnings will be reported
	 *  - MOORDYN_ERR_LEVEL Just errors will be reported
	 *  - MOORDYN_NO_OUTPUT No info will be reported
	 * @return MOORDYN_SUCESS If the verbosity level is correctly set, an error
	 * code otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetVerbosity(MoorDyn system, int verbosity);

	/** @brief Set the instance log file
	 * @param system The Moordyn system
	 * @param log_path The file path to print the log file
	 * @return MOORDYN_SUCESS If the log file is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetLogFile(MoorDyn system, const char* log_path);

	/** @brief Set the instance log file printing level
	 * @param system The Moordyn system
	 * @param verbosity The log file print level. It can take the following
	 * values
	 *  - MOORDYN_DBG_LEVEL Every single message will be printed
	 *  - MOORDYN_MSG_LEVEL Messages specially designed to help debugging the
	 * code will be omitted
	 *  - MOORDYN_WRN_LEVEL Just errors and warnings will be reported
	 *  - MOORDYN_ERR_LEVEL Just errors will be reported
	 *  - MOORDYN_NO_OUTPUT No info will be reported
	 * @return MOORDYN_SUCESS If the printing level is correctly set, an error
	 * code otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetLogLevel(MoorDyn system, int verbosity);

	/** @brief Log a message
	 * @param system The Moordyn system
	 * @param level The message level. It can take the following values
	 *  - MOORDYN_DBG_LEVEL for debugging messages
	 *  - MOORDYN_MSG_LEVEL for regular information messages
	 *  - MOORDYN_WRN_LEVEL for warnings
	 *  - MOORDYN_ERR_LEVEL for errors
	 * @param msg The message to log
	 * @return MOORDYN_SUCESS If the printing level is correctly set, an error
	 * code otherwise (see @ref moordyn_errors)
	 * @note This messages are subjected to the same rules than the inner
	 * messages, i.e. if @p level is lower than the threshold levels set with
	 * MoorDyn_SetVerbosity() and MoorDyn_SetLogLevel(), the message will not be
	 * logged in the terminal and the log file respectively
	 * @note This function will not log the file, line and function where it is
	 * called from, not even in case of warnings or errors
	 */
	int DECLDIR MoorDyn_Log(MoorDyn system, int level, const char* msg);

	/** @brief Compute the initial condition of a MoorDyn system
	 *
	 * At the time of creating a new MoorDyn instance, the input file is read
	 * and all the objects and structures are created. You must call afterwards
	 * MoorDyn_Init() to compute the initial conditions
	 *
	 * @param system The Moordyn system
	 * @param x Position vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param xd Velocity vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @return MOORDYN_SUCESS If the mooring system is correctly initialized,
	 * an error code otherwise (see @ref moordyn_errors)
	 * @note MoorDyn_NCoupledDOF() can be used to know the number of components
	 * required for \p x and \p xd
	 */
	int DECLDIR MoorDyn_Init(MoorDyn system, const double* x, const double* xd);

	/** @brief The same than MoorDyn_Init(), but the initial condition is not
	 * computed at all.
	 *
	 * This is of use when you are loading a state file afterwards with
	 * Moordyn_Load()
	 *
	 * @param system The Moordyn system
	 * @param x Position vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @param xd Velocity vector (6 components per coupled body or cantilevered
	 * rod and 3 components per pinned rod or coupled point)
	 * @return MOORDYN_SUCESS If the mooring system is correctly initialized,
	 * an error code otherwise (see @ref moordyn_errors)
	 * @note MoorDyn_NCoupledDOF() can be used to know the number of components
	 * required for \p x and \p xd
	 */
	int DECLDIR MoorDyn_Init_NoIC(MoorDyn system,
	                              const double* x,
	                              const double* xd);

	/** @brief Runs a time step of the MoorDyn system
	 * @param system The Moordyn system
	 * @param x Position vector
	 * @param xd Velocity vector
	 * @param f Output forces
	 * @param t Simulation time
	 * @param dt Time step
	 * @return MOORDYN_SUCESS if the mooring system has correctly evolved, an
	 * error code otherwise (see @ref moordyn_errors)
	 * @note MoorDyn_NCoupledDOF() can be used to know the number of components
	 * required for \p x, \p xd and \p f
	 */
	int DECLDIR MoorDyn_Step(MoorDyn system,
	                         const double* x,
	                         const double* xd,
	                         double* f,
	                         double* t,
	                         double* dt);

	/** @brief Releases MoorDyn allocated resources
	 * @param system The Moordyn system
	 * @return MOORDYN_SUCESS If the mooring system is correctly destroyed, an
	 * error code otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_Close(MoorDyn system);

	/** @brief Get the wave kinematics instance
	 *
	 * The wave kinematics instance is only useful if WaveKin option is set to 2
	 * in the input file.
	 * @param system The Moordyn system
	 * @return The waves instance, NULL if errors happened
	 */
	MoorDynWaves DECLDIR MoorDyn_GetWaves(MoorDyn system);

	/** @brief Get the 3D seafloor instance
	 *
	 * The seafloor instance is only not null if a SeafloorPath was given as an
	 * option.
	 * @param system The Moordyn system
	 * @return The Seafloor instance, NULL if errors happened or there is no 3D
	 * seafloor
	 */
	MoorDynSeafloor DECLDIR MoorDyn_GetSeafloor(MoorDyn system);

	/**
	 * @name External Wave Kinematics
	 * The functions for setting external wave kinematics.
	 */
	/// @{

	/** @brief Initializes the external Wave kinematics
	 *
	 * This is useless unless the WaveKin option is set to 1 in the input file.
	 * If that is the case, remember to call this function after MoorDyn_Init()
	 * @param system The Moordyn system
	 * @param n The number of points where the wave kinematics shall be provided
	 * @return MOORDYN_SUCESS If the external waves are correctly initialized,
	 * an error code otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_ExternalWaveKinInit(MoorDyn system, unsigned int* n);

	/** @brief Get the number of points where the waves kinematics shall be
	 * provided
	 *
	 * This is useless unless WaveKin option is set to 1 in the input file
	 * @param system The Moordyn system
	 * @param n The output number of points where the wave kinematics shall
	 * be provided
	 * @return MOORDYN_SUCESS
	 * @see MoorDyn_ExternalWaveKinInit()
	 */
	int DECLDIR MoorDyn_ExternalWaveKinGetN(MoorDyn system, unsigned int* n);

	/** @brief Get the points where the waves kinematics shall be provided
	 *
	 * The kinematics on those points shall be provided just if WaveKin is set
	 * to 1 in the input file. The pointer r should be to an array with enough
	 * space for 3 * N doubles, where N is the value from
	 * MoorDyn_ExternalWaveKinGetN or MoorDyn_ExternalWaveKinInit.
	 * @param system The Moordyn system
	 * @param r The output coordinates (3 components per point)
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDyn_ExternalWaveKinInit()
	 */
	int DECLDIR MoorDyn_ExternalWaveKinGetCoordinates(MoorDyn system,
	                                                  double* r);

	/** @brief Get the points where the waves kinematics shall be provided
	 *
	 * The kinematics on those points shall be provided just if WaveKin is set
	 * to 1 in the input file
	 * @param system The Moordyn system
	 * @param r The output coordinates (3 components per point)
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @deprecated This function has been renamed as
	 * MoorDyn_ExternalWaveKinGetCoordinates()
	 * @see MoorDyn_ExternalWaveKinInit()
	 */
	inline int DECLDIR DEPRECATED MoorDyn_GetWaveKinCoordinates(MoorDyn system,
	                                                            double* r)
	{
		return MoorDyn_ExternalWaveKinGetCoordinates(system, r);
	}

	/** @brief Set the kinematics of the waves
	 *
	 * Use this function if the WaveKin option is set to 1 in the input file
	 * @param system The Moordyn system
	 * @param U The velocities at the points (3 components per point)
	 * @param Ud The accelerations at the points (3 components per point)
	 * @param t Simulation time
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDyn_ExternalWaveKinInit()
	 * @see MoorDyn_ExternalWaveKinGetCoordinates()
	 */
	int DECLDIR MoorDyn_ExternalWaveKinSet(MoorDyn system,
	                                       const double* U,
	                                       const double* Ud,
	                                       double t);

	/** @brief Set the kinematics of the waves
	 *
	 * Use this function if WaveKin option is set to 1 in the input file
	 * @param system The Moordyn system
	 * @param U The velocities at the points (3 components per point)
	 * @param Ud The accelerations at the points (3 components per point)
	 * @param t Simulation time
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @deprecated This function has been renamed as
	 * MoorDyn_ExternalWaveKinSet()
	 * @see MoorDyn_ExternalWaveKinInit()
	 * @see MoorDyn_ExternalWaveKinGetCoordinates()
	 */
	inline int DECLDIR DEPRECATED MoorDyn_SetWaveKin(MoorDyn system,
	                                                 const double* U,
	                                                 const double* Ud,
	                                                 double t)
	{
		return MoorDyn_ExternalWaveKinSet(system, U, Ud, t);
	}

	/// @}

	/** @brief Get the number of bodies
	 *
	 * Remember that the first body index is 1
	 * @param system The Moordyn system
	 * @param n The output number of bodies
	 * @return MOORDYN_SUCESS If the number is successfully got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetNumberBodies(MoorDyn system, unsigned int* n);

	/** @brief Get a rigid body
	 *
	 * Remember that the first body index is 1
	 * @param system The Moordyn system
	 * @param b The body index
	 * @return The body instance, NULL if errors happened
	 */
	MoorDynBody DECLDIR MoorDyn_GetBody(MoorDyn system, unsigned int b);

	/** @brief Get the number of rods
	 *
	 * Remember that the first rod index is 1
	 * @param system The Moordyn system
	 * @param n The output number of rods
	 * @return MOORDYN_SUCESS If the number is successfully got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetNumberRods(MoorDyn system, unsigned int* n);

	/** @brief Get a rod
	 * @param system The Moordyn system
	 * @param r The rod
	 * @return The rod instance, NULL if errors happened
	 */
	MoorDynRod DECLDIR MoorDyn_GetRod(MoorDyn system, unsigned int r);

	/** @brief Get the number of points
	 *
	 * Remember that the first point index is 1
	 * @param system The Moordyn system
	 * @param n The output number of points
	 * @return MOORDYN_SUCESS If the number is successfully got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetNumberPoints(MoorDyn system, unsigned int* n);

	/** @brief Get a point
	 * @param system The Moordyn system
	 * @param c The point
	 * @return The point instance, NULL if errors happened
	 */
	MoorDynPoint DECLDIR MoorDyn_GetPoint(MoorDyn system, unsigned int c);

	/** @brief Get the number of lines
	 *
	 * Remember that the first line index is 1
	 * @param system The Moordyn system
	 * @param n The output number of lines
	 * @return MOORDYN_SUCESS If the number is successfully got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetNumberLines(MoorDyn system, unsigned int* n);

	/** @brief Get a line instance
	 * @param system The Moordyn system
	 * @param l The line identifier (from 1 to the number of lines)
	 * @return The line instance, NULL if errors happened
	 */
	MoorDynLine DECLDIR MoorDyn_GetLine(MoorDyn system, unsigned int l);

	/** @brief Function for providing FASTv7 customary line tension quantities
	 * @param system The Moordyn system
	 * @param numLines The number of lines
	 * @param FairHTen Allocated memory for the \p numLines horizontal forces at
	 * the fairlead
	 * @param FairVTen Allocated memory for the \p numLines vertical forces at
	 * the fairlead
	 * @param AnchHTen Allocated memory for the \p numLines horizontal forces at
	 * the anchor
	 * @param AnchVTen Allocated memory for the \p numLines vertical forces at
	 * the anchor
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetFASTtens(MoorDyn system,
	                                const int* numLines,
	                                float FairHTen[],
	                                float FairVTen[],
	                                float AnchHTen[],
	                                float AnchVTen[]);

	/** @brief Get the current model time step
	 * @param system The Moordyn system
	 * @param dt The output time step
	 * @return MOORDYN_SUCESS if the data is correctly got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetDt(MoorDyn system, double* dt);

	/** @brief Set the model time step
	 * @param system The Moordyn system
	 * @param dt The new time step
	 * @return MOORDYN_SUCESS if the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetDt(MoorDyn system, double dt);

	/** @brief Get the current model Courant–Friedrichs–Lewy factor
	 * @param system The Moordyn system
	 * @param cfl The output Courant–Friedrichs–Lewy factor
	 * @return MOORDYN_SUCESS if the data is correctly got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetCFL(MoorDyn system, double* cfl);

	/** @brief Set the model Courant–Friedrichs–Lewy factor
	 * @param system The Moordyn system
	 * @param cfl The new Courant–Friedrichs–Lewy factor
	 * @return MOORDYN_SUCESS if the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetCFL(MoorDyn system, double cfl);

	/** @brief Get the current time scheme name
	 * @param system The Moordyn system
	 * @param name The output name. Can be NULL.
	 * @param name_len The output number of bytes written. Can be NULL.
	 * @return MOORDYN_SUCESS if the data is correctly got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetTimeScheme(MoorDyn system,
	                                  char* name,
	                                  size_t* name_len);

	/** @brief Set the time scheme by its name
	 * @param system The Moordyn system
	 * @param name The new time scheme name.
	 * @return MOORDYN_SUCESS if the data is correctly got, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_SetTimeScheme(MoorDyn system, const char* name);

	/** @brief Serialize the system to bytes
	 *
	 * Typically you want to call this function twice. A first call to know the
	 * amount of memory to be allocated for the bytes array and a second one
	 * to actually get the bytes array
	 *
	 * The returned bytes can be used afterwards to restore the model calling
	 * to MoorDyn_Deserialize()
	 * @param system The Moordyn system
	 * @param size Output size of the bytes array. It can be null
	 * @param data Allocated memory for the output bytes array. It can be null
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_Serialize(MoorDyn system, size_t* size, uint64_t* data);

	/** @brief Load the system from the serialized data before
	 *
	 * You can restore the system to a previous state retrieved calling
	 * MoorDyn_Serialize()
	 * @param system The Moordyn system
	 * @param data Bytes array
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDyn_Save
	 * @see MoorDyn_Init_NoIC
	 */
	int DECLDIR MoorDyn_Deserialize(MoorDyn system, const uint64_t* data);

	/** @brief Save the system so it can be loaded afterwards
	 *
	 * At the time of loading the system, it is still required to create the
	 * system reading the same definition file and calling MoorDyn_Init_NoIC()
	 * @param system The Moordyn system
	 * @param filepath The path of the file to write
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDyn_Load
	 * @see MoorDyn_Init_NoIC
	 */
	int DECLDIR MoorDyn_Save(MoorDyn system, const char* filepath);

	/** @brief Load the system saved before
	 *
	 * You must still call MoorDyn_Create() and MoorDyn_Init_NoIC() before
	 * calling this function
	 * @param system The Moordyn system
	 * @param filepath The path of the MoorDyn saved system
	 * @return MOORDYN_SUCESS If the data is correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @see MoorDyn_Save
	 * @see MoorDyn_Init_NoIC
	 */
	int DECLDIR MoorDyn_Load(MoorDyn system, const char* filepath);

	/** @brief Save the whole system to a VTK (.vtm) file
	 *
	 * In general it is more convenient to handle each object independently,
	 * using MoorDyn_SaveLineVTK() and MoorDyn_SaveRodVTK() functions. However,
	 * if the number of subinstances is large, that would not be an option
	 * anymore. In that case you can use this function to pack everything
	 * together in a single file
	 * @param system The Moordyn system
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SaveVTK(MoorDyn system, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
