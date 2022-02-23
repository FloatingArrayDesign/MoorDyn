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

#ifndef __MOORDYN2_H__
#define __MOORDYN2_H__

#include "MoorDynAPI.h"

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
typedef void* MoorDyn;


/** @brief Creates a MoorDyn instance
 * 
 * At the time of creating a new MoorDyn instance, the input file is read and
 * all the objects and structures are created.
 * You must call afterwards MoorDyn_Init() to compute the initial conditions
 * 
 * @param infilename The input file, if either NULL or "", then
 * "Mooring/lines.txt" will be considered
 * @return The mooring instance, NULL if errors happened
 */
MoorDyn DECLDIR MoorDyn_Create(const char *infilename);

/** @brief Get the number of coupled Degrees Of Freedom (DOFs)
 *
 * The number of components for some parameters in MoorDyn_Init() and
 * MoorDyn_Step() can be known using this function
 * @return The number of coupled DOF, 0 if errors are detected
 */
unsigned int DECLDIR MoorDyn_NCoupledDOF(MoorDyn system);

/** @brief Set the instance verbosity level
 *
 * @param system The Moordyn system
 * @param verbosity The verbosity level. It can take the following values
 *  - MOORDYN_DBG_LEVEL Every single message will be printed
 *  - MOORDYN_MSG_LEVEL Messages specially designed to help debugging the code
 *    will be omitted
 *  - MOORDYN_WRN_LEVEL Just errors and warnings will be reported
 *  - MOORDYN_WRN_LEVEL Just errors will be reported
 * @return 0 If the verbosity level is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_SetVerbosity(MoorDyn system, int verbosity);

/** @brief Compute the initial condition of a MoorDyn system
 * 
 * At the time of creating a new MoorDyn instance, the input file is read and
 * all the objects and structures are created.
 * You must call afterwards MoorDyn_Init() to compute the initial conditions
 * 
 * @param system The Moordyn system
 * @param x Position vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @param xd Velocity vector (6 components per coupled body or cantilevered rod
 * and 3 components per pinned rod or coupled connection)
 * @return 0 If the mooring system is correctly initialized, an error code
 * otherwise (see @ref moordyn_errors)
 * @note MoorDyn_NCoupledDOF() can be used to know the number of components
 * required for \p x and \p xd
 */
int DECLDIR MoorDyn_Init(MoorDyn system, const double *x, const double *xd);

/** @brief Runs a time step of the MoorDyn system
 * @param system The Moordyn system
 * @param x Position vector
 * @param xd Velocity vector
 * @param f Output forces
 * @return 0 if the mooring system has correctly evolved, an error code
 * otherwise (see @ref moordyn_errors)
 * @note MoorDyn_NCoupledDOF() can be used to know the number of components
 * required for \p x, \p xd and \p f
 */
int DECLDIR MoorDyn_Step(MoorDyn system, const double *x, const double *xd,
                         double *f, double *t, double *dt);

/** @brief Releases MoorDyn allocated resources
 * @param system The Moordyn system
 * @return 0 If the mooring system is correctly destroyed, an error code
 * otherwise (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_Close(MoorDyn system);

/** @brief Initializes the external Wave kinetics
 *
 * This is useless unless WaveKin option is set in the input file. If that is
 * the case, remember calling this function after MoorDyn_Init()
 * @param system The Moordyn system
 * @param n The number of points where the wave kinematics shall be provided
 * @return 0 If the mooring system is correctly destroyed, an error code
 * otherwise (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_InitExtWaves(MoorDyn system, unsigned int *n);

/** @brief Get the points where the waves kinematics shall be provided
 * @param system The Moordyn system
 * @param r The output coordinates (3 components per point)
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 * @see MoorDyn_InitExtWaves()
 */
int DECLDIR MoorDyn_GetWavesCoords(MoorDyn system, double *r);

/** @brief Set the kinematics of the waves
 *
 * Use this function if WaveKin option is set in the input file
 * @param system The Moordyn system
 * @param U The velocities at the points (3 components per point)
 * @param Ud The accelerations at the points (3 components per point)
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 * @see MoorDyn_InitExtWaves()
 * @see MoorDyn_GetWavesCoords()
 */
int DECLDIR MoorDyn_SetWaves(MoorDyn system, const double *U,
                                             const double *Ud,
                                             double t);

/** @brief Get the number of bodies
 *
 * Remember that the first body index is 1
 * @param system The Moordyn system
 * @return The number of bodies, which may be 0 if errors happened
 * (see @ref moordyn_errors)
 */
unsigned int DECLDIR MoorDyn_GetNumberBodies(MoorDyn system);

/** @brief Get the number of rods
 *
 * Remember that the first rod index is 1
 * @param system The Moordyn system
 * @return The number of rods, which may be 0 if errors happened
 * (see @ref moordyn_errors)
 */
unsigned int DECLDIR MoorDyn_GetNumberRods(MoorDyn system);

/** @brief Get the number of connections
 *
 * Remember that the first connection index is 1
 * @param system The Moordyn system
 * @return The number of connections, which may be 0 if errors happened
 * (see @ref moordyn_errors)
 */
unsigned int DECLDIR MoorDyn_GetNumberConnections(MoorDyn system);

/** @brief Get the number of lines
 *
 * Remember that the first line index is 1
 * @param system The Moordyn system
 * @return The number of lines, which may be 0 if errors happened
 * (see @ref moordyn_errors)
 */
unsigned int DECLDIR MoorDyn_GetNumberLines(MoorDyn system);

/** @brief Get the number of line nodes
 * @param system The Moordyn system
 * @param line The line
 * @return The number of nodes, which may be 0 if errors happened
 * (see @ref moordyn_errors)
 * @note The number of nodes is equal to the specified number in the input file
 * plus one extra node. This function returns such extra node.
 */
unsigned int DECLDIR MoorDyn_GetNumberLineNodes(MoorDyn system,
                                                unsigned int line);

/** @brief Get the fairlead tension of a specific line
 * @param system The Moordyn system
 * @param line The line
 * @return The tension, -1 if no such line can be found
 */
double DECLDIR MoorDyn_GetFairTen(MoorDyn system, unsigned int line);

/** @brief Function for providing FASTv7 customary line tension quantities
 * @param system The Moordyn system
 * @param numLines The number of lines
 * @param FairHTen Allocated memory for the \p numLines horizontal forces at the
 * fairlead
 * @param FairVTen Allocated memory for the \p numLines vertical forces at the
 * fairlead
 * @param AnchHTen Allocated memory for the \p numLines horizontal forces at the
 * anchor
 * @param AnchVTen Allocated memory for the \p numLines vertical forces at the
 * anchor
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_GetFASTtens(MoorDyn system, const int* numLines,
                                float FairHTen[], float FairVTen[],
                                float AnchHTen[], float AnchVTen[]);

/** @brief Get the position of a connection
 * @param system The Moordyn system
 * @param l The connection
 * @param pos The output position
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_GetConnectPos(MoorDyn system,
                                  unsigned int l,
                                  double pos[3]);

/** @brief Get the force at a connection
 * @param system The Moordyn system
 * @param l The connection
 * @param f The output force
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_GetConnectForce(MoorDyn system,
                                    unsigned int l,
                                    double f[3]);

/** @brief Get a line node position
 * @param system The Moordyn system
 * @param LineNum The connection
 * @param NodeNum The connection
 * @param pos The output position
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 * @warning The first node index is 0, while the last node index is
 * the result of MoorDyn_GetNumberLineNodes() minus 1.
 */
int DECLDIR MoorDyn_GetNodePos(MoorDyn system,
                               unsigned int LineNum,
                               unsigned int NodeNum,
                               double pos[3]);

/** @brief Draw the lines and connections in the active OpenGL context
 *
 * The OpenGL context is assumed to be created by the caller before calling
 * this function
 * @param system The Moordyn system
 * @return 0 If the data is correctly set, an error code otherwise
 * (see @ref moordyn_errors)
 */
int DECLDIR MoorDyn_DrawWithGL(MoorDyn system);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
