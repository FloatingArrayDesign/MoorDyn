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

/** @file Waves.h
 * C API for the moordyn::Waves object
 */

#ifndef WAVES_H
#define WAVES_H

#include "MoorDynAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A seafloor descriptor
	typedef struct __MoorDynSeafloor* MoorDynSeafloor;

	/// A mooring point instance
	typedef struct __MoorDynWaves* MoorDynWaves;

	/** @brief Get the velocity, acceleration, wave height and dynamic pressure
	 * at a specific position and time
	 * @param waves The Waves instance
	 * @param x The point x coordinate
	 * @param y The point y coordinate
	 * @param z The point z coordinate
	 * @param U The output velocity
	 * @param Ud The output acceleration
	 * @param zeta The output wave height
	 * @param PDyn The output dynamic pressure
	 * @param seafloor The seafloor instance, see MoorDyn_GetSeafloor()
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetWavesKin(MoorDynWaves waves,
	                                double x,
	                                double y,
	                                double z,
	                                double U[3],
	                                double Ud[3],
	                                double* zeta,
	                                double* PDyn,
	                                MoorDynSeafloor seafloor);

	/** @brief Compute the wave number
	 * @param Omega The wave angular frequency
	 * @param g The gravity acceleration
	 * @param h The water depth
	 * @return The wave number
	 * @note credit: FAST source
	 */
	double DECLDIR WaveNumber(double Omega, double g, double h);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
