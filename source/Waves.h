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

#ifndef WAVES_H
#define WAVES_H

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup new_c_api
 *  @{
 */

/// A mooring connection instance
typedef struct __MoorDynWaves* MoorDynWaves;

/** @brief Compute the wave number
 * @param Omega The wave angular frequency
 * @param g The gravity acceleration
 * @param h The water depth
 * @return The wave number
 * @note credit: FAST source
 */
double WaveNumber( double Omega, double g, double h );

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
