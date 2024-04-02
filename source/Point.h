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

/** @file Point.h
 * C API for the moordyn::Point object
 */

#ifndef POINT_H
#define POINT_H

#include "Line.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring point instance
	typedef struct __MoorDynPoint* MoorDynPoint;

	/** @brief Get the point identifier
	 * @param point The Moordyn point
	 * @param id The output id
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetPointID(MoorDynPoint point, int* id);

	/** @brief Get the point type
	 * @param point The Moordyn point
	 * @param t The output type
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 * @see Point::types
	 */
	int DECLDIR MoorDyn_GetPointType(MoorDynPoint point, int* t);

	/** @brief Get the position of a point
	 * @param point The Moordyn point
	 * @param pos The output position
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointPos(MoorDynPoint point, double pos[3]);

	/** @brief Get the velocity of a point
	 * @param point The Moordyn point
	 * @param v The output velocity
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointVel(MoorDynPoint point, double v[3]);

	/** @brief Get the force at a point
	 * @param point The Moordyn point
	 * @param f The output force
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointForce(MoorDynPoint point, double f[3]);

	/** @brief Get the point mass matrix
	 * @param point The Moordyn point
	 * @param f The output force
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointM(MoorDynPoint point, double m[3][3]);

	/** @brief Get the number of connected lines
	 * @param point The Moordyn point
	 * @param n The output number of connected lines
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointNAttached(MoorDynPoint point, unsigned int* n);

	/** @brief Get the number of connected lines
	 * @param point The Moordyn point
	 * @param i The index of the attached line
	 * @param l The output attached line
	 * @param e The output endpoint, see moordyn::EndPoints
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetPointAttached(MoorDynPoint point,
	                                     unsigned int i,
	                                     MoorDynLine* l,
	                                     int* e);

	/** @brief Save the point to a VTK (.vtp) file
	 * @param point The Moordyn point
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SavePointVTK(MoorDynPoint point, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
