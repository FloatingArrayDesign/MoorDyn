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

/** @file Body.h
 * C API for the moordyn::Body object
 */

#ifndef MOORDYN_BODY_H
#define MOORDYN_BODY_H

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring line instance
	typedef struct __MoorDynBody* MoorDynBody;

	/** @brief Get the body identifier
	 * @param b The Moordyn body
	 * @param id The output id
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetBodyID(MoorDynBody b, int* id);

	/** @brief Get the point type
	 * @param b The Moordyn body
	 * @param t The output type
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 * @see Point::types
	 */
	int DECLDIR MoorDyn_GetBodyType(MoorDynBody b, int* t);

	/** @brief Get the body state
	 * @param b The Moordyn body
	 * @param r The output position (6dof)
	 * @param rd The output velocity (6dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyState(MoorDynBody b, double r[6], double rd[6]);

	/** @brief Get the body position
	 * @param b The Moordyn body
	 * @param r The output position (3dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyPos(MoorDynBody b, double r[3]);

	/** @brief Get the body angle
	 * @param b The Moordyn body
	 * @param r The output angles (3dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyAngle(MoorDynBody b, double r[3]);

	/** @brief Get the body velocity
	 * @param b The Moordyn body
	 * @param rd The output velocity (3dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyVel(MoorDynBody b, double rd[3]);

	/** @brief Get the body angular velocity
	 * @param b The Moordyn body
	 * @param rd The output angular velocity (3dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyAngVel(MoorDynBody b, double rd[3]);

	/** @brief Get the body angular velocity
	 * @param b The Moordyn body
	 * @param rd The output angular velocity (3dof)
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyForce(MoorDynBody b, double f[6]);

	/** @brief Get the body mass and intertia matrix
	 * @param b The Moordyn body
	 * @param m The output mass matrix
	 * @return MOORDYN_INVALID_VALUE if a NULL body is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetBodyM(MoorDynBody b, double m[6][6]);

	/** @brief Save the point to a VTK (.vtp) file
	 * @param b The Moordyn body
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SaveBodyVTK(MoorDynBody b, const char* filename);

	/** @brief Load the model that would represent the body
	 *
	 * The model can have one of the following formats:
	 *
	 *  - VTK PolyData (.vtp)
	 *  - Stereo Lithography (.stl)
	 *
	 * @param b The Moordyn body
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_UseBodyVTK(MoorDynBody b, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
