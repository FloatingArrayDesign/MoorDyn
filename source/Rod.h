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

/** @file Rod.h
 * C API for the moordyn::Rod object
 */

#ifndef ROD_H
#define ROD_H

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring line instance
	typedef struct __MoorDynRod* MoorDynRod;

	/** @brief Get the line identifier
	 * @param l The Moordyn rod
	 * @param id The output id
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodID(MoorDynRod l, int* id);

	/** @brief Get the line type
	 * @param l The Moordyn rod
	 * @param t The output type
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodType(MoorDynRod l, int* t);

	/** @brief Get the net force acting on the rod, as well as the moment at
	 * end point A if the node is not pinned
	 * @param l The Moordyn rod
	 * @param f The output force
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodForce(MoorDynRod l, double f[6]);

	/** @brief Get the total rod mass matrix
	 * @param l The Moordyn rod
	 * @param m The output mass matrix
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodM(MoorDynRod l, double m[6][6]);

	/** @brief Get the line number of segments
	 *
	 * The number of nodes is equal to this value plus 1
	 * @param l The Moordyn rod
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetRodN(MoorDynRod l, unsigned int* n);

	/** @brief Get the line number of nodes
	 * @param l The Moordyn rod
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetRodN()
	 */
	int DECLDIR MoorDyn_GetRodNumberNodes(MoorDynRod l, unsigned int* n);

	/** @brief Get a rod node position
	 * @param l The Moordyn rod
	 * @param i The node index
	 * @param pos The output node position
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodNodePos(MoorDynRod l,
	                                  unsigned int i,
	                                  double pos[3]);

	/** @brief Get a rod node velocity
	 * @param l The Moordyn rod
	 * @param i The node index
	 * @param vel The output node velocity
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodNodeVel(MoorDynRod l,
	                                  unsigned int i,
	                                  double vel[3]);

	/** @brief Save the line to a VTK (.vtp) file
	 * @param l The Moordyn rod
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SaveRodVTK(MoorDynRod l, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
