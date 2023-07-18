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

/** @file Line.h
 * C API for the moordyn::Line object
 */

#ifndef MOORDYN_LINE_H
#define MOORDYN_LINE_H

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring line instance
	typedef struct __MoorDynLine* MoorDynLine;

	/** @brief Get the line identifier
	 * @param l The Moordyn line
	 * @param id The output id
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineID(MoorDynLine l, int* id);

	/** @brief Get the line number of segments
	 *
	 * The number of nodes is equal to this value plus 1
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineN(MoorDynLine l, unsigned int* n);

	/** @brief Get the line number of nodes
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineN()
	 */
	int DECLDIR MoorDyn_GetLineNumberNodes(MoorDynLine l, unsigned int* n);

	/** @brief Get the line unstretched length
	 * @param l The Moordyn line
	 * @param ul The output length
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_SetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_GetLineUnstretchedLength(MoorDynLine l, double* ul);

	/** @brief Set the line unstretched length
	 * @param l The Moordyn line
	 * @param ul The new length
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_SetLineUnstretchedLength(MoorDynLine l, double ul);

	/** @brief Set the line unstretched length rate of change
	 * @param l The Moordyn line
	 * @param v The rate of change
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineUnstretchedLength()
	 * @see MoorDyn_SetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_SetLineUnstretchedLengthVel(MoorDynLine l, double v);

	/** @brief Get a line node position
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param pos The output node position
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodePos(MoorDynLine l,
	                                   unsigned int i,
	                                   double pos[3]);

	/** @brief Get a line node tension
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param t The output node tension
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeTen(MoorDynLine l,
	                                   unsigned int i,
	                                   double t[3]);

	/** @brief Get a line curvature at a node
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param c The output line curvature
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 * @note The curvature is only computed if bending stiffness
	 * (moordyn::Line::EI) is not zero. Otherwise the curvature of every single
	 * node will be zero.
	 */
	int DECLDIR MoorDyn_GetLineNodeCurv(MoorDynLine l,
	                                    unsigned int i,
	                                    double* c);

	/** @brief Get the tension module at the end point B (the fairlead)
	 * @param l The Moordyn line
	 * @param t The output node tension module
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineFairTen(MoorDynLine l, double* t);

	/** @brief Get the maximum tension module
	 * @param l The Moordyn line
	 * @param t The output maximum tension module
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineMaxTen(MoorDynLine l, double* t);

	/** @brief Save the line to a VTK (.vtp) file
	 * @param l The Moordyn line
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SaveLineVTK(MoorDynLine l, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
