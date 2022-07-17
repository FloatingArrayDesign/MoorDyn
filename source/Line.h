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
	 * @return The line identifier, MOORDYN_INVALID_VALUE if a NULL line is
	 * provided
	 */
	int DECLDIR MoorDyn_GetLineID(MoorDynLine l);

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

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
