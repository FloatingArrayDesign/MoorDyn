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
	 * @param l The Moordyn line
	 * @return The line identifier, MOORDYN_INVALID_VALUE if a NULL line is
	 * provided
	 */
	int DECLDIR MoorDyn_GetRodID(MoorDynRod l);

	/** @brief Get the line number of segments
	 *
	 * The number of nodes is equal to this value plus 1
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetRodN(MoorDynRod l, unsigned int* n);

	/** @brief Get the line number of nodes
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetRodN()
	 */
	int DECLDIR MoorDyn_GetRodNumberNodes(MoorDynRod l, unsigned int* n);

	/** @brief Get a line node position
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param pos The output node position
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetRodNodePos(MoorDynRod l,
	                                  unsigned int i,
	                                  double pos[3]);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
