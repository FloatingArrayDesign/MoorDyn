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
	 * @return The body identifier, MOORDYN_INVALID_VALUE if a NULL body is
	 * provided
	 */
	int DECLDIR MoorDyn_GetBodyID(MoorDynBody b);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
