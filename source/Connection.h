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

/** @file Connection.h
 * C API for the moordyn::Connection object
 */

#ifndef CONNECTION_H
#define CONNECTION_H

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring connection instance
	typedef struct __MoorDynConnection* MoorDynConnection;

	/** @brief Get the connection identifier
	 * @param conn The Moordyn connection
	 * @return The connection identifier, MOORDYN_INVALID_VALUE if a NULL
	 * connection is provided
	 */
	int DECLDIR MoorDyn_GetConnectID(MoorDynConnection conn);

	/** @brief Get the connection type
	 * @param conn The Moordyn connection
	 * @return The connection type, MOORDYN_INVALID_VALUE if a NULL connection
	 * is provided
	 * @see Connection::types
	 * @warning MOORDYN_INVALID_VALUE matchs the value of Connection::COUPLED,
	 * i.e. the returned value cannot be reliably used as error handling
	 */
	int DECLDIR MoorDyn_GetConnectType(MoorDynConnection conn);

	/** @brief Get the position of a connection
	 * @param conn The Moordyn connection
	 * @param pos The output position
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetConnectPos(MoorDynConnection conn, double pos[3]);

	/** @brief Get the velocity of a connection
	 * @param conn The Moordyn connection
	 * @param v The output velocity
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetConnectVel(MoorDynConnection conn, double v[3]);

	/** @brief Get the force at a connection
	 * @param conn The Moordyn connection
	 * @param f The output force
	 * @return 0 If the data is correctly set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	int DECLDIR MoorDyn_GetConnectForce(MoorDynConnection conn, double f[3]);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
