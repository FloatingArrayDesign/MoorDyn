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

/** @file Log.h
 * Utilities to log info on screen
 */

#pragma once

#include "MoorDynAPI.h"
#include <iostream>

namespace moordyn
{

/** @class Log
 * @brief A Logging utility
 *
 * This class is a sensible replacement for std::cout and std::cerr ofstream
 * entities, providing the appropriate streaming channel to each entity
 */
class Log
{
public:
	/** @brief Constructor
	 * @param verbosity The verbosity level (see @ref moordyn_log)
	 */
	Log(const int verbosity=MOORDYN_DBG_LEVEL);

	/** @brief Destuctor
	 */    
	~Log() {}

	/** @brief Get a stream to log data
	 * 
	 * Whether the message is logged, and where, depends on the verbosity level
	 *
	 * @param level Message level
	 */
	std::ostream& Cout(const int level=MOORDYN_MSG_LEVEL) const;

	/** @brief Get the verbosity level
	 * @return The verbosity level (see @ref moordyn_log)
	 */
	inline int GetVerbosity() const { return _verbosity; }

	/** @brief Set the verbosity level
	 * @param verbosity The verbosity level (see @ref moordyn_log)
	 */
	inline void SetVerbosity(const int verbosity) { _verbosity = verbosity; }

private:
	/// Verbosity level
	int _verbosity;
};

}  // ::moordyn
