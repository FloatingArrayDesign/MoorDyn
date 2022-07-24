/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
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

/** @file State.hpp
 * C++ API for the state variables, used in moordyn::TimeScheme
 */

#pragma once

#include "Misc.hpp"
#include <vector>

namespace moordyn {

/** @class State Time.hpp
 * @brief Generic state variables
 *
 * This is holding the position and velocitites
 */
template<typename T>
class StateVar
{
  public:
	/// @brief Costructor
	StateVar() {}

	/// @brief Destructor
	~StateVar() {}

	/// The position
	T pos;
	/// The velocity
	T vel;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	StateVar<T>& operator=(const StateVar<T>& visitor)
	{
		pos = visitor.pos;
		vel = visitor.vel;
		return *this;
	}

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVar<T> operator+(const StateVar<T>& visitor);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVar<T> operator-(const StateVar<T>& visitor);
};

/** @class StateDeriv Time.hpp
 * @brief Generic state variables derivative
 *
 * This is holding the velocitites and accelerations
 */
template<class T>
class StateVarDeriv
{
  public:
	/// @brief Costructor
	StateVarDeriv() {}

	/// @brief Destructor
	~StateVarDeriv() {}

	/// The velocity
	T vel;
	/// The acceleration
	T acc;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	StateVarDeriv<T>& operator=(const StateVarDeriv<T>& visitor)
	{
		vel = visitor.vel;
		acc = visitor.ac;
		return *this;
	}

	/** @brief Integrate in time
	 * @param dt The time step
	 * @return The state variables increment
	 */
	StateVar<T> operator*(const real& dt);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVarDeriv<T> operator+(const StateVarDeriv<T>& visitor);

	/** @brief Subtract operator
	 * @param visitor The entity to subtract
	 */
	StateVarDeriv<T> operator-(const StateVarDeriv<T>& visitor);
};

/// The state variables for lines
typedef StateVar<std::vector<vec>> LineState;

/// The state variables derivative for lines
typedef StateVarDeriv<std::vector<vec>> DLineStateDt;

/// The state variables for connections
typedef StateVar<vec> ConnState;

/// The state variables derivative for connections
typedef StateVarDeriv<vec> DConnStateDt;

/// The state variables for rods
typedef StateVar<vec6> RodState;

/// The state variables derivative for rods
typedef StateVarDeriv<vec6> DRodStateDt;

/// The state variables for bodies
typedef StateVar<vec6> BodyState;

/// The state variables derivative for bodies
typedef StateVarDeriv<vec6> DBodyStateDt;

/** @class MoorDynState Time.hpp
 * @brief The collection of state variables of the whole system
 */
class MoorDynState
{
  public:
	/// @brief Costructor
	MoorDynState() {}

	/// @brief Destructor
	~MoorDynState() {}

	/// The states of the lines
	std::vector<LineState> lines;

	/// The states of the connections
	std::vector<ConnState> conns;

	/// The states of the rods
	std::vector<RodState> rods;

	/// The states of the bodies
	std::vector<BodyState> bodies;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	MoorDynState& operator=(const MoorDynState& visitor);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	MoorDynState operator+(const MoorDynState& visitor);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	MoorDynState operator-(const MoorDynState& visitor);
};

/** @class DMoorDynStateDt Time.hpp
 * @brief The collection of state variable derivatives of the whole system
 */
class DMoorDynStateDt
{
  public:
	/// @brief Costructor
	DMoorDynStateDt() {}

	/// @brief Destructor
	~DMoorDynStateDt() {}

	/// The state derivatives of the lines
	std::vector<DLineStateDt> lines;

	/// The state derivatives of the connections
	std::vector<DConnStateDt> conns;

	/// The state derivatives of the rods
	std::vector<DRodStateDt> rods;

	/// The state derivatives of the bodies
	std::vector<DBodyStateDt> bodies;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	DMoorDynStateDt& operator=(const DMoorDynStateDt& visitor);

	/** @brief Integrate in time
	 * @param dt The time step
	 * @return The state variables increment
	 */
	MoorDynState operator*(const real& dt);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	DMoorDynStateDt operator+(const DMoorDynStateDt& visitor);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	DMoorDynStateDt operator-(const DMoorDynStateDt& visitor);
};

} // ::moordyn
