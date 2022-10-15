/*
 * Copyright (c) 2022, Jose Luis Cercos-Pita <jlc@core-marine.com>
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

/** @file State.hpp
 * C++ API for the state variables, used in moordyn::TimeScheme
 */

#pragma once

#include "Misc.hpp"
#include <vector>
#include <string>
#include <sstream>

namespace moordyn {

/** @class StateVar Time.hpp
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

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

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

/** @class StateVarDeriv Time.hpp
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

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	StateVarDeriv<T>& operator=(const StateVarDeriv<T>& visitor)
	{
		vel = visitor.vel;
		acc = visitor.acc;
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

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

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

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

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
