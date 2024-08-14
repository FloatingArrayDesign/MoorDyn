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
 * This is holding the position and velocities
 */
template<typename T, typename V = T>
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
	V vel;

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	StateVar<T, V>& operator=(const StateVar<T, V>& visitor)
	{
		pos = visitor.pos;
		vel = visitor.vel;
		return *this;
	}

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVar<T, V> operator+(const StateVar<T, V>& visitor);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVar<T, V> operator-(const StateVar<T, V>& visitor);

	/** @brief Mix this state with another one
	 *
	 * This can be used as a relaxation method when looking for stationary
	 * solutions
	 * @param visitor The other state
	 * @param f The mix factor. If 0.0, the state is not altered at all. If 1.0
	 * the state is completely replaced by the @p visitor
	 */
	void Mix(const StateVar<T, V>& visitor, const real& f);
};

/** @class StateVarDeriv Time.hpp
 * @brief Generic state variables derivative
 *
 * This is holding the velocities and accelerations
 */
template<class T, class V = T>
class StateVarDeriv
{
  public:
	/// @brief Constructor
	StateVarDeriv() {}

	/// @brief Destructor
	~StateVarDeriv() {}

	/// The velocity
	T vel;
	/// The acceleration
	V acc;

	/** @brief Give a string representation of the state variables
	 *
	 * Useful for debugging purposes
	 * @return A string representation
	 */
	string AsString() const;

	/** @brief Copy operator
	 * @param visitor The entity to copy
	 */
	StateVarDeriv<T, V>& operator=(const StateVarDeriv<T, V>& visitor)
	{
		vel = visitor.vel;
		acc = visitor.acc;
		return *this;
	}

	/** @brief Integrate in time
	 * @param dt The time step
	 * @return The state variables increment
	 */
	StateVar<T, V> operator*(const real& dt);

	/** @brief Sum operator
	 * @param visitor The entity to sum
	 */
	StateVarDeriv<T, V> operator+(const StateVarDeriv<T, V>& visitor);

	/** @brief Subtract operator
	 * @param visitor The entity to subtract
	 */
	StateVarDeriv<T, V> operator-(const StateVarDeriv<T, V>& visitor);

	/** @brief Transform the variation rate to a stationary case
	 *
	 * In MoorDyn the states variation rates are called velocity and
	 * acceleration, because that is indeed the physical meaning they have.
	 *
	 * However, they are actually the position variation rate and the velocity
	 * variation rate respectively. Thus, replacing the former by the later
	 * multiplied by half of the time step, and vanishing the later, would
	 * be equivalent to getting an infinite viscosity, i.e. the system would
	 * not take velocity at all.
	 *
	 * This can be use therefore to look for stationary solutions
	 * @param dt Time step.
	 * @return The module of the linear acceleration, or their sum in case
	 * of lists of accelerations
	 */
	real MakeStationary(const real &dt);

	/** @brief Carry out a Newmark step
	 *
	 * The resulting state rate of change will have the following velocity
	 *
	 * \f[ u(t_{n+1}) = u(t_{n}) + \Delta t (
	 *         (1/2 - \beta) \dot{u(t_{n})} +
	 *         \beta \dot{u(t_{n+1})}) \f]
	 *
	 * and the following acceleration
	 *
	 * \f[ \dot{u(t_{n+1})} = (1 - \gamma) \dot{u(t_{n})} +
	 *                        \gamma \dot{u(t_{n+1})}) \f]
	 *
	 * @param visitor The acceleration at the next time step
	 * @param dt Time step.
	 * @param gamma The Newmark gamma factor.
	 * @param beta Time Newmark beta factor.
	 */
	StateVarDeriv<T, V> Newmark(const StateVarDeriv<T, V>& visitor,
	                            const real& dt,
	                            real gamma = 0.5,
	                            real beta = 0.25);

	/** @brief Carry out a Wilson step
	 *
	 * The resulting state rate of change will have the following acceleration
	 *
	 * \f[ \dot{u(t_{n+1})} =
	 *         (1 - \frac{\tau}{2 \theta \Delta t}) \dot{u(t_{n})} +
	 *         \frac{\tau}{2 \theta \Delta t} \dot{u(t_{n+1})}) \f]
	 *
	 * and the following velocity
	 *
	 * \f[ u(t_{n+1}) = u(t_{n}) + \frac{\tau}{2} (
	 *         (1 - \frac{\tau}{3 \theta \Delta t}) \dot{u(t_{n})} +
	 *         \frac{\tau}{3 \theta \Delta t} \dot{u(t_{n+1})}) \f]
	 *
	 * Note that \f$ \tau \f$ can be smaller than \f$ \theta \Delta t \f$.
	 *
	 * @param visitor The acceleration at the next time step
	 * @param tau Time advancing, \f$ \tau \f$.
	 * @param dt Enlarged time step, \f$ \theta \Delta t \f$.
	 */
	StateVarDeriv<T, V> Wilson(const StateVarDeriv<T, V>& visitor,
	                           const real& tau,
	                           const real& dt);

	/** @brief Mix this state variation rate with another one
	 *
	 * This can be used as a relaxation method when looking for stationary
	 * solutions
	 * @param visitor The other state variation rate
	 * @param f The mix factor. If 0.0, the state is not altered at all. If 1.0
	 * the state is completely replaced by the @p visitor
	 */
	void Mix(const StateVarDeriv<T, V>& visitor, const real& f);
};

/// The state variables for lines
typedef StateVar<std::vector<vec>> LineState;

/// The state variables derivative for lines
typedef StateVarDeriv<std::vector<vec>> DLineStateDt;

/// The state variables for points
typedef StateVar<vec> PointState;

/// The state variables derivative for points
typedef StateVarDeriv<vec> DPointStateDt;

/// The state variables for rods
typedef StateVar<XYZQuat, vec6> RodState;

/// The state variables derivative for rods
typedef StateVarDeriv<XYZQuat, vec6> DRodStateDt;

/// The state variables for bodies
typedef StateVar<XYZQuat, vec6> BodyState;

/// The state variables derivative for bodies
typedef StateVarDeriv<XYZQuat, vec6> DBodyStateDt;

/** @class MoorDynState Time.hpp
 * @brief The collection of state variables of the whole system
 */
class MoorDynState
{
  public:
	/// @brief Constructor
	MoorDynState() {}

	/// @brief Destructor
	~MoorDynState() {}

	/// The states of the lines
	std::vector<LineState> lines;

	/// The states of the points
	std::vector<PointState> points;

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

	/** @brief Mix this state with another one
	 *
	 * This can be used as a relaxation method when looking for stationary
	 * solutions
	 * @param visitor The other state
	 * @param f The mix factor. If 0.0, the state is not altered at all. If 1.0
	 * the state is completely replaced by the @p visitor
	 */
	void Mix(const MoorDynState& visitor, const real& f);
};

/** @class DMoorDynStateDt Time.hpp
 * @brief The collection of state variable derivatives of the whole system
 */
class DMoorDynStateDt
{
  public:
	/// @brief Constructor
	DMoorDynStateDt() {}

	/// @brief Destructor
	~DMoorDynStateDt() {}

	/// The state derivatives of the lines
	std::vector<DLineStateDt> lines;

	/// The state derivatives of the points
	std::vector<DPointStateDt> points;

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

	/** @brief Transform the variation rate to a stationary case
	 *
	 * In MoorDyn the states variation rates are called velocity and
	 * acceleration, because that is indeed the physical meaning they have.
	 *
	 * However, they are actually the position variation rate and the velocity
	 * variation rate respectively. Thus, replacing the former by the later
	 * multiplied by half of the time step, and vanishing the later, would
	 * be equivalent to getting an infinite viscosity, i.e. the system would
	 * not take velocity at all.
	 *
	 * This can be use therefore to look for stationary solutions
	 * @param dt Time step.
	 * @return The sum of the linear acceleration norms
	 */
	real MakeStationary(const real &dt);

	/** @brief Carry out a Newmark step
	 *
	 * The resulting state rate of change will have the following velocity
	 *
	 * \f[ u(t_{n+1}) = u(t_{n}) + \Delta t (
	 *         (1/2 - \beta) \dot{u(t_{n})} +
	 *         \beta \dot{u(t_{n+1})}) \f]
	 *
	 * and the following acceleration
	 *
	 * \f[ \dot{u(t_{n+1})} = (1 - \gamma) \dot{u(t_{n})} +
	 *                        \gamma \dot{u(t_{n+1})}) \f]
	 *
	 * @param visitor The acceleration at the next time step
	 * @param dt Time step.
	 * @param gamma The Newmark gamma factor.
	 * @param beta Time Newmark beta factor.
	 */
	DMoorDynStateDt Newmark(const DMoorDynStateDt& visitor,
	                        const real& dt,
	                        real gamma = 0.5,
	                        real beta = 0.25);

	/** @brief Carry out a Wilson step
	 *
	 * The resulting state rate of change will have the following acceleration
	 *
	 * \f[ \dot{u(t_{n+1})} =
	 *         (1 - \frac{\tau}{2 \theta \Delta t}) \dot{u(t_{n})} +
	 *         \frac{\tau}{2 \theta \Delta t} \dot{u(t_{n+1})}) \f]
	 *
	 * and the following velocity
	 *
	 * \f[ u(t_{n+1}) = u(t_{n}) + \frac{\tau}{2} (
	 *         (1 - \frac{\tau}{3 \theta \Delta t}) \dot{u(t_{n})} +
	 *         \frac{\tau}{3 \theta \Delta t} \dot{u(t_{n+1})}) \f]
	 *
	 * Note that \f$ \tau \f$ can be smaller than \f$ \theta \Delta t \f$.
	 *
	 * @param visitor The acceleration at the next time step
	 * @param tau Time advancing, \f$ \tau \f$.
	 * @param dt Enlarged time step, \f$ \theta \Delta t \f$.
	 */
	DMoorDynStateDt Wilson(const DMoorDynStateDt& visitor,
	                       const real& tau,
	                       const real& dt);

	/** @brief Mix this state variation rate with another one
	 *
	 * This can be used as a relaxation method when looking for stationary
	 * solutions
	 * @param visitor The other state variation rate
	 * @param f The mix factor. If 0.0, the state is not altered at all. If 1.0
	 * the state is completely replaced by the @p visitor
	 */
	void Mix(const DMoorDynStateDt& visitor, const real& f);
};

} // ::moordyn
