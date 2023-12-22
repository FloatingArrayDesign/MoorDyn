/*
 * Copyright (c) 2023, Jose Luis Cercos-Pita & Matt Hall
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
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

/** @file CFL.hpp
 * Courant–Friedrichs–Lewy condition base classes
 */

#pragma once

#include "Misc.hpp"
#include <limits>
#include <cmath>

using namespace std;

namespace moordyn {

/** @class CFL CFL.hpp
 * @brief CFL object base class
 *
 * The base class does nothing but offering a tool which no matters the
 * timestep returns a 0 CFL factor, and no matters the CFL returns an infinite
 * timestep.
 *
 * Thus can be used for entitites which does not impose any kind of limitation
 * on the timestep
 */
class CFL
{
  public:
	/** @brief Constructor
	 */
	CFL() {};

	/** @brief Destructor
	 */
	virtual ~CFL() {};

	/** @brief Get the timestep from a CFL factor
	 * @param cfl CFL factor
	 * @return The timestep
	 */
	virtual inline real cfl2dt(const real cfl) const { return std::numeric_limits<real>::max(); }

	/** @brief Get the CFL factor from a timestep
	 * @param dt Timestep
	 * @return CFL factor
	 */
	virtual inline real dt2cfl(const real dt) const { return 0.0; }
};

/** @class NatFreqCFL CFL.hpp
 * @brief CFL for objects based on a natural frequency
 */
class NatFreqCFL
{
  public:
	/** @brief Constructor
	 */
	NatFreqCFL() {};

	/** @brief Destructor
	 */
	virtual ~NatFreqCFL() {};

	/** @brief Get the timestep from a CFL factor
	 * @param cfl CFL factor
	 * @return The timestUtilep
	 */
	virtual inline real cfl2dt(const real cfl) const { return cfl * period(); }

	/** @brief Get the CFL factor from a timestep
	 * @param dt Timestep
	 * @return CFL factor
	 */
	virtual inline real dt2cfl(const real dt) const { return dt / period(); }

  protected:
	/** @brief Set the stiffness of the system
	 * @param k stiffness
	 */
	inline void stiffness(real k) { _k = k; }

	/** @brief Get the stiffness of the system
	 * @return stiffness
	 */
	inline real stiffness() const { return _k; }

	/** @brief Set the mass of the system
	 * @param m mass
	 */
	inline void mass(real m) { _m = m; }

	/** @brief Get the mass of the system
	 * @return mass
	 */
	inline real mass() const { return _m; }

	/** @brief Get the natural frequency of the system
	 * @return natural angular frequency
	 */
	inline real frequency() const { return sqrt(_k / _m); }

	/** @brief Get the natural period of the system
	 * @return natural period
	 */
	inline real period() const { return 2.0 * pi / frequency(); }

  private:
	/// Stiffness
	real _k;

	/// mass
	real _m;
};

} // ::moordyn
