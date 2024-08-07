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
#include <vector>

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
	CFL() : _l((std::numeric_limits<real>::max)()) {};

	/** @brief Destructor
	 */
	virtual ~CFL() {};

	/** @brief Get the timestep from a CFL factor
	 * @param cfl CFL factor
	 * @return The timestep
	 */
	virtual inline real cfl2dt(const real cfl) const { return (std::numeric_limits<real>::max)(); }

	/** @brief Get the CFL factor from a timestep
	 * @param dt Timestep
	 * @return CFL factor
	 */
	virtual inline real dt2cfl(const real dt) const { return 0.0; }

	/** @brief Get the timestep from a CFL factor and velocity
	 * @param cfl CFL factor
	 * @param v velocity
	 * @return The timestep
	 */
	virtual inline real cfl2dt(const real cfl, const real v) const { return cfl * length() / v; }

	/** @brief Get the CFL factor from a timestep and velocity
	 * @param dt Timestep
	 * @param v velocity
	 * @return CFL factor
	 */
	virtual inline real dt2cfl(const real dt, const real v) const { return dt * v / length(); }

  protected:
	/** @brief Set the characteristic length of the system
	 * @param l lenght
	 */
	inline void length(real l) { _l = l; }

	/** @brief Get the characteristic length of the system
	 * @return lenght
	 */
	inline real length() const { return _l; }

  private:
	/// Length
	real _l;
};

/** @class NatFreqCFL CFL.hpp
 * @brief CFL for objects based on a natural frequency
 */
class NatFreqCFL : public CFL
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
	inline real cfl2dt(const real cfl) const { return cfl * period(); }

	/** @brief Get the CFL factor from a timestep
	 * @param dt Timestep
	 * @return CFL factor
	 */
	inline real dt2cfl(const real dt) const { return dt / period(); }

	/** @brief Get the timestep from a CFL factor and velocity
	 * @param cfl CFL factor
	 * @param v velocity
	 * @return The timestep
	 */
	inline real cfl2dt(const real cfl, const real v) const { return CFL::cfl2dt(cfl, v); }

	/** @brief Get the CFL factor from a timestep and velocity
	 * @param dt Timestep
	 * @param v velocity
	 * @return CFL factor
	 */
	inline real dt2cfl(const real dt, const real v) const { return CFL::dt2cfl(dt, v); }

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

/** @class SuperCFL CFL.hpp
 * @brief CFL extracted from connected objects
 *
 * Some objects has not an actual CFL definition, but they instead compute it
 * from the entities attached to it.
 */
class SuperCFL : public CFL
{
  public:
	/** @brief Constructor
	 */
	SuperCFL() {};

	/** @brief Destructor
	 */
	virtual ~SuperCFL() {};

	/** @brief Get the timestep from a CFL factor
	 * @param cfl CFL factor
	 * @return The timestUtilep
	 */
	inline real cfl2dt(const real cfl) const {
		auto dt = CFL::cfl2dt(cfl);
		for (auto obj : _children)
			dt = (std::min)(dt, obj->cfl2dt(cfl));
		return dt;
	}

	/** @brief Get the CFL factor from a timestep
	 * @param dt Timestep
	 * @return CFL factor
	 */
	inline real dt2cfl(const real dt) const {
		auto cfl = CFL::dt2cfl(dt);
		for (auto obj : _children)
			cfl = (std::max)(cfl, obj->dt2cfl(dt));
		return cfl;
	}

	/** @brief Get the timestep from a CFL factor and velocity
	 * @param cfl CFL factor
	 * @param v velocity
	 * @return The timestep
	 */
	inline real cfl2dt(const real cfl, const real v) const{
		auto dt = CFL::cfl2dt(cfl, v);
		for (auto obj : _children)
			dt = (std::min)(dt, obj->cfl2dt(cfl, v));
		return dt;
	}

	/** @brief Get the CFL factor from a timestep and velocity
	 * @param dt Timestep
	 * @param v velocity
	 * @return CFL factor
	 */
	inline real dt2cfl(const real dt, const real v) const {
		auto cfl = CFL::dt2cfl(dt, v);
		for (auto obj : _children)
			cfl = (std::max)(cfl, obj->dt2cfl(dt, v));
		return cfl;
	}

  protected:
	/** @brief Add a child
	 * @param c child
	 */
	inline void AddChild(CFL* c) { _children.push_back(c); }

	/** @brief Remove a child
	 * @param c child
	 */
	inline void RemoveChild(CFL* c) {
		_children.erase(std::remove(_children.begin(), _children.end(), c),
		                _children.end());
	}

  private:
	/// List of children
	std::vector<CFL*> _children;
};

} // ::moordyn
