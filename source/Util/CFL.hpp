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

/** @file IO.hpp
 * Input/Output operations
 */

#pragma once

#include "Misc.hpp"

namespace moordyn {

/** @class CFL CFL.hpp
 * @brief A base class for all the entitites that might be supported by a
 * Courant–Friedrichs–Lewy (CFL) condition 
 */
class CFL
{
  public:
	/** @brief Costructor
	 */
	CFL() {}

	/** @brief Destructor
	 */
	~CFL() {}

	/** @brief Compute the CFL number
	 *
	 * A Courant–Friedrichs–Lewy (CFL) bigger than 1.0 means that the physics
	 * cannot be correctly represented
	 * @param a Characteristic acceleration
	 * @param dt Time step
	 */
	virtual real CFLNumber(const real& a, const real& dt) const { return 0; }

  protected:
	/** @brief Compute the CFL number
	 * @param l Characteristic length
	 * @param v Characteristic velocity
	 * @param a Characteristic acceleration
	 * @param dt Time step
	 */
	virtual real CFLNumber(const real& l,
	                       const real& v,
	                       const real& a,
	                       const real& dt) const
	{
		real dl = dt * v + 0.5 * dt * dt * a;
		return dl / l;
	}
};

} // ::moordyn
