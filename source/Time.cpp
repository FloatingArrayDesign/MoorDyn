/*
 * Copyright (c) 2019 Matt Hall <mtjhall@alumni.uvic.ca>
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

#include "Time.hpp"

using namespace std;

namespace moordyn {

EulerScheme::EulerScheme(moordyn::Log* log)
  : TimeScheme(log)
{
	name = "1st order Euler";
}

void
EulerScheme::Step(real& dt)
{
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;
	t += dt;
	Update(t, 0);
}

HeunScheme::HeunScheme(moordyn::Log* log)
  : TimeScheme(log)
{
	name = "2nd order Heun";
}

void
HeunScheme::Step(real& dt)
{
	// Apply the latest knew derivative, as a predictor
	r[0] = r[0] + rd[1] * dt;
	rd[1] = rd[0];
	// Compute the new derivative
	CalcStateDeriv(0);
	// Correct the integration
	r[0] = r[0] + (rd[1] - rd[0]) * dt;

	t += dt;
	Update(t, 0);
}

RK2Scheme::RK2Scheme(moordyn::Log* log)
  : TimeScheme(log)
{
	name = "2nd order RUnge-Kutta";
}

void
RK2Scheme::Step(real& dt)
{
	// Compute the intermediate state
	CalcStateDeriv(0);
	r[1] = r[0] + rd[1] * (0.5 * dt);
	Update(t + 0.5 * dt, 1);
	// And so we can compute the new derivative and apply it
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;

	t += dt;
	Update(t, 0);
}

} // ::moordyn
