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
#include <sstream>

using namespace std;

namespace moordyn {

EulerScheme::EulerScheme(moordyn::Log* log)
  : TimeSchemeBase(log)
{
	name = "1st order Euler";
}

void
EulerScheme::Step(real& dt)
{
	Update(t, 0);
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;
	t += dt;
	Update(t, 0);
}

HeunScheme::HeunScheme(moordyn::Log* log)
  : TimeSchemeBase(log)
{
	name = "2nd order Heun";
}

void
HeunScheme::Step(real& dt)
{
	Update(t, 0);

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
  : TimeSchemeBase(log)
{
	name = "2nd order Runge-Kutta";
}

void
RK2Scheme::Step(real& dt)
{
	Update(t, 0);

	// Compute the intermediate state
	CalcStateDeriv(0);
	r[1] = r[0] + rd[0] * (0.5 * dt);
	Update(t + 0.5 * dt, 1);
	// And so we can compute the new derivative and apply it
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;

	t += dt;
	Update(t, 0);
}

RK4Scheme::RK4Scheme(moordyn::Log* log)
  : TimeSchemeBase(log)
{
	name = "4th order Runge-Kutta";
}

void
RK4Scheme::Step(real& dt)
{
	Update(t, 0);

	// k1
	CalcStateDeriv(0);

	// k2
	r[1] = r[0] + rd[0] * (0.5 * dt);
	Update(t + 0.5 * dt, 1);
	CalcStateDeriv(1);

	// k3
	r[1] = r[0] + rd[1] * (0.5 * dt);
	Update(t + 0.5 * dt, 1);
	CalcStateDeriv(2);

	// k4
	r[2] = r[0] + rd[2] * dt;
	Update(t + dt, 2);
	CalcStateDeriv(3);

	// Apply
	r[0] = r[0] + (rd[0] + rd[3]) * (dt / 6.0) + (rd[1] + rd[2]) * (dt / 3.0);

	t += dt;
	Update(t, 0);
}

template<unsigned int order>
ABScheme<order>::ABScheme(moordyn::Log* log)
  : TimeSchemeBase(log)
  , n_steps(0)
{
	stringstream s;
	s << order << "th order Adam-Bashforth";
	name = s.str();
	if (order > 4) {
		LOGWRN << name
		       << " scheme queried, but 4th order is the maximum implemented"
		       << endl;
	}
}

template<unsigned int order>
void
ABScheme<order>::Step(real& dt)
{
	Update(t, 0);
	shift();

	// Get the new derivative
	CalcStateDeriv(0);

	// Apply different formulas depending on the number of derivatives available
	switch (n_steps) {
		case 0:
			r[0] = r[0] + rd[0] * dt;
			break;
		case 1:
			r[0] = r[0] + rd[0] * (dt * 1.5) - rd[1] * (dt * 0.5);
			break;
		case 2:
			r[0] = r[0] + rd[0] * (dt * 23.0 / 12.0) -
			       rd[1] * (dt * 4.0 / 3.0) + rd[2] * (dt * 5.0 / 12.0);
			break;
		case 3:
			r[0] = r[0] + rd[0] * (dt * 55.0 / 24.0) -
			       rd[1] * (dt * 59.0 / 24.0) + rd[2] * (dt * 37.0 / 24.0) -
			       rd[3] * (dt * 3.0 / 8.0);
			break;
		default:
			r[0] = r[0] + rd[0] * (dt * 1901.0 / 720.0) -
			       rd[1] * (dt * 1387.0 / 360.0) + rd[2] * (dt * 109.0 / 30.0) -
			       rd[3] * (dt * 637.0 / 360.0) + rd[4] * (dt * 251.0 / 720.0);
	}

	t += dt;
	Update(t, 0);
}

TimeScheme*
create_time_scheme(const std::string& name, moordyn::Log* log)
{
	TimeScheme* out = NULL;
	if (str::lower(name) == "euler") {
		out = new EulerScheme(log);
	} else if (str::lower(name) == "heun") {
		out = new HeunScheme(log);
	} else if (str::lower(name) == "rk2") {
		out = new RK2Scheme(log);
	} else if (str::lower(name) == "rk4") {
		out = new RK4Scheme(log);
	} else if (str::lower(name) == "ab3") {
		out = new ABScheme<3>(log);
	} else if (str::lower(name) == "ab4") {
		out = new ABScheme<4>(log);
	} else {
		stringstream s;
		s << "Unknown time scheme '" << name << "'";
		throw moordyn::invalid_value_error(s.str().c_str());
	}
	if (!out)
		throw moordyn::mem_error("Failure allocating the time scheme");
	return out;
}

} // ::moordyn
