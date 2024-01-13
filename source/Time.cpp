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

#include "Time.hpp"
#include "Waves.hpp"
#include <sstream>

using namespace std;

namespace moordyn {

template<unsigned int NSTATE, unsigned int NDERIV>
void
TimeSchemeBase<NSTATE, NDERIV>::Update(real t_local, unsigned int substep)
{
	ground->updateFairlead(this->t);

	t_local += this->t_local;
	for (auto obj : bodies) {
		if ((obj->type != Body::COUPLED) && (obj->type != Body::CPLDPIN))
			continue;
		obj->updateFairlead(t_local);
	}
	for (auto obj : rods) {
		if ((obj->type != Rod::COUPLED) && (obj->type != Rod::CPLDPIN))
			continue;
		obj->updateFairlead(t_local);
	}
	for (auto obj : points) {
		if (obj->type != Point::COUPLED)
			continue;
		obj->updateFairlead(t_local);
	}
	for (auto obj : lines) {
		obj->updateUnstretchedLength(t_local);
	}

	for (unsigned int i = 0; i < bodies.size(); i++) {
		if ((bodies[i]->type != Body::FREE) && (bodies[i]->type != Body::CPLDPIN))
			continue;
		bodies[i]->setState(r[substep].bodies[i].pos, r[substep].bodies[i].vel);
	}

	for (unsigned int i = 0; i < rods.size(); i++) {
		rods[i]->setTime(this->t);
		if ((rods[i]->type != Rod::PINNED) && (rods[i]->type != Rod::CPLDPIN) &&
		    (rods[i]->type != Rod::FREE))
			continue;
		rods[i]->setState(r[substep].rods[i].pos, r[substep].rods[i].vel);
	}

	for (unsigned int i = 0; i < points.size(); i++) {
		if (points[i]->type != Point::FREE)
			continue;
		points[i]->setState(r[substep].points[i].pos, r[substep].points[i].vel);
	}

	for (unsigned int i = 0; i < lines.size(); i++) {
		lines[i]->setTime(this->t);
		lines[i]->setState(r[substep].lines[i].pos, r[substep].lines[i].vel);
	}
}

template<unsigned int NSTATE, unsigned int NDERIV>
void
TimeSchemeBase<NSTATE, NDERIV>::CalcStateDeriv(unsigned int substep)
{
	waves->updateWaves();

	for (unsigned int i = 0; i < lines.size(); i++) {
		if (!_calc_mask.lines[i])
			continue;
		std::tie(rd[substep].lines[i].vel, rd[substep].lines[i].acc) =
		    lines[i]->getStateDeriv();
	}

	for (unsigned int i = 0; i < points.size(); i++) {
		if (!_calc_mask.points[i])
			continue;
		if (points[i]->type != Point::FREE)
			continue;
		std::tie(rd[substep].points[i].vel, rd[substep].points[i].acc) =
		    points[i]->getStateDeriv();
	}

	for (unsigned int i = 0; i < rods.size(); i++) {
		if (!_calc_mask.rods[i])
			continue;
		if ((rods[i]->type != Rod::PINNED) && (rods[i]->type != Rod::CPLDPIN) &&
		    (rods[i]->type != Rod::FREE))
			continue;
		std::tie(rd[substep].rods[i].vel, rd[substep].rods[i].acc) =
		    rods[i]->getStateDeriv();
	}

	for (unsigned int i = 0; i < bodies.size(); i++) {
		if (!_calc_mask.bodies[i])
			continue;
		if ((bodies[i]->type != Body::FREE) && (bodies[i]->type != Body::CPLDPIN))
			continue;
		std::tie(rd[substep].bodies[i].vel, rd[substep].bodies[i].acc) =
		    bodies[i]->getStateDeriv();
	}

	for (auto obj : points) {
		if (obj->type != Point::COUPLED)
			continue;
		obj->doRHS();
	}
	for (auto obj : rods) {
		if (obj->type != Rod::COUPLED)
			continue;
		obj->doRHS();
	}
	for (auto obj : bodies) {
		if (obj->type != Body::COUPLED)
			continue;
		obj->doRHS();
	}

	// call ground body to update all the fixed things
	ground->setDependentStates(); // NOTE: (not likely needed)
}

template<unsigned int NSTATE, unsigned int NDERIV>
void
TimeSchemeBase<NSTATE, NDERIV>::FromStationary(const StationaryScheme& state)
{
	r[0] = state.r[0];
}


StationaryScheme::StationaryScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : TimeSchemeBase(log, waves)
  , _error(0.0)
  , _booster(1.0)
{
	name = "Stationary solution";
}

#ifndef STATIONARY_BOOSTING
#define STATIONARY_BOOSTING 1.01
#endif

#ifndef STATIONARY_MAX_BOOSTING
#define STATIONARY_MAX_BOOSTING 50.0
#endif

#ifndef STATIONARY_MIN_BOOSTING
#define STATIONARY_MIN_BOOSTING 0.1
#endif

#ifndef STATIONARY_RELAX
#define STATIONARY_RELAX 0.5
#endif

void
StationaryScheme::Step(real& dt)
{
	Update(0.0, 0);
	CalcStateDeriv(0);
	const real error_prev = _error;
	_error = rd[0].MakeStationary(dt);
	if (error_prev != 0.0) {
		if (error_prev >= _error) {
			// Let's try to boost the convergence
			_booster *= STATIONARY_BOOSTING;
		}
		else if (error_prev < _error) {
			// We clearly overshot, so let's relax the solution and reduce the
			// boosting
			_booster /= STATIONARY_BOOSTING;
			r[0].Mix(r[1], STATIONARY_RELAX);
			Update(0.0, 0);
			CalcStateDeriv(0);
			_error = rd[0].MakeStationary(dt);
		}
	}
	if (_booster > STATIONARY_MAX_BOOSTING)
		_booster = STATIONARY_MAX_BOOSTING;
	else if (_booster < STATIONARY_MIN_BOOSTING)
		_booster = STATIONARY_MIN_BOOSTING;

	real new_dt = _booster * dt;
	// Check that the time step is not too large, or limit it otherwise
	real v = 0.5 * dt * _error;
	for (auto obj : lines)
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	for (auto obj : points) {
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	}
	for (auto obj : rods) {
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	}
	for (auto obj : bodies) {
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	}

	r[1] = r[0];
	r[0] = r[0] + rd[0] * new_dt;
	t += dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

EulerScheme::EulerScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : TimeSchemeBase(log, waves)
{
	name = "1st order Euler";
}

void
EulerScheme::Step(real& dt)
{
	Update(0.0, 0);
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;
	t += dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

template<unsigned int NSTATE, unsigned int NDERIV>
void
LocalTimeSchemeBase<NSTATE, NDERIV>::SetCalcMask(real& dt)
{
	unsigned int i = 0;
	for (i = 0; i < this->lines.size(); i++) {
		_dt.lines[i] += dt;
		if (_dt.lines[i] >= _dt0.lines[i]) {
			_dt.lines[i] = dt;
			this->_calc_mask.lines[i] = true;
		} else {
			this->_calc_mask.lines[i] = false;
		}
	}
	for (i = 0; i < this->points.size(); i++) {
		_dt.points[i] += dt;
		if (_dt.points[i] >= _dt0.points[i]) {
			_dt.points[i] = dt;
			this->_calc_mask.points[i] = true;
		} else {
			this->_calc_mask.points[i] = false;
		}
	}
	for (i = 0; i < this->rods.size(); i++) {
		_dt.rods[i] += dt;
		if (_dt.rods[i] >= _dt0.rods[i]) {
			_dt.rods[i] = dt;
			this->_calc_mask.rods[i] = true;
		} else {
			this->_calc_mask.rods[i] = false;
		}
	}
	for (i = 0; i < this->bodies.size(); i++) {
		_dt.bodies[i] += dt;
		if (_dt.bodies[i] >= _dt0.bodies[i]) {
			_dt.bodies[i] = dt;
			this->_calc_mask.bodies[i] = true;
		} else {
			this->_calc_mask.bodies[i] = false;
		}
	}
}

template<unsigned int NSTATE, unsigned int NDERIV>
real
LocalTimeSchemeBase<NSTATE, NDERIV>::ComputeDt()
{
	this->LOGMSG << this->name << ":" << endl;
	real dt = std::numeric_limits<real>::max();
	for (auto obj : this->lines)
		dt = (std::min)(dt, obj->cfl2dt(this->cfl));
	for (auto obj : this->points)
		dt = (std::min)(dt, obj->cfl2dt(this->cfl));
	for (auto obj : this->rods)
		dt = (std::min)(dt, obj->cfl2dt(this->cfl));
	for (auto obj : this->bodies)
		dt = (std::min)(dt, obj->cfl2dt(this->cfl));

	for (auto line : this->lines) {
		const real dt_line = line->cfl2dt(this->cfl);
		_dt0.lines.push_back(0.999 * dt_line);
		_dt.lines.push_back(dt_line);
		this->LOGMSG << "Line " << line->number << ": dt = " << dt_line
		             << " s (updated each " << std::ceil(dt_line / dt)
		             << " timesteps)" << endl;
	}
	for (auto point : this->points) {
		this->LOGMSG << "Point " << point->number << ": dt = " << dt
		             << " s (updated each 1 timesteps)" << endl;
		_dt0.points.push_back(0.0);
		_dt.points.push_back(0.0);
	}
	for (auto rod : this->rods) {
		this->LOGMSG << "Rod " << rod->number << ": dt = " << dt
		             << " s (updated each 1 timesteps)" << endl;
		_dt0.rods.push_back(0.0);
		_dt.rods.push_back(0.0);
	}
	for (auto body : this->bodies) {
		this->LOGMSG << "Body " << body->number << ": dt = " << dt
		             << " s (updated each 1 timesteps)" << endl;
		_dt0.bodies.push_back(0.0);
		_dt.bodies.push_back(0.0);
	}

	return dt;
}

LocalEulerScheme::LocalEulerScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : LocalTimeSchemeBase(log, waves)
{
	name = "1st order Local-Timestep Euler";
}

void
LocalEulerScheme::Step(real& dt)
{
	SetCalcMask(dt);
	Update(0.0, 0);
	CalcStateDeriv(0);
	r[0] = r[0] + rd[0] * dt;
	t += dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

HeunScheme::HeunScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : TimeSchemeBase(log, waves)
{
	name = "2nd order Heun";
}

void
HeunScheme::Step(real& dt)
{
	// Apply the latest knew derivative, as a predictor
	r[0] = r[0] + rd[0] * dt;
	rd[1] = rd[0];
	// Compute the new derivative
	Update(0.0, 0);
	CalcStateDeriv(0);
	// Correct the integration
	r[0] = r[0] + (rd[0] - rd[1]) * (0.5 * dt);

	t += dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

RK2Scheme::RK2Scheme(moordyn::Log* log, moordyn::WavesRef waves)
  : TimeSchemeBase(log, waves)
{
	name = "2nd order Runge-Kutta";
}

void
RK2Scheme::Step(real& dt)
{
	Update(0.0, 0);

	// Compute the intermediate state
	CalcStateDeriv(0);
	t += 0.5 * dt;
	r[1] = r[0] + rd[0] * (0.5 * dt);
	Update(0.5 * dt, 1);
	// And so we can compute the new derivative and apply it
	CalcStateDeriv(1);
	r[0] = r[0] + rd[1] * dt;
	t += 0.5 * dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

RK4Scheme::RK4Scheme(moordyn::Log* log, moordyn::WavesRef waves)
  : TimeSchemeBase(log, waves)
{
	name = "4th order Runge-Kutta";
}

void
RK4Scheme::Step(real& dt)
{
	Update(0.0, 0);

	// k1
	CalcStateDeriv(0);

	// k2
	t += 0.5 * dt;
	r[1] = r[0] + rd[0] * (0.5 * dt);
	Update(0.5 * dt, 1);
	CalcStateDeriv(1);

	// k3
	r[1] = r[0] + rd[1] * (0.5 * dt);
	Update(0.5 * dt, 1);
	CalcStateDeriv(2);

	// k4
	t += 0.5 * dt;
	r[2] = r[0] + rd[2] * dt;
	Update(dt, 2);
	CalcStateDeriv(3);

	// Apply
	r[0] = r[0] + (rd[0] + rd[3]) * (dt / 6.0) + (rd[1] + rd[2]) * (dt / 3.0);

	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

template<unsigned int order, bool local>
ABScheme<order, local>::ABScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : LocalTimeSchemeBase(log, waves)
  , n_steps(0)
{
	stringstream s;
	s << order << "th order ";
	if (local)
		s << "Local-";
	s << "Adam-Bashforth";
	name = s.str();
	if (order > 4) {
		LOGWRN << name
		       << " scheme queried, but 4th order is the maximum implemented"
		       << endl;
	}
}

template<unsigned int order, bool local>
void
ABScheme<order, local>::Step(real& dt)
{
	Update(0.0, 0);
	shift();

	// Get the new derivative
	if (local && (n_steps == order))
		SetCalcMask(dt);
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

	n_steps = (std::min)(n_steps + 1, order);
	t += dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

ImplicitEulerScheme::ImplicitEulerScheme(moordyn::Log* log,
                                         moordyn::WavesRef waves,
                                         unsigned int iters,
                                         real dt_factor)
  : TimeSchemeBase(log, waves)
  , _iters(iters)
  , _dt_factor(dt_factor)
{
	stringstream s;
	s << "k=" << dt_factor << " implicit Euler (" << iters << " iterations)";
	name = s.str();
}

void
ImplicitEulerScheme::Step(real& dt)
{
	t += _dt_factor * dt;
	for (unsigned int i = 0; i < _iters; i++) {
		r[1] = r[0] + rd[0] * (_dt_factor * dt);
		Update(_dt_factor * dt, 1);
		CalcStateDeriv(0);
	}

	// Apply
	r[0] = r[0] + rd[0] * dt;
	t += (1.0 - _dt_factor) * dt;
	Update(dt, 0);
	TimeSchemeBase::Step(dt);
}

TimeScheme*
create_time_scheme(const std::string& name,
                   moordyn::Log* log,
                   moordyn::WavesRef waves)
{
	TimeScheme* out = NULL;
	if (str::lower(name) == "euler") {
		out = new EulerScheme(log, waves);
	} else if (str::lower(name) == "leuler") {
		out = new LocalEulerScheme(log, waves);
	} else if (str::lower(name) == "heun") {
		out = new HeunScheme(log, waves);
	} else if (str::lower(name) == "rk2") {
		out = new RK2Scheme(log, waves);
	} else if (str::lower(name) == "rk4") {
		out = new RK4Scheme(log, waves);
	} else if (str::lower(name) == "ab2") {
		out = new ABScheme<2, false>(log, waves);
	} else if (str::lower(name) == "ab3") {
		out = new ABScheme<3, false>(log, waves);
	} else if (str::lower(name) == "ab4") {
		out = new ABScheme<4, false>(log, waves);
	} else if (str::lower(name) == "lab2") {
		out = new ABScheme<2, false>(log, waves);
	} else if (str::lower(name) == "lab3") {
		out = new ABScheme<3, false>(log, waves);
	} else if (str::lower(name) == "lab4") {
		out = new ABScheme<4, false>(log, waves);
	} else if (str::startswith(str::lower(name), "beuler")) {
		try {
			unsigned int iters = std::stoi(name.substr(6));
			out = new ImplicitEulerScheme(log, waves, iters, 1.0);
		} catch (std::invalid_argument) {
			stringstream s;
			s << "Invalid Backward Euler name format '" << name << "'";
			throw moordyn::invalid_value_error(s.str().c_str());
		}
	} else if (str::startswith(str::lower(name), "midpoint")) {
		try {
			unsigned int iters = std::stoi(name.substr(8));
			out = new ImplicitEulerScheme(log, waves, iters, 0.5);
		} catch (std::invalid_argument) {
			stringstream s;
			s << "Invalid Midpoint name format '" << name << "'";
			throw moordyn::invalid_value_error(s.str().c_str());
		}
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
