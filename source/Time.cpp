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
#include <sstream>

using namespace std;

namespace moordyn {

namespace time {

template<unsigned int NSTATE, unsigned int NDERIV>
void
SchemeBase<NSTATE, NDERIV>::Update(real t_local, unsigned int substep)
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
		if ((bodies[i]->type != Body::FREE) &&
		    (bodies[i]->type != Body::CPLDPIN))
			continue;
		bodies[i]->setState(AS_STATE(_r[substep])->get(bodies[i]));
	}

	for (unsigned int i = 0; i < rods.size(); i++) {
		rods[i]->setTime(this->t);
		if ((rods[i]->type != Rod::PINNED) && (rods[i]->type != Rod::CPLDPIN) &&
		    (rods[i]->type != Rod::FREE))
			continue;
		rods[i]->setState(AS_STATE(_r[substep])->get(rods[i]));
	}

	for (unsigned int i = 0; i < points.size(); i++) {
		if (points[i]->type != Point::FREE)
			continue;
		points[i]->setState(AS_STATE(_r[substep])->get(points[i]));
	}

	for (unsigned int i = 0; i < lines.size(); i++) {
		lines[i]->setTime(this->t);
		lines[i]->setState(AS_STATE(_r[substep])->get(lines[i]));
	}
}

template<unsigned int NSTATE, unsigned int NDERIV>
void
SchemeBase<NSTATE, NDERIV>::CalcStateDeriv(unsigned int substep)
{
	waves->updateWaves();

	for (unsigned int i = 0; i < lines.size(); i++) {
		if (!_calc_mask.lines[i])
			continue;
		lines[i]->getStateDeriv(AS_STATE(_rd[substep])->get(lines[i]));
	}

	for (unsigned int i = 0; i < points.size(); i++) {
		if (!_calc_mask.points[i])
			continue;
		if (points[i]->type != Point::FREE)
			continue;
		points[i]->getStateDeriv(AS_STATE(_rd[substep])->get(points[i]));
	}

	for (unsigned int i = 0; i < rods.size(); i++) {
		if (!_calc_mask.rods[i])
			continue;
		if ((rods[i]->type != Rod::PINNED) && (rods[i]->type != Rod::CPLDPIN) &&
		    (rods[i]->type != Rod::FREE))
			continue;
		rods[i]->getStateDeriv(AS_STATE(_rd[substep])->get(rods[i]));
	}

	for (unsigned int i = 0; i < bodies.size(); i++) {
		if (!_calc_mask.bodies[i])
			continue;
		if ((bodies[i]->type != Body::FREE) &&
		    (bodies[i]->type != Body::CPLDPIN))
			continue;
		bodies[i]->getStateDeriv(AS_STATE(_rd[substep])->get(bodies[i]));
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

StationaryScheme::StationaryScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : SchemeBase(log, waves)
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
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto drdt0 = rd(0)->get();

	Update(0.0, 0);
	CalcStateDeriv(0);
	const real error_prev = _error;
	MakeStationary(dt);
	if (error_prev != 0.0) {
		if (error_prev >= _error) {
			// Let's try to boost the convergence
			_booster *= STATIONARY_BOOSTING;
		} else if (error_prev < _error) {
			// We clearly overshot, so let's relax the solution and reduce the
			// boosting
			_booster /= STATIONARY_BOOSTING;
			const real f = STATIONARY_RELAX, fi = 1.0 - STATIONARY_RELAX;
			r0 = fi * r0 + f * r1;
			Update(0.0, 0);
			CalcStateDeriv(0);
			MakeStationary(dt);
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
	for (auto obj : points)
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	for (auto obj : rods)
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));
	for (auto obj : bodies)
		new_dt = (std::min)(new_dt, obj->cfl2dt(cfl, v));

	r1 = r0;
	r0 += new_dt * drdt0;
	t += dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

/** @brief Handy tool to convert vec6 derivatives onto vec7 quaternions
 * @param r The base quaternion on top of which the derivative is applied
 * @param rd The vec6 derivative
 */
vec7
integrateVec6AsVec7(const moordyn::vec7& r, const moordyn::vec6& rd)
{
	XYZQuat r7 = XYZQuat::fromVec7(r), v7;
	v7.pos = rd.head<3>();
	v7.quat = 0.5 * (quaternion(0, rd[3], rd[4], rd[5]) * r7.quat);
	return v7.toVec7();
}

#define MAKE_STATIONARY_VEC3(obj)                                              \
	{                                                                          \
		auto drdt = rd(id)->get(obj);                                          \
		for (unsigned int j = 0; j < drdt.rows(); j++) {                       \
			auto acc = drdt.row(j).segment<3>(3);                              \
			_error += acc.norm();                                              \
			drdt.row(j).head<3>() = 0.5 * dt * acc;                            \
			acc = vec3::Zero();                                                \
		}                                                                      \
	}

#define MAKE_STATIONARY_QUAT(obj)                                              \
	{                                                                          \
		const moordyn::vec7 pos = r(i)->get(obj).row(0).head<7>();             \
		auto drdt = rd(id)->get(obj).row(0);                                   \
		auto vel = drdt.head<7>();                                             \
		auto acc = drdt.tail<6>();                                             \
		_error += acc.head<3>().norm();                                        \
		vel = integrateVec6AsVec7(pos, 0.5 * dt * acc);                        \
		acc = vec6::Zero();                                                    \
	}

void
StationaryScheme::MakeStationary(real& dt, unsigned int i, unsigned int id)
{
	_error = 0.0;
	for (auto obj : lines)
		MAKE_STATIONARY_VEC3(obj);
	for (auto obj : points) {
		if (obj->type != Point::FREE)
			continue;
		MAKE_STATIONARY_VEC3(obj);
	}
	for (auto obj : rods) {
		if ((obj->type != Rod::PINNED) && (obj->type != Rod::CPLDPIN) &&
		    (obj->type != Rod::FREE))
			continue;
		MAKE_STATIONARY_QUAT(obj);
	}
	for (auto obj : bodies) {
		if ((obj->type != Body::FREE) && (obj->type != Body::CPLDPIN))
			continue;
		MAKE_STATIONARY_QUAT(obj);
	}
}

EulerScheme::EulerScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : SchemeBase(log, waves)
{
	name = "1st order Euler";
}

void
EulerScheme::Step(real& dt)
{
	Update(0.0, 0);
	CalcStateDeriv(0);
	r(0)->get() += dt * rd(0)->get();
	t += dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

template<unsigned int NSTATE, unsigned int NDERIV>
void
LocalSchemeBase<NSTATE, NDERIV>::SetCalcMask(real& dt)
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
LocalSchemeBase<NSTATE, NDERIV>::ComputeDt()
{
	this->LOGMSG << this->name << ":" << endl;
	real dt = (std::numeric_limits<real>::max)();
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
  : LocalSchemeBase(log, waves)
{
	name = "1st order Local-Timestep Euler";
}

void
LocalEulerScheme::Step(real& dt)
{
	SetCalcMask(dt);
	Update(0.0, 0);
	CalcStateDeriv(0);
	r(0)->get() += dt * rd(0)->get();
	t += dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

HeunScheme::HeunScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : SchemeBase(log, waves)
{
	name = "2nd order Heun";
}

void
HeunScheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();

	// Apply the latest knew derivative, as a predictor
	r0 += dt * drdt0;
	drdt1 = drdt0;
	// Compute the new derivative
	Update(0.0, 0);
	CalcStateDeriv(0);
	// Correct the integration
	r0 += (0.5 * dt) * (drdt0 - drdt1);

	t += dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

RK2Scheme::RK2Scheme(moordyn::Log* log, moordyn::WavesRef waves)
  : SchemeBase(log, waves)
{
	name = "2nd order Runge-Kutta";
}

void
RK2Scheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();

	Update(0.0, 0);
	// Compute the intermediate state
	CalcStateDeriv(0);
	t += 0.5 * dt;
	r1 = r0 + 0.5 * dt * drdt0;
	Update(0.5 * dt, 1);
	// And so we can compute the new derivative and apply it
	CalcStateDeriv(1);
	r0 += dt * drdt1;
	t += 0.5 * dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

RK4Scheme::RK4Scheme(moordyn::Log* log, moordyn::WavesRef waves)
  : SchemeBase(log, waves)
{
	name = "4th order Runge-Kutta";
}

void
RK4Scheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto r2 = r(2)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();
	auto drdt2 = rd(2)->get();
	auto drdt3 = rd(3)->get();

	Update(0.0, 0);

	// k1
	CalcStateDeriv(0);

	// k2
	t += 0.5 * dt;
	r1 = r0 + 0.5 * dt * drdt0;

	Update(0.5 * dt, 1);
	CalcStateDeriv(1);

	// k3
	r1 = r0 + 0.5 * dt * drdt1;
	Update(0.5 * dt, 1);
	CalcStateDeriv(2);

	// k4
	t += 0.5 * dt;
	r2 = r0 + dt * drdt2;

	Update(dt, 2);
	CalcStateDeriv(3);

	// Apply
	r0 += (dt / 6.0) * (drdt0 + drdt3) + (dt / 3.0) * (drdt1 + drdt2);

	Update(dt, 0);
	SchemeBase::Step(dt);
}

template<unsigned int order, bool local>
ABScheme<order, local>::ABScheme(moordyn::Log* log, moordyn::WavesRef waves)
  : LocalSchemeBase(log, waves)
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

	auto r0 = r(0)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();
	auto drdt2 = rd(2)->get();
	auto drdt3 = rd(3)->get();
	auto drdt4 = rd(4)->get();

	// Get the new derivative
	if (local && (n_steps == order))
		SetCalcMask(dt);
	CalcStateDeriv(0);

	// Apply different formulas depending on the number of derivatives available
	switch (n_steps) {
		case 0:
			r0 += dt * drdt0;
			break;
		case 1:
			r0 += 1.5 * drdt0 + 0.5 * drdt1;
			break;
		case 2:
			r0 += 23.0 / 12.0 * dt * drdt0 + 4.0 / 3.0 * dt * drdt1 +
			      5.0 / 12.0 * dt * drdt2;
			break;
		case 3:
			r0 += 55.0 / 24.0 * dt * drdt0 + 59.0 / 24.0 * dt * drdt1 +
			      37.0 / 24.0 * dt * drdt2 + 3.0 / 8.0 * dt * drdt3;
			break;
		default:
			r0 += 1901.0 / 720.0 * dt * drdt0 + 1387.0 / 360.0 * dt * drdt1 +
			      109.0 / 360.0 * dt * drdt2 + 637.0 / 24.0 * dt * drdt3 +
			      251.0 / 720.0 * dt * drdt4;
	}

	n_steps = (std::min)(n_steps + 1, order);
	t += dt;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

template<unsigned int NSTATE, unsigned int NDERIV>
ImplicitSchemeBase<NSTATE, NDERIV>::ImplicitSchemeBase(moordyn::Log* log,
                                                       WavesRef waves,
                                                       unsigned int iters)
  : SchemeBase<NSTATE, NDERIV>(log, waves)
  , _iters(iters)
  , _c0(0.9)
  , _c1(0.0)
{
}

template<unsigned int NSTATE, unsigned int NDERIV>
real
ImplicitSchemeBase<NSTATE, NDERIV>::Relax(const unsigned int& iter)
{
	const real x = 4. * ((iter + 1.) / _iters - 0.5); // [-1, 1]
	const real y0 = 1. / _iters;                      // (0, 1]
	const real y1 = 0.5 * (tanh(x) + 1.);             // (0, 1)
	return c0() * (1. - y0) + c1() * (1. - y1);
}

ImplicitEulerScheme::ImplicitEulerScheme(moordyn::Log* log,
                                         moordyn::WavesRef waves,
                                         unsigned int iters,
                                         real dt_factor)
  : ImplicitSchemeBase(log, waves, iters)
  , _dt_factor(dt_factor)
{
	stringstream s;
	s << "k=" << dt_factor << " implicit Euler (" << iters << " iterations)";
	name = s.str();
}

void
ImplicitEulerScheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();

	t += _dt_factor * dt;
	drdt1 = drdt0;
	for (unsigned int i = 0; i < iters(); i++) {
		r1 = r0 + _dt_factor * dt * drdt0;
		Update(_dt_factor * dt, 1);
		CalcStateDeriv(0);

		if (i < iters() - 1) {
			// We cannot relax on the last step
			const real relax = Relax(i);
			drdt0 = (1.0 - relax) * drdt0 + relax * drdt1;
			drdt1 = drdt0;
		}
	}

	// Apply
	r0 += dt * drdt0;
	t += (1.0 - _dt_factor) * dt;
	Update(dt, 0);
	ImplicitSchemeBase::Step(dt);
}

ImplicitNewmarkScheme::ImplicitNewmarkScheme(moordyn::Log* log,
                                             moordyn::WavesRef waves,
                                             unsigned int iters,
                                             real gamma,
                                             real beta)
  : ImplicitSchemeBase(log, waves, iters)
  , _gamma(gamma)
  , _beta(beta)
{
	stringstream s;
	s << "gamma=" << gamma << ",beta=" << beta << " implicit Newmark (" << iters
	  << " iterations)";
	name = s.str();
	c0(0.9);
	c1(0.15);
}

void
ImplicitNewmarkScheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();
	auto drdt2 = rd(2)->get();

	// Initialize the velocity and acceleration for the next time step as
	// the ones from the current time step
	drdt1 = drdt0;

	t += dt;
	drdt2 = drdt0;
	for (unsigned int i = 0; i < iters(); i++) {
		MakeNewmark(dt);
		Update(dt, 1);
		CalcStateDeriv(1);

		if (i < iters() - 1) {
			// We cannot relax the last step
			const real relax = Relax(i);
			drdt1 = (1.0 - relax) * drdt1 + relax * drdt2;
			drdt2 = drdt1;
		}
	}

	// Apply
	MakeNewmark(dt);
	r0 = r1;
	drdt0 = drdt1;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

#define MAKE_NEWMARK_VEC3(obj)                                                 \
	{                                                                          \
		auto r0 = r(0)->get(obj);                                              \
		auto r1 = r(1)->get(obj);                                              \
		auto drdt0 = rd(0)->get(obj);                                          \
		auto drdt1 = rd(1)->get(obj);                                          \
		InstanceStateVar acc = (1.0 - _gamma) * drdt0.middleCols<3>(3) +       \
		                       _gamma * drdt1.middleCols<3>(3);                \
		InstanceStateVar acc_beta = (0.5 - _beta) * drdt0.middleCols<3>(3) +   \
		                            _beta * drdt1.middleCols<3>(3);            \
		InstanceStateVar vel = drdt0.leftCols<3>() + dt * acc_beta;            \
		r1.leftCols<3>() = r0.leftCols<3>() + vel * dt;                        \
		r1.middleCols<3>(3) = r0.middleCols<3>(3) + acc * dt;                  \
	}

#define MAKE_NEWMARK_QUAT(obj)                                                 \
	{                                                                          \
		auto r0 = r(0)->get(obj).row(0);                                       \
		auto r1 = r(1)->get(obj).row(0);                                       \
		auto drdt0 = rd(0)->get(obj).row(0);                                   \
		auto drdt1 = rd(1)->get(obj).row(0);                                   \
		const vec6 acc =                                                       \
		    (1.0 - _gamma) * drdt0.tail<6>() + _gamma * drdt1.tail<6>();       \
		const vec6 acc_beta =                                                  \
		    (1.0 - _beta) * drdt0.tail<6>() + _beta * drdt1.tail<6>();         \
		const vec6 vel =                                                       \
		    XYZQuat::fromVec7(drdt0.head<7>()).toVec6() + dt * acc_beta;       \
		const vec7 pos = r0.head<7>();                                         \
		r1.head<7>() = pos + integrateVec6AsVec7(pos, dt * vel);               \
		r1.tail<6>() = vec6(r0.tail<6>()) + acc * dt;                          \
	}

void
ImplicitNewmarkScheme::MakeNewmark(const real& dt)
{
	for (auto obj : lines)
		MAKE_NEWMARK_VEC3(obj);
	for (auto obj : points) {
		if (obj->type != Point::FREE)
			continue;
		MAKE_NEWMARK_VEC3(obj);
	}
	for (auto obj : rods) {
		if ((obj->type != Rod::PINNED) && (obj->type != Rod::CPLDPIN) &&
		    (obj->type != Rod::FREE))
			continue;
		MAKE_NEWMARK_QUAT(obj);
	}
	for (auto obj : bodies) {
		if ((obj->type != Body::FREE) && (obj->type != Body::CPLDPIN))
			continue;
		MAKE_NEWMARK_QUAT(obj);
	}
}

ImplicitWilsonScheme::ImplicitWilsonScheme(moordyn::Log* log,
                                           moordyn::WavesRef waves,
                                           unsigned int iters,
                                           real theta)
  : ImplicitSchemeBase(log, waves, iters)
  , _theta(theta)
{
	stringstream s;
	s << "theta=" << theta << " implicit Wilson (" << iters << " iterations)";
	name = s.str();
	c0(0.015);
	c1(0.000);
}

void
ImplicitWilsonScheme::Step(real& dt)
{
	auto r0 = r(0)->get();
	auto r1 = r(1)->get();
	auto drdt0 = rd(0)->get();
	auto drdt1 = rd(1)->get();

	const real tdt = _theta * dt;
	t += tdt;
	drdt1 = drdt0;
	for (unsigned int i = 0; i < iters(); i++) {
		MakeWilson(tdt, tdt);
		Update(tdt, 1);
		CalcStateDeriv(1);

		if (i < iters() - 1) {
			// We cannot relax on the last step
			const real relax = Relax(i);
			drdt0 = (1.0 - relax) * drdt0 + relax * drdt1;
			drdt1 = drdt0;
		}
	}

	// Apply
	t -= (1.f - _theta) * dt;
	MakeWilson(dt, tdt);
	r0 = r1;
	drdt0 = drdt1;
	Update(dt, 0);
	SchemeBase::Step(dt);
}

#define MAKE_WILSON_VEC3(obj)                                                  \
	{                                                                          \
		auto r0 = r(0)->get(obj);                                              \
		auto r1 = r(1)->get(obj);                                              \
		auto drdt0 = rd(0)->get(obj);                                          \
		auto drdt1 = rd(1)->get(obj);                                          \
		InstanceStateVar acc = (1 - 0.5 * f) * drdt0.middleCols<3>(3) +        \
		                       0.5 * f * drdt1.middleCols<3>(3);               \
		InstanceStateVar acc_tau =                                             \
		    (1 - 1.0 / 3.0 * f) * drdt0.middleCols<3>(3) +                     \
		    1.0 / 3.0 * f * drdt1.middleCols<3>(3);                            \
		InstanceStateVar vel = drdt0.leftCols<3>() + 0.5 * dt * acc_tau;       \
		r1.leftCols<3>() = r0.leftCols<3>() + vel * tau;                       \
		r1.middleCols<3>(3) = r0.middleCols<3>(3) + acc * tau;                 \
	}

#define MAKE_WILSON_QUAT(obj)                                                  \
	{                                                                          \
		auto r0 = r(0)->get(obj).row(0);                                       \
		auto r1 = r(1)->get(obj).row(0);                                       \
		auto drdt0 = rd(0)->get(obj).row(0);                                   \
		auto drdt1 = rd(1)->get(obj).row(0);                                   \
		const vec6 acc =                                                       \
		    (1 - 0.5 * f) * drdt0.tail<6>() + 0.5 * f * drdt1.tail<6>();       \
		const vec6 acc_tau = (1 - 1.0 / 3.0 * f) * drdt0.tail<6>() +           \
		                     1.0 / 3.0 * f * drdt1.tail<6>();                  \
		const vec6 vel =                                                       \
		    XYZQuat::fromVec7(drdt0.head<7>()).toVec6() + 0.5 * dt * acc_tau;  \
		const vec7 pos = r0.head<7>();                                         \
		r1.head<7>() = pos + integrateVec6AsVec7(pos, tau * vel);              \
		r1.tail<6>() = vec6(r0.tail<6>()) + acc * tau;                         \
	}

void
ImplicitWilsonScheme::MakeWilson(const real& tau, const real& dt)
{
	const real f = tau / dt;
	for (auto obj : lines)
		MAKE_WILSON_VEC3(obj);
	for (auto obj : points) {
		if (obj->type != Point::FREE)
			continue;
		MAKE_WILSON_VEC3(obj);
	}
	for (auto obj : rods) {
		if ((obj->type != Rod::PINNED) && (obj->type != Rod::CPLDPIN) &&
		    (obj->type != Rod::FREE))
			continue;
		MAKE_WILSON_QUAT(obj);
	}
	for (auto obj : bodies) {
		if ((obj->type != Body::FREE) && (obj->type != Body::CPLDPIN))
			continue;
		MAKE_WILSON_QUAT(obj);
	}
}

Scheme*
create_time_scheme(const std::string& name,
                   moordyn::Log* log,
                   moordyn::WavesRef waves)
{
	Scheme* out = NULL;
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
	} else if (str::startswith(str::lower(name), "aca")) {
		try {
			unsigned int iters = std::stoi(name.substr(3));
			out = new ImplicitNewmarkScheme(log, waves, iters, 0.5, 0.25);
		} catch (std::invalid_argument) {
			stringstream s;
			s << "Invalid Average Constant Acceleration name format '" << name
			  << "'";
			throw moordyn::invalid_value_error(s.str().c_str());
		}
	} else if (str::startswith(str::lower(name), "wilson")) {
		try {
			unsigned int iters = std::stoi(name.substr(6));
			out = new ImplicitWilsonScheme(log, waves, iters, 1.37);
		} catch (std::invalid_argument) {
			stringstream s;
			s << "Invalid Wilson name format '" << name << "'";
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

} // ::time

} // ::moordyn
