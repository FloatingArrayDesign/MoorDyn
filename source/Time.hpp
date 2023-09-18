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

/** @file Time.hpp
 * C++ API for the time integration
 */

#pragma once

#include "Misc.hpp"
#include "IO.hpp"
#include "State.hpp"
#include "Line.hpp"
#include "Point.hpp"
#include "Rod.hpp"
#include "Body.hpp"
#include <vector>
#include <string>

namespace moordyn {

/** @class TimeScheme Time.hpp
 * @brief Time scheme abstraction
 *
 * This class is helping mooring::MoorDyn to can consider every single time
 * scheme in the very same way
 */
class TimeScheme : public io::IO
{
  public:
	/// @brief Destructor
	virtual ~TimeScheme() {}

	/** @brief Set the ground body
	 * @param obj The ground body
	 */
	inline void SetGround(Body* obj) { ground = obj; }

	/** @brief Add a line
	 * @param obj The line
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddLine(Line* obj)
	{
		if (std::find(lines.begin(), lines.end(), obj) != lines.end()) {
			LOGERR << "The line " << obj->number << " was already registered"
			       << endl;
			throw moordyn::invalid_value_error("Repeated object");
		}
		lines.push_back(obj);
	}

	/** @brief Remove a line
	 * @param obj The line
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the line has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveLine(Line* obj)
	{
		auto it = std::find(lines.begin(), lines.end(), obj);
		if (it == lines.end()) {
			LOGERR << "The line " << obj->number << " was not registered"
			       << endl;
			throw moordyn::invalid_value_error("Missing object");
		}
		const unsigned int i = std::distance(lines.begin(), it);
		lines.erase(it);
		return i;
	}

	/** @brief Add a point
	 * @param obj The point
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddPoint(Point* obj)
	{
		if (std::find(points.begin(), points.end(), obj) != points.end()) {
			LOGERR << "The point " << obj->number << " was already registered"
			       << endl;
			throw moordyn::invalid_value_error("Repeated object");
		}
		points.push_back(obj);
	}

	/** @brief Remove a point
	 * @param obj The point
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the point has not been
	 * registered, or it was already removed
	 */
	virtual unsigned int RemovePoint(Point* obj)
	{
		auto it = std::find(points.begin(), points.end(), obj);
		if (it == points.end()) {
			LOGERR << "The point " << obj->number << " was not registered"
			       << endl;
			throw moordyn::invalid_value_error("Missing object");
		}
		const unsigned int i = std::distance(points.begin(), it);
		points.erase(it);
		return i;
	}

	/** @brief Add a rod
	 * @param obj The rod
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddRod(Rod* obj)
	{
		if (std::find(rods.begin(), rods.end(), obj) != rods.end()) {
			LOGERR << "The rod " << obj->number << " was already registered"
			       << endl;
			throw moordyn::invalid_value_error("Repeated object");
		}
		rods.push_back(obj);
	}

	/** @brief Remove a rod
	 * @param obj The rod
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the rod has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveRod(Rod* obj)
	{
		auto it = std::find(rods.begin(), rods.end(), obj);
		if (it == rods.end()) {
			LOGERR << "The rod " << obj->number << " was not registered"
			       << endl;
			throw moordyn::invalid_value_error("Missing object");
		}
		const unsigned int i = std::distance(rods.begin(), it);
		rods.erase(it);
		return i;
	}

	/** @brief Add a body
	 * @param obj The body
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddBody(Body* obj)
	{
		if (std::find(bodies.begin(), bodies.end(), obj) != bodies.end()) {
			LOGERR << "The body " << obj->number << " was already registered"
			       << endl;
			throw moordyn::invalid_value_error("Repeated object");
		}
		bodies.push_back(obj);
	}

	/** @brief Remove a body
	 * @param obj The body
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the body has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveBody(Body* obj)
	{
		auto it = std::find(bodies.begin(), bodies.end(), obj);
		if (it == bodies.end()) {
			LOGERR << "The body " << obj->number << " was not registered"
			       << endl;
			throw moordyn::invalid_value_error("Missing object");
		}
		const unsigned int i = std::distance(bodies.begin(), it);
		bodies.erase(it);
		return i;
	}

	/** @brief Get the name of the scheme
	 * @return The name
	 */
	inline std::string GetName() const { return name; }

	/** @brief Get the simulation time
	 * @return The time
	 */
	inline real GetTime() const { return t; }

	/** @brief Set the simulation time
	 * @param time The time
	 * @note This method is also calling to TimeScheme::Next()
	 */
	inline void SetTime(const real& time)
	{
		t = time;
		Next();
	}

	/** @brief Prepare everything for the next outer time step
	 *
	 * Always call this method before start calling TimeScheme::Step()
	 */
	inline void Next()
	{
		t_local = 0.0;
		for (unsigned int i = 0; i < lines.size(); i++) {
			lines[i]->updateUnstretchedLength();
		}
	}

	/** @brief Create an initial state for all the entities
	 * @note Just the first state is written. None of the following states, nor
	 * the derivatives are initialized in any way.
	 * @note It is assumed that the coupled entities were already initialized
	 */
	virtual void init() = 0;

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme,
	 * but remember to call it at the end of the inherited function to increment
	 * TimeScheme::t_local
	 * @param dt Time step
	 */
	virtual void Step(real& dt) { t_local += dt; };

  protected:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	TimeScheme(moordyn::Log* log)
	  : io::IO(log)
	  , name("None")
	  , t(0.0)
	{
	}

	/// The ground body
	Body* ground;

	/// The lines
	std::vector<Line*> lines;

	/// The points
	std::vector<Point*> points;

	/// The rods
	std::vector<Rod*> rods;

	/// The bodies
	std::vector<Body*> bodies;

	/// The scheme name
	std::string name;

	/// The simulation time
	real t;
	/// The local time, within the outer time step
	real t_local;
};

// Forward declare waves
class Waves;
typedef std::shared_ptr<Waves> WavesRef;

/** @class TimeSchemeBase Time.hpp
 * @brief A generic abstract integration scheme
 *
 * This class can be later overloaded to implement a plethora of time schemes
 */
template<unsigned int NSTATE, unsigned int NDERIV>
class TimeSchemeBase : public TimeScheme
{
  public:
	/// @brief Destructor
	virtual ~TimeSchemeBase() {}

	/** @brief Add a line
	 * @param obj The line
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddLine(Line* obj)
	{
		try {
			TimeScheme::AddLine(obj);
		} catch (...) {
			throw;
		}
		// Build up the states and states derivatives
		unsigned int n = obj->getN() - 1;
		LineState state;
		state.pos.assign(n, vec::Zero());
		state.vel.assign(n, vec::Zero());
		for (unsigned int i = 0; i < r.size(); i++) {
			r[i].lines.push_back(state);
		}
		DLineStateDt dstate;
		dstate.vel.assign(n, vec::Zero());
		dstate.acc.assign(n, vec::Zero());
		for (unsigned int i = 0; i < rd.size(); i++) {
			rd[i].lines.push_back(dstate);
		}
	}

	/** @brief Remove a line
	 * @param obj The line
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the line has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveLine(Line* obj)
	{
		unsigned int i;
		try {
			i = TimeScheme::RemoveLine(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < r.size(); i++)
			r[i].lines.erase(r[i].lines.begin() + i);
		for (unsigned int i = 0; i < rd.size(); i++)
			rd[i].lines.erase(rd[i].lines.begin() + i);
		return i;
	}

	/** @brief Add a point
	 * @param obj The point
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddPoint(Point* obj)
	{
		try {
			TimeScheme::AddPoint(obj);
		} catch (...) {
			throw;
		}
		// Build up the states and states derivatives
		PointState state;
		state.pos = vec::Zero();
		state.vel = vec::Zero();
		for (unsigned int i = 0; i < r.size(); i++) {
			r[i].points.push_back(state);
		}
		DPointStateDt dstate;
		dstate.vel = vec::Zero();
		dstate.acc = vec::Zero();
		for (unsigned int i = 0; i < rd.size(); i++) {
			rd[i].points.push_back(dstate);
		}
	}

	/** @brief Remove a point
	 * @param obj The point
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the point has not been
	 * registered, or it was already removed
	 */
	virtual unsigned int RemovePoint(Point* obj)
	{
		unsigned int i;
		try {
			i = TimeScheme::RemovePoint(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < r.size(); i++)
			r[i].points.erase(r[i].points.begin() + i);
		for (unsigned int i = 0; i < rd.size(); i++)
			rd[i].points.erase(rd[i].points.begin() + i);
		return i;
	}

	/** @brief Add a rod
	 * @param obj The rod
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddRod(Rod* obj)
	{
		try {
			TimeScheme::AddRod(obj);
		} catch (...) {
			throw;
		}
		// Build up the states and states derivatives
		RodState state;
		state.pos = XYZQuat::Zero();
		state.vel = vec6::Zero();
		for (unsigned int i = 0; i < r.size(); i++) {
			r[i].rods.push_back(state);
		}
		DRodStateDt dstate;
		dstate.vel = XYZQuat::Zero();
		dstate.acc = vec6::Zero();
		for (unsigned int i = 0; i < rd.size(); i++) {
			rd[i].rods.push_back(dstate);
		}
	}

	/** @brief Remove a rod
	 * @param obj The rod
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the rod has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveRod(Rod* obj)
	{
		unsigned int i;
		try {
			i = TimeScheme::RemoveRod(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < r.size(); i++)
			r[i].rods.erase(r[i].rods.begin() + i);
		for (unsigned int i = 0; i < rd.size(); i++)
			rd[i].rods.erase(rd[i].rods.begin() + i);
		return i;
	}

	/** @brief Add a body
	 * @param obj The body
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddBody(Body* obj)
	{
		try {
			TimeScheme::AddBody(obj);
		} catch (...) {
			throw;
		}
		// Build up the states and states derivatives
		BodyState state;
		state.pos = XYZQuat::Zero();
		state.vel = vec6::Zero();
		for (unsigned int i = 0; i < r.size(); i++) {
			r[i].bodies.push_back(state);
		}
		DBodyStateDt dstate;
		dstate.vel = XYZQuat::Zero();
		dstate.acc = vec6::Zero();
		for (unsigned int i = 0; i < rd.size(); i++) {
			rd[i].bodies.push_back(dstate);
		}
	}

	/** @brief Remove a body
	 * @param obj The body
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the body has not been registered,
	 * or it was already removed
	 */
	virtual unsigned int RemoveBody(Body* obj)
	{
		unsigned int i;
		try {
			i = TimeScheme::RemoveBody(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < r.size(); i++)
			r[i].bodies.erase(r[i].bodies.begin() + i);
		for (unsigned int i = 0; i < rd.size(); i++)
			rd[i].bodies.erase(rd[i].bodies.begin() + i);
		return i;
	}

	/** @brief Create an initial state for all the entities
	 * @note Just the first state is written. None of the following states, nor
	 * the derivatives are initialized in any way.
	 * @note It is assumed that the coupled entities were already initialized
	 */
	virtual void init()
	{
		// NOTE: Probably is best to populate all the entities to the time
		// integrator, no matter if they are free or not. Thus they can change
		// types (mutate) without needing to micromanage them in the time scheme
		for (unsigned int i = 0; i < bodies.size(); i++) {
			if (bodies[i]->type != Body::FREE)
				continue;
			std::tie(r[0].bodies[i].pos, r[0].bodies[i].vel) =
			    bodies[i]->initialize();
		}

		for (unsigned int i = 0; i < rods.size(); i++) {
			if ((rods[i]->type != Rod::FREE) && (rods[i]->type != Rod::PINNED))
				continue;
			std::tie(r[0].rods[i].pos, r[0].rods[i].vel) =
			    rods[i]->initialize();
		}

		for (unsigned int i = 0; i < points.size(); i++) {
			if (points[i]->type != Point::FREE)
				continue;
			std::tie(r[0].points[i].pos, r[0].points[i].vel) =
			    points[i]->initialize();
		}

		for (unsigned int i = 0; i < lines.size(); i++) {
			std::tie(r[0].lines[i].pos, r[0].lines[i].vel) =
			    lines[i]->initialize();
		}
	}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme,
	 * but remember to call it at the end of the inherited function to increment
	 * TimeScheme::t_local
	 * @param dt Time step
	 */
	virtual void Step(real& dt) { TimeScheme::Step(dt); };

	/** @brief Produce the packed data to be saved
	 *
	 * The produced data can be used afterwards to restore the saved information
	 * afterwards calling Deserialize(void).
	 * @return The packed data
	 */
	virtual std::vector<uint64_t> Serialize(void)
	{
		std::vector<uint64_t> data, subdata;

		data.push_back(io::IO::Serialize(t));

		// We do not need to save the number of states or derivatives, since
		// that information is already known by each specific time scheme.
		// Along the same line, we do not need to same information about the
		// number of lines, rods and so on. That information is already
		// collected from the definition file
		for (unsigned int substep = 0; substep < NSTATE; substep++) {
			for (unsigned int i = 0; i < bodies.size(); i++) {
				subdata = io::IO::Serialize(r[substep].bodies[i].pos);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(r[substep].bodies[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < rods.size(); i++) {
				subdata = io::IO::Serialize(r[substep].rods[i].pos);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(r[substep].rods[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < points.size(); i++) {
				subdata = io::IO::Serialize(r[substep].points[i].pos);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(r[substep].points[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < lines.size(); i++) {
				subdata = io::IO::Serialize(r[substep].lines[i].pos);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(r[substep].lines[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			for (unsigned int i = 0; i < bodies.size(); i++) {
				subdata = io::IO::Serialize(rd[substep].bodies[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(rd[substep].bodies[i].acc);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < rods.size(); i++) {
				subdata = io::IO::Serialize(rd[substep].rods[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(rd[substep].rods[i].acc);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < points.size(); i++) {
				subdata = io::IO::Serialize(rd[substep].points[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(rd[substep].points[i].acc);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
			for (unsigned int i = 0; i < lines.size(); i++) {
				subdata = io::IO::Serialize(rd[substep].lines[i].vel);
				data.insert(data.end(), subdata.begin(), subdata.end());
				subdata = io::IO::Serialize(rd[substep].lines[i].acc);
				data.insert(data.end(), subdata.begin(), subdata.end());
			}
		}

		return data;
	}

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the function that each inherited class must implement, and should
	 * be the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	virtual uint64_t* Deserialize(const uint64_t* data)
	{
		uint64_t* ptr = (uint64_t*)data;
		ptr = io::IO::Deserialize(ptr, t);

		// We did not save the number of states or derivatives, since that
		// information is already known by each specific time scheme.
		// Along the same line, we did not save information about the number of
		// lines, rods and so on
		for (unsigned int substep = 0; substep < NSTATE; substep++) {
			for (unsigned int i = 0; i < bodies.size(); i++) {
				ptr = io::IO::Deserialize(ptr, r[substep].bodies[i].pos);
				ptr = io::IO::Deserialize(ptr, r[substep].bodies[i].vel);
			}
			for (unsigned int i = 0; i < rods.size(); i++) {
				ptr = io::IO::Deserialize(ptr, r[substep].rods[i].pos);
				ptr = io::IO::Deserialize(ptr, r[substep].rods[i].vel);
			}
			for (unsigned int i = 0; i < points.size(); i++) {
				ptr = io::IO::Deserialize(ptr, r[substep].points[i].pos);
				ptr = io::IO::Deserialize(ptr, r[substep].points[i].vel);
			}
			for (unsigned int i = 0; i < lines.size(); i++) {
				ptr = io::IO::Deserialize(ptr, r[substep].lines[i].pos);
				ptr = io::IO::Deserialize(ptr, r[substep].lines[i].vel);
			}
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			for (unsigned int i = 0; i < bodies.size(); i++) {
				ptr = io::IO::Deserialize(ptr, rd[substep].bodies[i].vel);
				ptr = io::IO::Deserialize(ptr, rd[substep].bodies[i].acc);
			}
			for (unsigned int i = 0; i < rods.size(); i++) {
				ptr = io::IO::Deserialize(ptr, rd[substep].rods[i].vel);
				ptr = io::IO::Deserialize(ptr, rd[substep].rods[i].acc);
			}
			for (unsigned int i = 0; i < points.size(); i++) {
				ptr = io::IO::Deserialize(ptr, rd[substep].points[i].vel);
				ptr = io::IO::Deserialize(ptr, rd[substep].points[i].acc);
			}
			for (unsigned int i = 0; i < lines.size(); i++) {
				ptr = io::IO::Deserialize(ptr, rd[substep].lines[i].vel);
				ptr = io::IO::Deserialize(ptr, rd[substep].lines[i].acc);
			}
		}

		return ptr;
	}

  protected:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves The simulation waves object, needed so that we can tell it
	 * about substeps
	 */
	TimeSchemeBase(moordyn::Log* log, moordyn::WavesRef waves)
	  : TimeScheme(log)
	  , waves(waves)
	{
	}

	/** @brief Update all the entities to set the state
	 * @param substep The index within moordyn::TimeSchemeBase::r that will be
	 * applied to set the states
	 * @param t_local The local time, within the inner time step (from 0 to dt)
	 * @note This is only affecting to the free entities
	 * @see TimeScheme::Next()
	 * @see TimeScheme::Step()
	 */
	void Update(real t_local, unsigned int substep = 0);

	/** @brief Compute the time derivatives and store them
	 * @param substep The index within moordyn::TimeSchemeBase::rd where the
	 * info will be saved
	 */
	void CalcStateDeriv(unsigned int substep = 0);

	/// The list of states
	std::array<MoorDynState, NSTATE> r;

	/// The list of state derivatives
	std::array<DMoorDynStateDt, NDERIV> rd;

	std::shared_ptr<Waves> waves;
};

/** @class EulerScheme Time.hpp
 * @brief The simplest 1st order Euler's time scheme
 *
 * This time scheme is strongly discourage, and use only for testing/debugging
 * puposes
 */
class EulerScheme : public TimeSchemeBase<1, 1>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	EulerScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~EulerScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);
};

/** @class HeunScheme Time.hpp
 * @brief Quasi 2nd order Heun's time scheme
 *
 * The Heun's method is a really good compromise between accuracy and
 * performance, since it is only requiring a single derivative per time step,
 * while rendering similar results to other 2nd order schemes
 */
class HeunScheme : public TimeSchemeBase<1, 2>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	HeunScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~HeunScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);
};

/** @class RK2Scheme Time.hpp
 * @brief 2nd order Runge-Kutta time scheme
 *
 * This was the traditionally applied time scheme in MoorDyn v1
 */
class RK2Scheme : public TimeSchemeBase<2, 2>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	RK2Scheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~RK2Scheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);
};

/** @class RK4Scheme Time.hpp
 * @brief 4th order Runge-Kutta time scheme
 *
 * A very popular time integration scheme, it is however computationally
 * expensive, requiring 4 computations of the derivative per time step. On the
 * other hand, it might be possible to increase the time step size, compensating
 * such a drawback
 */
class RK4Scheme : public TimeSchemeBase<5, 4>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	RK4Scheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~RK4Scheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);
};

/** @class ABScheme Time.hpp
 * @brief Adam-Bashforth time schemes collection
 *
 * The Adam-Bashforth method tries to increase the order of the integrator by
 * using former derivatives. Thus it is requiring just a single derivative
 * computation per time step
 *
 * Actually, the 1st order and the 2nd order are the same schemes than the
 * Euler's and Heun's ones
 */
template<unsigned int order>
class ABScheme : public TimeSchemeBase<5, 1>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	ABScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~ABScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);

	/** @brief Produce the packed data to be saved
	 *
	 * The produced data can be used afterwards to restore the saved information
	 * afterwards calling Deserialize(void).
	 * @return The packed data
	 */
	virtual std::vector<uint64_t> Serialize(void)
	{
		std::vector<uint64_t> data = TimeSchemeBase::Serialize();
		// We append the number of available steps
		data.push_back(io::IO::Serialize((uint64_t)n_steps));

		return data;
	}

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the function that each inherited class must implement, and should
	 * be the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	virtual uint64_t* Deserialize(const uint64_t* data)
	{
		uint64_t* ptr = TimeSchemeBase::Deserialize(data);
		uint64_t n;
		ptr = io::IO::Deserialize(ptr, n);
		n_steps = n;

		return ptr;
	}

  private:
	/// The number of derivatives already available
	unsigned int n_steps;

	/** @brief Shift the derivatives
	 */
	inline void shift()
	{
		for (unsigned int i = 0; i < rd.size() - 1; i++)
			rd[i + 1] = rd[i];
	}
};

/** @class ImplicitEulerScheme Time.hpp
 * @brief Implicit 1st order Euler time scheme
 *
 * The implicit Euler method is an implicit method where the derivative is
 * evaluated somewhere inside the time step. Obviously, since that point depends
 * on the derivative itself, a fixed point problem shall be solved
 */
class ImplicitEulerScheme : public TimeSchemeBase<2, 1>
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 * @param iters The number of inner iterations to find the derivative
	 * @param dt_factor The inner evaluation point factor. 0.5 for the midpoint
	 * method, 1.0 for the backward Euler method
	 */
	ImplicitEulerScheme(moordyn::Log* log,
	                    WavesRef waves,
	                    unsigned int iters = 10,
	                    real dt_factor = 0.5);

	/// @brief Destructor
	~ImplicitEulerScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);

  private:
	/// The number of iterations
	unsigned int _iters;
	/// The evaluation point
	real _dt_factor;
};

/** @brief Create a time scheme
 * @param name The time scheme name, one of the following:
 * "Euler", "Heun", "RK2", "RK4", "AB3", "AB4"
 * @param log The log handler
 * @param waves Waves instance
 * @return The time scheme
 * @throw moordyn::invalid_value_error If there is not a time scheme named after
 * @p name
 * @throw moordyn::mem_error If the time scheme memory cannot be allocated
 * @note Remember to delete the returned time scheme at some point
 */
TimeScheme*
create_time_scheme(const std::string& name, moordyn::Log* log, WavesRef waves);

} // ::moordyn
