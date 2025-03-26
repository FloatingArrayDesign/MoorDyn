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
#include "Waves.hpp"
#include <vector>
#include <string>

namespace moordyn {

namespace time {

/** @class Scheme Time.hpp
 * @brief Time scheme abstraction
 *
 * This class is helping mooring::MoorDyn to can consider every single time
 * scheme in the very same way
 */
class Scheme : public io::IO
{
  public:
	/// @brief Destructor
	virtual ~Scheme() {}

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
	 * @note This method is also calling to Scheme::Next()
	 */
	inline void SetTime(const real& time)
	{
		t = time;
		Next();
	}

	/** @brief Get the CFL factor
	 * @return The CFL factor
	 */
	inline real GetCFL() const { return cfl; }

	/** @brief Set the CFL factor
	 * @param cfl The CFL factor
	 */
	inline void SetCFL(const real& cfl) { this->cfl = cfl; }

	/** @brief Prepare everything for the next outer time step
	 *
	 * Always call this method before start calling Scheme::Step()
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
	virtual void Init() = 0;

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme,
	 * but remember to call it at the end of the inherited function to increment
	 * Scheme::t_local
	 * @param dt Time step
	 */
	virtual void Step(real& dt) { t_local += dt; };

	/** @brief Get the state variable
	 * @param i The index of the state variable to take
	 * @return The state variable
	 */
	virtual moordyn::state::State GetState(unsigned int i = 0) = 0;

	/** @brief Resume the simulation from the stationary solution
	 * @param state The stationary solution
	 * @param i The index of the state variable to take
	 */
	inline virtual void SetState(const moordyn::state::State& state,
	                             unsigned int i = 0) {};

	/** @brief Save the system state on a file
	 * @param filepath The output file path
	 * @param i The index of the state variable to serialize
	 * @see ::SerializeState()
	 */
	inline virtual void SaveState(const std::string filepath,
	                              unsigned int i = 0)
	{
	}

	/** @brief Load the system state from a file
	 * @param filepath The output file path
	 * @param i The index of the state variable to overwrite
	 */
	inline virtual void LoadState(const std::string filepath,
	                              unsigned int i = 0)
	{
	}

  protected:
	/** @brief Constructor
	 * @param log Logging handler
	 */
	Scheme(moordyn::Log* log)
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

	/// Maximum CFL factor
	real cfl;
};

/** Definition to explicitely let the compiler know the type inside the
 * std::array.
 *
 * This is required to avoid problems with the templates
 */
#define AS_STATE(R) ((moordyn::state::State*)R)

/** @class SchemeBase Time.hpp
 * @brief A generic abstract integration scheme
 *
 * This class can be later overloaded to implement a plethora of time schemes
 */
template<unsigned int NSTATE, unsigned int NDERIV>
class SchemeBase : public Scheme
{
  public:
	/// @brief Destructor
	virtual ~SchemeBase()
	{
		for (unsigned int substep = 0; substep < NSTATE; substep++) {
			delete _r[substep];
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			delete _rd[substep];
		}
	}

	/** @brief Add a line
	 * @param obj The line
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddLine(Line* obj)
	{
		try {
			Scheme::AddLine(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->addInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->addInstance(obj);
		// Add the mask value
		_calc_mask.lines.push_back(true);
	}

	// virtual void AddLine(Line* obj)
	// {
	// 	try {
	// 		TimeScheme::AddLine(obj);
	// 	} catch (...) {
	// 		throw;
	// 	}
	// 	// Build up the states and states derivatives
	// 	unsigned int n = obj->getN() - 1;
	// 	LineState state;
	// 	LineState mstate; // misc state stuff
	// 	state.pos.assign(n, vec::Zero());
	// 	state.vel.assign(n, vec::Zero());
	// 	mstate.pos.assign(n+2, vec::Zero());
	// 	mstate.vel.assign(n+2, vec::Zero());
	// 	for (unsigned int i = 0; i < r.size(); i++) {
	// 		r[i].lines.push_back(state);
	// 		r[i].misc.push_back(mstate);
	// 	}
	// 	DLineStateDt dstate;
	// 	DLineStateDt mdstate; // misc state stuff
	// 	dstate.vel.assign(n, vec::Zero());
	// 	dstate.acc.assign(n, vec::Zero());
	// 	mdstate.vel.assign(n+2, vec::Zero());
	// 	mdstate.acc.assign(n+2, vec::Zero());
	// 	for (unsigned int i = 0; i < rd.size(); i++) {
	// 		rd[i].lines.push_back(dstate);
	// 		rd[i].misc.push_back(mdstate);
	// 	}
	// 	// Add the mask value
	// 	_calc_mask.lines.push_back(true);
	// }

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
			i = Scheme::RemoveLine(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->removeInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->removeInstance(obj);
		_calc_mask.lines.erase(_calc_mask.lines.begin() + i);
		return i;
	}

	// virtual unsigned int RemoveLine(Line* obj)
	// {
	// 	unsigned int i;
	// 	try {
	// 		i = TimeScheme::RemoveLine(obj);
	// 	} catch (...) {
	// 		throw;
	// 	}
	// 	for (unsigned int i = 0; i < r.size(); i++) {
	// 		r[i].lines.erase(r[i].lines.begin() + i);
	// 		r[i].misc.erase(r[i].misc.begin() + i);
	// 	}
	// 	for (unsigned int i = 0; i < rd.size(); i++) {
	// 		rd[i].lines.erase(rd[i].lines.begin() + i);
	// 		rd[i].misc.erase(rd[i].misc.begin() + i);
	// 	}
	// 	_calc_mask.lines.erase(_calc_mask.lines.begin() + i);
	// 	return i;
	// }

	/** @brief Add a point
	 * @param obj The point
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddPoint(Point* obj)
	{
		try {
			Scheme::AddPoint(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->addInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->addInstance(obj);
		// Add the mask value
		_calc_mask.points.push_back(true);
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
			i = Scheme::RemovePoint(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->removeInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->removeInstance(obj);
		_calc_mask.points.erase(_calc_mask.points.begin() + i);
		return i;
	}

	/** @brief Add a rod
	 * @param obj The rod
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddRod(Rod* obj)
	{
		try {
			Scheme::AddRod(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->addInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->addInstance(obj);
		// Add the mask value
		_calc_mask.rods.push_back(true);
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
			i = Scheme::RemoveRod(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->removeInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->removeInstance(obj);
		_calc_mask.rods.erase(_calc_mask.rods.begin() + i);
		return i;
	}

	/** @brief Add a body
	 * @param obj The body
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	virtual void AddBody(Body* obj)
	{
		try {
			Scheme::AddBody(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->addInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->addInstance(obj);
		// Add the mask value
		_calc_mask.bodies.push_back(true);
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
			i = Scheme::RemoveBody(obj);
		} catch (...) {
			throw;
		}
		for (unsigned int i = 0; i < NSTATE; i++)
			AS_STATE(_r[i])->removeInstance(obj);
		for (unsigned int i = 0; i < NDERIV; i++)
			AS_STATE(_rd[i])->removeInstance(obj);
		_calc_mask.bodies.erase(_calc_mask.bodies.begin() + i);
		return i;
	}

	/** @brief Create an initial state for all the entities
	 * @note Just the first state is written. None of the following states nor
	 * the derivatives are initialized in any way.
	 * @note It is assumed that the coupled entities were already initialized
	 */
	virtual void Init()
	{
		// NOTE: Probably is best to populate all the entities to the time
		// integrator, no matter if they are free or not. Thus they can change
		// types (mutate) without needing to micromanage them in the time
		// scheme
		for (unsigned int i = 0; i < bodies.size(); i++) {
			// Only fully coupled bodies are intialized in MD2.cpp
			if ((bodies[i]->type != Body::FREE) &&
			    (bodies[i]->type != Body::CPLDPIN))
				continue;
			bodies[i]->initialize(AS_STATE(_r[0])->get(bodies[i]));
			for (unsigned int j = 0; j < NDERIV; j++)
				AS_STATE(_rd[j])->get(bodies[i]).setZero();
		}

		for (unsigned int i = 0; i < rods.size(); i++) {
			if ((rods[i]->type != Rod::FREE) && (rods[i]->type != Rod::PINNED))
				continue;
			rods[i]->initialize(AS_STATE(_r[0])->get(rods[i]));
			for (unsigned int j = 0; j < NDERIV; j++)
				AS_STATE(_rd[j])->get(rods[i]).setZero();
		}

		for (unsigned int i = 0; i < points.size(); i++) {
			if (points[i]->type != Point::FREE)
				continue;
			points[i]->initialize(AS_STATE(_r[0])->get(points[i]));
			for (unsigned int j = 0; j < NDERIV; j++)
				AS_STATE(_rd[j])->get(points[i]).setZero();
		}

		for (unsigned int i = 0; i < lines.size(); i++) {
			lines[i]->initialize(AS_STATE(_r[0])->get(lines[i]));
			for (unsigned int j = 0; j < NDERIV; j++)
				AS_STATE(_rd[j])->get(lines[i]).setZero();
		}
	}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme,
	 * but remember to call it at the end of the inherited function to increment
	 * Scheme::t_local
	 * @param dt Time step
	 */
	virtual void Step(real& dt) { Scheme::Step(dt); };

	/** @brief Get the state
	 * @param i The index of the state variable to take
	 * @return The state variable
	 * @throws moordyn::invalid_value_error if the @p i index is greater or
	 * equal than the number of states
	 * @{
	 */
	inline state::State* r(unsigned int i = 0)
	{
		if (i >= NSTATE) {
			LOGERR << "State " << i << " cannot be got on a '" << name
			       << "' scheme that has " << NSTATE << "states" << endl;
			throw moordyn::invalid_value_error("Invalid state");
		}
		return _r[i];
	}

	inline state::State GetState(unsigned int i = 0) { return *r(i); }
	/**
	 * @}
	 */

	/** @brief Resume the simulation from the stationary solution
	 * @param state The stationary solution
	 * @param i The index of the state variable to take
	 * @throws moordyn::invalid_value_error if the @p i index is greater or
	 * equal than the number of states
	 */
	inline virtual void SetState(const state::State& state, unsigned int i = 0)
	{
		if (i >= NSTATE) {
			LOGERR << "State " << i << " cannot be setted on a '" << name
			       << "' scheme that has " << NSTATE << " states" << endl;
			throw moordyn::invalid_value_error("Invalid state");
		}
		*_r[i] = state;
	}

	/** @brief Save the system state on a file
	 * @param filepath The output file path
	 * @param i The index of the state variable to serialize
	 * @throws moordyn::invalid_value_error if the @p i index is greater or
	 * equal than the number of states
	 */
	inline virtual void SaveState(const std::string filepath,
	                              unsigned int i = 0)
	{
		auto f = MakeFile(filepath);
		if (i >= NSTATE) {
			LOGERR << "State " << i << " cannot be saved on a '" << name
			       << "' scheme that has " << NSTATE << "states" << endl;
			throw moordyn::invalid_value_error("Invalid state");
		}
		std::vector<uint64_t> data = AS_STATE(_r[i])->Serialize();
		const uint64_t size = data.size();
		f.write((char*)&size, sizeof(uint64_t));
		for (auto v : data) {
			f.write((char*)&v, sizeof(uint64_t));
		}
		f.close();
	}

	/** @brief Load the system state from a file
	 * @param filepath The output file path
	 * @param i The index of the state variable to overwrite
	 * @throws moordyn::invalid_value_error if the @p i index is greater or
	 * equal than the number of states
	 * @throws moordyn::mem_error if the expected size of the serialized data
	 * is not met
	 */
	inline virtual void LoadState(const std::string filepath,
	                              unsigned int i = 0)
	{
		auto [length, data] = LoadFile(filepath);
		if (i >= NSTATE) {
			LOGERR << "State " << i << " cannot be loaded on a '" << name
			       << "' scheme that has " << NSTATE << "states" << endl;
			throw moordyn::invalid_value_error("Invalid state");
		}
		const uint64_t* end = AS_STATE(_r[i])->Deserialize(data);
		if (data + length != end) {
			const uint64_t l = end - data;
			LOGERR << l * sizeof(uint64_t) << " bytes (vs. "
			       << length * sizeof(uint64_t)
			       << " bytes expected) unpacked from '" << filepath << "'"
			       << endl;
			throw moordyn::mem_error("Mismatching data size");
		}

		free(data);
	}

	/** @brief Get the state derivative
	 * @param i The index of the state derivative to take
	 * @return The state derivative
	 * @throws moordyn::invalid_value_error if the @p i index is greater or
	 * equal than the number of state derivatives
	 */
	inline state::State* rd(unsigned int i = 0)
	{
		if (i >= NDERIV) {
			LOGERR << "State derivative " << i << " cannot be got on a '"
			       << name << "' scheme that has " << NDERIV
			       << "state derivatives" << endl;
			throw moordyn::invalid_value_error("Invalid state derivative");
		}
		return _rd[i];
	}

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
			subdata = _r[substep]->Serialize();
			data.insert(data.end(), subdata.begin(), subdata.end());
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			subdata = _rd[substep]->Serialize();
			data.insert(data.end(), subdata.begin(), subdata.end());
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
			ptr = _r[substep]->Deserialize(ptr);
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			ptr = _rd[substep]->Deserialize(ptr);
		}

		return ptr;
	}

  protected:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves The simulation waves object, needed so that we can tell it
	 * about substeps
	 */
	SchemeBase(moordyn::Log* log, moordyn::WavesRef waves)
	  : Scheme(log)
	  , waves(waves)
	{
		// Register some basic variables that we know are always present
		// The position, and its associated derivative, have several different
		// types depending on the instance (e.g. moordyn::vec for lines and
		// points, moordyn::XYZQuat for rods and bodies), so better using lists
		// that we can resize on demand
		for (unsigned int substep = 0; substep < NSTATE; substep++) {
			_r[substep] = new moordyn::state::State(log);
		}
		for (unsigned int substep = 0; substep < NDERIV; substep++) {
			_rd[substep] = new moordyn::state::State(log);
		}
	}

	/** @brief Update all the entities to set the state
	 * @param substep The index within moordyn::SchemeBase::r that will be
	 * applied to set the states
	 * @param t_local The local time, within the inner time step (from 0 to dt)
	 * @note This is only affecting to the free entities
	 * @see Scheme::Next()
	 * @see Scheme::Step()
	 */
	void Update(real t_local, unsigned int substep = 0);

	/** @brief Compute the time derivatives and store them
	 * @param substep The index within moordyn::SchemeBase::rd where the
	 * info will be saved
	 */
	void CalcStateDeriv(unsigned int substep = 0);

	/// The list of states
	std::array<moordyn::state::State*, NSTATE> _r;

	/// The list of state derivatives
	std::array<moordyn::state::State*, NDERIV> _rd;

	/// The waves instance
	std::shared_ptr<Waves> waves;

	/** @brief A mask to determine which entities shall be computed.
	 *
	 * Useful for local time steps
	 */
	typedef struct _mask
	{
		/// The lines mask
		std::vector<bool> lines;
		/// The points mask
		std::vector<bool> points;
		/// The rods mask
		std::vector<bool> rods;
		/// The bodies mask
		std::vector<bool> bodies;
	} mask;

	/// The SchemeBase::CalcStateDeriv() mask
	mask _calc_mask;
};

/** @class StationaryScheme Time.hpp
 * @brief A stationary solution
 *
 * The stationary solution is featured by the lack of velocity on the system,
 * i.e. the system positions are integrating directly from the accelerations
 */
class StationaryScheme final : public SchemeBase<2, 1>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	StationaryScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~StationaryScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	void Step(real& dt);

	/** @brief Get the error computed at the last time step
	 * @return The error, StationaryScheme::_error
	 */
	inline real Error() const { return _error; }

	/** @brief Compute the number of state variables
	 *
	 * This can be used to renormalize the error, so it makes more sense to the
	 * final user
	 * @return The number of state variables
	 * @note Each entry on the states is considered a single variable, that is
	 * no matter if the state is a scalar, a vector or a quaternion, it is
	 * considered as a single entry
	 */
	inline size_t NStates() const
	{
		size_t n =
		    bodies.size() + rods.size() +
		    points.size(); // one row (state) per object here. <objects>.size()
		                   // gives the length of the list, i.e. the number of
		                   // that kind of object
		for (size_t i = 0; i < lines.size(); i++)
			n += lines[i]->stateN();
		return n;
	}

  private:
	/** @brief Make the state derivative stationary
	 *
	 * Making the derivative stationary means replacing the velocity by half
	 * the acceleration by the time step
	 * @param dt The time step
	 * @param i The index of the state
	 * @param id The index of the state derivative
	 */
	void MakeStationary(real& dt, unsigned int i = 0, unsigned int id = 0);

	/** The last computed acceleration module
	 * @see DMoorDynStateDt::MakeStationary()
	 * @see StationaryScheme::Error()
	 */
	real _error;

	/// The convergence boosting rate
	real _booster;
};

/** @class EulerScheme Time.hpp
 * @brief The simplest 1st order Euler's time scheme
 *
 * This time scheme is strongly discourage, and use only for testing/debugging
 * purposes
 */
class EulerScheme final : public SchemeBase<1, 1>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	EulerScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	virtual ~EulerScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);
};

/** @class LocalSchemeBase Time.hpp
 * @brief A generic abstract integration scheme
 *
 * This class can be later overloaded to implement a plethora of time schemes
 */
template<unsigned int NSTATE, unsigned int NDERIV>
class LocalSchemeBase : public SchemeBase<NSTATE, NDERIV>
{
  public:
	/// @brief Destructor
	virtual ~LocalSchemeBase() {}

	/** @brief Create an initial state for all the entities
	 * @note Just the first state is written. None of the following states, nor
	 * the derivatives are initialized in any way.
	 * @note It is assumed that the coupled entities were already initialized
	 */
	inline void Init()
	{
		SchemeBase<NSTATE, NDERIV>::Init();
		ComputeDt();
	}

	/** @brief Resume the simulation from the stationary solution
	 * @param state The stationary solution
	 * @param i The index of the state variable to take
	 */
	inline virtual void SetState(const state::State& state, unsigned int i = 0)
	{
		SchemeBase<NSTATE, NDERIV>::SetState(state, i);
		ComputeDt();
	}

  protected:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param waves The simulation waves object, needed so that we can tell it
	 * about substeps
	 */
	LocalSchemeBase(moordyn::Log* log, moordyn::WavesRef waves)
	  : SchemeBase<NSTATE, NDERIV>(log, waves)
	{
	}

	/** @brief Set the calculation mask
	 * @param dt Time step
	 */
	void SetCalcMask(real& dt);

  private:
	/** @brief Compute the model time step
	 *
	 * This can be done since we know the TimeScheme::cfl factor
	 * @return The model time step
	 */
	real ComputeDt();

	/** @brief The timestep of each instance
	 */
	typedef struct _sdeltat
	{
		/// The lines mask
		std::vector<real> lines;
		/// The points mask
		std::vector<real> points;
		/// The rods mask
		std::vector<real> rods;
		/// The bodies mask
		std::vector<real> bodies;
	} deltat;

	/// The timestep of each instance
	deltat _dt0;

	/// The counter of already integrated timestep for each instance.
	deltat _dt;
};

/** @class LocalEulerScheme Time.hpp
 * @brief A modification of the 1st order Euler's time scheme, which is
 * considering different time steps for each instance.
 *
 * The local time step of each entity is computed according to the maximum CFL
 * factor of all entities. Such local time step is indeed an integer times the
 * time step provided to LocalEulerScheme::Step().
 *
 * Thus, the derivatives recomputation is delayed until those time steps are
 * fulfilled
 */
class LocalEulerScheme final : public LocalSchemeBase<1, 1>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 */
	LocalEulerScheme(moordyn::Log* log, WavesRef waves);

	/// @brief Destructor
	~LocalEulerScheme() {}

	/** @brief Run a time step
	 * @param dt Time step
	 */
	void Step(real& dt);
};

/** @class HeunScheme Time.hpp
 * @brief Quasi 2nd order Heun's time scheme
 *
 * The Heun's method is a really good compromise between accuracy and
 * performance, since it is only requiring a single derivative per time step,
 * while rendering similar results to other 2nd order schemes
 */
class HeunScheme final : public SchemeBase<1, 2>
{
  public:
	/** @brief Constructor
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
class RK2Scheme final : public SchemeBase<2, 2>
{
  public:
	/** @brief Constructor
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
class RK4Scheme final : public SchemeBase<5, 4>
{
  public:
	/** @brief Constructor
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
template<unsigned int order, bool local>
class ABScheme final : public LocalSchemeBase<1, 5>
{
  public:
	/** @brief Constructor
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
		std::vector<uint64_t> data = SchemeBase::Serialize();
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
		uint64_t* ptr = SchemeBase::Deserialize(data);
		uint64_t n;
		ptr = io::IO::Deserialize(ptr, n);
		n_steps = n;

		return ptr;
	}

  private:
	/// The number of derivatives already available
	unsigned int n_steps;

	/** @brief Shift the derivatives
	 * @param org Origin derivative that will be assigned to the @p org + 1 one
	 */
	inline void shift(unsigned int org)
	{
		const unsigned int dst = org + 1;
		for (unsigned int i = 0; i < lines.size(); i++) {
			if (!_calc_mask.lines[i])
				continue;
			AS_STATE(rd(dst))->get(lines[i]) = AS_STATE(rd(org))->get(lines[i]);
		}

		for (unsigned int i = 0; i < points.size(); i++) {
			if (!_calc_mask.points[i] && (points[i]->type == Point::FREE))
				continue;
			AS_STATE(rd(dst))->get(points[i]) =
			    AS_STATE(rd(org))->get(points[i]);
		}

		for (unsigned int i = 0; i < rods.size(); i++) {
			if (!_calc_mask.rods[i] && ((rods[i]->type != Rod::FREE) ||
			                            (rods[i]->type != Rod::PINNED)))
				continue;
			AS_STATE(rd(dst))->get(rods[i]) = AS_STATE(rd(org))->get(rods[i]);
		}

		for (unsigned int i = 0; i < bodies.size(); i++) {
			if (!_calc_mask.bodies[i] && (bodies[i]->type == Body::FREE))
				continue;
			AS_STATE(rd(dst))->get(bodies[i]) =
			    AS_STATE(rd(org))->get(bodies[i]);
		}
	}

	/** @brief Shift the derivatives
	 */
	inline void shift()
	{
		for (unsigned int i = 0; i < _rd.size() - 1; i++)
			shift(i);
	}
};

/** @class ImplicitSchemeBase Time.hpp
 * @brief A generic abstract implicit scheme
 *
 * This class can be later overloaded to implement a plethora of time schemes
 */
template<unsigned int NSTATE, unsigned int NDERIV>
class ImplicitSchemeBase : public SchemeBase<NSTATE, NDERIV>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 * @param iters The number of inner iterations to find the derivative
	 */
	ImplicitSchemeBase(moordyn::Log* log,
	                   WavesRef waves,
	                   unsigned int iters = 10);

	/// @brief Destructor
	virtual ~ImplicitSchemeBase() {}

  protected:
	/** @brief Get the number of subiterations
	 * @return The number of iterations
	 */
	inline unsigned int iters() const { return _iters; }

	/** @brief Get the constant relaxation part coefficient
	 * @return The constant relaxation part coefficient
	 */
	inline real c0() const { return _c0; }

	/** @brief Set the constant relaxation part coefficient
	 * @param c The constant relaxation part coefficient
	 */
	inline void c0(const real c) { _c0 = c; }

	/** @brief Get the tanh relaxation part coefficient
	 * @return The tanh relaxation part coefficient
	 */
	inline real c1() const { return _c1; }

	/** @brief Set the tanh relaxation part coefficient
	 * @param c The tanh relaxation part coefficient
	 */
	inline void c1(const real c) { _c1 = c; }

	/** @brief Compute the relaxation factor
	 *
	 * This method is responsible of avoiding overshooting when computing the
	 * derivatives
	 * @param iter The current iteration
	 */
	real Relax(const unsigned int& iter);

  private:
	/// The number of iterations
	unsigned int _iters;

	/// The constant relaxation part coefficient
	real _c0;

	/// The tanh relaxation part coefficient
	real _c1;
};

/** @class ImplicitEulerScheme Time.hpp
 * @brief Implicit 1st order Euler time scheme
 *
 * The implicit Euler method is an implicit method where the derivative is
 * evaluated somewhere inside the time step. Obviously, since that point depends
 * on the derivative itself, a fixed point problem shall be solved
 */
class ImplicitEulerScheme final : public ImplicitSchemeBase<2, 2>
{
  public:
	/** @brief Constructor
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
	virtual ~ImplicitEulerScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);

  private:
	/// The evaluation point
	real _dt_factor;
};

/** @class ImplicitNewmarkScheme Time.hpp
 * @brief Implicit Newmark Scheme
 *
 * The implicit Newmark scheme is quite popular because is able to produce
 * unconditionally stable time integrators for dynamic response of structures
 * and solids, specifically on its Average Constant Acceleration incarnation
 * @see https://en.wikipedia.org/wiki/Newmark-beta_method
 */
class ImplicitNewmarkScheme final : public ImplicitSchemeBase<2, 3>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 * @param iters The number of inner iterations to find the derivative
	 * @param gamma The gamma factor
	 * @param beta The beta factor
	 */
	ImplicitNewmarkScheme(moordyn::Log* log,
	                      WavesRef waves,
	                      unsigned int iters = 10,
	                      real gamma = 0.5,
	                      real beta = 0.25);

	/// @brief Destructor
	~ImplicitNewmarkScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);

  private:
	/** @brief Make the gamma,beta-Newmark step
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
	 * ::r(0) and ::rd(0) are the input state and derivative, while ::r(1) is
	 * the output state.
	 * @param dt Time step.
	 */
	void MakeNewmark(const real& dt);

	/// Alpha factor
	real _gamma;
	/// Beta factor
	real _beta;
};

/** @class ImplicitNewmarkScheme Time.hpp
 * @brief Implicit Wilson Scheme
 *
 * The implicit Wilson scheme is so far similar to the Implicit Euler scheme,
 * but the derivatives are computed considering a time step larger than the
 * integration one, instead of lower.
 *
 * With the computed acceleration a Taylor series expansion is practised to
 * integrate.
 *
 * @see
 * https://www.academia.edu/download/59040594/wilson197220190426-49259-kipdfs.pdf
 */
class ImplicitWilsonScheme final : public ImplicitSchemeBase<2, 3>
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param waves Waves instance
	 * @param iters The number of inner iterations to find the derivative
	 * @param gamma The gamma factor
	 * @param beta The beta factor
	 */
	ImplicitWilsonScheme(moordyn::Log* log,
	                     WavesRef waves,
	                     unsigned int iters = 10,
	                     real theta = 1.37);

	/// @brief Destructor
	~ImplicitWilsonScheme() {}

	/** @brief Run a time step
	 *
	 * This function is the one that must be specialized on each time scheme
	 * @param dt Time step
	 */
	virtual void Step(real& dt);

  private:
	/** @brief Make the Wilson step
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
	 * ::r(0) and ::rd(0) are the input state and derivative, while ::r(1) is
	 * the output state.
	 * @param tau Time advancing, \f$ \tau \f$.
	 * @param dt Enlarged time step, \f$ \theta \Delta t \f$.
	 */
	void MakeWilson(const real& tau, const real& dt);

	/// Theta factor
	real _theta;
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
Scheme*
create_time_scheme(const std::string& name, moordyn::Log* log, WavesRef waves);

} // ::time

} // ::moordyn
