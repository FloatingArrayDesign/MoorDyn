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
#include "IO.hpp"
#include "Instance.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <utility>
#include <sstream>

namespace moordyn {

namespace state {

/** @class State State.hpp
 * @brief The collection of state variables of the whole system
 */
class DECLDIR State final : public moordyn::io::IO
{
  public:
	/** @brief Constructor
	 * @param log The logger
	 */
	State(moordyn::Log* log)
	  : moordyn::io::IO(log)
	{
	}

	/** @brief Copy constructor
	 * @param rhs State to copy
	 */
	State(const State& rhs)
	  : moordyn::io::IO(rhs._log)
	{
		copy(rhs);
	}

	/** @brief Destructor
	 */
	~State() { clear(); };

	/** @brief Add an instance capable of holding state variables
	 * @param obj The instance
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	void addInstance(moordyn::Instance* obj);

	/** @brief Remove an instance
	 * @param obj The instance
	 * @throw moordyn::invalid_value_error If the instance has not been
	 * registered, or it was already removed
	 */
	unsigned int removeInstance(moordyn::Instance* obj);

	/** @brief Get the state variables
	 */
	inline StateVarView get()
	{
		return _var(Eigen::seq(0, Eigen::placeholders::last));
	}

	/** @brief Get the state variables associated to an instance
	 * @param obj The instance
	 * @throw moordyn::invalid_value_error If the instance has not been
	 * registered, or it was already removed
	 */
	inline InstanceStateVarView get(moordyn::Instance* obj)
	{
		const auto i = indexer(obj);
		return _var(i).topLeftCorner(obj->stateN(), obj->stateDims());
	}

	/** @brief Get the index within a state var for a particular instance
	 * @param obj The objects
	 * @return The indexes
	 * @throw moordyn::invalid_value_error If the instance has not been
	 * registered, or it was already removed
	 * @{
	 */
	inline const Eigen::Index indexer(moordyn::Instance* obj)
	{
		const size_t id = obj->id();
		if ((id >= _indexes.size()) || (_indexes[id] < 0)) {
			throw moordyn::invalid_value_error("Missing instance");
		}
		return Eigen::Index(_indexes[id]);
	}

	inline const std::vector<Eigen::Index> indexer(
	    std::vector<moordyn::Instance*> obj)
	{
		std::vector<Eigen::Index> slcs;
		slcs.reserve(obj.size());
		for (auto o : obj) {
			slcs.push_back(indexer(o));
		}
		return slcs;
	}

	/**
	 * @}
	 */

	/** @brief Produce the packed data to be saved
	 *
	 * The produced data can be used afterwards to restore the saved
	 * information calling Deserialize(void).
	 *
	 * Thus, this function is not processing the information that is extracted
	 * from the definition file
	 * @return The packed data
	 */
	std::vector<uint64_t> Serialize(void);

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	uint64_t* Deserialize(const uint64_t* data);

	/** @brief Assignment operator
	 * @param rhs The entity to copy
	 */
	inline State& operator=(const State& rhs)
	{
		copy(rhs);
		return *this;
	}

  private:
	/** @brief Clear the state
	 */
	void clear();

	/** @brief State copy
	 *
	 * This function is a wrapper for the assignment operator and the copy
	 * constructor
	 * @param visitor The entity to copy
	 */
	void copy(const State& visitor);

	/** @brief Configure the number of dofs from the added instances
	 *
	 * This method builds the moordyn::State::indexes map
	 * @return the Total number of dofs
	 */
	inline std::vector<int> make_indexes()
	{
		std::vector<int> indexes;
		for (size_t i = 0; i < _objs.size(); i++) {
			size_t key = _objs[i]->id();
			if (indexes.size() <= key) {
				indexes.resize(key + 1, -1);
			}
			indexes[key] = i;
		}
		return indexes;
	}

	/// The state var
	StateVar _var;

	/// The instances
	std::vector<moordyn::Instance*> _objs;

	/// A link between the instances unique ids and the index into the var
	std::vector<int> _indexes;
};

} // ::state

} // ::moordyn

inline moordyn::StateVar
operator*(moordyn::StateVarView v, moordyn::real f)
{
	moordyn::StateVar out;
	out.resize(v.rows());
	for (unsigned int i = 0; i < v.rows(); i++) {
		out(i).resize(v(i).rows(), v(i).cols());
		out(i) = v(i) * f;
	}
	return out;
}

inline moordyn::StateVar
operator*(moordyn::real f, moordyn::StateVarView v)
{
	return v * f;
}

inline moordyn::StateVar
operator*(moordyn::StateVar v, moordyn::real f)
{
	moordyn::StateVar out;
	out.resize(v.rows());
	for (unsigned int i = 0; i < v.rows(); i++) {
		out(i).resize(v(i).rows(), v(i).cols());
		out(i) = v(i) * f;
	}
	return out;
}

inline moordyn::StateVar
operator*(moordyn::real f, moordyn::StateVar v)
{
	return v.topLeftCorner(v.rows(), v.cols()) * f;
}
