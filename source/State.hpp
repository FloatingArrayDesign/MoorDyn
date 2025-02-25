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
#include "Line.hpp"
#include "IO.hpp"
#include "Point.hpp"
#include "Rod.hpp"
#include "Body.hpp"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <utility>
#include <sstream>

namespace moordyn {

namespace state {

class State;

#define STATE_VAR_BLOCK(T)                                                     \
	Eigen::VectorBlock<Eigen::Matrix<T, Eigen::Dynamic, 1>, Eigen::Dynamic>

/// Abstract base class for all State Variables
class VarBase {
  public:
	/** @brief Destructor
	 */
	virtual ~VarBase() = default;

	/** @brief Considered valid types
	 */
	typedef enum
	{
		/// real type
		REAL,
		/// vec type
		VEC,
		/// vec6 type
		VEC6,
		/// XYZQuat type
		QUAT,
		/// list type
		LIST,
	} types;

	/// Get the type definition
	inline const VarBase::types inner_type() const { return this->_type; }

	// friend class declaration
	friend class State;

  protected:
	/// The type name
	VarBase::types _type;
};

/// Typed state variable
template <typename T>
class VarTyped : public VarBase, public Eigen::Matrix<T, Eigen::Dynamic, 1>
{
public:
	/** @brief Destructor
	 */
	virtual ~VarTyped() = default;

	/** @brief Get a reference of the state variable as an Eigen::Matrix
	 * @return A reference of the matrix
	 */
	template <typename Tout=T>
	inline Eigen::Matrix<Tout, Eigen::Dynamic, 1>& asMatrix() const
	{
		Eigen::Matrix<T, Eigen::Dynamic, 1>* m =
			(Eigen::Matrix<T, Eigen::Dynamic, 1>*)this;
		return *m;
	}
  protected:
	/** @brief Constructor
	 * @param t Type definition, to be used later
	 */
	VarTyped(const VarBase::types t)
	{
		this->_type = t;
	}
};

/// Scalar state basic Eigen type
typedef Eigen::Matrix<real, Eigen::Dynamic, 1> VarScalarBase;

/// Scalar state variable
class VarScalar final : public VarTyped<real>
{
  public:
	VarScalar()
		: VarTyped(VarBase::types::REAL)
	{}
};

/// 3-D vector state basic Eigen type
typedef Eigen::Matrix<vec, Eigen::Dynamic, 1> VarVecBase;

/// 3-D vector state variable
class VarVec final : public VarTyped<vec>
{
  public:
	VarVec()
		: VarTyped(VarBase::types::VEC)
	{}
};

/// 6-D vector state basic Eigen type
typedef Eigen::Matrix<vec6, Eigen::Dynamic, 1> VarVec6Base;

/// 6-D vector state variable
class VarVec6 final : public VarTyped<vec6>
{
  public:
	VarVec6()
		: VarTyped(VarBase::types::VEC6)
	{}
};

/// Quaternion state basic Eigen type
typedef Eigen::Matrix<XYZQuat, Eigen::Dynamic, 1> VarQuatBase;

/// Quaternion state variable
class VarQuat final : public VarTyped<XYZQuat>
{
  public:
	VarQuat()
		: VarTyped(VarBase::types::QUAT)
	{}
};

/// List state basic Eigen type
typedef Eigen::Matrix<list, Eigen::Dynamic, 1> VarListBase;

/// List state variable
class VarList final : public VarTyped<list>
{
  public:
	VarList()
		: VarTyped(VarBase::types::LIST)
	{}

	~VarList()
	{
		for (unsigned int i = 0; i < rows(); i++) {
			this->operator()(i).resize(0);
		}
	}
};

/// State var slicer
typedef Eigen::ArithmeticSequence<Eigen::Index, Eigen::Index, Eigen::internal::FixedInt<1>> Slicer;

/// State var indexer, useful for objects that only has an entry on the var
typedef Eigen::Index Indexer;

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

	/** @brief Add a line
	 * @param obj The line
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	void addLine(Line* obj);

	/** @brief Remove a line
	 * @param obj The line
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the line has not been registered,
	 * or it was already removed
	 */
	unsigned int removeLine(Line* obj);

	/** @brief Add a point
	 * @param obj The point
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	void addPoint(Point* obj);

	/** @brief Remove a point
	 * @param obj The point
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the point has not been
	 * registered, or it was already removed
	 */
	unsigned int removePoint(Point* obj);

	/** @brief Add a rod
	 * @param obj The rod
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	void addRod(Rod* obj);

	/** @brief Remove a rod
	 * @param obj The rod
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the rod has not been registered,
	 * or it was already removed
	 */
	unsigned int removeRod(Rod* obj);

	/** @brief Add a body
	 * @param obj The body
	 * @throw moordyn::invalid_value_error If it has been already registered
	 */
	void addBody(Body* obj);

	/** @brief Remove a body
	 * @param obj The body
	 * @return The index of the removed entity
	 * @throw moordyn::invalid_value_error If the body has not been registered,
	 * or it was already removed
	 */
	unsigned int removeBody(Body* obj);

	/** @brief Installs a new variable
	 * @param name Name of the variable
	 */
	template <typename T>
	void addVar(const char* name);

	/** @brief Installs a new variable
	 * @param name Name of the variable
	 * @param t The type of variable
	 */
	void addVar(const char* name, VarBase::types t);

	/** @brief Check if a variable already exists
	 * @param name Name of the variable
	 * @return true if the variable already exists, false otherwise
	 */
	inline bool hasVar(const char* name) const
	{
		return !(vars.find(name) == vars.end());
	}

	/** @brief Set a list length
	 *
	 * The lists might have a different number of components per instance
	 * (moordyn::Line / moordyn::Point / moordyn::Rod / moordyn::Body)
	 * @param name Name of the variable
	 * @param n Number of components
	 * @param obj The instance, NULL if the whole list should be resized
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not
	 * moordyn::State::VarList
	 * @throw moordyn::invalid_value_error If the instance does not exist
	 */
	void setListLength(const char* name, size_t n=1, void* obj=NULL);

	/** @brief Get a variable by its name
	 * @param name Name of the variable
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 * @throw moordyn::invalid_value_error If the instance does not exist
	 */
	template <typename T>
	inline STATE_VAR_BLOCK(T)
	get(const char* name)
	{
		return getRef<T>(name)(
			Eigen::seq(0, Eigen::placeholders::last));
	}

	/** @brief Get a variable by its name and instance
	 * @param name Name of the variable
	 * @param obj The instance, i.e. A moordyn::Line, moordyn::Point,
	 * moordyn::Rod or moordyn::Body
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 * @throw moordyn::invalid_value_error If the instance does not exist
	 */
	template <typename T>
	inline STATE_VAR_BLOCK(T)
	get(const char* name, void* obj)
	{
		auto ids = indexes[obj];
		auto n = ids.second - ids.first;
		return getRef<T>(name)(Eigen::seq(ids.first, ids.second - 1));
	}

	/** @brief Set a variable
	 * @param name Name of the variable
	 * @param v The new value
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 * @{
	 */
	template <typename T>
	void set(const char* name, Eigen::Matrix<T, Eigen::Dynamic, 1> v);

	/** @brief Set the part of a variable associated with an instance
	 * @param name Name of the variable
	 * @param obj The instance, i.e. A moordyn::Line, moordyn::Point,
	 * moordyn::Rod or moordyn::Body
	 * @param v The new value
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 * @throw moordyn::invalid_value_error If the instance does not exist
	 */
	template <typename T>
	void set(const char* name,
	         void* obj,
	         Eigen::Matrix<T, Eigen::Dynamic, 1> v);

	/** @brief Get a slicer for a state var
	 *
	 * It is rather inneficient to call ::get() many times, so if multiple
	 * operations have to be done on multiple instances, it is way more
	 * efficient to call ::get() just once per state var, and use the slicers
	 * from this method
	 * @param obj The objects
	 * @return The slicers
	 * @{
	 */
	inline const Slicer slicer(void* obj)
	{
		auto ids = indexes[obj];
		return Eigen::seq(ids.first, ids.second - 1);
	}

	inline const Indexer indexer(void* obj)
	{
		return Indexer(indexes[obj].first);
	}

	inline const std::vector<Slicer> slicer(std::vector<void*> obj)
	{
		std::vector<Slicer> slcs;
		for (auto o : obj) {
			slcs.push_back(slicer(o));
		}
		return slcs;
	}

	inline const std::vector<Indexer> indexer(std::vector<void*> obj)
	{
		std::vector<Indexer> slcs;
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
	/** @brief Get a variable by its name
	 * @param name Name of the variable
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 */
	template <typename T>
	Eigen::Matrix<T, Eigen::Dynamic, 1>& getRef(const char* name);

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
	inline size_t ndof() {
		size_t i = 0;
		for (auto line : lines) {
			size_t n = line->getN() - 1;
			indexes[line] = {i, i + n};
			i += n;
		}
		for (auto point : points) {
			indexes[point] = {i, i + 1};
			i++;
		}
		for (auto rod : rods) {
			indexes[rod] = {i, i + 1};
			i++;
		}
		for (auto body : bodies) {
			indexes[body] = {i, i + 1};
			i++;
		}
		return i;
	}

	/** @brief Check the type
	 *
	 * This function is assuming that the variable existance has been already
	 * checked
	 * @param name Name of the variable
	 * @return true if the type is correct, false otherwise
	 */
	template <typename T>
	bool checkType(const char* name);

	/** @brief Check the var
	 * @param name Name of the variable
	 * @throw moordyn::invalid_value_error If the variable does not exist
	 * @throw moordyn::invalid_type_error If the variable type is not correct
	 */
	template <typename T>
	inline void checkVar(const char* name)
	{
		if (vars.find(name) == vars.end())
			throw moordyn::invalid_value_error("Undefined variable");
		if (!checkType<T>(name))
			throw moordyn::invalid_type_error("Invalid variable type");
	}

	/** Get the type enum
	 * @return The type
	 */
	template <typename T>
	static VarBase::types getType();

	/** @brief Resize the state variables to take into account changes on the
	 * number of instances
	 */
	void resize();

	/** @brief Grow a variable
	 * @param var Variable
	 * @param n New variable size
	 * @param ids The new inserted indexes
	 * @{
	 */
	void grow(VarBase* var, size_t n, std::pair<size_t, size_t> ids);
	template <typename T>
	inline void grow(Eigen::Matrix<T, Eigen::Dynamic, 1>& var,
	                 size_t n,
	                 std::pair<size_t, size_t> ids)
	{
		const size_t first = ids.first;
		const size_t last = ids.second;
		const size_t offset = last - first;
		Eigen::Matrix<T, Eigen::Dynamic, 1> m = var;
		var.resize(n);
		for (size_t i = 0; i < first; i++) {
			var(i) = m(i);
		}
		for (size_t i = last; i < n; i++) {
			var(i) = m(i - offset);
		}
	}
	/**
	 * @}
	 */

	/** @brief Shrink a variable
	 * @param var Variable
	 * @param n New variable size
	 * @param ids The removed indexes
	 * @{
	 */
	void shrink(VarBase* var, size_t n, std::pair<size_t, size_t> ids);
	template <typename T>
	inline void shrink(Eigen::Matrix<T, Eigen::Dynamic, 1>& var,
	                   size_t n,
	                   std::pair<size_t, size_t> ids)
	{
		const size_t first = ids.first;
		const size_t last = ids.second;
		const size_t offset = last - first;
		Eigen::Matrix<T, Eigen::Dynamic, 1> m = var;
		var.resize(n);
		for (size_t i = 0; i < first; i++) {
			var(i) = m(i);
		}
		for (size_t i = last; i < n; i++) {
			var(i - offset) = m(i);
		}
	}
	/**
	 * @}
	 */

	/// The map of available variables
	std::map<std::string, VarBase*> vars;

	/// The map of available variable types
	std::map<std::string, VarBase::types> types;

	/// The lines
	std::vector<Line*> lines;

	/// The points
	std::vector<Point*> points;

	/// The rods
	std::vector<Rod*> rods;

	/// The bodies
	std::vector<Body*> bodies;

	/// The map that associate each instance with the variable indexes
	std::map<void*, std::pair<size_t, size_t>> indexes;
};

} // ::state

} // ::moordyn

moordyn::state::VarListBase operator*(const moordyn::real& k,
                                      moordyn::state::VarListBase v);
