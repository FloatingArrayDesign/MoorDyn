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

#include "State.hpp"

using namespace std;

namespace moordyn {

namespace state {

void
State::addLine(Line* obj)
{
	if (std::find(lines.begin(), lines.end(), obj) != lines.end()) {
		throw moordyn::invalid_value_error("Repeated line");
	}
	lines.push_back(obj);
	resize();
}

unsigned int
State::removeLine(Line* obj)
{
	auto it = std::find(lines.begin(), lines.end(), obj);
	if (it == lines.end()) {
		throw moordyn::invalid_value_error("Missing line");
	}
	const unsigned int i = std::distance(lines.begin(), it);
	lines.erase(it);
	resize();
	return i;
}

void
State::addPoint(Point* obj)
{
	if (std::find(points.begin(), points.end(), obj) != points.end()) {
		throw moordyn::invalid_value_error("Repeated point");
	}
	points.push_back(obj);
	resize();
}

unsigned int
State::removePoint(Point* obj)
{
	auto it = std::find(points.begin(), points.end(), obj);
	if (it == points.end()) {
		throw moordyn::invalid_value_error("Missing point");
	}
	const unsigned int i = std::distance(points.begin(), it);
	points.erase(it);
	resize();
	return i;
}

void
State::addRod(Rod* obj)
{
	if (std::find(rods.begin(), rods.end(), obj) != rods.end()) {
		throw moordyn::invalid_value_error("Repeated rod");
	}
	rods.push_back(obj);
	resize();
}

unsigned int
State::removeRod(Rod* obj)
{
	auto it = std::find(rods.begin(), rods.end(), obj);
	if (it == rods.end()) {
		throw moordyn::invalid_value_error("Missing rod");
	}
	const unsigned int i = std::distance(rods.begin(), it);
	rods.erase(it);
	resize();
	return i;
}

void
State::addBody(Body* obj)
{
	if (std::find(bodies.begin(), bodies.end(), obj) != bodies.end()) {
		throw moordyn::invalid_value_error("Repeated body");
	}
	bodies.push_back(obj);
	resize();
}

unsigned int
State::removeBody(Body* obj)
{
	auto it = std::find(bodies.begin(), bodies.end(), obj);
	if (it == bodies.end()) {
		throw moordyn::invalid_value_error("Missing body");
	}
	const unsigned int i = std::distance(bodies.begin(), it);
	bodies.erase(it);
	resize();
	return i;
}

#define TYPE_GETTER(T, TDEF)                                                   \
template <>                                                                    \
VarBase::types                                                                 \
State::getType<T>()                                                            \
{                                                                              \
	return VarBase::types::TDEF;                                               \
}

TYPE_GETTER(real, REAL)
TYPE_GETTER(VarScalar, REAL)
TYPE_GETTER(vec, VEC)
TYPE_GETTER(VarVec, VEC)
TYPE_GETTER(vec6, VEC6)
TYPE_GETTER(VarVec6, VEC6)
TYPE_GETTER(XYZQuat, QUAT)
TYPE_GETTER(VarQuat, QUAT)
TYPE_GETTER(list, LIST)
TYPE_GETTER(VarList, LIST)

#define STATE_ADDER(T, TBASE)                                                  \
template <>                                                                    \
void                                                                           \
State::addVar<T>(const char* name)                                             \
{                                                                              \
	TBASE* var = new TBASE();                                                  \
	var->resize(ndof());                                                       \
	vars[name] = var;                                                          \
	types[name] = getType<T>();                                                \
}


STATE_ADDER(VarScalar, VarScalar)
STATE_ADDER(real, VarScalar)
STATE_ADDER(VarVec, VarVec)
STATE_ADDER(vec, VarVec)
STATE_ADDER(VarVec6, VarVec6)
STATE_ADDER(vec6, VarVec6)
STATE_ADDER(VarQuat, VarQuat)
STATE_ADDER(XYZQuat, VarQuat)
STATE_ADDER(VarList, VarList)
STATE_ADDER(list, VarList)

void
State::addVar(const char* name, VarBase::types t)
{
	switch (t) {
		case VarBase::types::REAL:
			addVar<VarScalar>(name);
			break;
		case VarBase::types::VEC:
			addVar<VarVec>(name);
			break;
		case VarBase::types::VEC6:
			addVar<VarVec6>(name);
			break;
		case VarBase::types::QUAT:
			addVar<VarQuat>(name);
			break;
		case VarBase::types::LIST:
			addVar<VarList>(name);
			break;
		default:
			throw moordyn::invalid_type_error("Unrecognized variable type");
	}
}

void
State::setListLength(const char* name, size_t n, void* obj)
{
	checkVar<list>(name);
	std::pair<size_t, size_t> ids = obj ?
		indexes[obj] : std::make_pair((size_t)0, ndof());
	for (size_t i = ids.first; i < ids.second; i++) {
		((VarList*)vars[name])->operator()(i).resize(n);
	}
}

#define STATE_SETTER(T, TBASE)                                                 \
template <>                                                                    \
void                                                                           \
State::set<T>(const char* name,                                                \
              Eigen::Matrix<T, Eigen::Dynamic, 1> v)                           \
{                                                                              \
	checkVar<T>(name);                                                         \
	TBASE* var = (TBASE*)vars[name];                                           \
	if(var->rows() != v.rows())                                                \
		throw moordyn::invalid_value_error("Inconsistent lengths");            \
	var->asMatrix() = v;                                                       \
}

STATE_SETTER(real, VarScalar)
STATE_SETTER(vec, VarVec)
STATE_SETTER(vec6, VarVec6)
STATE_SETTER(XYZQuat, VarQuat)
STATE_SETTER(list, VarList)

#define STATE_OBJ_SETTER(T, TBASE)                                             \
template <>                                                                    \
void                                                                           \
State::set<T>(const char* name,                                                \
              void* obj,                                                       \
              Eigen::Matrix<T, Eigen::Dynamic, 1> v)                           \
{                                                                              \
	checkVar<T>(name);                                                         \
	TBASE* var = (TBASE*)vars[name];                                           \
	auto ids = indexes[obj];                                                   \
	if((ids.second - ids.first) != v.rows())                                   \
		throw moordyn::invalid_value_error("Inconsistent lengths");            \
	var->operator()(Eigen::seq(ids.first, ids.second - 1)) = v;                \
}

STATE_OBJ_SETTER(real, VarScalar)
STATE_OBJ_SETTER(vec, VarVec)
STATE_OBJ_SETTER(vec6, VarVec6)
STATE_OBJ_SETTER(XYZQuat, VarQuat)
STATE_OBJ_SETTER(list, VarList)

std::vector<uint64_t>
State::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	for (const auto& [key, var] : vars) {
		switch (var->inner_type()) {
			case VarBase::types::REAL:
				subdata = io::IO::Serialize(((VarScalar*)var)->asMatrix());
				break;
			case VarBase::types::VEC:
				subdata = io::IO::Serialize(((VarVec*)var)->asMatrix());
				break;
			case VarBase::types::VEC6:
				subdata = io::IO::Serialize(((VarVec6*)var)->asMatrix());
				break;
			case VarBase::types::QUAT:
				subdata = io::IO::Serialize(((VarQuat*)var)->asMatrix());
				break;
			case VarBase::types::LIST:
				subdata = io::IO::Serialize(((VarList*)var)->asMatrix());
				break;
			default:
				throw moordyn::invalid_type_error("Unhandled variable type");
		}
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

uint64_t*
State::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	for (const auto& [key, var] : vars) {
		switch (var->inner_type()) {
			case VarBase::types::REAL:
				ptr = io::IO::Deserialize(ptr, ((VarScalar*)var)->asMatrix());
				break;
			case VarBase::types::VEC:
				ptr = io::IO::Deserialize(ptr, ((VarVec*)var)->asMatrix());
				break;
			case VarBase::types::VEC6:
				ptr = io::IO::Deserialize(ptr, ((VarVec6*)var)->asMatrix());
				break;
			case VarBase::types::QUAT:
				ptr = io::IO::Deserialize(ptr, ((VarQuat*)var)->asMatrix());
				break;
			case VarBase::types::LIST:
				ptr = io::IO::Deserialize(ptr, ((VarList*)var)->asMatrix());
				break;
			default:
				throw moordyn::invalid_type_error("Unhandled variable type");
		}
	}
	return ptr;
}

#define STATE_GETTER(T, TBASE)                                                 \
template <>                                                                    \
Eigen::Matrix<T, Eigen::Dynamic, 1>&                                           \
State::getRef<T>(const char* name)                                             \
{                                                                              \
	checkVar<T>(name);                                                         \
	TBASE* var = (TBASE*)vars[name];                                           \
	return var->asMatrix();                                                    \
}

STATE_GETTER(real, VarScalar)
STATE_GETTER(vec, VarVec)
STATE_GETTER(vec6, VarVec6)
STATE_GETTER(XYZQuat, VarQuat)
STATE_GETTER(list, VarList)

void
State::clear()
{
	for (auto& [key, value] : vars) {
		switch (value->inner_type()) {
			case VarBase::types::REAL:
				delete (VarScalar*)value;
				break;
			case VarBase::types::VEC:
				delete (VarVec*)value;
				break;
			case VarBase::types::VEC6:
				delete (VarVec6*)value;
				break;
			case VarBase::types::QUAT:
				delete (VarQuat*)value;
				break;
			case VarBase::types::LIST:
				delete (VarList*)value;
				break;
			default:
				break;
		}
	}
	vars.clear();
	types.clear();
	lines.clear();
	points.clear();
	rods.clear();
	bodies.clear();
	indexes.clear();
}

void
State::copy(const State& rhs)
{
	clear();

	lines.reserve(rhs.lines.size());
	for (auto l : rhs.lines)
		addLine(l);
	points.reserve(rhs.points.size());
	for (auto l : rhs.points)
		addPoint(l);
	rods.reserve(rhs.rods.size());
	for (auto l : rhs.rods)
		addRod(l);
	bodies.reserve(rhs.bodies.size());
	for (auto l : rhs.bodies)
		addBody(l);

	for (const auto& [key, var] : rhs.vars) {
		addVar(key.c_str(), var->inner_type());
		switch (var->inner_type()) {
			case VarBase::types::REAL:
				*((VarScalar*)vars[key]) = *((VarScalar*)var);
				break;
			case VarBase::types::VEC:
				*((VarVec*)vars[key]) = *((VarVec*)var);
				break;
			case VarBase::types::VEC6:
				*((VarVec6*)vars[key]) = *((VarVec6*)var);
				break;
			case VarBase::types::QUAT:
				*((VarQuat*)vars[key]) = *((VarQuat*)var);
				break;
			case VarBase::types::LIST:
				*((VarList*)vars[key]) = *((VarList*)var);
				break;
			default:
				throw moordyn::invalid_type_error("Unhandled variable type");
		}
	}
}

#define STATE_TYPE_CHECKER(T, TDEF)                                            \
template <>                                                                    \
bool                                                                           \
State::checkType<T>(const char* name)                                          \
{                                                                              \
	return types[name] == VarBase::types::TDEF;                                \
}

STATE_TYPE_CHECKER(VarScalar, REAL)
STATE_TYPE_CHECKER(real, REAL)
STATE_TYPE_CHECKER(VarVec, VEC)
STATE_TYPE_CHECKER(vec, VEC)
STATE_TYPE_CHECKER(VarVec6, VEC6)
STATE_TYPE_CHECKER(vec6, VEC6)
STATE_TYPE_CHECKER(VarQuat, QUAT)
STATE_TYPE_CHECKER(XYZQuat, QUAT)
STATE_TYPE_CHECKER(VarList, LIST)
STATE_TYPE_CHECKER(list, LIST)

void State::resize()
{
	auto indexes_old = indexes;
	size_t n_old = 0;
	for (const auto& [key, value] : indexes_old) {
		n_old = value.second > n_old ? value.second : n_old;
	}

	size_t n = ndof();

	if (n > n_old) {
		// A new entity has been added
		std::pair<size_t, size_t> ids;
		for (const auto& [key, value] : indexes) {
			if (indexes_old.find(key) == indexes_old.end()) {
				ids = value;
				break;
			}
		}
		for (const auto& [key, value] : vars) {
			grow(value, n, ids);
		}
	} else {
		// An entity has been removed, find its indexes
		std::pair<size_t, size_t> ids;
		for (const auto& [key, value] : indexes_old) {
			if (indexes.find(key) == indexes.end()) {
				ids = value;
				break;
			}
		}
		for (const auto& [key, value] : vars) {
			shrink(value, n, ids);
		}
	}
}

void State::grow(VarBase* var, size_t n, std::pair<size_t, size_t> ids)
{
	switch (var->inner_type()) {
		case VarBase::types::REAL:
			grow(((VarScalar*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::VEC:
			grow(((VarVec*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::VEC6:
			grow(((VarVec6*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::QUAT:
			grow(((VarQuat*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::LIST:
			grow(((VarList*)var)->asMatrix(), n, ids);
			break;
		default:
			throw moordyn::invalid_type_error("Unrecognized variable type");
	}
}

void State::shrink(VarBase* var, size_t n, std::pair<size_t, size_t> ids)
{
	switch (var->inner_type()) {
		case VarBase::types::REAL:
			shrink(((VarScalar*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::VEC:
			shrink(((VarVec*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::VEC6:
			shrink(((VarVec6*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::QUAT:
			shrink(((VarQuat*)var)->asMatrix(), n, ids);
			break;
		case VarBase::types::LIST:
			shrink(((VarList*)var)->asMatrix(), n, ids);
			break;
		default:
			throw moordyn::invalid_type_error("Unrecognized variable type");
	}
}

} // ::state

} // ::moordyn

moordyn::state::VarListBase operator*(const moordyn::real& k,
                                      moordyn::state::VarListBase v)
{
	for (unsigned int i = 0; i < v.rows(); i++) {
		v(i) *= k;
	}
	return v;
}
