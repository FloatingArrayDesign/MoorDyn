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
State::addInstance(moordyn::Instance* obj)
{
	if (std::find(_objs.begin(), _objs.end(), obj) !=
	    _objs.end()) { // check that this quieried object is not in the _objs
		               // list already
		throw moordyn::invalid_value_error("Repeated instance");
	}
	StateVar new_var(_objs.size() +
	                 1); // make a new Statevar with space for the new instance
	for (size_t i = 0; i < _objs.size(); i++) {
		new_var(i) = _var(i); // copy over _var values to the new array
	}
	InstanceStateVar obj_var(
	    obj->stateN(), obj->stateDims()); // create a N x Dims array of states
	obj_var.setZero();                    // initialize the array to zeros
	new_var(_objs.size()) =
	    obj_var;               // place the array at the end of the _var array
	_var = new_var;            // same as above
	_objs.push_back(obj);      // add the object to the end of the _objs list
	_indexes = make_indexes(); // update indicies
}

unsigned int
State::removeInstance(moordyn::Instance* obj)
{
	auto it = std::find(_objs.begin(), _objs.end(), obj);
	if (it == _objs.end()) {
		throw moordyn::invalid_value_error("Missing instance");
	}
	const unsigned int removed = std::distance(_objs.begin(), it);
	StateVar new_var(_objs.size() - 1);
	for (size_t i = 0; i < removed; i++) {
		new_var(i) = _var(i);
	}
	for (size_t i = removed; i < _objs.size() - 1; i++) {
		new_var(i) = _var(i + 1);
	}
	_var = new_var;
	_objs.erase(it);
	_indexes = make_indexes();
	return removed;
}

std::vector<uint64_t>
State::Serialize(void)
{
	std::vector<uint64_t> data, subdata;
	const uint64_t n = _var.rows();
	data.push_back(io::IO::Serialize(n));
	for (unsigned int i = 0; i < n; i++) {
		subdata = io::IO::Serialize(_var(i));
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

uint64_t*
State::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	uint64_t n;
	ptr = io::IO::Deserialize(ptr, n);
	if (n != _var.rows()) {
		LOGERR << "A state variable with " << n << " instances cannot be "
		       << "deserialized into a state variable with " << _var.rows()
		       << " instances" << std::endl;
		throw moordyn::input_error("Incorrect number of instances");
	}
	for (unsigned int i = 0; i < n; i++)
		ptr = io::IO::Deserialize(ptr, _var(i));
	return ptr;
}

void
State::clear()
{
	_var.resize(0);
	_objs.clear();
	_indexes.clear();
}

void
State::copy(const State& rhs)
{
	clear();

	_objs.reserve(rhs._objs.size());
	for (auto obj : rhs._objs)
		addInstance(obj);

	_var = rhs._var;
}

} // ::state

} // ::moordyn
