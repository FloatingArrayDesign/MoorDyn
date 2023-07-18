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

template<>
string
StateVar<vec>::AsString() const
{
	stringstream s;
	s << "pos = [" << pos.transpose() << "]; ";
	s << "vel = [" << vel.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVar<vec6>::AsString() const
{
	stringstream s;
	s << "pos = [" << pos.transpose() << "]; ";
	s << "vel = [" << vel.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVar<XYZQuat, vec6>::AsString() const
{
	stringstream s;
	s << "pos = [" << pos.toVec7().transpose() << "]; ";
	s << "vel = [" << vel.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVar<std::vector<vec>>::AsString() const
{
	stringstream s;
	s << "pos = [";
	for (auto v : pos)
		s << "[" << v.transpose() << "], ";
	s << "]" << endl;
	s << "vel = [";
	for (auto v : vel)
		s << "[" << v.transpose() << "], ";
	s << "]" << endl;
	return s.str();
}

template<>
StateVar<vec>
StateVar<vec>::operator+(const StateVar<vec>& rhs)
{
	StateVar<vec> out;
	out.pos = pos + rhs.pos;
	out.vel = vel + rhs.vel;
	return out;
}

template<>
StateVar<vec6>
StateVar<vec6>::operator+(const StateVar<vec6>& rhs)
{
	StateVar<vec6> out;
	out.pos = pos + rhs.pos;
	out.vel = vel + rhs.vel;
	return out;
}

template<>
StateVar<XYZQuat, vec6>
StateVar<XYZQuat, vec6>::operator+(const StateVar<XYZQuat, vec6>& rhs)
{
	StateVar<XYZQuat, vec6> out;
	out.pos = pos + rhs.pos;
	out.vel = vel + rhs.vel;
	return out;
}

template<>
StateVar<XYZQuat, vec6>
StateVar<XYZQuat, vec6>::operator-(const StateVar<XYZQuat, vec6>& rhs)
{
	StateVar<XYZQuat, vec6> out;
	out.pos = pos + rhs.pos;
	out.vel = vel + rhs.vel;
	return out;
}

template<>
StateVar<std::vector<vec>>
StateVar<std::vector<vec>>::operator+(const StateVar<std::vector<vec>>& rhs)
{
	if ((pos.size() != rhs.pos.size()) || (vel.size() != rhs.vel.size()))
		throw moordyn::invalid_value_error("Invalid input size");
	StateVar<std::vector<vec>> out;
	out.pos.reserve(pos.size());
	out.vel.reserve(vel.size());
	// NOTE: At this point we are assuming that both pos and vel have the same
	// length
	for (unsigned int i = 0; i < pos.size(); i++) {
		out.pos.push_back(pos[i] + rhs.pos[i]);
		out.vel.push_back(vel[i] + rhs.vel[i]);
	}
	return out;
}

template<>
StateVar<vec>
StateVar<vec>::operator-(const StateVar<vec>& rhs)
{
	StateVar<vec> out;
	out.pos = pos - rhs.pos;
	out.vel = vel - rhs.vel;
	return out;
}

template<>
StateVar<vec6>
StateVar<vec6>::operator-(const StateVar<vec6>& rhs)
{
	StateVar<vec6> out;
	out.pos = pos - rhs.pos;
	out.vel = vel - rhs.vel;
	return out;
}

template<>
StateVar<std::vector<vec>>
StateVar<std::vector<vec>>::operator-(const StateVar<std::vector<vec>>& rhs)
{
	if ((pos.size() != rhs.pos.size()) || (vel.size() != rhs.vel.size()))
		throw moordyn::invalid_value_error("Invalid input size");
	StateVar<std::vector<vec>> out;
	out.pos.reserve(pos.size());
	out.vel.reserve(vel.size());
	// NOTE: At this point we are assuming that both pos and vel have the same
	// length
	for (unsigned int i = 0; i < pos.size(); i++) {
		out.pos.push_back(pos[i] - rhs.pos[i]);
		out.vel.push_back(vel[i] - rhs.vel[i]);
	}
	return out;
}

template<>
string
StateVarDeriv<vec>::AsString() const
{
	stringstream s;
	s << "vel = [" << vel.transpose() << "]; ";
	s << "acc = [" << acc.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVarDeriv<vec6>::AsString() const
{
	stringstream s;
	s << "vel = [" << vel.transpose() << "]; ";
	s << "acc = [" << acc.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVarDeriv<XYZQuat, vec6>::AsString() const
{
	stringstream s;
	s << "vel = [" << vel.toVec7().transpose() << "]; ";
	s << "acc = [" << acc.transpose() << "]" << endl;
	return s.str();
}

template<>
string
StateVarDeriv<std::vector<vec>>::AsString() const
{
	stringstream s;
	s << "vel = [";
	for (auto v : vel)
		s << "[" << v.transpose() << "], ";
	s << "]" << endl;
	s << "acc = [";
	for (auto v : acc)
		s << "[" << v.transpose() << "], ";
	s << "]" << endl;
	return s.str();
}

template<>
StateVar<vec>
StateVarDeriv<vec>::operator*(const real& dt)
{
	StateVar<vec> out;
	out.pos = vel * dt;
	out.vel = acc * dt;
	return out;
}

template<>
StateVar<vec6>
StateVarDeriv<vec6>::operator*(const real& dt)
{
	StateVar<vec6> out;
	out.pos = vel * dt;
	out.vel = acc * dt;
	return out;
}

template<>
StateVar<XYZQuat, vec6>
StateVarDeriv<XYZQuat, vec6>::operator*(const real& dt)
{
	StateVar<XYZQuat, vec6> out;
	out.pos = vel * dt;
	out.vel = acc * dt;
	return out;
}

template<>
StateVar<std::vector<vec>>
StateVarDeriv<std::vector<vec>>::operator*(const real& dt)
{
	StateVar<std::vector<vec>> out;
	out.pos.reserve(vel.size());
	out.vel.reserve(acc.size());
	// NOTE: At this point we are assuming that both vel and acc have the same
	// length
	for (unsigned int i = 0; i < vel.size(); i++) {
		out.pos.push_back(vel[i] * dt);
		out.vel.push_back(acc[i] * dt);
	}
	return out;
}

template<>
StateVarDeriv<vec>
StateVarDeriv<vec>::operator+(const StateVarDeriv<vec>& rhs)
{
	StateVarDeriv<vec> out;
	out.vel = vel + rhs.vel;
	out.acc = acc + rhs.acc;
	return out;
}

template<>
StateVarDeriv<vec6>
StateVarDeriv<vec6>::operator+(const StateVarDeriv<vec6>& rhs)
{
	StateVarDeriv<vec6> out;
	out.vel = vel + rhs.vel;
	out.acc = acc + rhs.acc;
	return out;
}

template<>
StateVarDeriv<XYZQuat, vec6>
StateVarDeriv<XYZQuat, vec6>::operator+(const StateVarDeriv<XYZQuat, vec6>& rhs)
{
	StateVarDeriv<XYZQuat, vec6> out;
	out.vel = vel + rhs.vel;
	out.acc = acc + rhs.acc;
	return out;
}

template<>
StateVarDeriv<std::vector<vec>>
StateVarDeriv<std::vector<vec>>::operator+(
    const StateVarDeriv<std::vector<vec>>& rhs)
{
	if ((vel.size() != rhs.vel.size()) || (acc.size() != rhs.acc.size()))
		throw moordyn::invalid_value_error("Invalid input size");
	StateVarDeriv<std::vector<vec>> out;
	out.vel.reserve(vel.size());
	out.acc.reserve(acc.size());
	// NOTE: At this point we are assuming that both vel and acc have the same
	// length
	for (unsigned int i = 0; i < vel.size(); i++) {
		out.vel.push_back(vel[i] + rhs.vel[i]);
		out.acc.push_back(acc[i] + rhs.acc[i]);
	}
	return out;
}

template<>
StateVarDeriv<vec>
StateVarDeriv<vec>::operator-(const StateVarDeriv<vec>& rhs)
{
	StateVarDeriv<vec> out;
	out.vel = vel - rhs.vel;
	out.acc = acc - rhs.acc;
	return out;
}

template<>
StateVarDeriv<vec6>
StateVarDeriv<vec6>::operator-(const StateVarDeriv<vec6>& rhs)
{
	StateVarDeriv<vec6> out;
	out.vel = vel - rhs.vel;
	out.acc = acc - rhs.acc;
	return out;
}

template<>
StateVarDeriv<XYZQuat, vec6>
StateVarDeriv<XYZQuat, vec6>::operator-(const StateVarDeriv<XYZQuat, vec6>& rhs)
{
	StateVarDeriv<XYZQuat, vec6> out;
	out.vel = vel - rhs.vel;
	out.acc = acc - rhs.acc;
	return out;
}

template<>
StateVarDeriv<std::vector<vec>>
StateVarDeriv<std::vector<vec>>::operator-(
    const StateVarDeriv<std::vector<vec>>& rhs)
{
	if ((vel.size() != rhs.vel.size()) || (acc.size() != rhs.acc.size()))
		throw moordyn::invalid_value_error("Invalid input size");
	StateVarDeriv<std::vector<vec>> out;
	out.vel.reserve(vel.size());
	out.acc.reserve(acc.size());
	// NOTE: At this point we are assuming that both vel and acc have the same
	// length
	for (unsigned int i = 0; i < vel.size(); i++) {
		out.vel.push_back(vel[i] - rhs.vel[i]);
		out.acc.push_back(acc[i] - rhs.acc[i]);
	}
	return out;
}

string
MoorDynState::AsString() const
{
	stringstream s;
	for (unsigned int i = 0; i < lines.size(); i++) {
		s << "Line " << i << ":" << endl;
		s << lines[i].AsString();
	}
	for (unsigned int i = 0; i < points.size(); i++) {
		s << "Point " << i << ":" << endl;
		s << points[i].AsString();
	}
	for (unsigned int i = 0; i < rods.size(); i++) {
		s << "Rod " << i << ":" << endl;
		s << rods[i].AsString();
	}
	for (unsigned int i = 0; i < bodies.size(); i++) {
		s << "Body " << i << ":" << endl;
		s << bodies[i].AsString();
	}
	s << endl;
	return s.str();
}

MoorDynState&
MoorDynState::operator=(const MoorDynState& rhs)
{
	lines.clear();
	lines.reserve(rhs.lines.size());
	for (auto l : rhs.lines)
		lines.push_back(l);
	points.clear();
	points.reserve(rhs.points.size());
	for (auto l : rhs.points)
		points.push_back(l);
	rods.clear();
	rods.reserve(rhs.rods.size());
	for (auto l : rhs.rods)
		rods.push_back(l);
	bodies.clear();
	bodies.reserve(rhs.bodies.size());
	for (auto l : rhs.bodies)
		bodies.push_back(l);

	return *this;
}

MoorDynState
MoorDynState::operator+(const MoorDynState& rhs)
{
	MoorDynState out;

	if (lines.size() != rhs.lines.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.lines.reserve(lines.size());
	for (unsigned int i = 0; i < lines.size(); i++)
		out.lines.push_back(lines[i] + rhs.lines[i]);
	if (points.size() != rhs.points.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.points.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
		out.points.push_back(points[i] + rhs.points[i]);
	if (rods.size() != rhs.rods.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.rods.reserve(rods.size());
	for (unsigned int i = 0; i < rods.size(); i++)
		out.rods.push_back(rods[i] + rhs.rods[i]);
	if (bodies.size() != rhs.bodies.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.bodies.reserve(bodies.size());
	for (unsigned int i = 0; i < bodies.size(); i++)
		out.bodies.push_back(bodies[i] + rhs.bodies[i]);

	return out;
}

MoorDynState
MoorDynState::operator-(const MoorDynState& rhs)
{
	MoorDynState out;

	if (lines.size() != rhs.lines.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.lines.reserve(lines.size());
	for (unsigned int i = 0; i < lines.size(); i++)
		out.lines.push_back(lines[i] - rhs.lines[i]);
	if (points.size() != rhs.points.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.points.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
		out.points.push_back(points[i] - rhs.points[i]);
	if (rods.size() != rhs.rods.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.rods.reserve(rods.size());
	for (unsigned int i = 0; i < rods.size(); i++)
		out.rods.push_back(rods[i] - rhs.rods[i]);
	if (bodies.size() != rhs.bodies.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.bodies.reserve(bodies.size());
	for (unsigned int i = 0; i < bodies.size(); i++)
		out.bodies.push_back(bodies[i] - rhs.bodies[i]);

	return out;
}

string
DMoorDynStateDt::AsString() const
{
	stringstream s;
	for (unsigned int i = 0; i < lines.size(); i++) {
		s << "Line " << i << ":" << endl;
		s << lines[i].AsString();
	}
	for (unsigned int i = 0; i < points.size(); i++) {
		s << "Point " << i << ":" << endl;
		s << points[i].AsString();
	}
	for (unsigned int i = 0; i < rods.size(); i++) {
		s << "Rod " << i << ":" << endl;
		s << rods[i].AsString();
	}
	for (unsigned int i = 0; i < bodies.size(); i++) {
		s << "Body " << i << ":" << endl;
		s << bodies[i].AsString();
	}
	s << endl;
	return s.str();
}

DMoorDynStateDt&
DMoorDynStateDt::operator=(const DMoorDynStateDt& rhs)
{
	lines.clear();
	lines.reserve(rhs.lines.size());
	for (auto l : rhs.lines)
		lines.push_back(l);
	points.clear();
	points.reserve(rhs.points.size());
	for (auto l : rhs.points)
		points.push_back(l);
	rods.clear();
	rods.reserve(rhs.rods.size());
	for (auto l : rhs.rods)
		rods.push_back(l);
	bodies.clear();
	bodies.reserve(rhs.bodies.size());
	for (auto l : rhs.bodies)
		bodies.push_back(l);

	return *this;
}

MoorDynState
DMoorDynStateDt::operator*(const real& dt)
{
	MoorDynState out;

	out.lines.reserve(lines.size());
	for (unsigned int i = 0; i < lines.size(); i++)
		out.lines.push_back(lines[i] * dt);
	out.points.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
		out.points.push_back(points[i] * dt);
	out.rods.reserve(rods.size());
	for (unsigned int i = 0; i < rods.size(); i++)
		out.rods.push_back(rods[i] * dt);
	out.bodies.reserve(bodies.size());
	for (unsigned int i = 0; i < bodies.size(); i++)
		out.bodies.push_back(bodies[i] * dt);

	return out;
}

DMoorDynStateDt
DMoorDynStateDt::operator+(const DMoorDynStateDt& rhs)
{
	DMoorDynStateDt out;

	if (lines.size() != rhs.lines.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.lines.reserve(lines.size());
	for (unsigned int i = 0; i < lines.size(); i++)
		out.lines.push_back(lines[i] + rhs.lines[i]);
	if (points.size() != rhs.points.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.points.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
		out.points.push_back(points[i] + rhs.points[i]);
	if (rods.size() != rhs.rods.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.rods.reserve(rods.size());
	for (unsigned int i = 0; i < rods.size(); i++)
		out.rods.push_back(rods[i] + rhs.rods[i]);
	if (bodies.size() != rhs.bodies.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.bodies.reserve(bodies.size());
	for (unsigned int i = 0; i < bodies.size(); i++)
		out.bodies.push_back(bodies[i] + rhs.bodies[i]);

	return out;
}

DMoorDynStateDt
DMoorDynStateDt::operator-(const DMoorDynStateDt& rhs)
{
	DMoorDynStateDt out;

	if (lines.size() != rhs.lines.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.lines.reserve(lines.size());
	for (unsigned int i = 0; i < lines.size(); i++)
		out.lines.push_back(lines[i] - rhs.lines[i]);
	if (points.size() != rhs.points.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.points.reserve(points.size());
	for (unsigned int i = 0; i < points.size(); i++)
		out.points.push_back(points[i] - rhs.points[i]);
	if (rods.size() != rhs.rods.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.rods.reserve(rods.size());
	for (unsigned int i = 0; i < rods.size(); i++)
		out.rods.push_back(rods[i] - rhs.rods[i]);
	if (bodies.size() != rhs.bodies.size())
		throw moordyn::invalid_value_error("Invalid input size");
	out.bodies.reserve(bodies.size());
	for (unsigned int i = 0; i < bodies.size(); i++)
		out.bodies.push_back(bodies[i] - rhs.bodies[i]);

	return out;
}

} // ::moordyn
