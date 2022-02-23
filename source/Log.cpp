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

 // This is version 2.a5, 2021-03-16
 
#include "Log.hpp"

namespace moordyn
{

/** @brief Null buffer to avoid printing on screen
* @see moordyn::cnul
*/
class null_out_buf : public std::streambuf
{
public:
	virtual std::streamsize xsputn(const char PARAM_UNUSED *s, std::streamsize n)
	{
		return n;
	}
	virtual int overflow (int c)
	{
		return c;
	}
};

/// The buffer to nowhere
null_out_buf __cnul_buff;

/// Stream to nowhere
std::ostream __cnul(&__cnul_buff);

/// Stream to nowhere, used when verbosity is not large enough to print the
/// message
std::ostream& cnul = __cnul;


Log::Log(const int verbosity)
	: _verbosity(verbosity)
{
}


std::ostream& Log::Cout(const int level) const
{
	if (level < _verbosity)
		return moordyn::cnul;
	if (level >= MOORDYN_ERR_LEVEL)
		return std::cerr;
	return std::cout;
}

}  // ::moordyn
