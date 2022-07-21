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
#include "Misc.hpp"

namespace moordyn {

std::string
log_level_name(int level)
{
	switch (level) {
		case MOORDYN_DBG_LEVEL:
			return "DBG";
		case MOORDYN_MSG_LEVEL:
			return "MSG";
		case MOORDYN_WRN_LEVEL:
			return "WRN";
		case MOORDYN_ERR_LEVEL:
			return "ERR";
	}
	return "???";
}

/** @brief Null buffer to avoid printing on screen
 * @see moordyn::cnul
 */
class null_out_buf : public std::streambuf
{
  public:
	virtual std::streamsize xsputn(const char PARAM_UNUSED* s,
	                               std::streamsize n)
	{
		return n;
	}
	virtual int overflow(int c) { return c; }
};

/// The buffer to nowhere
null_out_buf __cnul_buff;

/// Stream to nowhere
std::ostream __cnul(&__cnul_buff);

/// Stream to nowhere, used when verbosity is not large enough to print the
/// message
std::ostream& cnul = __cnul;

MultiStream::MultiStream()
  : _fpath("")
  , _fout_enabled(false)
  , _terminal(&cnul)
{}

MultiStream::~MultiStream()
{
	if (_fout.is_open())
		_fout.close();
}

void
MultiStream::SetFile(const char* file_path)
{
	if (_fout.is_open())
		_fout.close();
	_fout.open(file_path);
	if (!_fout.is_open())
		throw moordyn::output_file_error("Invalid file");
	_fpath = file_path;
}

Log::Log(const int verbosity, const int log_file_level)
  : _verbosity(verbosity)
  , _file_verbosity(log_file_level)
  , _streamer(NULL)
{
	_streamer = new MultiStream();
	if (!_streamer)
		throw moordyn::mem_error("Failure allocating the MultiStream");
}

Log::~Log()
{
	delete _streamer;
}

MultiStream&
Log::Cout(const int level) const
{
	if (level < _verbosity)
		_streamer->SetTerminal(moordyn::cnul);
	else if (level >= MOORDYN_ERR_LEVEL)
		_streamer->SetTerminal(std::cerr);
	else
		_streamer->SetTerminal(std::cout);

	if (level < _file_verbosity)
		_streamer->SetFile(false);
	else
		_streamer->SetFile(true);

	return *_streamer;
}

const char*
Log::GetFile() const
{
	return _streamer->GetFile();
}

void
Log::SetFile(const char* file_path)
{
	try {
		_streamer->SetFile(file_path);
	} catch (moordyn::output_file_error& e) {
		// Rethrow the exception to the caller
		throw e;
	}
}

} // ::moordyn
