/*
 * Copyright (c) 2022, Matt Hall
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

// This is version 2.a5, 2021-03-16

#include "Misc.hpp"
#include "Log.hpp"

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
{
}

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
