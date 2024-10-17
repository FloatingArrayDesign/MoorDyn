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

/** @file Log.hpp
 * Utilities to log info on screen. 2 main classes are provided, moordyn::Log
 * which is the logging handler itself, and moordyn::LogUser which shall be
 * inherited by all the objects which are intended to use the logging handler
 */

#pragma once

#include "MoorDynAPI.h"
#include <iostream>
#include <fstream>

namespace moordyn {

/** @brief Name the log level
 * @param level The log level
 * @see @ref moordyn_log
 */
std::string
log_level_name(int level);

/** @brief Utility to log messages
 *
 * This macro is assuming you have access to the log handler as the _log
 * variable. The easiest way to grant you can safety use this macro is
 * inheriting the LogUser class
 */
#define LOGGER(level)                                                          \
	_log->Cout(level) << log_level_name(level) << " " << __FILE__ << ":"       \
	                  << __LINE__ << " " << __FUNC_NAME__ << "(): "

/// Log a debug message, without extra info about the source code
#define LOGDBG _log->Cout(MOORDYN_DBG_LEVEL)
/// Log an info message, without extra info about the source code
#define LOGMSG _log->Cout(MOORDYN_MSG_LEVEL)
/// Log a warning
#define LOGWRN LOGGER(MOORDYN_WRN_LEVEL)
/// Log an error
#define LOGERR LOGGER(MOORDYN_ERR_LEVEL)

class MultiStream;

/** @brief Streamer able to redirect the output to several substreams
 *
 * This class is actually meant as a helper for moordyn::Log
 * @see Log
 */
class MultiStream
{
  public:
	/// Constructor
	MultiStream();
	/// Destructor
	~MultiStream();

	/** @brief Get the output file path
	 * @return The output file path, an empty string if no file has been set
	 */
	inline const char* GetFile() const { return _fpath.c_str(); }

	/** @brief Set the output file path
	 *
	 * This method will immediately try to open such file for writing.
	 * @param file_path The output file path
	 * @throws moordyn::output_file_error If the output file cannot be
	 * opened/created at @p file_path
	 */
	void SetFile(const char* file_path);

	/** @brief Enable/disable the file printing
	 * @param enable true to enable the file printing, false to disable it
	 */
	inline void SetFile(bool enable = true) { _fout_enabled = enable; };

	/** @brief Set the terminal streamer
	 * @param stream The terminal stream
	 */
	inline void SetTerminal(std::ostream& stream) { _terminal = &stream; };

	/** @brief Functionality for std::endl alike operators
	 */
	MultiStream& operator<<(std::ostream& (*pfun)(std::ostream&))
	{
		if (_fout_enabled && _fout.is_open())
			pfun(_fout);
		pfun(*_terminal);
		return *this;
	}

	/// The file path
	std::string _fpath;
	/// The output file streamer
	std::ofstream _fout;
	/// Flag to know if the file output is enabled
	bool _fout_enabled;
	/// The terminal active streamer
	std::ostream* _terminal;
};

/** @brief Streaming to the log file and the terminal
 */
template<class T>
MultiStream&
operator<<(MultiStream& st, T val)
{
	if (st._fout_enabled && st._fout.is_open())
		st._fout << val;
	*(st._terminal) << val;
	return st;
};

/** @class Log
 * @brief A Logging utility
 *
 * This class is a sensible replacement for std::cout and std::cerr ofstream
 * entities, providing the appropriate streaming channel to each entity
 */
class Log
{
  public:
	/** @brief Constructor
	 * @param verbosity The verbosity level (see @ref moordyn_log)
	 * @param log_file_level The same than @p verbosity, but for the log file
	 * (if any is open with SetFile(). It is disabled by default)
	 * @throws moordyn::mem_error If the inner streamer cannot be built
	 */
	Log(const int verbosity = MOORDYN_MSG_LEVEL,
	    const int log_file_level = MOORDYN_DBG_LEVEL);

	/** @brief Destructor
	 */
	~Log();

	/** @brief Get a stream to log data
	 *
	 * Whether the message is logged, and where, depends on the verbosity level
	 *
	 * @param level Message level
	 */
	MultiStream& Cout(const int level = MOORDYN_MSG_LEVEL) const;

	/** @brief Get the verbosity level
	 * @return The verbosity level (see @ref moordyn_log)
	 */
	inline int GetVerbosity() const { return _verbosity; }

	/** @brief Set the verbosity level
	 * @param verbosity The verbosity level (see @ref moordyn_log)
	 */
	inline void SetVerbosity(const int verbosity) { _verbosity = verbosity; }

	/** @brief Get the log file printing level
	 * @return The log file printing level (see @ref moordyn_log)
	 */
	inline int GetLogLevel() const { return _file_verbosity; }

	/** @brief Set the log file printing level
	 * @param level The log file printing level (see @ref moordyn_log)
	 */
	inline void SetLogLevel(const int level) { _file_verbosity = level; }

	/** @brief Get the log file path
	 * @return The log file path, an empty string if no log file is considered
	 */
	const char* GetFile() const;

	/** @brief Set the log file path
	 *
	 * This method will immediately try to open such file for writing.
	 * @param file_path The log file path
	 * @note To disable logging after open a file  with this method it is better
	 * to use SetLogLevel()
	 * @throws moordyn::output_file_error If the output file cannot be
	 * opened/created at @p file_path
	 */
	void SetFile(const char* file_path);

  private:
	/// Terminal verbosity level
	int _verbosity;
	/// Log file verbosity level
	int _file_verbosity;
	/// The streamer which might redirects to both the terminal and a file
	MultiStream* _streamer;
};

/** @brief A helper for the entities to use the logger
 *
 * Inheriting this class you can grant the class will have everything required
 * to use the macros LOGGER, LOGDBG, LOGMSG, LOGWRN and LOGERR
 */
class LogUser
{
  public:
	/** @brief Constructor
	 * @param log The log handler. NULL can be passed, providing later the log
	 * handler with SetLogger()
	 * @warning No messages shall be logged until a non NULL log handler is
	 * provided
	 */
	LogUser(Log* log = NULL)
	  : _log(log)
	{
	}

	/** @brief Destructor
	 */
	~LogUser() {}

	/** @brief Set the log handler
	 * @param log The log handler
	 */
	inline void SetLogger(Log* log) { _log = log; }

	/** @brief Get the log handler
	 * @return The log handler
	 */
	inline Log* GetLogger() const { return _log; }

  protected:
	/// The log handler
	Log* _log;
};

} // ::moordyn
