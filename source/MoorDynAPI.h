/*
 * Copyright (c) 2014 Matt Hall <mtjhall@alumni.uvic.ca>
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

#ifndef __MOORDYNAPI_H__
#define __MOORDYNAPI_H__

#ifdef MoorDyn_EXPORTS     // this is set as a preprocessor definition!!!
	#ifndef LINUX
		#define DECLDIR __declspec(dllexport)
	#else
		#define DECLDIR 
	#endif
#else
	#define DECLDIR //__declspec(dllimport)
#endif

#if (__GNUC__ > 2) || (__GNUC__ == 2 && __GNUC_MINOR__ > 6)
#define PARAM_UNUSED __attribute__((__unused__))
#else
#define PARAM_UNUSED
#endif

#ifndef __FUNC_NAME__
#ifdef WIN32
#define __FUNC_NAME__   __FUNCTION__  
#else
#define __FUNC_NAME__   __func__ 
#endif
#endif

#ifndef XSTR
#define XSTR(s) STR(s)
#endif
#ifndef STR
#define STR(s) #s
#endif

/** \addtogroup moordyn_log
 *  @{
 */

/// Error message
#define MOORDYN_ERR_LEVEL 3
/// Warning message
#define MOORDYN_WRN_LEVEL 2
/// Info message
#define MOORDYN_MSG_LEVEL 1
/// Debug message
#define MOORDYN_DBG_LEVEL 0

/**
 * @}
 */

/** \addtogroup moordyn_errors
 *  @{
 */

/// Successfully dispatched task
#define MOORDYN_SUCCESS 0
/// Invalid input file path
#define MOORDYN_INVALID_INPUT_FILE -1
/// Invalid output file path
#define MOORDYN_INVALID_OUTPUT_FILE -2
/// Invalid input in the input file
#define MOORDYN_INVALID_INPUT -3
/// NaN detected
#define MOORDYN_NAN_ERROR -4
/// Memory errors, like filures allocating memory
#define MOORDYN_MEM_ERROR -5
/// Invalid values
#define MOORDYN_INVALID_VALUE -6
/// Unhandled error
#define MOORDYN_UNHANDLED_ERROR -255

/**
 * @}
 */

#endif  // __MOORDYNAPI_H__
