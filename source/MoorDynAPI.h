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

/** @mainpage Moordyn v2 developers documentation
 *
 * If you are looking for the users documentation, please visit
 * moordyn.readthedocs.io
 *
 * Code styling
 * ============
 *
 * Moordyn v2 is now based in clang-format to control the code styling, so you
 * do not need to worry about the code styling.
 *
 * Along this line, a .clang-format file has been placed in the root folder.
 * Many IDEs are able to use that file to automagically assist you in the code
 * editing.
 *
 * Please, if you plan to submit a pull request execute clang-format over all
 * the source files and headers before.
 */

/** @file MoorDynAPI.h
 * A set of handful definitions
 */

#ifndef __MOORDYNAPI_H__
#define __MOORDYNAPI_H__

#ifdef MoorDyn_EXPORTS
#ifdef WIN32
#define DECLDIR __declspec(dllexport)
#else
#define DECLDIR
#endif
#else
#ifdef WIN32
#define DECLDIR __declspec(dllimport)
#else
/// Prefix to export C functions on the compiled library
#define DECLDIR
#endif
#endif

#if (__GNUC__ > 2) || (__GNUC__ == 2 && __GNUC_MINOR__ > 6)
#define PARAM_UNUSED __attribute__((__unused__))
#else
/// Attribute for unused function parameters
#define PARAM_UNUSED
#endif

#ifdef _MSC_VER
#define DEPRECATED __declspec(deprecated)
#elif defined(__GNUC__) | defined(__clang__)
#define DEPRECATED __attribute__((__deprecated__))
#else
/// Prefix for deprecated functions that will be removed on a future version
#define DEPRECATED
#endif

#ifndef __FUNC_NAME__
#ifdef _MSC_VER
#define __FUNC_NAME__ __FUNCTION__
#else
/// Macro that is substituted by the function name
#define __FUNC_NAME__ __func__
#endif
#endif

#ifndef __PRETTY_FUNC_NAME__
#ifdef _MSC_VER
#define __PRETTY_FUNC_NAME__ __FUNCSIG__
#else
/// Macro that is substituted by the beautified function name
#define __PRETTY_FUNC_NAME__ __PRETTY_FUNCTION__
#endif
#endif

#ifndef XSTR
/// Wrapper on STR macro to can stringify the content of some other macros
#define XSTR(s) STR(s)
#endif
#ifndef STR
/// Stringfication on a macro
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
/// Disable the output since no output will never reach this level
#define MOORDYN_NO_OUTPUT 4096

/**
 * @}
 */

/** \defgroup moordyn_errors Errors reported by MoorDyn
 *  @{
 */

/** \defgroup moordyn_errors_c The list of error codes returned by the C API
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
/// Invalid values
#define MOORDYN_NON_IMPLEMENTED -7
/// Unhandled error
#define MOORDYN_UNHANDLED_ERROR -255

/**
 * @}
 */

/**
 * @}
 */

#endif // __MOORDYNAPI_H__
