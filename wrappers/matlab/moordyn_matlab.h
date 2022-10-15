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

#pragma once

#include <sstream>

// Encode a pointer as an uint64_t
uint64_t
encode_ptr(void* ptr)
{
	union
	{
		uint64_t i;
		void* p;
	} ivp;
	ivp.i = 0; // Do this, the uint64_t might be larger than void*
	ivp.p = ptr;
	return ivp.i;
}

// Decode a pointer from an uint64_t
void*
decode_ptr(uint64_t ptr)
{
	union
	{
		uint64_t i;
		void* p;
	} ivp;
	ivp.i = ptr;
	return ivp.p;
}

// A macro to declare every single function which uses a MoorDyn instance as
// the first input, which is every single one except MoorDyn_Create()
// Such instance will be stored in a variable named "instance"
// Remember to call the macro MOORDYNM_MEX_FUNCTION_END to finish the work
#define MOORDYNM_MEX_FUNCTION_BEGIN(instance_type, n_in, n_out)                \
	class MexFunction : public matlab::mex::Function                           \
	{                                                                          \
	  public:                                                                  \
		void operator()(ArgumentList outputs, ArgumentList inputs)             \
		{                                                                      \
			std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr =          \
			    getEngine();                                                   \
			ArrayFactory factory;                                              \
                                                                               \
			if (inputs.size() != n_in) {                                       \
				std::stringstream err_msg;                                     \
				err_msg << n_in << " inputs are required";                     \
				matlabPtr->feval(u"error",                                     \
				                 0,                                            \
				                 std::vector<Array>({ factory.createScalar(    \
				                     err_msg.str()) }));                       \
			}                                                                  \
                                                                               \
			if (outputs.size() != n_out) {                                     \
				std::stringstream err_msg;                                     \
				err_msg << n_out << " outputs are required";                   \
				matlabPtr->feval(u"error",                                     \
				                 0,                                            \
				                 std::vector<Array>({ factory.createScalar(    \
				                     err_msg.str()) }));                       \
			}                                                                  \
			const uint64_t _instance_id = inputs[0][0];                        \
			instance_type instance = (instance_type)decode_ptr(_instance_id);

#define MOORDYNM_MEX_FUNCTION_END                                              \
	}                                                                          \
	}                                                                          \
	;

#define MOORDYNM_CHECK_ERROR(err)                                              \
	if (err != MOORDYN_SUCCESS) {                                              \
		std::stringstream err_msg;                                             \
		err_msg << "MoorDyn reported an error: ";                              \
		switch (err) {                                                         \
			case MOORDYN_INVALID_INPUT_FILE:                                   \
				err_msg << "MOORDYN_INVALID_INPUT_FILE";                       \
				break;                                                         \
			case MOORDYN_INVALID_OUTPUT_FILE:                                  \
				err_msg << "MOORDYN_INVALID_OUTPUT_FILE";                      \
				break;                                                         \
			case MOORDYN_INVALID_INPUT:                                        \
				err_msg << "MOORDYN_INVALID_INPUT";                            \
				break;                                                         \
			case MOORDYN_NAN_ERROR:                                            \
				err_msg << "MOORDYN_NAN_ERROR";                                \
				break;                                                         \
			case MOORDYN_MEM_ERROR:                                            \
				err_msg << "MOORDYN_MEM_ERROR";                                \
				break;                                                         \
			case MOORDYN_INVALID_VALUE:                                        \
				err_msg << "MOORDYN_INVALID_VALUE";                            \
				break;                                                         \
			case MOORDYN_NON_IMPLEMENTED:                                      \
				err_msg << "MOORDYN_NON_IMPLEMENTED";                          \
				break;                                                         \
			case MOORDYN_UNHANDLED_ERROR:                                      \
				err_msg << "MOORDYN_UNHANDLED_ERROR";                          \
				break;                                                         \
			default:                                                           \
				err_msg << err;                                                \
		}                                                                      \
		matlabPtr->feval(                                                      \
		    u"error",                                                          \
		    0,                                                                 \
		    std::vector<Array>({ factory.createScalar(err_msg.str()) }));      \
	}
