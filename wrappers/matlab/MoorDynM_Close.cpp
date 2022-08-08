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

#include "mex.hpp"
#include "mexAdapter.hpp"

#include "MoorDyn2.h"
#include "moordyn_matlab.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

class MexFunction : public matlab::mex::Function
{
  public:
	void operator()(ArgumentList outputs, ArgumentList inputs)
	{
		std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
		ArrayFactory factory;
		checkArguments(outputs, inputs);

		const uint64_t id = inputs[0][0];
		const int err = MoorDyn_Close((MoorDyn)decode_ptr(id));
		if (err != MOORDYN_SUCCESS) {
			matlabPtr->feval(u"error",
			                 0,
			                 std::vector<Array>({ factory.createScalar(
			                     "MoorDyn reported an error") }));
		}
	}
	void checkArguments(ArgumentList outputs, ArgumentList inputs)
	{
		std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
		ArrayFactory factory;

		if ((inputs.size() != 1) ||
		    (inputs[0].getType() != ArrayType::UINT64) ||
		    (inputs[0].getNumberOfElements() != 1)) {
			matlabPtr->feval(u"error",
			                 0,
			                 std::vector<Array>({ factory.createScalar(
			                     "1 input MoorDyn ID is required") }));
		}

		// Check number of outputs
		if (outputs.size() != 0) {
			matlabPtr->feval(u"error",
			                 0,
			                 std::vector<Array>({ factory.createScalar(
			                     "No outputs are accepted") }));
		}
	}
};
