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

MOORDYNM_MEX_FUNCTION_BEGIN(MoorDyn, 5, 2)
{
	TypedArray<double> x_matlab = std::move(inputs[1]);
	std::vector<double> x(x_matlab.begin(), x_matlab.end());
	TypedArray<double> xd_matlab = std::move(inputs[2]);
	std::vector<double> xd(xd_matlab.begin(), xd_matlab.end());
	double t = inputs[3][0];
	double dt = inputs[4][0];

	std::vector<double> f(x.size(), 0.0);
	const int err =
	    MoorDyn_Step(instance, x.data(), xd.data(), f.data(), &t, &dt);
	MOORDYNM_CHECK_ERROR(err);

	outputs[0] = factory.createScalar<double>(t);
	outputs[1] = factory.createArray<double>(
	    { 1, f.size() }, f.data(), f.data() + f.size());
}
MOORDYNM_MEX_FUNCTION_END
