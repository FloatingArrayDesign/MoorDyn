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

/** @file Instance.hpp
 * Base class for the moordyn::Body, moordyn::Line, moordyn::Point and
 * moordyn::Rod objects.
 */

#pragma once

#include "IO.hpp"
#include "Log.hpp"
#include <utility>

namespace moordyn {

class Waves;
typedef std::shared_ptr<Waves> WavesRef;

class Point;
class Rod;

/** @class Instance Instance.hpp
 * @brief A generic instance
 */
class DECLDIR Instance : public io::IO
{
  public:
	/** @brief Costructor
	 * @param log Logging handler defining where/how results should be logged.
	 */
	Instance(moordyn::Log* log);

	/** @brief Destructor
	 */
	virtual ~Instance() = default;

	/** @brief Get the unique identifier of this instance
	 *
	 * The unique identifier is a growing index which identifies every single
	 * created instance. Even if an instance is removed, the index will remain
	 * taken.
	 * @return The unique identifier
	 */
	inline const size_t Id() const { return _id; }

	/** @brief Get the number of state variables required by this instance
	 *
	 * This can be seen as the number of rows on the state variables holder.
	 * @return The number of state variables
	 */
	virtual inline const size_t state_n() const { return 1; }

	/** @brief Get the dimension of the state variable.
	 *
	 * This can be seen as the number of columns on the state variables holder.
	 * @return The dimension of the state variable
	 */
	virtual inline const size_t state_dims() const { return 6; }
  private:
	/// Unique identifier of this instance
	size_t _id;
};

} // ::moordyn
