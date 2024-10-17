/**
 * Copyright (c) 2023, Alex Kinley, David Anderson
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
 *
 * This class is used to represent a three-dimensional
 * seafloor for use in a MoorDyn simulation.
 */

/** @file Seafloor.hpp
 * C++ API for 3D seafloor object.
 */
#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include <vector>
#include <map>

namespace moordyn {

/** @class Seafloor Seafloor.hpp
 * @brief Bathymetry description for MoorDyn
 *
 * Seafloor can provide a 2-D map of depths for MoorDyn
 */
class Seafloor : LogUser
{
  public:
	Seafloor(moordyn::Log* log);
	~Seafloor();

	/** @brief Setup the seafloor
	 *
	 * Always call this function after the constructor
	 * @param env The environmental options
	 * @param filepath The depths map file
	 * @throws moordyn::input_file_error If an input file cannot be read, or if
	 * a file is ill-formatted
	 * @throws moordyn::invalid_value_error If invalid values are found
	 */
	void setup(EnvCondRef env, const string& filepath);

	/** @brief Get the depth at a particular x/y coordinate
	 *
	 * This should default to nearest edge depth if beyond the
	 * grid where depths have been explicitly defined.
	 *
	 * @param x The x-coordinate of the point being assessed
	 * @param y The x-coordinate of the point being assessed
	 */
	real getDepthAt(real x, real y);

	/** @brief The average of the depth at all the grid points
	 *
	 */
	real getAverageDepth() { return averageDepth; }

	/** @brief The depth of the seafloor at the shallowest point
	 * Potentially useful for optimizing collision against the seafloor
	 */
	real getMinimumDepth() { return minDepth; }

  private:
	/// number of grid points (ticks) in x direction
	unsigned int nx;
	/// number of grid points (ticks) in y direction
	unsigned int ny;

	/// grid x coordinate arrays (indicating tick values)
	std::vector<real> px;
	/// grid y coordinate arrays (indicating tick values)
	std::vector<real> py;

	/// Seafloor depth grid (nx by ny grid of z vals)
	std::vector<std::vector<real>> depthGrid;

	/// the average of the depth at the grid points
	real averageDepth;
	/// the minimum depth of a grid point
	real minDepth;
};

/// Shared pointer
typedef std::shared_ptr<Seafloor> SeafloorRef;
}
