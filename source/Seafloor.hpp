/**
* Copyright (c) 2023, Kelson Marine Co.
* 
* This class is used to represent a three-dimensional
* seafloor for use in a MoorDyn simulation.
/

/** @file Seafloor.hpp
* C++ API for 3D seafloor object.
*/
#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include "Time.hpp"
#include <vector>
#include <map>

namespace moordyn {
/** @class Seafloor Seafloor.hpp
* @brief 
*/
class Seafloor : LogUser
{
  public:
	Seafloor(moordyn::Log* log);
	~Seafloor();


	/** @brief Setup the seafloor 
	 *
	 * Always call this function after the construtor
	 * @param env The enviromental options
	 * @param folder The root folder where the wave data can be found
	 * @throws moordyn::input_file_error If an input file cannot be read, or if
	 * a file is ill-formatted
	 * @throws moordyn::invalid_value_error If invalid values are found
	 * @throws moordyn::mem_error If there were roblems allocating memory
	 * @throws moordyn::output_file_error If data cannot be written in \p folder
	 */
	void setup(EnvCond* env, const char* folder = "Mooring/");

	/** @brief Get the depth at a particular x/y coordinate
	* This should default to nearest edge depth if beyond the
	* grid where depths have been explicitly defined.
	* 
	* @param x The x-coordinate of the point being assessed
	* @param y The x-coordinate of the point being assessed
	*/
	real getDepthAt(real x, real y);

  private:
	/// number of grid points in x direction
	unsigned int nx;
	/// number of grid points in y direction
	unsigned int ny;

	/// grid x coordinate arrays
	std::vector<real> px;
	/// grid y coordinate arrays
	std::vector<real> py;

	/// Seafloor depth grid
	std::vector<std::vector<real>> depthGrid;

};
}
