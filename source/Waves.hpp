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

/** @file Waves.hpp
 * C++ API for the moordyn::Waves object
 */

#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include "Time.hpp"
#include <vector>
#include <map>

namespace moordyn {

/** @class Waves Waves.hpp
 * @brief Wave kinematics
 *
 * The wave kinematics can be used to set the underwater velocity field, which
 * is obviously affecting the dynamics. To this end both the velocity and
 * acceleration fields are provided in 4-D, i.e. 3D grid + time
 *
 * This system is used just if WaveKin is set to 2
 */
class Waves : public LogUser
{
  public:
	/// Constructor
	Waves(moordyn::Log* log);
	/// Destructor
	~Waves();

  private:
	/** @brief Setup the grid
	 *
	 * The grid is defined in a tabulated file (separator=' '). That file has 3
	 * head lines, which are ignored, and then 3 lines defining the grid points
	 * in x, y, z.
	 *
	 * Each one can be defined in 3 ways:
	 *   - Defining just a point in 0
	 *   - Defining the list of coordinates
	 *   - Defining the boundaries and the number of equispaced points
	 *
	 * @param filepath The definition file path
	 * @throws moordyn::input_file_error If the input file cannot be read, or if
	 * the fileis ill-formatted
	 * @throws moordyn::invalid_value_error If invalid values for the grid
	 * initialization are found
	 */
	void makeGrid(const char* filepath = "Mooring/water_grid.txt");

	/** @brief Allocate the needed memory for the kinematics storage
	 * @param filepath The definition file path. If NULL or "" isprovided, then
	 * "Mooring/water_grid.txt" is considered
	 * @throws moordyn::invalid_value_error If either the grid, or the time
	 * series has not been initialized yet
	 */
	void allocateKinematicsArrays();

	/** @brief instantiator that takes discrete wave elevation fft data only
	 * (MORE RECENT)
	 * @param zetaC0 Amplitude of each frequency component
	 * @param nw Number of wave components
	 * @param dw The difference in frequency between consequtive modes
	 * @param g Gravity accelerations
	 * @param h Water depth
	 * @throws moordyn::mem_error If there were roblems allocating memory
	 */
	void fillWaveGrid(const moordyn::complex* zetaC0,
	                  unsigned int nw,
	                  real dw,
	                  real g,
	                  real h);

	/** @brief Make a 2-D data grid
	 * @param nx Number of components in the first dimension
	 * @param ny Number of components in the second dimension
	 */
	static inline std::vector<std::vector<real>> init2DArray(unsigned int nx,
	                                                         unsigned int ny)
	{
		return std::vector<std::vector<real>>(nx, std::vector<real>(ny, 0.0));
	}

	/** @brief Make a 3-D data grid
	 * @param nx Number of components in the first dimension
	 * @param ny Number of components in the second dimension
	 * @param nz Number of components in the third dimension
	 */
	static inline std::vector<std::vector<std::vector<real>>>
	init3DArray(unsigned int nx, unsigned int ny, unsigned int nz)
	{
		return std::vector<std::vector<std::vector<real>>>(
		    nx, std::vector<std::vector<real>>(ny, std::vector<real>(nz, 0.0)));
	}

	/** @brief Make a 4-D data grid
	 * @param nx Number of components in the first dimension
	 * @param ny Number of components in the second dimension
	 * @param nz Number of components in the third dimension
	 * @param nw Number of components in the third dimension
	 */
	static inline std::vector<std::vector<std::vector<std::vector<real>>>>
	init4DArray(unsigned int nx,
	            unsigned int ny,
	            unsigned int nz,
	            unsigned int nw)
	{
		return std::vector<std::vector<std::vector<std::vector<real>>>>(
		    nx,
		    std::vector<std::vector<std::vector<real>>>(
		        ny,
		        std::vector<std::vector<real>>(nz,
		                                       std::vector<real>(nw, 0.0))));
	}

	/// number of grid points in x direction
	unsigned int nx;
	/// number of grid points in y direction
	unsigned int ny;
	/// number of grid points in z direction
	unsigned int nz;
	/// number of time steps used in wave kinematics time series
	unsigned int nt;
	/// time step for wave kinematics time series
	real dtWave;

	/// grid x coordinate arrays
	std::vector<real> px;
	/// grid y coordinate arrays
	std::vector<real> py;
	/// grid z coordinate arrays
	std::vector<real> pz;
	/// wave elevation [x,y,t]
	vector<vector<vector<real>>> zeta;
	/// dynamic pressure [x,y,z,t]
	vector<vector<vector<vector<real>>>> PDyn;
	/// wave velocity x component [x,y,z,t]
	vector<vector<vector<vector<real>>>> ux;
	/// wave velocity y component [x,y,z,t]
	vector<vector<vector<vector<real>>>> uy;
	/// wave velocity z component [x,y,z,t]
	vector<vector<vector<vector<real>>>> uz;
	/// wave acceleration x component [x,y,z,t]
	vector<vector<vector<vector<real>>>> ax;
	/// wave acceleration y component [x,y,z,t]
	vector<vector<vector<vector<real>>>> ay;
	/// wave acceleration z component [x,y,z,t]
	vector<vector<vector<vector<real>>>> az;

	/// gravity acceleration
	real g;
	/// water density
	real rho_w;

	/// The time integration scheme
	TimeScheme* _t_integrator;

	// ------------ from Line object... -----------
	// new additions for handling waves in-object and precalculating them	(not
	// necessarily used right now)
	//	int WaveMod;
	//	int WaveStMod;
	//	double Hs;
	//	double Tp;
	//	double gamma;
	//	float beta; 			// wave heading
	//
	//	vector< double > Ucurrent; // constant uniform current to add (three
	// components)

  public:
	/** @brief Types of coordinates input on the grid file
	 */
	typedef enum
	{
		/// Single point (0, 0, 0)
		GRID_SINGLE = 0,
		/// List of points
		GRID_LIST = 1,
		/// EQUISPACED POINTS
		GRID_LATTICE = 2,
	} coordtypes;

	/** @brief Setup the wave kinematics
	 *
	 * Always call this function after the construtor
	 * @param env The enviromental options
	 * @param t The time integration scheme
	 * @param folder The root folder where the wave data can be found
	 * @throws moordyn::input_file_error If an input file cannot be read, or if
	 * a file is ill-formatted
	 * @throws moordyn::invalid_value_error If invalid values are found
	 * @throws moordyn::non_implemented_error If WaveKin=2
	 * @throws moordyn::mem_error If there were roblems allocating memory
	 * @throws moordyn::output_file_error If data cannot be written in \p folder
	 */
	void setup(EnvCond* env, TimeScheme* t, const char* folder = "Mooring/");

	/** @brief Get the velocity, acceleration, wave height and dynamic pressure
	 * at a specific positon and time
	 * @param x The point x coordinate
	 * @param y The point y coordinate
	 * @param z The point z coordinate
	 * @param U_out The output velocity
	 * @param Ud_out The output acceleration
	 * @param zeta_out The output wave height
	 * @param PDyn_out The output dynamic pressure
	 */
	void getWaveKin(real x,
	                real y,
	                real z,
	                vec& U_out,
	                vec& Ud_out,
	                real& zeta_out,
	                real& PDyn_out);
};

// other relevant functions being thrown into this file for now (should move to
// Misc?) <<<<

/** @brief Compute the coordinates from a grid definition entry line
 * @param coordtype The type of coordinates input
 * @param entries Nothing if @p coordtype is 0; the list of coordinates if
 * @p coordtype is 1 and minimum limit; the maximum limit and the number of
 * points if @p coordtype is 2
 * @return The list of coordinates
 * @warning Memory will be allocated in coordarray. The user is responsible of
 * deallocating it afterwards
 */
std::vector<real>
gridAxisCoords(Waves::coordtypes coordtype, vector<string>& entries);

} // ::moordyn
