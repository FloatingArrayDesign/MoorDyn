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

#pragma once

#include "Misc.h"

using namespace std;

namespace moordyn
{

/** @class Waves Waves.hpp
 * @brief Wave kinematics
 *
 * The wave kinematics can be used to set the underwater velocity field, which
 * is obviously affecting the dynamics. To this end both the velocity and
 * acceleration fields are provided in 4-D, i.e. 3D grid + time
 *
 * This system is used just if WaveKin is set to 2
 */
class Waves
{
	/// number of grid points in x direction
	int nx;           
	/// number of grid points in y direction
	int ny;
	/// number of grid points in z direction
	int nz;
	/// number of time steps used in wave kinematics time series
	int nt;
	/// time step for wave kinematics time series
	double dtWave;

	/// grid x coordinate arrays
	double *px;
	/// grid y coordinate arrays
	double *py;
	/// grid z coordinate arrays
	double *pz;
	/// wave elevation [x,y,t]
	double  ***zeta;
	/// dynamic pressure [x,y,z,t]
	double ****PDyn;
	/// wave velocity x component [x,y,z,t]
	double ****ux;
	/// wave velocity y component [x,y,z,t]
	double ****uy;
	/// wave velocity z component [x,y,z,t]
	double ****uz;
	/// wave acceleration x component [x,y,z,t]
	double ****ax;
	/// wave acceleration y component [x,y,z,t]
	double ****ay;
	/// wave acceleration z component [x,y,z,t]
	double ****az;
	
	/// gravity acceleration
	double g;
	/// water density
	double rho_w;
	
	// ------------ from Line object... -----------
	// new additions for handling waves in-object and precalculating them	(not necessarily used right now)
//	int WaveMod;
//	int WaveStMod;
//	double Hs;
//	double Tp;
//	double gamma;
//	float beta; 			// wave heading
//
//	vector< double > Ucurrent; // constant uniform current to add (three components)
	
	
public:
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
	 * @param filepath The definition file path. If NULL or "" isprovided, then
	 * "Mooring/water_grid.txt" is considered
	 */
	void makeGrid(const char* filepath=NULL);

	/** @brief Allocate the needed memory for the kinematics storage
	 * @param filepath The definition file path. If NULL or "" isprovided, then
	 * "Mooring/water_grid.txt" is considered
	 */
	void allocateKinematicsArrays();

	/** @brief Setup the wave kinematics
	 *
	 * Always call this function after the construtor
	 * @param env The enviromental options
	 */
	void setup(EnvCond *env);

	/** @brief Get the velocity, acceleration, wave height and dynamic pressure
	 * at a specific positon and time
	 * @param x The point x coordinate
	 * @param y The point y coordinate
	 * @param z The point z coordinate
	 * @param U The output velocity
	 * @param Ud The output acceleration
	 * @param zeta The output wave height
	 * @param PDyn_out The output dynamic pressure
	 */
	void getWaveKin(double x, double y, double z, double t, double U[3], double Ud[3], double* zeta, double* PDyn_out);

	/** @brief instantiator that takes discrete wave elevation fft data only
	 * (MORE RECENT)
	 * @param zetaC0 Amplitude of each frequency component
	 * @param nw Number of wave components
	 * @param dw The difference in frequency between consequtive modes
	 * @param g Gravity accelerations
	 * @param h Water depth
	 */
	void fillWaveGrid(doubleC *zetaC0, int nw, double dw, double g, double h );

	/// Constructor
	Waves();
	/// Destructor
	~Waves();
};



// other relevant functions being thrown into this file for now (should move to Misc?) <<<<

/** @brief Compute the coordinates from a grid definition entry line
 * @param coordtype 0 for a single point in zero, 1 for a list of coords, 2
 * for equispaced points between the provided boundaries
 * @param entries Nothing if @p coordtype is 0; the list of coordinates if
 * @p coordtype is 1 and minimum limit; the maximum limit and the number of
 * points if @p coordtype is 2
 * @param coordarray The output coordinates array
 * @return The number of points in coordarray
 * @warning Memory will be allocated in coordarray. The user is responsible of
 * deallocating it afterwards
 */
int gridAxisCoords(int coordtype, vector< string > &entries, double *&coordarray);

void doIFFT(kiss_fftr_cfg cfg, int nFFT, kiss_fft_cpx* cx_in, kiss_fft_scalar* cx_out, doubleC *inputs, double *outputs);

}  // ::moordyn
