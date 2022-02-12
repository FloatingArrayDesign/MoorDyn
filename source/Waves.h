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

#ifndef WAVES_H
#define WAVES_H

#include "Misc.h"

using namespace std;



class Waves
{		
	int WaveKin;    // flag of wave types, same as in EnvCond

	
	int nx;           // number of grid points in x direction
	int ny;           //
	int nz;           //
	int nt;           // number of time steps used in wave kinematics time series
	double dtWave;      // time step for wave kinematics time series
	
	double *px;      // grid coordinate arrays
	double *py;
	double *pz;
	
	double  ***zeta;   // wave elevation [x,y,t]
	double ****PDyn;   // dynamic pressure [x,y,z,t]
	double ****ux;   // wave velocity [x,y,z,t]
	double ****uy;   //
	double ****uz;   //
	double ****ax;   // wave acceleration
	double ****ay;   //
	double ****az;   //
	
	double g;
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
	
	void makeGrid();
	void allocateKinematicsArrays();
	void setup(EnvCond *env);
	void getWaveKin(double x, double y, double z, double t, double U[3], double Ud[3], double* zeta, double* PDyn_out);
	void fillWaveGrid(doubleC *zetaC0, int nw, double dw, double g, double h );
	~Waves();
};



// other relevant functions being thrown into this file for now (should move to Misc?) <<<<

int gridAxisCoords(int coordtype, vector< string > &entries, double *&coordarray);

void doIFFT(kiss_fftr_cfg cfg, int nFFT, kiss_fft_cpx* cx_in, kiss_fft_scalar* cx_out, doubleC *inputs, double *outputs);

double WaveNumber( double Omega, double g, double h );

#endif


