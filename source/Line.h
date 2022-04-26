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

#ifndef LINE_H
#define LINE_H

#include "Misc.h"

using namespace std;

// here is the numbering scheme (N segments per line):
//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]

//class Connection;
class Waves;

class Line 
{
	
	// ENVIRONMENTAL STUFF	
	EnvCond *env;  // pointer to global struct that holds environmental settings
	Waves *waves;  // pointer to global Waves object
	
		
	// LINE STUFF
    
	// parameters
	LineProps props;	
	int N; // number of line nodes 
	moordyn::real UnstrLen;
	moordyn::real d;		// line diameter
	moordyn::real rho;		// line linear density
	moordyn::real E;		// line elasticity modulus [Pa] 
	moordyn::real EI;		// line bending stiffness [Nm^2] <<<<<<<< need to figure out how to load in through input file (where to put)
	moordyn::real c;		// line axial internal damping coefficient [Ns]
	moordyn::real cI;		// line bending internal damping coefficient [??]
	moordyn::real Can;
	moordyn::real Cat;
	moordyn::real Cdn;
	moordyn::real Cdt;
	
	moordyn::real BAin;		// line axial internal damping coefficient input (before proceessing)
	
	moordyn::real A; // line cross-sectional area to pre-compute
	
	int nEApoints = 0; // number of values in stress-strain lookup table (0 means using constant E)
	moordyn::real stiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	moordyn::real stiffYs[nCoef]; // y array for stress-strain lookup table
	int nCpoints;        // number of values in stress-strainrate lookup table (0 means using constant c)
	moordyn::real dampXs[nCoef]; // x array for stress-strainrate lookup table (up to nCoef)
	moordyn::real dampYs[nCoef]; // y array for stress-strainrate lookup table
	int nEIpoints = 0; // number of values in stress-strain lookup table (0 means using constant E)
	moordyn::real bstiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	moordyn::real bstiffYs[nCoef]; // y array for stress-strain lookup table
	
	
	// kinematics
	std::vector<vec> r;               // node positions [i][x/y/z]
	std::vector<vec> rd;              // node velocities [i][x/y/z]
	std::vector<vec> q;               // unit tangent vectors for each node
	std::vector<vec> qs;              // unit tangent vectors for each segment (used in bending calcs)
	std::vector<moordyn::real> l;     // line unstretched segment lengths
	std::vector<moordyn::real> lstr;  // stretched lengths
	std::vector<moordyn::real> ldstr; // rate of stretch
	std::vector<moordyn::real> Kurv;  // curvatures at node points (1/m)
	
	std::vector<mat> M;               // node mass + added mass matrix
	std::vector<moordyn::real> V;     // line segment volume	
				
	// forces 
	std::vector<vec> T;               // segment tensions
	std::vector<vec> Td;              // segment damping forces
	std::vector<vec> Bs;              // bending stiffness forces
	std::vector<vec> W;               // node weight 	
	std::vector<vec> Dp;              // node drag (transverse)
	std::vector<vec> Dq;              // node drag (axial)
	std::vector<vec> Ap;              // node added mass forcing (transverse)
	std::vector<vec> Aq;              // node added mass forcing (axial)
	std::vector<vec> B;               // node bottom contact force	
	std::vector<vec> Fnet;            // total force on node  <<<<<<< might remove this for Rods
		
	// wave things
	std::vector<moordyn::real> F;     // VOF scalar for each segment (1 = fully submerged, 0 = out of water)
	std::vector<moordyn::real> zeta;  // free surface elevation
	std::vector<moordyn::real> PDyn;  // dynamic pressure
	std::vector<vec> U;               // wave velocities	
	std::vector<vec> Ud;              // wave accelerations	
	
	
	// time
	moordyn::real t;                  // simulation time
	moordyn::real t0;                 // simulation time current integration was started at (used for BC function)
	moordyn::real tlast;
	
	// end conditions
	int endTypeA;                     // type of connection at end A: 0=pinned to Connection, 1=cantilevered to Rod.
	int endTypeB;
	vec endMomentA;                   // moment at end A from bending, to be applied on attached Rod/Body
	vec endMomentB;

	// file stuff	
	ofstream * outfile;               // if not a pointer, caused odeint system initialization error during compilation
	string channels;
	
	// data structures for precalculated nodal water kinematics if applicable
	std::vector<std::vector<moordyn::real>> zetaTS; // time series of wave elevations above each node
	std::vector<std::vector<moordyn::real>> FTS;
	std::vector<std::vector<vec>> UTS;
	std::vector<std::vector<vec>> UdTS;
	int ntWater;                                    // number of water kinematics time steps
	moordyn::real dtWater;                          // water kinematics time step size (s)

//	int ts0; 				// time step index used for interpolating wave kinematics time series data (put here so it's persistent) ????
	

public:
 	int number; // line "number" id
	
	int WaterKin;  // flag indicating whether wave/current kinematics will be considered for this linec
	// 0: none, or use value set externally for each node of the object; 1: interpolate from stored; 2: call interpolation function from global Waves grid

 
 	// unique to Line
	//Connection* AnchConnect;  // pointer to anchor connection
	//Connection* FairConnect;  // pointer to fairlead connection
 
	void setup(int number, LineProps *props_in, double UnstrLen_in, int NumNodes, 
	//	Connection& AnchConnect_in, Connection& FairConnect_in,
		shared_ptr<ofstream> outfile_pointer, string channels_in);
	
	void initializeLine(double* X );

	void setEnv(EnvCond *env_in, Waves* waves_in);

	double getNodeTen(int i);
	
	int getNodePos(int i, double pos[3]);
	void getNodeCoordinates(double r_out[]);
	void setNodeWaveKin(double U_in[], double Ud_in[]);
	
	double GetLineOutput(OutChanProps outChan);
	
	void storeWaterKin(int nt, double dt, double **zeta_in, double **f_in, double ***u_in, double ***ud_in);
	
	void getFASTtens(float* FairHTen, float* FairVTen, float* AnchHTen, float* AnchVTen);

	void getEndStuff(double Fnet_out[3], double Moment_out[3], double M_out[3][3], int topOfLine);

	int getN(); // returns N (number of segments)
	
	void setupWaves(vector<double> Ucurrent_in, float dt_in);
		
	double getNonlinearE(double l_stretched, double l_unstretched);
	double getNonlinearC(double ld_stretched, double l_unstretched);
		
	void scaleDrag(double scaler);	
	void setTime(double time);
	
	void setState( const double* X, const double time);
	
	void setEndState(double r_in[3], double rd_in[3], int topOfLine);
	void setEndState(vector<double> &r_in, vector<double> &rd_in, int topOfLine);
	
	void setEndOrientation(double *qin, int topOfLine, int rodEndB);
	
	void getEndSegmentInfo(double q_EI_dl[3], int topOfLine, int rodEndB);
	void getEndSegmentInfo(double qEnd[3], double *EIout, double *dlout, int topOfLine);
	
	void getStateDeriv(double* Xd, const double dt);
	
	void doRHS( const double* X,  double* Xd, const double time, const double dt);

	//void initiateStep(vector<double> &rFairIn, vector<double> &rdFairIn, double time);
		
	void Output(double );
	
	~Line();

#ifdef USEGL	
	void drawGL(void);
	void drawGL2(void);
#endif
};

#endif


