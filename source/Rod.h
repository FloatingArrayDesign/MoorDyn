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

#ifndef ROD_H
#define ROD_H

#include "Misc.h"

using namespace std;

// here is the numbering scheme (N segments per line):
//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]

class Line;

class Rod 
{
	// unique to Rod (like a doubling of Connection):
	Line* AttachedA[10]; 	// pointers to lines attached to this connection node
	Line* AttachedB[10]; 	// pointers to lines attached to this connection node
	int TopA[10]; 			// which end of line are we attached to? 1 = top/fairlead, 0 = bottom/anchor
	int TopB[10]; 			// which end of line are we attached to? 1 = top/fairlead, 0 = bottom/anchor
	int nAttachedA; 		// number of attached lines
	int nAttachedB; 		// number of attached lines
	
	// ENVIRONMENTAL STUFF
	
	EnvCond env;  // struct to hold environmental settings
	
		
	// ROD STUFF
    
	// parameters
	RodProps props;	
	int N; // number of line nodes 
	double UnstrLen; // the constrained length of the rod
	double d;		// rod diameter
	double rho;		// rod linear density
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	//double ReFac;
	
	double A; // line cross-sectional area to pre-compute
	
	// degrees of freedom (or states)
	double  r6 [6]; 		// Rod 6dof position [x/y/z] // double* to continuous state or input state or parameter/other-state
	double r6d [6];		// Rod 6dof velocity[x/y/z]  // double* to continuous state or input state or parameter/other-state
	
	
	// kinematics
	vector< vector< double > > r; 		// node positions [i][x/y/z]
	vector< vector< double > > rd;	// node velocities [i][x/y/z]
	vector< double > q;      	// unit tangent vector for rod
	
	// time
	double t; 					// simulation time
	double t0; // simulation time current integration was started at (used for BC function)
	double tlast;
		
	// motion component vectors (declaring these here as they caused problems if declared in the loop)
	double vi[3]; // relative velocity
	double vp[3]; // transverse component of relative velocity
	double vq[3]; // axial component of relative velocity
	double ap[3]; // transverse component of absolute acceleration - HAD TO MOVE THIS UP FROM LOWER DOWN (USED TO CRASH AFTER I=3)
	double aq[3]; // axial component of absolute acceleration
				
	// forces 
	vector< vector< double > > T; //
	vector< vector< double > > Td;//
	vector< double > Tmag;				// segment tension magnitude 
	vector< vector< double > > W;		// node weight 	
	vector< vector< double > > Dp;	// node drag (transverse)
	vector< vector< double > > Dq;	// node drag (axial)
	vector< vector< double > > Ap;	// node added mass forcing (transverse)
	vector< vector< double > > Aq;	// node added mass forcing (axial)
	vector< vector< double > > B; 	// node bottom contact force	
	vector< vector< double > > Fnet;	// total force on node  <<<<<<< might remove this for Rods
		
	vector< double > FnetA;				// net force for end A of Rod
	vector< double > FnetB;				// net force for end B of Rod	
	
	vector< vector< vector< double > > > M; // node mass + added mass matrix
	
	vector< vector< double > > MA;   // mass matrix for end A of Rod
	vector< vector< double > > MB;   // mass matrix for end B of Rod
			
	vector<double> F; 		// VOF scalar for each segment (1 = fully submerged, 0 = out of water)
	
	vector<double> l; 		// line unstretched segment lengths
	vector<double> V;		// line segment volume
	
	// set up output arrays, at each node i:
	vector< vector< double > >  U;     // wave velocities	
	vector< vector< double > >  Ud;     // wave accelerations
	
	vector<double> zeta;    // free surface elevation

	// file stuff
	
	ofstream * outfile; // if not a pointer, caused odeint system initialization error during compilation
	string channels;
	
	// new additions for handling waves in-object and precalculating them	(not necessarily used right now)
	int WaveMod;
	int WaveStMod;
	double Hs;
	double Tp;
	double gamma;
	float beta; 			// wave heading
		
	int Nw;  				// number of wave frequency components considered    //OK AS INT???
	vector<float> w;
	vector<float> k;
	float dw;			// FAST's dw (really small typically)
	
	vector<floatC> zetaC0;		// Fourier transform of wave elevation at origin
	vector<floatC> zetaC;		// Fourier transform of wave elevation
	vector< vector< floatC > > UC;     // Fourier transform of wave velocities
	vector< vector< floatC > > UdC;     // Fourier transform of wave accelerations
	
	vector<doubleC> WGNC;		// 
	vector<double> WaveS2Sdd;	// 
	doubleC WGNC_Fact; 		// sqrt( pi/(dw*WaveDT) );   // This factor is needed by the discrete time inverse Fourier transform to ensure that the time series WGN process has unit variance
	double S2Sd_Fact; 		// 1.0/WaveDT;                       // This factor is also needed by the discrete time inverse Fourier transform

	vector< double > Ucurrent; // constant uniform current to add (three components)
	
	// new additions for precalculating wave quantities
	vector< vector< double > > zetaTS;   // time series of wave elevations above each node
	vector< vector< double > > FTS;
	vector< vector< vector< double > > > UTS;
	vector< vector< vector< double > > > UdTS;
	int Nt; 				// number of wave time steps
	double WaveDT; 		// wave time step size (s)
	vector< double > tTS; 	// time step vector
	int ts0; 				// time step index used for interpolating wave kinematics time series data (put here so it's persistent)
	

public:
 	int number; // rod "number" id
	int type;  // defining whether part of a body (1), or independent (2)	
	
	int WaveKin;  // flag indicating whether wave kinematics will be considered for this line
 
 	// unique to Line
	//Connection* AnchConnect;  // pointer to anchor connection
	//Connection* FairConnect;  // pointer to fairlead connection
 
	int getN(); // returns N (number of segments)
	
	int setup(int type_in, int number_in, RodProps *props, double endCoords[6], int NumSegs, 
	shared_ptr<ofstream> outfile_pointer, string channels_in);
	
	void addLineToRodEndA(Line *theLine, int TopOfLine);
	void addLineToRodEndB(Line *theLine, int TopOfLine);
	
	void initializeRod(double* X );

	void setEnv(EnvCond env_in);

	int getNodePos(int i, double pos[3]);
	
	double GetRodOutput(OutChanProps outChan);
	
	void scaleDrag(double scaler);	
	void setTime(double time);
	
	int setState( const double* X, const double time);
	
	int getStateDeriv(double* Xd);
	
	void getNetForceAndMassContribution(double rBody[3], double Fnet_out[6], double M_out[6][6]);
	
	void doRHS();

	void Output(double );
	
	~Rod();

#ifdef USEGL	
	void drawGL(void);
	void drawGL2(void);
#endif
};

#endif


