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

class Connection;

class Line 
{
	
	// ENVIRONMENTAL STUFF
	
	EnvCond env;  // struct to hold environmental settings
	
		
	// LINE STUFF
    
	int N; // number of line nodes 
	double UnstrLen;
	LineProps props;	
	
	vector< vector< double > > r; 		// node positions [i][x/y/z]
	vector< vector< double > > rd;	// node velocities [i][x/y/z]
	vector< vector< double > > q;      	// unit tangent vectors for each segment
	
	double t; 					// simulation time
	double t0; // simulation time current integration was started at (used for BC function)
	
	double tlast;
		
	// declaring these here as they caused problems if declared in the loop
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
	
	vector< vector< double > > Fnet;	// total force on node
		
	vector< vector< vector< double > > > S;  // inverse mass matrices (3x3) for each node
	vector< vector< vector< double > > > M; // node mass + added mass matrix
			
	vector<double> F; 		// VOF scalar for each segment (1 = fully submerged, 0 = out of water)
	
	vector<double> l; 		// line unstretched segment lengths
	vector<double> lstr; 		// stretched lengths (m)
	vector<double> ldstr; 		// rate of stretch (m/s)
	double d;		// line diameter
	double rho;		// line density
	double E;		// line elasticity modulus [N]
	double c;		// line axial internal damping coefficient [Ns]
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	double ReFac;
	
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
 	int number; // line "number" id
	
	int WaveKin;  // flag indicating whether wave kinematics will be considered for this line
 
 	// unique to Line
	Connection* AnchConnect;  // pointer to anchor connection
	Connection* FairConnect;  // pointer to fairlead connection
 
	int getN(); // returns N (number of segments)
	
	void setup(int number, LineProps props_in, double UnstrLen_in, int NumNodes, 
		Connection& AnchConnect_in, Connection& FairConnect_in,
		shared_ptr<ofstream> outfile_pointer, string channels_in);
	
	void initialize( double* X );

	double getNodeTen(int i);
	
	int getNodePos(int i, double pos[3]);
	
	double GetLineOutput(OutChanProps outChan);
	
	void getFASTtens(float* FairHTen, float* FairVTen, float* AnchHTen, float* AnchVTen);
	
	void getAnchStuff(vector<double> &Fnet_out, vector< vector<double> > &M_out);

	void getFairStuff(vector<double> &Fnet_out, vector< vector<double> > &M_out);

			
	void setupWaves(EnvCond env_in, vector<double> Ucurrent_in, float dt_in);
		
	void scaleDrag(double scaler);
	
	void setTime(double time);
	
	void doRHS( const double* X,  double* Xd, const double time, const double dt);

	void initiateStep(vector<double> &rFairIn, vector<double> &rdFairIn, double time);
		
	void Output(double );

#ifdef USEGL	
	void drawGL(void);
	void drawGL2(void);
#endif
};

#endif


