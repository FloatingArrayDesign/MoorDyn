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

#ifndef CONNECTION_H
#define CONNECTION_H

#include "Misc.h"

using namespace std;

class Line;

class Connection
{
	// unique to Connection:
	Line* Attached[10]; 	// pointers to lines attached to this connection node
	int Top[10]; 			// which end of line are we attached to? 1 = top/fairlead, 0 = bottom/anchor
	int nAttached; 		// number of attached lines

	double conX; // constants set at startup from input file
	double conY;
	double conZ;
	double conM;
	double conV;
	double conFX;
	double conFY;
	double conFZ;
	double conCdA;
	double conCa;
	
	// environmental
	EnvCond env;  // struct to hold environmental settings
			
	// common properties with line internal nodes 	
	vector< double > r; 		// node position [x/y/z] // double* to continuous state or input state or parameter/other-state
	vector< double > rd;		// node velocity[x/y/z]  // double* to continuous state or input state or parameter/other-state
	vector< double > q;      	// unit tangent vector for end node (unused)
	
	double t; 					// simulation time
	double t0; // simulation time current integration was started at (used for BC function)
	vector< double > r_ves; 		// fairlead position for vessel node types [x/y/z]
	vector< double > rd_ves;		// fairlead velocity for vessel node  types [x/y/z]
	
	double tlast;
		
	vector< double > Fnet;	// total force on node
	vector< double > Fnet_i;
	vector< double > RHS;	// RHS of state-space equation (Forces divided by mass matrix)
	
	vector< vector< double > > S;  // inverse mass matrices (3x3) for each node
	vector< vector< double > > M; // node mass + added mass matrices
	vector< vector< double > > M_i;
			
	
public:
	
	//
	int number;
	int type;  // defining whether fixed 0, vessel 1, or connect 2	
	
	~Connection();
	
	void setup(ConnectProps& props);
	
	void addLineToConnect(Line& theLine, int TopOfLine);
	
	void getConnectState(vector<double> &r_out, vector<double> &rd_out);
		
	void getFnet(double Fnet_out[]);
	
	double GetConnectionOutput(OutChanProps outChan);
	
	void setEnv(EnvCond env_in);
	
	//void initialize( double* X, EnvCond env_in, double pX[], double TransMat[] );
	
	void initializeFairlead( double pX[], double TransMat[] );
	void initializeConnect( double* X );
	
	void getNetForceAndMass();
	
	void doRHS( const double* X,  double* Xd, const double time);
	
	void initiateStep(double FairIn[3], double rdFairIn[3], double time);	
	void updateFairlead( const double time);
	
	#ifdef USEGL
	void drawGL(void);
	#endif
	
};

#endif
