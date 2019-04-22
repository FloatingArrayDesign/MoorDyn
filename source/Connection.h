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
	double r[3];      // node position [x/y/z]
	double rd[3];	     // node velocity[x/y/z] 
	
	double t; 					// simulation time
	double t0; // simulation time current integration was started at (used for BC function)
	double r_ves[3]; 		// fairlead position for vessel node types [x/y/z]
	double rd_ves[3];		// fairlead velocity for vessel node  types [x/y/z]
	
	double tlast;
		
	double Fnet[3];	// total force on node
	double Fnet_i[3];
	double RHS[3];	// RHS of state-space equation (Forces divided by mass matrix)
	
	double M[3][3]; // node mass + added mass matrices
	
	//vector< vector< double > > S;  // inverse mass matrices (3x3) for each node
	//vector< vector< double > > M_i;
			
	
public:
	
	//
	int number;
	int type;  // defining whether fixed 0, vessel 1, or connect 2	
		
	void setup(int number_in, int type_in, double r0_in[3], double M_in,
	double V_in, double F_in[3], double CdA_in, double Ca_in);
	
	void addLineToConnect(Line *theLine, int TopOfLine);
	
	void initializeFairlead( double pX[], double TransMat[] );
	void initializeFairlead2( double pX[], double vX[] );
	void initializeConnect( double* X );
	void initializeAnchor();
	
	void getConnectState(vector<double> &r_out, vector<double> &rd_out);
		
	//void addRodEffect(vector<double> &F_rod, vector< vector<double> > &M_rod);
		
	void getFnet(double Fnet_out[]);
	
	void getM(double M_out[3][3]);
	
	double GetConnectionOutput(OutChanProps outChan);
	
	void setEnv(EnvCond env_in);
	
	void scaleDrag(double scaler);	
	void setTime(double time);
	
	//void initialize( double* X, EnvCond env_in, double pX[], double TransMat[] );
	
	int setState( const double* X, const double time);
	
	int getStateDeriv( double* Xd);
	
	int getNetForceAndMassContribution(double rBody[3], double Fnet_out[6], double M_out[6][6]);
	
	int doRHS();
	
	void initiateStep(double rFairIn[3], double rdFairIn[3], double time);
	
	//void sumNetForceAndMass();
	
	//void doRHS( const double* X,  double* Xd, const double time);
	
	void updateFairlead( const double time);
	
	~Connection();
	
	#ifdef USEGL
	void drawGL(void);
	#endif
	
	
};

#endif
