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
class Waves;

class Rod 
{
//
//Rod Types:
//	0: free to move
//	1: pinned to a fixed point (or a body/ptfm, in which case added to list?)
//	2: attached rigidly to a fixed point (or a body/ptfm, in which case added to list?)
//	-1: pinned to a coupling point (3dof)
//	-2: attached rigidly to a coupling point (6dof)

	// ENVIRONMENTAL STUFF	
	EnvCond *env;  // pointer to global struct that holds environmental settings
	Waves *waves;  // pointer to global Waves object
	

	// unique to Rod (like a doubling of Connection):
	Line* AttachedA[10]; 	// pointers to lines attached to this connection node
	Line* AttachedB[10]; 	// pointers to lines attached to this connection node
	int TopA[10]; 			// which end of line are we attached to? 1 = top/fairlead, 0 = bottom/anchor
	int TopB[10]; 			// which end of line are we attached to? 1 = top/fairlead, 0 = bottom/anchor
	int nAttachedA; 		// number of attached lines
	int nAttachedB; 		// number of attached lines
	

		
	// ROD STUFF
    
	// parameters
	RodProps props;	
	int N;                  // number of line nodes 
	double UnstrLen;        // the constrained length of the rod
	double d;		        // rod diameter
	double rho;		        // rod linear density
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	//double ReFac;

	// degrees of freedom (or states)
	double r6 [6];          // Rod 6dof position [x/y/z/u1/u2/u3] (end A coordinates and direction unit vector)
	double v6 [6];	        // Rod 6dof velocity[vx/vy/vz/wx/wy/wz] (end A velocity and rotational velocities about unrotated axes)

	// kinematics
	double **r;             // node positions [i][x/y/z]
	double **rd;            // node velocities [i][x/y/z]
	double q[3];      	    // unit tangent vector for rod
	double *l; 		        // line unstretched segment lengths

	double ***M;            // node mass + added mass matrix
	//double Mext[3];         // net moment from attached lines at either end <<< should rename to be clearly moment not mass
	double *V;              // line segment volume	
	double FextA[3];              // external forces from attached lines on/about end A 
	double FextB[3];              // external forces from attached lines on/about end A 
	double Mext[3];         // external moments (from attached cables or waterplane hydrostatic moment) 
	double F6net[6];        // total force and moment about end A (excluding inertial loads) that Rod may exert on whatever it's attached to
	double M6net[6][6];     // total mass matrix about end A of Rod and any attached Points

	// forces 
	double **W;             // node dry weight 	
	double **Bo;            // node buoyancy 	
	double **Pd;            // dynamic pressure
	double **Dp;            // node drag (transverse)
	double **Dq;            // node drag (axial)
	double **Ap;            // node added mass forcing (transverse)
	double **Aq;            // node added mass forcing (axial)
	double **B;             // node bottom contact force	
	double **Fnet;          // total force on node  <<<<<<< might remove this for Rods

	// wave things
	double *F; 		        // VOF scalar for each segment (1 = fully submerged, 0 = out of water)
	double *zeta;           // free surface elevation
	double *PDyn;           // dynamic pressure
	double **U;             // wave velocities	
	double **Ud;            // wave accelerations
	double h0;              // instantaneous axial submerged length [m]

	// time
	double t;               // simulation time
	double t0;              // simulation time current integration was started at (used for BC function)
	double r_ves[6]; 		// fairlead position for coupled rods [x/y/z]
	double rd_ves[6];		// fairlead velocity for coupled rods [x/y/z]

	// motion component vectors (declaring these here as they caused problems if declared in the loop)
	double vi[3];           // relative velocity
	double vp[3];           // transverse component of relative velocity
	double vq[3];           // axial component of relative velocity
	double ap[3];           // transverse component of absolute acceleration - HAD TO MOVE THIS UP FROM LOWER DOWN (USED TO CRASH AFTER I=3)
	double aq[3];           // axial component of absolute acceleration

	// file stuff
	ofstream * outfile;     // if not a pointer, caused odeint system initialization error during compilation
	string channels;

	// data structures for precalculated nodal water kinematics if applicable
	double **zetaTS;        // time series of wave elevations above each node
	double **FTS;
	double ***UTS;
	double ***UdTS;
	int ntWater;            // number of water kinematics time steps
	double dtWater;         // water kinematics time step size (s)

public:
	/** @brief Types of rods
	 */
	typedef enum {
		/// Is coupled, i.e. is controlled by the user
		COUPLED = -2,
		/// Is a pinned fairlead
		CPLDPIN = -1,
		/// Is free to move, controlled by MoorDyn
		FREE = 0,
		/// Is pinned
		PINNED = 1,
		/// Is fixed, either to a location or to another moving entity
		FIXED = 2,
		// Some aliases
		VESSEL = COUPLED,
		VESPIN = CPLDPIN,
		CONNECT = FREE,
		ANCHOR = FIXED,
	} types;

	/** @brief Return a string with the name of a type
	 *
	 * This tool is useful mainly for debugging
	 */
	static string TypeName(types t)
	{
		switch(t)
		{
		case COUPLED:
			return "COUPLED";
		case CPLDPIN:
			return "CPLDPIN";
		case FREE:
			return "FREE";
		case PINNED:
			return "PINNED";
		case FIXED:
			return "FIXED";
		}
		return "UNKNOWN";
	}

 	int number; // rod "number" id
	types type;  // 	0: free to move; 1: pinned; 2: attached rigidly (positive if to something, negative if coupled)
//	int pinned;      // flag indicating of Rod end A is pinned (1) or free (0/default). Triggered by setting BodyToAddTO to -1.
	
	double roll;
	double pitch;
	
	int WaterKin;  // flag indicating whether wave/current kinematics will be considered for this linec
	// 0: none, or use value set externally for each node of the object; 1: interpolate from stored; 2: call interpolation function from global Waves grid


 	// unique to Line
	//Connection* AnchConnect;  // pointer to anchor connection
	//Connection* FairConnect;  // pointer to fairlead connection
 
	int getN(); // returns N (number of segments)
	
	int setup(int number_in, types type_in, RodProps *props, double endCoords[6], int NumSegs, 
	shared_ptr<ofstream> outfile_pointer, string channels_in);
	
	void addLineToRodEndA(Line *theLine, int TopOfLine);
	void addLineToRodEndB(Line *theLine, int TopOfLine);
	void removeLineFromRodEndA(int lineID, int *topOfLine, double rEnd[], double rdEnd[]);
	void removeLineFromRodEndB(int lineID, int *topOfLine, double rEnd[], double rdEnd[]);
	
	void initializeRod(double *X );

	void setEnv(EnvCond *env_in, Waves* waves_in);

	int getNodePos(int i, double pos[3]);
	
	double GetRodOutput(OutChanProps outChan);
	
	void storeWaterKin(int nt, double dt, double **zeta_in, double **f_in, double ***u_in, double ***ud_in);
	
	void scaleDrag(double scaler);	
	void setTime(double time);
	
	void initiateStep(const double rFairIn[6], const double rdFairIn[6], double time);
	
	void updateFairlead(const double time);
	void setKinematics(double *r_in, double *rd_in);
	int setState( double* X, const double time);
	
	void setDependentStates();
	int getStateDeriv(double* Xd);
	
	void getFnet(double Fnet_out[]);
	void getNetForceAndMass(double rBody[3], double Fnet_out[6], double M_out[6][6]);
	
	void doRHS();

	void Output(double );
	
	~Rod();

#ifdef USEGL	
	void drawGL(void);
	void drawGL2(void);
#endif
};

#endif


