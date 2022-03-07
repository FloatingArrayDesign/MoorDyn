/*
 * Copyright (c) 2019 Matt Hall <mtjhall@alumni.uvic.ca>
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
 
#include "Line.h"
#include "Waves.h"
#include "QSlines.h" // the c++ version of quasi-static model Catenary

using namespace std;

// here is the new numbering scheme (N segments per line)

//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]




// ================== Line member functions ===========================


// set up line object
void Line::setup(int number_in, LineProps *props, double UnstrLen_in, int NumSegs, 
//	Connection &AnchConnect_in, Connection &FairConnect_in,
	shared_ptr<ofstream> outfile_pointer, string channels_in) 
{
	// ================== set up properties ===========	
	number = number_in;	
	UnstrLen = UnstrLen_in;    // Note, this is a temporary value that will be processed depending on sign during initializeLine 
	N = NumSegs; // assign number of nodes to line
	
//	WaveKin = 0;  // start off with wave kinematics disabled.  Can be enabled after initial conditions are found and wave kinematics are calculated
	
//	AnchConnect = &AnchConnect_in;		// assign line end connections <<<<<<<<< no longer needed?? <<<<<
//	FairConnect = &FairConnect_in;	
		
	// store passed line properties (and convert to numbers)
	d   = props->d;
	rho = props->w  /(pi/4.*d*d);
	E   = props->EA /(pi/4.*d*d);
	EI  = props->EI;
	BAin = props->c;                 // Note, this is a temporary value that will be processed depending on sign during initializeLine 
	Can = props->Can;
	Cat = props->Cat;
	Cdn = props->Cdn;
	Cdt = props->Cdt;
	
	nEApoints = props->nEApoints;    // copy in nonlinear stress-strain data if applicable
	for (int I=0; I<nEApoints; I++)
	{	stiffXs[I] = props->stiffXs[I];
		stiffYs[I] = props->stiffYs[I]/(pi/4.*d*d);
	}	
	
	nCpoints = props->nCpoints;    // copy in nonlinear stress-strainrate data if applicable
	for (int I=0; I<nCpoints; I++)
	{	dampXs[I] = props->dampXs[I];
		dampYs[I] = props->dampYs[I];  // should these divide by area too?
	}
	
	A = pi/4.*d*d;
	

	// ------------------------- size vectors -------------------------
		
	r   = make2Darray(N+1, 3);     // node positions [i][x/y/z]
	rd  = make2Darray(N+1, 3);     // node velocities [i][x/y/z]
	q   = make2Darray(N+1, 3);     // unit tangent vectors for each node
	qs  = make2Darray(N  , 3);     // unit tangent vectors for each segment
	l   = make1Darray(N);          // line unstretched segment lengths
	lstr= make1Darray(N);          // stretched lengths
	ldstr=make1Darray(N);          // rate of stretch
	Kurv= make1Darray(N+1);        // curvatures at node points (1/m)
	
	M   = make3Darray(N+1, 3, 3);  // mass matrices (3x3) for each node
	V   = make1Darray(N);          // segment volume?
	
	// forces 
	T   = make2Darray(N  , 3);     // segment tensions
	Td  = make2Darray(N  , 3);     // segment damping forces
	Bs  = make2Darray(N+1, 3);     // bending stiffness forces
	W   = make2Darray(N+1, 3);     // node weights
	Dp  = make2Darray(N+1, 3);     // node drag (transverse)
	Dq  = make2Darray(N+1, 3);     // node drag (axial)
	Ap  = make2Darray(N+1, 3);     // node added mass forcing (transverse)
	Aq  = make2Darray(N+1, 3);     // node added mass forcing (axial)
	B   = make2Darray(N+1, 3);     // node bottom contact force
	Fnet= make2Darray(N+1, 3);     // total force on node
	
	// wave things
	F   = make1Darray(N+1);        // VOF scaler for each NODE (mean of two half adjacent segments) (1 = fully submerged, 0 = out of water)
	zeta= make1Darray(N+1);        // wave elevation above each node
	PDyn= make1Darray(N+1);        // dynamic pressure
	U   = make2Darray(N+1, 3);     // wave velocities
	Ud  = make2Darray(N+1, 3);     // wave accelerations
	
	
	// ensure end moments start at zero
	for (int J=0; J<3; J++)        
	{	endMomentA[J] = 0.0;
		endMomentB[J] = 0.0;
	}
	
	// set the number of preset wave kinematic time steps to zero (flagging disabled) to start with
	ntWater = 0; 
	
	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	channels = channels_in; 				// copy string of output channels to object
	
	
	if (wordy >0)
		cout << "   Set up Line " << number << ". ";
};


void Line::setEnv(EnvCond *env_in, Waves *waves_in)
{
	env = env_in;      // set pointer to environment settings object
	waves = waves_in;  // set pointer to Waves  object
}


// get ICs for line using quasi-static approach
void Line::initializeLine(double* X)
{
	// write line information to log file
	if (env->writeLog > 1)
	{
		*(env->outfileLogPtr) << "  - Line" << number << ":" << endl;
		*(env->outfileLogPtr) << "    ID: " << number << endl;
		*(env->outfileLogPtr) << "    UnstrLen: " << UnstrLen << endl;
		*(env->outfileLogPtr) << "    N: " << N << endl;		
		*(env->outfileLogPtr) << "    d   : " << d    << endl;		
		*(env->outfileLogPtr) << "    rho : " << rho  << endl;		
		*(env->outfileLogPtr) << "    E   : " << E    << endl;		
		*(env->outfileLogPtr) << "    EI  : " << EI   << endl;		
		*(env->outfileLogPtr) << "    BAin: " << BAin << endl;		
		*(env->outfileLogPtr) << "    Can : " << Can  << endl;		
		*(env->outfileLogPtr) << "    Cat : " << Cat  << endl;		
		*(env->outfileLogPtr) << "    Cdn : " << Cdn  << endl;		
		*(env->outfileLogPtr) << "    Cdt : " << Cdt  << endl;		
		*(env->outfileLogPtr) << "    ww_l: " << ( (rho - env->rho_w)*(pi/4.*d*d) )*9.81 << endl;		
	}
	
	
	// create output file for writing output (and write channel header and units lines) if applicable

	if (outfile) // check it's not null.  Null signals no individual line output files
	{
		if (outfile->is_open())
		{	
			// ------------- write channel names line --------------------
		
			// output time
			*outfile << "Time" << "\t ";
			
			// output positions?
			//if (find(channels.begin(), channels.end(), "position") != channels.end())
			if (channels.find("p") != string::npos)
			{
				for (int i=0; i<=N; i++)	//loop through nodes
				{
					*outfile << "Node" << i << "px \t Node" <<  i << "py \t Node" <<  i << "pz \t ";
				}
			}
			// output curvatures?
			if (channels.find("K") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "Ku \t ";
				}
			}
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "vx \t Node" <<  i << "vy \t Node" <<  i << "vz \t ";
				}
			}
			// output wave velocities?
			if (channels.find("U") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "Ux \t Node" <<  i << "Uy \t Node" <<  i << "Uz \t ";
				}
			}
			// output hydro force
			if (channels.find("D") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "Dx \t Node" <<  i << "Dy \t Node" <<  i << "Dz \t ";
				}
			}
			// output internal damping force?
			if (channels.find("c") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << "Seg" << i << "cx \t Node" <<  i << "cy \t Node" <<  i << "cz \t ";
				}
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << "Seg" << i << "Te \t ";
				}
			}			
			// output segment strains?
			if (channels.find("s") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << "Seg" << i << "St \t ";
				}
			}	
			// output segment strain rates?
			if (channels.find("d") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << "Seg" << i << "dSt \t ";
				}
			}
			// output seabed contact forces?
			if (channels.find("b") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "bx \t Node" <<  i << "by \t Node" <<  i << "bz \t ";
				}
			}
			
			*outfile << "\n";   
			
			
			// ----------- write units line ---------------

			if (env->WriteUnits > 0)
			{
				// output time
				*outfile << "(s)" << "\t ";
				
				// output positions?
				//if (find(channels.begin(), channels.end(), "position") != channels.end())
				if (channels.find("p") != string::npos)
				{
					for (int i=0; i<=3*N+2; i++)	//loop through nodes
						*outfile << "(m) \t";
				}
				// output curvatures?
				if (channels.find("K") != string::npos) {
					for (int i=0; i<=N; i++)  {
						*outfile << "(1/m) \t ";
					}
				}
				// output velocities?
				if (channels.find("v") != string::npos) {
					for (int i=0; i<=3*N+2; i++)	//loop through nodes
						*outfile << "(m/s) \t";
					
				}
				// output wave velocities?
				if (channels.find("U") != string::npos) {
					for (int i=0; i<=3*N+2; i++)	//loop through nodes
						*outfile << "(m/s) \t";
				}
				// output hydro force
				if (channels.find("D") != string::npos) {
					for (int i=0; i<=3*N+2; i++)	//loop through nodes
						*outfile << "(N) \t";
				}
				// output internal damping force?
				if (channels.find("c") != string::npos) {
					for (int i=0; i<N; i++)	//loop through nodes
						*outfile << "(N) \t";
				}
				// output segment tensions?
				if (channels.find("t") != string::npos) {
					for (int i=0; i<N; i++)	//loop through nodes
						*outfile << "(N) \t";
				}			
				// output segment strains?
				if (channels.find("s") != string::npos) {
					for (int i=0; i<N; i++)	//loop through nodes
						*outfile << "(-) \t";
				}	
				// output segment strain rates?
				if (channels.find("d") != string::npos) {
					for (int i=0; i<N; i++)	//loop through nodes
						*outfile << "(-/s) \t";
				}
				// output seabed contact force?
				if (channels.find("D") != string::npos) {
					for (int i=0; i<=3*N+2; i++)	//loop through nodes
						*outfile << "(N) \t";
				}
				
				*outfile << "\n";   // should also write units at some point!
			}
		}
		else cout << "   Error: unable to write file Line" << number << ".out" << endl;  //TODO: handle this!
	}
	
	
//	// set end node positions and velocities from connect objects  <<<<<<<< these are now set by connect/rod
//	AnchConnect->getConnectState(r[0],rd[0]);
//	FairConnect->getConnectState(r[N],rd[N]);

	// The end node kinematics should already have been set by the 
	// corresponding Connection or Rod objects calling "setEndState",
	// so now we can proceed with figuring out the positions of the nodes along the line.

	if (-env->WtrDpth > r[0][2]) {
		cout << "   Error: water depth is shallower than Line " << number << " anchor." << endl;
		return;
	}
	
	
	// set water kinematics flag based on global wave and current settings (for now)
	if((env->WaveKin==2) || (env->WaveKin==3) || (env->WaveKin==6) || (env->Current==1) || (env->Current==2))
		WaterKin = 2;   // water kinematics to be considered through precalculated global grid stored in Waves object
	else if((env->WaveKin==4) || (env->WaveKin==5) || (env->Current==3) || (env->Current==4))
		WaterKin = 1;   // water kinematics to be considered through precalculated time series for each node
	else
	{	
		WaterKin = 0;   // no water kinematics to be considered (or to be set externally on each node)
	
		for (int i=0; i<=N; i++)   // in this case make sure kinematics for each node start at zeroed
		{
			for (int J=0; J<3; J++)		
			{	U[ i][J] = 0.0;
				Ud[i][J] = 0.0;
			}					
			F[i] = 1.0;   // set VOF variable to 1 for now (everything is submerged) <<<<<<<<
		}
	}
	
	
	// process unstretched line length input
	if (UnstrLen < 0)  // if a negative input, interpret as scaler relative to distance between initial line end points (which have now been set by the relevant Connection objects)
	{
		UnstrLen = -UnstrLen*sqrt( pow(( r[N][0] - r[0][0]), 2.0) + pow(( r[N][1] - r[0][1]), 2.0) + pow(( r[N][2] - r[0][2]), 2.0) );
		cout << "   Line " << number << " unstretched length set to " << UnstrLen << " m." << endl;
	}
	// otherwise just use the value provided (in m)
	
	// now that line length is known, assign length and volume properties
	for (int i=0; i<N; i++)	
	{
		l[i] = UnstrLen/double(N);	// distribute line length evenly over segments
		V[i] = l[i]*0.25*pi*d*d;    // previously missing second d
	}
		
	if (nEApoints > 0)  // For the sake of the following initialization steps, if using a nonlinear stiffness model,
		E = stiffYs[nEApoints-1]/stiffXs[nEApoints-1];   // set the stiffness based on the last entries in the lookup table
	
	// process internal damping input
	// automatic internal damping option (if negative BA provided (stored as BAin), then -BAin indicates desired damping ratio)
	if (BAin < 0) {
		c = -BAin * UnstrLen/N * sqrt(E*rho);   // rho = w/A
		if (wordy > 0) cout << "   Line " << number << " damping set to " << c << " Ns = " << c*(pi/4.*d*d) << " Pa-s based on input of " << BAin << endl;
	}
	else 
		c = BAin/(pi/4.*d*d);  // otherwise it's the regular internal damping coefficient, which should be divided by area to get a material coefficient
			
	
	// initialize line node positions as distributed linearly between the endpoints
	for (int i=1; i<N; i++)
	{
		r[i][0]  = r[0][0] + (r[N][0] - r[0][0]) * (float(i)/float(N));
		r[i][1]  = r[0][1] + (r[N][1] - r[0][1]) * (float(i)/float(N));
		r[i][2]  = r[0][2] + (r[N][2] - r[0][2]) * (float(i)/float(N));
	}
	
	// if conditions are ideal, try to calculate initial line profile using catenary routine (from FAST v.7)
	if (-r[0][0] == env->WtrDpth)
	{
		double XF = sqrt( pow(( r[N][0] - r[0][0]), 2.0) + pow(( r[N][1] - r[0][1]), 2.0) ); // horizontal spread
		double ZF = r[N][2] - r[0][2];	
		double LW = ((rho - env->rho_w) * (pi / 4. * d * d)) * 9.81;
		double CB = 0.;
		double Tol = 0.00001;	
		
		if(( XF > 0.0) && (ZF > 0.0))   
		{
			// locations of line nodes along line length - evenly distributed here 
			vector<double> snodes(N+1, 0.0);   					
			for (int i=1; i<=N; i++)
				snodes[i] = snodes[i-1] + l[i-1]; 
			snodes[N] = UnstrLen; 								// double check to ensure the last node does not surpass the line length
			
			// output variables
			double HF, VF, HA, VA, COSPhi, SINPhi;
			vector<double> Xl(N+1, 0.0); // x location of line nodes
			vector<double> Zl(N+1, 0.0);
			vector<double> Te(N+1, 0.0);
				
			COSPhi = (r[N][0] - r[0][0]) / XF;
			SINPhi = (r[N][1] - r[0][1]) / XF; 
		
			int success = Catenary(XF, ZF, UnstrLen, E * pi / 4. * d * d, LW, CB, Tol,
							   &HF, &VF, &HA, &VA, N, snodes, Xl, Zl, Te);

			if (success >= 0)   // if the catenary solve is successful, update the node positions <<< do we ever get here? <<<
			{
				cout << "   Catenary initial profile available for Line " << number << endl;
				for (int i=1; i<N; i++)
				{
					r[i][0]  = r[0][0] + Xl[i] * COSPhi;
					r[i][1]  = r[0][1] + Xl[i] * SINPhi;
					r[i][2]  = r[0][2] + Zl[i];
				}
			}
		}
	}

		
	// also assign the resulting internal node positions to the integrator initial state vector! (velocities leave at 0)
	for (int i=1; i<N; i++) {
		for (int J=0; J<3; J++) {
			X[3*N-3 + 3*i-3 + J] = r[i][J];  // positions
			X[        3*i-3 + J] = 0.0;      // velocities=0
		}
	}
	// now we need to return to the integrator for the dynamic relaxation stuff
	cout << "Initialized Line " << number << endl;
};


// smart (selective) function to get tension at any node including fairlead or anchor (accounting for weight in these latter cases) (added Nov 15th)
double Line::getNodeTen(int i)
{
	double NodeTen = 0.0;
	
	if (i==0) 
		NodeTen = sqrt(Fnet[i][0]*Fnet[i][0] + Fnet[i][1]*Fnet[i][1] + (Fnet[i][2]+M[i][0][0]*(-env->g))*(Fnet[i][2]+M[i][0][0]*(-env->g)));
	else if (i==N)                             
		NodeTen = sqrt(Fnet[i][0]*Fnet[i][0] + Fnet[i][1]*Fnet[i][1] + (Fnet[i][2]+M[i][0][0]*(-env->g))*(Fnet[i][2]+M[i][0][0]*(-env->g)));
	else 
	{
		double Tmag_squared = 0.; 
		for (int J=0; J<3; J++)  Tmag_squared += 0.25*(T[i][J] + T[i-1][J])*(T[i][J] + T[i-1][J]);  // take average of tension in adjacent segments 
		NodeTen = sqrt(Tmag_squared);  	// 		previously used: NodeTen = 0.5*(Tmag[i-1]+Tmag[i]); // should add damping in here too <<<<<<<<<<<<<
	}
	return NodeTen;
};


// function to get position of any node along the line
int Line::getNodePos(int NodeNum, double pos[3])
{
	if ((NodeNum >= 0 ) && (NodeNum <= N))
	{
		for (int i=0; i<3; i++)
			pos[i] = r[NodeNum][i];
		
		return 0;
	}
	else
		return -1;  // indicate an error
}

// function to return array of coordinates of all nodes along the line (size 3n+3)
void Line::getNodeCoordinates(double r_out[])
{
	for (int i=0; i<=N; i++)
		for (int j=0; j<3; j++)
			r_out[3*i + j] = r[i][j];
	
	return;
}

// function to set U and Ud of each node along the line (used when these are provided externally)
void Line::setNodeWaveKin(double U_in[], double Ud_in[])
{
	for (int i=0; i<=N; i++)
	{	for (int j=0; j<3; j++)
		{	U[ i][j] = U_in[ 3*i + j];
		 	Ud[i][j] = Ud_in[3*i + j];
		}
	}
	return;
}


// FASTv7 style line tension outputs
void Line::getFASTtens(float* FairHTen, float* FairVTen, float* AnchHTen, float* AnchVTen)
{		
	*FairHTen = (float)sqrt(Fnet[N][0]*Fnet[N][0] + Fnet[N][1]*Fnet[N][1]);
	*FairVTen = (float)(Fnet[N][2] + M[N][0][0]*(-env->g));
	*AnchHTen = (float)sqrt(Fnet[0][0]*Fnet[0][0] + Fnet[0][1]*Fnet[0][1]);
	*AnchVTen = (float)(Fnet[0][2] + M[0][0][0]*(-env->g));
	
	return;
};
	
   
   
// replaces getFairStuff and getAnchStuff
// void Line::getEndStuff(double Fnet_out[3], double M_out[3][3], int topOfLine)
// {
//    int i;
//    if (topOfLine==1)
//       i=N;
//    else
//       i=0;
//    
// 	for (int I=0; I<3; I++) 
// 	{	Fnet_out[I] = Fnet[i][I];			
// 		for (int J=0; J<3; J++) 	M_out[I][J] = M[i][I][J];
// 	}
// };
// version that includes moments
void Line::getEndStuff(double Fnet_out[3], double Moment_out[3], double M_out[3][3], int topOfLine)
{
   if (topOfLine==1)  // end B of line
   {
      for (int I=0; I<3; I++) 
      {	
         Fnet_out[  I] = Fnet[N][I];			
         Moment_out[I] = endMomentB[I];
         
         for (int J=0; J<3; J++) M_out[I][J] = M[N][I][J];
      }
   }
   else               // end A of line
   {
      for (int I=0; I<3; I++) 
      {	
         Fnet_out[  I] = Fnet[0][I];			
         Moment_out[I] = endMomentA[I];
         
         for (int J=0; J<3; J++) M_out[I][J] = M[0][I][J];
      }
   }
};

   

int Line::getN()
{
	return N;
};


double Line::GetLineOutput(OutChanProps outChan)
{	
	if      (outChan.QType == PosX)  return  r[outChan.NodeID][0];
	else if (outChan.QType == PosY)  return  r[outChan.NodeID][1];
	else if (outChan.QType == PosZ)  return  r[outChan.NodeID][2];
	else if (outChan.QType == VelX)  return  rd[outChan.NodeID][0];
	else if (outChan.QType == VelY)  return  rd[outChan.NodeID][1];
	else if (outChan.QType == VelZ)  return  rd[outChan.NodeID][2];
	else if (outChan.QType == Ten )  return  getNodeTen(outChan.NodeID);
	else if (outChan.QType == FX)  return  Fnet[outChan.NodeID][0];
	else if (outChan.QType == FY)  return  Fnet[outChan.NodeID][1];
	else if (outChan.QType == FZ)  return  Fnet[outChan.NodeID][2];
	else
	{
		//cout << "outChan.QType (value of " << outChan.QType << ") not recognized." << endl;
		return 0.0;
		//ErrStat = ErrID_Warn
		//ErrMsg = ' Unsupported output quantity from Connect object requested.'
	}	
}


// function to store wave/current kinematics time series for this line, if applicable <<<<<<<<<<<< work in progress!!!
void Line::storeWaterKin(int ntin, double dtin, double **zeta_in, double **f_in, double ***u_in, double ***ud_in)
{	
	if (wordy>1) cout << "   Setting up wave variables for Line " << number << "!  ---------------------" << endl;
	if (wordy>1) cout << "   nt=" << ntin << ", and WaveDT=" <<  dtin << ", env->WtrDpth=" << env->WtrDpth << endl;

	ntWater = ntin;
	dtWater = dtin;

	// resize the new time series vectors
	zetaTS = make2Darray(N+1, ntWater);
	FTS    = make2Darray(N+1, ntWater);
	UTS    = make3Darray(N+1, ntWater, 3);
	UdTS   = make3Darray(N+1, ntWater, 3);

	for (int i=0; i<N+1; i++)
	{
		for (int j=0; j<ntWater; j++)
		{
			zetaTS[i][j] = zeta_in[i][j];
			FTS[i][j] = f_in[i][j];
			for (int k=0; k<3; k++)
			{
				UTS[i][j][k] = u_in[i][j][k];
				UdTS[i][j][k] = ud_in[i][j][k];
			}
		}
	}

	return;
};


double Line::getNonlinearE(double l_stretched, double l_unstretched)
{
	
	double Xi = l_stretched/l_unstretched - 1.0;  // strain rate based on inputs
	double Yi = 0.0;
	
	// find stress based on strain
	if (Xi < 0.0)               // if negative strain (compression), zero stress
		Yi = 0.0;
	else if (Xi < stiffXs[0])  // if strain below first data point, interpolate from zero
		Yi = Xi * stiffYs[0]/stiffXs[0];
	else if (Xi >= stiffXs[nEApoints-1])     // if strain exceeds last data point, use last data point
		Yi = stiffYs[nEApoints-1];
	else                              // otherwise we're in range of the table so interpolate!
	{
		for (int I=0; I < nEApoints-1; I++) // go through lookup table until next entry exceeds inputted strain rate
		{   
			if (stiffXs[I+1] > Xi)
			{
				Yi = stiffYs[I] + (Xi-stiffXs[I]) * (stiffYs[I+1]-stiffYs[I])/(stiffXs[I+1]-stiffXs[I]);
				break;
			}
		}
	}
	
	// calculate equivalent elasticity (since that's what MoorDyn works with)
	return Yi/Xi;  // this is a young's modulus equivalent value based on stress/strain
}


double Line::getNonlinearC(double ld_stretched, double l_unstretched)
{
	
	double Xi = ld_stretched/l_unstretched;  // strain rate based on inputs
	double Yi = 0.0;
	
	// find stress based on strain rate
	// first check if lookup table includes compressing 
	if (dampXs[0] < 0)
	{
		if (Xi < dampXs[0])  // if strain below first data point, use first data point
			Yi = dampYs[0];
		else if (Xi >= dampXs[nCpoints-1])     // if strain exceeds last data point, use last data point
			Yi = dampYs[nCpoints-1];
		else                              // otherwise we're in range of the table so interpolate!
		{
			for (int I=0; I < nCpoints-1; I++) // go through lookup table until next entry exceeds inputted strain rate
			{   if (dampXs[I+1] > Xi)
				{	Yi = dampYs[I] + (Xi - dampXs[I]) * (dampYs[I+1]-dampYs[I])
														   /(dampXs[I+1]-dampXs[I]);
					break;
				}
			}
		}
	}
	else   // if no compressing data given, we'll flip-mirror so stretching and compressing are same
	{
		double Xsign = 1.0;
		if (Xi < 0)          // convert negative value to positive for interpolation
		{	Xsign = -1.0;
			Xi = -Xi;
		}
		
		if (Xi < dampXs[0])  // if strain rate is below first data point, interpolate from zero
			Yi = Xi * dampYs[0]/dampXs[0];
		else if (Xi >= dampXs[nCpoints-1])     // if strain rate exceeds last data point, use last data point
			Yi = dampYs[nCpoints-1];
		else                              // otherwise we're in range of the table so interpolate!
		{
			for (int I=0; I < nCpoints-1; I++) // go through lookup table until next entry exceeds inputted strain rate
			{   if (dampXs[I+1] > Xi)
				{	Yi = dampYs[I] + (Xi - dampXs[I]) * (dampYs[I+1]-dampYs[I])
														   /(dampXs[I+1]-dampXs[I]);
					break;
				}
			}
		}
		Yi = Yi*Xsign; // flip sine if negative strain
	}
		
	// calculate equivalent damping coefficient (since that's what MoorDyn works with)
	return Yi/Xi;  // this is a coefficient value based on stress/strainrate
}

	
// function for boosting drag coefficients during IC generation	
void Line::scaleDrag(double scaler)
{
	Cdn = Cdn*scaler;
	Cdt = Cdt*scaler;
	return;
}

// function to reset time after IC generation
void Line::setTime(double time)
{
	t = time;
	return;
}



// set the line positions and velocities based on latest states
void Line::setState(const double* X, const double time)
{
	// store current time
	t = time;
	
	// set interior node positions and velocities based on state vector
	for (int i=1; i<N; i++)
	{
		for (int J=0; J<3; J++)
		{
			r[i][J]  = X[3 * N - 3 + 3 * i - 3 + J]; // get positions
			rd[i][J] = X[            3 * i - 3 + J]; // get velocities
		}
	}
}


// set position and velocity of one of the line end nodes  <<<<<<<<< should rename SetEndKinematics
void Line::setEndState(double r_in[3], double rd_in[3], int topOfLine)
{
	int i;

	if (topOfLine==1)
	{
		i = N;        // fairlead case
		endTypeB = 0; // indicate pinned
	}
	else
	{
		i = 0;        // anchor case
		endTypeA = 0; // indicate pinned
	}

	memcpy(r[i], r_in, 3 * sizeof(double));
	memcpy(rd[i], rd_in, 3 * sizeof(double));
}

void Line::setEndState(vector<double> &r_in, vector<double> &rd_in, int topOfLine)
{
	setEndState(r_in.data(), rd_in.data(), topOfLine);
}

// set end node unit vector of a line (this is called by an attached to a Rod, only applicable for bending stiffness)
void Line::setEndOrientation(double *qin, int topOfLine, int rodEndB)
{  	// Note: qin is the rod's position/orientation vector, r6, passed at index 3
	// rodEndB=0 means the line is attached to Rod end A, =1 means attached to Rod end B (implication for unit vector sign)
	
	if (topOfLine==1)
	{
		endTypeB = 1;                  // indicate attached to Rod (at every time step, just in case line get detached)
		
		if (rodEndB==1)
			for (int J=0; J<3; J++)  q[N][J] = -qin[J];   // -----line----->[B<==ROD==A]
		else
			for (int J=0; J<3; J++)  q[N][J] =  qin[J];   // -----line----->[A==ROD==>B]
	}
	else
	{	
		endTypeA = 1;                  // indicate attached to Rod (at every time step, just in case line get detached)                 // indicate attached to Rod
		
		if (rodEndB==1)
			for (int J=0; J<3; J++)  q[0][J] =  qin[J];   // [A==ROD==>B]-----line----->
		else
			for (int J=0; J<3; J++)  q[0][J] = -qin[J];   // [B<==ROD==A]-----line----->
		
	}
	return;
}	


void Line::getEndSegmentInfo(double q_EI_dl[3], int topOfLine, int rodEndB)
{  	
	double dlEnd;
	double EIend;
	double qEnd[3];

	if (topOfLine==1)
	{	
		dlEnd = unitvector(qEnd, r[N-1], r[N]);  // unit vector of last line segment
		if (rodEndB == 0) EIend =  EI; // -----line----->[A==ROD==>B]
		else              EIend = -EI; // -----line----->[B==ROD==>A]
	}
	else
	{	
		dlEnd = unitvector(qEnd, r[0], r[1]);    // unit vector of first line segment
		if (rodEndB == 0) EIend = -EI; // <----line-----[A==ROD==>B]
		else              EIend =  EI; // <----line-----[B==ROD==>A]
	}
	
	for (int i=0; i<3; i++)
		q_EI_dl[i] = qEnd[i]*EIend/dlEnd;

	
	return;
}	

void Line::getEndSegmentInfo(double qEnd[3], double *EIout, double *dlout, int topOfLine)
{  	

	*EIout = EI;
	
	if (topOfLine==1)
	{	
		*dlout = unitvector(qEnd, r[N-1], r[N]);  // unit vector of last line segment
		
	}
	else
	{	
		*dlout = unitvector(qEnd, r[0], r[1]);    // unit vector of first line segment
	}
	
	return;
}


// calculate forces and get the derivative of the line's states
void Line::getStateDeriv(double* Xd, const double PARAM_UNUSED dt)
{
	
	//for (int i=0; i<=N; i++) cout << " " << r[i][0] << " " << r[i][1] << " " << r[i][2] << endl;
	
	// attempting error handling <<<<<<<<
	for (int i=0; i<=N; i++)
	{
		if (isnan(r[i][0] + r[i][1] + r[i][2])) 
		{
			stringstream s;
			s << "NaN detected" << endl
			  << "Line " << number << " node positions:" << endl;
			for (int j=0; j<=N; j++)
				s << j << " : "
				  << r[j][0] << ", " << r[j][1] << ", " << r[j][2] << ";"
				  << endl;
			throw moordyn::nan_error(s.str().c_str());
		}
	}
	
	if (env->writeLog > 2)
	{
		*(env->outfileLogPtr) << "Line " << number << " node positions at time " << t << ":\n";
		for (int j=0; j<=N; j++) *(env->outfileLogPtr) << r[j][0] << "\t" << r[j][1] << "\t" << r[j][2] << "\n";
	}
	
	// dt is possibly used for stability tricks...
	
	// -------------------- calculate various kinematic quantities ---------------------------
	for (int i=0; i<N; i++) 
	{
		//calculate current (Stretched) segment lengths and unit tangent vectors (qs) for each segment (this is used for bending calculations)
		lstr[i] = unitvector(qs[i], r[i], r[i+1]);
				
		double ldstr_top = 0.0;          // this is the denominator of how the stretch rate equation was formulated
		for (int J=0; J<3; J++)
			ldstr_top += (r[i+1][J] - r[i][J]) * (rd[i+1][J] - rd[i][J]);
		ldstr[i] = ldstr_top/lstr[i]; 	// strain rate of segment
						
		V[i] = A*l[i];		// volume attributed to segment
	}
		
	// calculate unit tangent vectors (q) for each internal node. note: I've reversed these from pointing toward 0 rather than N. Check sign of wave loads. <<<<
	for (int i=1; i<N; i++) 
		unitvector(q[i], r[i-1], r[i+1]);    // compute unit vector q ... using adjacent two nodes!
	
	// calculate unit tangent vectors for either end node if the line has no bending stiffness of if either end is pinned (otherwise it's already been set via setEndStateFromRod)
	if ((endTypeA == 0) || (EI==0)) unitvector(q[0], r[0  ], r[1]);  
	if ((endTypeB == 0) || (EI==0)) unitvector(q[N], r[N-1], r[N]);
	
		
	//============================================================================================
	// --------------------------------- apply wave kinematics -----------------------------
	
	if (WaterKin == 1) // wave kinematics time series set internally for each node
	{
		// =========== obtain (precalculated) wave kinematics at current time instant ============
		// get precalculated wave kinematics at previously-defined node positions for time instant t
		
		// get interpolation constant and wave time step index
		int it = floor(t/dtWater);
		double frac = remainder(t,dtWater)/dtWater;
				
		// loop through nodes 
		for (int i=0; i<=N; i++)
		{
			zeta[i] = zetaTS[i][it] + frac*( zetaTS[i][it+1] - zetaTS[i][it] );			
			F[i] = 1.0;   // FTS[i][it] + frac*(FTS[i][it+1] - FTS[i][it]);
			
			for (int J=0; J<3; J++)
			{
				U[i][J] = UTS[i][it][J] + frac*( UTS[i][it+1][J] - UTS[i][it][J] );				
				Ud[i][J] = UdTS[i][it][J] + frac*( UdTS[i][it+1][J] - UdTS[i][it][J] );
			}
		}	
	}
	else if (WaterKin == 2) // wave kinematics interpolated from global grid in Waves object
	{		
		for (int i=0; i<=N; i++)
		{
			waves->getWaveKin(r[i][0], r[i][1], r[i][2], t, U[i], Ud[i], &zeta[i], &PDyn[i]); // call generic function to get water velocities
			
			F[i] = 1.0; // set VOF value to one for now (everything submerged - eventually this should be element-based!!!) <<<<
		}
	}
	else if (WaterKin != 0) // Hopefully WaterKin is set to zero, meaning no waves or set externally, otherwise it's an error
		cout << "ERROR: We got a problem with WaterKin not being 0,1,2." << endl;

	
	//============================================================================================
	
	
    // calculate mass matrix 
	for (int i=0; i<=N; i++) 
	{
		double m_i; // node mass
		double v_i; // node submerged volume 
		
		if (i==0) 
		{
			m_i = pi/8.*d*d*l[0]*rho;
			v_i = 1./2. *F[i]*V[i];
		}
		else if (i==N) 
		{
			m_i = pi/8.*d*d*l[N-2]*rho;
			v_i = 1./2. *F[i-1]*V[i-1];
		}
		else
		{
			m_i = pi/8.*( d*d*rho*(l[i] + l[i-1]));
			v_i = 1./2. *(F[i-1]*V[i-1] + F[i]*V[i]);
		}
		
		// make node mass matrix
		for (int I=0; I<3; I++) {
			for (int J=0; J<3; J++) { 
				M[i][I][J] = m_i*eye(I,J) + env->rho_w*v_i *( Can*(eye(I,J) - q[i][I]*q[i][J]) + Cat*q[i][I]*q[i][J] );	//	    <<<<<<< check since I've reversed q
			}
		}		
		
	}
	

	// ============  CALCULATE FORCES ON EACH NODE ===============================
	
	// loop through the segments
	for (int i=0; i<N; i++)
	{
		
		/*
		// attempting error handling <<<<<<<<
		if (abs(lstr[i]/l[i] - 1) > 0.5) {
			stringstream s;
			s << "Line " << number << " segment strains:";
			for (int j=0; j<N; j++) s << " " << (lstr[i]/l[i] - 1);
			s << " at time " << t;
			throw string(s.str()); //"Too great a strain in segment");
		}
		*/
		
		// line tension
		if (nEApoints > 0)
			E = getNonlinearE(lstr[i], l[i]);
	
		if (lstr[i]/l[i] > 1.0)
			for (int J=0; J<3; J++)  T[i][J] = E*A* ( 1./l[i] - 1./lstr[i] ) * (r[i+1][J]-r[i][J]); 
		else
			for (int J=0; J<3; J++)  T[i][J] = 0.;	// cable can't "push" ... or can it, if bending stiffness is nonzero? <<<<<<<<<
				
		// line internal damping force
		if (nCpoints > 0)
			c = getNonlinearC(ldstr[i], l[i]);
		
		for (int J=0; J<3; J++)  Td[i][J] = c*A* ( ldstr[i] / l[i] ) * (r[i+1][J]-r[i][J])/lstr[i]; 
	}
	
	
	// Bending loads
	// first zero out the forces from last run
	for (int i=0; i<=N; i++)
		for (int J=0; J<3; J++)
			Bs[i][J] = 0.0;
	// and now compute them (if possible)
	if (EI > 0)
	{
		// loop through all nodes to calculate bending forces
		for (int i=0; i<=N; i++)
		{
			double Kurvi = 0.0;
			double pvec[3];
			double Mforce_im1[3];
			double Mforce_ip1[3];
			double Mforce_i[  3];
			
			// calculate force on each node due to bending stiffness!
			
			// end node A case (only if attached to a Rod, i.e. a cantilever rather than pinned connection)
			if (i==0)
			{
				if (endTypeA > 0) // if attached to Rod i.e. cantilever connection
				{
					Kurvi = GetCurvature(lstr[i], q[i], qs[i]);  // curvature <<< check if this approximation works for an end (assuming rod angle is node angle which is middle of if there was a segment -1/2
		
					crossProd(q[0], qs[i], pvec);           // get direction of bending radius axis
					
					crossProd(qs[i  ], pvec, Mforce_ip1);    // get direction of resulting force from bending to apply on node i+1
					
					// record bending moment at end for potential application to attached object   <<<< do double check this....
					scalevector(pvec, Kurvi*EI, endMomentA );
					
					// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
					scalevector(Mforce_ip1, Kurvi*EI/lstr[i  ], Mforce_ip1 );					
						
					// set force on node i to cancel out forces on adjacent nodes
					for (int J=0; J<3; J++) Mforce_i[J] = - Mforce_ip1[J];
					
					// apply these forces to the node forces
					for (int J=0; J<3; J++) 
					{
						Bs[i  ][J] += Mforce_i[  J];
						Bs[i+1][J] += Mforce_ip1[J];
					}
				}
			}
			// end node A case (only if attached to a Rod, i.e. a cantilever rather than pinned connection)
			else if (i==N)
			{
				if (endTypeB > 0) // if attached to Rod i.e. cantilever connection
				{
					Kurvi = GetCurvature(lstr[i-1], qs[i-1], q[i]);  // curvature <<< check if this approximation works for an end (assuming rod angle is node angle which is middle of if there was a segment -1/2
					
					crossProd(qs[i-1], q[N], pvec);         // get direction of bending radius axis
					
					crossProd(qs[i-1], pvec, Mforce_im1);    // get direction of resulting force from bending to apply on node i-1
					
					// record bending moment at end for potential application to attached object   <<<< do double check this....
					scalevector(pvec, -Kurvi*EI, endMomentB ); // note end B is oposite sign as end A
					
					// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
					scalevector(Mforce_im1, Kurvi*EI/lstr[i-1], Mforce_im1);
						
					// set force on node i to cancel out forces on adjacent nodes
					for (int J=0; J<3; J++) Mforce_i[J] = - Mforce_im1[J];
					
					// apply these forces to the node forces
					for (int J=0; J<3; J++) 
					{
						Bs[i-1][J] += Mforce_im1[J];
						Bs[i  ][J] += Mforce_i[  J];
					}
				}
			}
			else   // internal node
			{
				Kurvi = GetCurvature(lstr[i-1] + lstr[i], qs[i-1], qs[i]);  // curvature <<< remember to check sign, or just take abs

				crossProd(qs[i-1], qs[i], pvec);         // get direction of bending radius axis

				crossProd(qs[i-1], pvec, Mforce_im1);    // get direction of resulting force from bending to apply on node i-1
				crossProd(qs[i  ], pvec, Mforce_ip1);    // get direction of resulting force from bending to apply on node i+1

				// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
				scalevector(Mforce_im1, Kurvi * EI / lstr[i-1], Mforce_im1);
				scalevector(Mforce_ip1, Kurvi * EI / lstr[i  ], Mforce_ip1 );

				// set force on node i to cancel out forces on adjacent nodes
				for (int J=0; J<3; J++)
					Mforce_i[J] = - Mforce_im1[J] - Mforce_ip1[J];

				// apply these forces to the node forces
				for (int J=0; J<3; J++) 
				{
					Bs[i-1][J] += Mforce_im1[J]; 
					Bs[i  ][J] += Mforce_i[  J];
					Bs[i+1][J] += Mforce_ip1[J];
				}
			}
					
			// check for NaNs <<<<<<<<<<<<<<< temporary measure <<<<<<<
			for (int J=0; J<3; J++) 
			{
				if (isnan(Bs[i][J]))
				{
					cout << "   Error: NaN value detected in bending force at Line " << number << " node " << i << endl;
					cout << lstr[i-1]+lstr[i] << endl;
					cout << sqrt( 0.5*(1 - dotProd( qs[i-1], qs[i] ) ) ) << endl;

					cout << Bs[i-1][J] << endl;
					cout << Bs[i  ][J] << endl;
					cout << Bs[i+1][J] << endl;
					cout << Mforce_im1[J] << endl;
					cout << Mforce_i[  J] << endl;
					cout << Mforce_ip1[J] << endl;
				}
			}
			
			// record curvature at node!!
			Kurv[i] = Kurvi;
			
			// any damping forces for bending? I hope not...
			
			// get normal component at each adjacent node
		
			// trace along line to find torsion at each segment
			/*
			if (torsion)
			{	
				// assume first segment's twist coordinate is fixed
				if (i==0)
					s[0] = {0,0,1}; //or something
				else  // i =1..N-1
				{
					// R x = u(u . x) + cos(alpha) (u cross x) cross u + sin(alpha) (u cross x)
					// \uvec{s}_\iplus = \uvec{p}_i(\uvec{p}_i \cdot \uvec{s}_\iminus) 
					// + (\uvec{q}_\iplus \cdot \uvec{q}_\iminus) (\uvec{p}_i \times \uvec{s}_\iminus) \times \uvec{p}_i 
					// + \sin(\alpha_i) (\uvec{p}_i \times \uvec{s}_\iminus)
					
					double p_p_dot_s[3];
					scalevector( pvec, dotprod(pvec, s[i-1]), p_p_dot_s)  // \uvec{p}_i(\uvec{p}_i \cdot \uvec{s}_\iminus)
					
					double p_cross_s[3];
					crossprod(pvec, s[i-1], p_cross_s)// \uvec{p}_i \times \uvec{s}_\iminus
					
					double p_cross_s_cross_p[3];
					crossprod(p_cross_s, pvec, p_cross_s_cross_p)// (\uvec{p}_i \times \uvec{s}_\iminus) \times \uvec{p}_i
					
					for j in  <3)   // \uvec{s}_\iplus = \uvec{p}_i(\uvec{p}_i \cdot \uvec{s}_\iminus) + (\uvec{q}_\iplus \cdot \uvec{q}_\iminus) pcx \times \uvec{p}_i + \sin(\alpha_i) pcx
					{
						s[i][j] = p_p_dot_s[j] + cos_alpha*p_cross_s_cross_p[j] + sin_alpha*p_cross_s
					}				
				}
				// compare end alignment with torsion calculation to figure out actual cable twist, and distribute it uniformly along the cable .. but how?
				
			}
			*/
		}   // for i=0,N (looping through nodes)
	}  // if EI > 0



	// loop through the nodes
	for (int i=0; i<=N; i++)
	{
		W[i][0] = W[i][1] = 0.0;
		// submerged weight (including buoyancy)
		if (i==0)
			W[i][2] = 0.5*A*( l[i]*(rho-F[i]*env->rho_w) )*(-env->g);
		else if (i==N)
			W[i][2] = 0.5*A*( l[i-1]*(rho-F[i-1]*env->rho_w) )*(-env->g); // missing the "W[i][2] =" previously!
		else
			W[i][2] = 0.5*A*( l[i]*(rho-F[i]*env->rho_w) + l[i-1]*(rho-F[i-1]*env->rho_w) )*(-env->g);
				
		// flow velocity calculations       
		double vq_squared = 0.;
		double vp_squared = 0.;

		for (int J=0; J<3; J++)
			vi[J] = U[i][J] - rd[i][J];            // relative flow velocity over node

		for (int J=0; J<3; J++) 
		{
			vq[J] = dotProd(vi , q[i]) * q[i][J];  // tangential relative flow component  <<<<<<< check sign since I've reversed q
			vp[J] = vi[J] - vq[J];                 // transverse relative flow component
			vq_squared += vq[J] * vq[J];
			vp_squared += vp[J] * vp[J];
		}
		double vq_mag = sqrt(vq_squared);
		double vp_mag = sqrt(vp_squared);
		
		// transverse drag		
		if (i==0) 		
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env->rho_w*Cdn* (F[i]*d*l[i])/2. * vp_mag * vp[J]; 
		else if (i==N) 
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env->rho_w*Cdn* (F[i-1]*d*l[i-1])/2. * vp_mag * vp[J]; 
		else 
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env->rho_w*Cdn* (F[i]*d*l[i] + F[i-1]*d*l[i-1])/2. * vp_mag * vp[J]; 
		
		// tangential drag		
		if (i==0)
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env->rho_w*Cdt* pi*(F[i]*d*l[i])/2. * vq_mag * vq[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env->rho_w*Cdt* pi*(F[i-1]*d*l[i-1])/2. * vq_mag * vq[J]; 
		else
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env->rho_w*Cdt* pi*(F[i]*d*l[i] + F[i-1]*d*l[i-1])/2. * vq_mag * vq[J]; 
				
		
		// acceleration calculations					
		for (int J=0; J<3; J++)  {
			aq[J] = dotProd(Ud[i], q[i]) * q[i][J]; // tangential component of fluid acceleration    <<<<<<< check sign since I've reversed q
			ap[J] = Ud[i][J] - aq[J]; 			// normal component of fluid acceleration
		}
		
		// transverse Froude-Krylov force
		if (i==0)	
			for (int J=0; J<3; J++)  Ap[i][J] = env->rho_w*(1.+Can)*0.5*( V[i]) * ap[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Ap[i][J] = env->rho_w*(1.+Can)*0.5*(V[i-1] ) * ap[J]; 
		else
			for (int J=0; J<3; J++)  Ap[i][J] = env->rho_w*(1.+Can)*0.5*( V[i] + V[i-1] ) * ap[J]; 
		
		// tangential Froude-Krylov force					
		if (i==0)	
			for (int J=0; J<3; J++)  Aq[i][J] = env->rho_w*(1.+Cat)*0.5*( V[i]) * aq[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Aq[i][J] = env->rho_w*(1.+Cat)*0.5*( V[i-1] ) * aq[J]; 
		else
			for (int J=0; J<3; J++)  Aq[i][J] = env->rho_w*(1.+Cat)*0.5*( V[i] + V[i-1] ) * aq[J]; 
		
		// bottom contact (stiffness and damping, vertical-only for now) - updated for general case of potentially anchor or fairlead end in contact
		if (r[i][2] < -env->WtrDpth)
		{
			if (i==0)
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*(            d*l[i] );
			else if (i==N)
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*( d*l[i-1]          );
			else
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*( d*l[i-1] + d*l[i] );
			
			// new rough-draft addition of seabed friction
			double FrictionMax = abs(B[i][2])*env->FrictionCoefficient; // dynamic friction force saturation level based on bottom contact force
			
			// saturated damping approach to applying friction, for now
			double BottomVel = sqrt(rd[i][0]*rd[i][0] + rd[i][1]*rd[i][1]); // velocity of node along sea bed
			double FrictionForce = BottomVel * env->FrictionCoefficient*env->FricDamp; // some arbitrary damping scaling thing at end
			if (FrictionForce > env->StatDynFricScale*FrictionMax)  FrictionForce = FrictionMax;     // saturate (quickly) to static/dynamic friction force level 
			
			if (BottomVel == 0.0) { // check for zero velocity, in which case friction force is zero
				B[i][0] = 0.0;
				B[i][1] = 0.0;
			}
			else { // otherwise, apply friction force in correct direction(opposing direction of motion)
				B[i][0] = -FrictionForce*rd[i][0]/BottomVel;
				B[i][1] = -FrictionForce*rd[i][1]/BottomVel;
			}
		}
		else 
		{
			B[i][0] = 0.;
			B[i][1] = 0.;
			B[i][2] = 0.;
		}		
		// total forces
		for (int J=0; J<3; J++)
		{
			if (i==0)
				Fnet[i][J] = T[i][J]             + Td[i][J];
			else if (i==N)
				Fnet[i][J] =          -T[i-1][J]            - Td[i-1][J];
			else
				Fnet[i][J] = T[i][J] - T[i-1][J] + Td[i][J] - Td[i-1][J];
			Fnet[i][J] += W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J] + Bs[i][J];
		}
	}

//	if (t > 5)
//	{
//		cout << " in getStateDeriv of line " << number << endl;
//		
//		B[0][0] = 0.001; // meaningless
//	}


	// loop through internal nodes and update their states
	for (int i=1; i<N; i++)	
	{
	//	double M_out[9];
	//	double F_out[3];
	//	for (int I=0; I<3; I++) 
	//	{	F_out[I] = Fnet[i][I];
	//		for (int J=0; J<3; J++) M_out[3*I + J] = M[i][I][J];
	//	}
		
		// solve for accelerations in [M]{a}={f} using LU decomposition
	//	double LU[9];                        // serialized matrix that will hold LU matrices combined
	//	Crout(3, M_out, LU);                  // perform LU decomposition on mass matrix
		double acc[3];                        // acceleration vector to solve for
	//	solveCrout(3, LU, F_out, acc);     // solve for acceleration vector
						
	//	LUsolve3(M[i], acc, Fnet[i]);
	
		double LU[3][3];
		double ytemp[3];
		LUsolve(3, M[i], LU, Fnet[i], ytemp, acc);
		
		// fill in state derivatives
		for (int I=0; I<3; I++) 
		{
			Xd[            3 * i - 3 + I] = acc[I];    //RHSiI;         dVdt = RHS * A  (accelerations)
			Xd[3 * N - 3 + 3 * i - 3 + I] = rd[i][I];  //X[3*i-3 + I];  dxdt = V  (velocities)
		}
	}
};



// write output file for line  (accepts time parameter since retained time value (t) will be behind by one line time step
void Line::Output(double time)
{
	// run through output flags
	// if channel is flagged for output, write to file.
	// Flags changed to just be one character (case sensitive) per output flag.  To match FASTv8 version.
		
	if (outfile) // if not a null pointer (indicating no output)
	{
		if (outfile->is_open())
		{
			// output time
			*outfile << time << "\t "; 
		
			// output positions?
			//if (find(channels.begin(), channels.end(), "position") != channels.end())
			if (channels.find("p") != string::npos)
			{
				for (int i=0; i<=N; i++)	//loop through nodes
				{
					for (int J=0; J<3; J++)  *outfile << r[i][J] << "\t ";
				}
			}
			// output curvatures?
			if (channels.find("K") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << Kurv[i] << "\t ";
				}
			}
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << rd[i][J] << "\t ";
				}
			}
			// output wave velocities?
			if (channels.find("U") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << U[i][J] << "\t ";
				}
			}
			// output hydro drag force?
			if (channels.find("D") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J] << "\t ";
				}
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (int i=0; i<N; i++)  {
					double Tmag_squared = 0.; 
					for (int J=0; J<3; J++)  Tmag_squared += T[i][J]*T[i][J]; // doing this calculation here only, for the sake of speed
					*outfile << sqrt(Tmag_squared) << "\t ";
				}
			}
			// output internal damping force?
			if (channels.find("c") != string::npos) {
				for (int i=0; i<N; i++)  {
					for (int J=0; J<3; J++)  *outfile << Td[i][J] + Td[i][J] + Td[i][J] << "\t ";
				}
			}
			// output segment strains?
			if (channels.find("s") != string::npos) {
				for (int i=0; i<N; i++)  {
					*outfile << lstr[i]/l[i]-1.0 << "\t ";
				}
			}
			// output segment strain rates?
			if (channels.find("d") != string::npos) {
				for (int i=0; i<N; i++)  {
					*outfile << ldstr[i]/l[i] << "\t ";
				}
			}			
			// output seabed contact forces?
			if (channels.find("b") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << B[i][J] << "\t ";
				}
			}
			
			*outfile << "\n";
		}
		else cout << "Unable to write to output file " << endl;
	}
	return;
};


Line::~Line()
{
	// free memory

	free2Darray(r   , N+1);    
	free2Darray(rd  , N+1);    
	free2Darray(q   , N+1);    
	free2Darray(qs  , N  );    
	free(l);
	free(lstr);
	free(ldstr);
	free(Kurv);
	
	free3Darray(M   , N+1, 3); 
	free(V);
					 
	// forces        
	free2Darray(T   , N  );    
	free2Darray(Td  , N  );    
	free2Darray(Bs  , N+1);    
	free2Darray(W   , N+1);    
	free2Darray(Dp  , N+1);    
	free2Darray(Dq  , N+1);    
	free2Darray(Ap  , N+1);    
	free2Darray(Aq  , N+1);    
	free2Darray(B   , N+1);    
	free2Darray(Fnet, N+1);    
					
	// wave things   
	free(F   );       
	free(zeta);       
	free(PDyn);       
	free2Darray(U   , N+1);    
	free2Darray(Ud  , N+1);    
		
	// wave time series vectors (may never have been used)
	if (ntWater > 0)
	{	free2Darray(zetaTS, N+1);
		free2Darray(FTS   , N+1);
		free3Darray(UTS   , N+1, ntWater);
		free3Darray(UdTS  , N+1, ntWater);
	}
}


// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void Line::drawGL(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i=0; i<=N; i++)	
	{
		double newTen = getNodeTen(i);
		if (newTen > maxTen)
			maxTen = newTen;
	}
	
	glColor3f(0.5,0.5,1.0);
	glBegin(GL_LINE_STRIP);
	for (int i=0; i<=N; i++)	
	{
		glVertex3d(r[i][0], r[i][1], r[i][2]);
		if (i<N)  {
			normTen = getNodeTen(i)/maxTen;
			ColorMap(normTen, rgb);
			glColor3d(rgb[0],rgb[1],rgb[2]);
		}
	}
	glEnd();
};



void Line::drawGL2(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i=0; i<=N; i++)	
	{
		double newTen = getNodeTen(i);
		if (newTen > maxTen)
			maxTen = newTen;
	}
	
	// line
	for (int i=0; i<N; i++)	
	{
		normTen = 0.2+0.8*pow(getNodeTen(i)/maxTen, 4.0);
		ColorMap(normTen, rgb);
		glColor3d(rgb[0],rgb[1],rgb[2]);

		Cylinder(r[i][0], r[i][1], r[i][2], r[i+1][0], r[i+1][1], r[i+1][2], 27, 0.5);
	}
	// velocity vectors
	for (int i=0; i<=N; i++)	
	{
		glColor3d(0.0, 0.2, 0.8);
		double vscal = 5.0;

		Arrow(r[i][0], r[i][1], r[i][2], vscal*rd[i][0], vscal*rd[i][1], vscal*rd[i][2], 0.1, 0.7);
	}
};
#endif
