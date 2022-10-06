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
#include "Waves.hpp"
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
		
	r.assign(N+1, vec(0., 0., 0.));     // node positions [i][x/y/z]
	rd.assign(N+1, vec(0., 0., 0.));    // node positions [i][x/y/z]
	q.assign(N+1, vec(0., 0., 0.));     // unit tangent vectors for each node
	qs.assign(N, vec(0., 0., 0.));      // unit tangent vectors for each segment
	l.assign(N, 0.0);                   // line unstretched segment lengths
	lstr.assign(N, 0.0);                // stretched lengths
	ldstr.assign(N, 0.0);               // rate of stretch
	Kurv.assign(N+1, 0.0);              // curvatures at node points (1/m)
	
	M.assign(N+1, mat());               // mass matrices (3x3) for each node
	V.assign(N, 0.0);                   // segment volume?
	
	// forces 
	T.assign(N, vec(0., 0., 0.));       // segment tensions
	Td.assign(N, vec(0., 0., 0.));      // segment damping forces
	Bs.assign(N+1, vec(0., 0., 0.));    // bending stiffness forces
	W.assign(N+1, vec(0., 0., 0.));     // node weights
	Dp.assign(N+1, vec(0., 0., 0.));    // node drag (transverse)
	Dq.assign(N+1, vec(0., 0., 0.));    // node drag (axial)
	Ap.assign(N+1, vec(0., 0., 0.));    // node added mass forcing (transverse)
	Aq.assign(N+1, vec(0., 0., 0.));    // node added mass forcing (axial)
	B.assign(N+1, vec(0., 0., 0.));     // node bottom contact force
	Fnet.assign(N+1, vec(0., 0., 0.));  // total force on node
	
	// wave things
	F.assign(N+1, 0.0);                 // VOF scaler for each NODE (mean of two half adjacent segments) (1 = fully submerged, 0 = out of water)
	zeta.assign(N+1, 0.0);              // wave elevation above each node
	PDyn.assign(N+1, 0.0);              // dynamic pressure
	U.assign(N+1, vec(0., 0., 0.));     // wave velocities
	Ud.assign(N+1, vec(0., 0., 0.));    // wave accelerations
	
	
	// ensure end moments start at zero
	endMomentA = vec(0., 0., 0.);
	endMomentB = vec(0., 0., 0.);
	
	// set the number of preset wave kinematic time steps to zero (flagging disabled) to start with
	ntWater = 0; 
	
	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	channels = channels_in; 				// copy string of output channels to object
	
	
	if (wordy >0)
		cout << "   Set up Line " << number << ". ";
};


void Line::setEnv(EnvCond *env_in, moordyn::Waves *waves_in)
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
		*(env->outfileLogPtr) << "    N   : " << N << endl;		
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
			string name = getLineName();
			
			// output positions?:w

			//if (find(channels.begin(), channels.end(), "position") != channels.end())
			if (channels.find("p") != string::npos)
			{
				for (int i=0; i<=N; i++)	//loop through nodes
				{
					*outfile << name << "Node" << i << "px \t" 
					         << name << "Node" <<  i << "py \t"
							 << name << "Node" <<  i << "pz \t ";
				}
			}
			// output curvatures?
			if (channels.find("K") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << name << "Node" << i << "Ku \t ";
				}
			}
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << name << "Node" << i << "vx \t" 
					         << name << "Node" <<  i << "vy \t" 
							 << name << "Node" <<  i << "vz \t ";
				}
			}
			// output wave velocities?
			if (channels.find("U") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << name << "Node" << i << "Ux \t" 
						     << name << "Node" <<  i << "Uy \t" 
							 << name << "Node" <<  i << "Uz \t ";
				}
			}
			// output hydro force
			if (channels.find("D") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << name << "Node" << i << "Dx \t" 
					         << name << "Node" <<  i << "Dy \t" 
							 << name << "Node" <<  i << "Dz \t ";
				}
			}
			// output internal damping force?
			if (channels.find("c") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << name << "Seg" << i << "cx \t" 
					         << name << "Node" <<  i << "cy \t" 
							 << name << "Node" <<  i << "cz \t ";
				}
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << name << "Seg" << i << "Te \t ";
				}
			}			
			// output segment strains?
			if (channels.find("s") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << name << "Seg" << i << "St \t ";
				}
			}	
			// output segment strain rates?
			if (channels.find("d") != string::npos) {
				for (int i=1; i<=N; i++)  {
					*outfile << name << "Seg" << i << "dSt \t ";
				}
			}
			// output seabed contact forces?
			if (channels.find("b") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << name << "Node" << i << "bx \t" 
					         << name << "Node" <<  i << "by \t" 
							 << name << "Node" <<  i << "bz \t ";
				}
			}
			
			if (env->outputMode != 1)
				*outfile << "\n";
			
			
			// ----------- write units line ---------------

			if (env->WriteUnits > 0 && env->outputMode != 1)  // Don't write units if writing to single file
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
				
				if (env->outputMode != 1)
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
			U[ i] = vec(0.0, 0.0, 0.0);
			Ud[i] = vec(0.0, 0.0, 0.0);
			F[i] = 1.0;   // set VOF variable to 1 for now (everything is submerged) <<<<<<<<
		}
	}
	
	
	// process unstretched line length input
	vec dir = r[N] - r[0];
	if (UnstrLen < 0)  // if a negative input, interpret as scaler relative to distance between initial line end points (which have now been set by the relevant Connection objects)
	{
		UnstrLen = -UnstrLen*dir.norm();
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
		r[i] = r[0] + dir * (float(i)/float(N));
	}
	
	// if conditions are ideal, try to calculate initial line profile using catenary routine (from FAST v.7)
	if (-r[0][0] == env->WtrDpth)
	{
		double XF = dir(Eigen::seqN(0, 2)).norm(); // horizontal spread
		double ZF = dir[2];	
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
					vec l(Xl[i] * COSPhi, Xl[i] * SINPhi, Zl[i]);
					r[i] = r[0] + l;
				}
			}
		}
	}

		
	// also assign the resulting internal node positions to the integrator initial state vector! (velocities leave at 0)
	for (int i=1; i<N; i++) {
		moordyn::vec2array(r[i],               &(X[3*N-3 + 3*i-3]));
		moordyn::vec2array(vec(0.0, 0.0, 0.0), &(X[        3*i-3]));
	}
	// now we need to return to the integrator for the dynamic relaxation stuff
	cout << "Initialized Line " << number << endl;
};


// smart (selective) function to get tension at any node including fairlead or anchor (accounting for weight in these latter cases) (added Nov 15th)
double Line::getNodeTen(int i)
{
	double NodeTen = 0.0;
	if ((i==0) || (i==N))
		NodeTen = (Fnet[i] + vec(0.0, 0.0, M[i](0, 0) * (-env->g))).norm();  // <<< update to use W
	else 
	{
		// take average of tension in adjacent segments 
		NodeTen = (0.5 * (T[i] + T[i-1])).norm();  	// 		previously used: NodeTen = 0.5*(Tmag[i-1]+Tmag[i]); // should add damping in here too <<<<<<<<<<<<<
	}
	return NodeTen;
};


// function to get position of any node along the line
int Line::getNodePos(int NodeNum, double pos[3])
{
	if ((NodeNum >= 0 ) && (NodeNum <= N))
	{
		moordyn::vec2array(r[NodeNum], pos);
		
		return 0;
	}
	else
		return -1;  // indicate an error
}

// function to return array of coordinates of all nodes along the line (size 3n+3)
void Line::getNodeCoordinates(double r_out[])
{
	for (int i=0; i<=N; i++)
		moordyn::vec2array(r[i], &(r_out[3*i]));
	
	return;
}

// function to set U and Ud of each node along the line (used when these are provided externally)
void Line::setNodeWaveKin(double U_in[], double Ud_in[])
{
	for (int i=0; i<=N; i++)
	{
		moordyn::array2vec(&(U_in[ 3*i]), U[ i]);
		moordyn::array2vec(&(Ud_in[3*i]), Ud[i]);
	}
	return;
}


// FASTv7 style line tension outputs
void Line::getFASTtens(float* FairHTen, float* FairVTen, float* AnchHTen, float* AnchVTen)
{		
	*FairHTen = (float)(Fnet[N](Eigen::seqN(0, 2)).norm());
	*FairVTen = (float)(Fnet[N][2] + M[N](0, 0)*(-env->g));
	*AnchHTen = (float)(Fnet[0](Eigen::seqN(0, 2)).norm());
	*AnchVTen = (float)(Fnet[0][2] + M[0](0, 0)*(-env->g));
	
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
      moordyn::vec2array(Fnet[N], Fnet_out);
      moordyn::vec2array(endMomentB, Moment_out);
      moordyn::mat2array(M[N], M_out);
   }
   else               // end A of line
   {
      moordyn::vec2array(Fnet[0], Fnet_out);
      moordyn::vec2array(endMomentA, Moment_out);
      moordyn::mat2array(M[0], M_out);
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
	zetaTS.assign(N+1, std::vector<moordyn::real>(ntWater, 0.0));
	FTS.assign(N+1, std::vector<moordyn::real>(ntWater, 0.0));
	UTS.assign(N+1, std::vector<vec>(ntWater, vec(0.0, 0.0, 0.0)));
	UdTS.assign(N+1, std::vector<vec>(ntWater, vec(0.0, 0.0, 0.0)));

	for (int i=0; i<N+1; i++)
	{
		for (int j=0; j<ntWater; j++)
		{
			zetaTS[i][j] = zeta_in[i][j];
			FTS[i][j] = f_in[i][j];
			moordyn::array2vec(u_in[i][j], UTS[i][j]);
			moordyn::array2vec(ud_in[i][j], UdTS[i][j]);
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
		moordyn::array2vec(&(X[3 * N - 3 + 3 * i - 3]), r[i]);
		moordyn::array2vec(&(X[            3 * i - 3]), rd[i]);
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

	moordyn::array2vec(r_in, r[i]);
	moordyn::array2vec(rd_in, rd[i]);
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
		endTypeB = 1;                    // indicate attached to Rod (at every time step, just in case line get detached)
		moordyn::array2vec(qin, q[N]);   // -----line----->[A==ROD==>B]
		if (rodEndB==1)
			q[N] *= -1;                  // -----line----->[B<==ROD==A]
	}
	else
	{	
		endTypeA = 1;                    // indicate attached to Rod (at every time step, just in case line get detached)
		moordyn::array2vec(qin, q[0]);   // [A==ROD==>B]-----line----->
		if (rodEndB!=1)
			q[0] *= -1;                  // [B<==ROD==A]-----line----->
	}
	return;
}	


void Line::getEndSegmentInfo(double q_EI_dl[3], int topOfLine, int rodEndB)
{  	
	double dlEnd;
	double EIend;
	vec qEnd;

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

	moordyn::vec2array(qEnd * EIend / dlEnd, q_EI_dl);
	
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
	// Jose Luis Cercos-Pita: This is by far the most consuming function of the
	// whole library, just because it is called every single time substep and
	// it shall makecomputations in every single line node. Thus it is worthy
	// to invest effort on keeping it optimized.

	//for (int i=0; i<=N; i++) cout << " " << r[i][0] << " " << r[i][1] << " " << r[i][2] << endl;
	
	// attempting error handling <<<<<<<<
	for (int i=0; i<=N; i++)
	{
		if (isnan(r[i].sum())) 
		{
			stringstream s;
			s << "NaN detected" << endl
			  << "Line " << number << " node positions:" << endl;
			for (int j=0; j<=N; j++)
				s << j << " : " << r[j] << ";" << endl;
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

		// this is the denominator of how the stretch rate equation was formulated
		const double ldstr_top = (r[i+1]  - r[i]).dot(rd[i+1] - rd[i]);
		ldstr[i] = ldstr_top/lstr[i]; 	// strain rate of segment
						
		V[i] = A * l[i];		// volume attributed to segment
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
			
			U[i] = UTS[i][it] + frac*( UTS[i][it+1] - UTS[i][it] );
			Ud[i] = UdTS[i][it] + frac*( UdTS[i][it+1] - UdTS[i][it] );
		}	
	}
	else if (WaterKin == 2) // wave kinematics interpolated from global grid in Waves object
	{		
		for (int i=0; i<=N; i++)
		{
			waves->getWaveKin(r[i][0], r[i][1], r[i][2], t, U[i], Ud[i], zeta[i], PDyn[i]); // call generic function to get water velocities
			
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
		
		// Make node mass matrix
		const mat I = mat::Identity();
		const mat Q = q[i] * q[i].transpose();
		M[i] = m_i * I + env->rho_w * v_i * (Can * (I - Q) + Cat * Q);
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
	
		if (lstr[i] / l[i] > 1.0)
		{
			T[i] = E*A* ( 1./l[i] - 1./lstr[i] ) * (r[i+1]-r[i]);
		}
		else
		{
			// cable can't "push" ...
			// or can it, if bending stiffness is nonzero? <<<<<<<<<
			T[i] = vec(0.0, 0.0, 0.0);
		}
				
		// line internal damping force
		if (nCpoints > 0)
			c = getNonlinearC(ldstr[i], l[i]);
		
		Td[i] = c*A* ( ldstr[i] / l[i] ) * (r[i+1]-r[i])/lstr[i]; 
	}
	
	
	// Bending loads
	// first zero out the forces from last run
	for (int i=0; i<=N; i++)
		Bs[i] = vec(0.0, 0.0, 0.0);

	// and now compute them (if possible)
	if (EI > 0)
	{
		// loop through all nodes to calculate bending forces
		for (int i=0; i<=N; i++)
		{
			moordyn::real Kurvi = 0.0;
			vec pvec;
			vec Mforce_im1(0.0, 0.0, 0.0);
			vec Mforce_ip1(0.0, 0.0, 0.0);
			vec Mforce_i;
			
			// calculate force on each node due to bending stiffness!
			
			// end node A case (only if attached to a Rod, i.e. a cantilever rather than pinned connection)
			if (i==0)
			{
				if (endTypeA > 0) // if attached to Rod i.e. cantilever connection
				{
					Kurvi = GetCurvature(lstr[i], q[i], qs[i]);  // curvature <<< check if this approximation works for an end (assuming rod angle is node angle which is middle of if there was a segment -1/2
		
					pvec = q[0].cross(qs[i]);           // get direction of bending radius axis
					
					Mforce_ip1 = qs[i].cross(pvec);     // get direction of resulting force from bending to apply on node i+1
					
					// record bending moment at end for potential application to attached object   <<<< do double check this....
					scalevector(pvec, Kurvi*EI, endMomentA );
					
					// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
					scalevector(Mforce_ip1, Kurvi*EI/lstr[i  ], Mforce_ip1 );					
						
					// set force on node i to cancel out forces on adjacent nodes
					Mforce_i = - Mforce_ip1;
					
					// apply these forces to the node forces
					Bs[i  ] = Mforce_i;
					Bs[i+1] = Mforce_ip1;
				}
			}
			// end node A case (only if attached to a Rod, i.e. a cantilever rather than pinned connection)
			else if (i==N)
			{
				if (endTypeB > 0) // if attached to Rod i.e. cantilever connection
				{
					Kurvi = GetCurvature(lstr[i-1], qs[i-1], q[i]); // curvature <<< check if this approximation works for an end (assuming rod angle is node angle which is middle of if there was a segment -1/2
					
					pvec = qs[i-1].cross(q[N]);          // get direction of bending radius axis
					
					Mforce_im1 = qs[i-1].cross(pvec);    // get direction of resulting force from bending to apply on node i-1
					
					// record bending moment at end for potential application to attached object   <<<< do double check this....
					scalevector(pvec, -Kurvi*EI, endMomentB ); // note end B is oposite sign as end A
					
					// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
					scalevector(Mforce_im1, Kurvi*EI/lstr[i-1], Mforce_im1);
						
					// set force on node i to cancel out forces on adjacent nodes
					Mforce_i = -Mforce_im1;
					
					// apply these forces to the node forces
					Bs[i-1] = Mforce_im1;
					Bs[i  ] = Mforce_i;
				}
			}
			else   // internal node
			{
				Kurvi = GetCurvature(lstr[i-1] + lstr[i], qs[i-1], qs[i]);  // curvature <<< remember to check sign, or just take abs

				pvec = qs[i-1].cross(q[i]);          // get direction of bending radius axis

				Mforce_im1 = qs[i-1].cross(pvec);    // get direction of resulting force from bending to apply on node i-1
				Mforce_ip1 = qs[i  ].cross(pvec);    // get direction of resulting force from bending to apply on node i+1

				// scale force direction vectors by desired moment force magnitudes to get resulting forces on adjacent nodes
				scalevector(Mforce_im1, Kurvi * EI / lstr[i-1], Mforce_im1);
				scalevector(Mforce_ip1, Kurvi * EI / lstr[i  ], Mforce_ip1 );

				// set force on node i to cancel out forces on adjacent nodes
				Mforce_i = - Mforce_im1 - Mforce_ip1;

				// apply these forces to the node forces
				Bs[i-1] = Mforce_im1;
				Bs[i  ] = Mforce_i;
				Bs[i+1] = Mforce_ip1;
			}

			// check for NaNs <<<<<<<<<<<<<<< temporary measure <<<<<<<
			if (isnan(Bs[i][0]) || isnan(Bs[i][1]) || isnan(Bs[i][2]))
			{
				cout << "   Error: NaN value detected in bending force at Line "
				     << number << " node " << i << endl;
				cout << lstr[i-1]+lstr[i] << endl;
				cout << sqrt(0.5 * (1 - qs[i - 1].dot(qs[i]))) << endl;

				cout << Bs[i - 1] << endl;
				cout << Bs[i] << endl;
				cout << Bs[i + 1] << endl;
				cout << Mforce_im1 << endl;
				cout << Mforce_i << endl;
				cout << Mforce_ip1 << endl;
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

		// relative flow velocity over node
		const vec vi = U[i] - rd[i];
		// tangential relative flow component
		// <<<<<<< check sign since I've reversed q
		const moordyn::real vql = vi.dot(q[i]);
		const vec vq = vql * q[i];
		// transverse relative flow component
		const vec vp = vi - vq;

		const moordyn::real vq_mag = vq.norm();
		const moordyn::real vp_mag = vp.norm();

		// transverse drag
		if (i == 0)
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d *
				(F[i] * l[i]                     ) * vp;
		else if (i == N)
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d *
				(              F[i - 1] * l[i -1]) * vp;
		else
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d *
				(F[i] * l[i] + F[i - 1] * l[i -1]) * vp;

		// tangential drag
		if (i == 0)
			Dq[i] = 0.25 * vq_mag * env->rho_w * Cdt * pi * d *
				(F[i] * l[i]                     ) * vq;
		else if (i == N)
			Dq[i] = 0.25 * vq_mag * env->rho_w * Cdt * pi * d *
				(              F[i - 1] * l[i -1]) * vq;
		else
			Dq[i] = 0.25 * vq_mag * env->rho_w * Cdt * pi * d *
				(F[i] * l[i] + F[i - 1] * l[i -1]) * vq;

		// tangential component of fluid acceleration
		// <<<<<<< check sign since I've reversed q
		const moordyn::real aql = Ud[i].dot(q[i]);
		const vec aq = aql * q[i];
		// normal component of fluid acceleration
		const vec ap = Ud[i] - aq;
		
		// transverse Froude-Krylov force
		if (i==0)
			Ap[i] = env->rho_w*(1.+Can)*0.5*( V[i]          ) * ap;
		else if (i == N)
			Ap[i] = env->rho_w*(1.+Can)*0.5*(V[i-1] ) * ap;
		else
			Ap[i] = env->rho_w*(1.+Can)*0.5*( V[i] + V[i-1] ) * ap;
		// tangential Froude-Krylov force					
		if (i == 0)
			Aq[i] = 0.5 * env->rho_w*(1.+Cat)*0.5*( V[i]) * aq;
		else if (i == N)
			Aq[i] = 0.5 * env->rho_w*(1.+Cat)*0.5*( V[i-1] ) * aq;
		else
			Aq[i] =       env->rho_w*(1.+Cat)*0.5*( V[i] + V[i-1] ) * aq;
		
		// bottom contact (stiffness and damping, vertical-only for now) - updated for general case of potentially anchor or fairlead end in contact
		if (r[i][2] < -env->WtrDpth)
		{
			if (i==0)
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*d*(            l[i] );
			else if (i==N)
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*d*( l[i-1]          );
			else
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * 0.5*d*( l[i-1] + l[i] );
			
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
			B[i] = vec(0.0, 0.0, 0.0);

		// total forces
		if (i==0)
			Fnet[i] = T[i]            + Td[i];
		else if (i==N)
			Fnet[i] =      - T[i - 1]         - Td[i - 1];
		else
			Fnet[i] = T[i] - T[i - 1] + Td[i] - Td[i - 1];
		Fnet[i] += W[i] + (Dp[i] + Dq[i] + Ap[i] + Aq[i]) + B[i] + Bs[i];
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
	//	double acc[3];                        // acceleration vector to solve for
	//	solveCrout(3, LU, F_out, acc);     // solve for acceleration vector
						
	//	LUsolve3(M[i], acc, Fnet[i]);

	//	Solve3(M[i], acc, (const double*)Fnet[i]);

		// For small systems it is usually faster to compute the inverse
		// of the matrix. See
		// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		const vec acc = M[i].inverse() * Fnet[i];

		// fill in state derivatives
		moordyn::vec2array(acc,   &Xd[            3 * i - 3]);  //RHSiI;         dVdt = RHS * A  (accelerations)
		moordyn::vec2array(rd[i], &Xd[3 * N - 3 + 3 * i - 3]);  //X[3*i-3 + I];  dxdt = V  (velocities)
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
					*outfile << T[i].norm() << "\t ";
					// >>> preparation below for switching to outputs at nodes <<<
					// note that tension of end nodes will need weight and buoyancy adjustment 
					//if (i==0)
					//      *outfile << (T[i] + W[i]).norm() << "\t ";
					//else if (i==N)
					//      *outfile << (T[i] - W[i]).norm() << "\t ";
					//else
					//	*outfile << T[i].norm() << "\t ";
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
			
			if (env->outputMode != 1)
				*outfile << "\n";
		}
		else cout << "Unable to write to output file " << endl;
	}
	return;
};

ofstream* Line::getOutputFilestream()
{
	return outfile;
};

string Line::getLineName()
{
	if (env->outputMode == 0)
		return "";
	return "Line" + std::to_string(number);
};

Line::~Line()
{
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
