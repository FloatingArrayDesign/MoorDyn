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
 
#include "Line.h"
#include "Connection.h"
#include "QSlines.h" // the c++ version of quasi-static model Catenary

using namespace std;

// here is the new numbering scheme (N segments per line)

//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]



// ================== Line member functions ===========================


// set up line object
void Line::setup(int number_in, LineProps props, double UnstrLen_in, int NumNodes, 
	Connection &AnchConnect_in, Connection &FairConnect_in,
	shared_ptr<ofstream> outfile_pointer, string channels_in) 
{
	// ================== set up properties ===========	
	number = number_in;	
	UnstrLen = UnstrLen_in;
	N = NumNodes; // assign number of nodes to line
	
	WaveKin = 0;  // start off with wave kinematics disabled.  Can be enabled after initial conditions are found and wave kinematics are calculated
	
	AnchConnect = &AnchConnect_in;		// assign line end connections
	FairConnect = &FairConnect_in;	
		
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	channels = channels_in; 				// copy string of output channels to object
			
	d = props.d;
	rho = props.w/(pi/4.*d*d);
	E = props.EA/(pi/4.*d*d);
	c = props.c/(pi/4.*d*d);
	Can = props.Can;
	Cat = props.Cat;
	Cdn = props.Cdn;
	Cdt = props.Cdt;
	ReFac = props.ReFac;
	
	// automatic internal damping option (if negative BA provided, as damping ratio)
	if (props.c < 0) {
		double zeta = -props.c; // desired damping ratio
		c = zeta * UnstrLen/N * sqrt(E*rho);   // rho = w/A
		if (wordy > 1) cout << "   Line " << number << "damping set to " << c << " Ns." << endl;
	}
		
	
	// =============== size vectors =========================
		
	r.resize( N+1, vector<double>(3, 0.0));		// node positions [i][x/y/z]
	rd.resize(N+1, vector<double>(3, 0.0));		// node velocities [i][x/y/z]
	q.resize( N+1, vector<double>(3, 0.0));     	// unit tangent vectors for each node
	
	// forces 
	T.resize(N, vector<double>(3, 0.0));	// line tensions
	Td.resize(N, vector<double>(3, 0.0));   // line damping forces
//	Tmag.resize(N, 0.0);				// segment tension magnitudes << hardly used
	W.resize(N+1, vector<double>(3, 0.0));	// node weights

	Dp.resize(N+1, vector<double>(3, 0.0));		// node drag (transverse)
	Dq.resize(N+1, vector<double>(3, 0.0));		// node drag (axial)
	Ap.resize(N+1, vector<double>(3, 0.0));		// node added mass forcing (transverse)
	Aq.resize(N+1, vector<double>(3, 0.0));		// node added mass forcing (axial)
	B.resize(N+1, vector<double>(3, 0.0));		// node bottom contact force
	Fnet.resize(N+1, vector<double>(3, 0.0));	// total force on node
		
	S.resize(N+1, vector< vector< double > >(3, vector<double>(3, 0.0)));  // inverse mass matrices (3x3) for each node
	M.resize(N+1, vector< vector< double > >(3, vector<double>(3, 0.0)));  // mass matrices (3x3) for each node
			
	l.resize(N, 0.0); 		// line unstretched segment lengths
	lstr.resize(N, 0.0); 		// stretched lengths
	ldstr.resize(N, 0.0); 		// rate of stretch
	V.resize(N, 0.0);			// volume?
	
	zeta.resize(N+1, 0.0);					// wave elevation above each node
	F.resize(N+1, 0.0); 	// fixed 2014-12-07	// VOF scalar for each NODE (mean of two half adjacent segments) (1 = fully submerged, 0 = out of water)
	U.resize(N+1, vector<double>(3, 0.));     	// wave velocities
	Ud.resize(N+1, vector<double>(3, 0.));;     	// wave accelerations
	
	for (int i=0; i<N; i++)	
	{	l[i] = UnstrLen/double(N);	// distribute line length evenly over segments
		V[i] = l[i]*0.25*pi*d*d;
	}
	
	// moved output file creation to Line::Initialize
	
	return;
};


// get ICs for line using quasi-static approach
void Line::initialize( double* X )	
{
	
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
				for (int i=0; i<N; i++)  {
					*outfile << "Seg" << i << "cx \t Node" <<  i << "cy \t Node" <<  i << "cz \t ";
				}
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (int i=0; i<N; i++)  {
					*outfile << "Seg" << i << "Te \t ";
				}
			}			
			// output segment strains?
			if (channels.find("s") != string::npos) {
				for (int i=0; i<N; i++)  {
					*outfile << "Seg" << i << "St \t ";
				}
			}	
			// output segment strain rates?
			if (channels.find("d") != string::npos) {
				for (int i=0; i<N; i++)  {
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

			if (env.WriteUnits > 0)
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
	
	
	// set end node positions and velocities from connect objects
	AnchConnect->getConnectState(r[0],rd[0]);
	FairConnect->getConnectState(r[N],rd[N]);
	
		
	if (-env.WtrDpth > r[0][2]) {
		cout << "   Error: water depth is shallower than Line " << number << " anchor." << endl;
		return;
	}
	
	// try to calculate initial line profile using catenary routine (from FAST v.7)
	// note: much of this function is adapted from the FAST source code
		
	// input variables for the Catenary function
	double XF = sqrt( pow(( r[N][0] - r[0][0]), 2.0) + pow(( r[N][1] - r[0][1]), 2.0) ); // quasi-static mooring line coordinate system (vertical plane with corners at anchor and fairlead) 
	double ZF = r[N][2] - r[0][2];	
	double W = ( (rho - env.rho_w)*(pi/4.*d*d) )*9.81; 
	double CB = 0.;
	double Tol = 0.00001;	
	
	vector<double> snodes(N+1, 0.0);   					// locations of line nodes along line length - evenly distributed here 
	for (int i=1; i<=N; i++) snodes[i] = snodes[i-1] + l[i-1]; 
	snodes[N] = UnstrLen; 								// double check to ensure the last node does not surpass the line length
	
	
	// output variables
	double HF, VF, HA, VA, COSPhi, SINPhi;
	vector<double> Xl(N+1, 0.0); // x location of line nodes
	vector<double> Zl(N+1, 0.0);
	vector<double> Te(N+1, 0.0);
			
	if( XF == 0.0 ) // if the current mooring line is exactly vertical; thus, the solution below is ill-conditioned because the orientation is undefined; so set it such that the tensions and nodal positions are only vertical
	{   COSPhi = 0.0;   SINPhi = 0.0; }
	else 	// The current mooring line must not be vertical; use simple trigonometry
	{   	COSPhi = ( r[N][0] - r[0][0] )/XF;
		SINPhi = ( r[N][1] - r[0][1] )/XF; 
	}
			
	int success = Catenary( XF, ZF, UnstrLen, E*pi/4.*d*d, W , CB, Tol, &HF, &VF, &HA, &VA, N, snodes, Xl, Zl, Te);
	
	if (success>=0)
	{	// assign the resulting line positions to the model
		for (int i=1; i<N; i++)
		{
			r[i][0]  = r[0][0] + Xl[i]*COSPhi;
			r[i][1]  = r[0][1] + Xl[i]*SINPhi;
			r[i][2]  = r[0][2] + Zl[i];
		}
	}
	else
	{	// otherwise just stretch the nodes between the endpoints linearly and hope for the best
		if (wordy > 0)  cout << "   Catenary IC gen failed for Line" << number << ", so using linear node spacing." << endl;
		for (int i=1; i<N; i++)
		{
			r[i][0]  = r[0][0] + (r[N][0] - r[0][0]) * (float(i)/float(N));
			r[i][1]  = r[0][1] + (r[N][1] - r[0][1]) * (float(i)/float(N));
			r[i][2]  = r[0][2] + (r[N][2] - r[0][2]) * (float(i)/float(N));
		}
	}
		
	// also assign the resulting internal node positions to the integrator initial state vector! (velocities leave at 0)
	for (int i=1; i<N; i++) {
		for (int J=0; J<3; J++) {
			X[3*N-3 + 3*i-3 + J] = r[i][J];  // positions
			X[        3*i-3 + J] = 0.0;       // velocities=0
		}
	}
	// now we need to return to the integrator for the dynamic relaxation stuff
		
	return;
};


// smart (selective) function to get tension at any node including fairlead or anchor (accounting for weight in these latter cases) (added Nov 15th)
double Line::getNodeTen(int i)
{
	double NodeTen = 0.0;
	
	if (i==0) 
		NodeTen = sqrt(Fnet[i][0]*Fnet[i][0] + Fnet[i][1]*Fnet[i][1] + (Fnet[i][2]+M[i][0][0]*(-env.g))*(Fnet[i][2]+M[i][0][0]*(-env.g)));
	else if (i==N)                             
		NodeTen = sqrt(Fnet[i][0]*Fnet[i][0] + Fnet[i][1]*Fnet[i][1] + (Fnet[i][2]+M[i][0][0]*(-env.g))*(Fnet[i][2]+M[i][0][0]*(-env.g)));
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


// FASTv7 style line tension outputs
void Line::getFASTtens(float* FairHTen, float* FairVTen, float* AnchHTen, float* AnchVTen)
{		
	*FairHTen = (float)sqrt(Fnet[N][0]*Fnet[N][0] + Fnet[N][1]*Fnet[N][1]);
	*FairVTen = (float)(Fnet[N][2] + M[N][0][0]*(-env.g));
	*AnchHTen = (float)sqrt(Fnet[0][0]*Fnet[0][0] + Fnet[0][1]*Fnet[0][1]);
	*AnchVTen = (float)(Fnet[0][2] + M[0][0][0]*(-env.g));
	
	return;
};
	
	
void Line::getAnchStuff(vector<double> &Fnet_out, vector< vector<double> > &M_out)
{
	for (int I=0; I<3; I++) {
		Fnet_out[I] = Fnet[0][I];			
		for (int J=0; J<3; J++) 	M_out[I][J] = M[0][I][J];
	}
};

void Line::getFairStuff(vector<double> &Fnet_out, vector< vector<double> > &M_out)
{
	for (int I=0; I<3; I++) {
		Fnet_out[I] = Fnet[N][I];			
		for (int J=0; J<3; J++) 	M_out[I][J] = M[N][I][J];
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


// initialize wave parameters for no waves situation
void Line::setupWaves(EnvCond env_in, vector<double> Ucurrent_in, float dt_in)
{	
	env = env_in;
	Ucurrent = Ucurrent_in;
	WaveDT = dt_in; // new variable for wave time step (should be same as WaveDT I think...)	
	
	if (env.WaveKin > 0)  // if including wave kinematics
	{
		cout << "   WARNING - Line::setupWaves dummy function called when waves are supposed to be enabled!" << endl;
		system("pause");		
	}

	if (wordy>1) cout << "   Setting up wave variables for Line " << number << "!  ---------------------" << endl;
	if (wordy>1) cout << "   Nt=" << Nt << ", and WaveDT=" <<  WaveDT << ", env.WtrDpth=" << env.WtrDpth << endl;
	
	WGNC_Fact = 1.0;
	S2Sd_Fact = 1.0;	

	Nw = 0;    // use no components since not worrying about wave kinematics
	Nt = 2; // this is a new variable containing the number of wave time steps to be calculated
			
	WGNC_Fact = 1.0;
	S2Sd_Fact = 1.0;
	// resize the new time series vectors
	zetaTS.resize(N+1, vector<double>(Nt, 0.));
	FTS.resize   (N+1, vector<double>(Nt, 0.));
	UTS.resize   (N+1, vector< vector< double> >(Nt, vector<double>(3, 0.)));
	UdTS.resize  (N+1, vector< vector< double> >(Nt, vector<double>(3, 0.)));
	tTS.resize(Nt, 0.);
};


	
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



//  this is the big function that updates the states
void Line::doRHS( const double* X,  double* Xd, const double time, const double dt)
{
	t = time;

	// set end node positions and velocities from connect objects' states
	AnchConnect->getConnectState(r[0],rd[0]);
	FairConnect->getConnectState(r[N],rd[N]);

	// set interior node positions and velocities
	for (int i=1; i<N; i++) 
	{	for (int J=0; J<3; J++)
		{
			r[i][J]  = X[3*N-3 + 3*i-3 + J]; // get positions
			rd[i][J] = X[        3*i-3 + J]; // get velocities
		}
	}
	
	//calculate current (Stretched) segment lengths
	for (int i=0; i<N; i++) 
	{
		double lstr_squared = 0.0;
		for (int J=0; J<3; J++) lstr_squared += (r[i+1][J] - r[i][J])*(r[i+1][J] - r[i][J]);
		lstr[i] = sqrt(lstr_squared); 	// stretched segment length
		
		double ldstr_top = 0.0;
		for (int J=0; J<3; J++) ldstr_top += (r[i+1][J] - r[i][J])*(rd[i+1][J] - rd[i][J]);
		ldstr[i] = ldstr_top/lstr[i]; 	// strain rate of segment
						
		V[i] = pi/4. *( d*d*l[i] );		// volume attributed to segment
	}
		
	// calculate unit tangent vectors (q) for each node (including ends)  note: I think these are pointing toward 0 rather than N!
	for (int i=0; i<=N; i++) 
	{
		if (i==0) 	unitvector(q[i], r[i+1], r[i]  ); 	// compute unit vector q
		else if (i==N) unitvector(q[i], r[i]  , r[i-1]); 	// compute unit vector q
		else 		unitvector(q[i], r[i+1], r[i-1]);  // compute unit vector q ... using adjacent two nodes!
	}

	
	//============================================================================================
	// --------------------------------- apply wave kinematics ------------------------------------
	
	if (WaveKin == 0)   // if Wave Kinematics haven't been calculated.   ...this is a local Line switch (so wave kinematics can be enabled/disabled for individual lines)
	{
		for (int i=0; i<=N; i++)
		{
			zeta[i] = 0.0;			
			F[i] = 1.0;
			
			for (int J=0; J<3; J++)
			{
				U[i][J] = 0.0;				
				Ud[i][J] = 0.0;
			}
		}
		
		//if (wordy)
		//	if (number==1)
		//		cout << " t=" << t << ", U[4][0]=" << U[4][0] << endl;
		
	}
	else  // if wave kinematics time series have been precalculated.
	{
		// =========== obtain (precalculated) wave kinematics at current time instant ============
		// get precalculated wave kinematics at previously-defined node positions for time instant t
		
		// get interpolation constant and wave time step index
		double frac;
		for (int ts=0; ts<Nt-1; ts++)			// loop through precalculated wave time series time steps  (start ts at ts0 to save time)
		{	if (tTS[ts+1] > t) 				// moving precalculated time bracked "up".  Stop once upper end of bracket is greater than current time t.
			{
				ts0 = ts;
				frac = ( t - tTS[ts] )/( tTS[ts+1] - tTS[ts] );
				break;
			}
		}
				
		// loop through nodes 
		for (int i=0; i<=N; i++)
		{
			zeta[i] = zetaTS[i][ts0] + frac*( zetaTS[i][ts0+1] - zetaTS[i][ts0] );			
			F[i] = 1.0;   // FTS[i][ts0] + frac*(FTS[i][ts0+1] - FTS[i][ts0]);
			
			for (int J=0; J<3; J++)
			{
				U[i][J] = UTS[i][ts0][J] + frac*( UTS[i][ts0+1][J] - UTS[i][ts0][J] );				
				Ud[i][J] = UdTS[i][ts0][J] + frac*( UdTS[i][ts0+1][J] - UdTS[i][ts0][J] );
			}
			
//			if (wordy) {
//				if (i==N) {
//					cout << "ts0: " << ts0 << ", frac: " << frac << ", getting " << U[i][0] << " from " << UTS[i][ts0+1][0] << " - " << UTS[i][ts0][0] << endl;
//					system("pause");
//				}
//			}
			
		}	
		if (wordy > 2)
			if (number==1)
				  cout << " t=" << t << ", U[4][0]=" << U[4][0] << endl;
	}
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
				M[i][I][J] = m_i*eye(I,J) + env.rho_w*v_i *( Can*(eye(I,J) - q[i][I]*q[i][J]) + Cat*q[i][I]*q[i][J] );					
			}
		}		
		
		inverse3by3(S[i], M[i]);	// invert node mass matrix (written to S[i][:][:])	
	}
	

	// ============  CALCULATE FORCES ON EACH NODE ===============================
	
	// loop through the segments
	for (int i=0; i<N; i++)
	{
		// line tension
		if (lstr[i]/l[i] > 1.0)   
			for (int J=0; J<3; J++)  T[i][J] = E*pi/4.*d*d* ( 1./l[i] - 1./lstr[i] ) * (r[i+1][J]-r[i][J]); 
		else
			for (int J=0; J<3; J++)  T[i][J] = 0.;	// cable can't "push"
				
		// line internal damping force
		for (int J=0; J<3; J++)  Td[i][J] = c*pi/4.*d*d* ( ldstr[i] / l[i] ) * (r[i+1][J]-r[i][J])/lstr[i]; 
	}

	
	// loop through the nodes
	for (int i=0; i<=N; i++)
	{
		// submerged weight (including buoyancy)
		if (i==0)
			W[i][2] = pi/8.*( d*d*l[i]*(rho-F[i]*env.rho_w) )*(-env.g);
		else if (i==N)
			W[i][2] = pi/8.*( d*d*l[i-1]*(rho-F[i-1]*env.rho_w) )*(-env.g); // missing the "W[i][2] =" previously!
		else
			W[i][2] = pi/8.*( d*d*l[i]*(rho-F[i]*env.rho_w) + d*d*l[i-1]*(rho-F[i-1]*env.rho_w) )*(-env.g);
				
		// flow velocity calculations       
		double vq_squared = 0.;
		double vp_squared = 0.;
		
		for (int J=0; J<3; J++)  vi[J] = U[i][J] - rd[i][J]; // relative flow velocity over node
		
		for (int J=0; J<3; J++) 
		{	
			vq[J] = dotprod( vi , q[i] ) * q[i][J]; 	// tangential relative flow component
			vp[J] = vi[J] - vq[J];					// transverse relative flow component
			vq_squared += vq[J]*vq[J];
			vp_squared += vp[J]*vp[J];
		}
		double vp_mag = sqrt(vp_squared);
		double vq_mag = sqrt(vp_squared);
		
		// transverse drag		
		if (i==0) 		
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env.rho_w*Cdn* (F[i]*d*l[i])/2. * vp_mag * vp[J]; 
		else if (i==N) 
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env.rho_w*Cdn* (F[i-1]*d*l[i-1])/2. * vp_mag * vp[J]; 
		else 
			for (int J=0; J<3; J++)  Dp[i][J] = 1./2.*env.rho_w*Cdn* (F[i]*d*l[i] + F[i-1]*d*l[i-1])/2. * vp_mag * vp[J]; 
		
		// tangential drag		
		if (i==0)
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env.rho_w*Cdt* pi*(F[i]*d*l[i])/2. * vq_mag * vq[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env.rho_w*Cdt* pi*(F[i-1]*d*l[i-1])/2. * vq_mag * vq[J]; 
		else
			for (int J=0; J<3; J++)  Dq[i][J] = 1./2.*env.rho_w*Cdt* pi*(F[i]*d*l[i] + F[i-1]*d*l[i-1])/2. * vq_mag * vq[J]; 
				
		
		// acceleration calculations					
		for (int J=0; J<3; J++)  {
			aq[J] = dotprod(Ud[i], q[i]) * q[i][J]; // tangential component of fluid acceleration
			ap[J] = Ud[i][J] - aq[J]; 			// normal component of fluid acceleration
		}
		
		// transverse Froude-Krylov force
		if (i==0)	
			for (int J=0; J<3; J++)  Ap[i][J] = env.rho_w*(1.+Can)*0.5*( V[i]) * ap[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Ap[i][J] = env.rho_w*(1.+Can)*0.5*(V[i-1] ) * ap[J]; 
		else
			for (int J=0; J<3; J++)  Ap[i][J] = env.rho_w*(1.+Can)*0.5*( V[i] + V[i-1] ) * ap[J]; 
		
		// tangential Froude-Krylov force					
		if (i==0)	
			for (int J=0; J<3; J++)  Aq[i][J] = env.rho_w*(1.+Cat)*0.5*( V[i]) * aq[J]; 
		else if (i==N)
			for (int J=0; J<3; J++)  Aq[i][J] = env.rho_w*(1.+Cat)*0.5*( V[i-1] ) * aq[J]; 
		else
			for (int J=0; J<3; J++)  Aq[i][J] = env.rho_w*(1.+Cat)*0.5*( V[i] + V[i-1] ) * aq[J]; 
		
		// bottom contact (stiffness and damping, vertical-only for now) - updated for general case of potentially anchor or fairlead end in contact
		if (r[i][2] < -env.WtrDpth)
		{
			if (i==0)
				B[i][2] = ( (-env.WtrDpth-r[i][2])*env.kb - rd[i][2]*env.cb) * 0.5*(          d*l[i-1] );
			else if (i==N)
				B[i][2] = ( (-env.WtrDpth-r[i][2])*env.kb - rd[i][2]*env.cb) * 0.5*( d*l[i]            );
			else
				B[i][2] = ( (-env.WtrDpth-r[i][2])*env.kb - rd[i][2]*env.cb) * 0.5*( d*l[i] + d*l[i-1] );
			
			// // new rough-draft addition of seabed friction
			// //double FrictionCoefficient = 0.5;                  // just using one coefficient to start with          
			// double FrictionMax = abs(B[i][2])*env.FrictionCoefficient; // dynamic friction force saturation level based on bottom contact force
			// // saturated damping approach to applying friction, for now
			// double BottomVel = sqrt(rd[i][0]*rd[i][0] + rd[i][1]*rd[i][1]); // velocity of node along sea bed
			// double FrictionForce = BottomVel * env.FrictionCoefficient*env.FricDamp; // some arbitrary damping scaling thing at end
			// if (FrictionForce > env.StatDynFricScale*FrictionMax)  FrictionForce = FrictionMax;     // saturate (quickly) to static/dynamic friction force level 
			// // apply force in correct directions -- opposing direction of motion
			//  // could add ifs in here to handle end nodes
			// B[i][0] = -FrictionForce*rd[i][0]/BottomVel;
			// B[i][1] = -FrictionForce*rd[i][1]/BottomVel;
		}
		else 
		{
			B[i][0] = 0.;
			B[i][1] = 0.;
			B[i][2] = 0.;
		}		
		// total forces
		if (i==0)
			for (int J=0; J<3; J++) Fnet[i][J] = T[i][J]             + Td[i][J]              + W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		else if (i==N)                                               
			for (int J=0; J<3; J++) Fnet[i][J] =          -T[i-1][J]            - Td[i-1][J] + W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		else                                                           
			for (int J=0; J<3; J++) Fnet[i][J] = T[i][J] - T[i-1][J] + Td[i][J] - Td[i-1][J] + W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		
	}
		
		
	// loop through internal nodes and update their states
	for (int i=1; i<N; i++)	
	{
		// calculate RHS constant (premultiplying force vector by inverse of mass matrix  ... i.e. rhs = S*Forces)	
		for (int I=0; I<3; I++) 
		{
			double RHSiI = 0.0; // temporary accumulator 
			for (int J=0; J<3; J++) 
				RHSiI += S[i][I][J] * Fnet[i][J]; 	//  matrix multiplication [S i]{Forces i}
			
			// update states
			Xd[3*N-3 + 3*i-3 + I] = X[3*i-3 + I];    	// dxdt = V  (velocities)
			Xd[        3*i-3 + I] = RHSiI;      		// dVdt = RHS * A  (accelerations)
		}		
	}
	
	return;
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
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << rd[i][J] << "\t ";
				}
			}
			// output wave velocities?
			if (channels.find("u") != string::npos) {
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