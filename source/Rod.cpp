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
 
#include "Rod.h"
#include "Connection.h"
#include "Line.h"
//#include "QSlines.h" // the c++ version of quasi-static model Catenary

using namespace std;

// here is the new numbering scheme (N segments per line)

//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]



// ================== Line member functions ===========================


// set up line object
int Rod::setup(int type_in, int number_in, RodProps *props, double endCoords[6], int NumSegs, 
	shared_ptr<ofstream> outfile_pointer, string channels_in)
{
	// ================== set up properties ===========	
	number = number_in;	
	type = type_in;
	N = NumSegs;             // assign number of segments to rod
	
	if (wordy >0) cout << "Setting up Rod " << number << " with " << N << " segments." << endl;
			
	// store passed rod properties (and convert to numbers)
	d   = props->d;
	rho = props->w/(pi/4.*d*d);
	Can = props->Can;
	Cat = props->Cat;
	Cdn = props->Cdn;
	Cdt = props->Cdt;
	
	t=0.;		
	
	nAttachedA = 0;  // start off with zero connections
	nAttachedA = 0;

	
	WaveKin = 0;  // start off with wave kinematics disabled.  Can be enabled after initial conditions are found and wave kinematics are calculated


	// =============== size vectors =========================
		
	r.resize( N+1, vector<double>(3, 0.0));		// node positions [i][x/y/z]
	rd.resize(N+1, vector<double>(3, 0.0));		// node velocities [i][x/y/z]
	q.resize( 3, 0.0);     	// unit tangent vector for the rod as a whole
	
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
		
//	S.resize(N+1, vector< vector< double > >(3, vector<double>(3, 0.0)));  // inverse mass matrices (3x3) for each node
	M.resize(N+1, vector< vector< double > >(3, vector<double>(3, 0.0)));  // mass matrices (3x3) for each node
			
	l.resize(N, 0.0); 		// line unstretched segment lengths
//	lstr.resize(N, 0.0); 		// stretched lengths
//	ldstr.resize(N, 0.0); 		// rate of stretch
	V.resize(N, 0.0);			// volume?
	
	zeta.resize(N+1, 0.0);					// wave elevation above each node
	F.resize(N+1, 0.0); 	// fixed 2014-12-07	// VOF scalar for each NODE (mean of two half adjacent segments) (1 = fully submerged, 0 = out of water)
	U.resize(N+1, vector<double>(3, 0.));     	// wave velocities
	Ud.resize(N+1, vector<double>(3, 0.));;     	// wave accelerations
	


	// ======================== set starting kinematics ====================
	if (type==1)
		// do nothing - (initial position will later be set by parent)
		type = 1;	
	else if (type==2)                // for an independent rod, set the position right off the bat
	{		
		for (int J=0; J<3; J++)
		{
			r[0][J] = endCoords[J];     // start off position at that specified in input file 
			r[N][J] = endCoords[3+J];   //  (will be starting point for connect connections
		}                              //   and the permanent location of anchor connections.)
		
		// get direction vector (r[3-5]) and length
		double dummyUnitVector[3];
		directionAndLength(endCoords, endCoords+3, dummyUnitVector, &UnstrLen); 
	}
	else
	{	cout << "Error - Rod isn't type 1 or 2." << endl;
		return -1;
	}
			
			

	for (int i=0; i<N; i++)	
	{	l[i] = UnstrLen/double(N);	// distribute line length evenly over segments
		V[i] = l[i]*0.25*pi*d*d;   
	}
	
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	channels = channels_in; 				// copy string of output channels to object

	outfile = NULL; // temporary <<<
			
	return 0;
};

// this function handles assigning a line to a Rod end
void Rod::addLineToRodEndA(Line *theLine, int TopOfLine)
{
	if (wordy>0) cout << "L" << theLine->number << "->R" << number << "A ";
	
	if (nAttachedA <10) // this is currently just a maximum imposed by a fixed array size.  could be improved.
	{
		AttachedA[nAttachedA] = theLine;
		TopA[nAttachedA] = TopOfLine;
		nAttachedA += 1;
	}
	return;
};
void Rod::addLineToRodEndB(Line *theLine, int TopOfLine)
{
	if (wordy>0) cout << "L" << theLine->number << "->R" << number << "B ";
	
	if (nAttachedB <10) // this is currently just a maximum imposed by a fixed array size.  could be improved.
	{
		AttachedB[nAttachedB] = theLine;
		TopB[nAttachedB] = TopOfLine;
		nAttachedB += 1;
	}
	return;
};


void Rod::setEnv(EnvCond env_in)
{
	env = env_in;
}


// get ICs for line using quasi-static approach
void Rod::initializeRod(double* X )	
{	
	
	if (wordy > 0 ) cout << "hey we're initializing a rod now! " << endl;
	
	// create output file for writing output (and write channel header and units lines) if applicable
/*				
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
*/	
		
	//if (-env.WtrDpth > r[0][2]) {
	//	cout << "   Error: water depth is shallower than Line " << number << " anchor." << endl;
	//	return;
	//}
	
	
//	vector<double> snodes(N+1, 0.0);   					// locations of line nodes along line length - evenly distributed here 
//	for (int i=1; i<=N; i++) snodes[i] = snodes[i-1] + l[i-1]; 
//	snodes[N] = UnstrLen; 								// double check to ensure the last node does not surpass the line length
		
	// stretch the nodes between the endpoints linearly
	for (int i=1; i<N; i++)
		for (int J=0; J<3; J++)
			r[i][J]  = r[0][J] + (r[N][J] - r[0][J]) * (float(i)/float(N));
	
	// Pass kinematics to any attached lines (this is just like what a Connection does, except for both ends)
	// so that they have the correct initial positions at this initialization stage.
	for (int l=0; l < nAttachedA; l++)  AttachedA[l]->setEndState(r[0], rd[0], TopA[l]);
	for (int l=0; l < nAttachedB; l++)  AttachedB[l]->setEndState(r[N], rd[N], TopB[l]);
		
	
	// assign the resulting kinematics to its part of the state vector (only matters if it's an independent Rod)
	
	unitvector(q, r[0], r[N]  ); 	// compute unit vector q
	
	for (int J=0; J<3; J++) 
	{
		X[  J] = 0.0;       // zero velocities for initialization
		X[3+J] = 0.0;
		X[6+J] = r[0][J];   // end A position
		X[9+J] = q[J];      // rod direction unit vector
	}

	return;
};


// function to get position of any node along the line
int Rod::getNodePos(int NodeNum, double pos[3])
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

int Rod::getN()
{
	return N;
};


double Rod::GetRodOutput(OutChanProps outChan)
{	
	if      (outChan.QType == PosX)  return  r[outChan.NodeID][0];
	else if (outChan.QType == PosY)  return  r[outChan.NodeID][1];
	else if (outChan.QType == PosZ)  return  r[outChan.NodeID][2];
	else if (outChan.QType == VelX)  return  rd[outChan.NodeID][0];
	else if (outChan.QType == VelY)  return  rd[outChan.NodeID][1];
	else if (outChan.QType == VelZ)  return  rd[outChan.NodeID][2];
//	else if (outChan.QType == Ten )  return  getNodeTen(outChan.NodeID);
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


// function for boosting drag coefficients during IC generation	
void Rod::scaleDrag(double scaler)
{
	Cdn = Cdn*scaler;
	Cdt = Cdt*scaler;
	return;
}

// function to reset time after IC generation
void Rod::setTime(double time)
{
	t = time;
	return;
}

	
// pass the latest states to the rod (and calculate second end node position)
int Rod::setState( const double* X, const double time)
{
	// for a Rod, the states are:
	// [ x, y, z velocity of end A, then rate of change of u/v/w coordinates of unit vector pointing toward end B,
	// then x, y, z coordinate of end A, u/v/w coordinates of unit vector pointing toward end B]
	
	// this function is applicable for type 1 and 2 rods. 
	// (for type 1, X is not actual states, just kinematics from the parent Body)
	
	// store current time
	t = time;
	
	// copy over state values for potential use during derivative calculations
	for (int J=0; J<6; J++)
	{
		r6[J] = X[6+J];
		r6d[J]= X[  J];
	}
	
	// from state values, set positions of end nodes 
	for (int J=0; J<3; J++) 	
	{	// end A
		r[0][J]  = X[6 + J];      // get positions
		rd[0][J] = X[J];          // get velocities
		// end B
		r[N][J]  = X[6 + J] + UnstrLen*X[9 + J]; // get positions  xB = xA + L*dx_AB/dL
		rd[0][J] = X[J]   ; // + <<<<<<<<<<<<<<<<<<<<<<< this one's complicated!!
	}
	return 0;
	
	// pass kinematics to any attached lines (this is just like what a Connection does, except for both ends)
	for (int l=0; l < nAttachedA; l++)  AttachedA[l]->setEndState(r[0], rd[0], TopA[l]);
	for (int l=0; l < nAttachedB; l++)  AttachedB[l]->setEndState(r[N], rd[N], TopB[l]);
	
}
	
	
// calculate the forces and state derivatives of the rod	(only for type 2 rods)
int Rod::getStateDeriv(double* Xd)
{
	
	// attempting error handling <<<<<<<<
	for (int i=0; i<=N; i++)
	{
		if (isnan(r[i][0]+r[i][1]+r[i][2])) 
		{
			stringstream s;
			s << "Rod " << number << " node positions:";
			for (int j=0; j<N; j++) s << r[i][0] << "," << r[i][1] << "," << r[i][2] << "; ";
			s << " at time " << t;
			throw string(s.str());
		}
	}
	
	// calculate forces and added mass for each node (including those from lines attached to ends)
	doRHS();
	
	double Fnet_out[6] = {0.0};    // total force vector
	double M_out[36] = {0.0};  // total mass matrix (serialized for easy processing)
	
	// now go through each node's contributions, put them in end A ref frame, and sum them
	for (int i=1; i<N; i++)
	{		
		double rRel[3];     // position of a given node relative to the body reference point (global orientation frame)
		double Fnet_dub[3];
		double M_dub[3][3];
		double F6_i[6];     // 6dof force-moment from rod about body ref point (but global orientation frame of course)
		double M6_i[6][6];  // mass matrix of each rod to be added		
		
		for (int J=0; J<3; J++) 
		{	rRel[J] = r[i][J] - r[0][J];   // vector from end A to node
			Fnet_dub[J] = Fnet[i][J];      // convert to array for passing
			for (int K=0; K<3; K++) M_dub[J][K] = M[i][J][K];
		}	
		
		// convert segment net force into 6dof force about body ref point
		translateForce3to6DOF(rRel, Fnet_dub, F6_i);			
		
		// convert segment mass matrix to 6by6 mass matrix about body ref point
		translateMass3to6DOF(rRel, M_dub, M6_i);
		
		for (int J=0; J<6; J++)
		{
			Fnet_out[J] += F6_i[J]; // add force to total force vector
			
			for (int K=0; K<6; K++)
				M_out[6*J + K] += M6_i[J][K];  // add element mass matrix to body mass matrix
		}		
	}	
	
	// supplement mass matrix with rotational inertia terms for axial rotation of rod
	// (this is based on assigning Jaxial * cos^2(theta) to each axis...
	M_out[21] += rho * d*d*d*d/64 * r6[3]*r6[3];    // <<<< check the math on this!!!
	M_out[28] += rho * d*d*d*d/64 * r6[4]*r6[4];
	M_out[35] += rho * d*d*d*d/64 * r6[5]*r6[5];
	
	// solve for accelerations in [M]{a}={f} using LU decomposition
	double LU[36];                        // serialized matrix that will hold LU matrices combined
	Crout(6, M_out, LU);                  // perform LU decomposition on mass matrix
	double acc[6];                        // acceleration vector to solve for
	solveCrout(6, LU, Fnet_out, acc);     // solve for acceleration vector
			
	// invert body total mass matrix
	//inverse6by6(S, M_out);  
	
	
	// calculate unit tangent vector (q) for rod  (though this is redundant with passed values)
	unitvector(q, r[0], r[N]  ); 	// compute unit vector q
	
	
	
	
	// RHS constant - (premultiplying force vector by inverse of mass matrix  ... i.e. rhs = S*Forces
	for (int I=0; I<3; I++) 
	{
		//double RHSI = 0.0; // temporary accumulator 
		//for (int J=0; J<6; J++) {
		//	RHSI += S[I][J] * Fnet[J]; //  matrix multiplication [S i]{Forces i}
		//}
		
		// update states
		Xd[6 + I] = r6d[  I];    // dxdt = V        (velocities)
		Xd[9 + I] = r6d[3+I];    
		Xd[    I] = acc[  I];   // dVdt = RHS * A  (accelerations)
		Xd[3 + I] = acc[3+I];   
	}
					
	return 0;
}


// calculate the force and mass contributions of the rod on the parent body (only for type 1 Rods?)
void Rod::getNetForceAndMassContribution(double rBody[3], double Fnet_out[6], double M_out[6][6])
{
	// rBody is the location of the body reference point
	
	// question: do I really want to neglect the rotational inertia/drag/etc across the length of each segment?

	doRHS(); // do calculations of forces and masses on each rod node
	
	// make sure Fnet_out and M_out are zeroed first
	for (int J=0; J<6; J++)
	{	Fnet_out[J] = 0.0;
		for (int K=0; K<6; K++)
			M_out[J][K] = 0.0;
	}
	
	// now go through each node's contributions, put them in body ref frame, and sum them
	for (int i=1; i<N; i++)
	{		
		double rRel[3];     // position of a given node relative to the body reference point (global orientation frame)
		double Fnet_dub[3];
		double M_dub[3][3];
		double F6_i[6];     // 6dof force-moment from rod about body ref point (but global orientation frame of course)
		double M6_i[6][6];  // mass matrix of each rod to be added		
		
		for (int J=0; J<3; J++) 
		{	rRel[J] = r[i][J] - r[0][J];   // vector from end A to node
			Fnet_dub[J] = Fnet[i][J];      // convert to array for passing
			for (int K=0; K<3; K++) M_dub[J][K] = M[i][J][K];
		}	
		
		// convert segment net force into 6dof force about body ref point
		translateForce3to6DOF(rRel, Fnet_dub, F6_i);			
		
		// convert segment mass matrix to 6by6 mass matrix about body ref point
		translateMass3to6DOF(rRel, M_dub, M6_i);
				
		for (int J=0; J<6; J++)
		{
			Fnet_out[J] += F6_i[J]; // add force to total force vector
			
			for (int K=0; K<6; K++)
				M_out[J][K] += M6_i[J][K];  // add element mass matrix to body mass matrix
		}		
	}	
	return;
};


//  this is the big function that calculates the forces on the rod
void Rod::doRHS()
{
	
	
	// also get net force and mass on each connect object (used to apply constant-length constraint)	
//	double FconnA[3];
//	double FconnB[3];
//	double MconnA[3][3];
//	double MconnB[3][3];
//	AnchConnect->getFnet(FconnA);
//	FairConnect->getFnet(FconnB);
//	AnchConnect->getM(MconnA);
//	FairConnect->getM(MconnB);

	// set interior node positions and velocities (stretch the nodes between the endpoints linearly)
	for (int i=1; i<N; i++)
	{	for (int J=0; J<3; J++)
		{	 r[i][J]  =  r[0][J] + ( r[N][J] - r[0][J]) * (float(i)/float(N));
			rd[i][J]  = rd[0][J] + (rd[N][J] -rd[0][J]) * (float(i)/float(N));
		}
	
		V[i] = 0.25*pi * d*d*l[i];		// volume attributed to segment
	}
				
//	// stretched length of entire rod and strain rate
//	double lstr_squared = 0.0;
//	for (int J=0; J<3; J++) lstr_squared += (r[N][J] - r[0][J])*(r[N][J] - r[0][J]);
//	lstr = sqrt(lstr_squared); 	// stretched rod length
//	
//	double ldstr_top = 0.0;   // this is the denominator of how the stretch rate equation was formulated
//	for (int J=0; J<3; J++) ldstr_top += (r[N][J] - r[0][J])*(rd[N][J] - rd[0][J]);
//	ldstr = ldstr_top/lstr; 	// rate of rod stretch (m/s)

			
	// calculate unit tangent vector (q) for rod  (though this is redundant with passed values)
	unitvector(q, r[0], r[N]  ); 	// compute unit vector q
	
	
	
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
	
	
    // calculate mass matrix   <<<< can probably simplify/eliminate this...
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
				M[i][I][J] = m_i*eye(I,J) + env.rho_w*v_i *( Can*(eye(I,J) - q[I]*q[J]) + Cat*q[I]*q[J] );					
			}
		}		
		
		// mass matrices will be summed up before inversion, near end of this function
	}
	

	// ============  CALCULATE FORCES ON EACH NODE ===============================
	
	// no nodal axial structural loads calculated since it's assumed rigid
	
	// but can I at least calculate tension/compression due to other loads?
	
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
			vq[J] = dotProd( vi , q ) * q[J]; 	// tangential relative flow component
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
			aq[J] = dotProd(Ud[i], q) * q[J]; // tangential component of fluid acceleration
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
			
			// new rough-draft addition of seabed friction
			//double FrictionCoefficient = 0.5;                  // just using one coefficient to start with          
			double FrictionMax = abs(B[i][2])*env.FrictionCoefficient; // dynamic friction force saturation level based on bottom contact force
			// saturated damping approach to applying friction, for now
			double BottomVel = sqrt(rd[i][0]*rd[i][0] + rd[i][1]*rd[i][1]); // velocity of node along sea bed
			double FrictionForce = BottomVel * env.FrictionCoefficient*env.FricDamp; // some arbitrary damping scaling thing at end
			if (FrictionForce > env.StatDynFricScale*FrictionMax)  FrictionForce = FrictionMax;     // saturate (quickly) to static/dynamic friction force level 
			// apply force in correct directions -- opposing direction of motion
			 // could add ifs in here to handle end nodes
			B[i][0] = -FrictionForce*rd[i][0]/BottomVel;
			B[i][1] = -FrictionForce*rd[i][1]/BottomVel;
		}
		else 
		{
			B[i][0] = 0.;
			B[i][1] = 0.;
			B[i][2] = 0.;
		}		
		// total forces
		if (i==0)
			for (int J=0; J<3; J++) Fnet[i][J] = W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		else if (i==N)                          
			for (int J=0; J<3; J++) Fnet[i][J] = W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		else                                   
			for (int J=0; J<3; J++) Fnet[i][J] = W[i][J] + (Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]) + B[i][J];
		
	}
	
	// ============ now add in forces on end nodes from attached lines =============
	
	// loop through lines attached to end A
	for (int l=0; l < nAttachedA; l++)
	{
		double Fnet_i[3] = {0.0};  double M_i[3][3] = {{0.0}};
		
		// get quantities
		if (TopA[l] == 0) 	
			(AttachedA[l])->getAnchStuff(Fnet_i, M_i); 	// if attached to bottom/anchor of the line
		else 				
			(AttachedA[l])->getFairStuff(Fnet_i, M_i);  // if attached to top/fairlead of the line
			
		// Process outline for line failure, similar to as done for connections (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if failure time has elapsed for line
		// 2. create new massless connect with same instantaneous kinematics as current Rod end
		// 3. disconnect line end from current Rod end and instead attach to new connect
		// The above may require rearrangement of connection indices, expansion of state vector, etc.
			
		// sum quantitites
		for (int I=0; I<3; I++) {
			Fnet[0][I] += Fnet_i[I];
		
			for (int J=0; J<3; J++) 
				M[0][I][J] += M_i[I][J];					
		}
	}
	
	// loop through lines attached to end B
	for (int l=0; l < nAttachedB; l++)
	{
		double Fnet_i[3] = {0.0};  double M_i[3][3] = {{0.0}};
		
		// get quantities
		if (TopB[l] == 0) 	
			(AttachedB[l])->getAnchStuff(Fnet_i, M_i); 	// if attached to bottom/anchor of the line
		else 				
			(AttachedB[l])->getFairStuff(Fnet_i, M_i);  // if attached to top/fairlead of the line
			
		// sum quantitites
		for (int I=0; I<3; I++) {
			Fnet[N][I] += Fnet_i[I];
		
			for (int J=0; J<3; J++) 
				M[N][I][J] += M_i[I][J];					
		}
	}
	
	return;
};



// write output file for line  (accepts time parameter since retained time value (t) will be behind by one line time step
void Rod::Output(double time)
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


Rod::~Rod()
{
	// destructor

	r        .clear();
	rd       .clear();
	q        .clear();
	T        .clear();
	Td       .clear();
	Tmag     .clear();
	W        .clear();
	Dp       .clear();
	Dq       .clear();
	Ap       .clear();
	Aq       .clear();
	B        .clear();
	Fnet     .clear();
//	S        .clear();
	M        .clear();
	F        .clear();
	l        .clear();
//	lstr     .clear();
//	ldstr    .clear();
	V        .clear();
	U        .clear();
	Ud       .clear();
	zeta     .clear();
	w        .clear();
	k        .clear();
	zetaC0   .clear();
	zetaC    .clear();
	UC       .clear();
	UdC      .clear();
	WGNC     .clear();
	WaveS2Sdd.clear();
	Ucurrent .clear();
	zetaTS   .clear();
	FTS      .clear();
	UTS      .clear();
	UdTS     .clear();
	tTS      .clear();
}

// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void Rod::drawGL(void)
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



void Rod::drawGL2(void)
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