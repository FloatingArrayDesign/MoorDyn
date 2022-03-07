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
#include "Line.h"
#include "Waves.h"

using namespace std;

// here is the new numbering scheme (N segments per line)

//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] --- ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]



// ================== Rod member functions ===========================


// set up Rod object  
int Rod::setup(int number_in, types type_in, RodProps *props, double endCoords[6], int NumSegs, 
	shared_ptr<ofstream> outfile_pointer, string channels_in)
{
	// ================== set up properties ===========	
	number = number_in;
	type = type_in;

	N = NumSegs;             // assign number of segments to rod
	
	if (wordy >0) cout << "Setting up Rod " << number << " (type "<<type<<") with " << N << " segments." << endl;

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


//	WaveKin = 0;  // start off with wave kinematics disabled.  Can be enabled after initial conditions are found and wave kinematics are calculated

	// ----- size arrays that are the same no matter what -----
	
	r   = make2Darray(N+1, 3);     // node positions [i][x/y/z]
	rd  = make2Darray(N+1, 3);     // node velocities [i][x/y/z]
	M   = make3Darray(N+1, 3, 3);  // mass matrices (3x3) for each node
	
	// forces 
	W   = make2Darray(N+1, 3);     // node weights
	Bo  = make2Darray(N+1, 3);     // node buoyancy 
	Pd  = make2Darray(N+1, 3);     // dynamic pressure
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
	
	
	UnstrLen = unitvector(q, endCoords, endCoords+3); // get Rod axis direction vector and Rod length


	// ----- different handling for zero-length rods if N=0 -----
	
	if (N==0)  // special case of zero-length rod, which is denoted by numsegs=0 in the intput file 
	{
		l   = make1Darray(1);          // line unstretched segment lengths
		V   = make1Darray(1);          // segment volume?
		UnstrLen = 0.0;                // set Rod length to zero
		
		// calculate properties for one (imaginary) segment
		l[0] = 0.0;                    
		V[0] = 0.0;   
	}
	else       // normal finite-length case
	{
		l   = make1Darray(N);          // line unstretched segment lengths
		V   = make1Darray(N);          // segment volume?
		
		// calculate a few segment properties
		for (int i=0; i<N; i++)	
		{
			l[i] = UnstrLen/double(N);	// distribute line length evenly over segments
			V[i] = l[i]*0.25*pi*d*d;   
		}
	}

	// ------------------------- set starting kinematics -------------------------
	
	// set Rod positions if applicable
	if (type == FREE)                // for an independent rod, set the position right off the bat
	{
		for (int J=0; J<3; J++)
		{
			r6[J] = endCoords[J]; // (end A coordinates) 
			v6[J] = 0.0;      // (end A velocity, unrotated axes) 

			r6[3+J] = q[J];   // (Rod direction unit vector)
			v6[3+J] = 0.0;    // (rotational velocities about unrotated axes) 
		}
	}
	else if ((type == PINNED) || (type == CPLDPIN))     // for a pinned rod, just set the orientation (position will be set later by parent object)
	{
		for (int J=0; J<3; J++)
		{
			r6[3+J] = q[J];   // (Rod direction unit vector)
			v6[3+J] = 0.0;    // (rotational velocities about unrotated axes) 
		}
	}
	// otherwise (for a fixed rod) the positions will be set by the parent body or via coupling
	
	

	// Initialialize some variables (Just a bunch of them would be used, but we
	// better initialize everything just in case Output methods are called, so
	// no memory errors are triggered)
	for (int i = 0; i <= N; i++)
	{
		zeta[i] = 0.0;
		for (int J = 0; J < 3; J++)
		{
			const double f = i / (double)N;
			r[i][J] = endCoords[J] + f * (endCoords[J + 3] - endCoords[J]);
			rd[0][J] = 0.0;
		}
	}

	// set the number of preset wave kinematic time steps to zero (flagging disabled) to start with
	ntWater = 0; 

	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	channels = channels_in; 				// copy string of output channels to object


	if (wordy >0)  cout << "Set up Rod " << number << ", type " << type << ". ";

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


// this function handles removing a line from a Rod end
void Rod::removeLineFromRodEndA(int lineID, int *topOfLine, double rEnd[], double rdEnd[])
{
	for (int l=0; l<nAttachedA; l++)    // look through attached lines
	{
		if (AttachedA[l]->number == lineID)   // if this is the line's entry in the attachment list 
		{
			*topOfLine = TopA[l];                // record which end of the line was attached
			
			for (int m=l; m<nAttachedA-1; m++)
			{	
				AttachedA[m] = AttachedA[m+1];  // move subsequent line links forward one spot in the list to eliminate this line link
				TopA[      m] =       TopA[m+1]; 
			}
			nAttachedA -= 1;                       // reduce attached line counter by 1

			// also pass back the kinematics at the end
			for (int J=0; J<3; J++)
			{
				rEnd[ J] = r[ 0][J];
				rdEnd[J] = rd[0][J];
			}

			cout << "Detached line " << lineID << " from Rod " << number << " end A" << endl;
			break;
		}
		if (l==nAttachedA-1)   // detect if line not found
			cout << "Error: failed to find line to remove during removeLineFromRodEndA call to rod " << number << ". Line " << lineID << endl;
	}
};
void Rod::removeLineFromRodEndB(int lineID, int *topOfLine, double rEnd[], double rdEnd[])
{
	for (int l=0; l<nAttachedB; l++)    // look through attached lines
	{
		if (AttachedB[l]->number == lineID)   // if this is the line's entry in the attachment list
		{
			*topOfLine = TopB[l];                // record which end of the line was attached
			
			for (int m=l; m<nAttachedB-1; m++)
			{	
				AttachedB[m] = AttachedB[m+1];  // move subsequent line links forward one spot in the list to eliminate this line link
				TopB[      m] =       TopB[m+1]; 
			}
			nAttachedB -= 1;                       // reduce attached line counter by 1

			// also pass back the kinematics at the end
			for (int J=0; J<3; J++)
			{
				rEnd[ J] = r[ N][J];
				rdEnd[J] = rd[N][J];
			}

			cout << "Detached line " << lineID << " from Rod " << number << " end B" << endl;
			break;

		}
		if (l==nAttachedB-1)   // detect if line not found
			cout << "Error: failed to find line to remove during removeLineFromRodEndA call to rod " << number << ". Line " << lineID << endl;
	}
};
	


void Rod::setEnv(EnvCond *env_in, Waves *waves_in)
{
	env = env_in;      // set pointer to environment settings object
	waves = waves_in;  // set pointer to Waves  object
}

/*
// set rod fairlead ICs based on fairlead-centric coupling (no platform stuff)
void_Rod_initializeCpld( double pX[], double vX[] )	
{	
	
	if (wordy > 0 ) cout << "Initializing Rod "<<number<<" (type "<<type<<") now." << endl;
	
	// set rod positions.   ...should set velocities too? <<<<<
		
	if (type==-1)  // if just pinned
	{
		for (int J=0; J<3; J++)
			r[0][J] = pX[J];
		
		// setting dependent objects and possible output file stuff will be set by subsequent call to initializeFree since this rod has one free end
	}
	else if (type == -2) // if cantilever coupled
	{
		for (int J=0; J<6; J++)
		{
			r6[J] = pX[J];  
			v6[J] = 0.0; // for now  // vX
		}
		setDependentStates();  // set end node kinematics and pass to any attached lines
	}
	else // otherwise report error
		throw string("Error: wrong type of Rod called with initializeCpld");
	
	return;
};
*/


// Make output file for Rod and set end kinematics of any attached lines.
// For free Rods, fill in the initial states into the state vector.
// Notes: r6 and v6 must already be set.  
//        ground- or body-pinned rods have already had setKinematics called to set first 3 elements of r6, v6.
void Rod::initializeRod(double* X )
{	
	
	if (wordy > 0 ) cout << "Initializing Rod "<<number<<" (type "<<type<<") now." << endl;
	
	// create output file for writing output (and write channel header and units lines) if applicable
			
	if (outfile) // check it's not null.  Null signals no individual line output files
	{
		if (outfile->is_open())
		{	
			// ------------- write channel names line --------------------
		
			// output time
			*outfile << "Time" << "\t ";
			
			// output positions?
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
			// output net node forces?
			if (channels.find("f") != string::npos) {
				for (int i=0; i<=N; i++)  {
					*outfile << "Node" << i << "Fx \t Node" <<  i << "Fy \t Node" <<  i << "Fz \t ";
				}
			}
					
			*outfile << "\n";   
			
			
			// ----------- write units line ---------------

			if (env->WriteUnits > 0)
			{
				// output time
				*outfile << "(s)" << "\t ";
				
				// output positions?
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
				
				*outfile << "\n";  
			}
		}
		else cout << "   Error: unable to write file Line" << number << ".out" << endl;  //TODO: handle this!
	}

		
	//if (-env->WtrDpth > r[0][2]) {
	//	cout << "   Error: water depth is shallower than Line " << number << " anchor." << endl;
	//	return;
	//
	
	
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
	
	
	// the r6 and v6 vectors should have already been set
	// r and rd of ends have already been set by setup function or by parent object   <<<<< right? <<<<<
	
	
	// Pass kinematics to any attached lines (this is just like what a Connection does, except for both ends)
	// so that they have the correct initial positions at this initialization stage.
	if (type > COUPLED)
		setDependentStates();  // don't call this for type -2 coupled Rods as it's already been called


	// assign the resulting kinematics to its part of the state vector (only matters if it's an independent Rod)
	
	// copy over state values for potential use during derivative calculations
	if (type == FREE)               // free Rod type
	{	
		for (int J=0; J<3; J++)
		{
			X[  J] = 0.0;       // zero velocities for initialization
			X[3+J] = 0.0;
			X[6+J] = r[0][J];   // end A position
			X[9+J] = q[J];      // rod direction unit vector
		}
	}
	else if ((type == PINNED) || (type == CPLDPIN))           // pinned rod type (coupled or attached to something previously via setPinKin)
	{	
		for (int J=0; J<3; J++)
		{
			X[  J] = 0.0;       // zero rotational velocities for initialization
			X[3+J] = q[J];      // rod direction unit vector
		}
	}	
	// otherwise this was only called to make the rod an output file and set its dependent line end kinematics...

	if (wordy>0) cout << "Initialized Rod " << number << endl;

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
	else if (outChan.QType == Ten )    // for Rods, this option provides the net force applied by attached lines at end A (if 0) or end B (if >0)
	{	
		if (outChan.NodeID > 0)
			return  sqrt(FextB[0]*FextB[0] + FextB[1]*FextB[1] + FextB[2]*FextB[2]);
		else
			return  sqrt(FextA[0]*FextA[0] + FextA[1]*FextA[1] + FextA[2]*FextA[2]);
	}
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
void Rod::storeWaterKin(int PARAM_UNUSED nt, double PARAM_UNUSED dt,
                        double PARAM_UNUSED **zeta_in,
                        double PARAM_UNUSED **f_in,
                        double PARAM_UNUSED ***u_in,
                        double PARAM_UNUSED ***ud_in)
{
	return;
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


// called at the beginning of each coupling step to update the boundary conditions (fairlead kinematics) for the proceeding line time steps
void Rod::initiateStep(const double rFairIn[6], const double rdFairIn[6], double time)
{	
	t0 = time; // set start time for BC functions
	
	if (type == COUPLED)  // rod rigidly coupled to outside program
	{						
		// set Rod kinematics based on BCs (linear model for now) 
		for (int J=0; J<6; J++)  
		{
			r_ves[J] = rFairIn[J];
			rd_ves[J] = rdFairIn[J];
		}	
		
		// since this rod has no states and all DOFs have been set, pass its kinematics to dependent Lines
		setDependentStates();
	}
	else if (type == CPLDPIN)  // rod end A pinned and coupled to outside program
	{	
		// set Rod *end A only* kinematics based on BCs (linear model for now) 
		for (int J=0; J<3; J++)  // note only setting first three entries
		{
			r_ves[J] = rFairIn[J];
			rd_ves[J] = rdFairIn[J];
		}	
	}
	else
		throw string("Error: updateFairlead called for wrong Rod type.");
	
	return;
};


// updates kinematics for Rods ONLY if they are driven externally (otherwise shouldn't be called)
void Rod::updateFairlead(const double time)
{	
	t = time;

	if (type == COUPLED)  // rod rigidly coupled to outside program
	{						
		// set Rod kinematics based on BCs (linear model for now) 
		for (int J=0; J<6; J++)  
		{
			r6[J] = r_ves[J] + rd_ves[J]*(time-t0);
			v6[J] = rd_ves[J];
		}	
		
		scalevector(r6+3, 1.0, r6+3); // enforce direction vector to be a unit vector
		
		// since this rod has no states and all DOFs have been set, pass its kinematics to dependent Lines
		setDependentStates();
	}
	else if (type == CPLDPIN)  // rod end A pinned and coupled to outside program
	{	
		// set Rod *end A only* kinematics based on BCs (linear model for now) 
		for (int J=0; J<3; J++)  // note only setting first three entries
		{
			r6[J] = r_ves[J] + rd_ves[J]*(time-t0);
			v6[J] = rd_ves[J];
		}	
		
		// Rod is pinned so only end A is specified, rotations are left alone and will be 
		// handled, along with passing kinematics to dependent lines, by separate call to setState
	}
	else
		throw string("Error: updateFairlead called for wrong Rod type.");
	
	return;
}


// set kinematics for Rods ONLY if they are attached to a body (including a coupled body) (otherwise shouldn't be called)
void Rod::setKinematics(double *r_in, double *rd_in)
{	
	if (type == FIXED)  // rod rigidly coupled to body
	{		
		for (int J=0; J<6; J++)  
		{
			r6[J] = r_in[ J];
			v6[J] = rd_in[J];
		}	
		
		scalevector(r6+3, 1.0, r6+3); // enforce direction vector to be a unit vector
		
		// since this rod has no states and all DOFs have been set, pass its kinematics to dependent Lines
		setDependentStates();
	}
	else if (type == PINNED)  // rod end A pinned to a body
	{	
		// set Rod *end A only* kinematics based on BCs (linear model for now) 
		for (int J=0; J<3; J++)  // note only setting first three entries
		{
			r6[J] = r_in[ J];
			v6[J] = rd_in[J];
		}
		
		// Rod is pinned so only end A is specified, rotations are left alone and will be 
		// handled, along with passing kinematics to dependent lines, by separate call to setState
	}
	else
		throw string("Error: setKinematics called for wrong Rod type.");
	
	
	// update Rod direction unit vector (simply equal to last three entries of r6, presumably these were set elsewhere for pinned Rods)
	for (int J=0; J<3; J++)
		q[J] = r6[3+J];	
	
	return;
}

	
// pass the latest states to the rod if it has any DOFs/states (then update rod end kinematics including attached lines)
int Rod::setState( double* X, const double time)
{
	// for a free Rod, there are 12 states:
	// [ x, y, z velocity of end A, then rate of change of u/v/w coordinates of unit vector pointing toward end B,
	// then x, y, z coordinate of end A, u/v/w coordinates of unit vector pointing toward end B]
	
	// for a pinned Rod, there are 6 states (rotational only):
	// [ rate of change of u/v/w coordinates of unit vector pointing toward end B,
	// then u/v/w coordinates of unit vector pointing toward end B]
		
	// store current time
	t = time;
	
	// copy over state values for potential use during derivative calculations
	if (type == FREE)               // free Rod type
	{	
		scalevector(X+9, 1.0, X+9); // enforce direction vector to be a unit vector
	
		for (int J=0; J<3; J++)
		{
			r6[J  ] = X[6+J];   // (end A coordinates)
			v6[J  ] = X[  J];   // (end A velocity, unrotated axes) 
			r6[3+J] = X[9+J];   // (Rod direction unit vector)
			v6[3+J] = X[3+J];   // (rotational velocities about unrotated axes) 
		}
		setDependentStates();
	}
	else if ((type == CPLDPIN) || (type == PINNED))           // pinned rod type (coupled or attached to something)t previously via setPinKin)
	{	
		scalevector(X+3, 1.0, X+3); // enforce direction vector to be a unit vector
		
		for (int J=0; J<3; J++)
		{
			r6[3+J] = X[3+J];   // (Rod direction unit vector)
			v6[3+J] = X[  J];   // (rotational velocities about unrotated axes) 
		}
		setDependentStates();
	}	
	else
	{	
		throw string("Error: Rod::setState called for a non-free rod type");
	}	
	
	if (N==0) // for zero-length Rod case, set orientation stuff to zero (maybe not necessary...)
	{
		for (int J=0; J<3; J++)
		{
			r6[3+J] = 0.0;   // (Rod direction unit vector)
			v6[3+J] = 0.0;   // (rotational velocities about unrotated axes) 
		}
	}
	
	// update Rod direction unit vector (simply equal to last three entries of r6)
	for (int J=0; J<3; J++)
		q[J] = r6[3+J];	
	
	return 0;  // <<<< unused return value!
}


// Set the end kinematics then set the states (positions and velocities) of any line ends attached to this rod.
// This also determines the orientation of zero-length rods.
void Rod::setDependentStates()
{
	//TODO: add bool initialization flag that will skip the N==0 block if true, to avoid calling uninitialized lines during initialization <<<
	
	// from state values, set positions of end nodes 
	for (int J=0; J<3; J++) 	                                          // end A
	{	r[0][J]  = r6[J];  // get positions
		rd[0][J] = v6[J];  // get velocities
	}
	
	if (N > 0)   // set end B nodes only if the rod isn't zero length
		transformKinematicsAtoB(r6, r6+3, UnstrLen, v6, r[N], rd[N]);   // end B    
	
	// pass end node kinematics to any attached lines (this is just like what a Connection does, except for both ends)
	for (int l=0; l < nAttachedA; l++)  AttachedA[l]->setEndState(r[0], rd[0], TopA[l]);
	for (int l=0; l < nAttachedB; l++)  AttachedB[l]->setEndState(r[N], rd[N], TopB[l]);
	
	// if this is a zero-length Rod, get bending moment-related information from attached lines and compute Rod's equilibrium orientation
	if (N==0)
	{
//		double qEnd[3];       // unit vector of attached line end segment, following same direction convention as Rod's q vector
//		double EIend;         // bending stiffness of attached line end segment
//		double dlEnd;         // stretched length of attached line end segment
		double q_EI_dl[3];    // sign-corrected unit vector times EI divided by dl of attached line end segment
		double qMomentSum[3]; // summation of qEnd*EI/dl_stretched (with correct sign) for each attached line
		
		for (int J=0; J<3; J++)
			qMomentSum[J] = 0.0;
		
		for (int l=0; l < nAttachedA; l++)  
		{	
//			AttachedA[l]->getEndSegmentInfo(qEnd, &EIend, &dlEnd, TopA[l]);
			AttachedA[l]->getEndSegmentInfo(q_EI_dl, TopA[l], 0);
			
			for (int J=0; J<3; J++)
				qMomentSum[J] += q_EI_dl[J];  // add each component to the summation vector
				//qMomentSum[J] += qEnd[J]*EIend/dlEnd;  // add each component to the summation vector
				
		}

		for (int l=0; l < nAttachedB; l++)  
		{	
//			AttachedB[l]->getEndSegmentInfo(qEnd, &EIend, &dlEnd, TopB[l]);
			AttachedB[l]->getEndSegmentInfo(q_EI_dl, TopB[l], 1);

			for (int J=0; J<3; J++)
				qMomentSum[J] += q_EI_dl[J];  // add each component to the summation vector
				//qMomentSum[J] += qEnd[J]*EIend/dlEnd;  // add each component to the summation vector
		}
		
		// solve for line unit vector that balances all moments (unit vector of summation of qEnd*EI/dl_stretched over each line)
		scalevector(qMomentSum, 1.0, q);
		
		for (int J=0; J<3; J++) 	                           
			r6[3+J] = q[J];  // set orientation angles (maybe not used)
	
	}
	
	// pass Rod orientation to any attached lines
	for (int l=0; l < nAttachedA; l++)  AttachedA[l]->setEndOrientation(q, TopA[l], 0);
	for (int l=0; l < nAttachedB; l++)  AttachedB[l]->setEndOrientation(q, TopB[l], 1);
	
	return;
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
	double Fnet_out[6] = {0.0};    // total force vector
	double M_out6[6][6] = {{0.0}};  // total mass matrix
	
	getNetForceAndMass(NULL, Fnet_out, M_out6);  // call doRHS and sum each node's contributions about end A
	
	
	// >>>>>>>>> should the below only be done locally for free/pinned Rods? >>>>>>>>>>>>>>>

	// supplement mass matrix with rotational inertia terms for axial rotation of rod
	// (this is based on assigning Jaxial * cos^2(theta) to each axis...
	M_out6[3][3] += rho * d*d*d*d/64 * r6[3]*r6[3];    // <<<< check the math on this!!!
	M_out6[4][4] += rho * d*d*d*d/64 * r6[4]*r6[4];
	M_out6[5][5] += rho * d*d*d*d/64 * r6[5]*r6[5];
		
	
	
	// solve for accelerations in [M]{a}={f} using LU decomposition, then fill in state derivatives
	
	
	if (type == FREE)                  // free rod, 12 states  
	{
		if (N==0)   // special zero-length Rod case, orientation is not an actual state
		{
			
			double acc[3];      // acceleration vector to solve for (translational accelerations of end A in global orientation)
			double M_out3[3][3];
			for (int i=0; i<3; i++)
				for (int j=0; j<3; j++)
					M_out3[i][j] = M_out6[i][j];
		
			LUsolve3(M_out3, acc, Fnet_out);		
			
			for (int I=0; I<3; I++) 
			{
				Xd[6 + I] = v6[ I];       // dxdt = V   (velocities)
				Xd[9 + I] = 0.0;      
				Xd[    I] = acc[I];      // dVdt = a   (accelerations) 
				Xd[3 + I] = 0.0;        // rotational accelerations -- setting to zero, won't be used
			}
			
		}
		else       // regular free Rod case
		{
			double acc[6];                        // acceleration vector to solve for (translational and rotational accelerations of end A in global orientation)
			LUsolve6(M_out6, acc, Fnet_out);
		
			for (int I=0; I<3; I++) 
			{	
				Xd[6 + I] = v6[  I];       // dxdt = V   (velocities)
				Xd[    I] = acc[  I];      // dVdt = a   (accelerations) 
				Xd[3 + I] = acc[3+I];        // rotational accelerations	
			}            
			
			// rate of change of unit vector components!!  CHECK!   <<<<<
			Xd[9 + 0] =                - v6[5]*r6[4] + v6[4]*r6[5]; // i.e.  u_dot_x = -omega_z*u_y + omega_y*u_z
			Xd[9 + 1] =  v6[5]*r6[3]                 - v6[3]*r6[5]; // i.e.  u_dot_y =  omega_z*u_x - omega_x*u_z
			Xd[9 + 2] = -v6[4]*r6[3] + v6[3]*r6[4]                ; // i.e.  u_dot_z = -omega_y*u_x - omega_x*u_y
		}
	}
	else                             // pinned rod, 6 states (rotational only)
	{
		
		double acc[6];      
		LUsolve6(M_out6, acc, Fnet_out);		// <<< am I approach this right, or could this be reduce to 3X3, and is it neglecting inertial couplings?
		
		for (int I=0; I<3; I++) 
			Xd[    I] = acc[3+I];          // rotational accelerations
		

		// rate of change of unit vector components!!  CHECK!   <<<<<
		Xd[3 + 0] =                - v6[5]*r6[4] + v6[4]*r6[5]; // i.e.  u_dot_x = -omega_z*u_y + omega_y*u_z
		Xd[3 + 1] =  v6[5]*r6[3]                 - v6[3]*r6[5]; // i.e.  u_dot_y =  omega_z*u_x - omega_x*u_z
		Xd[3 + 2] = -v6[4]*r6[3] + v6[3]*r6[4]                ; // i.e.  u_dot_z = -omega_y*u_x - omega_x*u_y
	}	
	
	return 0;
}



// function to return net force on rod (and possibly moment at end A if it's not pinned)
// >>>>>>>>>>>>> do I want to leverage getNetForceAndMass or a saved global to save comp time and code?  >>>>>>>> this function likely not used >>>>>>>>>>>
void Rod::getFnet(double Fnet_out[])
{	
	// Fnet_out is assumed to point to a size-3 array if the rod is pinned and size-6 array if the rod is fixed

	int nDOF = 0;
	
	if (type == CPLDPIN)  // if coupled pinned
		nDOF = 3;
	else if (type == COUPLED) // if coupled rigidly
		nDOF = 6;
	else
		throw string("Error: getFnet called for a Rod that isn't of a coupled type.");
	

	// this assumes doRHS() has already been called
	
	// make sure Fnet_out is zeroed first
	for (int J=0; J<nDOF; J++) 
		Fnet_out[J] = 0.0;
	
	// now go through each node's contributions, put them in body ref frame, and sum them
	for (int i=0; i<=N; i++)
	{		
		double rRel[3];     // position of a given node relative to the body reference point (global orientation frame)
		double Fnet_dub[3];
		double F6_i[6];     // 6dof force-moment from rod about body ref point (but global orientation frame of course)
		
		for (int J=0; J<3; J++) 
		{	rRel[J] = r[i][J] - r[0][J];   // vector from end A to node
			Fnet_dub[J] = Fnet[i][J];      // convert to array for passing
		}	
		
		// convert segment net force into 6dof force about body ref point
		translateForce3to6DOF(rRel, Fnet_dub, F6_i);			
		
				
		for (int J=0; J<nDOF; J++)
			Fnet_out[J] += F6_i[J]; // add force to total force vector
	
	}	
	
	// add any moments applied from lines at either end (might be zero)
	for (int J=0; J<3; J++)
		Fnet_out[J+3] = Fnet_out[J+3] + Mext[J];
	
	//  this is where we'd add inertial loads for coupled rods! <<<<<<<<<<<<
	
	return;
}


// calculate the aggregate 6DOF rigid-body force and mass data of the rod 
void Rod::getNetForceAndMass(double rBody[3], double Fnet_out[6], double M_out[6][6])
{
	// rBody is the location of the body reference point. A NULL pointer value means the end A coordinates should be used instead.

	// question: do I really want to neglect the rotational inertia/drag/etc across the length of each segment?

	doRHS(); // do calculations of forces and masses on each rod node

	// make sure Fnet_out and M_out are zeroed first
	for (int J=0; J<6; J++)
	{
		Fnet_out[J] = 0.0;
		for (int K=0; K<6; K++)
			M_out[J][K] = 0.0;
	}

	double rRef[3];  // global x/y/z coordinate to sum forces, moments, masses about

	// set reference coordinate to return things about
	if (rBody == NULL)            // if this function is called by the Rod itself, the reference point is just end A
		for (int J=0; J<3; J++)
			rRef[J] = r[0][J];
	else
		for (int J=0; J<3; J++)
			rRef[J] = rBody[J];

	// NEW APPROACH:  (6 DOF properties now already summed in doRHS)

	// shift everything from end A reference to rRef reference point
	double rRel[3];     // position of a given node relative to the body reference point (global orientation frame)

	for (int J=0; J<3; J++)  
		rRel[J] = r[0][J] - rRef[J];                // vector from reference point to end A            

	translateForce3to6DOF(rRel, F6net, Fnet_out);   // shift net forces

	for (int J=3; J<6; J++)
		Fnet_out[J] += F6net[J];                    // add in the existing moments

	translateMass6to6DOF(rRel, M6net, M_out);       // shift mass matrix to be about ref point

	// >>> do we need to ensure zero moment is passed if it's pinned? <<<
	//if (abs(Rod%typeNum)==1) then
	//   Fnet_out(4:6) = 0.0_DbKi
	//end if

	// >>> OLD APPROACH:
	/*
	// now go through each node's contributions, put them in body ref frame, and sum them
	for (int i=0; i<=N; i++)
	{		
		double rRel[3];     // position of a given node relative to the body reference point (global orientation frame)
		double Fnet_dub[3];
		double M_dub[3][3];
		double F6_i[6];     // 6dof force-moment from rod about body ref point (but global orientation frame of course)
		double M6_i[6][6];  // mass matrix of each rod to be added		
		
		for (int J=0; J<3; J++) 
		{	rRel[J] = r[i][J] - rRef[J];   // vector from reference point to node
			Fnet_dub[J] = Fnet[i][J];      // convert to array for passing <<<<<<<<< this intermediate array can be skipped
			for (int K=0; K<3; K++) M_dub[J][K] = M[i][J][K];  // <<<<<<<<< this intermediate array can be skipped
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
	
	// add any moments applied from lines at either end (might be zero)
	for (int J=0; J<3; J++)
		Fnet_out[J+3] = Fnet_out[J+3] + Mext[J];
	
	*/
	
	
	return;
}


//  this is the big function that calculates the forces on the rod, including from attached lines
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

	double phi, beta, sinPhi, cosPhi, tanPhi, sinBeta, cosBeta; // various orientation things
	
	double Lsum = 0.0;

    // ---------------------------- initial rod and node calculations ------------------------

    // calculate some orientation information for the Rod as a whole
    GetOrientationAngles(q, &phi, &sinPhi, &cosPhi, &tanPhi, &beta, &sinBeta, &cosBeta);
 
    // save to internal roll and pitch variables for use in output <<< should check these, make Euler angles isntead of independent <<<
    roll  = -180.0/pi * phi*sinBeta;
    pitch =  180.0/pi * phi*cosBeta;

	// set interior node positions and velocities (stretch the nodes between the endpoints linearly) (skipped for zero-length Rods)
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

			
	// note: Rod's q unit tangent vector is already set by setState-type functions (and is same as last three entries of r6)
	
	
	//============================================================================================
	// --------------------------------- apply wave kinematics ------------------------------------

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
			PDyn[i] = 0.0;  // this option is out of date <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<		
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
			
			// >>> add Pd variable for dynamic pressure, which will be applied on Rod surface
			
			F[i] = 1.0; // set VOF value to one for now (everything submerged - eventually this should be element-based!!!) <<<<
		}
	}
	else if (WaterKin != 0) // Hopefully WaterKin is set to zero, meaning no waves or set externally, otherwise it's an error
		cout << "ERROR: We got a problem with WaterKin not being 0,1,2." << endl;

	
	// >>> remember to check for violated conditions, if there are any... <<<
           
	double zeta_i = zeta[N];    // just use the wave elevation computed at the location of the top node for now

	if ((r[0][2] < zeta_i) && (r[N][2] > zeta_i))  // check if it's crossing the water plane (should also add some limits to avoid near-horizontals at some point)
		h0 = (zeta_i - r[0][2])/q[2];              // distance along rod centerline from end A to the waterplane
	else if (r[0][2] < zeta_i)
		h0 = UnstrLen;                             // fully submerged case   <<<<<< remove the 2.0 and double check there are no if statements that get changed <<<<
	else
		h0 = 0.0;                                  // fully unsubmerged case (ever applicable?)
	
	
	
	// -------------------------- loop through all the nodes -----------------------------------
    for (int i=0; i<=N; i++) 
	{
		// calculate mass matrix   <<<< can probably simplify/eliminate this...
		double dL;  // segment length corresponding to the node
		double m_i; // node mass
		double v_i; // node submerged volume 
		double Area = 0.25*pi*d*d;
		
		if (i==0) 
		{	dL = 0.5*l[i];
			m_i = Area*dL*rho;   //  (will be zero for zero-length Rods)
			v_i = 0.5 *F[i]*V[i];
		}
		else if (i==N) 
		{	dL = 0.5*l[i-1];
			m_i = Area*dL*rho;
			v_i = 0.5 *F[i-1]*V[i-1];
		}
		else
		{	dL = 0.5*(l[i-1] + l[i]);
			m_i = Area*dL*rho;
			v_i = 0.5 *(F[i-1]*V[i-1] + F[i]*V[i]);   // <<< remove F term here and above 2 cases??
		}
		
		// get scalar for submerged portion
		double VOF; 
		if (Lsum + dL <= h0)        // if fully submerged 
			VOF = 1.0;
		else if (Lsum < h0)     // if partially below waterline 
			VOF = (h0 - Lsum)/dL;
		else                        // must be out of water
			VOF = 0.0;
		
		Lsum = Lsum + dL;            // add length attributed to this node to the total
		
		// make node mass matrix  (will be zero for zero-length Rods)
		for (int I=0; I<3; I++)
			for (int J=0; J<3; J++)
				M[i][I][J] = m_i*eye(I,J) + env->rho_w*v_i *( Can*(eye(I,J) - q[I]*q[J]) + Cat*q[I]*q[J] );
		
		// mass matrices will be summed up before inversion, near end of this function

		// ============  CALCULATE FORCES ON EACH NODE ===============================

		double vp_mag = 0.0;
		double vq_mag = 0.0;

		if (N > 0)   // this is only nonzero for finite-length rods (skipped for zero-length Rods)
		{
			// note: no nodal axial structural loads calculated since it's assumed rigid, but should I calculate tension/compression due to other loads?

			// weight (now only the dry weight)
			W[i][0] = W[i][1] = 0.0;
			W[i][2] = m_i*(-env->g);

			// buoyance (now calculated based on outside pressure, for submerged portion only)
			// radial buoyancy force from sides
			double Ftemp = -VOF * Area*dL * env->rho_w * env->g * sinPhi;
			Bo[i][0] =  Ftemp*cosBeta*cosPhi;
			Bo[i][1] =  Ftemp*sinBeta*cosPhi;
			Bo[i][2] = -Ftemp*sinPhi;

			// flow velocity calculations       
			double vq_squared = 0.;
			double vp_squared = 0.;
			
			for (int J=0; J<3; J++)  
				vi[J] = U[i][J] - rd[i][J]; // relative flow velocity over node
			
			for (int J=0; J<3; J++) 
			{
				vq[J] = dotProd( vi , q ) * q[J]; 	// tangential relative flow component
				vp[J] = vi[J] - vq[J];					// transverse relative flow component
				vq_squared += vq[J] * vq[J];
				vp_squared += vp[J] * vp[J];
			}
			vp_mag = sqrt(vp_squared);
			vq_mag = sqrt(vq_squared);
			
			// transverse and tangential drag		
			for (int J=0; J<3; J++)  Dp[i][J] = VOF * 0.5*env->rho_w*Cdn*    d*dL * vp_mag * vp[J];
			for (int J=0; J<3; J++)  Dq[i][J] = VOF * 0.5*env->rho_w*Cdt* pi*d*dL * vq_mag * vq[J]; 
			
			// fluid acceleration components for current node				
			for (int J=0; J<3; J++)  {
				aq[J] = dotProd(Ud[i], q) * q[J]; // tangential component of fluid acceleration
				ap[J] = Ud[i][J] - aq[J]; 			// normal component of fluid acceleration
			}
			
			// transverse and axial Froude-Krylov force
			for (int J=0; J<3; J++)  Ap[i][J] = VOF * env->rho_w*(1.+Can)* v_i * ap[J]; 
			for (int J=0; J<3; J++)  Aq[i][J] = 0.0;  // VOF * env->rho_w*(1.+Cat)* v_i * aq[J]; <<< should anything here be included?
			
            // dynamic pressure
            for (int J=0; J<3; J++)  Pd[i][J] = 0.0;  // assuming zero for sides for now, until taper comes into play
            
			// seabed contact (stiffness and damping, vertical-only for now) - updated for general case of potentially anchor or fairlead end in contact
			if (r[i][2] < -env->WtrDpth)
				B[i][2] = ( (-env->WtrDpth-r[i][2])*env->kb - rd[i][2]*env->cb) * d*dL;
			else  
			{	B[i][0] = 0.0;
				B[i][1] = 0.0;
				B[i][2] = 0.0;
			}	
		}
		else
		{	// for zero-length rods, make sure various forces are zero
			for (int J=0; J<3; J++)
			{	W[ i][J] = 0.0;
				Bo[i][J] = 0.0;
				Dp[i][J] = 0.0;
				Dq[i][J] = 0.0;
				Ap[i][J] = 0.0;
				Aq[i][J] = 0.0;
				Pd[i][J] = 0.0;
				B[ i][J] = 0.0;
			}
		}	
			
			
		// ------ now add forces, moments, and added mass from Rod end effects (these can exist even if N==0) -------
         
		// end A
		if ((i==0) && (h0 > 0.0))    // if this is end A and it is submerged 
		{
			// >>> eventually should consider a VOF approach for the ends    hTilt = 0.5*Rod%d/cosPhi <<<
         
            // buoyancy force
            double Ftemp = -VOF * Area * env->rho_w*env->g * r[i][2];
            Bo[i][0] +=  Ftemp*cosBeta*sinPhi; 
            Bo[i][1] +=  Ftemp*sinBeta*sinPhi; 
            Bo[i][2] +=  Ftemp*cosPhi;
         
            // buoyancy moment
            double Mtemp = -VOF * 1.0/64.0*pi*pow(d, 4) * env->rho_w*env->g * sinPhi;
            Mext[0] +=  Mtemp*sinBeta;
            Mext[1] += -Mtemp*cosBeta;
            Mext[2] +=  0.0;
         
            // axial drag
            for (int J=0; J<3; J++) 
				Dq[i][J] += VOF * Area * env->rho_w * Cdt * vq_mag * vq[J];
         
            // Froud-Krylov force
			for (int J=0; J<3; J++)
				Aq[i][J] += VOF * env->rho_w*(1.0+Cat)* (2.0/3.0*pi* pow(d,3) /8.0) * aq[J];
            
            // dynamic pressure force
			for (int J=0; J<3; J++)
				Pd[i][J] += VOF * Area * PDyn[i] * q[J];
            
            // added mass
			for (int I=0; I<3; I++) 
				for (int J=0; J<3; J++) 
					M[i][I][J] += VOF * env->rho_w * (2.0/3.0*pi*pow(d,3)/8.0) * Cat*q[I]*q[J]; 
		}
		 
		if ((i==N) && (h0 >= UnstrLen))  // if this end B and it is submerged (note, if N=0, both this and previous if statement are true)
		{
            // buoyancy force
            double Ftemp = VOF * Area * env->rho_w*env->g * r[i][2];
            Bo[i][0] +=  Ftemp*cosBeta*sinPhi; 
            Bo[i][1] +=  Ftemp*sinBeta*sinPhi; 
            Bo[i][2] +=  Ftemp*cosPhi;
         
            // buoyancy moment
            double Mtemp = VOF * 1.0/64.0*pi*pow(d, 4) * env->rho_w*env->g * sinPhi;
            Mext[0] +=  Mtemp*sinBeta;
            Mext[1] += -Mtemp*cosBeta;
            Mext[2] +=  0.0;
         
            // axial drag
            for (int J=0; J<3; J++) 
				Dq[i][J] += VOF * Area * env->rho_w * Cdt * vq_mag * vq[J];
         
            // Froud-Krylov force
			for (int J=0; J<3; J++)
				Aq[i][J] += VOF * env->rho_w*(1.0+Cat)* (2.0/3.0*pi* pow(d,3) /8.0) * aq[J];
            
            // dynamic pressure force
			for (int J=0; J<3; J++)
				Pd[i][J] += (-VOF * Area * PDyn[i] * q[J]);
            
            // added mass
			for (int I=0; I<3; I++) 
				for (int J=0; J<3; J++) 
					M[i][I][J] += VOF * env->rho_w * (2.0/3.0*pi*pow(d,3)/8.0) * Cat*q[I]*q[J]; 
		}
			
			
		// ----------------- total forces for this node --------------------
		for (int J=0; J<3; J++)
			Fnet[i][J] = W[i][J] + Bo[i][J] + Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J] + Pd[i][J] + B[i][J];
	} // i - done looping through nodes

	// ----- add waterplane moment of inertia moment if applicable -----
	if ((r[0][2] < zeta_i) && (r[N][2] > zeta_i))  // check if it's crossing the water plane
	{
		double Mtemp = 1.0/16.0 *pi* pow(d,4) * env->rho_w * env->g * sinPhi * (1.0 + 0.5* tanPhi*tanPhi);
		Mext[0] +=  Mtemp*sinBeta;
		Mext[1] += -Mtemp*cosBeta;
		Mext[2] +=  0.0;
	}

	// ============ now add in forces on end nodes from attached lines =============

	// zero the external force/moment sums (important!)
	for (int I=0; I<3; I++) 
	{
		FextA[I] = 0.0;
		FextB[I] = 0.0;  
		Mext[ I] = 0.0;
	}

	// loop through lines attached to end A
	for (int l=0; l < nAttachedA; l++)
	{
		double Fnet_i[3] = {0.0}; double Mnet_i[3] = {0.0};  double M_i[3][3] = {{0.0}};

		// get quantities
		AttachedA[l]->getEndStuff(Fnet_i, Mnet_i, M_i, TopA[l]);
			
		// Process outline for line failure, similar to as done for connections (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if failure time has elapsed for line
		// 2. create new massless connect with same instantaneous kinematics as current Rod end
		// 3. disconnect line end from current Rod end and instead attach to new connect
		// The above may require rearrangement of connection indices, expansion of state vector, etc.

		// sum quantitites
		for (int I=0; I<3; I++) 
		{
			Fnet[0][I] += Fnet_i[I];    // forces
			FextA[I]   += Fnet_i[I];    // a copy for outputting totalled line loads
			Mext[I]    += Mnet_i[I];    // moments
		
			for (int J=0; J<3; J++) 
				M[0][I][J] += M_i[I][J]; // mass matrix
        }
	}
	
	// loop through lines attached to end B
	for (int l=0; l < nAttachedB; l++)
	{
		double Fnet_i[3] = {0.0}; double Mnet_i[3] = {0.0}; double M_i[3][3] = {{0.0}};

		// get quantities
		AttachedB[l]->getEndStuff(Fnet_i, Mnet_i, M_i, TopB[l]);

		// sum quantitites
		for (int I=0; I<3; I++) 
		{
			Fnet[N][I] += Fnet_i[I];    // forces
			FextB[I]   += Fnet_i[I];    // a copy for outputting totalled line loads
			Mext[I]    += Mnet_i[I];    // moments
         
			for (int J=0; J<3; J++) 
				M[N][I][J] += M_i[I][J]; // mass matrix
		}
	}


	// ---------------- now lump everything in 6DOF about end A -----------------------------

	// question: do I really want to neglect the rotational inertia/drag/etc across the length of each segment?
   
	// make sure 6DOF quantiaties are zeroed before adding them up
	for (int J=0; J<6; J++)
	{	F6net[J] = 0.0;
		for (int K=0; K<6; K++)
			M6net[J][K] = 0.0;
	}

	// now go through each node's contributions, put them about end A, and sum them
	for (int i=0; i<=N; i++)
	{
		double rRel[3];                   // position of a given node relative to end A node
		double F6_i[6];     // 6dof force-moment from rod about body ref point (but global orientation frame of course)
		double M6_i[6][6];  // mass matrix of each rod to be added		
		double M_dub[3][3]; // 3x3 node mass matrix needed for passing to transformation function
		for (int J=0; J<3; J++) 
			for (int K=0; K<3; K++) 
				M_dub[J][K] = M[i][J][K];

		for (int J=0; J<3; J++)
			rRel[J] = r[i][J] - r[0][J];  // vector from reference end A to node      

		// convert segment net force into 6dof force about end A
		translateForce3to6DOF(rRel, Fnet[i], F6_i);

		// convert segment mass matrix to 6by6 mass matrix about end A
		translateMass3to6DOF(rRel, M_dub, M6_i);

		// sum contributions	 
		for (int J=0; J<6; J++)
		{
			F6net[J] += F6_i[J]; // add force to total force vector
			
			for (int K=0; K<6; K++)
				M6net[J][K] += M6_i[J][K];  // add element mass matrix to body mass matrix
		}
	}

	// ------------- Calculate some items for the Rod as a whole here -----------------

	// >>> could some of these be precalculated just once? <<<
	
	double mass = UnstrLen*0.25*pi*pi*rho;   // rod total mass, used to help with making generic inertia coefficients
		
	// add inertia terms for the Rod assuming it is uniform density (radial terms add to existing matrix which contains parallel-axis-theorem components only)
	double Imat_l[3][3] = {{0.0}};    // inertia about Rod CG in local orientations (as if Rod is vertical)
	if (N > 0)
	{
		double I_l = 0.125*mass * d*d;                                 // axial moment of inertia
		double I_r = mass/12.0 * (0.75*d*d + pow(UnstrLen/N,2)) *N;    // summed radial moment of inertia for each segment individually

		Imat_l[0][0] = I_r;  
		Imat_l[1][1] = I_r;
		Imat_l[2][2] = I_l;
	}

	// get rotation matrix to put things in global rather than rod-axis orientations
	double OrMat[9];
	double OrMat2[3][3];
	double Imat[3][3];
	RotMat(phi, beta, 0.0, OrMat);     // calculate an orientation matrix for the Rod
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) OrMat2[I][J]=OrMat[3*I+J];	// convert DCM formats (should really make this more consistent throughout MD)
	rotateM3(Imat_l, OrMat2, Imat);     // rotate to give inertia matrix about CG in global frame

	// these supplementary inertias can then be added the matrix (these are the terms ASIDE from the parallel axis terms)
	for (int J=0; J<3; J++)
		for (int K=0; K<3; K++)
			M6net[J+3][K+3] += Imat[J][K];

	// now add centripetal and gyroscopic forces/moments, and that should be everything
	/*
	double h_c = 0.5*UnstrLen;          // distance to center of mass
	double r_c[3];
	for (int J=0; J<3; J++)
		r_c[J] = h_c*q[J];                 // vector to center of mass

	// note that Rod%v6(4:6) is the rotational velocity vector, omega   
	for (int J=0; J<3; J++)
	{	Fcentripetal[J] = 0.0; //<<<TEMP<<< -cross_product(Rod%v6(4:6), cross_product(Rod%v6(4:6), r_c ))*Rod%mass <<<
		Mcentripetal[J] = 0.0; //<<<TEMP<<< cross_product(r_c, Fcentripetal) - cross_product(Rod%v6(4:6), MATMUL(Imat,Rod%v6(4:6)))
	}
	*/

	// add centripetal force/moment, gyroscopic moment, and any moments applied from lines at either end (might be zero)
	for (int J=0; J<3; J++)
	{	//F6net[J] += Fcentripetal[J] 
		F6net[J] += Mext[J]; // + Mcentripetal[J] 
	}

	// Note: F6net saves the Rod's net forces and moments (excluding inertial ones) for use in later output
	//       (this is what the rod will apply to whatever it's attached to, so should be zero moments if pinned).
	//       M6net saves the rod's mass matrix.

	return;
}



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
			// output net node forces?
			if (channels.find("f") != string::npos) {
				for (int i=0; i<=N; i++)  {
					for (int J=0; J<3; J++)  *outfile << Fnet[i][J] << "\t ";
				}
			}
			
			*outfile << "\n";
		}
		else cout << "Unable to write to output file " << endl;
	}
	return;
}


Rod::~Rod()
{
	// free memory

	free2Darray(r   , N+1);    
	free2Darray(rd  , N+1);    
	free(l);
	
	free3Darray(M   , N+1, 3); 
	free(V);         
	
	// forces        
	free2Darray(W   , N+1);    
	free2Darray(Bo  , N+1);    
	free2Darray(Pd  , N+1);    
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
	{
		free2Darray(zetaTS, N+1);
		free2Darray(FTS   , N+1);
		free3Darray(UTS   , N+1, ntWater);
		free3Darray(UdTS  , N+1, ntWater);
	}
	
	return;
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
