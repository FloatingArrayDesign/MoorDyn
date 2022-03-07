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

#include "Body.h"
#include "Connection.h"
#include "Rod.h"
#include "Waves.h"

// connection member functions

void Body::setup(int number_in, types type_in, double r6_in[6], double rCG_in[3], double M_in,
	double V_in, double I_in[3], double CdA_in[3], double Ca_in[3], shared_ptr<ofstream> outfile_pointer) 
{
	//props contains: input file has	Name/ID      X0   Y0   Z0   Xcg   Ycg   Zcg      M      V        IX       IY       IZ     CdA-x,y,z Ca-x,y,z
	number = number_in;
	type = type_in;
	
	outfile = outfile_pointer.get(); 		// make outfile point to the right place
	
	
	if (type == FREE)
	{
		bodyM = M_in;
		bodyV = V_in;
		
		for (int J=0; J<3; J++)
		{	body_r6[  J] = r6_in[  J]; 
			body_r6[3+J] = r6_in[3+J]; 
			body_rCG [J] = rCG_in[J];
			bodyI    [J] = I_in[J];
			bodyCdA[J] = CdA_in[J];
			bodyCa [J] =  Ca_in[J];		
		}
	}
	else // other types of bodies have no need for these variables...
	{
		bodyM = 0.0;
		bodyV = 0.0;
		
		for (int J=0; J<3; J++)
		{	body_r6[  J] = 0.0;
			body_r6[3+J] = 0.0;
			body_rCG [J] = 0.0;
			bodyI    [J]  =0.0;
			bodyCdA[J]   = 0.0;
			bodyCa [J]   = 0.0;	
		}
	}
	
	nAttachedC = 0;  // start off with zero connections
	nAttachedR = 0;  // start off with zero rods
	
	// set up body initial mass matrix (excluding any rods or attachements)
	double Mtemp[6][6] = {{0}};
	for (int J=0; J<3; J++)
	{	Mtemp[J][J] = bodyM;         // fill in mass
		Mtemp[3+J][3+J] = bodyI[J];   // fill in inertia
	}	
	translateMass6to6DOF(body_rCG, Mtemp, M0);  // account for potential CG offset <<< is the direction right? <<<
		
	for (int J=0; J<6; J++)
		M0[J][J] += bodyV*bodyCa[J]; // add added mass in each direction about ref point (so only diagonals)

	// --------------- if this is an independent body (not coupled) ----------
	// set initial position and orientation of body from input file 
	for (int J=0; J<6; J++)
	{	r6[J] = body_r6[J];
		v6[J] = 0.0;
	}
	
	// calculate orientation matrix based on latest angles
	RotMat(r6[3], r6[4], r6[5], OrMat);
	
	if (wordy >0)  cout << "Set up Body " << number << ", type " << type << ". " << endl;
	
};


// this function handles assigning a line to a connection node
void Body::addConnectionToBody(Connection *theConnection, double coords[3])
{
	if (wordy>0) cout << "C" << theConnection->number << "->B" << number << " ";	
	
	if (nAttachedC <30) // this is currently just a maximum imposed by a fixed array size. 
	{
		// store Connection address
		attachedC[nAttachedC] = theConnection;
		
		// store Connection relative location
		for (int I=0; I<3; I++)
			rConnectRel[nAttachedC][I] =   coords[I];
		
		nAttachedC += 1;
	}
	// <<<<<<<< add error catch <<<<<<<<
};

// this function handles assigning a line to a connection node
void Body::addRodToBody(Rod *theRod, double endCoords[6])
{
	if (wordy>0) cout << "R" << theRod->number << "->B" << number << " ";

	if (nAttachedR <30) // this is currently just a maximum imposed by a fixed array size. 
	{
		// store Rod address
		attachedR[nAttachedR] = theRod;

		// store Rod end A relative position and unit vector from end A to B
		double tempUnitVec[3];

		unitvector(tempUnitVec, endCoords, endCoords+3);

		for (int I=0; I<3; I++)
		{
            r6RodRel[nAttachedR][  I] =   endCoords[I];
			r6RodRel[nAttachedR][3+I] = tempUnitVec[I];
		}

		nAttachedR += 1;
	}
	// <<<<<<<< add error catch <<<<<<<<
};


void Body::setEnv(EnvCond *env_in, Waves *waves_in)
{
	env = env_in;      // set pointer to environment settings object
	waves = waves_in;  // set pointer to Waves  object
}



// used to initialize bodies that aren't free i.e. don't have states
void Body::initializeUnfreeBody(const double r6_in[6], const double v6_in[6], double time)
{
	initiateStep(r6_in, v6_in, time);
	updateFairlead( time);
	
	// If any Rod is fixed to the body (not pinned), initialize it now because otherwise it won't be initialized
	for (int i=0; i<nAttachedR; i++)
		if (attachedR[i]->type == Rod::FIXED)
			attachedR[i]->initializeRod(NULL); 
	// If there's an attached Point, initialize it now because it won't be initialized otherwise
	for (int i=0; i<nAttachedC; i++)
		attachedC[i]->initializeConnect(NULL); 

	return;
}


// this is only for independent bodies (not bodies that are coupling points)
void Body::initializeBody( double* X )
{
	// assign initial body kinematics to state vector
	for (int I=0; I<6; I++)  {
		X[6 + I] = r6[I];
		X[    I] = v6[I];
	}	
	
	// set positions of any dependent connections and rods now (before they are initialized)
	setDependentStates(); 	
	
	
	// If the Rod is fixed to the body (not pinned), initialize it now because it won't be initialized otherwise
	for (int i=0; i<nAttachedR; i++)
		if (attachedR[i]->type == Rod::FIXED)
			attachedR[i]->initializeRod(NULL); 
	// If there's an attached Point, initialize it now because it won't be initialized otherwise
	for (int i=0; i<nAttachedC; i++)
		attachedC[i]->initializeConnect(NULL); 
	
	
	
	
	// create output file for writing output (and write channel header and units lines) if applicable
				
	if (outfile) // check it's not null.  Null signals no individual line output files
	{
		if (outfile->is_open())
		{	
			// ------------- write channel names line --------------------
		
			// output time
			*outfile << "Time" << "\t ";
			
			*outfile << "x\ty\tz\troll\tpitch\tyaw";
			
			*outfile << "\n";   
			
			
			// ----------- write units line ---------------

			if (env->WriteUnits > 0)
			{
				// output time
				*outfile << "(s)" << "\t ";
				
				*outfile << "(m)\t(m)\t(m)\t(deg)\t(deg)\t(deg)";
				
				*outfile << "\n";   // should also write units at some point!
			}
		}
		else cout << "   Error: unable to write file Body" << number << ".out" << endl;  //TODO: handle this!
	}
	
	
	
	
	
	if (wordy>0) cout << "Initialized Body " << number << endl;
	
	return;
};


// function to return body position and velocity to Connection object
void Body::getBodyState(double r_out[6], double rd_out[6])
{
	for (int J=0; J<6; J++) {
		r_out[J] = r6[J];
		rd_out[J] = v6[J];
	}
};


// function to return net forces and moments on body (just to allow public reading of Fnet)
void Body::getFnet(double Fnet_out[]) const
{
	for (int I=0; I<6; I++) 	Fnet_out[I] = F6net[I];
};


// function to return mass matrix of body
void Body::getM(double M_out[6][6])
{
	for (int I=0; I<6; I++) 	
		for (int J=0; J<6; J++) 
			M_out[I][J] = M[I][J];	
};


double Body::GetBodyOutput(OutChanProps outChan)
{
	if      (outChan.QType == PosX)  return  r6[0];
	else if (outChan.QType == PosY)  return  r6[1];
	else if (outChan.QType == PosZ)  return  r6[2];
	else if (outChan.QType == VelX)  return  v6[0];
	else if (outChan.QType == VelY)  return  v6[1];
	else if (outChan.QType == VelZ)  return  v6[2];
	//else if (outChan.QType == Ten )  return  sqrt(Fnet[0]*Fnet[0] + Fnet[1]*Fnet[1] + Fnet[2]*Fnet[2]);
	else if (outChan.QType == FX)    return  F6net[0];  // added Oct 20
	else if (outChan.QType == FY)    return  F6net[1];
	else if (outChan.QType == FZ)    return  F6net[2];
	else
	{
		return 0.0;
		//ErrStat = ErrID_Warn
		//ErrMsg = ' Unsupported output quantity from Connect object requested.'
	}	
}



// function for boosting drag coefficients during IC generation	
void Body::scaleDrag(double scaler)
{
	for (int I=0; I<6; I++) bodyCdA[I] = bodyCdA[I]*scaler;
	return;
}

// function to reset time after IC generation
void Body::setTime(double time)
{
	t = time;
	return;
}



// called at the beginning of each coupling step to update the boundary conditions (body kinematics) for the proceeding time steps
void Body::initiateStep(const double r_in[6], const double rd_in[6], double time)
{
	
	t0 = time; // set start time for BC functions
	
	if (type==COUPLED)   // if coupled, update boundary conditions
	{
		for (int J=0; J<6; J++)  
		{
			r_ves[ J] = r_in[ J];
			rd_ves[J] = rd_in[J];
		}
	}
	else if (type==FIXED)   // if the ground body, set the BCs to stationary
	{
		for (int J=0; J<6; J++)  
		{
			r_ves[ J] = 0.0;
			rd_ves[J] = 0.0;
		}
	}
	else
		throw string("Error Body::initiateStep called for a body that isn't coupled/fixed!");

}


// sets Body kinematics ONLY if this body is driven externally (otherwise shouldn't be called)
void Body::updateFairlead( const double time)
{
	// store current time
	t = time;
	
	if ((type==COUPLED) || (type==FIXED))   // if coupled OR GROUND BODY
	{		
		// set Body kinematics based on BCs (linear model for now) 
		for (int J=0; J<6; J++)  
		{
			r6[J] = r_ves[J] + rd_ves[J]*(time-t0);
			v6[J] = rd_ves[J];
		}	
		
		// calculate orientation matrix based on latest angles
		RotMat(r6[3], r6[4], r6[5], OrMat);
		
		// set positions of any dependent connections and rods
		setDependentStates(); 
	}
	else
		throw string("Error Body::updateFairlead called for body that isn't coupled/fixed!");
	
	return;
}


// pass the latest states to the body if this body is NOT driven externally
void Body::setState( const double* X, const double time)
{
	// store current time
	t = time;
		
	// set position and velocity vectors from state vector
	for (int J=0; J<6; J++) 	
	{
		r6[J] = X[6 + J]; // get positions
		v6[J] = X[J]; // get velocities
	}		
	
	// calculate orientation matrix based on latest angles
	RotMat(r6[3], r6[4], r6[5], OrMat);
	
	// set positions of any dependent connections and rods
	setDependentStates(); 
	
	return;
}
	
	
// set the states (positions and velocities) of any connects or rods that are part of this body
void Body::setDependentStates()
{
	// set kinematics of any dependent connections (this is relevant for the dependent lines, yeah?)
	for (int c=0; c < nAttachedC; c++)
	{
		double rConnect[3];
		double rdConnect[3]; // this is making a "fake" state vector for the connect, describing its position and velocity
		
		transformKinematics(rConnectRel[c], r6, OrMat, v6, rConnect, rdConnect); //<<< should double check this function
					
		// pass above to the connection and get it to calculate the forces
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			attachedC[c]->setKinematics(rConnect, rdConnect);
		}
		MOORDYN_CATCHER(err, err_msg);
		// BUG: HANDLE THE ERROR
	}
	
	// set kinematics of any dependent Rods
	for (int i=0; i<nAttachedR; i++)
	{
		// calculate displaced coordinates/orientation and velocities of each rod <<<<<<<<<<<<<
		double rRod[6]; // this is making a "fake" state vector for the rod, describing its position and velocity
		double rdRod[6]; 
					
		// do 3d details of Rod ref point
		transformKinematics(r6RodRel[i], r6, OrMat, v6, rRod, rdRod);  // set first three entires (end A translation) of rRod and rdRod
		// does the above function need to take in all 6 elements of r6RodRel??
		
		//double rodTempUnitVec[3];
		rotateVector3(r6RodRel[i]+3, OrMat, rRod+3);   // rotate rod relative unit vector by OrMat to get unit vec in reference coords
				
		// do rotational stuff	
		for (int I=0; I<3; I++) 
		{
		//	rRod[ I+3] = rodTempUnitVec[I];  // set rod unit vector 
			rdRod[I+3] = v6[I+3];  // is this okay as is?
		}				
		
		// pass above to the rod and get it to calculate the forces
		attachedR[i]->setKinematics(rRod, rdRod);
	}
	return;
}


// calculate the forces and state derivatives of the body	
int Body::getStateDeriv(double* Xd)
{	

	if (type==FREE)  // this should ONLY be called for free bodies
	{
		// Get contributions from attached connections (and lines attached to them)

		// with current IC gen approach, we skip the first call to the line
		// objects, because they're set AFTER the call to the connects
		// above is no longer true!!! <<<
		if (t==0)
		{
			for (int I=0; I<6; I++)  {
				Xd[6+I] = v6[I];  // velocities - these are unused in integration
				Xd[I] = 0.;     // accelerations - these are unused in integration
			}
		}//<<<<<<<<<<<<<<<<<<<<
		else
		{
			doRHS();

			// solve for accelerations in [M]{a}={f} using LU decomposition
		//	double M_tot[36];                     // serialize total mass matrix for easy processing
		//	for (int I=0; I<6; I++) for (int J=0; J<6; J++) M_tot[6*I+J]=M[I][J];
		//	double LU[36];                        // serialized matrix that will hold LU matrices combined
		//	Crout(6, M_tot, LU);                  // perform LU decomposition on mass matrix
			double acc[6];                        // acceleration vector to solve for
		//	solveCrout(6, LU, F6net, acc);     // solve for acceleration vector

			LUsolve6(M, acc, F6net);
			

			// fill in state derivatives
			for (int I=0; I<6; I++) 
			{
				Xd[6 + I] = v6[I];    // dxdt = V    (velocities)
				Xd[I] = acc[I];      // dVdt = RHS * A  (accelerations)
			}		
			// is the above still valid even though it includes rotational DOFs? <<<<<<<
		}
	}
	else
	{
		throw string("Error, GetStateDeriv called for a body this is not free!");
	}
	return 0;		
};


//  this is the big function that calculates the forces on the body
void Body::doRHS()
{
	// TODO: somewhere should check for extreme orientation changes, i.e. "winding" close to 2pi, and maybe prevent it if it risks compromising angle assumptions <<<<<<
	
	// clear before re-summing
	for (int I=0; I<6; I++) { 
		F6net[I] = 0;		
		for (int J=0; J<6; J++)
			M[I][J] = 0.0; 		
	}

	// First, the body's own mass matrix must be adjusted based on its orientation so that 
	// we have a mass matrix in the global orientation frame	
	double OrMat2[3][3]; 
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) OrMat2[I][J]=OrMat[3*I+J];
	rotateM6(M0, OrMat2, M);

	// gravity on core body
	double body_rCGrotated[3];
	double Fgrav[3] = {0.0}; Fgrav[2] = bodyV*env->rho_w*env->g - bodyM*env->g; // weight+buoyancy vector
	rotateVector3(body_rCG, OrMat, body_rCGrotated);            // relative vector to body CG in inertial orientation
	translateForce3to6DOF(body_rCGrotated, Fgrav, F6net);      // gravity forces and moments about body ref point given CG location

	// --------------------------------- apply wave kinematics ------------------------------------
	
	//env->waves->getU(r6, t, U); // call generic function to get water velocities <<<<<<<<< all needs updating
	
	for (int J=0; J<3; J++)		
		Ud[J] = 0.0;                 // set water accelerations as zero for now
	
	// ------------------------------------------------------------------------------------------
	
	
	// viscous drag calculation (on core body)
	
	double vi[6]; // relative water velocity (last 3 terms are rotatonal and will be set to zero
	
	for (int J=0; J<3; J++) 
	{
		vi[J] = U[J] - v6[  J]; // relative flow velocity over body ref point
		vi[3+J] =    - v6[3+J]; // for rotation, this is just the negative of the body's rotation for now (not allowing flow rotation)
	}	
	for (int J=0; J<6; J++)
		F6net[J] += 0.5*env->rho_w*vi[J]*abs(vi[J])*bodyCdA[J]; //<<<
	  // <<< NOTE, for body this should be fixed to account for orientation!! <<< what about drag in rotational DOFs??? <<<<<<<<<<<<<<

	// Get contributions from any connections attached to the body
	for (int c=0; c < nAttachedC; c++)
	{			
		//double Fnet_i[3];
		//double M_i[3][3];
		
		// get net force and mass from connection (global orientation)
		//attachedC[c]->getFnet(Fnet_i);
		//attachedC[c]->getM(M_i);
		
		// calculate relative location of connection about body center in global orientation
		//double rConnect_i[3];
		//rotateVector3(rConnectRel[c], OrMat, rConnect_i)
					
		// convert force into 6dof force based on connection position
		double F6_i[6];  // 6dof force-moment from connection about body ref point (but global orientation frame of course)
		//translateForce3to6DOF(rConnect_i, Fnet_i, F6_i);
			
		// transform mass matrix to 6dof one about body center
		double M6_i[6][6];
		//translateMass3to6DOF(rConnect_i, M_i, M6_i);
					
					
		// get net force and mass from Connection on body ref point (global orientation)
		attachedC[c]->getNetForceAndMass(r6, F6_i, M6_i);
		
		// sum quantitites
		for (int I=0; I<6; I++) {
			F6net[I] += F6_i[I];
		
			for (int J=0; J<6; J++) 
				M[I][J] += M6_i[I][J];
		}
	}


	// Get contributions from any rods that are part of the body
	for (int i=0; i<nAttachedR; i++)
	{
		
		double F6_i[6];
		double M6_i[6][6];
		
		// get net force and mass from Rod on body ref point (global orientation)
		attachedR[i]->getNetForceAndMass(r6, F6_i, M6_i);
		
//			// calculate relative location of rod about body center in global orientation
//			double rRod_i[3]; 
//			rotateVector3(r6Rod[c], OrMat, rRod_i);   // this will only consider 3d position of rod (not orientation)
//						
//			// convert force into 6dof force based on rod position
//			double F6_i;  // 6dof force-moment from rod about body ref point (but global orientation frame of course)
//			translateForce6DOF(rRod_i, Fnet_i, F6_i);
//								
//			// transform mass matrix to 6dof one about body center
//			double M6_i[6][6];
//			translateMassInertia3to6DOF(M_i, I_i, rRod_i, M6_i); <<<<<<<<<
					
		// sum quantitites
		for (int I=0; I<6; I++) {
			F6net[I] += F6_i[I];
		
			for (int J=0; J<6; J++) 
				M[I][J] += M6_i[I][J];
		}			
		
	}

	return;
}	
	
	

// write output file for body
void Body::Output(double time)
{
	if (outfile) // if not a null pointer (indicating no output)
	{
		if (outfile->is_open())
		{
			// output time
			*outfile << time << "\t "; 
		
			for (int J=0; J<3; J++)  
				*outfile << r6[J] << "\t ";
			
			*outfile << r6[3]*rad2deg << "\t " << r6[4]*rad2deg << "\t " << r6[5]*rad2deg << "\n";
		}
		//else cout << "Unable to write to output file " << endl;
	}
	return;
};
	
	
Body::~Body()
{
	// destructor (nothing to destroy!)
	
}

// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void Body::drawGL(void)
{
	double radius = pow( BodyV/(4/3*pi), 0.33333);  //conV
	Sphere(r[0], r[1], r[2], radius);
};
#endif
