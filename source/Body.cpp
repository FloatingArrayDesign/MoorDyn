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

// connection member functions

void Body::setup(int number_in, double r6_in[6], double rCG_in[3], double M_in,
	double V_in, double I_in[3], double CdA_in[3], double Ca_in[3]) 
{
	//props contains: input file has	Name/ID      X0   Y0   Z0   Xcg   Ycg   Zcg      M      V        IX       IY       IZ     CdA-x,y,z Ca-x,y,z
	
	// copy in input file data
	number = number_in;
	
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
	{	r6[ J] = body_r6[J];
		r6d[J] = 0.0;
	}
	
	// calculate orientation matrix based on latest angles
	RotMat(r6[3], r6[4], r6[5], OrMat);
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
		double dummyLength;
		directionAndLength(endCoords, endCoords+3, tempUnitVec, &dummyLength);
		for (int I=0; I<3; I++)
		{	r6RodRel[nAttachedR][  I] =   endCoords[I];
			r6RodRel[nAttachedR][3+I] = tempUnitVec[I];
		}
		
		nAttachedR += 1;
		
	}
	// <<<<<<<< add error catch <<<<<<<<
};


void Body::setEnv(EnvCond env_in)
{
	env = env_in;
}


// this is only for independent bodies (not bodies that are coupling points)
void Body::initializeBody( double* X )
{
	// assign initial body kinematics to state vector
	for (int I=0; I<6; I++)  {
		X[6 + I] = r6[I];
		X[    I] = r6d[I];
	}	
	
	// also, set positions of any dependent connections and rods now (before they are initialized)
	setDependentStates(); 	
	
	// Rods need to be initialize to set their node points, so do that for any dependent rods now
	double dummyStateVector[12];
	for (int i=0; i<nAttachedR; i++)
		attachedR[i]->initializeRod(dummyStateVector);
	// connects don't need this
	
	return;
};


// set the states (positions and velocities) of any connects or rods that are part of this body
void Body::setDependentStates()
{
	// set position of any dependent connections (this is relevant for the dependent lines, yeah?)
	for (int c=0; c < nAttachedC; c++)
	{
		double connectFakeState[6]; // this is making a "fake" state vector for the connect, describing its position and velocity
		
		double rRel[3];
		for (int I=0; I<3; I++) rRel[I] = rConnectRel[c][I];  // convert vector to array
		transformKinematics(rRel, r6, OrMat, r6d, (connectFakeState+3), connectFakeState); //<<< should double check this function
					
		attachedC[c]->setState(connectFakeState, t);
	}
	
	// set position of any dependent Rods
	for (int i=0; i<nAttachedR; i++)
	{
		// calculate displaced coordinates/orientation and velocities of each rod <<<<<<<<<<<<<
		double rodFakeState[12]; // this is making a "fake" state vector for the rod, describing its position and velocity
					
		// do 3d details of Rod ref point
		double rRel[3];  for (int I=0; I<3; I++) rRel[I] = r6RodRel[i][I];  // convert vector to array
		
		transformKinematics(rRel, r6, OrMat, r6d, rodFakeState, (rodFakeState+6));
		// does the above function need to take in all 6 elements of r6RodRel??
		
		// do rotational stuff <<<<<<<<<< need to fix <<<<<<<<<<<<<		
		for (int I=0; I<3; I++) 
		{
			rodFakeState[I+3] =  r6[I+3] + r6RodRel[i][I+3];  // set angles << can they just be added like this?? <<<<<
			rodFakeState[I+9] = r6d[I+3];  // is this okay as is? <<< probably not <<<
		}				
		
		// pass above to the rod and get it to calculate the forces
		attachedR[i]->setState(rodFakeState, t);
	}
	return;
}



// function to return body position and velocity to Connection object
void Body::getBodyState(double r_out[6], double rd_out[6])
{
	for (int J=0; J<6; J++) {
		r_out[J] = r6[J];
		rd_out[J] = r6d[J];
	}
};


// function to return net force on connection (just to allow public reading of Fnet)
void Body::getFnet(double Fnet_out[])
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
	else if (outChan.QType == VelX)  return  r6d[0];
	else if (outChan.QType == VelY)  return  r6d[1];
	else if (outChan.QType == VelZ)  return  r6d[2];
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



// pass the latest states to the body
void Body::setState( const double* X, const double time)
{
	// store current time
	t = time;
	
	// set position and velocity vectors from state vector
	for (int J=0; J<6; J++) 	
	{
		r6[J]  = X[6 + J]; // get positions
		r6d[J] = X[J]; // get velocities
	}		
	
	// calculate orientation matrix based on latest angles
	RotMat(r6[3], r6[4], r6[5], OrMat);
	
	// set positions of any dependent connections and rods
	setDependentStates(); 
	
	return;
}
	
	
// calculate the forces and state derivatives of the body	
int Body::getStateDeriv(double* Xd)
{	

	// Get contributions from attached connections (and lines attached to them)
	
	if (t==0)   // with current IC gen approach, we skip the first call to the line objects, because they're set AFTER the call to the connects
	{		// above is no longer true!!! <<<
		for (int I=0; I<6; I++)  {
			Xd[6+I] = r6d[I];  // velocities - these are unused in integration
			Xd[I] = 0.;     // accelerations - these are unused in integration
		}
	}//<<<<<<<<<<<<<<<<<<<<
	else
	{
		// clear before re-summing	(probably not necessary) <<<
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
		double Fgrav[3] = {0.0}; Fgrav[2] = bodyV*env.rho_w*env.g - bodyM*env.g; // weight+buoyancy vector
		rotateVector3(body_rCG, OrMat, body_rCGrotated);  // relative vector to body CG in inertial orientation
		translateForce3to6DOF(body_rCGrotated, Fgrav, F6net);      // gravity forces and moments about body ref point given CG location
		
		// drag on core body
		for (int I=0; I<6; I++)
			F6net[I] -= 0.5*env.rho_w*r6d[I]*abs(r6d[I])*bodyCdA[I]; //<<<
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
			attachedC[c]->getNetForceAndMassContribution(r6, F6_i, M6_i);		
						
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
			attachedR[i]->getNetForceAndMassContribution(r6, F6_i, M6_i);
			
			

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
				
		// solve for accelerations in [M]{a}={f} using LU decomposition
		double M_tot[36];                     // serialize total mass matrix for easy processing
		for (int I=0; I<6; I++) for (int J=0; J<6; J++) M_tot[6*I+J]=M[I][J];
		double LU[36];                        // serialized matrix that will hold LU matrices combined
		Crout(6, M_tot, LU);                  // perform LU decomposition on mass matrix
		double acc[6];                        // acceleration vector to solve for
		solveCrout(6, LU, F6net, acc);     // solve for acceleration vector
		
		// update states
		for (int I=0; I<6; I++) 
		{
			Xd[6 + I] = r6d[I];    // dxdt = V    (velocities)
			Xd[I] = acc[I];      // dVdt = RHS * A  (accelerations)
		}		
		// is the above still valid even though it includes rotational DOFs? <<<<<<<
	}
	
	return 0;		
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