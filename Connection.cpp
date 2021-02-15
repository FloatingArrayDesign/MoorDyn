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

#include "Connection.h"
#include "Line.h"

// connection member functions

void Connection::setup(ConnectProps& props) 
{
	//props contains: Node      Type      X        Y         Z        M        V        FX       FY      FZ  CdA  Ca
	
	number = props.number;
	
	for (int i=0; i<props.type.length(); i++)
		props.type[i] = tolower(props.type[i]);				// convert to lower case for case independence
	
	if (props.type.find("fix") != string::npos)
		type = 0;
	else if (props.type.find("ves") != string::npos)
		type = 1;
	else if (props.type.find("con") != string::npos)
		type = 2;
	else {
		cout << "   Error: could not recognise type of connection " << number << "  (" << props.type << ")." << endl;
		return;
	}
	
	conX  = props.X ;
	conY  = props.Y ;
	conZ  = props.Z ;
	conM  = props.M ;
	conV  = props.V ;
	conFX = props.FX;
	conFY = props.FY;
	conFZ = props.FZ;
	conCdA= props.CdA;
	conCa = props.Ca;
	
	t=0.;		
	//beta = 0.0;
	
	nAttached = 0;  // start off with zero connections
	
	// size vectors (could change a lot of these to arrays)
	 r.resize(3, 0.0);				// node positions [i][x/y/z]
	rd.resize(3, 0.0);				// node velocities [i][x/y/z]
	 q.resize(3, 0.0);     			// unit tangent vectors for each node
	 
	r[0] = conX;  // start off position at that specified in input file 
	r[1] = conY;  //  (will be starting point for connect connections
	r[2] = conZ;  //   and the permanent location of anchor connections.)
						
	 r_ves.resize(3, 0.0);	
	rd_ves.resize(3, 0.0);	
			
	Fnet.resize(3, 0.0);	// total force on node
	Fnet_i.resize(3, 0.0);
	RHS.resize (3, 0.0);	// RHS of state-space equation (Forces divided by mass matrix)
	
	S.resize(3, vector< double >(3, 0.0));  // inverse mass matrices (3x3) for each node
	M.resize(3, vector< double >(3, 0.0)); // node mass + added mass matrix
	M_i.resize(3, vector< double >(3, 0.0));
};


// this function handles assigning a line to a connection node
void Connection::addLineToConnect(Line& theLine, int TopOfLine)
{
	if (wordy>0) cout << "L" << theLine.number << "->N" << number << " ";
	
	
	if (nAttached <10) // this is currently just a maximum imposed by a fixed array size.  could be improved.
	{
		Attached[nAttached] = &theLine;
		Top[nAttached] = TopOfLine;
		nAttached += 1;
	}
};


// function to return connection position and velocity to Line object
void Connection::getConnectState(vector<double> &r_out, vector<double> &rd_out)
{
	for (int J=0; J<3; J++) {
		r_out[J] = r[J];
		rd_out[J] = rd[J];
	}
};


// function to return net force on fairlead (just to allow public reading of Fnet
void Connection::getFnet(double Fnet_out[])
{
	for (int I=0; I<3; I++) 	Fnet_out[I] = Fnet[I];
	
	//Fnet_out[2] += M[0][0]*(-env.g); // add weight  NO this is alread in Fnet !!! (removed Oct 20)
	// should it include inertial "force"?  i.e.	for (int J=0; J<3; J++)  Fnet_out += M[I][J]*acceleration[J] 	
};


double Connection::GetConnectionOutput(OutChanProps outChan)
{
	if      (outChan.QType == PosX)  return  r[0];
	else if (outChan.QType == PosY)  return  r[1];
	else if (outChan.QType == PosZ)  return  r[2];
	else if (outChan.QType == VelX)  return  rd[0];
	else if (outChan.QType == VelY)  return  rd[1];
	else if (outChan.QType == VelZ)  return  rd[2];
	else if (outChan.QType == Ten )  return  sqrt(Fnet[0]*Fnet[0] + Fnet[1]*Fnet[1] + Fnet[2]*Fnet[2]);
	else if (outChan.QType == FX)    return  Fnet[0];  // added Oct 20
	else if (outChan.QType == FY)    return  Fnet[1];
	else if (outChan.QType == FZ)    return  Fnet[2];
	else
	{
		return 0.0;
		//ErrStat = ErrID_Warn
		//ErrMsg = ' Unsupported output quantity from Connect object requested.'
	}	
}


void Connection::setEnv(EnvCond env_in)
{
	env = env_in; // needed only for buoyancy calcs on connections that have a volumetric displacement
}


void Connection::initializeFairlead( double pX[], double TransMat[] )
{
	if (type==1)  // error check
	{	
		r[0] = TransMat[0]*conX + TransMat[1]*conY + TransMat[2]*conZ + pX[0];	// x
		r[1] = TransMat[3]*conX + TransMat[4]*conY + TransMat[5]*conZ + pX[1];	// y
		r[2] = TransMat[6]*conX + TransMat[7]*conY + TransMat[8]*conZ + pX[2];	// z
		
		for (int I=0; I<3; I++) rd[I] = 0.0;
		
		// also specify the above in the "vessel" arrays that prescribe the kinematics over the following time steps, for IC gen
		for (int J=0; J<3; J++)  {
			r_ves[J] = r[J];
			rd_ves[J] = 0.0;
		}
	}
	else  cout << "   Error: wrong connection type given to initializeFairlead().  Something's not right." << endl;
	// TODO: should handle.

	return;
};



void Connection::initializeConnect( double* X )
{
	if (type==2)  // error check
	{	
		// assign initial node kinematics to state vector
		for (int I=0; I<3; I++)  {
			X[3 + I] = r[I];
			X[    I] = rd[I];
		}
	}
	else  cout << "   Error: wrong connection type given to initializeConnect().  Something's not right." << endl;
	// TODO: should handle.

	return;
};


// helper function to sum forces and mass from attached lines - used for connect dynamics and fair/anch tensions
void Connection::getNetForceAndMass()
{
	// loop through each connected line, summing to get the final result
	
	int Nl = nAttached;				// number of attached line segments
	
	//cout << "Connection " << number << " nAttached is " << nAttached << endl;
	
	// clear before re-summing	
	for (int I=0; I<3; I++) { 
		Fnet[I] = 0;		
		for (int J=0; J<3; J++)
			M[I][J] = 0.0; 		
	}
	
	// loop through attached lines
	for (int l=0; l < Nl; l++)
	{
		// get quantities
		if (Top[l] == 0) 		// if attached to bottom/anchor of a line...
		{	(Attached[l])->getAnchStuff(Fnet_i, M_i);
			//cout << "Att. to bot of line " << (Attached[l])->number << " F:" << Fnet_i[0] << " " << Fnet_i[1] << " " << Fnet_i[2] << endl;
		}
		else 				// attached to top/fairlead
		{	(Attached[l])->getFairStuff(Fnet_i, M_i);
			//cout << "Att. to top of line " << (Attached[l])->number << " F:" << Fnet_i[0] << " " << Fnet_i[1] << " " << Fnet_i[2] << endl;
		}
			
		// sum quantitites
		for (int I=0; I<3; I++) {
			Fnet[I] += Fnet_i[I];
		
			for (int J=0; J<3; J++) 
				M[I][J] += M_i[I][J];					
		}
	}
		
	// add constant quantities for connection if applicable from input file
	Fnet[0] += conFX;
	Fnet[1] += conFY;
	Fnet[2] += conFZ + conV*env.rho_w*env.g - conM*env.g; 
	for (int I=0; I<3; I++) 	M[I][I] += conM;

}

	
//   this is the function that updates the states - also includes hydrodynamic forces
void Connection::doRHS( const double* X,  double* Xd, const double time)
{
	t = time;
	
	// below now called by master RHS function, for all connection types
	//getNetForceAndMass(); // in all cases, call to get net force and mass of connection (especially for returning fairlead forces to FAST)
	
	// ------ behavior dependant on connect type -------
	
//	if (type==0) // fixed type
//	{
//		r[0] = conX;
//		r[1] = conY;
//		r[2] = conZ;
//		for (int I=0; I<3; I++) rd[I] = 0;
//	}
//	else if (type==1) // vessel (moves with platform)		{
//	{						
//		// set fairlead position and velocity based on BCs (linear model for now)
//		for (int J=0; J<3; J++)  {
//			r[J] = r_ves[J] + rd_ves[J]*(t-t0);
//			rd[J] = rd_ves[J];
//		}		
//		
//		// assign states
//		for (int I=0; I<3; I++)  {
//			Xd[3+I] = rd[I];  // velocities - these are unused in integration
//			Xd[I] = 0.;     // accelerations - these are unused in integration
//		}
//	}			
	if (type==2) // "connect" type
	{
		if (t==0)   // with current IC gen approach, we skip the first call to the line objects, because they're set AFTER the call to the connects
		{		// above is no longer true!!! <<<
			for (int I=0; I<3; I++)  {
				Xd[3+I] = X[I];  // velocities - these are unused in integration
				Xd[I] = 0.;     // accelerations - these are unused in integration
			}
		}
		else
		{			
			// from state values, get r and rdot values 
			for (int J=0; J<3; J++) 	{
				r[J]  = X[3 + J]; // get positions
				rd[J] = X[J]; // get velocities
			}
			
			//cout << "ConRHS: m: " << M[0][0] << ", f: " << Fnet[0] << " " << Fnet[1] << " " << Fnet[2] << endl;
				
			// add dynamic quantities for connection as specified in input file (feature added 2015/01/15)
			Fnet[0] -= 0.5*env.rho_w*rd[0]*abs(rd[0])*conCdA;
			Fnet[1] -= 0.5*env.rho_w*rd[1]*abs(rd[1])*conCdA;
			Fnet[2] -= 0.5*env.rho_w*rd[2]*abs(rd[2])*conCdA;
			for (int I=0; I<3; I++) 	M[I][I] += conV*env.rho_w*conCa;
			
			// invert node mass matrix
			inverse3by3(S, M);
			
			// RHS constant - (premultiplying force vector by inverse of mass matrix  ... i.e. rhs = S*Forces
			for (int I=0; I<3; I++) 
			{
				double RHSI = 0.0; // temporary accumulator 
				for (int J=0; J<3; J++) {
					RHSI += S[I][J] * Fnet[J]; //  matrix multiplication [S i]{Forces i}
				}
				
				// update states
				Xd[3 + I] = X[I];    // dxdt = V    (velocities)
				Xd[I] = RHSI;      // dVdt = RHS * A  (accelerations)
			}
		}
	}
	else
		cout << "Error: wrong connection type sent to doRHS().  " << endl;
		//TODO: handle error
		
};


// called at the beginning of each coupling step to update the boundary conditions (fairlead kinematics) for the proceeding line time steps
void Connection::initiateStep(double rFairIn[3], double rdFairIn[3], double time)
{	
	t0 = time; // set start time for BC functions
	
	if (type==1)  {  // if vessel type 
		// update values to fairlead position and velocity functions (fn of time)
		for (int J=0; J<3; J++)  {
			r_ves[J] = rFairIn[J];
			rd_ves[J] = rdFairIn[J];
		}
	}
	
	// do I want to get precalculated values here at each FAST time step or at each line time step?
};


void Connection::updateFairlead( const double time)
{	
	t = time;

	if (type==1) // vessel (moves with platform)		
	{						
		// set fairlead position and velocity based on BCs (linear model for now)
		for (int J=0; J<3; J++)  {
			r[J] = r_ves[J] + rd_ves[J]*(time-t0);
			rd[J] = rd_ves[J];
		}	
	}
	else
		cout << "Error: wrong type sent to updateFairlead." << endl;
	
	return;
}
	
Connection::~Connection()
{
	// destructor
	 r.clear();				// node positions [i][x/y/z]
	rd.clear();				// node velocities [i][x/y/z]
	 q.clear();     			// unit tangent vectors for each node
	 
	 r_ves.clear();	
	rd_ves.clear();	
			
	Fnet.clear();	// total force on node
	Fnet_i.clear();
	RHS.clear();	// RHS of state-space equation (Forces divided by mass matrix)
	
	S.clear();  // inverse mass matrices (3x3) for each node
	M.clear(); // node mass + added mass matrix
	M_i.clear();
	
}

// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void Connection::drawGL(void)
{
	double radius = pow( conV/(4/3*pi), 0.33333);  //conV
	Sphere(r[0], r[1], r[2], radius);
};
#endif