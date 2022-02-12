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

 // This is version 2.a5, 2021-03-16
 
#include "Misc.h"
#include "Waves.h"
#include "MoorDyn.h"
#include "Line.h"
#include "Connection.h" 
#include "Rod.h" 
#include "Body.h"

#ifdef LINUX
	#include <cmath> 	// already in misc.h?
	#include <ctype.h>
#endif


using namespace std;

// static vectors for fairleads
//vector<double> FlinesS;					// net line force vector (6-DOF) - retains last solution for use when inputted dt is zero (such as during FAST predictor steps) when the model does not time step
//vector< vector< double > > rFairtS;		// fairlead locations in turbine/platform coordinates
//vector< vector< double > > rFairRel;		// fairlead locations relative to platform ref point but in inertial orientation
double** rFairi;			// fairlead locations in inertial reference frame
double** rdFairi;		// fairlead velocities in inertial reference frame

double** FairTensLast; // previous fairlead tensions over last n time steps, used for IC gen (nFair x 10 )

// static vectors to hold line and connection objects
LineProps** LinePropList;         // array of pointers to hold line library types
RodProps** RodPropList;           // array of pointers to hold rod library types
FailProps** FailList;           // array of pointers to hold failure condition structs

Body* GroundBody;
Body** BodyList;			       // array of pointers to connection objects (line joints or ends)
Rod** RodList;                   // array of pointers to Rod objects
Connection** ConnectionList;  // array of pointers to connection objects (line joints or ends)
Line** LineList;                // array of pointers to line objects

//vector< Line > LineList;                 // line objects
//vector< Rod > RodList;                   // Rod objects
//vector< Body > BodyList;			        // connection objects (line joints or ends)
//vector< Connection > ConnectionList;      // connection objects (line joints or ends)

//vector< int > RodType2Is;     // array of indices of which of RodList are actually independent Rods
vector< int > LineStateIs;    // array of starting indices for Lines in "states" array
vector< int > ConnectStateIs; // array of starting indices for indendent Connections in "states" array
vector< int > RodStateIs;     // array of starting indices for independent Rods in "states" array
vector< int > BodyStateIs;    // array of starting indices for Bodies in "states" array

vector< int > FreeBodyIs;  					// vector of free body indices in BodyList vector
vector< int > CpldBodyIs;  					// vector of coupled/fairlead body indices in BodyList vector

vector< int > FreeRodIs;  					// vector of free rod indices in RodList vector (this includes pinned rods because they are partially free and have states)
vector< int > CpldRodIs;  					// vector of coupled/fairlead rod indices in RodList vector

vector< int > FreeConIs;  					// vector of free connection indices in ConnectionList vector
vector< int > CpldConIs;  					// vector of coupled/fairlead connection indices in ConnectionList vector

//vector< int > FairIs;  					// vector of fairlead connection indices in ConnectionList vector
//vector< int > ConnIs;  					// vector of connect connection indices in ConnectionList vector
//vector< int > AnchIs;  					// vector of anchor connection indices in ConnectionList vector

int nLineTypes       ; // number of line types
int nRodTypes        ; // number of Rod types
int nLines           ; // number of Line objects
int nRods            ; // number of Rod objects
int nBodys           ; // number of Body objects
int nConnections     ; // total number of Connection objects
int nConnectionsExtra; // maximum number of Connection objects (allows addition of connections during simulation for line detachments)
int nFails           ; // number of failure conditions read in


EnvCond env; 							// struct of general environmental parameters
Waves *waves;                           // pointer to a Waves object that will be created to hold water kinematics info

int npW;  // number of points that wave kinematics are input at if using WaveKin=1
double* U_1;   // array of wave velocity and acceleration at each of the npW points (u1, v1, w1, u2, v2, w2, etc.)
double* Ud_1;
double tW_1;  // time corresponding to the wave kinematics data
double* U_2;
double* Ud_2;
double tW_2;
double* U_extrap;  // for extrapolated wave velocities

string MDbasepath;                             // directory of files 
string MDbasename;                            // name of input file (without extension)

//vector< shared_ptr< ifstream > > infiles; 	//
vector< shared_ptr< ofstream > > outfiles; 	// a vector to hold ofstreams for each line
ofstream outfileMain;					// main output file
ofstream outfileLog;					// log output file
vector< OutChanProps > outChans;		// list of structs describing selected output channels for main out file
const char* UnitList[] = {"(s)     ", "(m)     ", "(m)     ", "(m)     ", 
                          "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
					 "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
					 "(N)     ", "(N)     "};   // list of units for each of the QTypes (see misc.h)


// state vector and stuff
// state vector is organized in order of input file:
//    [ bodies, rods, connects, lines [future dynamic additions?]].
//    Remember that we need to be able to add extra connects on the end for line failures.
double* states;                      // pointer to array comprising global state vector
int nX;                                // used size of state vector array
int nXtra;                            // full size of state vector array including extra space for detaching up to all line ends, each which could get its own 6-state connect (nXtra = nX + 6*2*nLines)
double* xt;                           // more state vector things for rk2/rk4 integration 
double* f0;
double* f1;
//double* f2;
//double* f3;

//double** Ffair;	// pointer to 2-d array holding fairlead forces

//double dt; // FAST time step
double dtM0; // desired mooring line model time step   

double dtOut = 0;  // (s) desired output interval (the default zero value provides output at every call to MoorDyn)

// new temporary additions for waves
//vector< floatC > zetaCglobal;
//double dwW;


// new globals for creating output console window when needed
int hConHandle;
intptr_t lStdHandle;   //long lStdHandle;

char const* PromptPtr;  // pointer to be made to environment variable PROMPT
int OwnConsoleWindow = 0;	

#ifdef LINUX	// any differences from built-in in mingw?  what about on OSX?
// int isnan(double x) { return x != x; } 	// changed to lower case.  will this still work?  Apparently some compiler optimizations can ruin this method
#define isnan(x) std::isnan(x)     // contributed by Yi-Hsiang Yu at NREL
#endif

// master function for advancing the model and calculating state derivatives (akin to MD_CalcContStateDeriv in MoorDyn F)
void CalcStateDeriv( double X[],  double Xd[], const double t, const double dt)
{
	//--try
	//--{
		// call ground body to update all the fixed things...
		GroundBody->updateFairlead( t ); 
		
		// coupled things...
		
		// extrapolate instantaneous positions of any coupled bodies (type -1)
		for (int l=0; l<CpldBodyIs.size(); l++)  
			BodyList[CpldBodyIs[l]]->updateFairlead( t ); 	
		
		// extrapolate instantaneous positions of any coupled or fixed rods (type -1 or -2)
		for (int l=0; l<CpldRodIs.size(); l++)  
			RodList[CpldRodIs[l]]->updateFairlead( t ); 		

		// extrapolate instantaneous positions of any coupled or fixed connections (type -1)
		for (int l=0; l<CpldConIs.size(); l++)  
			ConnectionList[CpldConIs[l]]->updateFairlead( t ); 
//		// set (reduntantly?) instantaneous anchor positions
//		for (int l=0; l<nAnchs; l++)  
//			ConnectionList[AnchIs[l]]->updateFairlead( t ); 
		
		// update wave kinematics if applicable<<<<
		if (env.WaveKin==1)
		{
			// extrapolate velocities from accelerations (in future could extrapolote from most recent two points, U_1 and U_2)

			double t_delta = t - tW_1;

			for (int i=0; i<npW*3; i++)
				U_extrap[i] = U_1[i] + Ud_1[i]*t_delta;
				
			
			// distribute to the appropriate objects
			int i = 0;
		
			//for (int l=0; l<FreeBodyIs.size(); l++)  
			//	i += 3;
			//
			//for (int l=0; l<FreeRodIs.size(); l++)  
			//	i += 3*RodList[FreeRodIs[l]]->getN() + 3;
			//		
			//for (int l=0; l<FreeConIs.size(); l++)  
			//	i += 3
			//
			for (int l=0; l < nLines; l++) 	
			{	LineList[l]->setNodeWaveKin(U_extrap + 3*i, Ud_1 + 3*i);
				i += 3*LineList[l]->getN() + 3;
			}	
			
		}
		
		// independent or semi-independent things with their own states...
		
		// give Bodies latest state variables (kinematics will also be assigned to dependent connections and rods, and thus line ends)
		for (int l=0; l<FreeBodyIs.size(); l++)  
			BodyList[FreeBodyIs[l]]->setState((X + BodyStateIs[l]), t);
		
		// give independent or pinned rods' latest state variables (kinematics will also be assigned to attached line ends)
		for (int l=0; l<FreeRodIs.size(); l++)  
			RodList[FreeRodIs[l]]->setState((X + RodStateIs[l]), t);
		
		// give Connects (independent connections) latest state variable values (kinematics will also be assigned to attached line ends)
		for (int l=0; l<FreeConIs.size(); l++)  
			ConnectionList[FreeConIs[l]]->setState((X + ConnectStateIs[l]), t);
		
		// give Lines latest state variable values for internal nodes
		for (int l=0; l < nLines; l++) 	
			LineList[l]->setState((X + LineStateIs[l]), t);
      

		// calculate dynamics of free objects (will also calculate forces (doRHS()) from any child/dependent objects)...
		      
      // calculate line dynamics (and calculate line forces and masses attributed to connections)
		for (int l=0; l < nLines; l++) 	
			LineList[l]->getStateDeriv((Xd + LineStateIs[l]), dt);
		
		// calculate connect dynamics (including contributions from attached lines
		// as well as hydrodynamic forces etc. on connect object itself if applicable)
		for (int l=0; l<FreeConIs.size(); l++)  
			ConnectionList[FreeConIs[l]]->getStateDeriv((Xd + ConnectStateIs[l]));
			
		// calculate dynamics of independent Rods 
		for (int l=0; l<FreeRodIs.size(); l++)  
			RodList[FreeRodIs[l]]->getStateDeriv((Xd + RodStateIs[l]));
		
		// calculate dynamics of Bodies
		for (int l=0; l<FreeBodyIs.size(); l++)  
			BodyList[FreeBodyIs[l]]->getStateDeriv((Xd + BodyStateIs[l]));
		
		//// calculate forces on fairleads
		//for (int l=0; l<nFairs; l++)  
		//	ConnectionList[FairIs[l]]->doRHS();
		//
		//// calculate forces on anchors
		//for (int l=0; l<nAnchs; l++)  
		//	ConnectionList[AnchIs[l]]->doRHS();
	
		
		// get dynamics/forces (doRHS()) of coupled objects, which weren't addressed in above calls
		
		for (int l=0; l<CpldConIs.size(); l++)  
			ConnectionList[CpldConIs[l]]->doRHS();
		
		for (int l=0; l<CpldRodIs.size(); l++)  
			RodList[CpldRodIs[l]]->doRHS(); 		
 
		for (int l=0; l<CpldBodyIs.size(); l++)  
			BodyList[CpldBodyIs[l]]->doRHS(); 	
		
		// call ground body to update all the fixed things
		//GroundBody->doRHS(); 
		GroundBody->setDependentStates();  // (not likely needed) <<<
		
	//--}
	//--catch(string e) 
	//--{
	//--	cout << "Error found! " << e << endl;
	//--	throw string("problem!");
	//--}
	
	return;
}


// Runge-Kutta 2 integration routine  (integrates states and time)
void rk2 (double x0[], double *t0, const double dt )
{
	if (env.writeLog > 2)  outfileLog << "\n----- RK2 predictor call to CalcStateDeriv at time " << *t0 << " s -----\n";
	
	CalcStateDeriv(x0, f0, *t0, dt);                             // get derivatives at t0.      f0 = f ( t0, x0 );

	for (int i=0; i<nX; i++) 
		xt[i] = x0[i] + 0.5*dt*f0[i];  						// integrate to t0  + dt/2.        x1 = x0 + dt*f0/2.0;
	
	
	if (env.writeLog > 2)  outfileLog << "\n----- RK2 predictor call to CalcStateDeriv at time " << *t0 + 0.5*dt << " s -----\n";
	
	CalcStateDeriv(xt, f1, *t0 + 0.5*dt, dt);                    // get derivatives at t0  + dt/2.	f1 = f ( t1, x1 );

	for (int i=0; i<nX; i++) 
		x0[i] = x0[i] + dt*f1[i]; 							// integrate states to t0 + dt
		
	*t0 = *t0 + dt;										// update time
		
	// <<<<<<<<< maybe should check/force all rod unit vectors to be unit vectors here? <<<
		
	return;
}


double GetOutput(OutChanProps outChan)
{
	if (outChan.OType == 1)   // line type
		return LineList[outChan.ObjID-1]->GetLineOutput(outChan);
	else if (outChan.OType == 2)   // connection type
		return ConnectionList[outChan.ObjID-1]->GetConnectionOutput(outChan);
	else if (outChan.OType == 3)   // Rod type
		return RodList[outChan.ObjID-1]->GetRodOutput(outChan);
	else
	{
		stringstream s;
		s << "Error: output type of " << outChan.Name << " does not match a supported object type ";
		throw string(s.str());
	}
}

// write all the output files for the current timestep
int AllOutput(double t, double dtC)
{
	// if using a certain output time step, check whether we should output
	
	if (dtOut > 0)
		if (t < (floor((t-dtC)/dtOut) + 1.0)*dtOut)  // if output should occur over the course of this time step, then do it!
			return 0;
	
	// What the above does is say if ((dtOut==0) || (t >= (floor((t-dtC)/dtOut) + 1.0)*dtOut)), do the below.
	// This way we avoid the risk of division by zero.
	
	// write to master output file
	if (outfileMain.is_open())
	{
		outfileMain << t << "\t "; 		// output time
	
	
		// output all LINE fairlead (top end) tensions
		//for (int l=0; l<nLines; l++) outfileMain << 0.001*(LineList[l]->getNodeTen(LineList[l]->getN())) << "\t ";
			

		for (int lf=0; lf<outChans.size(); lf++)   
		{
			//cout << "Getting output: OType:" << outChans[lf].OType << ", ObjID:" << outChans[lf].ObjID << ", QType:" <<outChans[lf].QType << endl;
			outfileMain << GetOutput(outChans[lf]) << "\t ";		// output each channel's value
		}
			
		outfileMain << "\n";
	}
	else 
	{	cout << "Unable to write to main output file " << endl;
		return -1;
	}
		
	// write individual line output files
	for (int l=0; l < nLines; l++)  LineList[l]->Output(t); 
	
	// write individual rod output files
	for (int l=0; l < nRods; l++)  RodList[l]->Output(t); 
	
	// write individual body output files
	for (int l=0; l < nBodys; l++)  BodyList[l]->Output(t); 
	
	return 0;
}


// tentative function to simulate a mooring system failure point
// assumes new information is: Rod/connection location, line(s) to detach (together if multiple)
int detachLines( int attachID, int isRod, int lineIDs[], int lineTops[], int nLinesToDetach, double time)
{
	// parameters
	// int attachID - ID of connection or Rod the lines are attached to (index is -1 this value)
	// int isRod - 1 Rod end A, 2 Rod end B, 0 if connection
	// int lineIDs[] - array of one or more lines to detach (starting from 1...)
	// XXX int lineTops[] - an array that will be FILLED IN to return which end of each line was disconnected ... 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	// int nLinesToDetach - how many lines to dettach
		
	
	// create new massless connection for detached end(s) of line(s)
				
	double M = 0.0;
	double V = 0.0;
	double CdA;
	double Ca;
	double r0[3] = {0.0};
	double F[3] = {0.0};
	
	for (int I=0; I<3; I++)
		r0[I] = 0.0;   // <<<<<<< will be updated after setup

	CdA  = 0.0;
	Ca   = 0.0;
						
	int type = 0;                                
	
	// add connect to list of free ones and add states for it
	nConnections++;  // add 1 to the number of connections (this is now the number of the new connection, and nConnections-1 is its index)
	FreeConIs.push_back(nConnections-1);   
	ConnectStateIs.push_back(nX);   // assign start index of this connect's states
	nX += 6;                             // add 6 state variables for each connect
					

	// check to make sure we haven't gone beyond the extra size allotted to the state arrays or the connections list <<<< really should throw an error here
	if (nConnections > nConnectionsExtra)
		cout << "Error: nConnections > nConnectionsExtra" << endl;
	if (nX > nXtra)
		cout << "Error: nX > nXtra" << endl;
	
	// now make Connection object!
	ConnectionList[nConnections-1] = new Connection();
	ConnectionList[nConnections-1]->setup(nConnections, type, r0, M, V, F, CdA, Ca);
	ConnectionList[nConnections-1]->setEnv( &env, waves);
	

	double dummyConnectState[6];  // dummy state array to hold kinematics of old attachment point (format in terms of part of connection state vector: r[J]  = X[3 + J]; rd[J] = X[J]; )

	// detach lines from old Rod or Connection, and get kinematics of the old attachment point
	for (int l=0; l<nLinesToDetach; l++)
	{
		if (isRod==1)
			RodList[attachID-1]->removeLineFromRodEndA(lineIDs[l], lineTops+l, dummyConnectState+3, dummyConnectState);
		else if (isRod==2)
			RodList[attachID-1]->removeLineFromRodEndB(lineIDs[l], lineTops+l, dummyConnectState+3, dummyConnectState);
		else if (isRod==0)
			ConnectionList[attachID-1]->removeLineFromConnect(lineIDs[l], lineTops+l, dummyConnectState+3, dummyConnectState);
		else
			cout << "ERRoR: Failure doesn't have a valid isRod value of 0, 1, or 2." << endl;
	}
	
	
	// attach lines to new connection
	for (int l=0; l<nLinesToDetach; l++)  // for each relevant line
		ConnectionList[nConnections-1]->addLineToConnect(LineList[lineIDs[l]-1], lineTops[l]);
	
	// update connection kinematics to match old line attachment point kinematics and set positions of attached line ends
	ConnectionList[nConnections-1]->setState(dummyConnectState, time);
	
	// now make the state vector up to date!
	for (int J=0; J<6; J++)
		states[ConnectStateIs.back()+J] = dummyConnectState[J];
		
	return 0;
}


// read in stiffness/damping coefficient or load nonlinear data file if applicable
int getCoefficientOrCurve(const char entry[50], double *LineProp_c, int *LineProp_npoints, double *LineProp_Xs, double *LineProp_Ys)
{
	if (strpbrk(entry, "abcdfghijklmnopqrstuvwxyzABCDFGHIJKLMNOPQRSTUVWXYZ") == NULL) // "eE" are exluded as they're used for scientific notation!
	{
		//if (wordy > 0) cout << "found NO letter in the line coefficient value so treating it as a number." << endl;
		*LineProp_c = atof(entry);
		*LineProp_npoints = 0;
	}	
	else // otherwise interpet the input as a file name to load stress-strain lookup data from
	{
		if (wordy > 0) cout << "found A letter in the line coefficient value so will try to load the filename." << endl;
		*LineProp_c = 0.0;
		
		// load lookup table data from file
		vector<string> Clines;
		string Cline;
		
		stringstream iname;
		iname << MDbasepath << entry;
		
		ifstream myfile (iname.str());
		if (myfile.is_open())
		{
			while ( myfile.good() )
			{
				getline (myfile,Cline);
				Clines.push_back(Cline);
			}
			myfile.close();
		}
		else 
		{	cout << "Error: unable to open " << iname.str() << endl; 
			cout << "                    - " << MDbasepath << " - "<< entry << " - "<< iname.str() << endl; 
			return -1;  // <<<<<<<<<<<< need to make this a failure!!
		}
		
		// now process data	
		int nC = 0; // counter for number of data points in lookup table
		for (int I=2; I<Clines.size(); I++)   // skip first three lines (title, names, and units) then parse
		{
			vector<string> Centries = split(Clines[I]);
			if (Centries.size() >= 2) // if valid number of inputs
			{
				LineProp_Xs[nC]  = atof(Centries[0].c_str());
				LineProp_Ys[nC]  = atof(Centries[1].c_str());
				if (wordy > 0) cout << LineProp_Xs[nC] << ", " << LineProp_Ys[nC] << endl;
				nC++;
				if (nC >= nCoef)   // ensure number of entries doesn't exceed limit <<<< make proper
				{
					cout << "Error: too many coefficients for lookup table" << endl;
					return -1;
				}
			}
			else
			{	cout << "Error: failed to find two columns somewhere within " << iname.str() << endl; 
				return -1;
			}
		}		
		*LineProp_npoints = nC;
	}
	return 1;
}


/** initialization function, changed to work for any type of coupling

Passed parameters are positions, velocities (assumed 0), and root filename (used for input and output too).
*/

int MoorDynInit(double x[], double xd[], const char *infilename)
{	

#ifndef LINUX
#ifndef OSX
	// ------------ create console window for messages if none already available -----------------
	// adapted from Andrew S. Tucker, "Adding Console I/O to a Win32 GUI App" in Windows Developer Journal, December 1997. source code at http://dslweb.nwnexus.com/~ast/dload/guicon.htm

	//static const WORD MAX_CONSOLE_LINES = 500;  // maximum mumber of lines the output console should have
	//CONSOLE_SCREEN_BUFFER_INFO coninfo;
	FILE *fp;

	PromptPtr = getenv("PROMPT");		 // get pointer to environment variable "PROMPT" (NULL if not in console)
	
	//TODO: simplify this to just keep the output parts I need

	HWND consoleWnd = GetConsoleWindow();
//   DWORD dwProcessId;
//   GetWindowThreadProcessId(consoleWnd, &dwProcessId);
//   if (GetCurrentProcessId()==dwProcessId)
//   {
//       cout << "I have my own console, press enter to exit" << endl;
//       cin.get();
//	   FreeConsole();
//   }
//   else
//   {
//       cout << "This Console is not mine, good bye" << endl;   
//   }
		
	if (consoleWnd == NULL)  // if not in console, create our own
	{
		OwnConsoleWindow = 1; // set flag
		
		// allocate a console for this app
		AllocConsole();

		// set the screen buffer to be big enough to let us scroll text
	    static const WORD MAX_CONSOLE_LINES = 500;  // maximum mumber of lines the output console should have
	    CONSOLE_SCREEN_BUFFER_INFO coninfo;
		GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
		coninfo.dwSize.Y = MAX_CONSOLE_LINES;
		SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

		// redirect unbuffered STDOUT to the console
		//lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
		lStdHandle = (intptr_t)GetStdHandle(STD_OUTPUT_HANDLE);
		hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
		fp = _fdopen( hConHandle, "w" );
		*stdout = *fp;
		setvbuf( stdout, NULL, _IONBF, 0 );

	//	// redirect unbuffered STDIN to the console
	//	lStdHandle = (long)GetStdHandle(STD_INPUT_HANDLE);
	//	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	//	fp = _fdopen( hConHandle, "r" );
	//	*stdin = *fp;
	//	setvbuf( stdin, NULL, _IONBF, 0 );

	//	// redirect unbuffered STDERR to the console
	//	lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
	//	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	//	fp = _fdopen( hConHandle, "w" );
	//	*stderr = *fp;
	//	setvbuf( stderr, NULL, _IONBF, 0 );

		// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
		// point to console as well
		ios::sync_with_stdio();
		
		cout << "(MoorDyn-initiated console window)" << endl;
	}
#endif
#endif	
	
//--	try
//--	{
		
		// ---------------------------- MoorDyn title message ----------------------------
		cout << "\n Running MoorDyn (v2.a11, 2022-01-03)" << endl;
		cout << "   NOTE: This is an alpha version of MoorDyn v2, intended for testing and debugging." << endl;
		cout << "         MoorDyn v2 has significant ongoing input file changes from v1." << endl;  
		cout << "   Copyright: (C) 2021 National Renewable Energy Laboratory, (C) 2014-2019 Matt Hall" << endl;
		cout << "   This program is released under the GNU General Public License v3." << endl;

		//dt = *dTime; // store time step from FAST	
		

		// calculate TransMat
	//	double TransMat[9];
	//	RotMat(X[3], X[4], X[5], TransMat); <<<<<<<<<<<<<<<
			
		
		
		// ----------------------- initialize data holders -------------------------
		
		// defaults
		env.g = 9.8;
		env.WtrDpth = 0.;
		env.rho_w = 1025.;
		env.kb = 3.0e6;
		env.cb = 3.0e5;
		env.WaveKin = 0;    // 0=none
		env.Current = 0;   // 0=none
		env.dtWave = 0.25;
		env.WriteUnits = 1;	// by default, write units line
		env.writeLog = 0;   // by default, don't write out a log file
		env.FrictionCoefficient = 0.0;
		env.FricDamp = 200.0;
		env.StatDynFricScale = 1.0;
			
		double ICDfac = 5; // factor by which to boost drag coefficients during dynamic relaxation IC generation
		double ICdt = 1.0;						// convergence analysis time step for IC generation
		double ICTmax = 120;						// max time for IC generation
		double ICthresh = 0.001;					// threshold for relative change in tensions to call it converged
		int WaveKinTemp = 0;				// temporary wave kinematics flag used to store input value while keeping env.WaveKin=0 for IC gen
		npW = 0;   // assume no wave kinematics points are passed in externally, unless ExernalWaveKinInit is called later
		
		dtM0 = 0.001;  // default value for desired mooring model time step

		// fairlead position arrays
		vector< vector< double > > rFairRod;
		vector< vector< double > > rFairCon;
		vector< string > outchannels;  // string containing which channels to write to output
		
		// line connection info (temporary, until LineList addresses are done moving) <<????
	//	vector< int > LineInd;
	//	vector< int > RodInd;
	//	vector< int > AnchInd;
	//	vector< int > FairInd;

		// make a "ground body" that will be the parent of all fixed objects (connections and rods)
		GroundBody = new Body(); 
		GroundBody->setup(0, 1, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);  

		
		nX = 0; // Make sure the state vector counter starts at zero.
				// This will be incremented as each object is added.

		// make sure the object counteres start at zero (done here in case MoorDyn is called multiple times)
		
		nLineTypes        = 0; // number of line types
		nRodTypes         = 0; // number of Rod types
		nLines            = 0; // number of Line objects
		nRods             = 0; // number of Rod objects
		nBodys            = 0; // number of Body objects
		nConnections      = 0; // total number of Connection objects
		nConnectionsExtra = 0; // maximum number of Connection objects (allows addition of connections during simulation for line detachments)
		nFails            = 0; // number of failure conditions read in

		
		// ==================== load data about the mooring lines from lines.txt =====================
		
		char filename[50];
		
		if (strlen(infilename)==0)
			snprintf(filename, sizeof(filename), "%s", "Mooring/lines.txt");
		else		
			strncpy(filename, infilename, sizeof(filename));

		string sfilename = string(filename, strlen(filename));
		
		
			
		//MDbasename = filesystem::path(filename).stem().string();   // get just the file name without extension
		//MDbasepath = filesystem::path(filename).parent_path().string();   // get the directory of the files
		
		int lastSlash = sfilename.find_last_of("/\\");
		int lastDot = sfilename.find_last_of('.');
		
		
		cout << "Based on the provided infilename of " << infilename << endl;
		cout << "The filename is " << filename << " or " << sfilename << "  " << lastSlash << " " << lastDot << endl;
		
		MDbasename = sfilename.substr(lastSlash+1, lastDot-lastSlash-1);
		MDbasepath = sfilename.substr(0, lastSlash+1);  // the path to the folder where the files are located, including the last slash

		
		cout << "The MDbasename is " << MDbasename << endl;
		cout << "The MDbasepath is " << MDbasepath << endl;
		
		
		// --------------------------------- read data from file -----------------------------
		vector<string> lines;
		string line;
		ifstream myfile (filename);     // open an input stream to the line data input file
	//	ifstream myfile ("Mooring/lines.txt");     // open an input stream to the line data input file
		if (myfile.is_open())
		{
			while ( myfile.good() )
			{
				getline (myfile,line);
				lines.push_back(line);
			}
			myfile.close();
		}
		else 
		{	cout << "Error: unable to open file " << filename << endl; 
			return -1;
		}
		
		
		// ----------------- go through file contents a first time, counting each entry -----------------------
		
		int i=0; // file line number
		
		while (i < lines.size())  
		{
			if (lines[i].find("---") != string::npos) // look for header line
			{
				if ( (lines[i].find("LINE DICTIONARY") != string::npos) || (lines[i].find("LINE TYPES") != string::npos) ) // if line dictionary header
				{	
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nLineTypes++;
						i++;
					}
				}
				else if ( (lines[i].find("ROD DICTIONARY") != string::npos) || (lines[i].find("ROD TYPES") != string::npos) ) // if rod dictionary header
				{	
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nRodTypes++;
						i++;
					}	
				}
				else if ((lines[i].find("BODIES") != string::npos) || (lines[i].find("BODY LIST") != string::npos)  || (lines[i].find("BODY PROPERTIES") != string::npos))
				{	
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nBodys++;
						i++;
					}		
				}
				else if ((lines[i].find("RODS") != string::npos) || (lines[i].find("ROD LIST") != string::npos) || (lines[i].find("ROD PROPERTIES") != string::npos)) // if rod properties header
				{	
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nRods++;
						i++;
					}	
				}
				else if ((lines[i].find("POINTS") != string::npos) || (lines[i].find("POINT LIST") != string::npos) || (lines[i].find("CONNECTION PROPERTIES") != string::npos) || (lines[i].find("NODE PROPERTIES") != string::npos) ) // if node properties header
				{	
					if (nLineTypes < 1)
						cout << "   Error: began reading connection inputs before reading any line type inputs." << endl;
				
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nConnections++;
						i++;
					}
				}
				else if ((lines[i].find("LINES") != string::npos) || (lines[i].find("LINE LIST") != string::npos) || (lines[i].find("LINE PROPERTIES") != string::npos)) // if line properties header
				{	
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nLines++;
						i++;
					}
				}
				else if (lines[i].find("FAILURE") != string::npos) // if failure conditions header
				{	
					if (wordy>0) cout << "   Reading failure conditions: ";
					
					i += 3; // skip following two lines (label line and unit line)
					
					// find how many elements of this type there are
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						nFails++;
						i++;
					}
				}
				else if (lines[i].find("OPTIONS") != string::npos) // if options header
				{	
					i ++;
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						vector<string> entries = split(lines[i]);
						
						if (entries.size() >= 2) // if a valid "[value] [identifier] [optional comment]" format
						{
							if (entries[1] == "writeLog") 
							{	env.writeLog = atoi(entries[0].c_str());
								if (env.writeLog > 0)
								{	// open log file for writing if needed
									stringstream oname;
									oname << MDbasepath << MDbasename << ".log";
									
									outfileLog.open(oname.str());
									if (outfileLog.is_open())
									{
										outfileLog << "MoorDyn v2 log file with output level " << env.writeLog << "\n";
										outfileLog << "Note: options above the writeLog line in the input file will not be recorded\n";
										
										env.outfileLogPtr = & outfileLog; // get pointer to outfile for MD objects to use
										
										*env.outfileLogPtr << "will this work!\n";
									}
									else 
									{	cout << "   ERROR: Unable to write to log file " << oname.str() << endl;  //TODO: handle error <<<<<<<<<<<<<<<<<<<<
										return -1;
									}
								}
							}
							else if ((entries[1] == "dtM")           || (entries[1] == "DT"))        dtM0 = atof(entries[0].c_str());     // second is old way, should phase out
							else if ((entries[1] == "g") || (entries[1] == "gravity"))          env.g  = atof(entries[0].c_str()); 
							else if ((entries[1] =="Rho")||(entries[1]=="rho")||(entries[1]=="WtrDnsty"))   env.rho_w = atof(entries[0].c_str()); 
							else if (entries[1] == "WtrDpth")                                   env.WtrDpth = atof(entries[0].c_str()); 
							else if ((entries[1] == "kBot")     || (entries[1] == "kb"))        env.kb = atof(entries[0].c_str());   // "
							else if ((entries[1] == "cBot")     || (entries[1] == "cb"))        env.cb = atof(entries[0].c_str());   // "
							else if ((entries[1] == "dtIC")     || (entries[1] == "ICdt"))      ICdt     = atof(entries[0].c_str()); // "
							else if ((entries[1] == "TmaxIC")   || (entries[1] == "ICTmax"))    ICTmax   = atof(entries[0].c_str()); // "
							else if ((entries[1] == "CdScaleIC")|| (entries[1] == "ICDfac"))    ICDfac   = atof(entries[0].c_str()); // "
							else if ((entries[1] == "threshIC") || (entries[1] == "ICthresh"))  ICthresh = atof(entries[0].c_str()); // "
							else if (entries[1] == "WaveKin")                                   WaveKinTemp = atoi(entries[0].c_str());
							else if (entries[1] == "Currents")                                  env.Current = atoi(entries[0].c_str());
							else if (entries[1] == "WriteUnits")                                env.WriteUnits = atoi(entries[0].c_str());
							else if (entries[1] == "FrictionCoefficient")                       env.FrictionCoefficient = atof(entries[0].c_str());
							else if (entries[1] == "FricDamp")                       env.FricDamp = atof(entries[0].c_str());
							else if (entries[1] == "StatDynFricScale")             env.StatDynFricScale = atof(entries[0].c_str());
							else if (entries[1] == "dtOut")                                     dtOut = atof(entries[0].c_str()); // output writing period (0 for at every call)
							// >>>>>>>>>> add dtWave...
							else cout << "Warning: solver option keyword \"" << entries[1] << "\" not recognized." << endl;
						}
						i++;
					
					}
				}
				else
					i++;
			}
			else i++;
		}	
		
		nConnectionsExtra = nConnections + 2*nLines;    // set maximum number of connections, accounting for possible detachment of each line end and a connection for that
		
		
		// ------------------------- allocate necessary object arrays -------------------------------
		
		// allocate space for array of pointers to entries
		RodPropList    = new RodProps*[nRodTypes];			
		LinePropList   = new LineProps*[nLineTypes];		
		BodyList       = new Body*[nBodys];
		RodList        = new Rod*[nRods];		
		ConnectionList = new Connection*[nConnectionsExtra]; // using nConnectionsExtra to leave room for additional connections for line detachments
		LineList       = new Line*[nLines];
		FailList       = new FailProps*[nFails];
		
		
		// ---------------------- now go through again and process file contents --------------------
		
		i=0; // reset file line number
		
		while (i < lines.size())  
		{
			if (lines[i].find("---") != string::npos) // look for header line
			{
				
				// might want to convert to uppercase to support lowercase headings too <<<
				
				if ( (lines[i].find("LINE DICTIONARY") != string::npos) || (lines[i].find("LINE TYPES") != string::npos) ) // if line dictionary header
				{	
					if (wordy>0) cout << "   Reading line types: ";
					
					i += 3; // skip following two lines (label line and unit line)
					
					// set up each entry
					for (int iLineType=0; iLineType<nLineTypes; iLineType++)
					{
						// parse out entries: Name  Diam MassDenInAir EA cIntDamp EI    Cd  Ca  CdAx  CaAx 
						vector<string> entries = split(lines[i+iLineType]); // split by spaces
						
						if (entries.size() >= 10) // if valid number of inputs
						{
							LinePropList[iLineType] = new LineProps();
							
							LinePropList[iLineType]->type =    entries[0]; //.c_str());
							LinePropList[iLineType]->d  = atof(entries[1].c_str());
							LinePropList[iLineType]->w  = atof(entries[2].c_str());
							
							LinePropList[iLineType]->Cdn= atof(entries[6].c_str());
							LinePropList[iLineType]->Can= atof(entries[7].c_str());
							LinePropList[iLineType]->Cdt= atof(entries[8].c_str());
							LinePropList[iLineType]->Cat= atof(entries[9].c_str());
							
							// read in stiffness value (and load nonlinear file if needed)
							getCoefficientOrCurve(entries[3].c_str(), &(LinePropList[iLineType]->EA), 
								&(LinePropList[iLineType]->nEApoints), LinePropList[iLineType]->stiffXs, LinePropList[iLineType]->stiffYs);
								
							// read in damping value (and load nonlinear file if needed)
							getCoefficientOrCurve(entries[4].c_str(), &(LinePropList[iLineType]->c), 
								&(LinePropList[iLineType]->nCpoints), LinePropList[iLineType]->dampXs, LinePropList[iLineType]->dampYs);
								
								
							// read in bending stiffness value (and load nonlinear file if needed)
							getCoefficientOrCurve(entries[5].c_str(), &(LinePropList[iLineType]->EI), 
								&(LinePropList[iLineType]->nEIpoints), LinePropList[iLineType]->bstiffXs, LinePropList[iLineType]->bstiffYs);

							
							if (wordy>0)  cout << entries[0] << " ";
							
							
							// write lineType information to log file
							if (env.writeLog > 1)
							{
								outfileLog << "  - LineType" << iLineType+1 << ":" << endl;
								outfileLog << "    name: " << LinePropList[iLineType]->type << endl;
								outfileLog << "    d   : " << LinePropList[iLineType]->d    << endl;
								outfileLog << "    w   : " << LinePropList[iLineType]->w    << endl;
								outfileLog << "    Cdn : " << LinePropList[iLineType]->Cdn  << endl;
								outfileLog << "    Can : " << LinePropList[iLineType]->Can  << endl;
								outfileLog << "    Cdt : " << LinePropList[iLineType]->Cdt  << endl;
								outfileLog << "    Cat : " << LinePropList[iLineType]->Cat  << endl;
							}
							
						}
						else
						{
							//cout << "Error: make sure the Line Types entries have 10 columns (including bending stiffness)..." << endl;  // <<<< make this proper
							//cout << "Occured when reading line: " << lines[i+iLineType] << endl;
							
							stringstream s;
							s << " Error when reading LineType input: " << lines[i+iLineType] <<"\n";
							s << " Make sure the Line Types entries have 10 columns (including bending stiffness)\n";
							throw string(s.str());
						}
						//i++;
					}
					if (wordy>0) cout << "\n";
				}
				else if ( (lines[i].find("ROD DICTIONARY") != string::npos) || (lines[i].find("ROD TYPES") != string::npos) ) // if rod dictionary header
				{	
					if (wordy>0) cout << "   Reading rod types: ";
					
					i += 3; // skip following two lines (label line and unit line)
					
					
					// set up each entry
					for (int iRodType=0; iRodType<nRodTypes; iRodType++)
					{
						// parse out entries: Name  Diam MassDen Cd  Ca  CdEnd  CaEnd
						vector<string> entries = split(lines[i+iRodType]); // split by spaces
												
						if (entries.size() >= 7) // if valid number of inputs
						{	
							RodPropList[iRodType] = new RodProps();

							RodPropList[iRodType]->type =    entries[0]; //.c_str());
							RodPropList[iRodType]->d  = atof(entries[1].c_str());
							RodPropList[iRodType]->w  = atof(entries[2].c_str());
							RodPropList[iRodType]->Cdn= atof(entries[3].c_str());
							RodPropList[iRodType]->Can= atof(entries[4].c_str());
							RodPropList[iRodType]->Cdt= atof(entries[5].c_str());
							RodPropList[iRodType]->Cat= atof(entries[6].c_str());
							if (wordy>0)  cout << entries[0] << " ";
						}
						//i++;
					}
					if (wordy>0) cout << "\n";
				}	
				else if ((lines[i].find("BODIES") != string::npos) || (lines[i].find("BODY LIST") != string::npos)  || (lines[i].find("BODY PROPERTIES") != string::npos))
				{	
					if (wordy>0) cout << "   Reading Body properties: ";
					i += 3; // skip following two lines (label line and unit line)
					
					
					// set up each entry
					for (int iBody=0; iBody<nBodys; iBody++)
					{ 	
						// parse out entries: ID   Attachment  X0  Y0  Z0  r0  p0  y0    M  CG*  I*    V  CdA*  Ca*
						vector<string> entries = split(lines[i+iBody]); // split by spaces
						
						if (entries.size() >= 14) // if valid number of inputs
						{	
							
							int number          = atoi(entries[0].c_str());
							int type;							
							double r6[6];
							
							for (int I=0; I<6; I++) 
								r6[  I] = atof(entries[2+I].c_str());					
							
							double M = atof(entries[ 8].c_str());						
							double V = atof(entries[11].c_str());
							
							double rCG[3];
							double Inert[3];
							double CdA[3];
							double Ca[3];								
							
							// process CG
							vector<string> strings_rCG = splitBar(entries[ 9].c_str()); // split by braces, if any
							if (strings_rCG.size() == 1) {                                // if only one entry, it is the z coordinate
								rCG[0] = 0.0;
								rCG[1] = 0.0;
								rCG[2] = atof(strings_rCG[0].c_str());
							}
							else if (strings_rCG.size() == 3) {                           // all three coordinates provided
								rCG[0] = atof(strings_rCG[0].c_str());
								rCG[1] = atof(strings_rCG[1].c_str());
								rCG[2] = atof(strings_rCG[2].c_str());
							}
							else {
								cout << 'Body ' << number << ' CG entry (col 10) must have 1 or 3 numbers.' << endl;
								return -1;
							}
							// process mements of inertia
							vector<string> strings_I   = splitBar(entries[10].c_str());
							if (strings_I.size() == 1) {                                // if only one entry, use it for all directions
								Inert[0] = atof(strings_I[0].c_str());
								Inert[1] = Inert[0];
								Inert[2] = Inert[0];
							}
							else if (strings_I.size() == 3) {                           // all three coordinates provided
								Inert[0] = atof(strings_I[0].c_str());
								Inert[1] = atof(strings_I[1].c_str());
								Inert[2] = atof(strings_I[2].c_str());
							}
							else {
								cout << 'Body ' << number << ' inertia entry (col 11) must have 1 or 3 numbers.' << endl;
								return -1;
							}
							// process drag ceofficient by area product
							vector<string> strings_CdA = splitBar(entries[12].c_str());
							if (strings_CdA.size() == 1) {                                // if only one entry, use it for all directions
								CdA[0] = atof(strings_CdA[0].c_str());
								CdA[1] = CdA[0];
								CdA[2] = CdA[0];
							}
							else if (strings_CdA.size() == 3) {                           // all three coordinates provided
								CdA[0] = atof(strings_CdA[0].c_str());
								CdA[1] = atof(strings_CdA[1].c_str());
								CdA[2] = atof(strings_CdA[2].c_str());
							}
							else {
								cout << 'Body ' << number << ' CdA entry (col 13) must have 1 or 3 numbers.' << endl;
								return -1;
							}
							// process added mass coefficient
							vector<string> strings_Ca  = splitBar(entries[13].c_str());							
							if (strings_Ca.size() == 1) {                                // if only one entry, use it for all directions
								Ca[0] = atof(strings_Ca[0].c_str());
								Ca[1] = Ca[0];
								Ca[2] = Ca[0];
							}
							else if (strings_Ca.size() == 3) {                           // all three coordinates provided
								Ca[0] = atof(strings_Ca[0].c_str());
								Ca[1] = atof(strings_Ca[1].c_str());
								Ca[2] = atof(strings_Ca[2].c_str());
							}
							else {
								cout << 'Body ' << number << ' Ca entry (col 14) must have 1 or 3 numbers.' << endl;
								return -1;
							}
							
							
							
							// ----------- process body type -----------------

							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							char typeWord[10];							// the buffer
							
							snprintf(typeWord, 10, entries[1].c_str());		// copy body type word to buffer
							
							decomposeString(typeWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
							
							
							if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0))
							{
								type = 1;  // body is fixed  (this would just be used if someone wanted to temporarly fix a body that things were attached to)
							}
							if ((strcmp(let1, "COUPLED") ==0) || (strcmp(let1, "VESSEL") ==0) || (strcmp(let1, "VES") ==0) || (strcmp(let1, "CPLD") ==0))
							{
								type = -1;   // body is coupled - controlled from outside
								CpldBodyIs.push_back(iBody);									
							}
							else 
							{
								type = 0;      // body is free								
								FreeBodyIs.push_back(iBody);
							}							
							// make an output file for it
							//if (...)
							//{	
								stringstream oname;
								oname << MDbasepath << MDbasename << "_Body" << number << ".out";
								outfiles.push_back( make_shared<ofstream>(oname.str()));
							//}
							//else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
							
							
							// set up Body 
							BodyList[iBody] = new Body(); 
							BodyList[iBody]->setup(number, type, r6, rCG, M, V, Inert, CdA, Ca, outfiles.back());		
								
							
							// set up the body state info if applicable
							if (type == 0)
							{
								BodyStateIs.push_back(nX);   // assign start index of this body's states
								nX += 12;                        // add 12 state variables for the body
							}
							
								
							//if (BodyList.size() != number)  // check that ID numbers are in order
							//	cout << "Warning: body ID numbers should be in order (1,2,3...)." << endl;
							
						}
						else 
						{
							cout << endl << "   Error with Body " << entries[0] << " inputs (14 expected but only " << entries.size() << " provided)." << endl;
							return -1;
						}
					}		
					if (wordy>0) cout << "\n";		
				}
				else if ((lines[i].find("RODS") != string::npos) || (lines[i].find("ROD LIST") != string::npos) || (lines[i].find("ROD PROPERTIES") != string::npos)) // if rod properties header
				{	
					if (wordy>0) cout << "   Reading rod properties: ";
					i += 3; // skip following two lines (label line and unit line)
										
					
					// set up each entry
					for (int iRod=0; iRod<nRods; iRod++)
					{ 	
						//vector<string> entries = split(lines[i-nRods+iRod], ' '); // split by spaces
						vector<string> entries = split(lines[i+iRod]); // split by spaces
						
						if (entries.size() >= 8) // if valid number of inputs
						{
							// RodID  Type/BodyID  RodType  Xa   Ya   Za   Xb   Yb   Zb  NumSegs  Flags/Outputs
							
							
							// read in these properties first because they're needed for setting up various rod types
								
							int number          = atoi(entries[0].c_str());
							string RodType         = entries[1];
							double endCoords[6]; 
							for (int J=0; J<6; J++) endCoords[J]=atof(entries[3+J].c_str());
							int NumSegs         = atoi(entries[9].c_str());
							string outchannels  = entries[10];
							
							
							// ----------- process rod type -----------------
							int type;						
							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							char typeWord[10];							// the buffer
							
							snprintf(typeWord, 10, entries[2].c_str());		// copy rod type word to buffer
							
							decomposeString(typeWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
								
							if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0))
							{
								type = 2;
								//GroundBody->addRodToBody(RodList[iRod], endCoords);  // note: object not made yet, but only need pointer to pass
							}
							else if ((strcmp(let1, "PINNED") ==0) || (strcmp(let1, "PIN") ==0))
							{
								type = 1;       
								//GroundBody->addRodToBody(RodList[iRod], endCoords);  // note: object not made yet, but only need pointer to pass	
								FreeRodIs.push_back(iRod);	  // add this pinned rod to the free list because it is half free
								RodStateIs.push_back(nX);   // assign start index of this rod's states
								nX += 6;                        // add 6 state variables for each pinned rod							
							}
							else if (strcmp(let1, "BODY") ==0)
							{	// attached to a body (either rididly or pinned)
								if (strlen(num1)>0)
								{
									int bodyID = atoi(num1);
									if ((bodyID <= nBodys) && (bodyID > 0))
									{
										//BodyList[bodyID-1]->addRodToBody(RodList[iRod], endCoords);  // note: object not made yet, but only need pointer to pass
										
										if ((strcmp(let2, "PINNED") ==0) || (strcmp(let2, "PIN") ==0))
										{
											type = 1;
											FreeRodIs.push_back(iRod);	  // add this pinned rod to the free list because it is half free
											RodStateIs.push_back(nX);   // assign start index of this rod's states
											nX += 6;                        // add 6 state variables for each pinned rod	
										}
										else
										{
											type = 2;
										}
									}
									else
									{	cout << "Error: Body ID out of bounds for Rod " << number << "." << endl;
										// cout << "   Error: Invalid body ID (" << bodyID << ") given for Rod " << number << endl;
										return -1;								
									}
								}
								else
								{	cout << "Error: no number provided for Rod " << number << " Body attachment." << endl;
									return -1;
								}
							}
							else if ((strcmp(let1, "VESSEL") ==0) || (strcmp(let1, "VES") ==0) || (strcmp(let1, "COUPLED") ==0) || (strcmp(let1, "CPLD") ==0))
							{	// if a rigid fairlead, add to list and add 
								type = -2;
							//	rFairRod.push_back(vector<double>(6, 0.0));		                    // fairlead location in turbine ref frame
							//	for (int J=0; J<6; J++)  rFairRod.back().at(J) = endCoords[J]; 	// 6DOF coordinates of rigid connection
								CpldRodIs.push_back(iRod);			                                    // index of fairlead in RodList vector
							}		
							else if ((strcmp(let1, "VESSELPINNED") ==0) || (strcmp(let1, "VESPIN") ==0) || (strcmp(let1, "COUPLEDPINNED") ==0) || (strcmp(let1, "CPLDPIN") ==0))
							{	// if a pinned fairlead, add to list and add 
								type = -1;
							//	rFairRod.push_back(vector<double>(3, 0.0));		                    // fairlead location in turbine ref frame
							//	for (int J=0; J<3; J++)  rFairRod.back().at(J) = endCoords[J]; 	// x,y,z coordinates of pinned connection
								
								CpldRodIs.push_back(iRod);			                                    // index of fairlead in RodList vector
								FreeRodIs.push_back(iRod);	          // also add this pinned rod to the free list because it is half free
								RodStateIs.push_back(nX);            // assign start index of this rod's states
								nX += 6;                                 // add 6 state variables for each pinned rod	
							}						
							else if ((strcmp(let1, "CONNECT") ==0) || (strcmp(let1, "CON") ==0) || (strcmp(let1, "FREE") ==0))
							{
								type = 0;
								FreeRodIs.push_back(iRod);	          // add this free rod to the free list
								RodStateIs.push_back(nX);            // assign start index of this rod's states
								nX += 12;                                // add 12 state variables for each free rod
							}
							else 
							{	// throw error if not identified
								stringstream s;
								s << "Error: unidentified Type/BodyID for rod " << number << ": " << typeWord;
								throw string(s.str());
							}
							
							
							// find Rod properties index (look in the Rod dictionary)
							int TypeNum = -1;
							for (int J=0; J<nRodTypes; J++)  {
								if (RodPropList[J]->type.find(RodType) != string::npos)
									TypeNum = J;
							}						
							
							if (TypeNum == -1)
								cout << "   Error: unable to identify type of Rod " << number << " (" << RodType << ") from those provided." << endl;
							
							if (wordy>1) cout << "rod " << number << " type " << RodType << " typenum " << TypeNum << endl;
							
							// make an output file for it
							if ((outchannels.size() > 0) && (strcspn( outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))  // if 1+ output flag chars are given and they're valid
							{	stringstream oname;
								oname << MDbasepath << MDbasename << "_Rod" << number << ".out";
								outfiles.push_back( make_shared<ofstream>(oname.str()));
							}
							else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
						
						
							// set up Rod 						
							RodList[iRod] = new Rod(); 
							RodList[iRod]->setup(number, type, RodPropList[TypeNum], endCoords,  
								NumSegs, outfiles.back(), outchannels);							
								
							
							// depending on type, assign the Rod to its respective parent body
							
							if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0)) // type = 2;
								GroundBody->addRodToBody(RodList[iRod], endCoords);  
							else if ((strcmp(let1, "PINNED") ==0) || (strcmp(let1, "PIN") ==0)) //	type = 1;       
								GroundBody->addRodToBody(RodList[iRod], endCoords);  
							else if (strcmp(let1, "BODY") ==0)  // attached to a body (either rigidly or pinned)
							{	
								int bodyID = atoi(num1);
								BodyList[bodyID-1]->addRodToBody(RodList[iRod], endCoords); 
							}
							
							
							if (wordy>0)  cout << "Rod "<< number << " is type "<<type<<"."<< endl;
							
						}					
						else 
						{
							cout << endl << "   Error with rod " << entries[0] << " inputs (not enough)." << endl;
							return -1;
						}
						//i++;
					}		
					if (wordy>0) cout << "\n";
				}
				
				
				else if ((lines[i].find("POINTS") != string::npos) || (lines[i].find("POINT LIST") != string::npos) || (lines[i].find("CONNECTION PROPERTIES") != string::npos) || (lines[i].find("NODE PROPERTIES") != string::npos) ) // if node properties header
				{	
					if (nLineTypes < 1)
						cout << "   Error: began reading connection inputs before reading any line type inputs." << endl;
				
					if (wordy>0) cout << "   Reading point list: ";
					i += 3; // skip following two lines (label line and unit line)
										
					
					// set up each entry
					for (int iConnection=0; iConnection<nConnections; iConnection++)
					{ 	
						//vector<string> entries = split(lines[i-nConnections+iConnection], ' '); // split by spaces
						vector<string> entries = split(lines[i+iConnection]); // split by spaces
						
						// Node  Type/attachID    X  Y   Z  M  V  [optional:FX FY FZ] CdA Ca
						
						if (entries.size() >= 9) // if valid number of inputs
						{					
							int number=atoi(entries[0].c_str());
							
							// <<<<<<<<<< should check whether numbering is correct
							
							
							// read these properties first, because they'll be used when processing various connection types
							
							double M = atof(entries[5].c_str());
							double V = atof(entries[6].c_str());
							double CdA;
							double Ca;
							
							double r0[3];
							double F[3] = {0.0};
							
							if (entries.size() >= 12) // case with optional force inputs (12 total entries)
							{
								for (int I=0; I<3; I++)
								{	r0[I] = atof(entries[2+I].c_str());
									F[ I] = atof(entries[7+I].c_str());
								}
								CdA  = atof(entries[10].c_str());
								Ca   = atof(entries[11].c_str());
							}
							else // case without optional force inputs (9 total entries)
							{
								for (int I=0; I<3; I++)
									r0[I] = atof(entries[2+I].c_str());
							
								CdA  = atof(entries[7].c_str());
								Ca   = atof(entries[8].c_str());
							}
							
							
							// ----------- process connection type -----------------
							int type;						
							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							char typeWord[10];							// the buffer
													
							snprintf(typeWord, 10, entries[1].c_str());		// copy connection type word to buffer
							
							decomposeString(typeWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
								
							if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0))
							{
								type = 1;                                   // if an anchor
								
								// add to GroundBody
								//GroundBody->addConnectionToBody(ConnectionList[iConnection], r0);  // note: object not made yet, but only need pointer to pass
								//AnchIs.push_back(iConnection);
								//nAnchs ++;
							}
							else if (strcmp(let1, "BODY") ==0)
							{
								type = 1;
								if (strlen(num1)>0)
								{
									int bodyID = atoi(num1);
									if ((bodyID <= nBodys) && (bodyID > 0))
									{
										//BodyList[bodyID-1]->addConnectionToBody(ConnectionList[iConnection], r0);  // note: object not made yet, but only need pointer to pass
									}
									else
									{	cout << "Error: Body ID out of bounds for Connection " << number << "." << endl;
										return -1;								
									}
								}
								else
								{	cout << "Error: no number provided for Connection " << number << " Body attachment." << endl;
									return -1;
								}
							}
							else if ((strcmp(let1, "FAIRLEAD") ==0) || (strcmp(let1, "VESSEL") ==0) || (strcmp(let1, "VES") ==0) || (strcmp(let1, "COUPLED") ==0) || (strcmp(let1, "CPLD") ==0))
							{	// if a fairlead, add to list and add 
								type = -1;
								rFairCon.push_back(vector<double>(3, 0.0));		// fairlead location in turbine ref frame
								rFairCon.back().at(0) = atof(entries[2].c_str()); 	// x
								rFairCon.back().at(1) = atof(entries[3].c_str()); 	// y
								rFairCon.back().at(2) = atof(entries[4].c_str()); 	// z	
								CpldConIs.push_back(iConnection);			// index of fairlead in ConnectionList vector
								//nFairs ++;
							}						
							else if ((strcmp(let1, "CONNECT") ==0) || (strcmp(let1, "CON") ==0) || (strcmp(let1, "FREE") ==0))
							{
								type = 0;                                // if a connect, add to list and add states for it
								FreeConIs.push_back(iConnection);
								//nConns ++;
								
								ConnectStateIs.push_back(nX);   // assign start index of this connect's states
								nX += 6;                       // add 6 state variables for each connect
							}
							else 
							{	// throw error if not identified
								stringstream s;
								s << "Error: unidentified Type/BodyID for connection " << number << ": " << typeWord;
								throw string(s.str());
							}
							
							
							// make default water depth at least the depth of the lowest node (so water depth input is optional)
							if (r0[2] < -env.WtrDpth)  env.WtrDpth = -r0[2];
						
							// now make Connection object!
							ConnectionList[iConnection] = new Connection();
							ConnectionList[iConnection]->setup(number, type, r0, M, V, F, CdA, Ca);
							
							
							// depending on type, assign the Connection to its respective parent body
							if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0))// type = 1;                          
								GroundBody->addConnectionToBody(ConnectionList[iConnection], r0); 
							else if (strcmp(let1, "BODY") ==0) // type = 1;
							{
								int bodyID = atoi(num1);
								BodyList[bodyID-1]->addConnectionToBody(ConnectionList[iConnection], r0);
							}
							
							//if (ConnectionList.size() != number)  // check that ID numbers are in order
							//	cout << "Warning: connect ID numbers should be in order (1,2,3...)." << endl;
													
							
							if (wordy>0) cout << number << " ";
						}
						else 
						{
							cout << endl << "   Error - less than the 9 required input columns for connection " << entries[0] << " definition.  Remember CdA and Ca." << endl;
							cout << "   The line in question was read as: ";
							for (int ii=0; ii<entries.size(); ii++) cout << entries[ii] << " ";
							cout << endl;
							
							return -1;
						}
						//i++;
					}
					if (wordy>0) cout << "\n";
				}
				else if ((lines[i].find("LINES") != string::npos) || (lines[i].find("LINES LIST") != string::npos) || (lines[i].find("LINE PROPERTIES") != string::npos)) // if line properties header
				{	
					if (wordy>0) cout << "   Reading line list: ";
					i += 3; // skip following two lines (label line and unit line)
					
					
					// set up each entry
					for (int iLine=0; iLine<nLines; iLine++)
					{ 	
						//vector<string> entries = split(lines[i-nLines+iLine], ' '); // split by spaces
						vector<string> entries = split(lines[i+iLine]); // split by spaces
							
						if (entries.size() >= 7) // if valid number of inputs
						{
							// Line     LineType  UnstrLen   NodeAnch  NodeFair  Flags/Outputs
							
																			
							int number= atoi(entries[0].c_str());
							string type     = entries[1];
							double UnstrLen = atof(entries[4].c_str());
							int NumSegs    = atoi(entries[5].c_str()); // addition vs. MAP
							//string anchID    = entries[2];
							//string fairID    = entries[3];
							string outchannels  = entries[6];
							
							// find line properties index
							int TypeNum = -1;
							for (int J=0; J<nLineTypes; J++)  {
								if (LinePropList[J]->type.find(type) != string::npos)
									TypeNum = J;
							}
							
							if (TypeNum == -1)
								cout << "   Error: unable to identify type of line " << number << " (" << type << ") from those provided." << endl;
							
							if (wordy>1) cout << "line " << number << " type " << type << " typenum " << TypeNum << endl;
							
							// make an output file for it
							if ((outchannels.size() > 0) && (strcspn( outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))  // if 1+ output flag chars are given and they're valid
							{	stringstream oname;
								oname << MDbasepath << MDbasename << "_Line" << number << ".out";
								outfiles.push_back( make_shared<ofstream>(oname.str()));
							}
							else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
							
							// set up line properties
							LineList[iLine] = new Line();
							LineList[iLine]->setup(number, LinePropList[TypeNum], UnstrLen, NumSegs, 
								//ConnectionList[AnchIndex], ConnectionList[FairIndex], 
								outfiles.back(), outchannels);
									
							LineStateIs.push_back(nX);  // assign start index of this Line's states
							nX += 6*(NumSegs - 1);                   // add 6 state variables for each internal node of this line
									
							//if (LineList.size() != number)  // check that ID numbers are in order
							//	cout << "Warning: line ID numbers should be in order (1,2,3...)." << endl;
							
							
							// ================ Process attachment identfiers and attach line ends ===============
																		
							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							
							char outWord[10];							// the buffer
							
							// first for anchor...
							
							snprintf(outWord, 10, entries[2].c_str());		// copy anchor connection word to buffer
							
							decomposeString(outWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
												
							if (strlen(num1)<1)
							{	cout << "Error: no number provided for line " << number << " anchor attachment." << endl;
								return -1;
							}
							
							int id = atoi(num1);
								
							// if id starts with an "R" or "Rod"
							if ((strcmp(let1, "R") == 0) || (strcmp(let1, "ROD") == 0))
							{
								if ((id <= nRods) && (id > 0))
								{
									if (strcmp(let2, "A") == 0) 
										RodList[id-1]->addLineToRodEndA(LineList[iLine], 0);
									else if (strcmp(let2, "B") == 0) 
										RodList[id-1]->addLineToRodEndB(LineList[iLine], 0);
									else
									{	cout << "Error: rod end (A or B) must be specified for line " << number << " anchor attachment." << endl;
										return -1;										
									}
								}
								else
								{	cout << "Error: rod connection ID out of bounds for line " << number << " anchor attachment." << endl;
									return -1;								
								}
									
							}
							// if id starts with a "C" or "Con" or goes straight ot the number then it's attached to a Connection
							if ((strlen(let1)==0) || (strcmp(let1, "C") == 0) || (strcmp(let1, "CON") == 0))
							{
								if ((id <= nConnections) && (id > 0))
								{
									ConnectionList[id-1]->addLineToConnect(LineList[iLine], 0);
								}
								else
								{	cout << "Error: connection ID out of bounds for line " << number << " anchor attachment." << endl;
									return -1;								
								}
									
							}
							
							// then again for fairlead
							
							snprintf(outWord, 10, entries[3].c_str());		// copy fairlead connection word to buffer
							
							decomposeString(outWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
												
							if (strlen(num1)<1)
							{	cout << "Error: no number provided for line " << number << " fairlead attachment." << endl;
								return -1;
							}
							
							id = atoi(num1);
								
							// if id starts with an "R" or "Rod"
							if ((strcmp(let1, "R") == 0) || (strcmp(let1, "ROD") == 0))
							{
								if ((id <= nRods) && (id > 0))
								{
									if (strcmp(let2, "A") == 0) 
										RodList[id-1]->addLineToRodEndA(LineList[iLine], 1);
									else if (strcmp(let2, "B") == 0) 
										RodList[id-1]->addLineToRodEndB(LineList[iLine], 1);
									else
									{	cout << "Error: rod end (A or B) must be specified for line " << number << " fairlead attachment." << endl;
										return -1;										
									}
								}
								else
								{	cout << "Error: rod connection ID out of bounds for line " << number << " fairlead attachment." << endl;
									return -1;								
								}
									
							}
							// if id starts with a "C" or "Con" or goes straight ot the number then it's attached to a Connection
							if ((strlen(let1)==0) || (strcmp(let1, "C") == 0) || (strcmp(let1, "CON") == 0))
							{
								if ((id <= nConnections) && (id > 0))
								{
									ConnectionList[id-1]->addLineToConnect(LineList[iLine], 1);
								}
								else
								{	cout << "Error: connection ID out of bounds for line " << number << " fairlead attachment." << endl;
									return -1;								
								}
									
							}
							
						}					
						else 
						{
							cout << endl << "   Error with line " << entries[0] << " inputs." << endl;
							return -1;
						}
						//i++;
					}		
					if (wordy>0) cout << "\n";		
				}
				else if (lines[i].find("FAILURE") != string::npos) // if failure conditions header
				{	
					if (wordy>0) cout << "   Reading failure conditions: ";
					
					i += 3; // skip following two lines (label line and unit line)
					
					
					// set up each entry
					for (int iFail=0; iFail<nFails; iFail++)
					{
						cout << "failure case " << iFail+1 ;
						
						vector<string> entries = split(lines[i+iFail]); // split by spaces
												
						if (entries.size() >= 4) // if valid number of inputs
						{	
							FailList[iFail] = new FailProps();
							
							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							char outWord[10];							// the buffer
														
							snprintf(outWord, 10, entries[0].c_str());		// copy connection word to buffer
							
							decomposeString(outWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
							
							if (strlen(num1)<1)
							{	cout << "Error: no Node provided for Failure " << iFail+1 << endl;
								return -1;
							}
							
							FailList[iFail]->attachID = atoi(num1);   // ID of connection or Rod the lines are attached to (index is -1 this value)
								
							// if id starts with an "R" or "Rod"
							if ((strcmp(let1, "R") == 0) || (strcmp(let1, "ROD") == 0))
							{
								if ((FailList[iFail]->attachID <= nRods) && (FailList[iFail]->attachID > 0))
								{
									if (strcmp(let2, "A") == 0) 
										FailList[iFail]->isRod = 1;
									else if (strcmp(let2, "B") == 0) 
										FailList[iFail]->isRod = 2;
									else
									{	cout << "Error: at line 1589ish" << endl;
										return -1;										
									}
								}
								else
								{	cout << "Error at line 1594ish" << endl;
									return -1;								
								}
									
							}
							// if id starts with a "C" or "Con" or goes straight ot the number then it's attached to a Connection
							if ((strlen(let1)==0) || (strcmp(let1, "C") == 0) || (strcmp(let1, "CON") == 0))
							{
								if ((FailList[iFail]->attachID <= nConnections) && (FailList[iFail]->attachID > 0))
								{
									FailList[iFail]->isRod = 0;
								}
								else
								{	cout << "Error: at line 1607ish" << endl;
									return -1;								
								}
									
							}
							
							// get lines
							vector<string> lineNums = splitComma(entries[1]); // split by commas
							FailList[iFail]->nLinesToDetach = lineNums.size();  // how many lines to dettach
							for (int il=0; il<lineNums.size(); il++)
								FailList[iFail]->lineIDs[il] = atoi(lineNums[il].c_str());  

							FailList[iFail]->failTime = atof(entries[2].c_str());
							FailList[iFail]->failTen  = atof(entries[3].c_str());
							
							cout << " failTime is "<< FailList[iFail]->failTime << endl;
							
							FailList[iFail]->failStatus  = 0; // initialize as unfailed, of course
						}
						//i++;
					}
					if (wordy>0) cout << "\n";
				}
				else if (lines[i].find("OUTPUT") != string::npos) // if output list header
				{	
					//cout << "in output section" << endl;
					i ++;
					while (lines[i].find("---") == string::npos) // while we DON'T find another header line
					{ 	
						vector<string> entries = split(lines[i]);
						
						for (int j=0; j<entries.size(); j++)  //loop through each word on each line
						{

							// ----------------- Process each "word" -----------------------
							// (set index, name, and units for all of each output channel)

							char outWord[10];							// the buffer
							snprintf(outWord, 10, entries[j].c_str());		// copy word to buffer
																		
							// substrings of grouped letters or numbers for processing each parameter                          
							char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
							
							decomposeString(outWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
							
							// <<<<<<<<<<<<< check for errors! <<<<<<<<<<<<<<<						
							//	cout << "   Error: no number in channel name ("  << outWord << ")." << endl;
							//	cout << "Warning: invalid output specifier (must start with letter)." << endl;
							
							
							OutChanProps dummy;  		// declare dummy struct to be copied onto end of vector (and filled in later);
							strncpy(dummy.Name, outWord, 10); //strlen(outWord));		// label channel with whatever name was inputted, for now

							// figure out what type of output it is and process accordingly 
							// TODO: add checks of first char of num1,2, let1,2,3 not being NULL to below and handle errors (e.g. invalid line number)

							const int UnitsSize = 10;
							// fairlead tension case (changed to just be for single line, not all connected lines)
							if (strcmp(let1, "FAIRTEN")==0)
							{	
								dummy.OType = 1;             							// line object type
								dummy.QType = Ten;           							// tension quantity type
								strncpy(dummy.Units, UnitList[Ten], UnitsSize);     // set units according to QType
								dummy.ObjID = atoi(num1);                             // get the line number
								dummy.NodeID = LineList[dummy.ObjID-1]->getN();       // specify node N (fairlead)
							}
							// achor tension case (changed to just be for single line, not all connected lines)
							else if (strcmp(let1, "ANCHTEN")==0) 
							{	
								dummy.OType = 1;             							// line object type
								dummy.QType = Ten;           							// tension quantity type
								strncpy(dummy.Units, UnitList[Ten], UnitsSize);     // set units according to QType
								dummy.ObjID = atoi(num1);                             // get the line number
								dummy.NodeID = 0;                                      // specify node 0 (anchor)
							}
							// more general case
							else
							{
								// get object type and node number if applicable
								// Line case                                          ... L?N?xxxx
								if (strcmp(let1, "L")==0)
								{
									//cout << "found line" << endl;
									dummy.OType = 1;                // Line object type
									// for now we'll just assume the next character(s) are "n" to represent node number:
									dummy.NodeID = atoi(num2);
								}
								// Connect case                                     ... C?xxx or Con?xxx
								else if((strcmp(let1, "C")==0) || (strcmp(let1, "CON")==0))
								{
									//cout << "found connect " << endl;
									dummy.OType = 2;                // Connect object type
									dummy.NodeID = -1;
									strncpy(let3, let2, 10);				// copy quantity chars (let2) to let3 (unused for a connect) because let3 is what's checked below
								}
								// Rod case                                     ... R?xxx or Rod?xxx
								else if((strcmp(let1, "R")==0) || (strcmp(let1, "ROD")==0))
								{
									//cout << "found rod " << endl;
									dummy.OType = 3;                // Rod object type
									dummy.NodeID = atoi(num2);
									
								}
								// should do fairlead option also!
								
								else   // error
								{	//CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
									cout << "Warning: invalid output specifier: "  << let1 << ".  Type must be L or C/Con." << endl;
									dummy.OType = -1;  // flag as invalid
									continue;  // break out of this loop iteration (don't add current output channel to list)
								}

								// object number
								dummy.ObjID =  atoi(num1);             // line or connect ID number

								// which kind of quantity?
								if (strcmp(let3, "PX")==0) {
									//cout << "SETTING QTYPE to " << PosX << endl;
								  dummy.QType = PosX;
								  strncpy(dummy.Units, UnitList[PosX], UnitsSize);
								}
								else if (strcmp(let3, "PY")==0)  {
								  dummy.QType = PosY;
								  strncpy(dummy.Units, UnitList[PosY], UnitsSize);
								}
								else if (strcmp(let3, "PZ")==0)  {
								  dummy.QType = PosZ;
								  strncpy(dummy.Units, UnitList[PosZ], UnitsSize);
								}
								else if (strcmp(let3, "VX")==0)  {
								  dummy.QType = VelX;
								  strncpy(dummy.Units, UnitList[VelX], UnitsSize);
								}
								else if (strcmp(let3, "VY")==0)  {
								  dummy.QType = VelY;
								  strncpy(dummy.Units, UnitList[VelY], UnitsSize);
								}
								else if (strcmp(let3, "VZ")==0)  {
								  dummy.QType = VelZ;
								  strncpy(dummy.Units, UnitList[VelZ], UnitsSize);
								}
								else if (strcmp(let3, "AX")==0)  {
								  dummy.QType = AccX;
								  strncpy(dummy.Units, UnitList[AccX], UnitsSize);
								}
								else if (strcmp(let3, "Ay")==0)  {
								  dummy.QType = AccY;
								  strncpy(dummy.Units, UnitList[AccY], UnitsSize);
								}
								else if (strcmp(let3, "AZ")==0)  {
								  dummy.QType = AccZ;
								  strncpy(dummy.Units, UnitList[AccZ], UnitsSize);
								}
								else if ((strcmp(let3, "T")==0) || (strcmp(let3, "TEN")==0)) {
								  dummy.QType = Ten;
								  strncpy(dummy.Units, UnitList[Ten], UnitsSize);
								}
								else if (strcmp(let3, "FX")==0)  {
								  dummy.QType = FX;
								  strncpy(dummy.Units, UnitList[FX], UnitsSize);
								}
								else if (strcmp(let3, "FY")==0)  {
								  dummy.QType = FY;
								  strncpy(dummy.Units, UnitList[FY], UnitsSize);
								}
								else if (strcmp(let3, "FZ")==0)  {
								  dummy.QType = FZ;
								  strncpy(dummy.Units, UnitList[FZ], UnitsSize);
								}
								else
								{  	
									cout << "Warning: invalid output specifier - quantity type not recognized" << endl;
									dummy.QType = -1;  // flag as invalid
									continue;  // break out of this loop iteration (don't add current output channel to list)
									
								}

							}
							
							
							// some name adjusting for special cases (maybe should handle this elsewhere...)
							if ((dummy.OType==3) && (dummy.QType==Ten))
							{
								if (dummy.NodeID > 0) strncpy(dummy.Name,"TenEndB", 10);
								else                  strncpy(dummy.Name,"TenEndA", 10);
							}
							
							

							//  ! also check whether each object index and node index (if applicable) is in range
							//  IF (p%OutParam(I)%OType==2) THEN
							//    IF (p%OutParam(I)%ObjID > p%nConnections) THEN
							//      call wrscr('warning, output Connect index excedes number of Connects')
							//      CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
							//    END IF
							//  ELSE IF (p%OutParam(I)%OType==1) THEN
							//    IF (p%OutParam(I)%ObjID > p%NLines) THEN
							//      call wrscr('warning, output Line index excedes number of Line')
							//      CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
							//    END IF
							//    IF (p%OutParam(I)%NodeID > other%LineList(p%OutParam(I)%ObjID)%N) THEN
							//      call wrscr('warning, output node index excedes number of nodes')
							//      CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
							//    ELSE IF (p%OutParam(I)%NodeID < 0) THEN
							//      call wrscr('warning, output node index is less than zero')
							//      CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
							//    END IF
							//  END IF

							
							if ((dummy.OType > 0) && (dummy.QType > 0))
								outChans.push_back(dummy);  	// if valid, add new entry to list!
							
							
							
						}  // looping through words on line
					//     SUBROUTINE DenoteInvalidOutput( OutParm )
					//        TYPE(MD_OutParmType), INTENT (INOUT)  :: OutParm
					//
					//        OutParm%OType = 0  ! flag as invalid
					//        OutParm%Name = 'Invalid'
					//        OutParm%Units = ' - '
					//
					//     END SUBROUTINE DenoteInvalidOutput					
						i++;
						
					}  // looping through lines
				}
				else i++;
			}
			else i++;
		}
		
		
		// ==============================================================================

	// do some input validity checking?
	// should there be a flag in the input file that clearly distingiushes the coupling type? <<<<< I guess it's implied by whether bodies are coupled or not??
	// TODO: make sure things are consistent for only ONE coupling type (body centric or fairlead centric) <<<<<<<<<<<<<<<< also do checks when time step function is called...


	   if (wordy > 1) 
	   {
		   cout << "  nLineTypes     = " << nLineTypes        << endl;
		   cout << "  nRodTypes      = " << nRodTypes         << endl;
		   cout << "  nPoints        = " << nConnections      << endl;
		   cout << "  nBodies        = " << nBodys            << endl;
		   cout << "  nRods          = " << nRods             << endl;
		   cout << "  nLines         = " << nLines            << endl;
		   cout << "  nFails         = " << nFails            << endl;
		   cout << "  nFreeBodies    = " << FreeBodyIs.size() << endl;  
		   cout << "  nFreeRods      = " << FreeRodIs.size()  << endl;  
		   cout << "  nFreePonts     = " << FreeConIs.size()  << endl;  
		   cout << "  nCpldBodies    = " << CpldBodyIs.size() << endl;
		   cout << "  nCpldRods      = " << CpldRodIs.size()  << endl; 
		   cout << "  nCpldPoints    = " << CpldConIs.size()  << endl;
	   }
	   
		// write system description to log file
		if (env.writeLog > 0)
		{
			char outline[100];
		
			snprintf(outline, sizeof(outline), "%s", "----- MoorDyn Model Summary (to be written) -----");
		
			outfileLog << outline << "\n";
		}

		// make sure non-NULL kinematics are being passed if anything is coupled
		int nCpldDOF = 6*CpldBodyIs.size() +  3*CpldConIs.size();  // number of coupled degrees of freedom
		for (int l=0; l<CpldRodIs.size(); l++)  
		{	if (RodList[CpldRodIs[l]]->type == -2)
				nCpldDOF += 6;                                     // for cantilevered rods 6 entries will be taken
			else
				nCpldDOF += 3;                                     // for pinned rods 3 entries will be taken
		}			
		
		cout << "Based on the input file, MoorDyn is expecting " << nCpldDOF << " coupled degrees of freedom." << endl;
			
		if (nCpldDOF > 0)
		{
			if (x==NULL)
				cout << "ERROR: MoorDynInit received a Null position vector, but expects size " << nCpldDOF << endl;
		}


		if (nX == 0)
		{
			cout << "ERROR: MoorDyn has no state variables. (Is there a mooring sytem?) " << nCpldDOF << endl;
		}
		//nConnections = ConnectionList.size();
		//nLines = LineList.size();
		//nRods = RodList.size();
		
		// <<<<<<<<< need to add bodys
				

		//  ------------------------ set up waves if needed -------------------------------
		
		waves = new Waves();        // make waves object
		
		waves->setup(&env);          // set up the waves (this will pass the WaveKin flag in the env struct, which the setup function will then use as needed)
		
//		env.waves = waves;  // now assign the waves pointer to the env struct so that it gets passed to other objects that might need it
		
	/*	// @mth: new approach to wave kinematics will be implemented - this part needs to be redone	
	//	// (in general, set up wave stuff here BEFORE adding line to vector)
		vector<double> Ucurrent(3, 0.0);  // should make this an input to the DLL at some point.

	
	//	if (env.WaveKin == 2)
	//	{
	//		if (wordy>0) cout << "   Setting up wave kinematics by reading from file" << endl;
	//		SetupWavesFromFile();
	//				
	//		for (int l=0; l<nLines; l++) 
	//			LineList[l].setupWaves(env, zetaCglobal,  dwW, 0.25 );  // TODO: update.  last entry is bogus!
	//	}			
	//	else
	//	{ 	// no waves case
			for (int l=0; l<nLines; l++) 
				LineList[l].setupWaves(env, Ucurrent, 0.);   // sending env struct (important)
	//	}								
		
		// note: each Line's WaveKin switch should be off by default, and can be switched on when the wave kinematics 
		// are calculated AFTER the initial position has been solved for.
	*/
		
		// send environmental properties struct to all objects 
		GroundBody->setEnv( &env, waves); 
		for (int l=0; l<nBodys; l++)  
			BodyList[l]->setEnv( &env, waves); 
		for (int l=0; l<nRods; l++)  
			RodList[l]->setEnv( &env, waves); 
		for (int l=0; l<nConnections; l++)  
			ConnectionList[l]->setEnv( &env, waves); 
		for (int l=0; l<nLines; l++)  
			LineList[l]->setEnv( &env, waves); 

		
		
		// ----------------- allocate arrays ------------------
		
		// calculate maximum possible size of state vector if new connects were created for each line end that 
		// could become detached (limit case). This just avoids having to reallocate the states vectors if lines detach.
		nXtra = nX + 6*2*nLines;     
			
		// make state vector	
		if (wordy > 1) cout << "   Creating state vectors of size " << nXtra << endl;
		states    = (double*) malloc( nXtra*sizeof(double) );

		// make arrays for integration
		f0 = (double*) malloc( nXtra*sizeof(double) );
		f1 = (double*) malloc( nXtra*sizeof(double) );
		xt = (double*) malloc( nXtra*sizeof(double) );
		
		memset(states, 0.0, nXtra*sizeof(double));
		
		// make array used for passing fairlead kinematics and forces between fairlead- and platform-centric interface functions
	//	Ffair = make2Darray(nFairs, 3); 
		
	//	rFairi = make2Darray(nFairs, 3);
	//	rdFairi = make2Darray(nFairs, 3);
		
		// --------- Allocate/size some global, persistent vectors -------------


		FairTensLast = make2Darray(nLines, 10); // allocate past line fairlead tension array, which is used for convergence test during IC gen
		
		for (int i=0; i < nLines; i++) for (int j=0; j < 10; j++) FairTensLast[i][j] = 1.0*j;
		
			
		// ------------------- initialize system, including trying catenary IC gen of Lines -------------------
		
		cout << "   Creating mooring system.  " << endl; // << nFairs << " fairleads, " << nAnchs << " anchors, " << nConns << " connections." << endl;	
		//for (int l=0; l<nConnections; l++)  {
		//	ConnectionList[l].initialize( (states + 6*l), env, X, TransMat); // connections
		//}	
		
		
		// call ground body to update all the fixed things...
		GroundBody->initializeUnfreeBody(NULL, NULL, 0.0);
		
		
		// initialize coupled objects based on passed kinematics
		
		int ix = 0;
		
		for (int l=0; l<CpldBodyIs.size(); l++)
		{	if (wordy>0) cout << "Initializing coupled Body " << CpldBodyIs[l] << endl;
			BodyList[CpldBodyIs[l]]->initializeUnfreeBody(x + ix, xd + ix, 0.0);   // this calls initiateStep and updateFairlead, then initializes dependent Rods
			ix += 6;
		}
		
		for (int l=0; l<CpldRodIs.size(); l++)  
		{	if (wordy>0) cout << "Initializing coupled Rod " << CpldRodIs[l] << endl;
			RodList[CpldRodIs[l]]->initiateStep(x + ix, xd + ix, 0.0);
			RodList[CpldRodIs[l]]->updateFairlead(0.0);
			RodList[CpldRodIs[l]]->initializeRod(NULL);  // call this just to set up the output file header
			
			if (RodList[CpldRodIs[l]]->type == -2)
				ix += 6;                                     // for cantilevered rods 6 entries will be taken
			else
				ix += 3;                                     // for pinned rods 3 entries will be taken
		}	

		for (int l=0; l<CpldConIs.size(); l++)  
		{	if (wordy>0) cout << "Initializing coupled Connection " << CpldConIs[l] << endl;
			ConnectionList[CpldConIs[l]]->initiateStep(x + ix, xd + ix, 0.0);
			ConnectionList[CpldConIs[l]]->updateFairlead(0.0);
			ix += 3;
		}
		
		
		// initialize objects with states, writing their initial states to the master state vector (states)
		
		// Go through Bodys and write the coordinates to the state vector
		for (int l=0; l<FreeBodyIs.size(); l++)  
			BodyList[FreeBodyIs[l]]->initializeBody(states + BodyStateIs[l]);
		
		// Go through independent (including pinned) Rods and write the coordinates to the state vector
		for (int l=0; l<FreeRodIs.size(); l++)  
			RodList[FreeRodIs[l]]->initializeRod(states + RodStateIs[l]);
		
		// Go through independent connections (Connects) and write the coordinates to 
		// the state vector and set positions of attached line ends
		for (int l=0; l<FreeConIs.size(); l++)  
			ConnectionList[FreeConIs[l]]->initializeConnect(states + ConnectStateIs[l]);
			
			
		// Lastly, go through lines and initialize internal node positions using quasi-static model
		for (int l=0; l<nLines; l++)  
			LineList[l]->initializeLine( states + LineStateIs[l] ); 
		
		
		
		// write t=-1 output line for troubleshooting preliminary ICs
		//AllOutput(-1.0);
		//cout << "outputting ICs for troubleshooting" << endl;
		
		
		// ------------------ do dynamic relaxation IC gen --------------------
		
		cout << "   Finalizing ICs using dynamic relaxation (" << ICDfac << "X normal drag)" << endl;
		
		// boost drag coefficients to speed static equilibrium convergence
		for (int l=0; l < nLines; l++)               LineList[l]->scaleDrag(ICDfac); 	
		for (int l=0; l < nConnections; l++) ConnectionList[l]->scaleDrag(ICDfac);
		for (int l=0; l < nRods; l++)                 RodList[l]->scaleDrag(ICDfac);
		for (int l=0; l < nBodys; l++)               BodyList[l]->scaleDrag(ICDfac);
		
		int niic = round(ICTmax/ICdt);			// max number of IC gen time steps
		
	//	vector< double > Tensions(nLines*3*niic, 0.0); // vector to store tensions for analyzing convergence
		vector< double > FairTens(nLines, 0.0); 		// vector to store tensions for analyzing convergence
		//vector< double > FairTensLast(nFairs, 0.0); 	// vector to store tensions for analyzing convergence
		//vector< double > FairTensLast2(nFairs, 0.0); 	// vector to store tensions for analyzing convergence
				
		// round to get appropriate mooring model time step
		int NdtM = ceil(ICdt/dtM0);   // number of mooring model time steps per outer time step
		double dtM = ICdt/NdtM;		// mooring model time step size (s)
			
		// loop through IC generation time analysis time steps
		for (int iic=0; iic<niic; iic++)
		{
			double t = iic*ICdt;			// IC gen time (s).  << is this a robust way to handle time progression?
			
			// loop through line integration time steps
			for (int its = 0; its < NdtM; its++)
				rk2 (states, &t, dtM );  			// call RK2 time integrator (which calls the model)
		
			// check for NaNs
			for (int i=0; i<nX; i++)
			{
				if (isnan(states[i]))
				{
					cout << "   Error: NaN value detected in MoorDyn state at dynamic relaxation time " << t << " s." << endl;
					return -1;
				}
			}
		
			// store previous fairlead tensions for comparison
			for (int lf=0; lf<nLines; lf++) 
			{
				for (int pt=9; pt>0; pt--)
					FairTensLast[lf][pt] = FairTensLast[lf][pt-1];
				
				FairTensLast[lf][0] = FairTens[lf];
			}
		
			// go through connections to get fairlead forces 
			for (int lf=0; lf<nLines; lf++) 
			{
				//double Ffair[3];						// array to temporarily store fairlead force components
				//double FairTenSqrd = 0.0;
		
				FairTens[lf] = LineList[lf]->getNodeTen(LineList[lf]->getN());  // get fairlead tension of each line
				
				//for (int j=0; j<3; j++) 
				//	FairTenSqrd += Ffair[j]*Ffair[j];
				//
				//FairTens[lf] = sqrt(FairTenSqrd);        // <<<<< why is this returnign zeros???
			}
					
		//	cout << "size of FairIs is " << FairIs.size() << endl;
		//	cout << "FairIs 1,2 are " << FairIs[0] << FairIs[1] << endl;
		//	cout << "size of ConnectionList is " << ConnectionList.size() << endl;
		//	cout << " nFairs is " << nFairs << endl;
			
					
			//cout << "    t = " << t << " s, tension at first fairlead is " << FairTens[0] << " N    \r";   // write status update and send cursor back to start of line

		//	cout << "IC step number is " << iic << " and latest tensions are as follows:" << endl;
		//	
		//	for (int lf=0; lf<nLines; lf++) {
		//		cout << "Line " << lf << " fairlead" << endl;
		//		cout << FairTens[lf] << endl;
		//		//for (int pt=0; pt<10; pt++)
		//		//	cout << FairTensLast[lf][pt] << endl; 
		//	}	


	// >>>>>>>>> is convergence check working yet for all lines???

			// check for convergence (compare current tension at each fairlead with previous 9 values)
			if (iic > 10)
			{
				// check for any non-convergence, and continue to the next time step if any occurs
				int converged=1;
				for (int lf=0; lf<nLines; lf++) 
				{
					for (int pt=0; pt<10; pt++)
					{
						if ( abs( FairTens[lf] / FairTensLast[lf][pt] - 1.0 ) > ICthresh ) 
						{
							converged=0;	
							break;						
						}
					}
					if (converged == 0) break; // break out of this inner loop
				}
				
				if (converged == 1) // if no non-convergence conditions have occured, i.e. we've converged, move on
				{
					cout << "   Fairlead tensions converged to " << 100.0*ICthresh << "\% after " << t << " seconds.        " << endl;
				//	if (wordy > 0) 
				//	{	
				//		cout << "IC step number is " << iic << " and converged tensions are as follows:" << endl;
				//		
				//	//	for (lf=0; lf<nFairs; lf++) {
				//	//		cout << "Fairlead " << lf << endl;
				//	//		cout << FairTens[lf] << endl;
				//	//		for (int pt=0; pt<10; pt++)
				//	//			cout << FairTensLast[lf][pt] << endl; 
				//	//	}	
				//	}
					
					break; // break out of the time stepping loop
				}
			}
		}
		
		// restore drag coefficients to normal values and restart time counter of each object
		for (int l=0; l < nLines; l++) 
		{	LineList[l]->scaleDrag(1.0/ICDfac); // restore drag coefficients
			LineList[l]->setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
		}
		for (int l=0; l < nConnections; l++) 
		{	ConnectionList[l]->scaleDrag(1.0/ICDfac); // restore drag coefficients
			ConnectionList[l]->setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
		}
		for (int l=0; l < nRods; l++) 
		{	RodList[l]->scaleDrag(1.0/ICDfac); // restore drag coefficients
			RodList[l]->setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
		}
		for (int l=0; l < nBodys; l++) 
		{	BodyList[l]->scaleDrag(1.0/ICDfac); // restore drag coefficients
			BodyList[l]->setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
		}
		

		// store passed WaveKin value to enable waves in simulation if applicable (they're not enabled during IC gen)
		env.WaveKin = WaveKinTemp;

	// @mth: new approach to be implemented
	//	// ------------------------- calculate wave time series if needed -------------------
	//	if (env.WaveKin == 2)
	//	{
	//		for (int l=0; l<nLines; l++) 
	//			LineList[l]->makeWaveKinematics( 0.0 );
	//	}

		
		// -------------------------- start main output file --------------------------------
		
		stringstream oname;
		oname << MDbasepath << MDbasename << ".out";
		
		outfileMain.open(oname.str());
		if (outfileMain.is_open())
		{
			// --- channel titles ---
			outfileMain << "Time" << "\t "; 	
			// output all LINE fairlead (top end) tensions
			for (int lf=0; lf<outChans.size(); lf++) outfileMain << outChans[lf].Name << "\t ";
			
			outfileMain << "\n";
			
			if (env.WriteUnits > 0)
			{
				// --- units ---
				outfileMain << "(s)" << "\t "; 	
				// output all LINE fairlead (top end) tensions
				for (int lf=0; lf<outChans.size(); lf++) outfileMain << outChans[lf].Units << "\t ";
				outfileMain << "\n";
			}
		}
		else 
		{	cout << "   ERROR: Unable to write to main output file " << oname.str() << endl;  //TODO: handle error <<<<<<<<<<<<<<<<<<<<
			return -1;
		}
		
		// write t=0 output line
		if (AllOutput(0.0, 0.0) < 0)
			return -1;
		
							
		cout <<endl;
		return 0;
//--	}
//--	catch(string e) 
//--	{
//--		cout << "\nError in MoorDynInit\n" << e << endl;
//--		throw string("MoorDynInit error!");
//--		return -1;
//--	}
//--	
//--	cout << "It seems whtere was an unahndeld exception, and Im not sure i can know what it is" << endl;
//--	return -1;
	
}


// @mth: placeholder for now 
// 
// // accept wave parameters from calling program and precalculate wave kinematics time series for each node
// int DECLDIR SetupWaves(int* WaveMod, int* WaveStMod, 
// 	float* WaveHs, float* WaveTp, float* WaveDir, int* NStepWave2, float* WaveDOmega, 
// 	float* WGNC_Fact, float WGNCreal[], float WGNCimag[], float* S2Sd_Fact, float WaveS2Sdd[], float* WaveDT)
// {
// 
// 	return 0.0;
// }


// @mth: moved
// // load time series of wave elevations and process to calculate wave kinematics time series for each node
// int SetupWavesFromFile(void)




// This is the original time stepping function, for platform-centric coupling, but now it has capabilities for multiple 6DOF coupled bodies
int DECLDIR MoorDynStep(double x[], double xd[], double f[], double* t_in, double* dt_in) 
{

	double t =  *t_in;		// this is the current time
	double dtC =  *dt_in;	// this is the coupling time step
	
	
	int ix = 0; // index of each successive object's starting point within the x, xd, and f arrays
	
	// should check if wave kinematics have been set up if expected!
	
	if (dtC > 0) // if DT > 0, do simulation, otherwise just return last calculated values.
	{
	
		// ---------------- set positions and velocities -----------------------
		//... of any coupled bodies, rods, and connections at this instant, to be used later for extrapolating motions
	
		ix = 0;
		
		for (int l=0; l<CpldBodyIs.size(); l++)
		{
			BodyList[CpldBodyIs[l]]->initiateStep(x + ix, xd + ix, t);
			ix += 6;
		}
		
		for (int l=0; l<CpldRodIs.size(); l++)  
		{
			RodList[CpldRodIs[l]]->initiateStep(x + ix, xd + ix, t); 	
			
			if (RodList[CpldRodIs[l]]->type == -2)
				ix += 6;                                     // for cantilevered rods 6 entries will be taken
			else
				ix += 3;                                     // for pinned rods 3 entries will be taken
		}	

		for (int l=0; l<CpldConIs.size(); l++)  
		{
			ConnectionList[CpldConIs[l]]->initiateStep(x + ix, xd + ix, t);
			ix += 3;
		}
		
		
		// -------------------- do time stepping -----------------------

		// round to get appropriate mooring model time step
		int NdtM = ceil(dtC/dtM0);   // number of mooring model time steps per outer time step
		if (NdtM < 1)  
		{	
			// cout << "   Error: dtC is less than dtM.  (" << dtC << " < " << dtM0 << ")" << endl;
			return -1;
		}
		double dtM = dtC/NdtM;		// mooring model time step size (s)

		
		// loop through line integration time steps (integrate solution forward by dtC)
		for (int its = 0; its < NdtM; its++)
			rk2 (states, &t, dtM );  			// call RK2 time integrator (which calls the model)

		
		
		// -------------------- check for NaNs ---------------------------
		for (int i=0; i<nX; i++)
		{
			if (isnan(states[i]))
			{
				cout << "   Error: NaN value detected in MoorDyn state at time " << t << " s."<< endl;
				return -1;
			}
		}
		
		
		// --------------- check for line failures (detachments!) ----------------
		// step 1: check for time-triggered failures
		
		for (int l=0; l<nFails; l++)
		{
			if (FailList[l]->failStatus == 0)
			{
				if (t >= FailList[l]->failTime)
				{
					FailList[l]->failStatus = 1; // set status to failed so it's not checked again
					
					cout << "Failure number " << l+1 << " is happening!!!" << endl;
					
					detachLines(FailList[l]->attachID, FailList[l]->isRod, FailList[l]->lineIDs, FailList[l]->lineTops, FailList[l]->nLinesToDetach, t);
				}
			}
		}
		
		// step 2: check for tension-triggered failures (this will require specifying max tension things)
		
		
		
		// ------------------------ write outputs --------------------------
		if (AllOutput(t, dtC) < 0)
			return -1;
		
	}  // if dtC > 0 i.e. there will be a time step
		
		
		
	// ---------------- get reaction forces ------------------------
	//... from any coupled bodies, rods, and connections at the end of the time step, regardless of whether the time step occured (was nonzero)
	// >>>>> perhaps at some point it should be an option to return the mean forces, or even the curve over dt? <<<<<<<<<<<
	
	ix = 0;
	
	//double dummyMat3[3][3];
	//double dummyMat6[6][6];
	
	for (int l=0; l<CpldBodyIs.size(); l++)
	{
		BodyList[CpldBodyIs[l]]->getFnet(f + ix);
		ix += 6;
	}
	
	for (int l=0; l<CpldRodIs.size(); l++)  
	{
		RodList[CpldRodIs[l]]->getFnet(f + ix);
		
		if (RodList[CpldRodIs[l]]->type == -2)
			ix += 6;                                     // for cantilevered rods 6 entries will be taken
		else
			ix += 3;                                     // for pinned rods 3 entries will be taken
	}	

	for (int l=0; l<CpldConIs.size(); l++)  
	{
		ConnectionList[CpldConIs[l]]->getFnet(f + ix);
		ix += 3;
	}
		
		
	return 0;
}

/*
int DECLDIR FairleadsCalc2(double rFairIn[], double rdFairIn[], double fFairIn[], double* t_in, double *dt_in)
{
	for (int l=0; l < nFairs; l++)  
	{
		for (int j=0; j<3; j++)
		{
			rFairi [l][j] = rFairIn [l*3+j];
			rdFairi[l][j] = rdFairIn[l*3+j];
		}
	}
	
	int resultflag = FairleadsCalc(rFairi, rdFairi, Ffair, t_in, dt_in);
	
	for (int l=0; l < nFairs; l++)  
	{
		for (int j=0; j<3; j++)
		{
			fFairIn[l*3+j] = Ffair[l][j];
		}
	}
	
	return resultflag;  // returns -1 if an error
}



// This function now handles the assignment of fairlead boundary conditions, time stepping, and collection of resulting forces at fairleads
// It is called by the old LinesCalc function.  It can also be called externally for fairlead-centric coupling.
// It is now swithced with FairleadsCalc2 in that it expects 1D arrays.
// It now handles connections (x/y/z), pinned rods(x/y/z), and cantilevered rods (x/y/z/r/p/y), in the order of connections then rods
int DECLDIR FairleadsCalc(double **rFairIn, double **rdFairIn, double ** fFairIn, double* t_in, double *dt_in)
{
	double t =  *t_in;		// this is the current time
	double dtC =  *dt_in;	// this is the coupling time step
	
try {	
	if (dtC > 0) // if DT > 0, do simulation, otherwise leave passed fFairs unadjusted.
	{
	
		// send latest fairlead kinematics to fairlead objects
		for (int l=0; l < nFairs; l++)  
			ConnectionList[FairIs[l]]->initiateStep(rFairIn[l], rdFairIn[l], t);					
					
	>>>>>>> these functions need changing to treat platform(s) as bodies, couple with Rods too, 
	      and provide coupling function options for all <<<<<<<<<<<<<<<<<<<<<<
		  
		  // standard call sequence type 1: all bodies6
		  // standard call sequence type 2: all rods6, then all connects6?
		  // should there be a flag in the input file that clearly distingiushes the coupling type?
					
		// round to get appropriate mooring model time step
		int NdtM = ceil(dtC/dtM0);   // number of mooring model time steps per outer time step
		if (NdtM < 1)  
		{	
			// cout << "   Error: dtC is less than dtM.  (" << dtC << " < " << dtM0 << ")" << endl;
			return -1;
		}
		double dtM = dtC/NdtM;		// mooring model time step size (s)
		
		//if (wordy) cout << "In FairleadsCalc gonna run rk2 for " << NdtM << " times starting at " << t << " with dt of " << dtM << endl;
		
		
		// loop through line integration time steps (integrate solution forward by dtC)
		for (int its = 0; its < NdtM; its++)
			rk2 (states, &t, dtM );  			// call RK2 time integrator (which calls the model)

		
		// check for NaNs
		for (int i=0; i<nX; i++)
		{
			if (isnan(states[i]))
			{
				cout << "   Error: NaN value detected in MoorDyn state at time " << t << " s."<< endl;
				return -1;
			}
		}
		
			
		// go through connections to get fairlead forces		
		for (int l=0; l < nFairs; l++)
			ConnectionList[FairIs[l]]->getFnet(fFairIn[l]);

		if (AllOutput(t, dtC) < 0)   // write outputs
			return -1;
			
	}
}
catch(...) {
	cout << "error in FairleadsCalc" << endl;
}	
	return 0;
}
*/

int DECLDIR MoorDynClose(void)
{
	free(states);
	free(f0       );
	free(f1       );
	free(xt       );	
	
//	free2Darray(Ffair, nFairs);
//	free2Darray(rFairi, nFairs);
//	free2Darray(rdFairi, nFairs);
	free2Darray(FairTensLast, nLines);
	
	// close any open output files
	if (outfileMain.is_open())
		outfileMain.close();
	if (outfileLog.is_open())
		outfileLog.close();
	for (int l=0; l<nLines; l++) 
		if (outfiles[l])							// if not null
			if (outfiles[l]->is_open())
				outfiles[l]->close();
	
	// reset counters to zero!
//	nFairs  = 0;
//	nAnchs  = 0;
//	nConns  = 0;
		
		
	// delete created mooring system objects

	delete waves;


	for (int l = 0; l<nLineTypes; l++)
		delete LinePropList[l];
	delete[]   LinePropList;
	
	for (int l = 0; l<nRodTypes; l++)
		delete RodPropList[l];
	delete[]   RodPropList;
	
	for (int l = 0; l<nFails; l++)
		delete FailList[l];
	delete[]   FailList;

	delete GroundBody;

	for (int l = 0; l<nBodys; l++)
		delete BodyList[l];
	delete[]   BodyList;

	for (int l = 0; l<nRods; l++)
		delete RodList[l];
	delete[]   RodList;

	for (int l = 0; l<nConnections; l++)  // only delete the connections that have been created (excludes empty spots for extras)
		delete ConnectionList[l];
	delete[]   ConnectionList;

	for (int l = 0; l<nLines; l++)
		delete LineList[l];
	delete[]   LineList;

	// clear any global vectors
	//	FlinesS.clear();		
	//	rFairtS.clear();		
	//	rFairRel.clear();		
	//	rFairi.clear();		
	//	rdFairi.clear();		
		//    LinePropList.clear(); 	
	//	LineList.clear(); 		
	//	ConnectionList.clear();	
	//	FairIs.clear();  		
	//	ConnIs.clear();  
	LineStateIs.clear()   ; 
	ConnectStateIs.clear(); 
	RodStateIs.clear()    ; 
	BodyStateIs.clear()   ;  
	FreeBodyIs.clear()    ;  
	CpldBodyIs.clear()    ;  
	FreeRodIs.clear()     ;  
	CpldRodIs.clear()     ;  
	FreeConIs.clear()     ;  
	CpldConIs.clear()     ;  


	
	outfiles.clear(); 		
	outChans.clear();		
	LineStateIs.clear();
	
	cout << "   MoorDyn closed." << endl;

#ifndef OSX	
#ifndef LINUX	
	if (OwnConsoleWindow == 1)  {
		cout << "press enter to close: " << endl;
		cin.get();
		FreeConsole();  //_close(hConHandle); // close console window if we made our own.
	}
#endif
#endif

	return 0;
}


// Initializes internal arrays for holding wave kinematics passed in externally each time step, returns number of points required
int DECLDIR externalWaveKinInit()
{
	
	// count required number of wave kinematics input points
	npW = 0;
	
	//npW += 3*FreeBodyIs.size();
	//
	//for (int l=0; l<FreeRodIs.size(); l++)  
	//	npW += 3*RodList[FreeRodIs[l]]->getN() + 3;
	//		
	//for (int l=0; l<FreeConIs.size(); l++)  
	//	npW += 3
	
	for (int l=0; l < nLines; l++) 	
		npW += LineList[l]->getN() + 1;
	
	// allocate arrays to hold data that could be passed in
	U_1      = make1Darray(3*npW);
	Ud_1     = make1Darray(3*npW);
	U_2      = make1Darray(3*npW);
	Ud_2     = make1Darray(3*npW);
	U_extrap = make1Darray(3*npW);
	
	// initialize with zeros for safety
	tW_2 = 0.0;
	tW_1 = 0.0;		
	for (int i=0; i<3*npW; i++)
	{   U_2[ i] = 0.0;
		Ud_2[i] = 0.0;
		U_1[ i] = 0.0;
		Ud_1[i] = 0.0;
	}
	
	// return nwP so the calling program knows what size of arrays to send
	return npW;
}

// returns array providing coordinates of all points that will be receiving wave kinematics
void DECLDIR getWaveKinCoordinates(double r_out[])
{
	int i = 0;
		
	//i += 3*FreeBodyIs.size();
	//
	//for (int l=0; l<FreeRodIs.size(); l++)  
	//	i += 3*RodList[FreeRodIs[l]]->getN() + 3;
	//		
	//for (int l=0; l<FreeConIs.size(); l++)  
	//	i += 3
	
	for (int l=0; l < nLines; l++) 	
	{	LineList[l]->getNodeCoordinates(r_out + 3*i);
		i += LineList[l]->getN() + 1;
	}
	
	return;
}

// receives arrays containing U and Ud for each point at which wave kinematics will be applied (and the time they are calculated at)
void DECLDIR setWaveKin(double U_in[], double Ud_in[], double t_in)
{
	// set time stamp
	tW_2 = tW_1;  // first save the old one
	tW_1 = t_in;  // then store the new one
		
	for (int i=0; i<3*npW; i++)
	{	U_2[ i] =  U_1[ i];  // first save the old ones
		Ud_2[i] =  Ud_1[i];
		U_1[ i] = U_in[ i];  // then store the new ones
		Ud_1[i] = Ud_in[i];
	}
	
	return;
}


double DECLDIR GetFairTen(int l)
{
	// output LINE fairlead (top end) tensions
	if ((l > 0) && (l <= nLines))
		return LineList[l-1]->getNodeTen(LineList[l-1]->getN());  // fixed the index to adjust to 0 start on March 2!
	else
		return -1;
}



int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] )
{
	// function for providing FASTv7 customary line tension quantities.  Each array is expected as length nLines
	
	for (int l=0; l< *numLines; l++)
		LineList[l]->getFASTtens( &FairHTen[l], &FairVTen[l], &AnchHTen[l], &AnchVTen[l] );		
	
	return 0;
}

int DECLDIR GetConnectPos(int l, double pos[3])
{
	if ((l > 0) && (l <= nConnections))
	{
		vector< double > rs(3);
		vector< double > rds(3);
		ConnectionList[l-1]->getConnectState(rs, rds);
		for (int i=0; i<3; i++)
			pos[i] = rs[i];
		return 0;
	}
	else
		return -1;
}

int DECLDIR GetConnectForce(int l, double force[3])
{
	if ((l > 0) && (l <= nConnections))
	{
		ConnectionList[l-1]->getFnet(force);
		return 0;
	}
	else
		return -1;
}


int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3])
{
	// output LINE fairlead (top end) tensions
	if ((LineNum > 0) && (LineNum <= nLines))
	{
		int worked = LineList[LineNum]->getNodePos(NodeNum, pos);	// call line member function to fill in coordinates of the node of interest 
		if (worked >= 0)
			return 0;  // success
	}
	return -1;		// otherwise indicate error (invalid node and line number comination)
}


int DECLDIR DrawWithGL()
{
#ifdef USEGL
	// draw the mooring system with OpenGL commands (assuming a GL context has been created by the calling program)
	for (int l=0; l< nLines; l++)
		LineList[l]->drawGL2();  
	for (int l=0; l< nConnections; l++)
		ConnectionList[l]->drawGL();  
	return 0;
#endif
}

