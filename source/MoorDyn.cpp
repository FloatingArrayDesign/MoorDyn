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

 // This is version 2.a0 (version 2, alpha 0).  Mar. 8, 2018.
 
#include "Misc.h"
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
vector<double> FlinesS;					// net line force vector (6-DOF) - retains last solution for use when inputted dt is zero (such as during FAST predictor steps) when the model does not time step
vector< vector< double > > rFairtS;		// fairlead locations in turbine/platform coordinates
vector< vector< double > > rFairRel;		// fairlead locations relative to platform ref point but in inertial orientation
double** rFairi;			// fairlead locations in inertial reference frame
double** rdFairi;		// fairlead velocities in inertial reference frame

// static vectors to hold line and connection objects
LineProps** LinePropList;         // array of pointers to hold line library types
RodProps** RodPropList;           // array of pointers to hold rod library types

Line** LineList;                 // array of pointers to line objects
Rod** RodList;                   // array of pointers to Rod objects
Body** BodyList;			        // array of pointers to connection objects (line joints or ends)
Connection** ConnectionList;      // array of pointers to connection objects (line joints or ends)
//vector< Line > LineList;                 // line objects
//vector< Rod > RodList;                   // Rod objects
//vector< Body > BodyList;			        // connection objects (line joints or ends)
//vector< Connection > ConnectionList;      // connection objects (line joints or ends)


vector< int > RodType2Is;     // array of indices of which of RodList are actually independent Rods
vector< int > LineStateIs;    // array of starting indices for Lines in "states" array
vector< int > ConnectStateIs; // array of starting indices for indendent Connections in "states" array
vector< int > RodStateIs;     // array of starting indices for independent Rods in "states" array
vector< int > BodyStateIs;    // array of starting indices for Bodies in "states" array
vector< int > FairIs;  					// vector of fairlead connection indices in ConnectionList vector
vector< int > ConnIs;  					// vector of connect connection indices in ConnectionList vector
vector< int > AnchIs;  					// vector of anchor connection indices in ConnectionList vector

int nLineTypes = 0; // number of line types
int nRodTypes = 0; // number of Rod types
int nLines = 0;							// number of Line objects
int nRods = 0;							// number of Rod objects
int nBodys = 0;							// number of Body objects
int nConnections=0; 						// total number of Connection objects
int nFairs  = 0;							// number of fairlead connections
int nAnchs  = 0;							// number of anchor connections
int nConns  = 0;							// number of "connect" connections


EnvCond env; 							// struct of general environmental parameters
//vector< shared_ptr< ifstream > > infiles; 	//
vector< shared_ptr< ofstream > > outfiles; 	// a vector to hold ofstreams for each line
ofstream outfileMain;					// main output file
vector< OutChanProps > outChans;		// list of structs describing selected output channels for main out file
const char* UnitList[] = {"(s)     ", "(m)     ", "(m)     ", "(m)     ", 
                          "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
					 "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
					 "(N)     ", "(N)     "};   // list of units for each of the QTypes (see misc.h)


// state vector and stuff
// state vector is organized in order of input file:
//    [ bodies, rods, connects, lines [future dynamic additions?]].
//    Remember that we need to be able to add extra connects on the end for line failures.
double* states; 						// pointer to array comprising global state vector
int nX; 								// size of state vector array
double* xt; 							// more state vector things for rk2/rk4 integration 
double* f0;
double* f1;
//double* f2;
//double* f3;

double** Ffair;	// pointer to 2-d array holding fairlead forces

//double dt; // FAST time step
double dtM0; // desired mooring line model time step   

double dtOut = 0;  // (s) desired output interval (the default zero value provides output at every call to MoorDyn)

// new temporary additions for waves
vector< floatC > zetaCglobal;
double dwW;


// new globals for creating output console window when needed
int hConHandle;
intptr_t lStdHandle;   //long lStdHandle;

char const* PromptPtr;  // pointer to be made to environment variable PROMPT
int OwnConsoleWindow = 0;	

#ifdef LINUX	// any differences from built-in in mingw?  what about on OSX?
// int isnan(double x) { return x != x; } 	// changed to lower case.  will this still work?  Apparently some compiler optimizations can ruin this method
#define isnan(x) std::isnan(x)     // contributed by Yi-Hsiang Yu at NREL
#endif

// master function to handle time stepping (updated in v1.0.1 to follow MoorDyn F)
void RHSmaster( const double X[],  double Xd[], const double t, const double dt)
{

try
{
	// extrapolate instantaneous fairlead positions
	for (int l=0; l<nFairs; l++)  
		ConnectionList[FairIs[l]]->updateFairlead( t ); 
	
	// give Bodies latest state variables (kinematics will also be assigned to dependent connections and rods, and thus line ends)
	for (int l=0; l<nBodys; l++)  
		BodyList[l]->setState((X + BodyStateIs[l]), t);
	
	// give independent Rods latest state variables (kinematics will also be assigned to attached line ends)
	for (int l=0; l<RodType2Is.size(); l++)  
		RodList[RodType2Is[l]]->setState((X + RodStateIs[l]), t);
	
	// give Connects (independent connectoins) latest state variable values (kinematics will also be assigned to attached line ends)
	for (int l=0; l<ConnIs.size(); l++)  
		ConnectionList[ConnIs[l]]->setState((X + ConnectStateIs[l]), t);
	
	// calculate line dynamics (and calculate line forces and masses attributed to connections)
	for (int l=0; l < nLines; l++) 	
		LineList[l]->setState((X + LineStateIs[l]), t);
	for (int l=0; l < nLines; l++) 	
		LineList[l]->getStateDeriv((Xd + LineStateIs[l]), dt);
			
	// calculate connect dynamics (including contributions from attached lines
	// as well as hydrodynamic forces etc. on connect object itself if applicable)
	for (int l=0; l<ConnIs.size(); l++)  
		ConnectionList[ConnIs[l]]->getStateDeriv((Xd + ConnectStateIs[l]));
		
	// calculate dynamics of independent Rods 
	for (int l=0; l<RodType2Is.size(); l++)  
		RodList[RodType2Is[l]]->getStateDeriv((Xd + RodStateIs[l]));
	
	// calculate dynamics of Bodies
	for (int l=0; l<nBodys; l++)  
		BodyList[l]->getStateDeriv((Xd + BodyStateIs[l]));

}
catch(string e) {
    cout << "Error found! " << e << endl;
	throw string("problem!");
}
	
	return;
}


// Runge-Kutta 2 integration routine  (integrates states and time)
void rk2 (double x0[], double *t0, const double dt )
{
	RHSmaster(x0, f0, *t0, dt);                             // get derivatives at t0.      f0 = f ( t0, x0 );

	for (int i=0; i<nX; i++) 
		xt[i] = x0[i] + 0.5*dt*f0[i];  						// integrate to t0  + dt/2.        x1 = x0 + dt*f0/2.0;
	
	RHSmaster(xt, f1, *t0 + 0.5*dt, dt);                    // get derivatives at t0  + dt/2.	f1 = f ( t1, x1 );

	for (int i=0; i<nX; i++) 
		x0[i] = x0[i] + dt*f1[i]; 							// integrate states to t0 + dt
		
	*t0 = *t0 + dt;										// update time
		
	return;
}


double GetOutput(OutChanProps outChan)
{
	if (outChan.OType == 1)   // line type
		return LineList[outChan.ObjID-1]->GetLineOutput(outChan);
	else if (outChan.OType == 2)   // connection type
		return ConnectionList[outChan.ObjID-1]->GetConnectionOutput(outChan);
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
	
	return 0;
}


// initialization function for platform-centric coupling
int DECLDIR LinesInit(double X[], double XD[])
{	
	int value = MoorDynInit(X, XD, 0);	
	return value;
}

// initialization function for fairlead-based coupling
int DECLDIR FairleadsInit(double X[], double XD[])
{
	int value = MoorDynInit(X, XD, 1);	
	return value;
}

// initialization function
int MoorDynInit(double X[], double XD[], int FairleadCoupling)
{	
	// expects 6 dof platform position and velocity arrays if FairleadCoupling == 0
	// expects 3*nfair arrays of each fairlead's 3D position and velocity if FairleadCoupling == 1

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
	
	// ---------------------------- MoorDyn title message ----------------------------
	cout << "\n Running MoorDyn (v2.a0, 2019-04-21)\n   Copyright (c) Matt Hall, licensed under GPL v3.\n";
	cout << "\n This is an alpha version, intended for debugging.\n";

	//dt = *dTime; // store time step from FAST	
	

	// calculate TransMat
	double TransMat[9];
	RotMat(X[3], X[4], X[5], TransMat);
		
	
	// ==================== load data about the mooring lines from lines.txt =====================
	
	// --------------------------------- read data from file -----------------------------
	vector<string> lines;
	string line;
	ifstream myfile ("Mooring/lines.txt");     // open an input stream to the line data input file
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
	{	cout << "Error: unable to open lines.txt file" << endl; 
		return -1;
	}
	
	// ----------------------- initialize data holders -------------------------
	
	// defaults
	env.g = 9.8;
	env.WtrDpth = 0.;
	env.rho_w = 1025.;
	env.kb = 3.0e6;
	env.cb = 3.0e5;
	env.WaveKin = 0;   // 0=none, 1=from function, 2=from file
	env.WriteUnits = 1;	// by default, write units line
	env.FrictionCoefficient = 0.0;
	env.FricDamp = 200.0;
	env.StatDynFricScale = 1.0;
		
	double ICDfac = 5; // factor by which to boost drag coefficients during dynamic relaxation IC generation
	double ICdt = 1.0;						// convergence analysis time step for IC generation
	double ICTmax = 120;						// max time for IC generation
	double ICthresh = 0.001;					// threshold for relative change in tensions to call it converged
	
	dtM0 = 0.001;  // default value for desired mooring model time step

	// fairlead and anchor position arrays
	vector< vector< double > > rFairt;
	vector< string > outchannels;  // string containing which channels to write to output
	
	// line connection info (temporary, until LineList addresses are done moving) <<????
//	vector< int > LineInd;
//	vector< int > RodInd;
//	vector< int > AnchInd;
//	vector< int > FairInd;

	
	nX = 0; // Make sure the state vector counter starts at zero.
	        // This will be incremented as each object is added.
		
	
	// ------------------------- process file contents -----------------------------------
	
	int i=0; // file line number
	
	while (i < lines.size())  
	{
		if (lines[i].find("---") != string::npos) // look for header line
		{
			if ( (lines[i].find("LINE DICTIONARY") != string::npos) || (lines[i].find("LINE TYPES") != string::npos) ) // if line dictionary header
			{	
				if (wordy>0) cout << "   Reading line types: ";
				
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nLineTypes++;
					i++;
				}
				
				// allocate space for array of pointers to entries
				LinePropList = new LineProps*[nLineTypes];
				
				// set up each entry
				for (int iLineType=0; iLineType<nLineTypes; iLineType++)
				{
					std::vector<std::string> entries = split(lines[i-nLineTypes+iLineType], ' '); // split by spaces
					
					if (entries.size() >= 7) // if valid number of inputs
					{
						LinePropList[iLineType] = new LineProps();
						
						LinePropList[iLineType]->type =    entries[0]; //.c_str());
						LinePropList[iLineType]->d  = atof(entries[1].c_str());
						LinePropList[iLineType]->w  = atof(entries[2].c_str());
						LinePropList[iLineType]->Can= atof(entries[5].c_str());
						LinePropList[iLineType]->Cat= atof(entries[6].c_str());
						LinePropList[iLineType]->Cdn= atof(entries[7].c_str());
						LinePropList[iLineType]->Cdt= atof(entries[8].c_str());
						
						// read in stiffness value (and load nonlinear file if needed)
						if (strpbrk(entries[3].c_str(), "abcdfghijklmnopqrstuvwxyzABCDFGHIJKLMNOPQRSTUVWXYZ") == NULL) // "eE" are exluded as they're used for scientific notation!
						{
							if (wordy > 0) cout << "found NO letter in the line EA value so treating it as a number." << endl;
							LinePropList[iLineType]->EA = atof(entries[3].c_str());
							LinePropList[iLineType]->nEpoints = 0;
						}	
						else // otherwise interpet the input as a file name to load stress-strain lookup data from
						{
							if (wordy > 0) cout << "found A letter in the line EA value so will try to load the filename." << endl;
							LinePropList[iLineType]->EA = 0.0;
							
							// load lookup table data from file
							vector<string> Elines;
							string Eline;
							char Efilename[256];
							snprintf(Efilename, sizeof Efilename, "%s%s", "Mooring/", entries[3].c_str());
  
							ifstream myfile (Efilename);
							if (myfile.is_open())
							{
								while ( myfile.good() )
								{
									getline (myfile,Eline);
									Elines.push_back(Eline);
								}
								myfile.close();
							}
							else 
							{	cout << "Error: unable to open " << Efilename << endl; 
								return -1;
							}
							
							// now process data	
							int nE = 0; // counter for number of data points in lookup table
							for (int I=2; I<Elines.size(); I++)   // skip first three lines (title, names, and units) then parse
							{
								std::vector<std::string> Eentries = split(Elines[I], ' '); // what about TABS rather than spaces???
								if (Eentries.size() >= 2) // if valid number of inputs
								{
									LinePropList[iLineType]->stiffXs[nE]  = atof(Eentries[0].c_str());
									LinePropList[iLineType]->stiffYs[nE]  = atof(Eentries[0].c_str());
									nE++;
								}
								else
								{	cout << "Error: failed to find two columns somewhere within " << Efilename << endl; 
									return -1;
								}
							}		
							LinePropList[iLineType]->nEpoints = nE;
						}
												
						// read in damping value (and load nonlinear file if needed)
						if (strpbrk(entries[4].c_str(), "abcdfghijklmnopqrstuvwxyzABCDFGHIJKLMNOPQRSTUVWXYZ") == NULL) // "eE" are exluded as they're used for scientific notation!
						{
							if (wordy > 0) cout << "found NO letter in the line cInt value so treating it as a number." << endl;
							LinePropList[iLineType]->c = atof(entries[4].c_str());
							LinePropList[iLineType]->nCpoints = 0;
						}	
						else // otherwise interpet the input as a file name to load stress-strain lookup data from
						{
							if (wordy > 0) cout << "found A letter in the line cInt value so will try to load the filename." << endl;
							LinePropList[iLineType]->c = 0.0;
							
							// load lookup table data from file
							vector<string> Clines;
							string Cline;
							char Cfilename[256];
							snprintf(Cfilename, sizeof Cfilename, "%s%s", "Mooring/", entries[4].c_str());
  
							ifstream myfile (Cfilename);
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
							{	cout << "Error: unable to open " << Cfilename << endl; 
								return -1;
							}
							
							// now process data	
							int nC = 0; // counter for number of data points in lookup table
							for (int I=2; I<Clines.size(); I++)   // skip first three lines (title, names, and units) then parse
							{
								std::vector<std::string> Centries = split(Clines[I], ' '); // what about TABS rather than spaces???
								if (Centries.size() >= 2) // if valid number of inputs
								{
									LinePropList[iLineType]->dampXs[nC]  = atof(Centries[0].c_str());
									LinePropList[iLineType]->dampYs[nC]  = atof(Centries[0].c_str());
									nC++;
								}
								else
								{	cout << "Error: failed to find two columns somewhere within " << Cfilename << endl; 
									return -1;
								}
							}		
							LinePropList[iLineType]->nCpoints = nC;
						}
							
						
						if (wordy>0)  cout << entries[0] << " ";
					}
					//i++;
				}
				if (wordy>0) cout << "\n";
			}
			else if ( (lines[i].find("ROD DICTIONARY") != string::npos) || (lines[i].find("ROD TYPES") != string::npos) ) // if rod dictionary header
			{	
				if (wordy>0) cout << "   Reading rod types: ";
				
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nRodTypes++;
					i++;
				}
				
				// allocate space for array of pointers to entries
				RodPropList = new RodProps*[nRodTypes];
				
				// set up each entry
				for (int iRodType=0; iRodType<nRodTypes; iRodType++)
				{
					std::vector<std::string> entries = split(lines[i-nRodTypes+iRodType], ' '); // split by spaces
					
					if (wordy>0)  cout << " hlo ";
					
					if (entries.size() >= 7) // if valid number of inputs
					{	
						RodPropList[iRodType] = new RodProps();

						RodPropList[iRodType]->type =    entries[0]; //.c_str());
						RodPropList[iRodType]->d  = atof(entries[1].c_str());
						RodPropList[iRodType]->w  = atof(entries[2].c_str());
						RodPropList[iRodType]->Can= atof(entries[3].c_str());
						RodPropList[iRodType]->Cat= atof(entries[4].c_str());
						RodPropList[iRodType]->Cdn= atof(entries[5].c_str());
						RodPropList[iRodType]->Cdt= atof(entries[6].c_str());
						if (wordy>0)  cout << entries[0] << " ";
					}
					//i++;
				}
				if (wordy>0) cout << "\n";
			}	
			else if (lines[i].find("BODY LIST") != string::npos) 
			{	
				if (wordy>0) cout << "   Reading Body properties: ";
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nBodys++;
					i++;
				}
				
				// allocate space for array of pointers to entries and state indices
				BodyList = new Body*[nBodys];
				//BodyStateIs = (int*) malloc(nBodys*sizeof(int));
				
				// set up each entry
				for (int iBody=0; iBody<nBodys; iBody++)
				{ 	
					std::vector<std::string> entries = split(lines[i-nBodys+iBody], ' '); // split by spaces
					
					if (entries.size() >= 21) // if valid number of inputs
					{
						// Name/ID X0 Y0 Z0 r0 p0 y0 Xcg Ycg Zcg M  V  IX IY IZ CdA-x,y,z Ca-x,y,z
						
						int number = atoi(entries[0].c_str());
						
						double M = atof(entries[10].c_str());						
						double V = atof(entries[11].c_str());
						
						double r6[6];
						double rCG[3];
						double Inert[3];
						double CdA[3];
						double Ca[3];
						for (int I=0; I<3; I++) 
						{
							r6[  I] = atof(entries[1+I].c_str());
							r6[3+I] = atof(entries[4+I].c_str());							
							rCG[ I] = atof(entries[7+I].c_str());
							Inert[I]= atof(entries[12+I].c_str());
							CdA[ I] = atof(entries[15+I].c_str());
							Ca[  I] = atof(entries[18+I].c_str());							
						}	
					
						// set up Body 
						BodyList[iBody] = new Body(); 
						BodyList[iBody]->setup(number, r6, rCG, M, V, Inert, CdA, Ca);						
							
						BodyStateIs.push_back(nX);  // assign start index of this Body's states
						nX += 12;                   // add 12 state variables for each Body						
							
						//if (BodyList.size() != number)  // check that ID numbers are in order
						//	cout << "Warning: body ID numbers should be in order (1,2,3...)." << endl;
						
					}
					else 
					{
						cout << endl << "   Error with Body " << entries[0] << " inputs (" << entries.size() << " is not enough)." << endl;
						return -1;
					}
					//i++;
				}		
				if (wordy>0) cout << "\n";		
			}
			else if (lines[i].find("ROD LIST") != string::npos) // if rod properties header
			{	
				if (wordy>0) cout << "   Reading rod properties: ";
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nRods++;
					i++;
				}
				
				// allocate space for array of pointers to entries and state indices
				RodList = new Rod*[nRods];
				//RodStateIs = (int*) malloc(nRods*sizeof(int));  // <<< this isn't quite right cuz not all have states
				
				// set up each entry
				for (int iRod=0; iRod<nRods; iRod++)
				{ 	
					std::vector<std::string> entries = split(lines[i-nRods+iRod], ' '); // split by spaces
					
					if (entries.size() >= 8) // if valid number of inputs
					{
						// BodyToAddTo RodID  RodType  Diam    MassDenInAir   Can     Cat    Cdn     Cdt 
						
						int bodyID          = atoi(entries[0].c_str());						
						int number          = atoi(entries[1].c_str());
						string type         = entries[2];
						double endCoords[6]; 
						for (int I=0; I<6; I++) endCoords[I]=atof(entries[3+I].c_str());
						int NumSegs         = atoi(entries[9].c_str());
						string outchannels  = entries[10];
						
						// find Rod properties index (look in the Rod dictionary)
						int TypeNum = -1;
						for (int J=0; J<nRodTypes; J++)  {
							if (RodPropList[J]->type.find(type) != string::npos)
								TypeNum = J;
						}
						
						if (TypeNum == -1)
							cout << "   Error: unable to identify type of Rod " << number << " (" << type << ") from those provided." << endl;
						
						if (wordy>1) cout << "rod " << number << " type " << type << " typenum " << TypeNum << endl;
						
						// make an output file for it
						if ((outchannels.size() > 0) && (strcspn( outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))  // if 1+ output flag chars are given and they're valid
						{	stringstream oname;
							oname << "Mooring/Rod" << number << ".out";
							outfiles.push_back( make_shared<ofstream>(oname.str()));
						}
						else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
					
						// find its parent Body (meaning it's a type 1 Rod) or whether it is independent (type 2 Rod)
						
						int rodType;
							
						if (bodyID==0)  // independent rod case (no parent body)
							rodType = 2;
						else if ((bodyID < nBodys) && (bodyID > 0))  // dependent rod case (attached to body)
							rodType = 1;
						else
						{
							cout << "   Error: Invalid body ID (" << bodyID << ") given for Rod " << number << endl;
							return -1;
						}
						
						// set up Rod 						
						RodList[iRod] = new Rod(); 
						RodList[iRod]->setup(rodType, number, RodPropList[TypeNum], endCoords, 
							NumSegs, outfiles.back(), outchannels);							
							
						//if (RodList.size() != number)  // check that ID numbers are in order
						//	cout << "Warning: rod ID numbers should be in order (1,2,3...)." << endl;
						
							
						// pass parent Body the Rod address	and the relative position of the Rod ends
						if (rodType==1)
							BodyList[bodyID-1]->addRodToBody(RodList[iRod], endCoords);
						else 
						{    // for an independent rod, states need to be added
							RodStateIs.push_back(nX);    // assign start index of this independent Rod's states
							nX += 12;                   // add 12 state variables for each independent Rod
							RodType2Is.push_back(iRod);     // add this Rod's index to the list of independent Rod indices
						}		
									
						if (wordy>0) cout << number << " ";									
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
			
			
			else if ( (lines[i].find("CONNECTION PROPERTIES") != string::npos) || (lines[i].find("NODE PROPERTIES") != string::npos) ) // if node properties header
			{	
				if (nLineTypes < 1)
					cout << "   Error: began reading connection inputs before reading any line type inputs." << endl;
			
				if (wordy>0) cout << "   Reading connection properties: ";
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nConnections++;
					i++;
				}
				
				// allocate space for array of pointers to entries and state indices
				ConnectionList = new Connection*[nConnections];
				// ConnectStateIs = (int*) malloc(nConnections*sizeof(int));  // <<< this isn't quite right cuz not all have states
				
				// set up each entry
				for (int iConnection=0; iConnection<nConnections; iConnection++)
				{ 	
					std::vector<std::string> entries = split(lines[i-nConnections+iConnection], ' '); // split by spaces
					
					// Node  Type/attachID    X  Y   Z  M  V  [optional:FX FY FZ] CdA Ca
					
					if (entries.size() >= 9) // if valid number of inputs
					{					
						int number=atoi(entries[0].c_str());
						
						// ----------- process connection type -----------------
						int type;						
						// substrings of grouped letters or numbers for processing each parameter                          
						char let1 [10]; char num1 [10]; char let2 [10]; char num2 [10]; char let3 [10]; 
						char typeWord[10];							// the buffer
												
						snprintf(typeWord, 10, entries[1].c_str());		// copy connection type word to buffer
						
						decomposeString(typeWord, let1, num1, let2, num2, let3); // divided outWord into letters and numbers
							
						if ((strcmp(let1, "ANCHOR") ==0) || (strcmp(let1, "FIXED") ==0) || (strcmp(let1, "FIX") ==0))
							type = 0;
						else if ((strcmp(let1, "FAIRLEAD") ==0) || (strcmp(let1, "VESSEL") ==0) || (strcmp(let1, "VES") ==0))
							type = 1;
						else if ((strcmp(let1, "CONNECT") ==0) || (strcmp(let1, "CON") ==0))
							type = 2;
						else if (strcmp(let1, "BODY") ==0)
							type = 3;
						else {
							cout << "   Error: could not recognise type of connection " << number << "." << endl;
							return -1;
						}
						
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
						
						// make default water depth at least the depth of the lowest node (so water depth input is optional)
						if (r0[2] < -env.WtrDpth)  env.WtrDpth = -r0[2];
					
						// now make Connection object!
						ConnectionList[iConnection] = new Connection();
						ConnectionList[iConnection]->setup(number, type, r0, M, V, F, CdA, Ca);
						
						//if (ConnectionList.size() != number)  // check that ID numbers are in order
						//	cout << "Warning: connect ID numbers should be in order (1,2,3...)." << endl;
												
						// count and add to the appropriate list
						if (type==0)      // if an anchor
						{	AnchIs.push_back(iConnection);
							nAnchs ++;
						}
						else if (type==1)   // if a fairlead, add to list
						{	rFairt.push_back(vector<double>(3, 0.0));		// fairlead location in turbine ref frame
							rFairt.back().at(0) = atof(entries[2].c_str()); 	// x
							rFairt.back().at(1) = atof(entries[3].c_str()); 	// y
							rFairt.back().at(2) = atof(entries[4].c_str()); 	// z	
							FairIs.push_back(iConnection);			// index of fairlead in ConnectionList vector
							nFairs ++;
						}
						else if (type==2)   // if a connect, add to list and add states for it
						{	ConnIs.push_back(iConnection);	
							nConns ++;
							
							ConnectStateIs.push_back(nX);   // assign start index of this connect's states
							nX += 6;                       // add 6 state variables for each connect
						}
						else if (type==3) // if attached to a body, figure out which body and assign to it
						{
							if (strlen(num1)>0)
							{
								int bodyID = atoi(num1);
								if ((bodyID <= nBodys) && (bodyID > 0))
								{
									BodyList[bodyID-1]->addConnectionToBody(ConnectionList[iConnection], r0);
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
						else
							cout << "   Error: type of connection " << number << " is unknown." << endl;
						
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
			else if (lines[i].find("LINE PROPERTIES") != string::npos) // if line properties header
			{	
				if (wordy>0) cout << "   Reading line properties: ";
				i += 3; // skip following two lines (label line and unit line)
				
				// find how many elements of this type there are
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					nLines++;
					i++;
				}
				
				// allocate space for array of pointers to entries and state indices
				LineList = new Line*[nLines];
				//LineStateIs = (int*) malloc(nLines*sizeof(int));  
				
				// set up each entry
				for (int iLine=0; iLine<nLines; iLine++)
				{ 	
					std::vector<std::string> entries = split(lines[i-nLines+iLine], ' '); // split by spaces
						
					if (entries.size() >= 7) // if valid number of inputs
					{
						// Line     LineType  UnstrLen   NodeAnch  NodeFair  Flags/Outputs
						
																		
						int number= atoi(entries[0].c_str());
						string type     = entries[1];
						double UnstrLen = atof(entries[2].c_str());
						int NumSegs    = atoi(entries[3].c_str()); // addition vs. MAP
						//string anchID    = entries[4];
						//string fairID    = entries[5];
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
							oname << "Mooring/Line" << number << ".out";
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
						
						snprintf(outWord, 10, entries[4].c_str());		// copy anchor connection word to buffer
						
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
						
						snprintf(outWord, 10, entries[5].c_str());		// copy fairlead connection word to buffer
						
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
						
					//	// store connection info to apply later (once lines are all created)
					//	LineInd.push_back(LineList.size()-1);
					//	AnchInd.push_back(AnchIndex);
					//	FairInd.push_back(FairIndex);
											
																				
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
			else if (lines[i].find("SOLVER OPTIONS") != string::npos) // if solver options header
			{	
				i ++;
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' ');
					
					if (entries.size() >= 2) // if a valid "[i] [j] C[i][j] [optional comment]" format
					{
//						if (entries[1] == "NumNodes")       nNodes = atoi(entries[0].c_str());
						if ((entries[1] == "dtM")           || (entries[1] == "DT"))        dtM0 = atof(entries[0].c_str());     // second is old way, should phase out
						//else if (entries[1] == "DWWave")    dw_in = atof(entries[0].c_str());
						else if ((entries[1] == "kBot")     || (entries[1] == "kb"))        env.kb = atof(entries[0].c_str());   // "
						else if ((entries[1] == "cBot")     || (entries[1] == "cb"))        env.cb = atof(entries[0].c_str());   // "
						else if (entries[1] == "WtrDpth")                                   env.WtrDpth = atof(entries[0].c_str()); 
						else if ((entries[1] == "CdScaleIC")|| (entries[1] == "ICDfac"))    ICDfac   = atof(entries[0].c_str()); // "
						else if ((entries[1] == "dtIC")     || (entries[1] == "ICdt"))      ICdt     = atof(entries[0].c_str()); // "
						else if ((entries[1] == "TmaxIC")   || (entries[1] == "ICTmax"))    ICTmax   = atof(entries[0].c_str()); // "
						else if ((entries[1] == "threshIC") || (entries[1] == "ICthresh"))  ICthresh = atof(entries[0].c_str()); // "
						else if (entries[1] == "WaveKin")                                   env.WaveKin = atoi(entries[0].c_str());
						else if (entries[1] == "WriteUnits")                                env.WriteUnits = atoi(entries[0].c_str());
						else if (entries[1] == "FrictionCoefficient")                       env.FrictionCoefficient = atof(entries[0].c_str());
						else if (entries[1] == "FricDamp")                       env.FricDamp = atof(entries[0].c_str());
						else if (entries[1] == "StatDynFricScale")             env.StatDynFricScale = atof(entries[0].c_str());
						else if (entries[1] == "dtOut")                                     dtOut = atof(entries[0].c_str()); // output writing period (0 for at every call)
					}
					i++;
				}
			}
			else if (lines[i].find("OUTPUT") != string::npos) // if output list header
			{	
				//cout << "in output section" << endl;
				i ++;
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' ');
					
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
						// fairlead tension case (updated)   NOTE - these will include contributions of ALL lines connected at these fairlead points
						if (strcmp(let1, "FAIRTEN")==0)
						{	
							//cout << "found fairten" << endl;
							dummy.OType = 2;             							// connection object type
							dummy.QType = Ten;           							// tension quantity type
							strncpy(dummy.Units, UnitList[Ten], UnitsSize); 							// set units according to QType
							int LineID = atoi(num1);               							// this is the line number
// >>>>							dummy.ObjID = LineList[LineID-1].FairConnect->number;		// get the connection ID of the fairlead
							dummy.NodeID = -1;           							// not used.    other%LineList(oID)%N  ! specify node N (fairlead)
						}
						// achor tension case
						else if (strcmp(let1, "ANCHTEN")==0) 
						{	
							//cout << "found anchtoen" << endl;dummy.OType = 2;             							// connection object type
							dummy.QType = Ten;           							// tension quantity type
							strncpy(dummy.Units, UnitList[Ten], UnitsSize);						// set units according to QType
							int LineID = atoi(num1);               							// this is the line number
// >>>>									dummy.ObjID = LineList[LineID-1].AnchConnect->number;		// get the connection ID of the anchor
							dummy.NodeID = -1;           							// not used.    other%LineList(oID)%N  ! specify node N (fairlead)
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
							// should do fairlead option also!
							
							else   // error
							{	//CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
								cout << "Warning: invalid output specifier: "  << let1 << ".  Type must be L or C/Con." << endl;
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
								//CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
								//CALL WrScr('Warning: invalid output specifier - quantity type not recognized.')  ! need to figure out how to add numbers/strings to these warning messages...
								//CONTINUE
							}

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

	//nConnections = ConnectionList.size();
	//nLines = LineList.size();
	//nRods = RodList.size();
	
	// <<<<<<<<< need to add bodys
			
	
/*
	//  ------------------------ set up waves if needed -------------------------------
// @mth: new approach to wave kinematics will be implemented - this part needs to be redone	
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
	for (int l=0; l<nBodys; l++)  
		BodyList[l]->setEnv( env); 
	for (int l=0; l<nRods; l++)  
		RodList[l]->setEnv( env); 
	for (int l=0; l<nConnections; l++)  
		ConnectionList[l]->setEnv( env); 
	for (int l=0; l<nLines; l++)  
		LineList[l]->setEnv( env); 

	
	
	// ----------------- allocate arrays ------------------
		
	// make state vector	
	if (wordy > 1) cout << "   Creating state vectors of size " << nX << endl;
	states    = (double*) malloc( nX*sizeof(double) );

	// make arrays for integration
	f0 = (double*) malloc( nX*sizeof(double) );
	f1 = (double*) malloc( nX*sizeof(double) );
	//f2 = (double*) malloc( nX*sizeof(double) );
	//f3 = (double*) malloc( nX*sizeof(double) );
	xt = (double*) malloc( nX*sizeof(double) );
	
	memset(states, 0.0, nX*sizeof(double));
	
	// make array used for passing fairlead kinematics and forces between fairlead- and platform-centric interface functions
	Ffair = make2Darray(nFairs, 3); 
	
	rFairi = make2Darray(nFairs, 3);
	rdFairi = make2Darray(nFairs, 3);
	
	// --------- Allocate/size some global, persistent vectors -------------

	nFairs = rFairt.size();
	
	FlinesS.resize(6);  // should clean up these var names
	rFairtS.resize (nFairs);
	rFairRel.resize(nFairs);
//	rFairi.resize  (nFairs);  // after applying platform DOF ICs, should eventually pass this rather than rFairt to Line.setup()
//	rdFairi.resize (nFairs);	
	

	for (unsigned int ii=0; ii<nFairs; ii++)
	{
		rFairtS[ii].resize(3);
		rFairRel[ii].resize(3);
//		rFairi[ii].resize(3);
//		rdFairi[ii].resize(3);
		
		rFairtS[ii][0] = rFairt[ii][0];	// store relative fairlead locations statically for internal use
		rFairtS[ii][1] = rFairt[ii][1];
		rFairtS[ii][2] = rFairt[ii][2];
		
	}
		
	// ------------------- initialize system, including trying catenary IC gen of Lines -------------------
	
	cout << "   Creating mooring system.  " << nFairs << " fairleads, " << nAnchs << " anchors, " << nConns << " connections." << endl;	
	//for (int l=0; l<nConnections; l++)  {
	//	ConnectionList[l].initialize( (states + 6*l), env, X, TransMat); // connections
	//}	
	
	// set positions of fairleads based on inputted platform position
	if (FairleadCoupling == 1)
	{
		for (int l=0; l<nFairs; l++)  
			ConnectionList[FairIs[l]]->initializeFairlead2( X + 3*l, XD + 3*l); // 
	}
	else
	{
		for (int l=0; l<nFairs; l++)  
			ConnectionList[FairIs[l]]->initializeFairlead( X, TransMat ); // 
	}
	
	// Go through Bodys and write the coordinates to the state vector
	for (int l=0; l<nBodys; l++)  
		BodyList[l]->initializeBody( states + BodyStateIs[l] ); 
	
	// Go through independent Rods and write the coordinates to the state vector
	for (int l=0; l<RodType2Is.size(); l++)  
		RodList[RodType2Is[l]]->initializeRod( states + RodStateIs[l] ); 
	 
	// Go through independent connections (Connects) and write the coordinates to 
	// the state vector and set positions of attached line ends
	for (int l=0; l<ConnIs.size(); l++)  
		ConnectionList[ConnIs[l]]->initializeConnect( states + ConnectStateIs[l]); //
	
	// Go through fixed (anchor) connections and ensure they set positions of attached line ends
	for (int l=0; l<AnchIs.size(); l++)  
		ConnectionList[AnchIs[l]]->initializeAnchor(); //
	
	// Lastly, go through lines and initialize internal node positions using quasi-static model
	for (int l=0; l<nLines; l++)  
		LineList[l]->initializeLine( states + LineStateIs[l] ); 
	
	
	
	// write t=-1 output line for troubleshooting preliminary ICs
	//AllOutput(-1.0);
	//cout << "outputting ICs for troubleshooting" << endl;
	
	
	// ------------------ do dynamic relaxation IC gen --------------------
	
	cout << "   Finalizing ICs using dynamic relaxation (" << ICDfac << "X normal drag)" << endl;
	
	// boost drag coefficients to speed static equilibrium convergence
	for (int l=0; l < nLines; l++)       LineList[l]->scaleDrag(ICDfac); 	
	for (int l=0; l < nConnections; l++) ConnectionList[l]->scaleDrag(ICDfac);
	for (int l=0; l < nRods; l++)        RodList[l]->scaleDrag(ICDfac);
	for (int l=0; l < nBodys; l++)      BodyList[l]->scaleDrag(ICDfac);
	
	int niic = round(ICTmax/ICdt);			// max number of IC gen time steps
	
	double Ffair[3];						// array to temporarily store fairlead force components
	vector< double > Tensions(nFairs*3*niic, 0.0); // vector to store tensions for analyzing convergence
	vector< double > FairTens(nFairs, 0.0); 		// vector to store tensions for analyzing convergence
	vector< double > FairTensLast(nFairs, 0.0); 	// vector to store tensions for analyzing convergence
	vector< double > FairTensLast2(nFairs, 0.0); 	// vector to store tensions for analyzing convergence
	int lf; 								// fairlead index
	
	
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
		for (lf=0; lf<nFairs; lf++) {
			FairTensLast2[lf] = FairTensLast[lf];
			FairTensLast[lf] = FairTens[lf];
		}
	
		// go through connections to get fairlead forces
		for (lf=0; lf<nFairs; lf++) {
			ConnectionList[FairIs[lf]]->getFnet(Ffair);
			FairTens[lf] = 0.0;
			for (int j=0; j<3; j++) FairTens[lf] += Ffair[j]*Ffair[j];
			FairTens[lf] = sqrt(FairTens[lf]);
		}
				
	//	cout << "size of FairIs is " << FairIs.size() << endl;
	//	cout << "FairIs 1,2 are " << FairIs[0] << FairIs[1] << endl;
	//	cout << "size of ConnectionList is " << ConnectionList.size() << endl;
	//	cout << " nFairs is " << nFairs << endl;
		
				
		cout << "    t = " << t << " s, tension at first fairlead is " << FairTens[0] << " N    \r";   // write status update and send cursor back to start of line

		// check for convergence (compare current tension at each fairlead with previous two values)
		if (iic > 2)
		{
			for (lf=0; lf<nFairs; lf++) {
				if (( abs( FairTens[lf]/FairTensLast[lf] - 1.0 ) > ICthresh ) || ( abs( FairTens[lf]/FairTensLast2[lf] - 1.0 ) > ICthresh ) )
					break;				
			}
			
			if (lf == nFairs) {  // if we made it with all cases satisfying the threshold
				cout << "   Fairlead tensions converged to " << 100.0*ICthresh << "\% after " << t << " seconds.        " << endl;
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
	

// @mth: new approach to be implemented
//	// ------------------------- calculate wave time series if needed -------------------
//	if (env.WaveKin == 2)
//	{
//		for (int l=0; l<nLines; l++) 
//			LineList[l]->makeWaveKinematics( 0.0 );
//	}

	
	// -------------------------- start main output file --------------------------------
	outfileMain.open("Mooring/Lines.out");
	if (outfileMain.is_open())
	{
		// --- channel titles ---
		outfileMain << "Time" << "\t "; 	
		// output all LINE fairlead (top end) tensions
		for (lf=0; lf<outChans.size(); lf++) outfileMain << outChans[lf].Name << "\t ";
		outfileMain << "\n";
		
		if (env.WriteUnits > 0)
		{
			// --- units ---
			outfileMain << "(s)" << "\t "; 	
			// output all LINE fairlead (top end) tensions
			for (lf=0; lf<outChans.size(); lf++) outfileMain << outChans[lf].Units << "\t ";
			outfileMain << "\n";
		}
	}
	else 
	{	cout << "   ERROR: Unable to write to main output file " << endl;  //TODO: handle error
		return -1;
	}
	
	// write t=0 output line
	if (AllOutput(0.0, 0.0) < 0)
		return -1;
	
						
	cout <<endl;
	
	return 0;
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




// This is the original time stepping function, for platform-centric coupling.
int DECLDIR LinesCalc(double X[], double XD[], double Flines[], double* t_in, double* dt_in) 
{
	     // From FAST: The primary output of this routine is array Flines(:), which must
         // contain the 3 components of the total force from all mooring lines
         // (in N) acting at the platform reference and the 3 components of the
         // total moment from all mooring lines (in N-m) acting at the platform
         // reference; positive forces are in the direction of positive
         // platform displacement.  This primary output effects the overall
         // dynamic response of the system.  However, this routine must also
         // compute:
         //   Array FairHTen(:)   - Effective horizontal tension at the fairlead of each mooring line
         //   Array FairVTen(:)   - Effective vertical   tension at the fairlead of each mooring line
         //   Array AnchHTen(:)   - Effective horizontal tension at the anchor   of each mooring line
         //   Array AnchVTen(:)   - Effective vertical   tension at the anchor   of each mooring line
	    // BUT NOT THE FOLLOWING (I've removed them for simplicity - MH)
         //   Array NodesTen(:,:) - Effective line tensions              at each node of each line
         //   Array Nodesxi (:,:) - xi-coordinates in the inertial frame of each node of each line
         //   Array Nodesyi (:,:) - yi-coordinates in the inertial frame of each node of each line
         //   Array Nodeszi (:,:) - zi-coordinates in the inertial frame of each node of each line

	double t =  *t_in;		// this is the current time
	double dtC =  *dt_in;	// this is the coupling time step
	
	
	// should check if wave kinematics have been set up if expected!
	
	
	// calculate TransMat     <<< check correct directions of this
	double TransMat[9];
	RotMat(X[3], X[4], X[5], TransMat);
	
	
	if (dtC > 0) // if DT > 0, do simulation, otherwise just return last calculated values.
	{	
				
		
		// calculate positions and velocities for fairleads ("vessel" connections)
		for (int ln=0; ln < nFairs; ln++)
		{			
			// locations (unrotated reference frame) about platform reference point
			rFairRel[ln][0] = TransMat[0]*rFairtS[ln][0] + TransMat[1]*rFairtS[ln][1] + TransMat[2]*rFairtS[ln][2];	// x
			rFairRel[ln][1] = TransMat[3]*rFairtS[ln][0] + TransMat[4]*rFairtS[ln][1] + TransMat[5]*rFairtS[ln][2];	// y
			rFairRel[ln][2] = TransMat[6]*rFairtS[ln][0] + TransMat[7]*rFairtS[ln][1] + TransMat[8]*rFairtS[ln][2];	// z

			// absolute locations
			rFairi[ln][0] = rFairRel[ln][0] + X[0];	//rFairtS[ln][0] - X[5]*rFairtS[ln][1] + X[4]*rFairtS[ln][2] + X[0];		// x
			rFairi[ln][1] = rFairRel[ln][1] + X[1];	//X[5]*rFairtS[ln][0] + rFairtS[ln][1] - X[3]*rFairtS[ln][2] + X[1];		// y
			rFairi[ln][2] = rFairRel[ln][2] + X[2];	//-X[4]*rFairtS[ln][0] + X[3]*rFairtS[ln][1] + rFairtS[ln][2] + X[2];		// z

			// absolute velocities
			rdFairi[ln][0] =                        - XD[5]*rFairRel[ln][1] + XD[4]*rFairRel[ln][2] + XD[0];		// x   
			rdFairi[ln][1] =  XD[5]*rFairRel[ln][0]	                        - XD[3]*rFairRel[ln][2] + XD[1];		// y
			rdFairi[ln][2] = -XD[4]*rFairRel[ln][0] + XD[3]*rFairRel[ln][1]	                       + XD[2];		// z
		}
		
		
		// call new fairlead-centric time stepping function (replaced part of what used to be in this function)
		FairleadsCalc(rFairi, rdFairi, Ffair, t_in, dt_in);
		
		
	//	// send latest fairlead kinematics to fairlead objects
	//	for (int l=0; l < nFairs; l++)  
	//		ConnectionList[FairIs[l]].initiateStep(rFairi[l], rdFairi[l], t);					
	//				
	//				
	//	// round to get appropriate mooring model time step
	//	int NdtM = ceil(dtC/dtM0);   // number of mooring model time steps per outer time step
	//	if (NdtM < 1)  
	//	{	cout << "   Error: dtC is less than dtM.  (" << dtC << " < " << dtM0 << ")" << endl;
	//		return -1;
	//	}
	//	double dtM = dtC/NdtM;		// mooring model time step size (s)
	//	
	//	
	//	// loop through line integration time steps (integrate solution forward by dtC)
	//	for (int its = 0; its < NdtM; its++)
	//		rk2 (states, &t, dtM );  			// call RK2 time integrator (which calls the model)
	//		//t = t + dtM;                      // update time XXX TIME IS UPDATED BY RK2!
     //
	//	
	//	// check for NaNs
	//	for (int i=0; i<nX; i++)
	//	{
	//		if (isnan(states[i]))
	//		{
	//			cout << "   Error: NaN value detected in MoorDyn state at time " << t << " s."<< endl;
	//			return -1;
	//		}
	//	}
	//	
	//		
	//	// call end routines to write output files and get forces to send to FAST
	//			
	//	// go through connections to get fairlead forces
	//	//double Ffair[3];
		double tFlines[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
		for (int l=0; l < nFairs; l++)  {
			ConnectionList[FairIs[l]]->getFnet(Ffair[l]);	// Ffair is now a global sized during setup

			// Calculate Flines! (Note direction sign conversions)  >> Recently fixed moments to use rFairRel (location relative to platform reference in inertial coord system)
			tFlines[0] = tFlines[0] + Ffair[l][0];		// x force 
			tFlines[1] = tFlines[1] + Ffair[l][1];		// y force
			tFlines[2] = tFlines[2] + Ffair[l][2];		// z force
			tFlines[3] = tFlines[3] - Ffair[l][1]*rFairRel[l][2] + Ffair[l][2]*rFairRel[l][1];	// Mx = FzRy - FyRz    fixed 2016-01-10 (-= typo)
			tFlines[4] = tFlines[4] + Ffair[l][0]*rFairRel[l][2] - Ffair[l][2]*rFairRel[l][0];	// My = FxRz - FzRx
			tFlines[5] = tFlines[5] - Ffair[l][0]*rFairRel[l][1] + Ffair[l][1]*rFairRel[l][0];	// Mz = FyRx - FxRy    fixed 2016-01-10 (-= typo)			
		}		
				 
		for (int ii=0; ii<6; ii++) FlinesS[ii] = tFlines[ii];  // assign forces to static Flines vector		
	}
	
	for (int ii=0; ii<6; ii++) Flines[ii] = FlinesS[ii];  // assign static Flines vector to returned Flines vector (for FAST)
	
	return 0;
}

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
					
					
		// round to get appropriate mooring model time step
		int NdtM = ceil(dtC/dtM0);   // number of mooring model time steps per outer time step
		if (NdtM < 1)  
		{	
			// cout << "   Error: dtC is less than dtM.  (" << dtC << " < " << dtM0 << ")" << endl;
			return -1;
		}
		double dtM = dtC/NdtM;		// mooring model time step size (s)
		
		if (wordy) cout << "In FairleadsCalc gonna run rk2 for " << NdtM << " times starting at " << t << " with dt of " << dtM << endl;
		
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
	cout << "error in LinesCalc" << endl;
}	
	return 0;
}


int DECLDIR LinesClose(void)
{
	free(states);
	free(f0       );
	free(f1       );
	free(xt       );	
	
	free2Darray(Ffair, nFairs);
	free2Darray(rFairi, nFairs);
	free2Darray(rdFairi, nFairs);
	
	// close any open output files
	if (outfileMain.is_open())
		outfileMain.close();
	for (int l=0; l<nLines; l++) 
		if (outfiles[l])							// if not null
			if (outfiles[l]->is_open())
				outfiles[l]->close();
	
	// reset counters to zero!
	nFairs  = 0;
	nAnchs  = 0;
	nConns  = 0;
		
	// clear any global vectors
	FlinesS.clear();		
	rFairtS.clear();		
	rFairRel.clear();		
//	rFairi.clear();		
//	rdFairi.clear();		
	//    LinePropList.clear(); 	
//	LineList.clear(); 		
//	ConnectionList.clear();	
	FairIs.clear();  		
	ConnIs.clear();  		
	outfiles.clear(); 		
	outChans.clear();		
	LineStateIs.clear();
	zetaCglobal.clear();
	
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

