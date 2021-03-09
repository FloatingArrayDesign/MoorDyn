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

 // This is version 1.01.03.  March 9, 2021.
 
#include "Misc.h"
#include "MoorDyn.h"
#include "Line.h" 
#include "Connection.h"

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
vector< LineProps > LinePropList; 			// to hold line library types
vector< Line > LineList; 				// line objects
vector< Connection > ConnectList;			// connection objects (line joints or ends)
int nLines;							// number of line objects
int nConnects; 						// total number of Connection objects
int nFairs  = 0;							// number of fairlead connections
int nAnchs  = 0;							// number of anchor connections
int nConns  = 0;							// number of "connect" connections

vector< int > FairIs;  					// vector of fairlead connection indices in ConnectList vector
vector< int > ConnIs;  					// vector of connect connection indices in ConnectList vector
EnvCond env; 							// struct of general environmental parameters
vector< shared_ptr< ofstream > > outfiles; 	// a vector to hold ofstreams for each line
ofstream outfileMain;					// main output file
vector< OutChanProps > outChans;		// list of structs describing selected output channels for main out file
const char* UnitList[] = {"(s)     ", "(m)     ", "(m)     ", "(m)     ", 
                          "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
					 "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
					 "(N)     ", "(N)     "};   // list of units for each of the QTypes (see misc.h)


// state vector and stuff
double* states; 						// pointer to array comprising global state vector
int nX; 								// size of state vector array
double* xt; 							// more state vector things for rk2/rk4 integration 
double* f0;
double* f1;
//double* f2;
//double* f3;

double** Ffair;	// pointer to 2-d array holding fairlead forces

vector< int > LineStateIs;  // vector of line starting indices in "states" array

//double dt; // FAST time step
double dtM0; // desired mooring line model time step   

double dtOut = 0;  // (s) desired output interval (the default zero value provides output at every call to MoorDyn)

// new temporary additions for waves
vector< floatC > zetaCglobal;
double dwW;


// new globals for creating output console window when needed
	int hConHandle;
	//long lStdHandle;
	intptr_t lStdHandle;

char const* PromptPtr;  // pointer to be made to environment variable PROMPT
int OwnConsoleWindow = 0;	

#ifdef LINUX	// any differences from built-in in mingw?  what about on OSX?
// int isnan(double x) { return x != x; } 	// changed to lower case.  will this still work?  Apparently some compiler optimizations can ruin this method
#define isnan(x) std::isnan(x)     // contributed by Yi-Hsiang Yu at NREL
#endif

// master function to handle time stepping (updated in v1.0.1 to follow MoorDyn F)
void RHSmaster( const double X[],  double Xd[], const double t, const double dt)
{
	//for (int l=0; l < nConnects; l++)  {	
	//	ConnectList[l].doRHS((X + 6*l), (Xd + 6*l), t);
	//}		

	// extrapolate instaneous fairlead positions
	for (int l=0; l<nFairs; l++)  
		ConnectList[FairIs[l]].updateFairlead( t ); 
	
	
	// calculate forces on all connection objects	
	for (int l=0; l<nConnects; l++)  
		ConnectList[l].getNetForceAndMass(); 

	// calculate connect dynamics (including contributions from latest line dynamics, above, as well as hydrodynamic forces)
	for (int l=0; l<nConns; l++)  
		ConnectList[ConnIs[l]].doRHS((X + 6*l), (Xd + 6*l), t);
		

	
	// calculate line dynamics
	for (int l=0; l < nLines; l++) 	
		LineList[l].doRHS((X + LineStateIs[l]), (Xd + LineStateIs[l]), t, dt);
		
	
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
		return LineList[outChan.ObjID-1].GetLineOutput(outChan);
	else if (outChan.OType == 2)   // connection type
		return ConnectList[outChan.ObjID-1].GetConnectionOutput(outChan);
}

// write all the output files for the current timestep
void AllOutput(double t, double dtC)
{
	// if using a certain output time step, check whether we should output
	
	if (dtOut > 0)
		if (t < (floor((t-dtC)/dtOut) + 1.0)*dtOut)  // if output should occur over the course of this time step, then do it!
			return;
	
	// What the above does is say if ((dtOut==0) || (t >= (floor((t-dtC)/dtOut) + 1.0)*dtOut)), do the below.
	// This way we avoid the risk of division by zero.
	
	// write to master output file
	if (outfileMain.is_open())
	{
		outfileMain << t << "\t "; 		// output time
	
	
		// output all LINE fairlead (top end) tensions
		//for (int l=0; l<nLines; l++) outfileMain << 0.001*(LineList[l].getNodeTen(LineList[l].getN())) << "\t ";
			

		for (int lf=0; lf<outChans.size(); lf++)   
		{
			//cout << "Getting output: OType:" << outChans[lf].OType << ", ObjID:" << outChans[lf].ObjID << ", QType:" <<outChans[lf].QType << endl;
			outfileMain << GetOutput(outChans[lf]) << "\t ";		// output each channel's value
		}
			
		outfileMain << "\n";
	}
	else cout << "Unable to write to main output file " << endl;
		
	// write individual line output files
	for (int l=0; l < nLines; l++)  LineList[l].Output(t); 
	
	return;
}



// initialization function
int DECLDIR LinesInit(double X[], double XD[])
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
	
	// ---------------------------- MoorDyn title message ----------------------------
	cout << "\n Running MoorDyn (v1.01.03C, 2021-03-09)\n   Copyright (c) Matt Hall and NREL, licensed under GPL v3.\n";

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
	
	// line connection info (temporary, until LineList addresses are done moving)
	vector< int > LineInd;
	vector< int > AnchInd;
	vector< int > FairInd;
	
	
	// ------------------------- process file contents -----------------------------------
	
	int i=0; // file line number
	
	while (i < lines.size())   // note: this doesn't do any matching between lines and nodes yet!!!
	{
		if (lines[i].find("---") != string::npos) // look for header line
		{
			if ( (lines[i].find("LINE DICTIONARY") != string::npos) || (lines[i].find("LINE TYPES") != string::npos) ) // if line dictionary header
			{	
				if (wordy>0) cout << "   Reading line types: ";
				
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' '); // what about TABS rather than spaces???
					
					if (entries.size() >= 7) // if valid number of inputs
					{
						LinePropList.push_back(LineProps());
						
						(LinePropList.back()).type =    entries[0]; //.c_str());
						(LinePropList.back()).d  = atof(entries[1].c_str());
						(LinePropList.back()).w  = atof(entries[2].c_str());
						(LinePropList.back()).EA = atof(entries[3].c_str());
						(LinePropList.back()).c  = atof(entries[4].c_str());
						(LinePropList.back()).Can= atof(entries[5].c_str());
						(LinePropList.back()).Cat= atof(entries[6].c_str());
						(LinePropList.back()).Cdn= atof(entries[7].c_str());
						(LinePropList.back()).Cdt= atof(entries[8].c_str());
						//(LinePropList.back()).ReFac= atof(entries[9].c_str());
						if (wordy>0)  cout << entries[0] << " ";
					}
					i++;
				}
				if (wordy>0) cout << "\n";
			}
			else if ( (lines[i].find("CONNECTION PROPERTIES") != string::npos) || (lines[i].find("NODE PROPERTIES") != string::npos) ) // if node properties header
			{	
				if (LinePropList.size() < 1)
					cout << "   Error: began reading connection inputs before reading any line type inputs." << endl;
			
				if (wordy>0) cout << "   Reading node properties: ";
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' '); // what about TABS rather than spaces???
					
					if (entries.size() >= 12) // if valid number of inputs
					{					
						ConnectProps newConnect;
						
						newConnect.number=atoi(entries[0].c_str());
						newConnect.type = entries[1];
						newConnect.X    = atof(entries[2].c_str());
						newConnect.Y    = atof(entries[3].c_str());
						newConnect.Z    = atof(entries[4].c_str());
						// for now we'll just say all 10 inputs need to be provided
						newConnect.M    = atof(entries[5].c_str());
						newConnect.V    = atof(entries[6].c_str());
						newConnect.FX   = atof(entries[7].c_str());
						newConnect.FY   = atof(entries[8].c_str());
						newConnect.FZ   = atof(entries[9].c_str());
						newConnect.CdA  = atof(entries[10].c_str());
						newConnect.Ca   = atof(entries[11].c_str());
						
						// make default water depth at least the depth of the lowest node (so water depth input is optional)
						if (newConnect.Z < -env.WtrDpth)  env.WtrDpth = -newConnect.Z;
					
						// now make Connection object!
						Connection tempConnect = Connection();
						tempConnect.setup(newConnect);
						ConnectList.push_back(tempConnect);
						
						// count and add to the appropriate list
						if (tempConnect.type==0)  { // if an anchor
							nAnchs ++;
						}
						else if (tempConnect.type==1)  { // if a fairlead, add to list
							rFairt.push_back(vector<double>(3, 0.0));		// fairlead location in turbine ref frame
							rFairt.back().at(0) = atof(entries[2].c_str()); 	// x
							rFairt.back().at(1) = atof(entries[3].c_str()); 	// y
							rFairt.back().at(2) = atof(entries[4].c_str()); 	// z	
							FairIs.push_back(ConnectList.size()-1);			// index of fairlead in ConnectList vector
							nFairs ++;
						}
						else if (tempConnect.type==2)  { // if a connect
							ConnIs.push_back(ConnectList.size()-1);	
							nConns ++;
						}
						else
							cout << "   Error: type of connection " << tempConnect.number << " (" << newConnect.type << ") is unknown." << endl;
						
						if (wordy>0) cout << newConnect.number << " ";
					}
					else 
					{
						cout << endl << "   Error - less than the 12 required input columns for connection " << entries[0] << " definition.  Remember CdA and Ca." << endl;
						cout << "   The line in question was read as: ";
						for (int ii=0; ii<entries.size(); ii++) cout << entries[ii] << " ";
						cout << endl;
						
						return -1;
					}
					i++;
				}
				if (wordy>0) cout << "\n";
			}
			else if (lines[i].find("LINE PROPERTIES") != string::npos) // if line properties header
			{	
				if (wordy>0) cout << "   Reading line properties: ";
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' '); // what about TABS rather than spaces???
					
					if (entries.size() >= 7) // if valid number of inputs
					{
						// Line     LineType  UnstrLen   NodeAnch  NodeFair  Flags/Outputs
						
						Line tempLine = Line(); // Jun 24 mod
																		
						int number      = atoi(entries[0].c_str());
						string type     = entries[1];
						double UnstrLen = atof(entries[2].c_str());
						int NumNodes    = atoi(entries[3].c_str()); // addition vs. MAP
						int NodeAnch    = atoi(entries[4].c_str());
						int NodeFair    = atoi(entries[5].c_str());
						string outchannels  = entries[6];
						
						// find line properties index
						int TypeNum = -1;
						for (int J=0; J<LinePropList.size(); J++)  {
							if (LinePropList[J].type.find(type) != string::npos)
								TypeNum = J;
						}
						
						if (TypeNum == -1)
							cout << "   Error: unable to identify type of line " << number << " (" << type << ") from those provided." << endl;
						
						if (wordy>1) cout << "line " << number << " type " << type << " typenum " << TypeNum << endl;
						
						// make an output file for it
						if ((outchannels.size() > 0) && (strcspn( outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))  // if 1+ output flag chars are given and they're valid
						{	stringstream oname;
							oname << "Mooring/Line" << number << ".out";
							outfiles.push_back( make_shared<ofstream>(oname.str())); // used to trigger a problem
						}
						else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
						
						// find correct connection indices
						int AnchIndex = -1;  int FairIndex = -1;
						for (int J=0; J<ConnectList.size(); J++)  {
							if (ConnectList[J].number == NodeAnch)
								AnchIndex = J;
							if (ConnectList[J].number == NodeFair)
								FairIndex = J;							
						}	
						if (FairIndex < 0)  {
							cout << "   Error: Invalid fairlead index (" << NodeFair << ") given for Line " << number << endl;
							return -1;
						}
						if (AnchIndex < 0)  {
							cout << "   Error: Invalid anchor index (" << NodeFair << ") given for Line " << number << endl;
							return -1;
						}
						// set up line properties
						tempLine.setup(number, LinePropList[TypeNum], UnstrLen, NumNodes, 
							ConnectList[AnchIndex], ConnectList[FairIndex], 
							outfiles.back(), outchannels);
							
							
						LineList.push_back(tempLine); // new  -- resizing the Line contents before adding to LineList (seems to prevent memory bugs)
							
						// store connection info to apply later (once lines are all created)
						LineInd.push_back(LineList.size()-1);
						AnchInd.push_back(AnchIndex);
						FairInd.push_back(FairIndex);
						
						if (wordy>0) cout << number << " ";
														
					}					
					else 
					{
						cout << endl << "   Error with line " << entries[0] << " inputs." << endl;
						return -1;
					}
					i++;
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

						// Process each "word" - set index, name, and units for all of each output channel

						char outWord[10];							// the buffer
						snprintf(outWord, 10, entries[j].c_str());		// copy word to buffer

						
						// convert to uppercase for string matching purposes
						// windows only  _strupr_s(outWord,10);								

						for (int charIdx=0; charIdx<10; charIdx++) 
						{
							outWord[charIdx] = toupper(outWord[charIdx]);
						}
						//cout << "  upper word is " << outWord << endl;
						
						
						// substrings for processing each parameter                              
						char let1  [10];  // letters  
						char num1  [10];  // number
						char let2  [10];  // letters
						char num2  [10];  // number                      
						char let3  [10];  // letters
						char qVal  [10];  // letters from let2 or let3
						
						//int wordLength = strlen(outWord);  // get length of input word (based on null termination)
							//cout << "1";	
						//! find indicies of changes in number-vs-letter in characters
						int in1 = strcspn( outWord, "1234567890");  // scan( OutListTmp , '1234567890' )              ! index of first number in the string
						strncpy(let1, outWord, in1);   // copy up to first number as object type
						let1[in1] = '\0';				// add null termination
						
						if (in1 < strlen(outWord))								// if there is a first number
						{	
							char *outWord1 = strpbrk(outWord, "1234567890");  		// get pointer to first number
							int il1 = strcspn( outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");  // in1+verify( OutListTmp(in1+1:) , '1234567890' )  ! second letter start (assuming first character is a letter, i.e. in1>1)
							strncpy(num1, outWord1, il1);   	// copy number
							num1[il1] = '\0';				// add null termination
							
							if (il1 < strlen(outWord1))							// if there is a second letter
							{	
								char *outWord2 = strpbrk(outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
								//cout << "3 il1=" << il1 << ", " ;	
								int in2 = strcspn( outWord2, "1234567890");  // il1+scan( OutListTmp(il1+1:) , '1234567890' )    ! second number start
								strncpy(let2, outWord2, in2);   // copy chars
								let2[in2] = '\0';				// add null termination
								
								if (in2 < strlen(outWord2))		// if there is a second number
								{
									char *outWord3 = strpbrk(outWord2, "1234567890");
									//cout << "4";	
									int il2 = strcspn( outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");  // in2+verify( OutListTmp(in2+1:) , '1234567890' )  ! third letter start
									strncpy(num2, outWord3, il2);   // copy number
									num2[il2] = '\0';				// add null termination
									
									if (il2 < strlen(outWord3))		// if there is a third letter
									{
										char *outWord4 = strpbrk(outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
										//cout << "5";	
										strncpy(let3, outWord4, strlen(outWord4));   // copy remaining chars (should be letters)  ??
										let3[strlen(outWord4)] = '\0';				// add null termination  (hopefully takes care of case where letter D.N.E.)
									}
									else
										let3[0] = '\0';
									
								}
								else
								{
									num2[0] = '\0';
									let3[0] = '\0';
								}
							}
							else
							{
								let2[0] = '\0';
								num2[0] = '\0';
								let3[0] = '\0';
							}
						
							
						}
						else
						{	
							num1[0] = '\0';
							let2[0] = '\0';
							num2[0] = '\0';
							let3[0] = '\0';					
							
							cout << "   Error: no number in channel name ("  << outWord << ")." << endl;
							return - 1;  // TODO: handle error
						}
						
						//cout << "  broken into " << let1 << ", " << num1 << ", " << let2 << ", " << num2 << ", " << let3 << endl;
						
						// error check
						if (in1 < 1) 
						{ 	// CALL DenoteInvalidOutput(p%OutParam(I)) ! flag as invalid
							//CALL WrScr('Warning: invalid output specifier.')
							//CONTINUE
							cout << "Warning: invalid output specifier (must start with letter)." << endl;
						}
						
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
							dummy.ObjID = LineList[LineID-1].FairConnect->number;		// get the connection ID of the fairlead
							dummy.NodeID = -1;           							// not used.    other%LineList(oID)%N  ! specify node N (fairlead)
						}
						// achor tension case
						else if (strcmp(let1, "ANCHTEN")==0) 
						{	
							//cout << "found anchtoen" << endl;dummy.OType = 2;             							// connection object type
							dummy.QType = Ten;           							// tension quantity type
							strncpy(dummy.Units, UnitList[Ten], UnitsSize);						// set units according to QType
							int LineID = atoi(num1);               							// this is the line number
							dummy.ObjID = LineList[LineID-1].AnchConnect->number;		// get the connection ID of the anchor
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
						//    IF (p%OutParam(I)%ObjID > p%NConnects) THEN
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

	nConnects = ConnectList.size();
	nLines = LineList.size();
			
	
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
	
	// send environmental properties struct to Connections (this is pretty mundane) 
	for (int l=0; l<nConnects; l++)  
		ConnectList[l].setEnv( env); 

		
	// ------------------------------ make connections ---------------------------------------
		
	// now that everything is initialized, tell the Connections about the connections
	if (wordy>0) cout <<   "   Connecting anchors:   ";
	for (int l=0; l<nLines; l++) ConnectList[AnchInd[l]].addLineToConnect(LineList[LineInd[l]], 0);
	if (wordy>0) cout << "\n   Connecting fairleads: ";
	for (int l=0; l<nLines; l++) ConnectList[FairInd[l]].addLineToConnect(LineList[LineInd[l]], 1);
	if (wordy>0) cout << "\n";
		
	
	
	// ----------------- prepare state vector ------------------
	
	// go through objects to figure out starting indices (changed in v1.0.1)
	int n = nConns*6; 	// start index of first line's states (add six state variables for each "connect"-type Connection)
	for (int l=0; l<nLines; l++) 
	{
		LineStateIs.push_back(n);  		// assign start index of each line
		n += (LineList[l].getN()-1)*6;	// add 6 state variables for each internal node
	}		
	
	// make state vector	
	nX = n;  // size of state vector array
	if (wordy > 1) cout << "   Creating state vectors of size " << nX << endl;
	states    = (double*) malloc( nX*sizeof(double) );

	// make arrays for integration
	f0 = (double*) malloc( nX*sizeof(double) );
	f1 = (double*) malloc( nX*sizeof(double) );
	//f2 = (double*) malloc( nX*sizeof(double) );
	//f3 = (double*) malloc( nX*sizeof(double) );
	xt = (double*) malloc( nX*sizeof(double) );
	
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
	//for (int l=0; l<nConnects; l++)  {
	//	ConnectList[l].initialize( (states + 6*l), env, X, TransMat); // connections
	//}	
	
	// set positions of fairleads based on inputted platform position
	for (int l=0; l<nFairs; l++)  
		ConnectList[FairIs[l]].initializeFairlead( X, TransMat ); // 
	
	// for connect types, write the coordinates to the state vector
	for (int l=0; l<nConns; l++)  
		ConnectList[ConnIs[l]].initializeConnect( states + 6*l ); //
	
	// go through lines and initialize internal node positions using quasi-static model
	for (int l=0; l<nLines; l++)  
		LineList[l].initialize( states + LineStateIs[l] );   // lines
	
	
	// write t=-1 output line for troubleshooting preliminary ICs
	//AllOutput(-1.0);
	//cout << "outputting ICs for troubleshooting" << endl;
	
	
	// ------------------ do dynamic relaxation IC gen --------------------
	
	cout << "   Finalizing ICs using dynamic relaxation (" << ICDfac << "X normal drag)" << endl;
	
	for (int l=0; l < nLines; l++) LineList[l].scaleDrag(ICDfac); // boost drag coefficient
	
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
			ConnectList[FairIs[lf]].getFnet(Ffair);
			FairTens[lf] = 0.0;
			for (int j=0; j<3; j++) FairTens[lf] += Ffair[j]*Ffair[j];
			FairTens[lf] = sqrt(FairTens[lf]);
		}
				
	//	cout << "size of FairIs is " << FairIs.size() << endl;
	//	cout << "FairIs 1,2 are " << FairIs[0] << FairIs[1] << endl;
	//	cout << "size of ConnectList is " << ConnectList.size() << endl;
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
	
	for (int l=0; l < nLines; l++) 
	{
		LineList[l].scaleDrag(1.0/ICDfac); // restore drag coefficients
		LineList[l].setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
	}
	
	
// @mth: new approach to be implemented
//	// ------------------------- calculate wave time series if needed -------------------
//	if (env.WaveKin == 2)
//	{
//		for (int l=0; l<nLines; l++) 
//			LineList[l].makeWaveKinematics( 0.0 );
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
	else cout << "   ERROR: Unable to write to main output file " << endl;  //TODO: handle error
	
	// write t=0 output line
	AllOutput(0.0, 0.0);
	
						
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
	//		ConnectList[FairIs[l]].initiateStep(rFairi[l], rdFairi[l], t);					
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
			ConnectList[FairIs[l]].getFnet(Ffair[l]);	// Ffair is now a global sized during setup

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
	
	
	if (dtC > 0) // if DT > 0, do simulation, otherwise leave passed fFairs unadjusted.
	{
	
		// send latest fairlead kinematics to fairlead objects
		for (int l=0; l < nFairs; l++)  
			ConnectList[FairIs[l]].initiateStep(rFairIn[l], rdFairIn[l], t);					
					
					
		// round to get appropriate mooring model time step
		int NdtM = ceil(dtC/dtM0);   // number of mooring model time steps per outer time step
		if (NdtM < 1)  
		{	cout << "   Error: dtC is less than dtM.  (" << dtC << " < " << dtM0 << ")" << endl;
			return -1;
		}
		double dtM = dtC/NdtM;		// mooring model time step size (s)
		
		
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
			ConnectList[FairIs[l]].getFnet(fFairIn[l]);

		AllOutput(t, dtC);   // write outputs
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
	LinePropList.clear(); 	
	LineList.clear(); 		
	ConnectList.clear();	
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
		return LineList[l-1].getNodeTen(LineList[l-1].getN());  // fixed the index to adjust to 0 start on March 2!
	else
		return -1;
}



int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] )
{
	// function for providing FASTv7 customary line tension quantities.  Each array is expected as length nLines
	
	for (int l=0; l< *numLines; l++)
		LineList[l].getFASTtens( &FairHTen[l], &FairVTen[l], &AnchHTen[l], &AnchVTen[l] );		
	
	return 0;
}

int DECLDIR GetConnectPos(int l, double pos[3])
{
	if ((l > 0) && (l <= nConnects))
	{
		vector< double > rs(3);
		vector< double > rds(3);
		ConnectList[l-1].getConnectState(rs, rds);
		for (int i=0; i<3; i++)
			pos[i] = rs[i];
		return 0;
	}
	else
		return -1;
}

int DECLDIR GetConnectForce(int l, double force[3])
{
	if ((l > 0) && (l <= nConnects))
	{
		ConnectList[l-1].getFnet(force);
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
		int worked = LineList[LineNum-1].getNodePos(NodeNum, pos);	// call line member function to fill in coordinates of the node of interest 
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
		LineList[l].drawGL2();  
	for (int l=0; l< nConnects; l++)
		ConnectList[l].drawGL();  
	return 0;
#endif
}

