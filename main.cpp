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

#include "Misc.h"
#include "main.h"
#include "Line.h" 
#include "Connection.h"


using namespace std;

// static vectors for fairleads
vector<double> FlinesS;					// net line force vector (6-DOF) - retains last solution for use when inputted dt is zero (such as during FAST predictor steps) when the model does not time step
vector< vector< double > > rFairtS;		// fairlead locations in turbine/platform coordinates
vector< vector< double > > rFairRel;		// fairlead locations relative to platform ref point but in inertial orientation
vector< vector< double > > rFairi;			// fairlead locations in inertial reference frame
vector< vector< double > > rdFairi;		// fairlead velocities in inertial reference frame

// static vectors to hold line and connection objects
vector< LineProps > LinePropList; 			// to hold line library types
vector< Line > LineList; 				// line objects
vector< Connection > ConnectList;			// connection objects (line joints or ends)
int nConnects; 
int nLines;
int nFairs;							// number of fairlead connections
vector< int > FairIs;  					// vector of fairlead connection indices in ConnectList vector
EnvCond env; 							// struct of general environmental parameters
vector< shared_ptr< ofstream > > outfiles; 	// a vector to hold ofstreams for each line
ofstream outfileMain;					// main output file

// state vector and stuff
double* states; 						// pointer to array comprising global state vector
int nX; 								// size of state vector array
double* xt; 							// more state vector things for rk2/rk4 integration 
double* f0;
double* f1;
//double* f2;
//double* f3;
vector< int > LineStateIs;  // vector of line starting indices in "states" array

double dt; // FAST time step
double dts; // mooring line time step   



// master function to handle time stepping
void RHSmaster( const double X[],  double Xd[], const double t)
{
	for (int l=0; l < nConnects; l++)  {	
		ConnectList[l].doRHS((X + 6*l), (Xd + 6*l), t);
	}		
	for (int l=0; l < nLines; l++) 	{
		LineList[l].doRHS((X + LineStateIs[l]), (Xd + LineStateIs[l]), t);
	}
	return;
}


// Runge-Kutta 2 integration routine
void rk2 (double x0[], double t0, double dt )
{
	RHSmaster(x0, f0, t0);	 								//f0 = f ( t0, x0 );

	for (int i=0; i<nX; i++) 
		xt[i] = x0[i] + 0.5*dt*f0[i];  						//x1 = x0 + dt*f0/2.0;
	RHSmaster(xt, f1, t0 + 0.5*dt);									//f1 = f ( t1, x1 );

	for (int i=0; i<nX; i++) 
		x0[i] = x0[i] + dt*f1[i]; 
	
	return;
}


// write all the output files for the current timestep
void AllOutput(double t)
{
	// write master output file
	if (outfileMain.is_open())
	{
		// output time
		outfileMain << t << "\t "; 
		
		// output all LINE fairlead (top end) tensions
		for (int l=0; l<nLines; l++) outfileMain << 0.001*(LineList[l].getNodeTen(LineList[l].getN())) << "\t ";
			
		outfileMain << "\n";
	}
	else cout << "Unable to write to main output file " << endl;
		
	// write individual line output files
	for (int l=0; l < nLines; l++)  LineList[l].Output(t); 
}


// initialization function
int DECLDIR LinesInit(double X[], double XD[], double* dTime)
{	
	cout << "\n Running MoorDyn (v0.9.01-mth, 20-Feb-2015).\n\n";

	dt = *dTime; // store time step from FAST	
	

	// calculate TransMat
	double TransMat[9];
	RotMat(X[3], X[4], X[5], TransMat);
		
	
	// First, load data about the mooring lines from lines.txt for use in this DLL and also by FAST
	
	// read data from file
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
	else cout << "Unable to open file" << endl; 
	
	
	// initialize data holders
	
	// defaults
	env.g = 9.8;
	env.WtrDpth = 0.;
	env.rho_w = 1025.;
	env.kb = 3.0e6;
	env.cb = 3.0e5;
	env.WaveKin = 0;   // 0=none, 1=from function, 2=from file
	
	double ICDfac = 5; // factor by which to boost drag coefficients during dynamic relaxation IC generation
	double ICdt = 1.0;						// convergence analysis time step for IC generation
	double ICTmax = 120;						// max time for IC generation
	double ICthresh = 0.001;					// threshold for relative change in tensions to call it converged
	
	double dts_in = 0.001; // requested mooring time step

	// fairlead and anchor position arrays
	vector< vector< double > > rFairt;
	vector< string > outchannels;  // string containing which channels to write to output
	
	// line connection info (temporary, until LineList addresses are done moving)
	vector< int > LineInd;
	vector< int > AnchInd;
	vector< int > FairInd;
	
	
	
	int i=0; // file line number
	
	while (i < lines.size())   // note: this doesn't do any matching between lines and nodes yet!!!
	{
		if (lines[i].find("---") != string::npos) // look for header line
		{
			if (lines[i].find("LINE DICTIONARY") != string::npos) // if line dictionary header
			{	
				cout << "   Reading line types: ";
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
						cout << entries[0] << " ";
					}
					i++;
				}
				cout << "\n";
			}
			else if (lines[i].find("NODE PROPERTIES") != string::npos) // if node properties header
			{	
				cout << "   Reading node properties: ";
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' '); // what about TABS rather than spaces???
					
					if (entries.size() >= 12) // if valid number of inputs
					{					
						ConnectProps newConnect;
						
						newConnect.number=atof(entries[0].c_str());
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
						newConnect.Cd   = atof(entries[10].c_str());
						newConnect.Ca   = atof(entries[11].c_str());
						
						// make default water depth at least the depth of the lowest node (so water depth input is optional)
						if (newConnect.Z < -env.WtrDpth)  env.WtrDpth = -newConnect.Z;
					
						// now MAKE Connection object!
						Connection tempConnect = Connection();
						tempConnect.setup(newConnect);
						ConnectList.push_back(tempConnect);
										
						if (entries[1] == "Vessel")  { // if a fairlead, add to list
							rFairt.push_back(vector<double>(3, 0.0));		// fairlead location in turbine ref frame
							rFairt.back().at(0) = atof(entries[2].c_str()); 	// x
							rFairt.back().at(1) = atof(entries[3].c_str()); 	// y
							rFairt.back().at(2) = atof(entries[4].c_str()); 	// z	
							FairIs.push_back(ConnectList.size()-1);			// index of fairlead in ConnectList vector
						}
						
						cout << newConnect.number << " ";
					}
					i++;
				}
				cout << "\n";
			}
			else if (lines[i].find("LINE PROPERTIES") != string::npos) // if line properties header
			{	
				cout << "   Reading line properties: ";
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
						int TypeNum = 0;
						for (int J=0; J<LinePropList.size(); J++)  {
							if (LinePropList[J].type.find(type) != string::npos)
								TypeNum = J;
						}
						
						// make an output file for it
						if (outchannels.size() > 1) 
						{
							stringstream oname;
							oname << "Mooring/Line" << number << ".out";
							outfiles.push_back( make_shared<ofstream>(oname.str())); // used to trigger a problem
						}
						else  outfiles.push_back(NULL);  // null pointer to indicate we're not using an output file here
						
						// find correct connection indices
						for (int J=0; J<ConnectList.size(); J++)  {
							if (ConnectList[J].number == NodeAnch)
								NodeAnch = J;
							if (ConnectList[J].number == NodeFair)
								NodeFair = J;							
						}	
						
						// set up line properties
						tempLine.setup(number, LinePropList[TypeNum], UnstrLen, NumNodes, 
							ConnectList[NodeAnch], ConnectList[NodeFair], 
							outfiles.back(), outchannels);
							
							
						LineList.push_back(tempLine); // new  -- resizing the Line contents before adding to LineList (seems to prevent memory bugs)
							
						// store connection info to apply later (once lines are all created)
						LineInd.push_back(LineList.size()-1);
						AnchInd.push_back(NodeAnch);
						FairInd.push_back(NodeFair);
						
						cout << number << " ";
														
					}
					i++;
				}		
				cout << "\n";		
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
						if (entries[1] == "DT")        dts_in = atof(entries[0].c_str());
						//else if (entries[1] == "DWWave")    dw_in = atof(entries[0].c_str());
						else if (entries[1] == "kb")        env.kb = atof(entries[0].c_str());
						else if (entries[1] == "cb")        env.cb = atof(entries[0].c_str());
						else if (entries[1] == "WtrDpth")   env.WtrDpth = atof(entries[0].c_str());
						else if (entries[1] == "ICDfac")    ICDfac   = atof(entries[0].c_str());
						else if (entries[1] == "ICdt")      ICdt     = atof(entries[0].c_str());
						else if (entries[1] == "ICTmax")    ICTmax   = atof(entries[0].c_str());
						else if (entries[1] == "ICthresh")  ICthresh = atof(entries[0].c_str());
						else if (entries[1] == "WaveKin")   env.WaveKin = atoi(entries[0].c_str());
					}
					i++;
				}
			}
			else i++;
		}
		else i++;
	}
	
	
	// ==============================================================================

	nConnects = ConnectList.size();
	nLines = LineList.size();
			
			
	dts = dt/round(dt/dts_in);
	cout << "   Mooring model dt = " << dts << " s based on requested value of " << dts_in << endl;
	
	
	// initialize dummy wave stuff for now
	// (in general, set up wave stuff here BEFORE adding line to vector)
	vector<double> Ucurrent(3, 0.0);  // should make this an input to the DLL at some point.
	for (int l=0; l<nLines; l++) 
		LineList[l].setupWaves(env, Ucurrent, 9000.);   // sending env struct (important)
															
	// now that everything is initialized, tell the Connections about the connections
	cout <<   "   Connecting anchors:   ";
	for (int l=0; l<nLines; l++) ConnectList[AnchInd[l]].addLineToConnect(LineList[LineInd[l]], 0);
	cout << "\n   Connecting fairleads: ";
	for (int l=0; l<nLines; l++) ConnectList[FairInd[l]].addLineToConnect(LineList[LineInd[l]], 1);
	cout << "\n";
		
	
	// ----------------- prepare state vector ------------------
	
	// go through objects to figure out starting indices
	int n = nConnects*6; 	// start index of first line's states (added six state variables for each connection)
	for (int l=0; l<nLines; l++) 
	{
		LineStateIs.push_back(n);  		// assign start index of each line
		n += (LineList[l].getN()-1)*6;	// add 6 state variables for each internal node
	}		
	
	// make state vector	
	nX = n;  // size of state vector array
	states    = (double*) malloc( nX*sizeof(double) );

	// make arrays for integration
	f0 = (double*) malloc( nX*sizeof(double) );
	f1 = (double*) malloc( nX*sizeof(double) );
	//f2 = (double*) malloc( nX*sizeof(double) );
	//f3 = (double*) malloc( nX*sizeof(double) );
	xt = (double*) malloc( nX*sizeof(double) );
	
	
	// --------- Allocate/size some global, persistent vectors -------------

	nFairs = rFairt.size();
	
	FlinesS.resize(6);  // should clean up these var names
	rFairtS.resize (nFairs);
	rFairRel.resize(nFairs);
	rFairi.resize  (nFairs);  // after applying platform DOF ICs, should eventually pass this rather than rFairt to Line.setup()
	rdFairi.resize (nFairs);	
	

	for (unsigned int ii=0; ii<nFairs; ii++)
	{
		rFairtS[ii].resize(3);
		rFairRel[ii].resize(3);
		rFairi[ii].resize(3);
		rdFairi[ii].resize(3);
		
		rFairtS[ii][0] = rFairt[ii][0];	// store relative fairlead locations statically for internal use
		rFairtS[ii][1] = rFairt[ii][1];
		rFairtS[ii][2] = rFairt[ii][2];
		
	}
		
	
	// ------------------- initialize system, including trying catenary IC gen of Lines -------------------
	
	cout << "   Initializing mooring system" << endl;	
	for (int l=0; l<nConnects; l++)  {
		ConnectList[l].initialize( (states + 6*l), env, X, TransMat); // connections
	}	
	for (int l=0; l<nLines; l++)  {
		LineList[l].initialize( states + LineStateIs[l] );   // lines
	}
	
	// write t=-1 output line for troubleshooting preliminary ICs
	//AllOutput(-1.0);
	cout << "outputting ICs for troubleshooting" << endl;
	
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
		
	// loop through IC generation time analysis time steps
	for (int iic=0; iic<niic; iic++)
	{
		double t = iic*ICdt;			// IC gen time (s).
		
		// loop through line integration time steps
		for (double ts=t; ts<=t+ICdt-dts; ts+=dts)
			rk2 (states, ts, dts ); // changed to rk2 !!!
	
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
				
		cout << "   t = " << t << " s, tension at Node" << ConnectList[FairIs[0]].number << " is " << FairTens[0] << "\r";

		// check for convergence (compare current tension at each fairlead with previous two values)
		if (iic > 2)
		{
			for (lf=0; lf<nFairs; lf++) {
				if (( abs( FairTens[lf]/FairTensLast[lf] - 1.0 ) > ICthresh ) || ( abs( FairTens[lf]/FairTensLast2[lf] - 1.0 ) > ICthresh ) )
					break;				
			}
			
			if (lf == nFairs) {  // if we made it with all cases satisfying the threshold
				cout << "   Fairlead tensions converged to " << 100.0*ICthresh << "\% after " << t << " seconds." << endl;
				break; // break out of the time stepping loop
			}
		}
	}
	
	for (int l=0; l < nLines; l++) 
	{
		LineList[l].scaleDrag(1.0/ICDfac); // restore drag coefficients
		LineList[l].setTime(0.0);		// reset time to zero so first output line doesn't start at > 0 s
	}
	
	
	// set up waves if needed 
	if (env.waveKin == 2)
	{
		// call SetupWavesFromFile
	}		
	
	
	// start main output file
	outfileMain.open("Mooring/Lines.out");
	if (outfileMain.is_open())
	{
		// --- channel titles ---
		outfileMain << "Time" << "\t "; 	
		// output all LINE fairlead (top end) tensions
		for (lf=0; lf<nLines; lf++) outfileMain << "Fair" << LineList[lf].number << "Ten" << "\t ";
		outfileMain << "\n";
		
		// --- units ---
		outfileMain << "(s)" << "\t "; 	
		// output all LINE fairlead (top end) tensions
		for (lf=0; lf<nLines; lf++) outfileMain << "(kN)" << "\t ";
		outfileMain << "\n";
	}
	else cout << "   ERROR: Unable to write to main output file " << endl;
	
	// write t=0 output line
	AllOutput(0.0);
	
						
	cout <<endl;
	
	return 0;
}


// accept wave parameters from calling program and precalculate wave kinematics time series for each node
int DECLDIR SetupWaves(int* WaveMod, int* WaveStMod, 
	float* WaveHs, float* WaveTp, float* WaveDir, int* NStepWave2, float* WaveDOmega, 
	float* WGNC_Fact, float WGNCreal[], float WGNCimag[], float* S2Sd_Fact, float WaveS2Sdd[], float* WaveDT)
{


}



// load time series of wave elevations and process to calculate wave kinematics time series for each node
// int SetupWavesFromFile()
/*
{
	// if AS YET UNDEFINED flag, load wave elevation time series and use to overwrite fast WGNC and WaveS2Sdd variables
	
	if (need to make flag)
	{
		// read data from file
		vector<string> lines2;
		string line2;
		ifstream myfile2 ("Mooring/waves.txt");     // open an input stream to the wave elevation time series file
		if (myfile2.is_open())
		{
			while ( myfile2.good() )
			{
				getline (myfile2,line2);
				lines2.push_back(line2);
			}
			myfile2.close();
		}
		else cout << "Unable to open wave time series file" << endl; 
	
		// save data internally
	
		vector< double > wavetimes;
		vector< double > waveelevs;
		
		for (int i=0; i<lines2.size(); i++)
		{ 	
			std::vector<std::string> entries = split(lines[i], ' ');
			
			//if (entries.size() >= 2) // if a valid "[i] [j] C[i][j] [optional comment]" format
			//{
			wavetimes.push_back(entries2[0]);
			waveelevs.push_back(entries2[1]);
			}
			// now send to correctly sized fft and overwrite fast variables
			

		}
		
		// FFT operation
		
		// interpolate wave time series to match DTwave and Tend  with Nw = Tend/DTwave
		int ts0 = 0;
		vector<double> zeta(Nw, 0.0); // interpolated wave elevation time series
		for (int iw=0; iw<Nw; iw++)
		{
			double frac;
			for (int ts=ts0; ts<wavetimes.size()-1; ts++)
			{	
				if (wavetimes[ts+1] > iw*DTwave)
				{
					ts0 = ts;  //  ???
					frac = ( iw*DTwave - wavetimes[ts] )/( wavetimes[ts+1] - wavetimes[ts] );
					zeta[iw] = waveelevs[ts] + frac*(waveelevs[ts+1] - waveelevs[ts]);    // write interpolated wave time series entry
					break;
				}
			}
		}
		
		note: IFFT operation (to check FFT) can be done as follows:
		ifft(x) = conj( fft( conj(x) ) )/length(x)  //  Conjugate, fft, conjugate, scale


	}
}
	*/




int DECLDIR LinesCalc(double X[], double XD[], double Flines[], double* ZTime, double* dTime) 
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

	double t =  *ZTime;		
	double DTin =  *dTime;	
	
	// should check if wave kinematics have been set up if expected!
	
	
	// calculate TransMat     <<< check correct directions of this
	double TransMat[9];
	RotMat(X[3], X[4], X[5], TransMat);
	
	// if  (during repeat time step in initial 0.1 s) || (during a FAST predictor timestep after 0.1 s), do nothing - use the old Flines values (stored as static) so skip all the calculations
	
	// otherwise (if it's a corrector time step after t=0.05 s)
	//if ( (DTin > 0 && t < 0.1) || (DTin ==0 && t >= 0.1) )     // normal case - proceed to calculate mooring dynamics over time step
	if (DTin > 0) // if DT > 0, do simulation, otherwise just return last calculated values.
	{		
		
		//cout << " gonna do time step stuff.   t=" << t << " and dt=" << dt << endl;
		
		//cout << "FYI, Line " << LineList[0].number << " address=" << &LineList[0] << " -------------2" << endl;
		
		
		// calculate positions and velocities for fairleads ("vessel" connections)
		for (int ln=0; ln < rFairi.size(); ln++)
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
		
		//cout << "1 ";
		
		
		
		// initiate step of each object	
		int lf = 0; // index of fairlead
		for (int l=0; l < nConnects; l++)  
		{
			if (ConnectList[l].type == 1)  
			{
				ConnectList[l].initiateStep(rFairi[lf], rdFairi[lf], t);
				lf++;
			}
			else
				continue;  // other connect types don't need anything, right?				
		}		
					
			
		// loop through line integration time steps
		double ts; // time step time (s)
		for (ts=t; ts<=t+DTin-dts; ts+=dts)
			rk2 (states, ts, dts ); // changed to rk2 !!!
		
		
			
		// call end routines to write output files and get forces to send to FAST
		
		
		// go through connections to get fairlead forces
		double Ffair[3];
		float tFlines[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		lf = 0; // index of fairlead (THIS IS THE INDEX TO USE BELOW)
		for (int l=0; l < nConnects; l++)  {	
			if (ConnectList[l].type == 1)  {
				ConnectList[l].getFnet(Ffair);

				// Calculate Flines! (Note direction sign conversions)  >> Recently fixed moments to use rFairRel (location relative to platform reference in inertial coord system)
				tFlines[0] += Ffair[0];		// x force 
				tFlines[1] += Ffair[1];		// y force
				tFlines[2] += Ffair[2];		// z force
				tFlines[3] -= Ffair[1]*rFairRel[lf][2] + Ffair[2]*rFairRel[lf][1];	// Mx = FzRy - FyRz
				tFlines[4] += Ffair[0]*rFairRel[lf][2] - Ffair[2]*rFairRel[lf][0];	// My = FxRz - FzRx
				tFlines[5] -= Ffair[0]*rFairRel[lf][1] + Ffair[1]*rFairRel[lf][0];	// Mz = FyRx - FxRy
				
				// Assign horizontal and vertical fairlead force components to return to FAST
	//			FairHTen[lf] = sqrt(Ffair[0]*Ffair[0] + Ffair[1]*Ffair[1]);
	//			FairVTen[lf] = -Ffair[2];
				lf++;
			}			
		}		
		
		// now go through connections to get anchor forces (this won't work if there are more anchors than fairleads (or NumLines in FAST))
		double Fanch[3];
		lf = 0; // index of anchor
		for (int l=0; l < nConnects; l++)  {	
			if (ConnectList[l].type == 0)  {
				ConnectList[l].getFnet(Fanch);
				// Assign horizontal and vertical anchor force components to return to FAST
	//			AnchHTen[lf] = sqrt(Fanch[0]*Fanch[0] + Fanch[1]*Fanch[1]);
	//			AnchVTen[lf] = Fanch[2];	
				lf++;
			}			
		}
		
		
		AllOutput(ts);   // write outputs
		 
		for (int ii=0; ii<6; ii++) FlinesS[ii] = tFlines[ii];  // assign forces to static Flines vector		
	}
	
	for (int ii=0; ii<6; ii++) Flines[ii] = FlinesS[ii];  // assign static Flines vector to returned Flines vector (for FAST)
	
	return 0;
}


int DECLDIR LinesClose(void)
{
	free(states);
	free(f0       );
	free(f1       );
	//free(f2       );
	//free(f3       );
	free(xt       );
	
	// close output files
	outfileMain.close();
	for (int l=0; l<nLines; l++) outfiles[l]->close();
	
	return 0;
}


double DECLDIR GetFairTen(int l)
{
	// output LINE fairlead (top end) tensions
	return LineList[l].getNodeTen(LineList[l].getN());
}

