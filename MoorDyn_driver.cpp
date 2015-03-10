#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <complex>


#include <fstream>
#include <sstream>
#include <cstring>
#include <Windows.h>

#include <memory>

#include "main.h" // include the MoorDyn header file !!!
#include "misc.h" // for string splitting functions

// this is a simple driver program for MoorDyn v0.9.0 
// it will read from an input file called "PtfmMotions.dat"
// expecting seven columns: time, surge, sway, heave, roll, pitch, yaw (s, m, m, m, deg, deg, deg)
// expects tab seperated numbers


using namespace std;

//double pi = 3.1415926;

//std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
//{
//    std::stringstream ss(s);
//    std::string item;
//    while (std::getline(ss, item, delim)) {
//		if (!item.empty())  elems.push_back(item); // skip empty ones
//    }
//    return elems;
//}
//
//std::vector<std::string> split(const std::string &s, char delim) 
//{
//    std::vector<std::string> elems;
//    split(s, delim, elems);
//    return elems;
//}



// TransMat function copied from FAST v7 NWTClib
//=======================================================================
void SmllRotTrans(float Theta1, float Theta2, float Theta3, float TransMat[9] )
{
	// Copied from NWTC Library

      // This routine computes the 3x3 transformation matrix, TransMat,
      //   to a coordinate system x (with orthogonal axes x1, x2, x3)
      //   resulting from three rotations (Theta1, Theta2, Theta3) about the
      //   orthogonal axes (X1, X2, X3) of coordinate system X.  All angles
      //   are assummed to be small, as such, the order of rotations does
      //   not matter and Euler angles do not need to be used.  This routine
      //   is used to compute the transformation matrix (TransMat) between
      //   undeflected (X) and deflected (x) coordinate systems.  In matrix
      //   form:
      //      {x1}   [TransMat(Theta1, ] {X1}
      //      {x2} = [         Theta2, ]*{X2}
      //      {x3}   [         Theta3 )] {X3}

      // The transformation matrix, TransMat, is the closest orthonormal
      //   matrix to the nonorthonormal, but skew-symmetric, Bernoulli-Euler
      //   matrix:
      //          [   1.0    Theta3 -Theta2 ]
      //      A = [ -Theta3   1.0    Theta1 ]
      //          [  Theta2 -Theta1   1.0   ]
      //
      //   In the Frobenius Norm sense, the closest orthornormal matrix is:
      //      TransMat = U*V^T,
      //
      //   where the columns of U contain the eigenvectors of A*A^T and the
      //   columns of V contain the eigenvectors of A^T*A (^T = transpose).
      //   This result comes directly from the Singular Value Decomposition
      //   (SVD) of A = U*S*V^T where S is a diagonal matrix containing the
      //   singular values of A, which are SQRT( eigenvalues of A*A^T ) =
      //   SQRT( eigenvalues of A^T*A ).

      // The algebraic form of the transformation matrix, as implemented
      //   below, was derived symbolically by J. Jonkman by computing U*V^T
      //   by hand with verification in Mathematica.


	// Compute some intermediate results:

	float Theta11      = Theta1*Theta1;
	float Theta22      = Theta2*Theta2;
	float Theta33      = Theta3*Theta3;
     float SqrdSum      = Theta11 + Theta22 + Theta33;
	float SQRT1SqrdSum = sqrt( 1.0 + SqrdSum );
	float ComDenom     = SqrdSum*SQRT1SqrdSum;
     float Theta12S     = Theta1*Theta2*( SQRT1SqrdSum - 1.0 );
	float Theta13S     = Theta1*Theta3*( SQRT1SqrdSum - 1.0 );
	float Theta23S     = Theta2*Theta3*( SQRT1SqrdSum - 1.0 );


      // Define the transformation matrix:

	if ( ComDenom == 0.0 )    // All angles are zero and matrix is ill-conditioned (the matrix is derived assuming that the angles are not zero); return identity
	{
		 for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				if (i==j) TransMat[3*i+j] = 1;
				else TransMat[3*i+j] = 0;
			}
		}
	}

	else                          // At least one angle is nonzero
	{
		TransMat[0] = ( Theta11*SQRT1SqrdSum + Theta22              + Theta33              )/ComDenom;
		TransMat[4] = ( Theta11              + Theta22*SQRT1SqrdSum + Theta33              )/ComDenom;
		TransMat[8] = ( Theta11              + Theta22              + Theta33*SQRT1SqrdSum )/ComDenom;
		TransMat[1] = (  Theta3*SqrdSum + Theta12S )/ComDenom;
		TransMat[3] = ( -Theta3*SqrdSum + Theta12S )/ComDenom;
		TransMat[2] = ( -Theta2*SqrdSum + Theta13S )/ComDenom;
		TransMat[6] = (  Theta2*SqrdSum + Theta13S )/ComDenom;
		TransMat[5] = (  Theta1*SqrdSum + Theta23S )/ComDenom;
		TransMat[7] = ( -Theta1*SqrdSum + Theta23S )/ComDenom;
	}
	return;
}



int main()	
{	
	cout << "     ------------------------------------------------------       " << endl;
	cout << "       This is MoorDyn_Driver_PtfmMotion - 2014-12-07-mth        " << endl;
	cout << "     ------------------------------------------------------       " << endl;

	
	
	

//	// define some pointers to the dll's functions
//
//	typedef int(*InitFunc)(float X[], float XD[], float TransMat[], float* dTime);
//	typedef int(*CalcFunc)(float X[], float XD[], float TransMat[], float Flines[], float* ZTime, float* dTime, 
//		int* NumLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[]);
//	typedef int(*CloseFunc)(void);
//
//	InitFunc  LinesInit; 	// Reference to the Init function in Proteus DLL
//	CalcFunc  LinesCalc;	// Reference to the AdvanceTime function in Proteus DLL
//	CloseFunc LinesClose;	// Reference to the AdvanceTime function in Proteus DLL
//
//	int iresult;
//
//	HINSTANCE hInstLibrary = LoadLibrary("Lines.dll");		// a handle to the instance of the dll
//
//	cout << "check2" << endl;
//
//	// ----------------- Get connections to MoorDyn DLL functions --------------------------------
//
//	if (hInstLibrary)
//	{
//		LinesInit = (InitFunc)GetProcAddress(hInstLibrary, "LinesInit");	// get addresses
//		LinesCalc = (CalcFunc)GetProcAddress(hInstLibrary, "LinesCalc");
//		LinesClose = (CloseFunc)GetProcAddress(hInstLibrary, "LinesClose");
//		
//		cout << "check3" << endl;
//
//		if (LinesInit)
//			std::cout << " Got address for LinesInit" << std::endl;
//		else
//			std::cout << " ERROR: Failed to get address for LinesInit" << std::endl;
//		if (LinesCalc)
//			std::cout << " Got address for LinesCalc" << std::endl;
//		else
//			std::cout << " ERROR: Failed to get address for LinesCalc" << std::endl;
//		if (LinesClose)
//			std::cout << " Got address for LinesClose" << std::endl;
//		else
//			std::cout << " ERROR: Failed to get address for LinesClose" << std::endl;
//	}
//	else
//		 std::cout << "ERROR: DLL Failed To Load!" << std::endl;
//	
	
	
	// ====================== legacy functionality for OC4 study, added 2015-03-04 ======================
	
	// First, load key inputs from lines.txt 
	
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
	else cout << "Unable to open lines.txt file" << endl; 
	
	
	// -------------------- initialize data holders --------------------------
	
	// default values
	
	double tf;  // requested end time of simulation
	double dt;   // communicatoin and output time step size
	
	string outFileName = "PtfmMotions.dat";   	// name of platform motions output file to read in
	int isMARINfile = 0;    					// for enabling special functions for reading marin files
	
	double CornerFreq = 0.0;					// corner frequency of low pass filter to be applied to platform motion data
	double zShift = 0.0;					// distance (positive up) from water line to platform reference point (possible COG?) used for platform motion in read file
	
	
	// fairlead and anchor position arrays
	//vector< vector< double > > rFairt;
	//vector< vector< double > > rAnchi;
	//vector< double > lengths;
	//vector< string > outchannels;  			// string containing which channels to write to output
	
	
	
	
	int i=0; // file line number
	
	while (i < lines.size())  
	{
		if (lines[i].find("---") != string::npos) // look for header line
		{
			if (lines[i].find("LINE DICTIONARY") != string::npos) // if line dictionary header
			{	
	//			cout << "loading line types" << endl;
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					i++;
				}
			}
			else if (lines[i].find("NODE PROPERTIES") != string::npos) // if node properties header
			{	
	//			cout << "loading node properties" << endl;
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					i++;
				}
			}
			else if (lines[i].find("LINE PROPERTIES") != string::npos) // if line properties header
			{	
	//			cout << "loading line properties.  " << endl;
				i += 3; // skip following two lines (label line and unit line)
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					i++;
				}
				//cout << nLines << " lines inputted." << endl;
			}
			else if (lines[i].find("SOLVER OPTIONS") != string::npos) // if solver options header -- this is the only section we care about
			{	
				i ++;
				while (lines[i].find("---") == string::npos) // while we DON'T find another header line
				{ 	
					std::vector<std::string> entries = split(lines[i], ' ');
					
					if (entries.size() >= 2) // if a valid "[i] [j] C[i][j] [optional comment]" format
					{
						if (entries[1] == "OutFileName")     outFileName = entries[0].c_str();   // the .out file name to read to the platform motions form
						else if (entries[1] == "isMARINfile")   isMARINfile = atoi(entries[0].c_str()); // end time for standalone version
						else if (entries[1] == "TFinal")     tf = atof(entries[0].c_str()); // end time for standalone version
						else if (entries[1] == "DTout")      dt = atof(entries[0].c_str());
						else if (entries[1] == "CornerFreq") CornerFreq = atof(entries[0].c_str());  // no longer used
						else if (entries[1] == "zShift")     zShift = atof(entries[0].c_str());
						
					}
					i++;
				}
			}
			else i++;
		}
		else i++;
	}
	
		
	
	
	// ----------------------- initialize -------------------------------------

	//double tf = 10.0;  					// requested end time of simulation
	
	//string outFileName = "PtfmMotions.dat";   	// name of FAST/MARIN output file to read in
	
	double X[6]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // platform positions
	double XD[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // platform velocities
	
	double Flines[6]; 			// dummy matrix for net mooring forces on platform, which aren't used in this version
	int NumLines = 3;
	float FairHTen[3]; 			// more dummy things (should have size of NumLines) <<<<<<<<<<
	float FairVTen[3];
	float AnchHTen[3];
	float AnchVTen[3];
	
	
	// make TransMat matrix (zero)
	//float TransMat[9];			// direction cosines matrix as used in FAST
	//SmllRotTrans(0.0, 0.0, 0.0, TransMat);
	
	//double dt = 0.0125; // desired time step for communicating with MoorDyn and outputting results
	double t = 0;
	
	// initialize MoorDyn
	LinesInit(X, XD);
	
	cout << "Done initializing MoorDyn." << endl;
	
	
	
	// ---------------------------- read platform motion data from file -----------------------------------
	
	int Xind[6];				// column indicies of platform DOFs in read FAST .out file
	vector< vector< float > > XpFAST;	// array (vector of vector double) to hold platform DOF results
	vector<float> tFAST;		// vector to hold corresponding time steps
	string line2;
	i=0; //  file line number
	

			
	// open file
	ifstream myfile2 (outFileName);     // open an input stream to the line data input file
	if (myfile2.is_open()) cout << outFileName << " opened." <<endl;
	else {	
		cout << "ERROR: Unable to open " << outFileName << "." <<endl;
		return 0;
	}	
	
	// handle differently depending on type of file
	
	
	if (isMARINfile == 0)   // if a generic 7-column file with no header
	{
		cout << "Opening geeneric platform motion data file " << outFileName << " to get platform motions." << endl;		
		double scaler[6] = {1., 1., 1., 1., 1., 1.}; 		// no special scaling or direction changing
		
		while ( myfile2.good() )
		{
			getline (myfile2,line2);
			std::vector<string> datarow = split(line2, '\t');
			
			if (wordy) cout << line2 << endl;
			if (datarow.size() < 7) break;  // break if we're on a last empty line
			//for (int k=0; k<datarow.size(); k++) if (wordy) 	cout << datarow[k] << " " ;
			if (wordy) cout << endl;
			
			
			tFAST.push_back(atof(datarow[0].c_str()));		// add time
			
			XpFAST.push_back(vector<float>(6, 0.0));			
			for (int j=0; j<6; j++)  XpFAST.back()[j] = atof(datarow[j+1].c_str())*scaler[j]; // add platform positions
			
			i++;
		}
	}
	else if (isMARINfile == 1)  // if MARIN file
	{
		cout << "Opening MARIN data file " << outFileName << " to get platform motions." << endl;		
		double scaler[6] = {-1., -1., 1., -pi/180., -pi/180., pi/180.}; 		// correct directions from MARIN convention to FAST convention
		
		while ( myfile2.good() )
		{
			getline (myfile2,line2);
			
			if (i==1)		// get channel numbers
			{	std::vector<std::string> channels = split(line2, ' ');
				
				for (int j=0; j<channels.size(); j++)
				{
//					cout << "j=" << j << " and channels[" << j << "]=" << channels[j] << endl;					
					if      (channels[j] == "36")  Xind[0] = j + 1;   // + 1 since first (Time) column isn't numbered
					else if (channels[j] == "37")  Xind[1] = j + 1;
					else if (channels[j] == "38")  Xind[2] = j + 1;
					else if (channels[j] == "158") Xind[3] = j + 1;
					else if (channels[j] == "159") Xind[4] = j + 1;
					else if (channels[j] == "160") Xind[5] = j + 1;
				}				
			}
			
			if (i > 3)	// read data lines
			{
				std::vector<string> datarow = split(line2, ' ');
				
				if (datarow.size() < 6) break;  // break if we're on a last empty line
				
				XpFAST.push_back(vector<float>(6, 0.0));
								
				for (int j=0; j<6; j++)  	XpFAST.back()[j] = atof(datarow[Xind[j]].c_str())*scaler[j];
				
				tFAST.push_back(atof(datarow[0].c_str()));		// add time
			}
			
			i++;  // count what line we're at
		}
	}
	else  // if FAST (rather than MARIN) file 
	{
		cout << "Opening FAST output file " << outFileName << " to get platform motions." << endl;				
		double scaler[6] = {1., 1., 1., pi/180., pi/180., pi/180.};
				
		if (myfile2.is_open())
		{
			while ( myfile2.good() )
			{				
				getline (myfile2,line2);
				
				if (i==6)		// get channel headers
				{
					std::vector<std::string> channels = split(line2, '\t');
					
					for (int j=0; j<channels.size(); j++)
					{	if      (channels[j] == "PtfmSurge") Xind[0] = j;
						else if (channels[j] == "PtfmSway")  Xind[1] = j;
						else if (channels[j] == "PtfmHeave") Xind[2] = j;
						else if (channels[j] == "PtfmRoll")  Xind[3] = j;
						else if (channels[j] == "PtfmPitch") Xind[4] = j;
						else if (channels[j] == "PtfmYaw")   Xind[5] = j;
					}
				}
				
				if (i > 7)	// read data lines
				{
					std::vector<string> datarow = split(line2, '\t');
					
					if (datarow.size() < 6) break;  // break if we're on a last empty line
					
					XpFAST.push_back(vector<float>(6, 0.0));
					
					//cout << Xind << endl;
					
					for (int j=0; j<6; j++)  
					{	
					//	cout << "j=" << j << " and Xind[j]=" << Xind[j] << endl;
						XpFAST.back()[j] = atof(datarow[Xind[j]].c_str());
					}
					tFAST.push_back(atof(datarow[0].c_str()));		// add time
				}
				
				i++;  // count what line we're at
			}
		}
		else cout << "Unable to open FAST output file" << endl; 
		
	}

	myfile2.close();
	
	
	
	// ---------------------- now do filtering if it's a MARIN data file (as in for OC4 study) --------------------------
	
	if (isMARINfile == 1) 
	{
		int ts = 0;	 	/// index for interpolation (so it doesn't start at the beginning every time)					
		int nts = floor(tf/dt);

		double** Xf =  make2Darray(6, nts);  // custom 2D array creation function (don't forget to free after)
		//double** XDf = make2Darray(6, nts); 
		double** Xff = make2Darray(6, nts); 		
	
	
		// -------------- downsample/uniformify platform motions --------------------
	
		for (int its=0; its<nts; its++)				// loop through upcoming simulation in/out time steps
		{		
			t = its*dt;
			
			// interpolate platform positions from .out file data
			while (ts < tFAST.size()) // search through platform motion data time steps (from .out file)
			{	
				if (tFAST[ts+1] > t)
				{				
					double frac = ( t - tFAST[ts] )/( tFAST[ts+1] - tFAST[ts] );				// get interpolation fraction
					
					for (int j=0; j<6; j++)	Xf[j][its] = XpFAST[ts][j] + frac*( XpFAST[ts+1][j] - XpFAST[ts][j] ); // interpolate for each platform DOF
					
					break;
				}			
				ts++; // move to next recorded time step
			}
		}
		

		// --------------------------- apply filter ------------------------------

		// state-space form filter coefficients
		double a[4] = {-3.91790787,  5.75707638, -3.76034951,  0.92118193}; 
		double beta[4] = {4.62812992e-07,   1.41992316e-08,   4.53603482e-07, 4.60702850e-09};
		double b0 = 5.84514243e-08;
		
		for (int j=0; j<6; j++) 
		{		
			doSSfilter(Xf[j],Xff[j], nts, a, beta, b0, 4);
			reverse(Xff[j], nts);				// flip array
			doSSfilter(Xff[j],Xf[j], nts, a, beta, b0, 4);
			reverse(Xf[j], nts);				// flip array back again
		
			// then calculate derivative (forward difference)    
		//	for (int its=0; its<nts-1; its++)	
		//		XDf[j][its] = (Xf[j][its+1] - Xf[j][its])/dt;
		//	XDf[j][nts-1] = 0.0; // last entry
		}
		
		
		// -------------------- copy filtered data to original data holders ---------------------
		
		tFAST.resize(nts);
		XpFAST.resize(nts, vector<float>(6, 0.0) );
	
		for (int its=0; its<nts; its++)				// loop through upcoming simulation in/out time steps
		{
			tFAST[its] = its*dt;
		
			for (int j=0; j<6; j++)  XpFAST[its][j] = Xf[j][its];
		}
	
	
	}
	
	
		
	// ================================== end of additions/modifications of 2015-03-04 =====================================
		
	
	
	// ----------------------- start output file for writing ------------------------
	
	ofstream linesout("DeepCwindModelTurbine.out");
	
	linesout << "FAST-style output file from MoorDyn \n\n\n\n\n\n";
	
	linesout << "Time\tPtfmSurge\tPtfmSway\tPtfmHeave\tPtfmRoll\tPtfmPitch\tPtfmYaw";
//	for (int i=0; i<6; i++) linesout << "\tXD[" << i <<"]";
	for (int i=0; i<NumLines; i++) linesout << "\tFair" << i+1 << "Ten";
//	linesout << "\tFair3x\tFair3y\tFair3z\tFair3xd\tFair3yd\tFair3zd";
	linesout << endl;
	
	linesout << "(s)\t(m)\t(m)\t(m)\t(deg)\t(deg)\t(deg)";
//	for (int i=0; i<6; i++) linesout << "\t(bla)";
	for (int i=0; i<NumLines; i++) linesout << "\t(kN)";	
//	linesout << "(s)\t(m)\t(m)\t(m)\t(m/s)\t(m/s)\t(m/s)";
	linesout << endl;
	
	

		// -------------------------- run simulation ------------------------------------
				
		int nts = tFAST.size(); 			// number of time steps
				
		cout << "Running simulation with duration of " << nts << " steps." << endl;
				
		for (int its=0; its<nts; its++)    // loop though time steps in input file
		{
			cout << "Time = " << tFAST[its] << ".\r";
				
			double t = tFAST[its];
		
			// assign platform position
			for (int j=0; j<6; j++) {
				X[j]  = XpFAST[its][j];
			}
			
			// calculate platform velocity (forward difference)  
			if (its < nts-1)
			{
				for (int j=0; j<6; j++) 
					XD[j] = (XpFAST[its+1][j] - XpFAST[its][j])/(tFAST[its+1]-tFAST[its]);			
				
				double dt = tFAST[its+1]-tFAST[its];
			}
		
			
			// calculate TransMat matrix
			//SmllRotTrans(X[3], X[4], X[5], TransMat);
			
			// call MoorDyn time stepping function
			LinesCalc(X, XD, Flines, &t, &dt);
				
			
			// write time and platform motion outputs			
			linesout << t;
			for (int i=0; i<3; i++) linesout << "\t" << X[i];
			for (int i=0; i<3; i++) linesout << "\t" << X[i+3]*180./pi;
			
	//		for (int i=0; i<3; i++) linesout << "\t" << XD[i]; // velocities too
	//		for (int i=0; i<3; i++) linesout << "\t" << XD[i+3]*180./pi;
			
			
			// write line tension outputs
			for (int l=0; l < NumLines; l++)  {	
				linesout << "\t" << 0.001*GetFairTen(l+1);  		// write fairlead tension magnitude
			}		
					
			//write fairlead 3 positions and velocities for debugging
	//		for (int i=0; i<3; i++) linesout << "\t" << rFairi[2][i]; 
	//		for (int i=0; i<3; i++) linesout << "\t" << rdFairi[2][i];
					
			linesout << endl;
	
		}	// end of in/out processing time step loop
	
		
	// freeing
	LinesClose();
	
	system("pause");
	return 0;
}
	
	
