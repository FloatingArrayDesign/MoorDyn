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

#include "Waves.h"

using namespace std;


// -------------------------------------------------------


// get grid axis coordinates, initialize/record in array, and return size
int gridAxisCoords(int coordtype, vector< string > &entries, double *&coordarray)
{
	
	// set number of coordinates
	int n = 0;
	
	if (     coordtype==0)    // 0: not used - make one grid point at zero
		n = 1;
	else if (coordtype==1)    // 1: list values in ascending order
		n = entries.size();
	else if (coordtype==2)    // 2: uniform specified by -xlim, xlim, num
		n = atoi(entries[2].c_str());
	else
	{
		cout << "Error: invalid coordinate type specified to gridAxisCoords" << endl;
		return 0;
	}

	// allocate coordinate array
	//coordarray = (double*) malloc(n*sizeof(double)); 
	coordarray = make1Darray(n);

	// fill in coordinates
	if (     coordtype==0)   
		coordarray[0] = 0.0;

	else if (coordtype==1)   
		for (int i=0; i<n; i++)
			coordarray[i] = atof(entries[i].c_str());

	else if (coordtype==2)   
	{
		coordarray[0  ] = atof(entries[0].c_str());
		coordarray[n-1] = atof(entries[1].c_str());
		double dx = (coordarray[n-1]-coordarray[0])/((double)(n-1));
		for (int i=1; i<n-1; i++)
			coordarray[i] = coordarray[0] + ((double)i)*dx;
	}
	else
		cout << "Error: invalid coordinate type specified to gridAxisCoords" << endl;

	cout << "Set water grid coordinates to :";
	for (int i=0; i<n; i++)
		cout << " " << coordarray[i];
	cout << endl;

	return n;
}


// function to set up the wave/current kinematics grid from file water_grid.txt
void Waves::makeGrid()
{

	// --------------------- read grid data from file ------------------------
	vector<string> lines2;
	string line2;
	string WaveFilename = "Mooring/water_grid.txt";  // should set as overideable default later
	
	ifstream myfile2 (WaveFilename);     // open an input stream to the wave elevation time series file
	if (myfile2.is_open())
	{
		while ( getline(myfile2,line2) )
			lines2.push_back(line2);
		myfile2.close();
	}
	else cout << "   Error: Unable to open water_grid.txt" << endl; 

	
	// ----------------------- save data internally ---------------------------

	vector< double > wavefreqs;
	vector< double > waveelevs;
	
	if (lines2.size() >= 9)   // make sure enough lines
	{ 	

		vector< string > entries2;
		int coordtype;

		entries2 = split(lines2[3]);                  // get the entry type		
		coordtype = atoi(entries2[0].c_str());
		entries2 = split(lines2[4]);                  // get entries
		nx = gridAxisCoords(coordtype, entries2, px);
		
		entries2 = split(lines2[5]);                  // get the entry type	
		coordtype = atoi(entries2[0].c_str());		
		entries2 = split(lines2[6]);                  // get entries
		ny = gridAxisCoords(coordtype, entries2, py);
		
		entries2 = split(lines2[7]);                  // get the entry type		
		coordtype = atoi(entries2[0].c_str());	
		entries2 = split(lines2[8]);                  // get entries
		nz = gridAxisCoords(coordtype, entries2, pz);


	}
	else
		cout << "Error: water_grid.txt needs to have 9 lines." << endl;
	
	if (wordy > 1) cout << "   Done reading file. " << endl;
	
	
	// allocate remaining variables for grid data
	allocateKinematicsArrays();
}	

	
// allocate water kinematics arrays
void Waves::allocateKinematicsArrays()
{	
	if ((nx > 0) && (ny > 0) && (nz > 0) && (nt > 0))
	{
		zeta = make3Darray(nx,ny,nt);
		PDyn = make4Darray(nx,ny,nz,nt);
		ux   = make4Darray(nx,ny,nz,nt);
		uy   = make4Darray(nx,ny,nz,nt);
		uz   = make4Darray(nx,ny,nz,nt);
		ax   = make4Darray(nx,ny,nz,nt);
		ay   = make4Darray(nx,ny,nz,nt);
		az   = make4Darray(nx,ny,nz,nt);
		if (wordy>1) cout << "   Done allocating Waves grid" << endl;
	}
	else
		cout << "Error in Waves::makeGrid, a time or space array is size zero." << endl;

}


void Waves::setup(EnvCond *env)
{
	//WaveKin = env_in.WaveKin;
	
	dtWave = env->dtWave;
	rho_w = env->rho_w;
	g     = env->g;
	
	
	// ------------------- start with wave kinematics -----------------------
	
	// WaveKin options: 0 - none or set externally during the sim (Waves object not needed unless there's current) [default]
	//                  1 - set externally for each node in each object (Waves object not needed unless there's current)
	//                  2 - set from inputted wave elevation FFT, grid approach*
	//                  3 - set from inputted wave elevation time series, grid approach*
	//                  4 - set from inputted wave elevation FFT, node approach
	//                  5 - set from inputted wave elevation time series, node approach
	//                  6 - set from inputted velocity, acceleration, and wave elevation grid data (TBD)**

	// Current options: 0 - no currents or set externally (as part of WaveKin =0 or 1 approach) [default]
    //                  1 - read in steady current profile, grid approach (current_profile.txt)**
    //                  2 - read in dynamic current profile, grid approach (current_profile_dynamic.txt)**
    //                  3 - read in steady current profile, node approach (current_profile.txt)
    //                  4 - read in dynamic current profile, node approach (current_profile_dynamic.txt)
    
	// * the first call to any of these will attempt to load water_grid.txt to define the grid to put things on
	// ** if a grid has already been set, these will interpolate onto it, otherwise they'll make a new grid based on their provided coordinates


	// start grid size at zero (used as a flag later)
	nx = 0;
	ny = 0;
	nz = 0;
	
	
	// ======================== check compatibility of wave and current settings =====================
	
	if (env->WaveKin==0)
	{
		if      (env->Current==0)     cout << "No Waves or Currents, or set externally" << endl;
		else if (env->Current==1)     cout << "Current only: option 1 - read in steady current profile, grid approach (current_profile.txt)" << endl;
		else if (env->Current==2)     cout << "Current only: option 2 - read in dynamic current profile, grid approach (current_profile_dynamic.txt)**" << endl;
		else if (env->Current==3)     cout << "Current only: opt TBD3 - read in steady current profile, node approach (current_profile.txt)" << endl;
		else if (env->Current==4)     cout << "Current only: opt TBD4 - read in dynamic current profile, node approach (current_profile_dynamic.txt)" << endl;
		else                          cout << "ERROR in Current input settings (must be 0-4)" << endl;
	}
	else if (env->Current==0)
	{
		if      (env->WaveKin==0)     cout << "No Waves or Currents, or set externally" << endl;  // this line redundant
		else if (env->WaveKin==1)     cout << "Waves only: option 1 - set externally for each node in each object" << endl;
		else if (env->WaveKin==2)     cout << "Waves only: option 2 - set from inputted wave elevation FFT, grid approach*" << endl;
		else if (env->WaveKin==3)     cout << "Waves only: option 3 - set from inputted wave elevation time series, grid approach*" << endl;
		else if (env->WaveKin==4)     cout << "Waves only: opt TBD4 - set from inputted wave elevation FFT, node approach" << endl;
		else if (env->WaveKin==5)     cout << "Waves only: opt TBD5 - set from inputted wave elevation time series, node approach" << endl;
		else if (env->WaveKin==6)     cout << "Waves only: opt TBD6 - set from inputted velocity, acceleration, and wave elevation grid data (TBD)**" << endl;
		else                          cout << "ERROR in WaveKin input settings (must be 0-6)" << endl;
	}
	// grid approaches
	else if ((env->WaveKin==2) && (env->Current==1)) cout << "Waves and currents: options "<<env->WaveKin<<" & "<<env->Current<<" - grid approahces" << endl;
	else if ((env->WaveKin==2) && (env->Current==2)) cout << "Waves and currents: options "<<env->WaveKin<<" & "<<env->Current<<" - grid approahces" << endl;
	else if ((env->WaveKin==3) && (env->Current==1)) cout << "Waves and currents: options "<<env->WaveKin<<" & "<<env->Current<<" - grid approahces" << endl;
	else if ((env->WaveKin==3) && (env->Current==2)) cout << "Waves and currents: options "<<env->WaveKin<<" & "<<env->Current<<" - grid approahces" << endl;
	// node approaches
	else if ((env->WaveKin==4) && (env->Current==3)) cout << "Waves and currents: options TBD "<<env->WaveKin<<" & "<<env->Current<<" - node approahces" << endl;
	else if ((env->WaveKin==4) && (env->Current==4)) cout << "Waves and currents: options TBD "<<env->WaveKin<<" & "<<env->Current<<" - node approahces" << endl;
	else if ((env->WaveKin==5) && (env->Current==3)) cout << "Waves and currents: options TBD "<<env->WaveKin<<" & "<<env->Current<<" - node approahces" << endl;
	else if ((env->WaveKin==5) && (env->Current==4)) cout << "Waves and currents: options TBD "<<env->WaveKin<<" & "<<env->Current<<" - node approahces" << endl;

	else cout << "ERROR: Waves and currents settings are incompatible!" << endl;
	
	
	// NOTE: nodal settings should use storeWaterKin in objects
	

	// now go through each applicable WaveKin option
	
	// ===================== set from inputted wave elevation FFT, grid approach =====================
	if (env->WaveKin == 2)
	{
		// load discrete wave elevation FFT data from file
		
		cout << "   About to try reading wave_frequencies.txt" << endl; 
		
		
		// <<<<<<<<<<< need to decide what inputs/format to expect in file (1vs2-sided spectrum?)
		
		
		// --------------------- read data from file ------------------------
		vector<string> lines2;
		string line2;
		string WaveFilename = "Mooring/wave_frequencies.txt";  // should set as overideable default later
		
		ifstream myfile2 (WaveFilename);     // open an input stream to the wave elevation time series file
		if (myfile2.is_open())
		{
			while ( getline(myfile2,line2) )
			{
				lines2.push_back(line2);
			}
			myfile2.close();
			
			// should add error checking.  two columns of data, and time column must start at zero?
		}
		else cout << "   Error: Unable to open wave time series file wave_frequencies.txt" << endl; 

		
		// ----------------------- save data internally ---------------------------

		vector< double > wavefreqs;
		vector< double > waveelevs;
		
		for (unsigned int i=0; i<lines2.size(); i++)
		{
			vector< string > entries2 = split(lines2[i]);

			if (entries2.size() >= 2) {
				wavefreqs.push_back(atof(entries2[0].c_str()));
				waveelevs.push_back(atof(entries2[1].c_str()));
			}
			else
			{
				cout << "Issue with reading wave_frequencies.txt - not enough columns" << endl;
				break;
			}

		}
		if (wordy > 1) cout << "   Done reading file. " << endl;
		
		
		// -------------------- interpolate/check frequency data ----------------------
		
		cout << " ERROR - WaveKin=2 IS NOT SET UP YET " << endl;
		
		
		//nFFT = ?
		
	//	doubleC *zetaC0 = (doubleC*) malloc(nFFT*sizeof(doubleC)); 
	//
	//	double zetaCRMS = 0.0;
	//	
	//	for (int i=0; i<nFFT; i++)  {
	//		zetaC0[i] = cx_out[i].r + i1*(cx_out[i].i);
	//		zetaCRMS += norm(zetaCglobal[i]);
	//	}
	//	
	//	
	//	dw = pi/dtWave;    // wave frequency interval (rad/s)  <<< make sure this calculates correctly!
	
	
		// ---------------- calculate wave kinematics throughout the grid --------------------
		
		makeGrid(); // make a grid for wave kinematics based on settings in water_grid.txt
	
	//	fillWaveGrid(zetaC0, nFFT, dw, env.g, env.WtrDepth );
		
		
	}
	// =========================== set from inputted wave elevation time series, grid approach =========================
	else if (env->WaveKin == 3)
	{
		// load wave elevation time series from file (similar to what's done in GenerateWaveExtnFile.py, and was previously in misc2.cpp)
				
		cout << "   About to try reading wave_elevation.txt" << endl; 
		
		
		// --------------------- read data from file ------------------------
		vector<string> lines2;
		string line2;
		string WaveFilename = "Mooring/wave_elevation.txt";  // should set as overideable default later
		
		ifstream myfile2 (WaveFilename);     // open an input stream to the wave elevation time series file
		if (myfile2.is_open())
		{
			while ( getline(myfile2,line2) )
			{
				lines2.push_back(line2);
			}
			myfile2.close();
			
			// should add error checking.  two columns of data, and time column must start at zero?
		}
		else cout << "   Error: Unable to open wave time series file wave_elevation.txt" << endl; 


		// ----------------------- save data internally ---------------------------

		vector< double > wavetimes;
		vector< double > waveelevs;
		
		for (unsigned int i=0; i<lines2.size(); i++)
		{
			vector< string > entries2 = split(lines2[i]);

			if (entries2.size() >= 2) {
				wavetimes.push_back(atof(entries2[0].c_str()));
				waveelevs.push_back(atof(entries2[1].c_str()));
			}
			else
			{
				cout << "Issue with reading wave_elevation.txt - not enough columns" << endl;
				break;
			}

		}
		if (wordy > 1) cout << "   Done reading file. " << endl;
		
		
		// -------------------- downsample to dtWave ---------------------------
		
		nt = floor(wavetimes.back()/dtWave);  	// number of time steps
		
		if (wordy > 1)  cout << "Nt is " << nt << endl;
		
		vector< double > waveTime (nt, 0.0); 
		vector< double > waveElev (nt, 0.0); 
		
		if (wordy > 1)  cout << "interpolated to reduce time steps from " << wavetimes.size()  << " to " << nt << endl;
		
		unsigned int ts = 0; 							// index for interpolation (so it doesn't start at the beginning every time)
		for (int i=0; i<nt; i++)
		{
			
			waveTime[i] = i*dtWave;			
						
			// interpolate wave elevation
			while (ts < wavetimes.size()) 		// start to search through input data
			{	
				if (wavetimes[ts+1] > waveTime[i])  // if using index i means the current time point waveTime[i] is spanned by input data wavetimes[ts] and wavetimes[ts+1]
				{				
					double frac = ( waveTime[i] - wavetimes[ts] )/( wavetimes[ts+1] - wavetimes[ts] );    // get interpolation fraction				
					waveElev[i] = waveelevs[ts] + frac*( waveelevs[ts+1] - waveelevs[ts] ); 			// interpolate wave elevation
					break;
				}
				ts++; // move to next recorded time step
			}
		}    	// alternatively could use interpArray(Time0.size(), nt, Time0, Elev0, Time, Elev);
		
		// // interpolate wave time series to match DTwave and Tend  with Nw = Tend/DTwave
		// int ts0 = 0;
		// vector<double> zeta(Nw, 0.0); // interpolated wave elevation time series
		// for (int iw=0; iw<Nw; iw++)
		// {
		// 	double frac;
		// 	for (int ts=ts0; ts<wavetimes.size()-1; ts++)
		// 	{	
		// 		if (wavetimes[ts+1] > iw*DTwave)
		// 		{
		// 			ts0 = ts;  //  ???
		// 			frac = ( iw*DTwave - wavetimes[ts] )/( wavetimes[ts+1] - wavetimes[ts] );
		// 			zeta[iw] = waveelevs[ts] + frac*(waveelevs[ts+1] - waveelevs[ts]);    // write interpolated wave time series entry
		// 			break;
		// 		}
		// 	}
		// }

		// ensure N is even				
		if ( nt %2 != 0 )					// if odd, trim off last value of time series
		{	nt = nt-1;
			waveTime.pop_back();
			waveElev.pop_back();
			cout << "Odd number of samples in elevation time series so trimming last sample." << endl;
		}


		// ----------------  FFT the wave elevation using kiss_fftr ---------------------------------------
		
		if (wordy > 1) cout << "FFTing wave elevation time series " << endl;

		int nFFT = nt;           // FFT size, i.e. number of time steps
		int is_inverse_fft = 0;
		int nw = nFFT/2 + 1;         // number of FFT frequencies  <<<< should check consistency
		
		//     Note: frequency-domain data is stored from dc up to 2pi.
		//	so cx_out[0] is the dc bin of the FFT
		//	and cx_out[nfft/2] is the Nyquist bin (if exists)                 ???

		double dw = pi/dtWave/nw;    // wave frequency interval (rad/s), also the minimum frequency  
	

		// allocate memory for kiss_fftr
		kiss_fftr_cfg cfg = kiss_fftr_alloc( nFFT , is_inverse_fft ,0,0 );          
			
		// allocate input and output arrays for kiss_fftr  (note that kiss_fft_scalar is set to double)
		kiss_fft_scalar* cx_in_t   = (kiss_fft_scalar*)malloc(nFFT*sizeof(kiss_fft_scalar));  // input timedata has nfft scalar points
		kiss_fft_cpx* cx_out_w  = (kiss_fft_cpx*)malloc(nw*sizeof(kiss_fft_cpx));    // output freqdata has nfft/2+1 complex points
		
		
		double zetaRMS = 0.0; // <<<<<<<<<< should print these and other stats as a check... <<<<<<
		
		
		// copy wave elevation time series into input vector (could bypass this <<<<)
		for (int i=0; i<nFFT; i++)  {
			cx_in_t[i] = waveElev[i]; 
			zetaRMS += waveElev[i]*waveElev[i];
		}
		zetaRMS = sqrt(zetaRMS/nFFT);   
		
		if (wordy > 1) cout << "processing fft" << endl;
		
		
		// perform the real-valued FFT
		kiss_fftr( cfg , cx_in_t , cx_out_w );
		
		
		if (wordy > 1) cout << "done" << endl;
		// convert
		
		// allocate stuff to get passed to line functions
		
		
		doubleC *zetaC0 = (doubleC*) malloc(nFFT*sizeof(doubleC)); 
		
		
		// copy frequencies over from FFT output (should try to bypass this copy operation too <<<)
		for (int i=0; i<nw; i++)  {
			zetaC0[i] = cx_out_w[i].r + i1*(cx_out_w[i].i);
		}
		
		// cut frequencies above 0.5 Hz (2 s) to avoid FTT noise getting amplified when moving to other points in the wave field... <<<<
		for (int i=0; i<nw; i++) 
			if (i*dw > 0.5*2*pi)
				zetaC0[i] = 0.0;
			
		
		
	
		// ---------------- calculate wave kinematics throughout the grid --------------------
	
		makeGrid(); // make a grid for wave kinematics based on settings in water_grid.txt
	
		fillWaveGrid(zetaC0, nw, dw, env->g, env->WtrDpth );
		
		// free things up  (getting errors here! suggests heap corruption elsewhere?)
		free(cx_in_t);
		free(cx_out_w);
		free(cfg);
		free(zetaC0);
		
		if (wordy > 1) cout << "freed" << endl;
		
	}
	
	
		
	// ============ now potentially add in current velocities (add to unsteady wave kinematics) ===========
	
	// =========================== read in steady current profile (current_profile.txt) =========================
	if (env->Current == 1)
	{
		cout << "   About to try reading current_profile.txt" << endl; 
		
		// some temporary vectors for loading and processing the current data
		vector< double > UProfileZ ;
		vector< double > UProfileUx;
		vector< double > UProfileUy;
		vector< double > UProfileUz;
		
		
		// --------------------- read data from file ------------------------
		vector<string> lines2;
		string line2;
		ifstream myfile2 ("Mooring/current_profile.txt");     // open an input stream to the wave elevation time series file
		if (myfile2.is_open())
		{
			while ( getline(myfile2,line2) )
			{
				lines2.push_back(line2);
			}
			myfile2.close();
			
			// should add error checking.  two columns of data
		}
		else 
		{
			cout << "   Error: Unable to open current_profile" << endl; 
			return;
		}

		// ----------------------- save data internally ---------------------------

		if (lines2.size() < 4)
			cout << "ERROR: not enough lines in current_profile.txt" << endl;
		
		for (unsigned int i=0; i<lines2.size(); i++)
		{ 	
			if (i<3)
				continue; // skip first three lines

			vector< string > entries2 = split(lines2[i]);

			if (entries2.size() >= 2) {
				UProfileZ.push_back( atof(entries2[0].c_str()));
				UProfileUx.push_back(atof(entries2[1].c_str()));
			}
			else
			{
				cout << "Issue with reading current_profile.txt - not enough columns" << endl;
				break;
			}
			
			if (entries2.size() >= 3)
				UProfileUy.push_back(atof(entries2[2].c_str()));
			else
				UProfileUy.push_back(0.0);	
			
			if (entries2.size() >= 4)
				UProfileUz.push_back(atof(entries2[3].c_str()));
			else
				UProfileUz.push_back(0.0);
				
		}
		if (wordy > 1) cout << "   Done reading file. " << endl;
		
		
		// ------------- check data ----------------

		// ---------interpolate and add data to wave kinematics grid ---------
		
		if (nx*ny*nz == 0)   // if a grid hasn't been set up yet, make it based on the read-in z values
		{
			// it's implied that WaveKin must have been 0 or 1, so now let's set it to 10 
//			env->WaveKin = 10;    // this signifies that a grid exists, and external wave kinematics inputs shouldn't be expected
			
			
			nx = 1;
			px = (double*) malloc(nx*sizeof(double));  // allocate coordinate array
			
			ny = 1;
			py = (double*) malloc(ny*sizeof(double));
			
			nz = UProfileZ.size();
			pz = (double*) malloc(nz*sizeof(double));
			
			for (int i=0; i<nz; i++)
				pz[i] = UProfileZ[i];                  // set grid z entries based on current input file depths
			
			nt = 1;                                    // set 1 time step to indicate steady data
			dtWave = 1.0;                                  // arbitrary entry
			
			
			// allocate output arrays
			allocateKinematicsArrays();
			
			// fill in output arrays
			zeta[0][0][0]    = 0.0;
			for (int i=0; i<nz; i++)
			{	PDyn[0][0][i][0] = 0.0;
				ux  [0][0][i][0] = UProfileUx[i];
				uy  [0][0][i][0] = 0.0; //UProfileUy[i];   <<<<<<<    temporary hack
				uz  [0][0][i][0] = 0.0; //UProfileUz[i];   <<<<<<<   
				ax  [0][0][i][0] = 0.0;
				ay  [0][0][i][0] = 0.0;
				az  [0][0][i][0] = 0.0;
			}
		}
		else    // otherwise interpolate read in data and add to existing grid
		{
			// make an array of the input depths (interpolation function needs an array)
			int nzin = UProfileZ.size();
			double *pzin = (double*) malloc(nzin*sizeof(double));
			for (int iz=0; iz<nz; iz++)
				pzin[iz] = UProfileZ[iz];
		
			for (int iz=0; iz<nz; iz++) // loop through grid depths and interpolate from inputted depths
			{
				double fzi[2];          // interpolation fractions
				int izi[2];             // upper index for interpolation
			
				getInterpNums(pzin, nzin, pz[iz], fzi, izi); // note this is an alternative form of this function
				
				for (int ix=0; ix<nx; ix++)
				{	for (int iy=0; iy<ny; iy++)
					{	for (int it=0; it<nt; it++)
						{
							ux[ix][iy][iz][it] += UProfileUx[izi[0]]*fzi[0] + UProfileUx[izi[1]]*fzi[1];
							uy[ix][iy][iz][it] += UProfileUy[izi[0]]*fzi[0] + UProfileUy[izi[1]]*fzi[1];
							uz[ix][iy][iz][it] += UProfileUz[izi[0]]*fzi[0] + UProfileUz[izi[1]]*fzi[1];
						}
					}
				}
			}
			free(pzin);
			
		}
		
		
	}
	
	// =========================== read in dynamic current profile (current_profile_dynamic.txt) =========================
	else if (env->Current == 2)
	{
		cout << "   About to try reading current_profile_dynamic.txt" << endl; 
		
		// some temporary vectors for loading and processing the current data
		vector< double > UProfileZ ;
		vector< double > UProfileUx;
		vector< double > UProfileUy;
		vector< double > UProfileUz;
		
		
		// --------------------- read data from file ------------------------
		vector<string> lines2;
		string line2;
		ifstream myfile2 ("Mooring/current_profile_dynamic.txt");     // open an input stream to the wave elevation time series file
		if (myfile2.is_open())
		{
			while ( getline(myfile2,line2) )
			{
				lines2.push_back(line2);
			}
			myfile2.close();
			
			// should add error checking.  two columns of data
		}
		else cout << "   Error: Unable to open current_profile_dynamic" << endl; 


		// ----------------------- save data internally ---------------------------

		if (lines2.size() < 7)
			cout << "ERROR: not enough lines in current_profile_dynamic.txt" << endl;
		
		
		vector< string > entries2;


		// determine sizes and make input data arrays
		
		entries2 = split(lines2[4]);              // this is the depths row
		
		unsigned int nzin = entries2.size();                    // number of water depths
		
		int ntin = lines2.size()-6;                    // number of time steps expected, based on number of lines in file
		
		double *pzin = (double*) malloc(nzin*sizeof(double));    // depths array
		
		double *tin = (double*) malloc(ntin*sizeof(double));    // time steps array
		
		double **Uxin = make2Darray(nzin, ntin);       // current velocity array [depth, time]
		double **Uyin = make2Darray(nzin, ntin);
		double **Uzin = make2Darray(nzin, ntin);
		
		
		// read in the depths  (Depth list (m), lowest to highest)
		for (unsigned int i=0; i<nzin; i++)
			pzin[i] = atof(entries2[i].c_str());
		
		
		// now read in the time steps
		// Time-varying values (First column time in s, then x velocities at each depth (m/s) then optionally y and z velocities at each depth
		// t(s)    u1    u2    u3    u4     [v1 v2 v3 v4]    [w1 w2 w3 w4]
		for (unsigned int i=6; i<lines2.size(); i++)
		{ 	
			int it = i-6;  // just a lazy substitution
	
			vector< string > entries2 = split(lines2[i]);

			if (entries2.size() >= 1 + nzin) 
			{	
				tin[it] = atof(entries2[0].c_str());
				
				for (unsigned int k=0; k<nzin; k++)
					Uxin[it][k] = atof(entries2[1+k].c_str());	
			}
			else // it's a bad line... throw an error or just stop here and use lines up to this point?? <<<<
			{
				cout << "Issue with reading current_profile_dynamic.txt - not enough columns at line " << i+1 << endl;
				
				nzin = it;   // only consider entries up to (but excluding) this point
				
				break;
			}
			
			if (entries2.size() >= 1 + 2*nzin)                      // if there are enough columns to contain y current data
			{	
				for (unsigned int k=0; k<nzin; k++)
					Uyin[it][k] = atof(entries2[1+nzin+k].c_str());	    // read it in
			}
			else
				for (unsigned int k=0; k<nzin; k++)                          // otherwise fill the y current array with zeros
					Uyin[it][k] = 0.0;	
			
			if (entries2.size() >= 1 + 3*nzin)                      // same for z current data
			{	
				for (unsigned int k=0; k<nzin; k++)
					Uzin[it][k] = atof(entries2[1+2*nzin+k].c_str());	
			}
			else
				for (unsigned int k=0; k<nzin; k++)
					Uzin[it][k] = 0.0;	
				
		}
		if (wordy > 1) cout << "   Done reading file. " << endl;
		free(pzin);
		free(tin);
		
		// ------------- check data ----------------
		
		if (nt < 1)
			cout << "Error: need at least one time step of current data read in from current_profile_dynamic.txt" << endl;


		// ---------interpolate and add data to wave kinematics grid ---------
		
		if (nx*ny*nz == 0)   // if a grid hasn't been set up yet, make it based on the read-in z values
		{
			
			// it's implied that WaveKin must have been 0 or 1, so now let's set it to 10 
//			env->WaveKin = 10;    // this signifies that a grid exists, and external wave kinematics inputs shouldn't be expected
			
			
			nx = 1;
			px = (double*) malloc(nx*sizeof(double));  // allocate coordinate array
			
			ny = 1;
			py = (double*) malloc(ny*sizeof(double));
			
			nz = nzin;
			pz = (double*) malloc(nz*sizeof(double));
			
			for (int i=0; i<nz; i++)
				pz[i] = pzin[i];                  // set grid z entries based on current input file depths
			
			
			dtWave = 1.0e10;
			for (int i=1; i<ntin; i++)
				if (tin[i]-tin[i-1] < dtWave)
					dtWave = tin[i]-tin[i-1];              // set the time step size to be the smallest interval in the inputted times
				
			nt = floor(tin[ntin-1]/dtWave) + 1;            // set the number of time steps to ensure the last inputted time is included
			
			
			// allocate output arrays
			allocateKinematicsArrays();
			
			// fill in output arrays
			for (int iz=0; iz<nz; iz++)      // loop through depths
			{
				for (int it=0; it<nt; it++)  // loop through time steps and interpolate from inputted time steps
				{
					double fti[2];           // interpolation fractions
					int iti[2];              // upper index for interpolation
					
					getInterpNums(tin, ntin, it*dtWave, fti, iti); // note this is an alternative form of this function

					zeta[0][0][it]     = 0.0;
					PDyn[0][0][iz][it] = 0.0;
					ux  [0][0][iz][it] = Uxin[iti[0]][iz]*fti[0] + Uxin[iti[1]][iz]*fti[1];
					uy  [0][0][iz][it] = Uyin[iti[0]][iz]*fti[0] + Uyin[iti[1]][iz]*fti[1];
					uz  [0][0][iz][it] = Uzin[iti[0]][iz]*fti[0] + Uzin[iti[1]][iz]*fti[1];
					ax  [0][0][iz][it] = 0.0;   // approximate fluid accelerations using finite difference??? <<<<<<< TODO
					ay  [0][0][iz][it] = 0.0;
					az  [0][0][iz][it] = 0.0;
				}
			}
		}
		else    // otherwise interpolate read in data and add to existing grid (dtWave, px, etc are already set in the grid)
		{
			for (int iz=0; iz<nz; iz++)      // loop through grid depths,
			{	for (int it=0; it<nt; it++)  // and loop through time steps, and interpolate from inputted depths and times
				{

					double fzi, fti;          // interpolation fractions

					int izi, iti;             // upper index for interpolation
					
					izi = getInterpNums(pzin, nzin, pz[iz], &fzi);
					iti = getInterpNums( tin, ntin, dtWave*it , &fti);
					
					double Uxi = calculate2Dinterpolation(Uxin, izi, iti, fzi, fti);
					double Uyi = calculate2Dinterpolation(Uyin, izi, iti, fzi, fti);
					double Uzi = calculate2Dinterpolation(Uzin, izi, iti, fzi, fti);

					for (int ix=0; ix<nx; ix++)   // loop through x and y grid points, applying currents to each z layer uniformly for each time step
					{	for (int iy=0; iy<ny; iy++)
						{	
							ux[ix][iy][iz][it] += Uxi;
							uy[ix][iy][iz][it] += Uyi;
							uz[ix][iy][iz][it] += Uzi;
							
							// approximate fluid accelerations using finite difference??? <<<<<<< TODO
						}
					}
				}
			}
			
		}
		
		
	}
}
	
	


// master function to get wave/water kinematics at a given point -- called by each object fro grid-based data
void Waves::getWaveKin(double x, double y, double z, double PARAM_UNUSED t,
                       double U[3], double Ud[3], double* zeta_out,
                       double* PDyn_out)
{
	
	double fx, fy, fz, ft;          // interpolation fractions

	int ix, iy, iz, it;             // lower index for interpolation
	
	ix = getInterpNums(px, nx, x, &fx);
	iy = getInterpNums(py, ny, y, &fy);
	iz = getInterpNums(pz, nz, z, &fz);
	
	//double quot = t/dtWave;   // <<<<<<<< this was a temporary hack for steady conditions <<<<<<<<<
	it =  0;// floor(quot);
	ft =  0;// quot - it; //(t-(it*dtWave))/dtWave;   //remainder(t,dtWave)/dtWave;	
	
	*zeta_out  = calculate3Dinterpolation(zeta, ix, iy, it, fx, fy, ft);
	
	*PDyn_out  = calculate4Dinterpolation(PDyn, ix, iy, iz, it, fx, fy, fz, ft);
	
	U[0]  = calculate4Dinterpolation(ux, ix, iy, iz, it, fx, fy, fz, ft);
	U[1]  = calculate4Dinterpolation(uy, ix, iy, iz, it, fx, fy, fz, ft);
	U[2]  = calculate4Dinterpolation(uz, ix, iy, iz, it, fx, fy, fz, ft);
	
	Ud[0] = calculate4Dinterpolation(ax, ix, iy, iz, it, fx, fy, fz, ft);
	Ud[1] = calculate4Dinterpolation(ay, ix, iy, iz, it, fx, fy, fz, ft);
	Ud[2] = calculate4Dinterpolation(az, ix, iy, iz, it, fx, fy, fz, ft);
	
	return;
}
	


// NEW - instantiator that takes discrete wave elevation fft data only (MORE RECENT)
void Waves::fillWaveGrid(doubleC *zetaC0, int nw, double dw, double g, double h )
{
	// go about converting inputted wave stuff into a more friendly form (unnecessary now)

	double beta = 0.0; //WaveDir_in;   <<  should enable wave spreading at some point!
	
	//nw = zetaC_in.size();   // number of wave frequency components
		
	// initialize some frequency-domain wave calc vectors
	vector< double > w(nw, 0.);
	vector< double > k(nw, 0.);	
	
	doubleC *zetaC = (doubleC*) malloc(nw*sizeof(doubleC));  // Fourier transform of wave elevation
	doubleC *PDynC = (doubleC*) malloc(nw*sizeof(doubleC));  // Fourier transform of dynamic pressure
	doubleC *UCx   = (doubleC*) malloc(nw*sizeof(doubleC));  // Fourier transform of wave velocities
	doubleC *UCy   = (doubleC*) malloc(nw*sizeof(doubleC));
	doubleC *UCz   = (doubleC*) malloc(nw*sizeof(doubleC));
	doubleC *UdCx  = (doubleC*) malloc(nw*sizeof(doubleC));  // Fourier transform of wave accelerations
	doubleC *UdCy  = (doubleC*) malloc(nw*sizeof(doubleC));
	doubleC *UdCz  = (doubleC*) malloc(nw*sizeof(doubleC));
	
	
	//dw = WaveDOmega_in;
	nt = 2*(nw-1); // this is a new variable containing the number of wave time steps to be calculated
	

	// ====================== code starts here for now ========================
	
//	// set up double-sided frequency array (rad/s) - should be symmetric: # [0,dw,2dw,....w(N/2),w(-N/2+1)...-2dw,-dw]
//	for (int i=0; i<=nw/2; i++)  w[i]       = i*dw;		
//	for (int i=1; i< nw/2; i++)  w[nw/2+i] = -w[nw/2-i];	// including negative frequencies
	
	for (int i=0; i<nw; i++)  w[i] = (double)i*dw;		// single-sided spectrum for real fft
	
	cout << "Wave frequencies in rad/s are from " << w[0] << " to " << w[nw-1] << " in increments of " << dw << endl;
	
	cout << "Wave numbers in rad/m are ";
	for (int I=0; I<nw; I++)
	{
		k[I] = WaveNumber( w[I], g, h );  // wave number
		cout << k[I] << " ";
	}
	cout << endl;
	
	
		
	if (wordy>1) cout << "   nt=" << nt << " h=" << h << endl;
	
	// precalculates wave kinematics for a given set of node points for a series of time steps

	
	if (wordy>2) cout << "    making wave Kinematics." << endl;
	
		
	// start the FFT stuff using kiss_fft
	
	int nFFT = nt;
	int is_inverse_fft = 1;
	
	// allocate memory for kiss_fftr
	kiss_fftr_cfg cfg = kiss_fftr_alloc(nFFT, is_inverse_fft, NULL, NULL);
	
	// allocate input and output arrays for kiss_fftr  (note that kiss_fft_scalar is set to double)
	kiss_fft_cpx* cx_in_w      = (kiss_fft_cpx*)malloc(nw*sizeof(kiss_fft_cpx));
	kiss_fft_scalar* cx_out_t  = (kiss_fft_scalar*)malloc(nFFT*sizeof(kiss_fft_scalar));
		
		
	// ----------------- calculating wave kinematics for each grid point -------------------------

	for (int ix=0; ix<nx; ix++) // loop through x coordinates
	{
		double x = px[ix];  // rename coordinates for convenience 
		
		for (int iy=0; iy<ny; iy++) // loop through y coordinates
		{
			double y = py[iy];
			
			
			// .............................. wave elevation ...............................
			
			// handle all (not just positive-frequency half?) of spectrum?
			for (int I=0; I<nw; I++)  // Loop through the positive frequency components (including zero) of the Fourier transforms
				zetaC[I] = zetaC0[I]* exp( -i1*(k[I]*(cos(beta)*x + sin(beta)*y)));   // shift each zetaC to account for location <<< check minus sign in exponent! <<<<<
			
			
			// IFFT the wave elevation spectrum
			doIFFT(cfg, nFFT, cx_in_w, cx_out_t, zetaC, zeta[ix][iy]);


			// ...................... wave velocities and accelerations ............................

			for (int iz=0; iz<nz; iz++) // loop through z coordinates
			{	
				double z = pz[iz];
			
				for (int I=0; I<nw; I++)  // Loop through the positive frequency components (including zero) of the Fourier transforms
				{

					// Calculate SINH( k*( z + h ) )/SINH( k*h ) and COSH( k*( z + h ) )/SINH( k*h ) 
					// and COSH( k*( z + h ) )/COSH( k*h )
					// given the wave number, k, water depth, h, and elevation z, as inputs.
					double SINHNumOvrSIHNDen;
					double COSHNumOvrSIHNDen;
					double COSHNumOvrCOSHDen;
					
					if (    k[I]   == 0.0  )  					// When .TRUE., the shallow water formulation is ill-conditioned; thus, the known value of unity is returned.
					{	SINHNumOvrSIHNDen = 1.0;
						COSHNumOvrSIHNDen = 99999;
						COSHNumOvrCOSHDen = 99999;
					}
					else if ( k[I]*h >  89.4 )  				// When .TRUE., the shallow water formulation will trigger a floating point overflow error; however, with h > 14.23*wavelength (since k = 2*Pi/wavelength) we can use the numerically-stable deep water formulation instead.
					{	SINHNumOvrSIHNDen = exp(  k[I]*z );
						COSHNumOvrSIHNDen = exp(  k[I]*z );
						COSHNumOvrCOSHDen = exp(  k[I]*z ) + exp( -k[I]*( z + 2.0*h ));
					}
					else if (-k[I]*h >  89.4 )    					// @mth: added negative k case
					{	SINHNumOvrSIHNDen = -exp( -k[I]*z );
						COSHNumOvrSIHNDen = -exp( -k[I]*z );
						COSHNumOvrCOSHDen = -exp( -k[I]*z ) + exp( -k[I]*( z + 2.0*h ));  // <<< CHECK CORRECTNESS <<< 
					}
					else                          // 0 < k*h <= 89.4; use the shallow water formulation.
					{	SINHNumOvrSIHNDen = sinh( k[I]*( z + h ) )/sinh( k[I]*h );
						COSHNumOvrSIHNDen = cosh( k[I]*( z + h ) )/sinh( k[I]*h );
						COSHNumOvrCOSHDen = cosh( k[I]*( z + h ) )/cosh( k[I]*h );
					}

					
					// Fourier transform of dynamic pressure
					PDynC[I] = rho_w*g* zetaC[I]*COSHNumOvrCOSHDen;

					// Fourier transform of wave velocities 
					// (note: need to multiply by abs(w) to avoid inverting negative half of spectrum) <<< ???
					UCx[I] =     w[I]* zetaC[I]*COSHNumOvrSIHNDen *cos(beta); 
					UCy[I] =     w[I]* zetaC[I]*COSHNumOvrSIHNDen *sin(beta);
					UCz[I] = i1* w[I]* zetaC[I]*SINHNumOvrSIHNDen;

					// Fourier transform of wave accelerations					
					UdCx[I] = i1*w[I]*UCx[I];	// should confirm correct signs of +/- halves of spectrum here
					UdCy[I] = i1*w[I]*UCy[I];
					UdCz[I] = i1*w[I]*UCz[I];
					
					
				}
				// could handle negative-frequency half of spectrum with for (int I=nw/2+1; I<nw; I++) <<<
				
				//cout << "about to call IFFT " << ix << " " << iy << " " << iz << endl;
			
				// IFFT the dynamic pressure
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, PDynC, PDyn[ix][iy][iz]);
				
				// IFFT the wave velocities
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UCx, ux[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UCy, uy[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UCz, uz[ix][iy][iz]);
				
				// IFFT the wave accelerations
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UdCx, ax[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UdCy, ay[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_in_w, cx_out_t, UdCz, az[ix][iy][iz]);
				
				
				// wave stretching stuff would maybe go here?? <<<


				// output frequency data for checking
				if ((iz==nz-1) && (iy==0) && (ix==nx-2))
				{
					// write wave kinematics at some point to a file for checking...
					ofstream outfileMain;
					outfileMain.open("Mooring/WaveFreqs.out");
					if (outfileMain.is_open())
					{
						// --- channel titles ---
						outfileMain << "w\tk\tZetaC0\timag\tZetaC1\timag\tUCx1\timag\n";
						
						for (int i=0; i<nw; i++)
						{
							outfileMain << w[i] << "\t";
							outfileMain << k[i] << "\t";
							
							outfileMain << real(zetaC0[i]) << "\t";
							outfileMain << imag(zetaC0[i]) << "\t";
							
							outfileMain << real(zetaC[i]) << "\t";
							outfileMain << imag(zetaC[i]) << "\t";
							
							outfileMain << real(UCx[i]) << "\t";
							outfileMain << imag(UCx[i]) << "\t";
							
							outfileMain << endl;
						}
						outfileMain.close();
						
					}
					else 
						cout << "   ERROR: Unable to write to freq bin output file " << endl;
		
				}
				
			} // iz
		} // iy
	} // ix
	
	
	// free kiss_fft data structures cout << "   about to free fft data structures." << endl;
	free(cx_in_w);
	free(cx_out_t);
	free(cfg);
	
	// free complex vectors
	free(zetaC);
	free(PDynC);
	free(UCx  );
	free(UCy  );
	free(UCz  );
	free(UdCx );
	free(UdCy );
	free(UdCz ); 
	
	
	if (wordy>1) 
		cout << "    done wave Kinematics" << endl;
	
	
	
	// write wave kinematics at some point to a file for checking...
	
	ofstream outfileMain;
	outfileMain.open("Mooring/WaveKin.out");
	if (outfileMain.is_open())
	{
		// --- channel titles ---
		outfileMain << "Time\tEta\tux\tuz\tax\taz\tEta\tux\tuz\tax\taz\n";
		
		for (int i=0; i<nt; i++)
		{
			outfileMain << i*dtWave << "\t";
			outfileMain << zeta[nx-1][0][i] << "\t";
			outfileMain << ux[nx-1][0][nz-1][i] << "\t";
			outfileMain << uz[nx-1][0][nz-1][i] << "\t";
			outfileMain << ax[nx-1][0][nz-1][i] << "\t";
			outfileMain << az[nx-1][0][nz-1][i] << "\t";
			outfileMain << zeta[nx-2][0][i] << "\t";    // a point upstream
			outfileMain << ux[nx-2][0][nz-1][i] << "\t";
			outfileMain << uz[nx-2][0][nz-1][i] << "\t";
			outfileMain << ax[nx-2][0][nz-1][i] << "\t";
			outfileMain << az[nx-2][0][nz-1][i] << "\t";
			outfileMain << zeta[nx-3][0][i] << "\t";    // another point upstream
			outfileMain << ux[nx-3][0][nz-1][i] << "\t";
			outfileMain << uz[nx-3][0][nz-1][i] << "\t";
			outfileMain << ax[nx-3][0][nz-1][i] << "\t";
			outfileMain << az[nx-3][0][nz-1][i] << "\t";
			outfileMain << zeta[nx-1][0][i] << "\t";    // a point below
			outfileMain << ux[nx-1][0][nz-3][i] << "\t";
			outfileMain << uz[nx-1][0][nz-3][i] << "\t";
			outfileMain << ax[nx-1][0][nz-3][i] << "\t";
			outfileMain << az[nx-1][0][nz-3][i] << "\t";
			outfileMain << endl;
		}
		outfileMain.close();
		
	}
	else 
		cout << "   ERROR: Unable to write to main output file " << endl;  //TODO: handle error <<<<<<<<<<<<<<<<<<<<
		
	
	
	
	
	return;
}


// function to clear any remaining data allocations in Waves
Waves::~Waves()
{
	// clear everything 
	free(px);
	free(py);
	free(pz);
	
	free(zeta);
	free(PDyn);
	free(ux  );
	free(uy  );
	free(uz  );
	free(ax  );
	free(ay  );
	free(az  );
}



// perform a real-valued IFFT using kiss_fftr
void doIFFT(kiss_fftr_cfg cfg, int nFFT, kiss_fft_cpx* cx_in_w, kiss_fft_scalar* cx_out_t, doubleC *inputs, double *outputs)
{
	
	int nw = nFFT/2 + 1;

	for (int I=0; I<nw; I++)  
	{	                                    // copy frequency-domain data into input vector (simpler way to do this, or bypass altogether? <<<)
		cx_in_w[I].r = real(inputs[I]);       // real component - kiss_fft likes floats
		cx_in_w[I].i = imag(inputs[I]);       // imaginary component
	}
	
	// do the IFFT
	kiss_fftri(cfg, cx_in_w, cx_out_t); // kiss_fftri(kiss_fftr_cfg cfg,const kiss_fft_cpx *freqdata,kiss_fft_scalar *timedata);
	
	
	// copy out the IFFT data to the time series
	for (int I=0; I<nFFT; I++) 
		outputs[I] = cx_out_t[I] /(double)nFFT;     // is dividing by nFFT correct? (prevously was nw) <<<<<<<<<<<
	
	return;
}

// calculate wave number from frequency, g, and depth (credit: FAST source)
double WaveNumber( double Omega, double g, double h )
{
	// 
	// This FUNCTION solves the finite depth dispersion relationship:
	// 
	//                   k*tanh(k*h)=(Omega^2)/g
	// 
	// for k, the wavenumber (WaveNumber) given the frequency, Omega,
	// gravitational constant, g, and water depth, h, as inputs.  A
	// high order initial guess is used in conjunction with a quadratic
	// Newton's method for the solution with seven significant digits
	// accuracy using only one iteration pass.  The method is due to
	// Professor J.N. Newman of M.I.T. as found in routine EIGVAL of
	// the SWIM-MOTION-LINES (SML) software package in source file
	// Solve.f of the SWIM module.
	// 
	// Compute the wavenumber, unless Omega is zero, in which case, return
	//   zero:
	// 
	double k, X0;
	
	if ( Omega == 0.0 ) 	// When .TRUE., the formulation below is ill-conditioned; thus, the known value of zero is returned.
	{
		k = 0.0;  
		return k;
	}
	else 		// Omega > 0.0 solve for the wavenumber as usual.
	{
		double C  = Omega*Omega*h/g;
		double CC = C*C;

		// Find X0:
		if ( C <= 2.0 ) 
		{
			X0 =sqrt(C)*( 1.0 + C*( 0.169 + (0.031*C) ) );
		}
		else
		{
			double E2 = exp(-2.0*C);
			X0 = C*( 1.0 + ( E2*( 2.0 - (12.0*E2) ) ) );
		}

		// Find the WaveNumber:

		if ( C <= 4.8 )
		{
			double C2 = CC - X0*X0;
			double A  = 1.0/( C - C2 );
			double B  = A*( ( 0.5*log( ( X0 + C )/( X0 - C ) ) ) - X0 );

			k = ( X0 - ( B*C2*( 1.0 + (A*B*C*X0) ) ) )/h;
		}
		else
		{
			k = X0/h;
		}
	
		if (Omega < 0)  k = -k;  // @mth: modified to return negative k for negative Omega
		return k; 

	}
}



