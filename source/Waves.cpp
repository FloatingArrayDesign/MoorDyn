/*
 * Copyright (c) 2022, Matt Hall
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Waves.hpp"
#include "Waves.h"
#include "kiss_fftr.h"

#if defined WIN32 && defined max
// We must avoid max messes up with std::numeric_limits<>::max()
#undef max
#endif

using namespace std;

namespace moordyn {

std::vector<real>
gridAxisCoords(Waves::coordtypes coordtype, vector<string>& entries)
{
	// set number of coordinates
	unsigned int n = 0;
	std::vector<real> coordarray;

	if (coordtype == Waves::GRID_SINGLE)
		n = 1;
	else if (coordtype == Waves::GRID_LIST)
		n = entries.size();
	else if (coordtype == Waves::GRID_LATTICE)
		n = atoi(entries[2].c_str());
	else
		return coordarray;

	// fill in coordinates
	if (coordtype == Waves::GRID_SINGLE)
		coordarray.push_back(0.0);
	else if (coordtype == Waves::GRID_LIST) {
		for (unsigned int i = 0; i < n; i++) {
			coordarray.push_back(atof(entries[i].c_str()));
		}
	} else if (coordtype == Waves::GRID_LATTICE) {
		real first = atof(entries[0].c_str()), last = atof(entries[1].c_str());
		coordarray.push_back(first);
		real dx = (last - first) / ((real)(n - 1));
		for (unsigned int i = 1; i < n - 1; i++)
			coordarray.push_back(first + ((real)i) * dx);
		coordarray.push_back(last);
	}

	return coordarray;
}

/** @brief Carry out the inverse Fourier transform
 * @param cfg KISS FFT instance
 * @param nFFT Numer of fourier components
 * @param cx_w_in KISS FFT frequency-domain data
 * @param cx_t_out KISS FFT time-domain output
 * @param inputs Input FFT values
 * @param outputs Output time-domain values
 */
void
doIFFT(kiss_fftr_cfg cfg,
       unsigned int nFFT,
       kiss_fft_cpx* cx_w_in,
       kiss_fft_scalar* cx_t_out,
       const moordyn::complex* inputs,
       std::vector<real>& outputs)
{
	unsigned int nw = nFFT / 2 + 1;

	// copy frequency-domain data into input vector
	// NOTE: (simpler way to do this, or bypass altogether?)
	for (unsigned int i = 0; i < nw; i++) {
		cx_w_in[i].r = std::real(inputs[i]);
		cx_w_in[i].i = std::imag(inputs[i]);
	}

	kiss_fftri(cfg, cx_w_in, cx_t_out);

	// copy out the IFFT data to the time series
	for (unsigned int i = 0; i < nFFT; i++) {
		// NOTE: is dividing by nFFT correct? (prevously was nw)
		outputs[i] = cx_t_out[i] / (real)nFFT;
	}

	return;
}

Waves::Waves(moordyn::Log* log)
  : LogUser(log)
  , _t_integrator(NULL)
{
}

// function to clear any remaining data allocations in Waves
Waves::~Waves() {}

void
Waves::makeGrid(const char* filepath)
{
	LOGMSG << "Reading waves coordinates grid from '" << filepath << "'..."
	       << endl;

	// --------------------- read grid data from file ------------------------
	vector<string> lines;
	string line;

	ifstream f(filepath);
	if (!f.is_open()) {
		LOGERR << "Cannot read the file '" << filepath << "'" << endl;
		throw moordyn::input_file_error("Invalid file");
	}
	while (getline(f, line))
		lines.push_back(line);
	f.close();

	if (lines.size() < 9) {
		LOGERR << "The waves grid file '" << filepath << "' has only "
		       << lines.size() << "lines, but at least 9 are required" << endl;
		throw moordyn::input_file_error("Invalid file format");
	}

	vector<string> entries;
	coordtypes coordtype;

	entries = moordyn::str::split(lines[3]);
	coordtype = (coordtypes)atoi(entries[0].c_str());
	entries = moordyn::str::split(lines[4]);
	px = gridAxisCoords(coordtype, entries);
	nx = px.size();
	if (!nx) {
		LOGERR << "Invalid entry for the grid x values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	entries = moordyn::str::split(lines[5]);
	coordtype = (coordtypes)atoi(entries[0].c_str());
	entries = moordyn::str::split(lines[6]);
	py = gridAxisCoords(coordtype, entries);
	ny = py.size();
	if (!ny) {
		LOGERR << "Invalid entry for the grid y values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	entries = moordyn::str::split(lines[7]);
	coordtype = (coordtypes)atoi(entries[0].c_str());
	entries = moordyn::str::split(lines[8]);
	pz = gridAxisCoords(coordtype, entries);
	nz = pz.size();
	if (!nz) {
		LOGERR << "Invalid entry for the grid z values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	LOGDBG << "Setup the waves grid with " << nx << " x " << ny << " x " << nz
	       << " points " << endl;

	LOGMSG << "'" << filepath << "' parsed" << endl;

	try {
		allocateKinematicsArrays();
	} catch (...) {
		throw;
	}
}

void
Waves::allocateKinematicsArrays()
{
	if (!nx || !ny || !nz) {
		LOGERR << "The grid has not been initializated..." << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}
	if (!nt) {
		LOGERR << "The time series has null size" << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}

	zeta = init3DArray(nx, ny, nt);
	PDyn = init4DArray(nx, ny, nz, nt);
	ux = init4DArray(nx, ny, nz, nt);
	uy = init4DArray(nx, ny, nz, nt);
	uz = init4DArray(nx, ny, nz, nt);
	ax = init4DArray(nx, ny, nz, nt);
	ay = init4DArray(nx, ny, nz, nt);
	az = init4DArray(nx, ny, nz, nt);

	LOGDBG << "Allocated the waves data grid";
}

void
Waves::setup(EnvCond* env, TimeScheme* t, const char* folder)
{
	dtWave = env->dtWave;
	rho_w = env->rho_w;
	g = env->g;
	_t_integrator = t;

	// ------------------- start with wave kinematics -----------------------

	// start grid size at zero (used as a flag later)
	nx = 0;
	ny = 0;
	nz = 0;

	// ======================== check compatibility of wave and current settings
	// =====================

	if ((env->WaveKin == moordyn::WAVES_NONE) &&
	    (env->Current == moordyn::CURRENTS_NONE)) {
		LOGMSG << "No Waves or Currents, or set externally" << endl;
		return;
	}

	if (env->WaveKin == moordyn::WAVES_NONE) {
		if (env->Current == moordyn::CURRENTS_STEADY_GRID)
			LOGDBG << "Current only: option 1 - "
			       << "read in steady current profile, grid approach "
			       << "(current_profile.txt)" << endl;
		else if (env->Current == moordyn::CURRENTS_DYNAMIC_GRID)
			LOGDBG << "Current only: option 2 - "
			       << "read in dynamic current profile, grid approach "
			       << "(current_profile_dynamic.txt)" << endl;
		else if (env->Current == moordyn::CURRENTS_STEADY_NODE)
			LOGDBG << "Current only: option TBD3 - "
			       << "read in steady current profile, node approach "
			       << "(current_profile.txt)" << endl;
		else if (env->Current == moordyn::CURRENTS_DYNAMIC_NODE)
			LOGDBG << "Current only: option TBD4 - "
			       << "read in dynamic current profile, node approach "
			       << "(current_profile_dynamic.txt)" << endl;
		else {
			LOGDBG << "Invald current input settings (must be 0-4)" << endl;
			throw moordyn::invalid_value_error("Invalid settings");
		}
	} else if (env->Current == moordyn::CURRENTS_NONE) {
		if (env->WaveKin == moordyn::WAVES_EXTERNAL)
			LOGDBG << "Waves only: option 1 - "
			       << "set externally for each node in each object" << endl;
		else if (env->WaveKin == moordyn::WAVES_FFT_GRID)
			LOGDBG << "Waves only: option 2 - "
			       << "set from inputted wave elevation FFT, grid approach "
			       << "(NOT IMPLEMENTED YET)" << endl;
		else if (env->WaveKin == moordyn::WAVES_GRID)
			LOGDBG
			    << "Waves only: option 3 - "
			    << "set from inputted wave elevation time series, grid approach"
			    << endl;
		else if (env->WaveKin == moordyn::WAVES_FFT_GRID)
			LOGDBG << "Waves only: option TBD4 - "
			       << "set from inputted wave elevation FFT, node approach "
			       << "(NOT IMPLEMENTED YET)" << endl;
		else if (env->WaveKin == moordyn::WAVES_NODE)
			LOGDBG
			    << "Waves only: option TBD5 - "
			    << "set from inputted wave elevation time series, node approach"
			    << endl;
		else if (env->WaveKin == moordyn::WAVES_KIN)
			LOGDBG << "Waves only: option TBD6 - "
			       << "set from inputted velocity, acceleration, and wave "
			          "elevation grid data (TBD)"
			       << endl;
		else {
			LOGDBG << "Invald wave kinematics input settings (must be 0-6)"
			       << endl;
			throw moordyn::invalid_value_error("Invalid settings");
		}
	} else if (is_waves_grid(env->WaveKin) && is_currents_grid(env->Current)) {
		LOGDBG << "Waves and currents: options " << env->WaveKin << " & "
		       << env->Current << endl;
	} else if (is_waves_node(env->WaveKin) && is_currents_node(env->Current)) {
		LOGDBG << "Waves and currents: options TBD " << env->WaveKin << " & "
		       << env->Current << endl;
	} else {
		LOGDBG << "Incompatible waves (" << env->WaveKin << ") and currents ("
		       << env->Current << ") settings" << endl;
		throw moordyn::invalid_value_error("Invalid settings");
	}

	// NOTE: nodal settings should use storeWaterKin in objects

	// now go through each applicable WaveKin option
	if (env->WaveKin == moordyn::WAVES_FFT_GRID) {
		const string WaveFilename = (string)folder + "/wave_frequencies.txt";
		LOGMSG << "Reading waves FFT from '" << WaveFilename << "'..." << endl;

		// NOTE: need to decide what inputs/format to expect in file
		// (1vs2-sided spectrum?)

		vector<string> lines;
		string line;

		ifstream f(WaveFilename);
		if (!f.is_open()) {
			LOGERR << "Cannot read the file '" << WaveFilename << "'" << endl;
			throw moordyn::input_file_error("Invalid file");
		}
		while (getline(f, line)) {
			lines.push_back(line);
		}
		f.close();

		if (lines.size() < 2) {
			LOGERR << "At least 2 frequency components shall be provided in '"
			       << WaveFilename << "'" << endl;
			throw moordyn::input_file_error("Invalid file format");
		}

		vector<real> wavefreqs;
		vector<moordyn::complex> waveelevs;

		for (auto line : lines) {
			vector<string> entries = moordyn::str::split(line);
			if (entries.size() < 3) {
				LOGERR << "The file '" << WaveFilename
				       << "' should have 3 columns" << endl;
				throw moordyn::input_file_error("Invalid file format");
			}
			wavefreqs.push_back(atof(entries[0].c_str()));
			waveelevs.push_back((real)atof(entries[1].c_str()) +
			                    i1 * (real)atof(entries[2].c_str()));
		}
		LOGMSG << "'" << WaveFilename << "' parsed" << endl;

		if (wavefreqs[0] != 0.0) {
			LOGERR << "The first shall be 0 rad/s" << endl;
			throw moordyn::invalid_value_error("Invalid frequencies");
		}

		// Interpolate/check frequency data
		real dw = std::numeric_limits<real>::max();
		for (unsigned int i = 1; i < wavefreqs.size(); i++)
			if (wavefreqs[i] - wavefreqs[i - 1] < dw)
				dw = wavefreqs[i] - wavefreqs[i - 1];
		if (dw <= 0) {
			LOGERR << "Ascending frequencies are expected" << endl;
			throw moordyn::invalid_value_error("Invalid frequencies");
		}

		unsigned int nw = floor(wavefreqs.back() / dw) + 1;
		vector<real> freqs;
		for (unsigned int i = 0; i < wavefreqs.size(); i++)
			freqs.push_back(i * dw);
		if (freqs.back() > 0.5 * 2.0 * pi) {
			LOGERR << "The maximum frequency is " << freqs.back() / (2.0 * pi)
			       << " Hz, which is bigger than the recommended 0.5 Hz value"
			       << endl;
		}

		vector<moordyn::complex> zetaC0(nw);
		interp(wavefreqs, waveelevs, freqs, zetaC0);

		// calculate wave kinematics throughout the grid
		try {
			makeGrid(((string)folder + "/water_grid.txt").c_str());
			fillWaveGrid(zetaC0.data(), nw, dw, env->g, env->WtrDpth);
		} catch (...) {
			throw;
		}
	} else if (env->WaveKin == moordyn::WAVES_GRID) {
		// load wave elevation time series from file (similar to what's done in
		// GenerateWaveExtnFile.py, and was previously in misc2.cpp)
		const string WaveFilename = (string)folder + "/wave_elevation.txt";
		LOGMSG << "Reading waves elevation from '" << WaveFilename << "'..."
		       << endl;

		vector<string> lines;
		string line;

		ifstream f(WaveFilename);
		if (!f.is_open()) {
			LOGERR << "Cannot read the file '" << WaveFilename << "'" << endl;
			throw moordyn::input_file_error("Invalid file");
		}
		while (getline(f, line)) {
			lines.push_back(line);
		}
		f.close();

		// should add error checking.  two columns of data, and time column must
		// start at zero?

		vector<real> wavetimes;
		vector<real> waveelevs;

		for (auto line : lines) {
			vector<string> entries = moordyn::str::split(line);
			if (entries.size() < 2) {
				LOGERR << "The file '" << WaveFilename
				       << "' should have 2 columns" << endl;
				throw moordyn::input_file_error("Invalid file format");
			}
			wavetimes.push_back(atof(entries[0].c_str()));
			waveelevs.push_back(atof(entries[1].c_str()));
		}
		LOGMSG << "'" << WaveFilename << "' parsed" << endl;

		// downsample to dtWave
		nt = floor(wavetimes.back() / dtWave);
		LOGDBG << "Number of wave time samples = " << nt << "("
		       << wavetimes.size() << " samples provided in the file)" << endl;

		vector<real> waveTime(nt, 0.0);
		vector<real> waveElev(nt, 0.0);

		for (unsigned int i = 0; i < nt; i++)
			waveTime[i] = i * dtWave;
		moordyn::interp(wavetimes, waveelevs, waveTime, waveElev);

		// ensure N is even
		if (nt % 2 != 0) {
			nt = nt - 1;
			waveTime.pop_back();
			waveElev.pop_back();
			LOGWRN << "The number of wave time samples was odd, "
			       << "so it is decreased to " << nt << endl;
		}

		// FFT the wave elevation using kiss_fftr
		LOGDBG << "Computing FFT..." << endl;
		unsigned int nFFT = nt;
		const int is_inverse_fft = 0;
		// number of FFT frequencies (Nyquist)
		// NOTE: should check consistency
		unsigned int nw = nFFT / 2 + 1;

		// Note: frequency-domain data is stored from dc up to 2pi.
		// so cx_out[0] is the dc bin of the FFT
		// and cx_out[nfft/2] is the Nyquist bin (if exists)                 ???
		double dw = pi / dtWave / nw; // wave frequency interval (rad/s)

		// allocate memory for kiss_fftr
		kiss_fftr_cfg cfg = kiss_fftr_alloc(nFFT, is_inverse_fft, 0, 0);

		// allocate input and output arrays for kiss_fftr
		// (note that kiss_fft_scalar is set to double)
		kiss_fft_scalar* cx_t_in =
		    (kiss_fft_scalar*)malloc(nFFT * sizeof(kiss_fft_scalar));
		kiss_fft_cpx* cx_w_out =
		    (kiss_fft_cpx*)malloc(nw * sizeof(kiss_fft_cpx));
		if (!cx_t_in || !cx_w_out) {
			LOGERR << "Failure allocating "
			       << nFFT * sizeof(kiss_fft_scalar) + nw * sizeof(kiss_fft_cpx)
			       << "bytes for the FFT computation" << endl;
			throw moordyn::mem_error("Insufficient memory");
		}

		// copy wave elevation time series into input vector
		real zetaRMS = 0.0;
		for (unsigned int i = 0; i < nFFT; i++) {
			cx_t_in[i] = waveElev[i];
			zetaRMS += waveElev[i] * waveElev[i];
		}
		zetaRMS = sqrt(zetaRMS / nFFT);

		// perform the real-valued FFT
		kiss_fftr(cfg, cx_t_in, cx_w_out);
		LOGDBG << "Done!" << endl;

		// copy frequencies over from FFT output
		moordyn::complex* zetaC0 =
		    (moordyn::complex*)malloc(nw * sizeof(moordyn::complex));
		if (!zetaC0) {
			LOGERR << "Failure allocating " << nw * sizeof(moordyn::complex)
			       << "bytes for the FFT elevation" << endl;
			throw moordyn::mem_error("Insufficient memory");
		}
		for (unsigned int i = 0; i < nw; i++)
			zetaC0[i] = (real)(cx_w_out[i].r) + i1 * (real)(cx_w_out[i].i);

		// cut frequencies above 0.5 Hz (2 s) to avoid FTT noise getting
		// amplified when moving to other points in the wave field...
		for (unsigned int i = 0; i < nw; i++)
			if (i * dw > 0.5 * 2 * pi)
				zetaC0[i] = 0.0;

		// calculate wave kinematics throughout the grid
		try {
			// make a grid for wave kinematics based on settings in
			// water_grid.txt
			makeGrid(((string)folder + "/water_grid.txt").c_str());
			fillWaveGrid(zetaC0, nw, dw, env->g, env->WtrDpth);
		} catch (...) {
			throw;
		}

		// free things up
		free(cx_t_in);
		free(cx_w_out);
		free(cfg);
		free(zetaC0);
	}

	// Now add in current velocities (add to unsteady wave kinematics)
	if (env->Current == CURRENTS_STEADY_GRID) {
		const string CurrentsFilename = (string)folder + "/current_profile.txt";
		LOGMSG << "Reading currents profile from '" << CurrentsFilename
		       << "'..." << endl;

		vector<string> lines;
		string line;

		ifstream f(CurrentsFilename);
		if (!f.is_open()) {
			LOGERR << "Cannot read the file '" << CurrentsFilename << "'"
			       << endl;
			throw moordyn::input_file_error("Invalid file");
		}
		while (getline(f, line)) {
			lines.push_back(line);
		}
		f.close();

		if (lines.size() < 4) {
			LOGERR << "The file '" << CurrentsFilename
			       << "' should have at least 4 lines" << endl;
			throw moordyn::input_file_error("Invalid file format");
		}

		vector<real> UProfileZ;
		vector<real> UProfileUx;
		vector<real> UProfileUy;
		vector<real> UProfileUz;

		for (unsigned int i = 3; i < lines.size(); i++) {
			vector<string> entries = moordyn::str::split(lines[i]);
			if (entries.size() < 2) {
				LOGERR << "The file '" << CurrentsFilename
				       << "' should have at least 2 columns" << endl;
				throw moordyn::input_file_error("Invalid file format");
			}
			UProfileZ.push_back(atof(entries[0].c_str()));
			UProfileUx.push_back(atof(entries[1].c_str()));

			if (entries.size() >= 3)
				UProfileUy.push_back(atof(entries[2].c_str()));
			else
				UProfileUy.push_back(0.0);

			if (entries.size() >= 4)
				UProfileUz.push_back(atof(entries[3].c_str()));
			else
				UProfileUz.push_back(0.0);
		}
		LOGMSG << "'" << CurrentsFilename << "' parsed" << endl;

		// NOTE: check data

		// interpolate and add data to wave kinematics grid

		if (nx * ny * nz == 0) {
			// A grid hasn't been set up yet, make it based on the read-in z
			// values
			nx = 1;
			px.assign(nx, 0.0);
			ny = 1;
			py.assign(ny, 0.0);
			nz = UProfileZ.size();
			pz.assign(nz, 0.0);
			for (unsigned int i = 0; i < nz; i++)
				pz[i] = UProfileZ[i];

			// set 1 time step to indicate steady data
			nt = 1;
			dtWave = 1.0; // arbitrary entry

			try {
				allocateKinematicsArrays();
			} catch (...) {
				throw;
			}

			// fill in output arrays
			for (unsigned int i = 0; i < nz; i++) {
				ux[0][0][i][0] = UProfileUx[i];
				uy[0][0][i][0] = UProfileUy[i];
				uz[0][0][i][0] = UProfileUz[i];
			}
		} else {
			real fz;
			unsigned izi = 1;
			for (unsigned int iz = 0; iz < nz; iz++) {
				izi = interp_factor(UProfileZ, izi, pz[iz], fz);
				for (unsigned int ix = 0; ix < nx; ix++) {
					for (unsigned int iy = 0; iy < ny; iy++) {
						for (unsigned int it = 0; it < nt; it++) {
							ux[ix][iy][iz][it] +=
							    UProfileUx[izi] * fz +
							    UProfileUx[izi - 1] * (1. - fz);
							uy[ix][iy][iz][it] +=
							    UProfileUy[izi] * fz +
							    UProfileUy[izi - 1] * (1. - fz);
							uz[ix][iy][iz][it] +=
							    UProfileUz[izi] * fz +
							    UProfileUz[izi - 1] * (1. - fz);
						}
					}
				}
			}
		}
	} else if (env->Current == CURRENTS_DYNAMIC_GRID) {
		const string CurrentsFilename =
		    (string)folder + "/current_profile_dynamic.txt";
		LOGMSG << "Reading currents dynamic profile from '" << CurrentsFilename
		       << "'..." << endl;

		vector<string> lines;
		string line;

		ifstream f(CurrentsFilename);
		if (!f.is_open()) {
			LOGERR << "Cannot read the file '" << CurrentsFilename << "'"
			       << endl;
			throw moordyn::input_file_error("Invalid file");
		}
		while (getline(f, line)) {
			lines.push_back(line);
		}
		f.close();

		if (lines.size() < 7) {
			LOGERR << "The file '" << CurrentsFilename
			       << "' should have at least 7 lines" << endl;
			throw moordyn::input_file_error("Invalid file format");
		}

		vector<real> UProfileZ;
		vector<real> UProfileT;
		vector<vector<real>> UProfileUx;
		vector<vector<real>> UProfileUy;
		vector<vector<real>> UProfileUz;

		// this is the depths row
		vector<string> entries = moordyn::str::split(lines[4]);
		const unsigned int nzin = entries.size();
		for (unsigned int i = 0; i < nzin; i++)
			UProfileZ.push_back(atof(entries[i].c_str()));

		// Read the time rows
		const unsigned int ntin = lines.size() - 6;
		UProfileUx = init2DArray(nzin, ntin);
		UProfileUy = init2DArray(nzin, ntin);
		UProfileUz = init2DArray(nzin, ntin);
		for (unsigned int i = 6; i < lines.size(); i++) {
			entries = moordyn::str::split(lines[i]);
			const unsigned int it = i - 6;
			if (entries.size() <= nzin) {
				LOGERR << "The file '" << CurrentsFilename
				       << "' should have at least " << nzin + 1 << " columns"
				       << endl;
				throw moordyn::input_file_error("Invalid file format");
			}
			UProfileT.push_back(atof(entries[0].c_str()));
			for (unsigned int iz = 0; iz < nzin; iz++)
				UProfileUx[iz][it] = atof(entries[iz + 1].c_str());

			if (entries.size() >= 2 * nzin + 1)
				for (unsigned int iz = 0; iz < nzin; iz++)
					UProfileUy[iz][it] = atof(entries[nzin + iz + 1].c_str());
			else
				for (unsigned int iz = 0; iz < nzin; iz++)
					UProfileUy[iz][it] = 0.0;

			if (entries.size() >= 3 * nzin + 1)
				for (unsigned int iz = 0; iz < nzin; iz++)
					UProfileUz[iz][it] =
					    atof(entries[2 * nzin + iz + 1].c_str());
			else
				for (unsigned int iz = 0; iz < nzin; iz++)
					UProfileUz[iz][it] = 0.0;
		}
		LOGMSG << "'" << CurrentsFilename << "' parsed" << endl;

		// check data
		if ((nx * ny * nz != 0) && !nt) {
			LOGERR << "At least one time step of current data read from '"
			       << CurrentsFilename << "', but nt = 0" << endl;
			throw moordyn::invalid_value_error("No time data");
		}

		// interpolate and add data to wave kinematics grid
		if (nx * ny * nz == 0) {
			// A grid hasn't been set up yet, make it based on the read-in z
			// values
			nx = 1;
			px.assign(nx, 0.0);
			ny = 1;
			py.assign(ny, 0.0);
			nz = UProfileZ.size();
			pz.assign(nz, 0.0);
			for (unsigned int i = 0; i < nz; i++)
				pz[i] = UProfileZ[i];

			// set the time step size to be the smallest interval in the
			// inputted times
			dtWave = std::numeric_limits<real>::max();
			for (unsigned int i = 1; i < ntin; i++)
				if (UProfileT[i] - UProfileT[i - 1] < dtWave)
					dtWave = UProfileT[i] - UProfileT[i - 1];
			nt = floor(UProfileT[ntin - 1] / dtWave) + 1;

			try {
				allocateKinematicsArrays();
			} catch (...) {
				throw;
			}

			// fill in output arrays
			real ft;
			unsigned iti = 1;
			for (unsigned int iz = 0; iz < nz; iz++) {
				for (unsigned int it = 0; it < nt; it++) {
					// need to set iti, otherwise it will lock to final t after one pass through
					// initially the upper index we check should always be one timestep ahead of it
					iti = it + 1;  
					iti = interp_factor(UProfileT, iti, it * dtWave, ft);
					ux[0][0][iz][it] = UProfileUx[iz][iti] * ft +
					                   UProfileUx[iz][iti - 1] * (1. - ft);
					uy[0][0][iz][it] = UProfileUy[iz][iti] * ft +
					                   UProfileUy[iz][iti - 1] * (1. - ft);
					uz[0][0][iz][it] = UProfileUz[iz][iti] * ft +
					                   UProfileUz[iz][iti - 1] * (1. - ft);
					// TODO: approximate fluid accelerations using finite
					//       differences
					ax[0][0][iz][it] = 0.0;
					ay[0][0][iz][it] = 0.0;
					az[0][0][iz][it] = 0.0;
				}
			}
		} else // otherwise interpolate read in data and add to existing grid
		       // (dtWave, px, etc are already set in the grid)
		{
			real fz;
			unsigned izi = 1;
			for (unsigned int iz = 0; iz < nz; iz++) {
				izi = interp_factor(UProfileZ, izi, pz[iz], fz);
				real ft;
				unsigned iti = 1;
				for (unsigned int it = 0; it < nt; it++) {
					iti = it + 1;  // Set initial iti
					iti = interp_factor(UProfileT, iti, it * dtWave, ft);
					for (unsigned int ix = 0; ix < nx; ix++) {
						for (unsigned int iy = 0; iy < ny; iy++) {
							ux[ix][iy][iz][it] +=
							    interp2(UProfileUx, izi, iti, fz, ft);
							uy[ix][iy][iz][it] +=
							    interp2(UProfileUy, izi, iti, fz, ft);
							uz[ix][iy][iz][it] +=
							    interp2(UProfileUz, izi, iti, fz, ft);
							// TODO: approximate fluid accelerations using
							//       finite differences
							ax[0][0][iz][it] = 0.0;
							ay[0][0][iz][it] = 0.0;
							az[0][0][iz][it] = 0.0;
						}
					}
				}
			}
		}
	}
}

void
Waves::getWaveKin(real x,
                  real y,
                  real z,
                  vec& U_out,
                  vec& Ud_out,
                  moordyn::real& zeta_out,
                  moordyn::real& PDyn_out)
{
	real fx, fy, fz;

	auto ix = interp_factor(px, x, fx);
	auto iy = interp_factor(py, y, fy);
	auto iz = interp_factor(pz, z, fz);

	unsigned int it = 0;
	real ft = 0.0;
	if (nt > 1) {
		real quot = _t_integrator->GetTime() / dtWave;
		it = floor(quot);
		ft = quot - it;
		it++; // We use the upper bound
		while (it > nt - 1)
			it -= nt;
	}

	zeta_out = interp3(zeta, ix, iy, it, fx, fy, ft);

	U_out[0] = interp4(ux, ix, iy, iz, it, fx, fy, fz, ft);
	U_out[1] = interp4(uy, ix, iy, iz, it, fx, fy, fz, ft);
	U_out[2] = interp4(uz, ix, iy, iz, it, fx, fy, fz, ft);

	Ud_out[0] = interp4(ax, ix, iy, iz, it, fx, fy, fz, ft);
	Ud_out[1] = interp4(ay, ix, iy, iz, it, fx, fy, fz, ft);
	Ud_out[2] = interp4(az, ix, iy, iz, it, fx, fy, fz, ft);

	PDyn_out = interp4(PDyn, ix, iy, iz, it, fx, fy, fz, ft);
}

void
Waves::fillWaveGrid(const moordyn::complex* zetaC0,
                    unsigned int nw,
                    real dw,
                    real g,
                    real h)
{
	// NOTE: should enable wave spreading at some point!
	real beta = 0.0; // WaveDir_in;

	// initialize some frequency-domain wave calc vectors
	vector<real> w(nw, 0.);
	vector<real> k(nw, 0.);
	auto data_size = nw * sizeof(moordyn::complex);
	// Fourier transform of wave elevation
	moordyn::complex* zetaC = (moordyn::complex*)malloc(data_size);
	// Fourier transform of dynamic pressure
	moordyn::complex* PDynC = (moordyn::complex*)malloc(data_size);
	// Fourier transform of wave velocities
	moordyn::complex* UCx = (moordyn::complex*)malloc(data_size);
	moordyn::complex* UCy = (moordyn::complex*)malloc(data_size);
	moordyn::complex* UCz = (moordyn::complex*)malloc(data_size);
	// Fourier transform of wave accelerations
	moordyn::complex* UdCx = (moordyn::complex*)malloc(data_size);
	moordyn::complex* UdCy = (moordyn::complex*)malloc(data_size);
	moordyn::complex* UdCz = (moordyn::complex*)malloc(data_size);
	if (!zetaC || !PDynC || !UCx || !UCy || !UCz || !UdCx || !UdCy || !UdCz) {
		LOGERR << "Failure allocating " << 8 * data_size
		       << "bytes for the FFT data" << endl;
		throw moordyn::mem_error("Insufficient memory");
	}

	// The number of wave time steps to be calculated
	nt = 2 * (nw - 1);

	// single-sided spectrum for real fft
	for (unsigned int i = 0; i < nw; i++)
		w[i] = (real)i * dw;

	LOGMSG << "Wave frequencies from " << w[0] << " rad/s to " << w[nw - 1]
	       << " rad/s in increments of " << dw << " rad/s" << endl;

	LOGDBG << "Wave numbers in rad/m are ";
	for (unsigned int I = 0; I < nw; I++) {
		k[I] = WaveNumber(w[I], g, h);
		LOGDBG << k[I] << ", ";
	}
	LOGDBG << endl;

	LOGDBG << "   nt = " << nt << ", h = " << h << endl;

	// precalculates wave kinematics for a given set of node points for a series
	// of time steps
	LOGDBG << "Making wave Kinematics (iFFT)..." << endl;

	// start the FFT stuff using kiss_fft
	unsigned int nFFT = nt;
	const int is_inverse_fft = 1;

	// allocate memory for kiss_fftr
	kiss_fftr_cfg cfg = kiss_fftr_alloc(nFFT, is_inverse_fft, NULL, NULL);

	// allocate input and output arrays for kiss_fftr  (note that
	// kiss_fft_scalar is set to double)
	kiss_fft_cpx* cx_w_in = (kiss_fft_cpx*)malloc(nw * sizeof(kiss_fft_cpx));
	kiss_fft_scalar* cx_t_out =
	    (kiss_fft_scalar*)malloc(nFFT * sizeof(kiss_fft_scalar));
	if (!cx_w_in || !cx_t_out) {
		LOGERR << "Failure allocating "
		       << nw * sizeof(kiss_fft_cpx) + nFFT * sizeof(kiss_fft_scalar)
		       << "bytes for the iFFT" << endl;
		throw moordyn::mem_error("Insufficient memory");
	}

	// calculating wave kinematics for each grid point

	for (unsigned int ix = 0; ix < nx; ix++) {
		real x = px[ix];
		for (unsigned int iy = 0; iy < ny; iy++) {
			real y = py[iy];
			// wave elevation
			// handle all (not just positive-frequency half?) of spectrum?
			for (unsigned int I = 0; I < nw; I++) {
				// shift each zetaC to account for location
				const real l = cos(beta) * x + sin(beta) * y;
				// NOTE: check minus sign in exponent!
				zetaC[I] = zetaC0[I] * exp(-i1 * (k[I] * l));
			}

			// IFFT the wave elevation spectrum
			doIFFT(cfg, nFFT, cx_w_in, cx_t_out, zetaC, zeta[ix][iy]);

			// wave velocities and accelerations
			for (unsigned int iz = 0; iz < nz; iz++) {
				real z = pz[iz];

				// Loop through the positive frequency components (including
				// zero) of the Fourier transforms
				for (unsigned int I = 0; I < nw; I++) {
					// Calculate
					//     SINH( k*( z + h ) )/SINH( k*h )
					//     COSH( k*( z + h ) )/SINH( k*h )
					//     COSH( k*( z + h ) )/COSH( k*h )
					real SINHNumOvrSIHNDen;
					real COSHNumOvrSIHNDen;
					real COSHNumOvrCOSHDen;

					if (k[I] == 0.0) {
						// The shallow water formulation is ill-conditioned;
						// thus, the known value of unity is returned.
						SINHNumOvrSIHNDen = 1.0;
						COSHNumOvrSIHNDen = 99999.0;
						COSHNumOvrCOSHDen = 99999.0;
					} else if (k[I] * h > 89.4) {
						// The shallow water formulation will trigger a floating
						// point overflow error; however, for
						// h > 14.23 * wavelength (since k = 2 * Pi /
						// wavelength) we can use the numerically-stable deep
						// water formulation instead.
						SINHNumOvrSIHNDen = exp(k[I] * z);
						COSHNumOvrSIHNDen = exp(k[I] * z);
						COSHNumOvrCOSHDen =
						    exp(k[I] * z) + exp(-k[I] * (z + 2.0 * h));
					} else if (-k[I] * h > 89.4) {
						// @mth: added negative k case
						// NOTE: CHECK CORRECTNESS
						SINHNumOvrSIHNDen = -exp(-k[I] * z);
						COSHNumOvrSIHNDen = -exp(-k[I] * z);
						COSHNumOvrCOSHDen =
						    -exp(-k[I] * z) + exp(-k[I] * (z + 2.0 * h));
					} else {
						// shallow water formulation
						SINHNumOvrSIHNDen =
						    sinh(k[I] * (z + h)) / sinh(k[I] * h);
						COSHNumOvrSIHNDen =
						    cosh(k[I] * (z + h)) / sinh(k[I] * h);
						COSHNumOvrCOSHDen =
						    cosh(k[I] * (z + h)) / cosh(k[I] * h);
					}

					// Fourier transform of dynamic pressure
					PDynC[I] = rho_w * g * zetaC[I] * COSHNumOvrCOSHDen;

					// Fourier transform of wave velocities
					// (note: need to multiply by abs(w) to avoid inverting
					//  negative half of spectrum) <<< ???
					UCx[I] = w[I] * zetaC[I] * COSHNumOvrSIHNDen * cos(beta);
					UCy[I] = w[I] * zetaC[I] * COSHNumOvrSIHNDen * sin(beta);
					UCz[I] = i1 * w[I] * zetaC[I] * SINHNumOvrSIHNDen;

					// Fourier transform of wave accelerations
					// NOTE: should confirm correct signs of +/- halves of
					// spectrum here
					UdCx[I] = i1 * w[I] * UCx[I];
					UdCy[I] = i1 * w[I] * UCy[I];
					UdCz[I] = i1 * w[I] * UCz[I];
				}

				// NOTE: could handle negative-frequency half of spectrum with
				// for (int I=nw/2+1; I<nw; I++) <<<

				// IFFT the dynamic pressure
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, PDynC, PDyn[ix][iy][iz]);
				// IFFT the wave velocities
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCx, ux[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCy, uy[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCz, uz[ix][iy][iz]);
				// IFFT the wave accelerations
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCx, ax[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCy, ay[ix][iy][iz]);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCz, az[ix][iy][iz]);

				// NOTE: wave stretching stuff would maybe go here?? <<<
			}
		}
	}

	free(cx_w_in);
	free(cx_t_out);
	free(cfg);
	free(zetaC);
	free(PDynC);
	free(UCx);
	free(UCy);
	free(UCz);
	free(UdCx);
	free(UdCy);
	free(UdCz);

	LOGDBG << "Done!" << endl;
}

} // ::moordyn

// =============================================================================
//
//                     ||                     ||
//                     ||        C API        ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

/// Check that the provided waves instance is not Null
#define CHECK_WAVES(w)                                                         \
	if (!w) {                                                                  \
		cerr << "Null waves instance received in " << __FUNC_NAME__ << " ("    \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetWavesKin(MoorDynWaves waves,
                    double x,
                    double y,
                    double z,
                    double U[3],
                    double Ud[3],
                    double* zeta,
                    double* PDyn)
{
	CHECK_WAVES(waves);
	moordyn::vec u, ud;
	moordyn::real h, p;
	((moordyn::Waves*)waves)->getWaveKin(x, y, z, u, ud, h, p);
	moordyn::vec2array(u, U);
	moordyn::vec2array(ud, Ud);
	*zeta = h;
	*PDyn = p;
	return MOORDYN_SUCCESS;
}

double DECLDIR
WaveNumber(double Omega, double g, double h)
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

	if (Omega == 0.0) // When .TRUE., the formulation below is ill-conditioned;
	                  // thus, the known value of zero is returned.
	{
		k = 0.0;
		return k;
	} else // Omega > 0.0 solve for the wavenumber as usual.
	{
		double C = Omega * Omega * h / g;
		double CC = C * C;

		// Find X0:
		if (C <= 2.0) {
			X0 = sqrt(C) * (1.0 + C * (0.169 + (0.031 * C)));
		} else {
			double E2 = exp(-2.0 * C);
			X0 = C * (1.0 + (E2 * (2.0 - (12.0 * E2))));
		}

		// Find the WaveNumber:

		if (C <= 4.8) {
			double C2 = CC - X0 * X0;
			double A = 1.0 / (C - C2);
			double B = A * ((0.5 * log((X0 + C) / (X0 - C))) - X0);

			k = (X0 - (B * C2 * (1.0 + (A * B * C * X0)))) / h;
		} else {
			k = X0 / h;
		}

		if (Omega < 0)
			k = -k; // @mth: modified to return negative k for negative Omega
		return k;
	}
}
