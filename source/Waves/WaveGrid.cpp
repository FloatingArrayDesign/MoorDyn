#include "WaveGrid.hpp"
#include "../Waves.h"
#include "../Waves.hpp"
#include "MoorDyn2.hpp"
#include "Util/Interp.hpp"
#include "WaveOptions.hpp"
#include "kiss_fftr.h"
#include <exception>
#include <memory>

#if defined WIN32 && defined max
// We must avoid max messes up with std::numeric_limits<>::max()
#undef max
#endif

#include <limits>

namespace moordyn {
namespace waves {

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
       std::vector<kiss_fft_cpx>& cx_w_in,
       std::vector<kiss_fft_scalar>& cx_t_out,
       const std::vector<moordyn::complex>& inputs,
       std::vector<real>& outputs)
{
	unsigned int nw = nFFT / 2 + 1;

	// copy frequency-domain data into input vector
	// NOTE: (simpler way to do this, or bypass altogether?)
	for (unsigned int i = 0; i < nw; i++) {
		cx_w_in[i].r = std::real(inputs[i]);
		cx_w_in[i].i = std::imag(inputs[i]);
	}

	// input freqdata has nfft/2 + 1 complex points
	// output timedata has nfft scalar points

	kiss_fftri(cfg, cx_w_in.data(), cx_t_out.data());

	// copy out the IFFT data to the time series
	for (unsigned int i = 0; i < nFFT; i++) {
		// NOTE: is dividing by nFFT correct? (prevously was nw)
		outputs[i] = cx_t_out[i] / (real)nFFT;
	}

	return;
}

/** @brief Fill the wave grid using time series data from the based on the fft
 * data (MORE RECENT)
 * @param zetaC0 Amplitude of each frequency component
 * @param nw Number of wave components
 * @param dw The difference in frequency between consequtive modes
 * @param g Gravity accelerations
 * @param h Water depth
 * @throws moordyn::mem_error If there were problems allocating memory
 */
std::unique_ptr<WaveGrid>
fillWaveGrid(std::unique_ptr<WaveGrid> waveGrid,

             const moordyn::complex* zetaC0,
             unsigned int nw,
             const std::vector<real>& betas,
             real dw,
             EnvCondRef env,
             moordyn::Log* _log)
{
	// NOTE: should enable wave spreading at some point!
	// real beta = 0.0; // WaveDir_in;

	// initialize some frequency-domain wave calc vectors
	vector<real> w(nw, 0.);
	vector<real> k(nw, 0.);

	// Fourier transform of wave elevation
	std::vector<moordyn::complex> zetaC(nw);
	// Fourier transform of dynamic pressure
	std::vector<moordyn::complex> PDynC(nw);
	// Fourier transform of wave velocities
	std::vector<moordyn::complex> UCx(nw);
	std::vector<moordyn::complex> UCy(nw);
	std::vector<moordyn::complex> UCz(nw);
	// Fourier transform of wave accelerations
	std::vector<moordyn::complex> UdCx(nw);
	std::vector<moordyn::complex> UdCy(nw);
	std::vector<moordyn::complex> UdCz(nw);

	// The number of wave time steps to be calculated
	// nt = 2 * (nw - 1);
	auto nt = waveGrid->nt;
	// This computes the time distance between samples returned by the ifft
	// We entirely ignore the env->dtWave value in this case for now
	waveGrid->dtWave = ((2 * pi) / dw) / nt;
	LOGMSG << "in new fillWaveGrid, setting waveGrid->dtWave to be "
	       << waveGrid->dtWave << endl;
	auto h = env->WtrDpth;
	auto g = env->g;
	auto rho_w = env->rho_w;

	// single-sided spectrum for real fft
	for (unsigned int i = 0; i < nw; i++)
		w[i] = (real)i * dw;

	LOGMSG << "Wave frequencies from " << w[0] << " rad/s to " << w[nw - 1]
	       << " rad/s in increments of " << dw << " rad/s" << endl;

	LOGDBG << "Wave numbers in rad/m are ";
	for (unsigned int I = 0; I < nw; I++) {
		k[I] = WaveNumber(w[I], env->g, h);
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
	std::vector<kiss_fft_cpx> cx_w_in(nw);
	std::vector<kiss_fft_scalar> cx_t_out(nFFT);

	// calculating wave kinematics for each grid point
	const auto& px = waveGrid->Px();
	const auto& py = waveGrid->Py();
	const auto& pz = waveGrid->Pz();
	for (unsigned int ix = 0; ix < waveGrid->nx; ix++) {
		real x = px[ix];
		for (unsigned int iy = 0; iy < waveGrid->ny; iy++) {
			real y = py[iy];
			// wave elevation
			// handle all (not just positive-frequency half?) of spectrum?
			for (unsigned int I = 0; I < nw; I++) {
				// shift each zetaC to account for location
				const real l = cos(betas[I]) * x + sin(betas[I]) * y;
				// NOTE: check minus sign in exponent!
				zetaC[I] = zetaC0[I] * exp(-i1 * (k[I] * l));
			}

			// IFFT the wave elevation spectrum
			doIFFT(
			    cfg, nFFT, cx_w_in, cx_t_out, zetaC, waveGrid->Zetas()[ix][iy]);

			// wave velocities and accelerations
			for (unsigned int iz = 0; iz < waveGrid->nz; iz++) {
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
						COSHNumOvrCOSHDen = 1.0;
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
					UCx[I] =
					    w[I] * zetaC[I] * COSHNumOvrSIHNDen * cos(betas[I]);
					UCy[I] =
					    w[I] * zetaC[I] * COSHNumOvrSIHNDen * sin(betas[I]);
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
				doIFFT(cfg,
				       nFFT,
				       cx_w_in,
				       cx_t_out,
				       PDynC,
				       waveGrid->PDyn()[ix][iy][iz]);
				// IFFT the wave velocities
				std::vector<real> x_vel(nt);
				std::vector<real> y_vel(nt);
				std::vector<real> z_vel(nt);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCx, x_vel);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCy, y_vel);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UCz, z_vel);
				for (unsigned int i = 0; i < nt; i++) {
					waveGrid->WaveVel()[ix][iy][iz][i] =
					    vec3(x_vel[i], y_vel[i], z_vel[i]);
				}

				// IFFT the wave accelerations
				std::vector<real> x_acc(nt);
				std::vector<real> y_acc(nt);
				std::vector<real> z_acc(nt);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCx, x_acc);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCy, y_acc);
				doIFFT(cfg, nFFT, cx_w_in, cx_t_out, UdCz, z_acc);
				for (unsigned int i = 0; i < nt; i++) {
					waveGrid->WaveAcc()[ix][iy][iz][i] =
					    vec3(x_acc[i], y_acc[i], z_acc[i]);
				}
				// NOTE: wave stretching stuff would maybe go here?? <<<
			}
		}
	}

	free(cfg);

	LOGDBG << "Done!" << endl;
	return waveGrid;
}

/** @brief Read the grid file and return the three axes
 *
 * The grid is defined in a tabulated file (separator=' '). That file has 3
 * head lines, which are ignored, and then 3 lines defining the grid points
 * in x, y, z.
 *
 * Each one can be defined in 3 ways:
 *   - Defining just a point in 0
 *   - Defining the list of coordinates
 *   - Defining the boundaries and the number of equispaced points
 *
 * @param filepath The definition file path
 * @throws moordyn::input_file_error If the input file cannot be read, or if
 * the fileis ill-formatted
 * @throws moordyn::invalid_value_error If invalid values for the grid
 * initialization are found
 */
std::array<std::vector<real>, 3>
rectilinearGridFromFile(std::filesystem::path filepath, moordyn::Log* _log)
{

	LOGMSG << "Reading waves coordinates grid from '" << filepath << "'..."
	       << endl;

	// --------------------- read grid data from file ------------------------
	vector<string> lines;
	string line;

	lines = moordyn::fileIO::fileToLines(filepath);

	if (lines.size() < 9) {
		LOGERR << "The waves grid file '" << filepath << "' has only "
		       << lines.size() << "lines, but at least 9 are required" << endl;
		throw moordyn::input_file_error("Invalid file format");
	}

	vector<string> entries;
	Waves::coordtypes coordtype;

	entries = moordyn::str::split(lines[3]);
	coordtype = (Waves::coordtypes)stoi(entries[0]);
	entries = moordyn::str::split(lines[4]);
	auto px = gridAxisCoords(coordtype, entries);
	if (px.empty()) {
		LOGERR << "Invalid entry for the grid x values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	entries = moordyn::str::split(lines[5]);
	coordtype = (Waves::coordtypes)stoi(entries[0]);
	entries = moordyn::str::split(lines[6]);
	auto py = gridAxisCoords(coordtype, entries);
	if (py.empty()) {
		LOGERR << "Invalid entry for the grid y values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	entries = moordyn::str::split(lines[7]);
	coordtype = (Waves::coordtypes)stoi(entries[0]);
	entries = moordyn::str::split(lines[8]);
	auto pz = gridAxisCoords(coordtype, entries);
	if (pz.empty()) {
		LOGERR << "Invalid entry for the grid z values in file '" << filepath
		       << "'" << endl;
		throw moordyn::invalid_value_error("Invalid line");
	}

	LOGDBG << "Setup the waves grid with " << px.size() << " x " << py.size()
	       << " x " << pz.size() << " points " << endl;

	LOGMSG << "'" << filepath << "' parsed" << endl;

	return { px, py, pz };
}

std::unique_ptr<WaveGrid>
constructWaveGridSpectrumData(const std::string& folder,
                              const EnvCondRef env,
                              moordyn::Log* _log)
{

	const string WaveFilename = folder + "/wave_frequencies.txt";
	LOGMSG << "Reading waves FFT from '" << WaveFilename << "'..." << endl;

	// NOTE: need to decide what inputs/format to expect in file
	// (1vs2-sided spectrum?)

	waves::DiscreteWaveSpectrum spectrum = spectrumFromFile(WaveFilename, _log);
	LOGMSG << "'" << WaveFilename << "' parsed" << endl;

	if (spectrum[0].omega != 0.0) {
		LOGERR << "The first shall be 0 rad/s" << endl;
		throw moordyn::invalid_value_error("Invalid frequencies");
	}

	const vector<waves::FrequencyComponent> evenFreqComps =
	    spectrum.interpEvenlySpaced();
	// LOGMSG << "Frequency Spectrum: \n";
	// for(auto& freqComp : evenFreqComps) {
	// 	LOGMSG << "freq(" << freqComp.omega << ") = " << freqComp.amplitude
	// << endl;
	// }
	vector<moordyn::complex> zetaC0(evenFreqComps.size());
	vector<real> betas(evenFreqComps.size());
	for (unsigned int i = 0; i < evenFreqComps.size(); i++) {
		zetaC0[i] =
		    evenFreqComps[i].amplitude * (double)(evenFreqComps.size() - 1);
		betas[i] = evenFreqComps[i].beta;
	}

	auto dw = evenFreqComps.at(1).omega - evenFreqComps.at(0).omega;

	// The number of wave time steps to be calculated
	auto nt = static_cast<unsigned int>(2 * (evenFreqComps.size() - 1));

	auto [px, py, pz] =
	    rectilinearGridFromFile(folder + "/water_grid.txt", _log);

	auto waveGrid = make_unique<WaveGrid>(
	    _log, px, py, pz, nt, env->waterKinOptions.dtWave);
	waveGrid->allocateKinematicArrays();

	// calculate wave kinematics throughout the grid
	return fillWaveGrid(std::move(waveGrid),
	                    zetaC0.data(),
	                    static_cast<unsigned int>(zetaC0.size()),
	                    betas,
	                    dw,
	                    env,
	                    _log);
}
std::unique_ptr<WaveGrid>
constructWaveGridElevationData(const std::string& folder,
                               const EnvCondRef env,
                               moordyn::Log* _log)
{

	// load wave elevation time series from file (similar to what's done in
	// GenerateWaveExtnFile.py, and was previously in misc2.cpp)
	const string WaveFilename = folder + "/wave_elevation.txt";
	LOGMSG << "Reading waves elevation from '" << WaveFilename << "'..."
	       << endl;

	vector<string> lines;
	try {
		lines = moordyn::fileIO::fileToLines(WaveFilename);
	} catch (moordyn::input_file_error& err) {
		LOGERR << "Cannot read the file '" << WaveFilename << "'" << endl;
		std::stringstream ss;
		ss << "Waves::setup failed to read wave_elevation.txt file: "
		   << err.what();
		throw input_file_error(ss.str().c_str());
	}

	// should add error checking.  two columns of data, and time column must
	// start at zero?

	vector<real> wavetimes;
	vector<real> waveelevs;

	for (auto line : lines) {
		vector<string> entries = moordyn::str::split(line);
		if (entries.size() < 2) {
			LOGERR << "The file '" << WaveFilename << "' should have 2 columns"
			       << endl;
			throw moordyn::input_file_error("Invalid file format");
		}
		wavetimes.push_back(stod(entries[0]));
		waveelevs.push_back(stod(entries[1]));
	}
	LOGMSG << "'" << WaveFilename << "' parsed" << endl;

	auto dtWave = env->waterKinOptions.dtWave;
	// downsample to dtWave
	// this makes the implicit assumption that dtWave >= interval between
	// samples the 1 extra is for the point at zero, [0.0, 1.0, 2.0] is 3
	// points even though 2.0/1.0 = 2
	unsigned int nt = floor(wavetimes.back() / dtWave) + 1;
	LOGDBG << "Number of wave time samples = " << nt << "(" << wavetimes.size()
	       << " samples provided in the file)" << endl;

	vector<real> waveTime(nt, 0.0);
	vector<real> waveElev(nt, 0.0);

	for (unsigned int i = 0; i < nt; i++)
		waveTime[i] = i * dtWave;

	moordyn::interp(wavetimes, waveelevs, waveTime, waveElev);

	// ensure N is even
	// this is a requirement of kiss_fft
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
	// cx_out[nfft/2] = pi (rad/s) nfft = T = 16, time.back() = 15
	// dw  = cx_out[nfft/2] / (nff/2)
	// dw = pi/8
	// double dw = pi / dtWave / nw; // wave frequency interval (rad/s)
	double dw =
	    pi / (dtWave * (int)(nt / 2)); // wave frequency interval (rad/s)

	// allocate memory for kiss_fftr
	kiss_fftr_cfg cfg = kiss_fftr_alloc(nFFT, is_inverse_fft, 0, 0);

	// allocate input and output arrays for kiss_fftr
	// (note that kiss_fft_scalar is set to double)
	std::vector<kiss_fft_scalar> cx_t_in(nFFT);
	std::vector<kiss_fft_cpx> cx_w_out(nw);

	// copy wave elevation time series into input vector
	real zetaRMS = 0.0;
	for (unsigned int i = 0; i < nFFT; i++) {
		cx_t_in[i] = waveElev[i];
		zetaRMS += waveElev[i] * waveElev[i];
	}
	zetaRMS = sqrt(zetaRMS / nFFT);

	// perform the real-valued FFT
	kiss_fftr(cfg, cx_t_in.data(), cx_w_out.data());
	LOGDBG << "Done!" << endl;

	free(cfg);
	// copy frequencies over from FFT output
	std::vector<moordyn::complex> zetaC0(nw);
	for (unsigned int i = 0; i < nw; i++)
		zetaC0[i] = (real)(cx_w_out[i].r) + i1 * (real)(cx_w_out[i].i);

	// cut frequencies above 0.5 Hz (2 s) to avoid FTT noise getting
	// amplified when moving to other points in the wave field...
	for (unsigned int i = 0; i < nw; i++)
		if (i * dw > 0.5 * 2 * pi)
			zetaC0[i] = 0.0;

	std::unique_ptr<WaveGrid> waveGrid{};
	// calculate wave kinematics throughout the grid
	// make a grid for wave kinematics based on settings in
	// water_grid.txt
	auto [px, py, pz] =
	    rectilinearGridFromFile(folder + "/water_grid.txt", _log);

	waveGrid = make_unique<WaveGrid>(_log, px, py, pz, nt, dtWave);
	waveGrid->allocateKinematicArrays();
	// makeGrid(((string)folder + "/water_grid.txt").c_str());
	std::vector<real> betas(nw, 0);
	return fillWaveGrid(
	    std::move(waveGrid), zetaC0.data(), nw, betas, dw, env, _log);
}

std::unique_ptr<CurrentGrid>
constructSteadyCurrentGrid(const std::string& folder,
                           const EnvCondRef env,
                           moordyn::Log* _log)
{

	const string CurrentsFilename = folder + "/current_profile.txt";
	LOGMSG << "Reading currents profile from '" << CurrentsFilename << "'..."
	       << endl;

	vector<string> lines;
	string line;

	try {
		lines = moordyn::fileIO::fileToLines(CurrentsFilename);
	} catch (moordyn::input_file_error& err) {
		LOGERR << "Cannot read the file '" << CurrentsFilename << "'" << endl;
		std::stringstream ss;
		ss << "constructSteadyCurrentGrid failed to read currents_profile.txt "
		      "file: "
		   << err.what();
		throw input_file_error(ss.str().c_str());
	}

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
		UProfileZ.push_back(stod(entries[0]));
		UProfileUx.push_back(stod(entries[1]));

		if (entries.size() >= 3)
			UProfileUy.push_back(stod(entries[2]));
		else
			UProfileUy.push_back(0.0);

		if (entries.size() >= 4)
			UProfileUz.push_back(stod(entries[3]));
		else
			UProfileUz.push_back(0.0);
	}
	LOGMSG << "'" << CurrentsFilename << "' parsed" << endl;

	// NOTE: check data

	std::vector<real> px = { 0.0 };
	std::vector<real> py = { 0.0 };
	// set 1 time step to indicate steady data
	unsigned int nt = 1;
	real dtWave = 1.0; // arbitrary entry

	auto currentGrid =
	    make_unique<CurrentGrid>(_log, px, py, UProfileZ, nt, dtWave);
	currentGrid->allocateKinematicArrays();

	// fill in output arrays
	for (unsigned int i = 0; i < currentGrid->nz; i++) {
		currentGrid->CurrentVel()[0][0][i][0] =
		    vec3(UProfileUx[i], UProfileUy[i], UProfileUz[i]);
	}
	return currentGrid;
}

std::unique_ptr<CurrentGrid>
constructDynamicCurrentGrid(const std::string& folder,
                            const EnvCondRef env,
                            moordyn::Log* _log)
{

	const string CurrentsFilename = folder + "/current_profile_dynamic.txt";
	LOGMSG << "Reading currents dynamic profile from '" << CurrentsFilename
	       << "'..." << endl;

	vector<string> lines;

	try {
		lines = moordyn::fileIO::fileToLines(CurrentsFilename);
	} catch (moordyn::input_file_error& err) {
		LOGERR << "Cannot read the file '" << CurrentsFilename << "'" << endl;
		std::stringstream ss;
		ss << "Waves::setup failed to read currents_profile_dynamic.txt "
		      "file: "
		   << err.what();
		throw input_file_error(ss.str().c_str());
	}

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
	const unsigned int nzin = static_cast<unsigned int>(entries.size());
	for (unsigned int i = 0; i < nzin; i++)
		UProfileZ.push_back(stof(entries[i]));

	// Read the time rows
	const unsigned int ntin = static_cast<unsigned int>(lines.size() - 6);
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
		UProfileT.push_back(stod(entries[0]));
		for (unsigned int iz = 0; iz < nzin; iz++)
			UProfileUx[iz][it] = stod(entries[iz + 1]);

		if (entries.size() >= 2 * nzin + 1)
			for (unsigned int iz = 0; iz < nzin; iz++)
				UProfileUy[iz][it] = stod(entries[nzin + iz + 1]);
		else
			for (unsigned int iz = 0; iz < nzin; iz++)
				UProfileUy[iz][it] = 0.0;

		if (entries.size() >= 3 * nzin + 1)
			for (unsigned int iz = 0; iz < nzin; iz++)
				UProfileUz[iz][it] = stod(entries[2 * nzin + iz + 1]);
		else
			for (unsigned int iz = 0; iz < nzin; iz++)
				UProfileUz[iz][it] = 0.0;
	}
	LOGMSG << "'" << CurrentsFilename << "' parsed" << endl;

	// A grid hasn't been set up yet, make it based on the read-in z
	// values

	std::vector<real> px = { 0.0 };
	std::vector<real> py = { 0.0 };

	// set the time step size to be the smallest interval in the
	// inputted times
	// real dtWave = std::numeric_limits<real>::max();

	real dtWave = std::numeric_limits<real>::max();
	for (unsigned int i = 1; i < ntin; i++)
		if (UProfileT[i] - UProfileT[i - 1] < dtWave)
			dtWave = UProfileT[i] - UProfileT[i - 1];
	unsigned int nt = floor(UProfileT[ntin - 1] / dtWave) + 1;

	auto currentGrid =
	    make_unique<CurrentGrid>(_log, px, py, UProfileZ, nt, dtWave);
	currentGrid->allocateKinematicArrays();

	// fill in output arrays
	real ft;
	for (unsigned int iz = 0; iz < currentGrid->nz; iz++) {
		unsigned iti = 1;
		for (unsigned int it = 0; it < currentGrid->nt; it++) {
			iti = interp_factor(UProfileT, iti, it * dtWave, ft);

			auto x = lerp(UProfileUx[iz][iti - 1], UProfileUx[iz][iti], ft);
			auto y = lerp(UProfileUy[iz][iti - 1], UProfileUy[iz][iti], ft);
			auto z = lerp(UProfileUz[iz][iti - 1], UProfileUz[iz][iti], ft);
			currentGrid->CurrentVel()[0][0][iz][it] = vec3(x, y, z);
			// TODO: approximate fluid accelerations using finite
			//       differences
			currentGrid->CurrentAcc()[0][0][iz][it] = vec3::Zero();
		}
	}

	return currentGrid;
}

std::unique_ptr<CurrentGrid>
construct4DCurrentGrid(const std::string& folder,
                       const EnvCondRef env,
                       moordyn::Log* _log)
{

	const string CurrentsFilename = folder + "current_profile_4d.txt";
	LOGMSG << "Reading 4d currents dynamic profile from '" << CurrentsFilename
	       << "'..." << endl;

	vector<string> lines;

	try {
		lines = moordyn::fileIO::fileToLines(CurrentsFilename);
	} catch (moordyn::input_file_error& err) {
		LOGERR << "Cannot read the file '" << CurrentsFilename << "'" << endl;
		std::stringstream ss;
		ss << "Waves::setup failed to read currents_profile_4d.txt "
		      "file: "
		   << err.what();
		throw input_file_error(ss.str().c_str());
	}

	// A better check here is that the grid is at least as large as the
	// waves grid, i.e. nxin * nyin * nzin * ntin >= size of waves
	if (lines.size() < 7) { // TODO: Remove this check? Depends on final format
		LOGERR << "The file '" << CurrentsFilename
		       << "' should have at least 7 lines" << endl;
		throw moordyn::input_file_error("Invalid file format");
	}

	// The first line must contain the number of x, y, z, t values in the
	// input meshgrid (1-indexed, space delimited)
	vector<string> gridDimensions = moordyn::str::split(lines[0]);
	unsigned int nxCurGrid = stoul(gridDimensions[0]);
	unsigned int nyCurGrid = stoul(gridDimensions[1]);
	unsigned int nzCurGrid = stoul(gridDimensions[2]);
	unsigned int ntCurGrid = stoul(gridDimensions[3]);

	// Next four rows specify grid points x, y, t, z
	vector<real> UProfileX;
	vector<real> UProfileY;
	vector<real> UProfileZ;
	vector<real> UProfileT;

	// Need to keep track of

	// And a map to keep track of coord->idx mapping...
	map<string, unsigned int> XMap;
	map<string, unsigned int> YMap;
	map<string, unsigned int> ZMap;
	map<string, unsigned int> TMap;

	auto entry = moordyn::str::split(lines[1]);
	for (unsigned int i = 0; i < nxCurGrid; i++) {
		UProfileX.push_back(stod(entry[i]));
		XMap[entry[i]] = i;
	}

	entry = moordyn::str::split(lines[2]);
	for (unsigned int i = 0; i < nyCurGrid; i++) {
		UProfileY.push_back(stod(entry[i]));
		YMap[entry[i]] = i;
	}

	entry = moordyn::str::split(lines[3]);
	for (unsigned int i = 0; i < nzCurGrid; i++) {
		UProfileZ.push_back(stod(entry[i]));
		ZMap[entry[i]] = i;
	}

	entry = moordyn::str::split(lines[4]);
	for (unsigned int i = 0; i < ntCurGrid; i++) {
		UProfileT.push_back(stod(entry[i]));
		TMap[entry[i]] = i;
	}

	// Need 3 4D grids - one for each component of current velocity:
	Vec4D<real> currentGridUx =
	    init4DArray(nxCurGrid, nyCurGrid, nzCurGrid, ntCurGrid);
	Vec4D<real> currentGridUy =
	    init4DArray(nxCurGrid, nyCurGrid, nzCurGrid, ntCurGrid);
	Vec4D<real> currentGridUz =
	    init4DArray(nxCurGrid, nyCurGrid, nzCurGrid, ntCurGrid);

	// Number of points in the current grid (important for iteration):
	unsigned int nCurGridPoints = nxCurGrid * nyCurGrid * nzCurGrid * ntCurGrid;

	// Need to ensure that there is an entry for each gridpoint:
	if (lines.size() < nCurGridPoints + 5) {
		LOGERR << "The file'" << CurrentsFilename
		       << "' should have a line for each gridpoint\n";
		throw moordyn::input_file_error("Invalid file format\n");
	}

	// Read values into 4D array
	for (int i = 6; i < lines.size(); i++) {
		vector<string> entry = moordyn::str::split(lines[i]);

		// Need to get indices (don't match coord values)
		auto ix = XMap[entry[0]];
		auto iy = YMap[entry[1]];
		auto iz = ZMap[entry[2]];
		auto it = TMap[entry[3]];

		// Need to set velocity components in three separate vectors
		// at the coordinate above
		currentGridUx[ix][iy][iz][it] = stod(entry[4]);
		currentGridUy[ix][iy][iz][it] = stod(entry[5]);
		currentGridUz[ix][iy][iz][it] = stod(entry[6]);
	}

	// set the time step size to be the smallest interval in the
	// inputted times
	real dtWave = std::numeric_limits<real>::max();
	for (unsigned int i = 1; i < ntCurGrid; i++)
		if (UProfileT[i] - UProfileT[i - 1] < dtWave)
			dtWave = UProfileT[i] - UProfileT[i - 1];

	// set number of timesteps
	unsigned int nt = floor(UProfileT[ntCurGrid - 1] / dtWave) + 1;

	auto currentGrid = make_unique<CurrentGrid>(
	    _log, UProfileX, UProfileY, UProfileZ, nt, dtWave);
	currentGrid->allocateKinematicArrays();

	// fill in output arrays
	real ft;
	for (unsigned int ix = 0; ix < currentGrid->nx; ix++) {
		for (unsigned int iy = 0; iy < currentGrid->ny; iy++) {
			for (unsigned int iz = 0; iz < currentGrid->nz; iz++) {
				unsigned iti = 1;
				for (unsigned int it = 0; it < currentGrid->nt; it++) {
					iti = interp_factor(UProfileT, iti, it * dtWave, ft);

					auto x = lerp(currentGridUx[ix][iy][iz][iti - 1],
					              currentGridUx[ix][iy][iz][iti],
					              ft);
					auto y = lerp(currentGridUy[ix][iy][iz][iti - 1],
					              currentGridUy[ix][iy][iz][iti],
					              ft);
					auto z = lerp(currentGridUz[ix][iy][iz][iti - 1],
					              currentGridUz[ix][iy][iz][iti],
					              ft);
					currentGrid->CurrentVel()[ix][iy][iz][it] = vec3(x, y, z);
					// TODO: approximate fluid accelerations using finite
					//       differences
					currentGrid->CurrentAcc()[ix][iy][iz][it] = vec3::Zero();
				}
			}
		}
	}
	return currentGrid;
}

} // namespace waves
} // namespace moordyn
