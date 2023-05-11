#include "WaveSpectrum.hpp"
#include "Misc.hpp"
#include "Waves/WaveSpectrum.hpp"
#include "Util/Interp.hpp"
#include <cmath>
#include <limits>

namespace moordyn {

namespace waves {

DiscreteWaveSpectrum::DiscreteWaveSpectrum()
  : components()
{
}

DiscreteWaveSpectrum::~DiscreteWaveSpectrum() {}

void
DiscreteWaveSpectrum::addFreqComp(FrequencyComponent comp)
{
	// TODO: decide how this should deal with unsorted frequencies
	this->components.push_back(comp);
}

std::vector<FrequencyComponent>&
DiscreteWaveSpectrum::getComponents()
{
	return this->components;
}

std::vector<FrequencyComponent>
DiscreteWaveSpectrum::interpEvenlySpaced()
{
	real dw = std::numeric_limits<real>::infinity(); // Change in wave freq.
	for (unsigned int i = 1; i < components.size(); i++) {
		real diff = components[i].omega - components[i - 1].omega;
		if (diff < dw)
			dw = diff;
	}
	if (dw <= 0) {
		throw moordyn::invalid_value_error(
		    "Ascending frequencies are expected");
	}
	// number of frequencies (equal to wavefreqs.size when evenly spaced)
	unsigned int nw =
	    static_cast<unsigned int>(floor(components.back().omega / dw)) + 1;

	// kind of ugly converting from array of struct to multiple arrays
	std::vector<real> omegas(size());
	std::vector<std::complex<real>> amplitudes(size());
	std::vector<real> betas(size());
	for (auto i = 0; i < size(); i++) {
		omegas[i] = components[i].omega;
		amplitudes[i] = components[i].amplitude;
		betas[i] = components[i].beta;
	}

	std::vector<FrequencyComponent> freqComps(nw);
	real f;
	unsigned int j = 1;
	for (unsigned int i = 0; i < nw; i++) {
		j = interp_factor(omegas, j, i * dw, f);

		auto newAmplitude = lerp(amplitudes[j - 1], amplitudes[j], f);
		auto newBeta = lerp(betas[j - 1], betas[j], f);
		freqComps[i] = FrequencyComponent(i * dw, newAmplitude, newBeta);
	}
	return freqComps;
}

moordyn::waves::DiscreteWaveSpectrum
spectrumFromFile(const std::string& path, moordyn::Log* _log)
{
	LOGMSG << "reading spectrum from file: " << std::filesystem::absolute(path)
	       << endl;
	vector<string> lines;
	try {
		lines = moordyn::fileIO::fileToLines(path);
	} catch (moordyn::input_file_error& err) {
		std::stringstream ss;
		ss << "Waves::setup failed to read wave_frequencies file: "
		   << err.what();
		throw input_file_error(ss.str().c_str());
	}

	if (lines.size() < 2) {
		LOGERR << "At least 2 frequency components shall be provided in '"
		       << path << "'" << endl;
		throw moordyn::input_file_error("Invalid file format");
	}

	waves::DiscreteWaveSpectrum spectrum;
	vector<real> wavefreqs;
	vector<moordyn::complex> waveelevs;

	for (auto line : lines) {
		vector<string> entries = moordyn::str::split(line);
		if (entries.size() < 3) {
			LOGERR << "The file '" << path << "' should have 3 or 4 columns"
			       << endl;
			throw moordyn::input_file_error("Invalid file format");
		}
		try {
			real freq = stod(entries[0]);
			// wavefreqs.push_back();
			real r = stod(entries[1]);
			real i = stod(entries[2]);
			real beta = entries.size() == 4 ? stod(entries[3]) : 0.0;
			if (beta > 2 * pi || beta < -2 * pi) {
				LOGERR << "Cannot specify a wave frequency with a direction of "
				          "great than 2pi or less than -2pi. The value should "
				          "be in radians."
				       << endl;
				throw moordyn::input_file_error(
				    "Invalid wave_frequencies.txt file");
			}
			spectrum.addFreqComp(freq, r, i, beta);
			// waveelevs.push_back(r + i1 * i);
		} catch (std::exception& e) {
			LOGERR << "Tried to parse the line \"" << line
			       << "\" as 3 floats, but failed:" << e.what() << endl;
			throw moordyn::input_file_error("Invalid file format");
		}
	}
	return spectrum;
}

std::vector<FrequencyComponent>
ParametricSpectrum::getAmplitudes(real minOmega, real maxOmega, real numComps)
{
	real dw = (maxOmega - minOmega) / (numComps - 1);
	auto omegas = Eigen::ArrayX<real>::LinSpaced(numComps, minOmega, maxOmega);

	auto freqs = omegas / (2 * pi);
	auto spectrumValues =
	    freqs.unaryExpr([&](auto f) { return this->getSpectrum(f); });

	auto amplitudes = (2 * spectrumValues * dw).sqrt();

	std::vector<FrequencyComponent> components;
	for (auto i = 0; i < omegas.size(); i++) {
		components.emplace_back(omegas[i], amplitudes[i]);
	}
	return components;
}
PMSpectrum::PMSpectrum(real Hs, real f_p)
  : Hs(Hs)
  , f_p(f_p)
{
}

real
PMSpectrum::getSpectrum(real f)
{
	// Pierson-Moskowitz Spectrum from IEC TS 62600-2 ED1 Annex F.2
	const real front = 0.3125 * Hs * Hs * pow(f, 4) * pow(f, -5.0);
	const real back = exp(-1.25 * pow(f / f, 4));
	return front * back;
}
} // namespace waves
} // namespace moordyn
