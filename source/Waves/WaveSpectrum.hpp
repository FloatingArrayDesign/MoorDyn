
#pragma once

#include "../Misc.hpp"
#include "Log.hpp"
#include "Misc.hpp"
#include <vector>

namespace moordyn {

/**
 * @brief Wave and current helpers
 *
 */
namespace waves {

/**
 * @brief Represents a single monochromatic wave
 *
 * Primarily used by @ref DiscreteWaveSpectrum
 *
 */
struct FrequencyComponent
{
	/// Angular velocity of the frequency component (rad/s)
	real omega;
	/// Complex amplitude of the frequency component
	moordyn::complex amplitude;
	/// Angle, in radians, of the frequency component
	real beta = 0;

	FrequencyComponent(){};
	FrequencyComponent(real omega, moordyn::complex amplitude, real beta = 0)
	  : omega(omega)
	  , amplitude(amplitude)
	  , beta(beta)
	{
	}
};

/**
 * @brief A wave spectrum represented by some number of angular frequency
 * components with complex amplitudes.
 *
 */
class DiscreteWaveSpectrum
{
  public:
	DiscreteWaveSpectrum();
	~DiscreteWaveSpectrum();

	/**
	 * @brief Add a frequency component to the spectrum.
	 *
	 * The angular velocity of this component should be larger than the angular
	 * velocity of all the previous components in order for interpEventlySpaced
	 * to work
	 * @param comp New frequency component
	 */
	void addFreqComp(FrequencyComponent comp);

	/**
	 * @brief Add a frequency component using an angular velocity and the real
	 * and imaginary parts of the complex amplitude.
	 *
	 * @param omega Angular velocity
	 * @param r Real part of the amplitude
	 * @param i Imaginary part of the amplitude
	 */
	inline void addFreqComp(real omega, real r, real i, real beta = 0)
	{
		addFreqComp(FrequencyComponent(omega, std::complex<real>(r, i), beta));
	}

	/**
	 * @brief Get the frequency components
	 *
	 * @return std::vector<FrequencyComponent>&
	 */
	std::vector<FrequencyComponent>& getComponents();

	/**
	 * @brief Access a frequency component by index.
	 *
	 * Not currently bounds checked
	 * @param index Index of component to access
	 * @return FrequencyComponent&
	 */
	inline FrequencyComponent& operator[](unsigned int index)
	{
		return this->components[index];
	}

	/**
	 * @brief Get the current number of frequency components
	 *
	 * @return size_t
	 */
	inline size_t size() const { return this->components.size(); }

	/**
	 * @brief Computes a list of frequency components with evenly spaced angular
	 * velocities.
	 *
	 * The computed list has the same range of angular velocities, but the
	 * difference between subsequent angular velocities is equal to the smallest
	 * difference between adjacent frequency components. The amplitudes of the
	 * new frequency components are computed by linearly interpolating between
	 * the nearest smaller and nearest larger frequency from the frequency
	 * components.
	 *
	 * Ex:
	 *
	 * The frequency components
	 *
	 * [(w = 0, a = 0), (w = 1, a = 1), (w = 2, a = 2), (w = 2.5, a = 3), (w =
	 * 3, a = 4) ]
	 *
	 * would become
	 *
	 * [(w = 0, a = 0), (w = 0.5, a = 0.5), (w = 1, a = 1), (w = 1.5, a = 1.5),
	 * (w = 2, a = 2), (w = 2.5, a = 3), (w = 3, a = 4) ]
	 *
	 * @return std::vector<FrequencyComponent>
	 */
	std::vector<FrequencyComponent> interpEvenlySpaced();

  private:
	/// List of frequency components
	std::vector<FrequencyComponent> components;
};

class ParametricSpectrum
{
	virtual real getSpectrum(real f) = 0;
	std::vector<FrequencyComponent> getAmplitudes(real minOmega,
	                                              real maxOmega,
	                                              real numComps);
};

class PMSpectrum : ParametricSpectrum
{
  public:
	PMSpectrum(real Hs, real f_p);
	real getSpectrum(real f) override;

  private:
	/// Significant wave height (meters)
	real Hs;
	/// Peak frequency (1 / peak period) (Hz)
	real f_p;
};

/**
 * @brief Parses a wave_frequencies.txt file (or any file with the same format)
 *
 * @param path Path to the file
 * @param _log Logger
 * @return moordyn::waves::DiscreteWaveSpectrum The discrete wave spectrum
 * represented by the file
 * @throws moordyn::input_file_error if the input file is not of the correct
 * format
 */
moordyn::waves::DiscreteWaveSpectrum
spectrumFromFile(const std::string& path, moordyn::Log* _log);

}
}
