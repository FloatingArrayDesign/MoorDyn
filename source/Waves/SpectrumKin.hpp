#pragma once

#include "Misc.hpp"
#include "Seafloor.hpp"
#include "Waves/WaveSpectrum.hpp"
#include <vector>

namespace moordyn {

namespace waves {

/**
 * @brief Capable of calculating the wave kinematics at a point in time from a
 * irregular wave spectrum by summing each of the component waves.
 *
 */
class SpectrumKin
{
  public:
	/**
	 * @brief Do setup and initialization of the wave spectrum
	 *
	 * Creates a copy of the frequency components.
	 *
	 * @param freqComps List of frequency components that make up the spectrum
	 * @param env Environmental conditions and settings
	 * @param seafloor A optional 3D seafloor, if not given uses `env->WtrDpth`
	 */
	void setup(const std::vector<FrequencyComponent>& freqComps,
	           EnvCondRef env,
	           SeafloorRef seafloor = nullptr);

	/**
	 * @brief Get the Wave kinematics at an (x, y, z) point at time t
	 *
	 * @param pos The (x, y, z) coordinates of the point to calculate the waves
	 * at
	 * @param t The time to calculate the waves at
	 * @param avgDepth A negative number representing the average seafloor depth
	 * (used for calculating wave numbers)
	 * @param actualDepth A negative number representing the actual seafloor
	 * depth at that point (used in wave stretching, can be the same as
	 * avgDepth)
	 * @param zeta surface height, only set if not null
	 * @param vel water velocity, only set if not null
	 * @param acc water acceleration, only set if not null
	 */
	void getWaveKin(vec3 pos,
	                real t,
	                real avgDepth,
	                real actualDepth,
	                real* zeta,
	                vec3* vel,
	                vec3* acc) const;

  private:
	/// Angular velocities of the spectrum components
	Eigen::ArrayX<real> omegas;
	/// Real amplitudes of spectrum components
	Eigen::ArrayX<real> amplitudes;
	/// Angle of the spectrum components (radians)
	Eigen::ArrayX<real> betas;
	/// Phase offset (radians) of spectrum components
	Eigen::ArrayX<real> phases;
	/// Wave numbers of spectrum components
	Eigen::ArrayX<real> kValues;
};

}

}
