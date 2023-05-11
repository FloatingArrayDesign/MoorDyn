
#pragma once

namespace moordyn {
namespace waves {

/** @brief Available settings for waves
 */
typedef enum
{
	/// No waves
	WAVES_NONE = 0,
	/// Waves externally provided
	WAVES_EXTERNAL = 1,
	/// Wave elevation FFT, grid approach
	WAVES_FFT_GRID = 2,
	/// Wave elevation time series, grid approach
	WAVES_GRID = 3,
	/// Wave elevation FFT, node approach
	WAVES_FFT_NODE = 4,
	/// Wave elevation time series, node approach
	WAVES_NODE = 5,
	/// velocity, acceleration, and wave elevation grid data
	WAVES_KIN = 6,
	/**
	 * @brief Sum frequency components on outer time steps.
	 *
	 * Take in a set of frequency components. At every outer
	 * time step, we sum those frequency components at each node
	 * location.
	 */
	WAVES_SUM_COMPONENTS_NODE = 7,
} waves_settings;

// Current options: 0 - no currents or set externally (as part of WaveKin =0 or
// 1 approach) [default]
//                  1 - read in steady current profile, grid approach
//                  (current_profile.txt)** 2 - read in dynamic current profile,
//                  grid approach (current_profile_dynamic.txt)** 3 - read in
//                  steady current profile, node approach (current_profile.txt)
//                  4 - read in dynamic current profile, node approach
//                  (current_profile_dynamic.txt)

/** @brief Available settings for currents
 */
typedef enum
{
	/// No currents
	CURRENTS_NONE = 0,
	/// steady current profile, grid approach
	CURRENTS_STEADY_GRID = 1,
	/// dynamic current profile, grid approach
	CURRENTS_DYNAMIC_GRID = 2,
	/// steady current profile, node approach
	CURRENTS_STEADY_NODE = 3,
	/// dynamic current profile, node approach
	CURRENTS_DYNAMIC_NODE = 4,
	/// 4D current profile
	CURRENTS_4D = 5
} currents_settings;

/**
 * @brief Container for all the wave and current options
 *
 */
struct WaterKinOptions
{
	/// WaveKin Option
	waves_settings waveMode;
	/// Currents Option
	currents_settings currentMode;
	/**
	 * UnifyCurrentGrid Option
	 *
	 * 0 means to not unify the wave and current grids and keep them separate
	 * 1 means that if the waves are defined with a grid and the currents are
	 * defined with a grid, then the currents will be interpolated on to the
	 * wave grid, resulting in a single stored grid
	 */
	bool unifyCurrentGrid;
	/// dtWaveOption
	double dtWave;

	/**
	 * @brief Construct a new Water Kin Options object with default values
	 *
	 * By default waves and currents are off,
	 * the update frequency is at every substep,
	 * and to unify wave and current grids
	 */
	WaterKinOptions()
	  : waveMode(WAVES_NONE)
	  , currentMode(CURRENTS_NONE)
	  , unifyCurrentGrid(true)
	  , dtWave(0.25)
	{
	}
};

/** @brief Are the waves settings grid based?
 * @param opt Waves settings
 * @return true if the waves are provided in a grid, false otherwise
 */
inline bool
is_waves_grid(waves_settings opt)
{
	if (opt == WAVES_FFT_GRID)
		return true;
	if (opt == WAVES_GRID)
		return true;
	return false;
}

/** @brief Are the waves settings node based?
 * @param opt Waves settings
 * @return true if the waves are provided in the nodes, false otherwise
 */
inline bool
is_waves_node(waves_settings opt)
{
	if (opt == WAVES_FFT_NODE)
		return true;
	if (opt == WAVES_NODE)
		return true;
	return false;
}

/** @brief Are the currents settings grid based?
 * @param opt Currents settings
 * @return true if the currents are provided in a grid, false otherwise
 */
inline bool
is_currents_grid(currents_settings opt)
{
	if (opt == CURRENTS_STEADY_GRID)
		return true;
	if (opt == CURRENTS_DYNAMIC_GRID)
		return true;
	if (opt == CURRENTS_4D)
		return true;
	return false;
}

/** @brief Are the currents settings node based?
 * @param opt Currents settings
 * @return true if the currents are provided in the nodes, false otherwise
 */
inline bool
is_currents_node(currents_settings opt)
{
	if (opt == CURRENTS_STEADY_NODE)
		return true;
	if (opt == CURRENTS_DYNAMIC_NODE)
		return true;
	return false;
}

} // namespace waves
} // namespace moordyn
