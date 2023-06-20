#pragma once

#include <string>
#include <memory>

struct EnvCond;
typedef std::shared_ptr<EnvCond> EnvCondRef;
namespace moordyn {
class WaveGrid;
class CurrentGrid;

class Log;

namespace waves {

/**
 * @brief Does the setup for the WAVE_FFT_GRID wave mode
 *
 *
 * @param folder The folder to look for the wave_frequencies.txt file in
 * @param env The environment options
 * @param _log Log pointer to allow logging from this function
 * @return std::unique_ptr<WaveGrid> A wave grid object containing the
 * precalculated wave data
 */
std::unique_ptr<WaveGrid>
constructWaveGridSpectrumData(const std::string& folder,
                              const EnvCondRef env,
                              moordyn::Log* _log);
/**
 * @brief Does the setup for the WAVE_GRID wave mode
 *
 *
 * @param folder The folder to look for the wave_elevation.txt and
 * water_grid.txt files in
 * @param env The environment options
 * @param _log Log pointer to allow logging from this function
 * @return std::unique_ptr<WaveGrid> A wave grid object containing the
 * precalculated wave data
 */
std::unique_ptr<WaveGrid>
constructWaveGridElevationData(const std::string& folder,
                               const EnvCondRef env,
                               moordyn::Log* _log);

/**
 * @brief Does the setup for the CURRENTS_STEADY_GRID mode
 *
 * @param folder The folder to look for the files defining the steady currents
 * @param env The environment options
 * @param _log Log pointer to allow logging from this function
 * @return std::unique_ptr<CurrentGrid> A current grid object containing the
 * precalculated current data
 */
std::unique_ptr<CurrentGrid>
constructSteadyCurrentGrid(const std::string& folder,
                           const EnvCondRef env,
                           moordyn::Log* _log);

/**
 * @brief Does the setup for the CURRENTS_DYNAMIC_GRID mode
 *
 * @param folder The folder to look for the files defining the dynamic currents
 * @param env The environment options
 * @param _log Log pointer to allow logging from this function
 * @return std::unique_ptr<CurrentGrid> A current grid object containing the
 * precalculated current data
 */
std::unique_ptr<CurrentGrid>
constructDynamicCurrentGrid(const std::string& folder,
                            const EnvCondRef env,
                            moordyn::Log* _log);

/**
 * @brief Does the setup for the CURRENTS_4D mode
 *
 * @param folder The folder to look for the files defining the 4D currents
 * @param env The environment options
 * @param _log Log pointer to allow logging from this function
 * @return std::unique_ptr<CurrentGrid> A current grid object containing the
 * precalculated current data
 */
std::unique_ptr<CurrentGrid>
construct4DCurrentGrid(const std::string& folder,
                       const EnvCondRef env,
                       moordyn::Log* _log);
}
}
