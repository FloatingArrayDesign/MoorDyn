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
#include "Time.hpp"
#include "Seafloor.hpp"
#include "Waves/WaveGrid.hpp"
#include "Util/Interp.hpp"
#include <filesystem>

#if defined WIN32 && defined max
// We must avoid max messes up with std::numeric_limits<>::max()
#undef max
#endif

using namespace std;

namespace moordyn {

using namespace waves;

real
SeafloorProvider::getAverageDepth() const
{

	if (seafloor) {
		return seafloor->getAverageDepth();
	}
	return waterDepth;
}
real
SeafloorProvider::getDepth(const vec2& pos) const
{
	if (seafloor) {
		return seafloor->getDepthAt(pos.x(), pos.y());
	}
	return waterDepth;
}

std::vector<real>
gridAxisCoords(Waves::coordtypes coordtype, vector<string>& entries)
{
	// set number of coordinates
	unsigned int n = 0;
	std::vector<real> coordarray;

	if (coordtype == Waves::GRID_SINGLE)
		n = 1;
	else if (coordtype == Waves::GRID_LIST)
		n = static_cast<unsigned int>(entries.size());
	else if (coordtype == Waves::GRID_LATTICE)
		n = static_cast<unsigned int>(stoul(entries[2]));
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

void
WaveGrid::allocateKinematicArrays()
{
	if (!nx || !ny || !nz) {
		LOGERR << "The grid has not been initialized..." << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}
	if (!nt) {
		LOGERR << "The time series has null size" << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}

	zetas = init3DArray(nx, ny, nt);
	pDyn = init4DArray(nx, ny, nz, nt);
	wave_vel = init4DArrayVec(nx, ny, nz, nt);
	wave_acc = init4DArrayVec(nx, ny, nz, nt);

	LOGDBG << "Allocated the waves data grid";
}

void
WaveGrid::getWaveKin(const vec3& pos,
                     real time,
                     const SeafloorProvider& seafloor,
                     real* zeta,
                     vec3* vel,
                     vec3* acc,
                     real* pdyn)
{
	real fx, fy, fz;

	auto ix = interp_factor(px, pos.x(), fx);
	auto iy = interp_factor(py, pos.y(), fy);

	unsigned int it = 0;
	real ft = 0.0;
	if (nt > 1) {
		real quot = time / dtWave;
		it = floor(quot);
		ft = quot - it;
		it++; // We use the upper bound
		while (it > nt - 1)
			it -= nt;
	}

	// wave elevation
	auto wave_elev = interp3(zetas, ix, iy, it, fx, fy, ft);

	if (zeta) {
		*zeta = wave_elev;
	}

	real stretched_z = 0.0;
	const real bottom = seafloor.getDepth(vec2(pos.x(), pos.y()));
	const real actual_depth = wave_elev - bottom;

	const real avgDepth = seafloor.getAverageDepth();

	stretched_z = (-avgDepth * (pos.z() - bottom)) / actual_depth + avgDepth;

	// If the point is above the water surface, return the values at the water
	// surface. This is important for situations where a point is out of the
	// water but the object itself may have significant submergence (sideway
	// floating line, top node of buoy, etc)
	if (stretched_z > 0.0) {
		stretched_z = 0.0;
	}
	// LOGMSG << "WaveGrid::getWaveKin - stretched_z = " << stretched_z << endl;

	auto iz = interp_factor(pz, stretched_z, fz);

	if (vel) {
		*vel = interp4Vec(wave_vel, ix, iy, iz, it, fx, fy, fz, ft);
	}
	if (acc) {
		*acc = interp4Vec(wave_acc, ix, iy, iz, it, fx, fy, fz, ft);
	}
	if (pdyn) {
		*pdyn = interp4(pDyn, ix, iy, iz, it, fx, fy, fz, ft);
	}
}

void
CurrentGrid::allocateKinematicArrays()
{
	if (!nx || !ny || !nz) {
		LOGERR << "The grid has not been initialized..." << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}
	if (!nt) {
		LOGERR << "The time series has null size" << endl;
		throw moordyn::invalid_value_error("Uninitialized values");
	}

	current_vel = init4DArrayVec(nx, ny, nz, nt);
	current_acc = init4DArrayVec(nx, ny, nz, nt);

	LOGDBG << "Allocated the current data grid";
}

void
CurrentGrid::getCurrentKin(const vec3& pos,
                           real time,
                           const SeafloorProvider& seafloor,
                           vec3* vel,
                           vec3* acc)
{
	real fx, fy, fz;

	auto ix = interp_factor(px, pos.x(), fx);
	auto iy = interp_factor(py, pos.y(), fy);

	unsigned int it = 0;
	real ft = 0.0;
	if (nt > 1) {
		real quot = time / dtWave;
		it = floor(quot);
		ft = quot - it;
		it++; // We use the upper bound
		while (it > nt - 1)
			it -= nt;
	}

	// real stretched_z = 0.0;
	// const real bottom = seafloor.getDepth(vec2(pos.x(), pos.y()));
	// const real actual_depth = -bottom;

	// const real avgDepth = seafloor.getAverageDepth();

	// stretched_z = (-avgDepth * (pos.z() - bottom)) / actual_depth + avgDepth;

	// LOGMSG << "WaveGrid::getWaveKin - stretched_z = " << stretched_z << endl;

	// TODO - current stretching?
	auto iz = interp_factor(pz, pos.z(), fz);

	if (vel) {
		*vel = interp4Vec(current_vel, ix, iy, iz, it, fx, fy, fz, ft);
	}
	if (acc) {
		*acc = interp4Vec(current_acc, ix, iy, iz, it, fx, fy, fz, ft);
	}
}

Waves::Waves(moordyn::Log* log)
  : LogUser(log)
  , _t_integrator(NULL)
{
}

// function to clear any remaining data allocations in Waves
Waves::~Waves() {}

void
Waves::setup(EnvCondRef env_in,
             SeafloorRef seafloor,
             time::Scheme* t,
             const char* folder)
{
	// make sure to reset the kinematics if setup gets called multiple times
	// (like before dynamic relaxation and then before the main simulation)
	waveKinematics.reset();
	currentKinematics.reset();
	this->env = env_in;
	this->seafloor = seafloor;
	rho_w = env->rho_w;
	g = env->g;
	_t_integrator = t;

	// ------------------- start with wave kinematics -----------------------

	// ======================== check compatibility of wave and current settings
	// =====================

	auto wave_mode = env->waterKinOptions.waveMode;
	auto current_mode = env->waterKinOptions.currentMode;
	if ((wave_mode == moordyn::WAVES_NONE) &&
	    (current_mode == moordyn::CURRENTS_NONE)) {
		LOGMSG << "   No Waves or Currents, or set externally" << endl;
		return;
	}

	if (wave_mode == waves::WAVES_NONE) {
		if (current_mode == waves::CURRENTS_STEADY_GRID)
			LOGDBG << "   Current only: option 1 - "
			       << "   read in steady current profile, grid approach "
			       << "   (current_profile.txt)" << endl;
		else if (current_mode == waves::CURRENTS_DYNAMIC_GRID)
			LOGDBG << "   Current only: option 2 - "
			       << "   read in dynamic current profile, grid approach "
			       << "   (current_profile_dynamic.txt)" << endl;
		else if (current_mode == waves::CURRENTS_STEADY_NODE)
			LOGDBG << "   Current only: option TBD3 - "
			       << "   read in steady current profile, node approach "
			       << "   (current_profile.txt)" << endl;
		else if (current_mode == waves::CURRENTS_DYNAMIC_NODE)
			LOGDBG << "   Current only: option TBD4 - "
			       << "   read in dynamic current profile, node approach "
			       << "   (current_profile_dynamic.txt)" << endl;
		else if (current_mode == waves::CURRENTS_4D)
			LOGDBG
			    << "   Current only: option 4D - read in current profile, grid "
			    << "   approach (current_profile_4d.txt" << endl;
		else {
			LOGDBG << "   Invalid current input settings (must be 0-4)" << endl;
			throw moordyn::invalid_value_error("Invalid settings");
		}
	} else if (current_mode == waves::CURRENTS_NONE) {
		if (wave_mode == waves::WAVES_EXTERNAL)
			LOGDBG << "   Waves only: option 1 - "
			       << "   set externally for each node in each object" << endl;
		else if (wave_mode == waves::WAVES_FFT_GRID)
			LOGDBG << "   Waves only: option 2 - "
			       << "   set from inputted wave elevation FFT, grid approach "
			       << "   (NOT IMPLEMENTED YET)" << endl;
		else if (wave_mode == waves::WAVES_GRID)
			LOGDBG << "   Waves only: option 3 - "
			       << "   set from inputted wave elevation time series, grid "
			          "approach"
			       << endl;
		else if (wave_mode == waves::WAVES_FFT_GRID)
			LOGDBG << "   Waves only: option TBD4 - "
			       << "   set from inputted wave elevation FFT, node approach "
			       << "   (NOT IMPLEMENTED YET)" << endl;
		else if (wave_mode == waves::WAVES_NODE)
			LOGDBG << "   Waves only: option TBD5 - "
			       << "   set from inputted wave elevation time series, node "
			          "approach"
			       << endl;
		else if (wave_mode == waves::WAVES_KIN)
			LOGDBG << "   Waves only: option TBD6 - "
			       << "   set from inputted velocity, acceleration, and wave "
			          "   elevation grid data (TBD)"
			       << endl;
		else if (wave_mode == waves::WAVES_SUM_COMPONENTS_NODE)
			LOGDBG
			    << "   Waves only: option 7 - "
			    << "   set from inputted wave spectrum, computed at nodes on "
			       "   update "
			    << endl;
		else {
			LOGDBG << "   Invald wave kinematics input settings (must be 0-7)"
			       << endl;
			throw moordyn::invalid_value_error("Invalid settings");
		}
	} else if (wave_mode == waves::WAVES_EXTERNAL &&
	           current_mode != waves::CURRENTS_NONE) {
		LOGDBG << "   External waves as well as currents, current options: "
		       << current_mode << endl;

	} else if (waves::is_waves_grid(wave_mode) &&
	           waves::is_currents_grid(current_mode)) {
		LOGDBG << "   Waves and currents: options " << wave_mode << " & "
		       << current_mode << endl;
	} else if (waves::is_waves_node(wave_mode) &&
	           waves::is_currents_node(current_mode)) {
		LOGDBG << "   Waves and currents: options TBD " << wave_mode << " & "
		       << current_mode << endl;
	}

	// NOTE: nodal settings should use storeWaterKin in objects
	// now go through each applicable WaveKin option
	if (wave_mode == waves::WAVES_SUM_COMPONENTS_NODE) {
		const string WaveFilename = (string)folder + "wave_frequencies.txt";
		LOGMSG << "Reading waves spectrum frequencies from '" << WaveFilename
		       << "'..." << endl;
		auto waveSpectrum = spectrumFromFile(WaveFilename, _log);
		LOGMSG << "   '" << WaveFilename << "' parsed" << endl;

		if (waveSpectrum[0].omega != 0.0) {
			LOGERR << "   The first shall be 0 rad/s" << endl;
			throw moordyn::invalid_value_error("Invalid frequencies");
		}
		SpectrumKin spectrumKin{};
		spectrumKin.setup(waveSpectrum.getComponents(), env);

		waveKinematics =
		    std::make_unique<SpectrumKinWrapper>(std::move(spectrumKin));
	} else if (wave_mode == waves::WAVES_FFT_GRID) {
		waveGrid = constructWaveGridSpectrumData((string)folder, env, _log);
	} else if (wave_mode == waves::WAVES_GRID) {
		// load wave elevation time series from file (similar to what's done in
		// GenerateWaveExtnFile.py, and was previously in misc2.cpp)
		waveGrid = constructWaveGridElevationData((string)folder, env, _log);
	}

	// Now add in current velocities (add to unsteady wave kinematics)
	if (current_mode == CURRENTS_STEADY_GRID) {
		auto currentGrid = constructSteadyCurrentGrid(folder, env, _log);

		// if there is an existing wave grid an we are set to unify the wave
		// and current grids
		if (waveGrid && env->waterKinOptions.unifyCurrentGrid) {
			SeafloorProvider floorProvider{ -env->WtrDpth, seafloor };
			for (unsigned int iz = 0; iz < waveGrid->nz; iz++) {
				auto z = waveGrid->Pz()[iz];
				vec3 curr_vel, curr_acc;
				currentGrid->getCurrentKin(
				    vec3(0, 0, z), 0.0, floorProvider, &curr_vel, &curr_acc);

				for (unsigned int ix = 0; ix < waveGrid->nx; ix++) {
					for (unsigned int iy = 0; iy < waveGrid->ny; iy++) {
						for (unsigned int it = 0; it < waveGrid->nt; it++) {
							waveGrid->WaveVel()[ix][iy][iz][it] += curr_vel;
							waveGrid->WaveAcc()[ix][iy][iz][it] += curr_acc;
						}
					}
				}
			}
		} else {
			currentKinematics = std::move(currentGrid);
		}
	} else if (current_mode == CURRENTS_DYNAMIC_GRID) {
		auto currentGrid = constructDynamicCurrentGrid(folder, env, _log);

		if (waveGrid && env->waterKinOptions.unifyCurrentGrid) {
			// interpolate currents on to wave existing wave grid
			SeafloorProvider floorProvider{ -env->WtrDpth, seafloor };
			for (unsigned int iz = 0; iz < waveGrid->nz; iz++) {
				real z = waveGrid->Pz()[iz];
				for (unsigned int it = 0; it < waveGrid->nt; it++) {
					vec3 currentVel, currentAcc;
					currentGrid->getCurrentKin(vec3(0.0, 0.0, z),
					                           it * waveGrid->dtWave,
					                           floorProvider,
					                           &currentVel,
					                           &currentAcc);
					for (unsigned int ix = 0; ix < waveGrid->nx; ix++) {
						for (unsigned int iy = 0; iy < waveGrid->ny; iy++) {
							waveGrid->WaveVel()[ix][iy][iz][it] += currentVel;
							waveGrid->WaveVel()[ix][iy][iz][it] += currentAcc;
						}
					}
				}
			}
		} else {
			currentKinematics = std::move(currentGrid);
		}
	} else if (current_mode == CURRENTS_4D) {
		auto currentGrid = construct4DCurrentGrid(folder, env, _log);
		if (waveGrid && env->waterKinOptions.unifyCurrentGrid) {
			// interpolate read in data and add to existing grid
			// (dtWave, px, etc are already set in the grid)
			// LOGMSG << "interpolating 4d current grid onto wave grid" << endl;
			// LOGMSG << "4D current grid stats: nx = " << currentGrid->nx
			//        << " ny = " << currentGrid->ny << " nz = " <<
			//        currentGrid->nz
			//        << " nt = " << currentGrid->nt
			//        << " dtWave = " << currentGrid->dtWave << endl;

			// LOGMSG << "wave grid stats: nx = " << waveGrid->nx
			//        << " ny = " << waveGrid->ny << " nz = " << waveGrid->nz
			//        << " nt = " << waveGrid->nt
			//        << " dtWave = " << waveGrid->dtWave << endl;
			SeafloorProvider floorProvider{ -env->WtrDpth, seafloor };
			for (unsigned int ix = 0; ix < waveGrid->nx; ix++) {
				const real x = waveGrid->Px()[ix];
				for (unsigned int iy = 0; iy < waveGrid->ny; iy++) {
					const real y = waveGrid->Py()[iy];
					for (unsigned int iz = 0; iz < waveGrid->nz; iz++) {
						const real z = waveGrid->Pz()[iz];
						for (unsigned int it = 0; it < waveGrid->nt; it++) {

							vec3 currentVel, currentAcc;
							currentGrid->getCurrentKin(vec3(x, y, z),
							                           it * waveGrid->dtWave,
							                           floorProvider,
							                           &currentVel,
							                           &currentAcc);
							waveGrid->WaveVel()[ix][iy][iz][it] += currentVel;
							waveGrid->WaveAcc()[ix][iy][iz][it] += currentAcc;
						}
					}
				}
			}
		} else {
			currentKinematics = std::move(currentGrid);
		}
	}

	// waveGrid is a temporary value to help with unifying wave and current
	// grids if it is not null and wave kinematics is null then make that wave
	// grid our wave kinematics
	if (waveGrid && !waveKinematics) {
		waveKinematics = std::move(waveGrid);
	}
	// waveGrid stores a temporary value that should have been moved out
	assert(!waveGrid);

	// either there are no waves (or external waves), or waveKinematics is not
	// null
	assert((wave_mode == WAVES_NONE || wave_mode == WAVES_EXTERNAL) !=
	       (bool)(waveKinematics));

	// if there is a current, at least one of waveKin and currentKind should not
	// be null. currentKin could be null if the wave mode is a grid mode and
	// it's set to unify current grids (which happens by default)
	if (current_mode != CURRENTS_NONE) {
		// if currents are defined, either wave or current kinematics should be
		// defined
		assert((bool)(waveKinematics) || (bool)(currentKinematics));
	} else {
		// if currents are not defined, current kinematics should not be defined
		assert(!bool(currentKinematics));
	}
}

void
Waves::addLine(moordyn::Line* line)
{
	if (line->lineId == static_cast<size_t>(nodeKin.lines.structures.size())) {
		auto num_nodes = line->getN() + 1;
		genericAdd(line, num_nodes, nodeKin.lines);
		// At some point we should figure out how to determine when to not do
		// this, but because solving for initial conditions can cause waves to
		// first be setup as off and then be setup as on, this is the easiest
		// method
		genericAdd(line, num_nodes, waveKin.lines);
	} else {
		throw "the lines id should be equal to its index in the lines "
		      "array";
	}
}
void
Waves::addRod(moordyn::Rod* rod)
{
	if (rod->rodId == static_cast<size_t>(nodeKin.rods.structures.size())) {
		auto num_nodes = rod->getN() + 1;
		genericAdd(rod, num_nodes, nodeKin.rods);
		// TODO - only do this when needed, see comment in addLIne
		genericAdd(rod, num_nodes, waveKin.rods);
	} else {
		throw "the rod id should be equal to its index in the rod array";
	}
}
void
Waves::addBody(moordyn::Body* body)
{
	if (body->bodyId == static_cast<size_t>(nodeKin.bodies.structures.size())) {
		auto num_nodes = 1;
		genericAdd(body, num_nodes, nodeKin.bodies);
		// TODO - only do this when needed, see comment in addLIne
		genericAdd(body, num_nodes, waveKin.bodies);
	} else {
		throw "the body id should be equal to its index in the body array";
	}
}

void
Waves::addPoint(moordyn::Point* point)
{
	if (point->pointId ==
	    static_cast<size_t>(nodeKin.points.structures.size())) {
		auto num_nodes = 1;
		genericAdd(point, num_nodes, nodeKin.points);
		// TODO - only do this when needed, see comment in addLIne
		genericAdd(point, num_nodes, waveKin.points);
	} else {
		throw "the point id should be equal to its index in the "
		      "point array";
	}
}

Waves::NodeKinReturnType
Waves::getWaveKinLine(size_t lineId)
{
	return nodeKin.lines[lineId];
}

Waves::NodeKinReturnType
Waves::getWaveKinRod(size_t rodId)
{
	return nodeKin.rods[rodId];
}

Waves::NodeKinReturnType
Waves::getWaveKinBody(size_t bodyId)
{
	return nodeKin.bodies[bodyId];
}

Waves::NodeKinReturnType
Waves::getWaveKinPoint(size_t pointId)
{
	return nodeKin.points[pointId];
}

real
Waves::getWaveHeightPoint(vec2 point)
{
	vec3 pos(point.x(), point.y(), 0.0);

	real zeta;

	SeafloorProvider floorProvider{ -env->WtrDpth, seafloor };
	waveKinematics->getWaveKin(pos,
	                           _t_integrator->GetTime(),
	                           floorProvider,
	                           &zeta,
	                           nullptr,
	                           nullptr,
	                           nullptr);
	return zeta;
}
void
Waves::getWaveKin(const vec3& pos,
                  real& zeta,
                  vec3& vel,
                  vec3& acc,
                  real& pdyn,
                  Seafloor* seafloor)
{
	if (!waveKinematics && !currentKinematics) {
		zeta = 0;
		pdyn = 0;
		vel = vec::Zero();
		acc = vec::Zero();
		return;
	}

	real zeta_sum = 0;
	real pdyn_sum = 0;
	vec vel_sum{ 0.0, 0.0, 0.0 };
	vec acc_sum{ 0.0, 0.0, 0.0 };

	// Because SeafloorProvider expects a shared_ptr but we only have a ptr,
	// we need to make a shared_ptr that won't call delete on seafloor when
	// it goes out of scope.
	// This constructor (called the aliasing constructor), allows us to do
	// just that, so that calling Waves::getWaveKin with a seafloor ptr doesn't
	// cause that seafloor object to be deleted.
	SeafloorRef floor = SeafloorRef(SeafloorRef{}, seafloor);
	SeafloorProvider floorProvider{ -env->WtrDpth, floor };
	if (waveKinematics) {
		real wave_zeta, wave_pdyn;
		vec wave_vel{}, wave_acc{};
		waveKinematics->getWaveKin(pos,
		                           _t_integrator->GetTime(),
		                           floorProvider,
		                           &wave_zeta,
		                           &wave_vel,
		                           &wave_acc,
		                           &wave_pdyn);
		zeta_sum += wave_zeta;
		pdyn_sum += wave_pdyn;
		vel_sum += wave_vel;
		acc_sum += wave_acc;
	}

	if (currentKinematics) {
		vec wave_vel{}, wave_acc{};
		currentKinematics->getCurrentKin(
		    pos, _t_integrator->GetTime(), floorProvider, &wave_vel, &wave_acc);
		vel_sum += wave_vel;
		acc_sum += wave_acc;
	}
	zeta = zeta_sum;
	pdyn = pdyn_sum;
	vel = vel_sum;
	acc = acc_sum;
}

std::vector<vec3>
Waves::getWaveKinematicsPoints()
{

	std::vector<vec> rout;
	// we kind of abuse this function just to get all of the positions
	// but doing it this way does mean we always get consistency with
	// setWaveKinematics
	kinematicsForAllNodes(
	    nodeKin, [&](vec pos, vec& _U, vec& _Ud, real& _zeta, real& _pdyn) {
		    rout.push_back(pos);
	    });
	return rout;
}

void
Waves::setWaveKinematics(std::vector<vec> const& U_in,
                         std::vector<vec> const& Ud_in)
{
	unsigned int i = 0;
	if (U_in.size() != Ud_in.size()) {
		throw moordyn::invalid_value_error(
		    "Waves::setWaveKinematics U and Ud must have the same size");
	}
	AllNodesKin& kinematics = currentKinematics ? waveKin : nodeKin;
	kinematicsForAllNodes(
	    kinematics, [&](vec _pos, vec& U, vec& Ud, real& _zeta, real& _pdyn) {
		    if (i >= U_in.size()) {
			    throw moordyn::invalid_value_error(
			        "not enough points supplied to Waves::setWaveKinematics");
		    }
		    U = U_in[i];
		    Ud = Ud_in[i];
		    i++;
	    });
}
template<typename F>
void
Waves::kinematicsForAllNodes(AllNodesKin& nodeKinematics, F f)
{
	auto& lines = nodeKinematics.lines;
	for (const auto& line : lines.structures) {
		for (unsigned int i = 0; i <= line->getN(); i++) {
			const vec pos = line->getNodePos(i);
			const auto id = line->lineId;
			f(pos,
			  lines.U[id][i],
			  lines.Ud[id][i],
			  lines.zetas[id][i],
			  lines.Pdyn[id][i]);
		}
	}

	auto& rods = nodeKinematics.rods;
	for (const auto& rod : rods.structures) {
		for (unsigned int i = 0; i <= rod->getN(); i++) {
			const vec pos = rod->getNodePos(i);
			const auto id = rod->rodId;
			f(pos,
			  rods.U[id][i],
			  rods.Ud[id][i],
			  rods.zetas[id][i],
			  rods.Pdyn[id][i]);
		}
	}

	auto& points = nodeKinematics.points;
	for (const auto& point : points.structures) {
		const vec& pos = point->getPosition();
		const auto id = point->pointId;
		f(pos,
		  points.U[id][0],
		  points.Ud[id][0],
		  points.zetas[id][0],
		  points.Pdyn[id][0]);
	}

	auto& bodies = nodeKinematics.bodies;
	for (const auto& body : bodies.structures) {
		const vec pos = body->getPosition();
		const auto id = body->bodyId;
		f(pos,
		  bodies.U[id][0],
		  bodies.Ud[id][0],
		  bodies.zetas[id][0],
		  bodies.Pdyn[id][0]);
	}
}

void
Waves::updateWaves()
{
	SeafloorProvider floorProvider{ -env->WtrDpth, seafloor };
	if (env->waterKinOptions.waveMode == waves::WAVES_EXTERNAL &&
	    currentKinematics) {
		// if we have external waves and currents, then we go through and add
		// together the externally defined wave kinematics with the calculated
		// current kinematics
		auto& lines = nodeKin.lines;
		for (const auto& line : lines.structures) {
			for (unsigned int i = 0; i <= line->getN(); i++) {
				const vec pos = line->getNodePos(i);
				const auto id = line->lineId;

				vec3 curr_U{}, curr_Ud{};
				currentKinematics->getCurrentKin(pos,
				                                 _t_integrator->GetTime(),
				                                 floorProvider,
				                                 &curr_U,
				                                 &curr_Ud);
				lines.U[id][i] = waveKin.lines.U[id][i] + curr_U;
				lines.Ud[id][i] = waveKin.lines.Ud[id][i] + curr_Ud;
			}
		}

		auto& rods = nodeKin.rods;
		for (const auto& rod : rods.structures) {
			for (unsigned int i = 0; i <= rod->getN(); i++) {
				const vec pos = rod->getNodePos(i);
				const auto id = rod->rodId;
				vec3 curr_U{}, curr_Ud{};
				currentKinematics->getCurrentKin(pos,
				                                 _t_integrator->GetTime(),
				                                 floorProvider,
				                                 &curr_U,
				                                 &curr_Ud);
				rods.U[id][i] = waveKin.rods.U[id][i] + curr_U;
				rods.Ud[id][i] = waveKin.rods.Ud[id][i] + curr_Ud;
			}
		}

		auto& points = nodeKin.points;
		for (const auto& point : points.structures) {
			const vec& pos = point->getPosition();
			const auto id = point->pointId;

			vec3 curr_U{}, curr_Ud{};
			currentKinematics->getCurrentKin(pos,
			                                 _t_integrator->GetTime(),
			                                 floorProvider,
			                                 &curr_U,
			                                 &curr_Ud);
			points.U[id][0] = waveKin.points.U[id][0] + curr_U;
			points.Ud[id][0] = waveKin.points.Ud[id][0] + curr_Ud;
		}

		auto& bodies = nodeKin.bodies;
		for (const auto& body : bodies.structures) {
			const vec pos = body->getPosition();
			const auto id = body->bodyId;

			vec3 curr_U{}, curr_Ud{};
			currentKinematics->getCurrentKin(pos,
			                                 _t_integrator->GetTime(),
			                                 floorProvider,
			                                 &curr_U,
			                                 &curr_Ud);
			bodies.U[id][0] = waveKin.bodies.U[id][0] + curr_U;
			bodies.Ud[id][0] = waveKin.bodies.Ud[id][0] + curr_Ud;
		}
		return;
	}
	// if there are both waves and currents, then we calculate and sum
	if (waveKinematics && currentKinematics) {
		kinematicsForAllNodes(
		    nodeKin, [&](vec pos, vec& U, vec& Ud, real& zeta, real& pdyn) {
			    vec3 wave_U{}, wave_Ud{};
			    waveKinematics->getWaveKin(pos,
			                               _t_integrator->GetTime(),
			                               floorProvider,
			                               &zeta,
			                               &wave_U,
			                               &wave_Ud,
			                               &pdyn);
			    vec3 curr_U{}, curr_Ud{};
			    currentKinematics->getCurrentKin(pos,
			                                     _t_integrator->GetTime(),
			                                     floorProvider,
			                                     &curr_U,
			                                     &curr_Ud);
			    U = wave_U + curr_U;
			    Ud = wave_Ud + curr_Ud;
		    });
		return;
	}
	// if there are just waves then we just do wave calculations
	if (waveKinematics) {
		kinematicsForAllNodes(
		    nodeKin, [&](vec pos, vec& U, vec& Ud, real& zeta, real& pdyn) {
			    waveKinematics->getWaveKin(pos,
			                               _t_integrator->GetTime(),
			                               floorProvider,
			                               &zeta,
			                               &U,
			                               &Ud,
			                               &pdyn);
		    });
		return;
	}
	// if there are just currents then we just do current calculations
	if (currentKinematics) {
		kinematicsForAllNodes(
		    nodeKin, [&](vec pos, vec& U, vec& Ud, real& zeta, real& pdyn) {
			    currentKinematics->getCurrentKin(
			        pos, _t_integrator->GetTime(), floorProvider, &U, &Ud);
		    });
		return;
	}
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
                    double* PDyn,
                    MoorDynSeafloor seafloor)
{
	CHECK_WAVES(waves);
	moordyn::vec u{}, ud{};
	moordyn::real h{}, p{};
	((moordyn::Waves*)waves)
	    ->getWaveKin(moordyn::vec3({ x, y, z }),
	                 h,
	                 u,
	                 ud,
	                 p,
	                 (moordyn::Seafloor*)seafloor);
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
			k = -k; // @mth: modified to return negative k for negative
			        // Omega
		return k;
	}
}
