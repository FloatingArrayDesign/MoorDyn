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

/** @file Waves.hpp
 * C++ API for the moordyn::Waves object
 */

#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include "Line.hpp"
#include "Point.hpp"
#include "Body.hpp"
#include "Rod.hpp"
#include "Waves/SpectrumKin.hpp"
#include <vector>

namespace moordyn {

class TimeScheme;
class Seafloor;
typedef std::shared_ptr<Seafloor> SeafloorRef;

/// STL std::vector of 2 dimensions
template<class T>
using Vec2D = std::vector<std::vector<T>>;

/// STL std::vector of 3 dimensions
template<class T>
using Vec3D = std::vector<std::vector<std::vector<T>>>;

/// STL std::vector of 4 dimensions
template<class T>
using Vec4D = std::vector<std::vector<std::vector<std::vector<T>>>>;

/** @brief Make a 2-D data grid
 * @param nx Number of components in the first dimension
 * @param ny Number of components in the second dimension
 */
static inline Vec2D<real>
init2DArray(unsigned int nx, unsigned int ny)
{
	return Vec2D<real>(nx, std::vector<real>(ny, 0.0));
}

/** @brief Make a 3-D data grid
 * @param nx Number of components in the first dimension
 * @param ny Number of components in the second dimension
 * @param nz Number of components in the third dimension
 */
static inline Vec3D<real>
init3DArray(unsigned int nx, unsigned int ny, unsigned int nz)
{
	return Vec3D<real>(nx, init2DArray(ny, nz));
}

/** @brief Make a 4-D data grid
 * @param nx Number of components in the first dimension
 * @param ny Number of components in the second dimension
 * @param nz Number of components in the third dimension
 * @param nw Number of components in the third dimension
 */
static inline Vec4D<vec3>
init4DArrayVec(unsigned int nx,
               unsigned int ny,
               unsigned int nz,
               unsigned int nw)
{
	return Vec4D<vec3>(
	    nx,
	    Vec3D<vec3>(ny, Vec2D<vec3>(nz, std::vector<vec3>(nw, vec3::Zero()))));
}
/** @brief Make a 4-D data grid
 * @param nx Number of components in the first dimension
 * @param ny Number of components in the second dimension
 * @param nz Number of components in the third dimension
 * @param nw Number of components in the third dimension
 */
static inline Vec4D<real>
init4DArray(unsigned int nx, unsigned int ny, unsigned int nz, unsigned int nw)
{
	return Vec4D<real>(nx, init3DArray(ny, nz, nw));
}

/** @brief Helper for moordyn::AbstractCurrentKin
 */
struct SeafloorProvider
{
	real waterDepth;
	SeafloorRef seafloor{};
	real getAverageDepth() const;
	real getDepth(const vec2& pos) const;
};

/**
 * @brief An abstract class representing the capability of providing water
 * current data at some point and time.
 */
class AbstractCurrentKin
{
  public:
	virtual ~AbstractCurrentKin(){};
	/** @brief Get the velocity and acceleration at a specific position and time
	 *
	 * @param pos The location
	 * @param time The time
	 * @param seafloor A SeafloorProvider, could be used for kinematic
	 * stretching
	 * @param vel The output velocity, not set if null
	 * @param acc The output acceleration, not set if null
	 */
	virtual void getCurrentKin(const vec3& pos,
	                           real time,
	                           const SeafloorProvider& seafloor,
	                           vec3* vel,
	                           vec3* acc) = 0;
};

/**
 * @brief An abstract class representing having the capability of providing wave
 * kinematics data
 *
 */
class AbstractWaveKin
{
  public:
	virtual ~AbstractWaveKin(){};
	/** @brief Get the velocity, acceleration, wave height and dynamic pressure
	 * at a specific position and time
	 * @param pos The location
	 * @param time The time
	 * @param seafloor A SeafloorProvider used for kinematic stretching
	 * @param zeta The output wave height, not set if null
	 * @param U The output velocity, not set if null
	 * @param Ud The output acceleration, not set if null
	 * @param PDyn The output dynamic pressure, not set if null
	 */
	virtual void getWaveKin(const vec3& pos,
	                        real time,
	                        const SeafloorProvider& seafloor,
	                        real* zeta,
	                        vec3* vel,
	                        vec3* acc,
	                        real* pdyn) = 0;
};

/**
 * @brief Wrapper around waves::SpectrumKin to make it adhere to the
 * AbstractWaveKin interface
 *
 * This probably shouldn't exist, and SpectrumKin should do this by itself, but
 * this is easy enough
 *
 */
class SpectrumKinWrapper : public AbstractWaveKin
{
	waves::SpectrumKin spectrumKin;

  public:
	/**
	 * @brief Construct a new Spectrum Kin Wrapper object
	 *
	 * SpectrumKin should be fully initialized, including having called setup.
	 * Move the spectrum into the wrapper just to avoid an unnecessary copy.
	 *
	 * @param spectrum
	 */
	SpectrumKinWrapper(waves::SpectrumKin&& spectrum)
	  : spectrumKin(spectrum)
	{
	}

	~SpectrumKinWrapper() override = default;

	void getWaveKin(const vec3& pos,
	                real time,
	                const SeafloorProvider& seafloor,
	                real* zeta,
	                vec3* vel,
	                vec3* acc,
	                real* pdyn) override
	{
		if (pdyn) {
			*pdyn = 0.0;
		}

		real avgDepth = seafloor.getAverageDepth();
		real actualDepth = seafloor.getDepth(pos.head<2>());
		spectrumKin.getWaveKin(
		    pos, time, avgDepth, actualDepth, zeta, vel, acc);
	}
};
/**
 * @brief A rectilinear grid with x, y, z, and t axes.
 *
 * The x, y, and z axis are defined by arbitrary lists of sorted values, and the
 * time axis is represented by some number of equally spaced times starting at
 * zero.
 *
 */
class GridXYZT
{
  public:
	GridXYZT(const std::vector<real>& px,
	         const std::vector<real>& py,
	         const std::vector<real>& pz,
	         unsigned int nt,
	         real dt)
	  : nx(static_cast<unsigned int>(px.size()))
	  , ny(static_cast<unsigned int>(py.size()))
	  , nz(static_cast<unsigned int>(pz.size()))
	  , nt(nt)
	  , dtWave(dt)
	  , px(px)
	  , py(py)
	  , pz(pz)
	{
	}
	/// number of grid points in x direction
	unsigned int nx;
	/// number of grid points in y direction
	unsigned int ny;
	/// number of grid points in z direction
	unsigned int nz;
	/// number of time steps used in wave kinematics time series
	unsigned int nt = 0;
	/// time step for wave kinematics time series
	real dtWave;

  protected:
	/// grid x coordinate arrays
	std::vector<real> px;
	/// grid y coordinate arrays
	std::vector<real> py;
	/// grid z coordinate arrays
	std::vector<real> pz;
};

/**
 * @brief Contains the data and functionality for the Wave grid kinematics modes
 *
 * Is an AbstractWaveKin
 *
 */
class WaveGrid
  : public AbstractWaveKin
  , public GridXYZT
  , LogUser
{
  public:
	WaveGrid(moordyn::Log* log,
	         const std::vector<real>& px,
	         const std::vector<real>& py,
	         const std::vector<real>& pz,
	         unsigned int nt,
	         real dt)
	  : GridXYZT(px, py, pz, nt, dt)
	  , LogUser(log)
	{
	}
	~WaveGrid() override = default;

	void allocateKinematicArrays();

	void getWaveKin(const vec3& pos,
	                real time,
	                const SeafloorProvider& seafloor,
	                real* zeta,
	                vec3* vel,
	                vec3* acc,
	                real* pdyn) override;

	inline Vec3D<real>& Zetas() { return zetas; }
	inline const Vec3D<real>& getZetas() const { return zetas; }

	inline Vec4D<real>& PDyn() { return pDyn; }
	inline const Vec4D<real>& getPDyn() const { return pDyn; }

	inline Vec4D<vec3>& WaveVel() { return wave_vel; }
	inline const Vec4D<vec3>& getWaveVel() const { return wave_vel; }

	inline Vec4D<vec3>& WaveAcc() { return wave_acc; }
	inline const Vec4D<vec3>& getWaveAcc() const { return wave_acc; }

	inline const std::vector<real>& Px() const { return px; }
	inline const std::vector<real>& Py() const { return py; }
	inline const std::vector<real>& Pz() const { return pz; }

  private:
	/// wave elevation [x,y,t]
	Vec3D<real> zetas;
	/// dynamic pressure [x,y,z,t]
	Vec4D<real> pDyn;
	/// Wave velocity [x, y, z, t]
	Vec4D<vec3> wave_vel;
	/// Wave acceleration [x, y, z, t]
	Vec4D<vec3> wave_acc;
};

/**
 * @brief Contains grid based current data
 *
 * Is an AbstractCurrentKin
 */
class CurrentGrid
  : public AbstractCurrentKin
  , public GridXYZT
  , LogUser
{
  public:
	CurrentGrid(moordyn::Log* log,
	            const std::vector<real>& px,
	            const std::vector<real>& py,
	            const std::vector<real>& pz,
	            unsigned int nt,
	            real dt)
	  : GridXYZT(px, py, pz, nt, dt)
	  , LogUser(log)
	{
	}

	~CurrentGrid() override = default;

	void allocateKinematicArrays();

	void getCurrentKin(const vec3& pos,
	                   real time,
	                   const SeafloorProvider& seafloor,
	                   vec3* vel,
	                   vec3* acc) override;

	inline Vec4D<vec3>& CurrentVel() { return current_vel; }
	inline const Vec4D<vec3>& getCurrentVel() const { return current_vel; }

	inline Vec4D<vec3>& CurrentAcc() { return current_acc; }
	inline const Vec4D<vec3>& getCurrentAcc() const { return current_acc; }

	inline const std::vector<real>& Px() const { return px; }
	inline const std::vector<real>& Py() const { return py; }
	inline const std::vector<real>& Pz() const { return pz; }

  private:
	/// Current velocity [x, y, z, t]
	Vec4D<vec3> current_vel;
	/// Current acceleration [x, y, z, t]
	Vec4D<vec3> current_acc;
};

/** @class Waves Waves.hpp
 * @brief Class that handles wave and current kinematics
 *
 * This class keep track of all the structures (lines, points, rods,
 * bodies, etc) being simulated. For each of these structures, it stores the
 * water and wave kinematics for each node of the structure. The Waves class is
 * in charge of understanding the wave/current settings and following through
 * with the correct behavior.
 *
 * Waves and current kinematics can be calculated by subclasses of
 * AbstractWaveKin and AbstractCurrentKin. This makes it simple to add new wave
 * or current kinematics modes, assuming they can adhere to the simple
 * interface. The waves class itself just stores an AbstractWaveKin and
 * AbstractCurrentKin, it has no ability to compute waves or currents on its
 * own.
 *
 */
class Waves : public LogUser
{
  public:
	/// Constructor
	Waves(moordyn::Log* log);
	/// Destructor
	~Waves();

	/**
	 * @brief Get the positions of all the nodes with wave kinematics
	 *
	 * Returns a vector of the positions of all the line nodes, points, rod
	 * nodes, bodies, etc at the current time.
	 * @return std::vector<vec3> Position of all the structural nodes with wave
	 * kinematics
	 */
	std::vector<vec3> getWaveKinematicsPoints();

	/**
	 * @brief Set the wave kinematics for all the structural nodes
	 *
	 * This expects the order of the points to be the same as what was returned
	 * by getWaveKinematicsPoints
	 * @param U Water velocities at the nodes
	 * @param Ud Water accelerations at the nodes
	 */
	void setWaveKinematics(std::vector<vec> const& U,
	                       std::vector<vec> const& Ud);

	/**
	 * @brief Recalculates any wave and current kinematics for all the nodes
	 *
	 */
	void updateWaves();

	/**
	 * @brief Adds a line to the list of structures we calculate water
	 * kinematics for
	 *
	 * Should be called after Waves::setup
	 * @param line
	 */
	void addLine(moordyn::Line* line);

	/**
	 * @brief Adds a rod to the list of structures we calculate water kinematics
	 * for
	 *
	 * Should be called after Waves::setup
	 * @param rod
	 */
	void addRod(moordyn::Rod* rod);

	/**
	 * @brief Adds a body to the list of structures we calculate water
	 * kinematics for
	 *
	 * Should be called after Waves::setup
	 * @param body
	 */
	void addBody(moordyn::Body* body);

	/**
	 * @brief Adds a point to the list of structures we calculate water
	 * kinematics for
	 *
	 * Should be called after Waves::setup
	 * @param point
	 */
	void addPoint(moordyn::Point* point);

	using NodeKinReturnType = std::tuple<const std::vector<real>&,
	                                     const std::vector<vec3>&,
	                                     const std::vector<vec3>&,
	                                     const std::vector<real>&>;
	/**
	 * @brief Get the water kinematics for the line with id
	 *
	 * @param lineId Id of the line
	 * @return NodeKinReturnType (zeta, U, Ud, pDyn)
	 */
	NodeKinReturnType getWaveKinLine(size_t lineId);

	/**
	 * @brief Get the water kinematics for the rod with given Id.
	 *
	 * @param rodId Id of the rod
	 * @return NodeKinReturnType (zeta, U, Ud, pDyn)
	 */
	NodeKinReturnType getWaveKinRod(size_t rodId);

	/**
	 * @brief Get the water kinematics for the body with given Id
	 *
	 * The vectors should all be of length 1
	 *
	 * @param bodyId Id of the body
	 * @return NodeKinReturnType (zeta, U, Ud, pDyn)
	 */
	NodeKinReturnType getWaveKinBody(size_t bodyId);

	/**
	 * @brief Get the water kinematics for the point with given Id
	 *
	 * The vectors should all be of length 1
	 *
	 * @param pointId Id of the points
	 * @return NodeKinReturnType (zeta, U, Ud, pDyn)
	 */
	NodeKinReturnType getWaveKinPoint(size_t pointId);

	/**
	 * @brief Gets the surface height at a given (x, y) point
	 *
	 * Used mainly for debugging and visualization purposes
	 *
	 * @param point
	 * @return real
	 */
	real getWaveHeightPoint(vec2 point);

	/**
	 * @brief Get the wave kinematics at a point at the current time
	 *
	 * @param pos Point
	 * @param seafloor O
	 * @param zeta
	 * @param vel
	 * @param acc
	 * @param pdyn
	 */
	void getWaveKin(const vec3& pos,
	                real& zeta,
	                vec3& vel,
	                vec3& acc,
	                real& pdyn,
	                Seafloor* seafloor = nullptr);

  private:
	/**
	 * @brief Holds the water kinematics for all of the structures of a certain
	 * type
	 *
	 * Mostly just to reduce code repetition
	 *
	 * @tparam C Type of the structure
	 */
	template<class C>
	struct NodeKinematics
	{
		std::vector<const C*> structures;
		std::vector<std::vector<real>> zetas;
		std::vector<std::vector<vec3>> U;
		std::vector<std::vector<vec3>> Ud;
		std::vector<std::vector<real>> Pdyn;

		NodeKinReturnType operator[](size_t idx)
		{

			return { zetas[idx], U[idx], Ud[idx], Pdyn[idx] };
		}
	};

	/**
	 * @brief Holds all of the water kinematic data for the simulation
	 *
	 * This is its own structure because when we hae separate current and wave
	 * kinematics, we need 3 copies of this
	 */
	struct AllNodesKin
	{
		NodeKinematics<moordyn::Line> lines;
		NodeKinematics<moordyn::Body> bodies;
		NodeKinematics<moordyn::Rod> rods;
		NodeKinematics<moordyn::Point> points;
	};

	/// The final computed water kinematics at each node
	AllNodesKin nodeKin;
	/// The contribution of waves to the water kinematics at each node
	/// Only used when there are external wave kinematics and internal currents
	AllNodesKin waveKin;
	/**
	 * @brief Used for adding to a NodeKinematics instance
	 *
	 * @tparam C The type of structure being added
	 * @param structure The instance being added
	 * @param num_nodes Number of nodes in the instance
	 * @param nodeKinematics A NodeKinematics instance for the structure type
	 */
	template<class C>
	void genericAdd(const C* const structure,
	                unsigned int num_nodes,
	                NodeKinematics<C>& nodeKinematics)
	{
		nodeKinematics.structures.push_back(structure);
		nodeKinematics.zetas.emplace_back(num_nodes, 0.0);
		nodeKinematics.U.emplace_back(num_nodes, vec3::Zero());
		nodeKinematics.Ud.emplace_back(num_nodes, vec3::Zero());
		nodeKinematics.Pdyn.emplace_back(num_nodes, 0.0);
	}

	/**
	 * @brief Helper function that calls the given function on every node
	 * AlNodesKin object
	 *
	 * @tparam F Callable object/function with signature void f(vec pos, vec& U,
	 * vec& Ud, real& zeta, real& pdyn)
	 * @param nodeKinematics
	 * @param f Function to call for every node
	 */
	template<typename F>
	void kinematicsForAllNodes(AllNodesKin& nodeKinematics, F f);

	/// The generic wave kinematics provider object
	std::unique_ptr<AbstractWaveKin> waveKinematics{};
	/// The generic current kinematics provider object
	std::unique_ptr<AbstractCurrentKin> currentKinematics{};

	/**
	 * @brief A member for temporary storage of wave grids.
	 *
	 * This exists because in cases where we want to unify wave and current
	 * grids, it's preferable to have an object that is known to be a wave grid
	 * rather than trying to do runtime type information deduction.
	 */
	std::unique_ptr<WaveGrid> waveGrid{};

	/// Keep a reference to the environment
	EnvCondRef env{};
	// Keep a reference to any potential 3d seafloor
	SeafloorRef seafloor{};
	/// gravity acceleration
	real g;
	/// water density
	real rho_w;

	/// The time integration scheme
	moordyn::TimeScheme* _t_integrator;

	// ------------ from Line object... -----------
	// new additions for handling waves in-object and precalculating them	(not
	// necessarily used right now)
	//	int WaveMod;
	//	int WaveStMod;
	//	double Hs;
	//	double Tp;
	//	double gamma;
	//	float beta; 			// wave heading
	//
	//	vector< double > Ucurrent; // constant uniform current to add (three
	// components)

  public:
	/** @brief Types of coordinates input on the grid file
	 */
	typedef enum
	{
		/// Single point (0, 0, 0)
		GRID_SINGLE = 0,
		/// List of points
		GRID_LIST = 1,
		/// EQUISPACED POINTS
		GRID_LATTICE = 2,
	} coordtypes;

	/** @brief Setup the wave kinematics
	 *
	 * Always call this function after the construtor
	 * @param env The enviromental options
	 * @param t The time integration scheme
	 * @param folder The root folder where the wave data can be found
	 * @throws moordyn::input_file_error If an input file cannot be read, or if
	 * a file is ill-formatted
	 * @throws moordyn::invalid_value_error If invalid values are found
	 * @throws moordyn::non_implemented_error If WaveKin=2
	 * @throws moordyn::mem_error If there were roblems allocating memory
	 * @throws moordyn::output_file_error If data cannot be written in \p folder
	 */
	void setup(EnvCondRef env,
	           SeafloorRef seafloor,
	           TimeScheme* t,
	           const char* folder = "Mooring/");
};

// other relevant functions being thrown into this file for now (should move to
// Misc?) <<<<

/** @brief Compute the coordinates from a grid definition entry line
 * @param coordtype The type of coordinates input
 * @param entries Nothing if @p coordtype is 0; the list of coordinates if
 * @p coordtype is 1 and minimum limit; the maximum limit and the number of
 * points if @p coordtype is 2
 * @return The list of coordinates
 * @warning Memory will be allocated in coordarray. The user is responsible of
 * deallocating it afterwards
 */
std::vector<real>
gridAxisCoords(Waves::coordtypes coordtype, vector<string>& entries);

/**
 * Renaming the shared pointer to be a little more ergonomic
 */
typedef std::shared_ptr<Waves> WavesRef;
} // ::moordyn
