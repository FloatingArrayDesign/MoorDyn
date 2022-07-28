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

/** @file Rod.hpp
 * C++ API for the moordyn::Rod object
 */

#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include <vector>
#include <utility>

using namespace std;

namespace moordyn {

class Waves;
class Line;

/** @class Rod Rod.hpp
 * @brief A cylindrical rod
 *
 * Rod are cylindrical structures with much more complete dynamics than
 * moordyn::Body.
 *
 * Each end point of the rod can be fixed or pinned to another object, let free
 * or control it externally
 */
class Rod : public LogUser
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	Rod(moordyn::Log* log);

	/** @brief Destructor
	 */
	~Rod();

  private:
	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCond* env;
	/// global Waves object
	moordyn::Waves* waves;

	/// Attached lines to the rod
	typedef struct _attachment
	{
		/// The attached line
		Line* line;
		/// The attachment end point
		EndPoints end_point;
	} attachment;

	/// Lines attached to the end point A of the rod
	std::vector<attachment> attachedA;

	/// Lines attached to the end point B of the rod
	std::vector<attachment> attachedB;

	// ROD STUFF

	// parameters
	/// Number of line segments
	unsigned int N;
	/// The constrained length of the rod
	moordyn::real UnstrLen;
	/// rod diameter
	moordyn::real d;
	/// rod linear density
	moordyn::real rho;
	/// normal added mass coefficient [-]
	/// with respect to rod displacement
	moordyn::real Can;
	/// axial added mass coefficient [-]
	/// with respect to rod displacement
	moordyn::real Cat;
	/// normal drag coefficient [-]
	/// with respect to frontal area, \f$ d l \f$
	moordyn::real Cdn;
	/// axial drag coefficient [-]
	/// with respect to surface area, \f$ \pi d l \f$
	moordyn::real Cdt;

	/// Rod 6dof position [x,y,z,u1,u2,u3] (end A coordinates and direction unit
	/// vector)
	vec6 r6;
	/// Rod 6dof velocity[vx,vy,vz,wx,wy,wz] (end A velocity and rotational
	/// velocities about unrotated axes)
	vec6 v6;

	// kinematics
	/// node positions
	std::vector<vec> r;
	/// node velocities
	std::vector<vec> rd;
	/// unit tangent vector for the whole rod
	vec q;
	/// unstretched rod segment lengths
	std::vector<moordyn::real> l;

	/// node mass + added mass matrix
	std::vector<mat> M;
	// line segment volumes
	std::vector<moordyn::real> V;

	/// external forces from attached lines on/about end A
	vec FextA;
	/// external forces from attached lines on/about end B
	vec FextB;
	/// external moments (from attached cables or waterplane hydrostatic moment)
	vec Mext;
	/// total force and moment about end A (excluding inertial loads) that Rod
	/// may exert on whatever it's attached to
	vec6 F6net;
	/// total mass matrix about end A of Rod and any attached Points
	mat6 M6net;

	// forces
	/// node dry weights
	std::vector<vec> W;
	/// node bouyancy
	std::vector<vec> Bo;
	/// Dynamic pressure
	std::vector<vec> Pd;
	/// node drag (transversal)
	std::vector<vec> Dp;
	/// node drag (axial)
	std::vector<vec> Dq;
	/// node added mass forcing (transversal)
	std::vector<vec> Ap;
	/// node added mass forcing (axial)
	std::vector<vec> Aq;
	/// node bottom contact force
	std::vector<vec> B;
	/// total force on node
	/// @deprecated Might remove this for Rods
	std::vector<vec> Fnet;

	// wave things
	/// VOF scalar for each segment (1 = fully submerged, 0 = out of water)
	std::vector<moordyn::real> F;
	/// free surface elevations
	std::vector<moordyn::real> zeta;
	/// dynamic pressures
	std::vector<moordyn::real> PDyn;
	/// wave velocities
	std::vector<vec> U;
	/// wave accelerations
	std::vector<vec> Ud;
	/// instantaneous axial submerged length [m]
	real h0;

	// time
	/// simulation time
	moordyn::real t;
	/// simulation time current integration was started at (used for BC
	/// function)
	moordyn::real t0;

	/// fairlead position for coupled rods [x/y/z]
	vec6 r_ves;
	/// fairlead velocity for coupled rods [x/y/z]
	vec6 rd_ves;

	// file stuff
	/// Pointer to moordyn::MoorDyn::outfileMain
	ofstream* outfile;
	/// A copy of moordyn::MoorDyn::outChans
	string channels;

	/** data structures for precalculated nodal water kinematics if applicable
	 * @{
	 */

	/// time series of wave elevations above each node
	std::vector<std::vector<moordyn::real>> zetaTS;
	/// Volume of fluid at each node (currently unused)
	std::vector<std::vector<moordyn::real>> FTS;
	/// time series of velocities at each node
	std::vector<std::vector<vec>> UTS;
	/// time series of accelerations at each node
	std::vector<std::vector<vec>> UdTS;
	/// number of water kinematics time steps
	unsigned int ntWater;
	/// water kinematics time step size (s)
	moordyn::real dtWater;

	/**
	 * @}
	 */

  public:
	/** @brief Types of rods
	 */
	typedef enum
	{
		/// Is attached rigidly to a coupling point (6dof)
		COUPLED = -2,
		/// Is pinned to a coupling point (3dof)
		CPLDPIN = -1,
		/// Is free to move, controlled by MoorDyn
		FREE = 0,
		/// Is pinned to a fixed point (or a body/ptfm, in which case added to
		/// list?)
		PINNED = 1,
		/// Is attached rigidly to a fixed point (or a body/ptfm, in which case
		/// added to list?)
		FIXED = 2,
		// Some aliases
		VESSEL = COUPLED,
		VESPIN = CPLDPIN,
		CONNECT = FREE,
		ANCHOR = FIXED,
	} types;

	/** @brief Return a string with the name of a type
	 *
	 * This tool is useful mainly for debugging
	 */
	static string TypeName(types t)
	{
		switch (t) {
			case COUPLED:
				return "COUPLED";
			case CPLDPIN:
				return "CPLDPIN";
			case FREE:
				return "FREE";
			case PINNED:
				return "PINNED";
			case FIXED:
				return "FIXED";
		}
		return "UNKNOWN";
	}

	/// Rod ID
	int number;
	/// Rod type
	types type;

	/// The roll angle (useful for output)
	real roll;
	/// The pitch angle (useful for output)
	real pitch;

	/** flag indicating whether wave/current kinematics will be considered
	 *
	 * Thisis not exactly a copy of EnvCond::WaveKin, but a compiled value
	 * dependig on both EnvCond::WaveKin and EnvCond::Current
	 */
	moordyn::waves_settings WaterKin;

	/** @brief Setup a rod
	 * @param number Rod ID
	 * @param type Rod type
	 * @param props Rod properties
	 * @param endCoords The coordinates of both end points
	 * @param n Number of segments
	 * @param outfile The outfile where information shall be witten
	 * @param channels The channels/fields that shall be printed in the file
	 */
	void setup(int number,
	           types type,
	           RodProps* props,
	           vec6 endCoords,
	           unsigned int n,
	           shared_ptr<ofstream> outfile,
	           string channels);

	/** @brief Attach a line endpoint to the rod end point A
	 * @param line The line to be attached
	 * @param line_end_point The line endpoint
	 * @param rod_end_point The rod endpoint
	 * @throws moordyn::invalid_value_error If the end points are not valid
	 */
	void addLine(Line* line, EndPoints line_end_point, EndPoints rod_end_point);

	/** @brief Dettach a line
	 * @param end_point The rod end point where the line is attached
	 * @param line The line
	 * @return The line end point that was attached to the rod
	 * @throws moordyn::invalid_value_error If there is no an attached line
	 * with the provided @p lineID
	 */
	EndPoints removeLine(EndPoints end_point, Line* line);

	/** @brief Set the environmental data
	 * @param env_in Global struct that holds environmental settings
	 * @param waves_in Global Waves object
	 */
	inline void setEnv(EnvCond* env_in, moordyn::Waves* waves_in)
	{
		env = env_in;
		waves = waves_in;
	}

	/** @brief Initialize the rod state
	 * @return The position and direction (first) and the linear and angular
	 * velocity (second)
	 * @note moordyn::Rod::r6 and moordyn::Rod::v6 must already be set
	 * @note ground- or body-pinned rods have already had
	 * moordyn::Rod::setKinematics() called
	 * @note In the case of pinned rods, just the rod directions and angular
	 * velocites shall be considered
	 */
	std::pair<vec6, vec6> initialize();

	/** @brief Number of segments
	 *
	 * The number of nodes can be computed as moordyn::Rod::getN() + 1
	 * @return The number of segments, moordyn::Rod::N
	 */
	inline unsigned int getN() const { return N; }

	/** @brief Get the position of a node
	 * @param i The line node index
	 * @return The position
	 * @throws invalid_value_error If the node index \p i is bigger than the
	 * number of nodes, moordyn::Line::N + 1
	 */
	inline vec getNodePos(unsigned int i) const
	{
		if (i > N) {
			LOGERR << "Asking node " << i << " of rod " << number
			       << ", which only has " << N + 1 << " nodes" << std::endl;
			throw moordyn::invalid_value_error("Invalid node index");
		}
		return r[i];
	}

	/** @brief Get the velocity of a node
	 * @param i The line node index
	 * @return The velocity
	 * @throws invalid_value_error If the node index \p i is bigger than the
	 * number of nodes, moordyn::Line::N + 1
	 */
	inline vec getNodeVel(unsigned int i) const
	{
		if (i > N) {
			LOGERR << "Asking node " << i << " of rod " << number
			       << ", which only has " << N + 1 << " nodes" << std::endl;
			throw moordyn::invalid_value_error("Invalid node index");
		}
		return rd[i];
	}

	/** @brief Get rod output
	 *
	 * This funtion is useful when outputs are set in the rod properties
	 * @param outChan The output channel/field
	 * @return The output value, 0.0 if a non-valid field is set
	 */
	real GetRodOutput(OutChanProps outChan);

	/** @brief store wave/current kinematics time series for this line
	 *
	 * This is used when nodal approaches are selected, i.e.
	 * WaveKin = WAVES_FFT_NODE or WAVES_NODE, Currents = CURRENTS_STEADY_NODE
	 * or CURRENTS_DYNAMIC_NODE
	 * @param dt Time step
	 * @param zeta The wave elevations
	 * @param f The fluid fractions
	 * @param u The flow velocity
	 * @param ud The flow acceleration
	 * @throws invalid_value_error If @p zeta, @p f, @p u and @p ud have not the
	 * same size, in both dimensions.
	 * @throws invalid_value_error If the length of @p zeta, @p f, @p u or @p ud
	 * is not equal to moordyn::Line::getN() + 1
	 * @note Working in progress
	 */
	void storeWaterKin(real dt,
	                   std::vector<std::vector<moordyn::real>> zeta,
	                   std::vector<std::vector<moordyn::real>> f,
	                   std::vector<std::vector<vec>> u,
	                   std::vector<std::vector<vec>> ud);

	/** @brief Get the drag coefficients
	 * @return The normal (transversal) and tangential (axial) drag coefficients
	 */
	inline std::pair<real, real> getDrag() const { return make_pair(Cdn, Cdt); }

	/** @brief Set the drag coefficients
	 * @param cdn Normal (transversal) coefficient
	 * @param cdt tangential (axial) coefficient
	 */
	inline void setDrag(real cdn, real cdt)
	{
		Cdn = cdn;
		Cdt = cdt;
	}

	/** @brief Scale the drag coefficients
	 * @param scaler The drag coefficients scale factor
	 */
	inline void scaleDrag(real scaler)
	{
		Cdn = Cdn * scaler;
		Cdt = Cdt * scaler;
	}

	/** @brief Set the line  simulation time
	 * @param time Simulation time
	 */
	inline void setTime(real time) { t = time; }

	/** @brief Set the rod state
	 *
	 * for a free Rod, there are 12 states:
	 * [x, y, z velocity of end A, then rate of change of u/v/w coordinates of
	 * unit vector pointing toward end B, then x, y, z coordinate of end A,
	 * u/v/w coordinates of unit vector pointing toward end B]
	 *
	 * for a pinned Rod, there are 6 states (rotational only):
	 * [rate of change of u/v/w coordinates of unit vector pointing toward end
	 * B, then u/v/w coordinates of unit vector pointing toward end B]
	 *
	 * @param pos The position and direction (position is ignored in pinned
	 * rods)
	 * @param vel The linear and angular velocity (linear one is ignored in
	 * pinned rods)
	 * @param time Simulation time
	 * @throws invalid_value_error If the rod is not of type FREE, CPLDPIN or
	 * PINNED
	 */
	void setState(vec6 pos, vec6 vel, double time);

	/** @brief Called at the beginning of each coupling step to update the
	 * boundary conditions (rod kinematics) for the proceeding time steps
	 * @param r The output position
	 * @param rd The output velocity
	 * @param time The simulation time
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 * @note If the rod is of type moordyn::Rod::CPLDPIN, then just 3 components
	 * of @p r and @p rd are considered
	 */
	void initiateStep(vec6 r, vec6 rd, real time);

	/** @brief Sets the kinematics
	 *
	 * This function is meant only for coupled or fixed bodies
	 * @param time The simulation time
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 */
	void updateFairlead(real time);

	/** @brief Take the kinematics from the fairlead information
	 *
	 * set kinematics for Rods ONLY if they are attached to a body (including a
	 * coupled body) (otherwise shouldn't be called)
	 * @param r Position
	 * @param rd Velocity
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::FIXED or moordyn::Rod::PINNED
	 * @note If the rod is of type moordyn::Rod::PINNED, then just 3 components
	 * of @p r and @p rd are considered
	 */
	void setKinematics(vec6 r, vec6 rd);

	/** @brief Set the end kinematics then set the states (positions and
	 * velocities) of any line ends attached to this rod.
	 *
	 * This also determines the orientation of zero-length rods.
	 */
	void setDependentStates();

	/** @brief calculate the forces and state derivatives of the rod
	 * @return The linear and angular velocity (first), and the linear and
	 * angular accelerations (second)
	 * @throws nan_error If nan values are detected in any node position
	 * @note The returned linear velocity and accelerations for pinned rods
	 * should be ignored
	 */
	std::pair<vec6, vec6> getStateDeriv();

	/** @brief function to return net force on rod (and possibly moment at end A
	 * if it's not pinned)
	 * @return The net force
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 */
	vec6 getFnet();

	/** @brief Calculate the force and mass contributions of the connect on the
	 * parent body
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 * @param rBody The body position. If NULL, {0, 0, 0} is considered
	 */
	void getNetForceAndMass(vec6& Fnet_out, mat6& M_out, vec rBody);

	/** @brief Calculate the force and mass contributions of the connect on the
	 * parent body
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 */
	inline void getNetForceAndMass(vec6& Fnet_out, mat6& M_out)
	{
		getNetForceAndMass(Fnet_out, M_out, r[0]);
	}

	/** @brief This is the big function that calculates the forces on the rod,
	 * including from attached lines
	 */
	void doRHS();

	void Output(real);

#ifdef USEGL
	void drawGL(void);
	void drawGL2(void);
#endif
};

} // ::moordyn
