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

/** @file Body.hpp
 * C++ API for the moordyn::Body object
 */

#pragma once

#include "Misc.hpp"
#include "IO.hpp"
#include <vector>
#include <utility>

using namespace std;

namespace moordyn {

class Waves;
class Connection;
class Rod;

/** @class Body Body.hpp
 * @brief A rigid body
 *
 * Some really basic dynamics are implemented for rigid bodiesout of the box,
 * which can be anyway extended through the usage of coupled ones.
 *
 * In the configuration file the options are:
 *
 * Name/ID, X0, Y0, Z0, Xcg, Ycg, Zcg, M, V, IX, IY, IZ, CdA-x,y,z Ca-x,y,z
 */
class Body : public io::IO
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	Body(moordyn::Log* log);

	/** @brief Destructor
	 */
	~Body();

	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCond* env;
	/// global Waves object
	moordyn::Waves* waves;

	// unique to Body:
	/// Connections attached to this body
	std::vector<moordyn::Connection*> attachedC;
	/// Rods attached to this body
	std::vector<Rod*> attachedR;

	/// Attachment points of each connection
	std::vector<vec> rConnectRel;
	/// Attachment points of each rod
	std::vector<vec6> r6RodRel;

	// Constants set at startup from input file

	/// The reference point
	vec6 body_r6;
	/// The center of gravity
	vec body_rCG;
	/// The mass
	real bodyM;
	/// The volume
	real bodyV;
	/// The inertia diagonal components
	vec bodyI;
	/// The drag coefficients
	vec6 bodyCdA;
	/// The added mass coefficients
	vec6 bodyCa;

	// degrees of freedom (or states)
	/// body 6dof position [x/y/z]
	vec6 r6;
	/// body 6dof velocity[x/y/z]
	vec6 v6;

	/// simulation time
	real t;
	/// simulation time current integration was started at (used for BC
	/// function)
	real t0;
	/// fairlead position for coupled bodies [x/y/z]
	vec6 r_ves;
	/// fairlead velocity for coupled bodies [x/y/z]
	vec6 rd_ves;

	/// total force and moment vector on node
	vec6 F6net;

	/// total body mass + added mass matrix including all elements
	mat6 M;
	/// starting mass and added mass matrix (6x6) of body without any rod
	/// elements in inertital orientation
	mat6 M0;

	/// orientation matrix of body (rotation matrix that gets it to its current
	/// orientation)
	mat OrMat;

	/// wave velocities at body reference point
	vec U;
	/// wave accelerations
	vec Ud;

	/// Pointer to moordyn::MoorDyn::outfileMain
	ofstream* outfile;

	/** @brief Types of bodies
	 */
	typedef enum
	{
		/// Is coupled, i.e. is controlled by the user
		COUPLED = -1,
		/// Is free to move, controlled by MoorDyn
		FREE = 0,
		/// Is fixed, either to a location or to another moving entity
		FIXED = 1,
		// Some aliases
		VESSEL = COUPLED,
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
			case FREE:
				return "FREE";
			case FIXED:
				return "FIXED";
		}
		return "UNKNOWN";
	}

	/// Body ID
	int number;
	/// Type of body
	types type;

	/** @brief Setup a rigid body
	 * @param number Body ID
	 * @param type Body type
	 * @param r6 6dof position
	 * @param rCG Center of gravity position
	 * @param M Mass
	 * @param V Volume
	 * @param I Inertia (diagonal matrix components)
	 * @param CdA Drag coefficient
	 * @param Ca Added mass coefficient
	 * @param outfile The outfile where information shall be witten
	 */
	void setup(int number,
	           types type,
	           vec6 r6,
	           vec rCG,
	           real M,
	           real V,
	           vec I,
	           vec6 CdA,
	           vec6 Ca,
	           shared_ptr<ofstream> outfile);

	/** @brief Attach a connection to the body
	 * @param conn The connection
	 * @param coords The fixation point
	 * @throw moordyn::invalid_value_error If @p conn is NULL
	 */
	void addConnection(moordyn::Connection* conn, vec coords);

	/** @brief Attach a rod to the body
	 * @param rod The rod
	 * @param coords The fixation point (6dof)
	 * @throw moordyn::invalid_value_error If @p rod is NULL
	 */
	void addRod(Rod* rod, vec6 coords);

	/** @brief Initialize the body that aren't free i.e. don't have states
	 *
	 * Those are the bodies with types moordyn::Body::COUPLED and
	 * moordyn::Body::FIXED
	 * @param r The position (6 dof)
	 * @param rd The velocity (6 dof)
	 * @param time The simulation time
	 * @throw moordyn::invalid_value_error If the body is of type
	 * moordyn::Body::FREE
	 */
	void initializeUnfreeBody(vec6 r = vec6::Zero(),
	                          vec6 rd = vec6::Zero(),
	                          real time = 0.0);

	/** @brief Initialize the FREE connection state
	 * @return The 6-dof position (first) and the 6-dof velocity (second)
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::FREE
	 * @throws moordyn::output_file_error If an outfile has been provided, but
	 * it cannot be written
	 */
	std::pair<vec6, vec6> initialize();

	/** @brief Initialize the free body
	 *
	 * Those are the bodies with type moordyn::Body::FREE
	 * @param r The position (6 dof)
	 * @param rd The velocity (6 dof)
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::FREE
	 * @throws moordyn::output_file_error If an outfile has been provided, but
	 * it cannot be written
	 */
	void initializeBody(vec6 r = vec6::Zero(), vec6 rd = vec6::Zero());

	/** @brief Set the environmental data
	 * @param env_in Global struct that holds environmental settings
	 * @param waves_in Global Waves object
	 */
	void setEnv(EnvCond* env_in, moordyn::Waves* waves_in);

	/** @brief set the states (positions and velocities) to all the attached
	 * entities
	 * @throws moordyn::invalid_value_error If a non FIXED connection is tried
	 * to be edited
	 */
	void setDependentStates();

	/** @brief Get the body kinematics
	 * @param pos The output position
	 * @param vel The output velocity
	 */
	inline void getState(vec6& pos, vec6& vel) const
	{
		pos = r6;
		vel = v6;
	}

	/** @brief Get the body kinematics
	 * @param pos The output position
	 * @param vel The output velocity
	 */
	inline std::pair<vec6, vec6> getState() const
	{
		return std::make_pair(r6, v6);
	}

	/** @brief Get the body position
	 * @return The body position
	 */
	inline vec getPosition() const { return r6(Eigen::seqN(0, 3)); }

	/** @brief Get the body Euler XYZ angles
	 * @return The body Euler XYZ angles
	 */
	inline vec getAngles() const { return r6(Eigen::seqN(3, 3)); }

	/** @brief Get the body velocity
	 * @return The body velocity
	 */
	inline vec getVelocity() const { return v6(Eigen::seqN(0, 3)); }

	/** @brief Get the body angular velocity
	 * @return The body angular velocity
	 */
	inline vec getAngularVelocity() const { return v6(Eigen::seqN(3, 3)); }

	/** @brief Get the forces and moments exerted over the body
	 * @return The net force
	 */
	inline vec6 getFnet() const { return F6net; }

	/** @brief Get the mass and intertia matrix
	 * @return The mass and inertia matrix
	 */
	inline mat6 getM() const { return M; };

	/** @brief Get body output
	 *
	 * This funtion is useful when output options are set in the system
	 * @param outChan The output channel/field
	 * @return The output value, 0.0 if a non-valid field is set
	 */
	real GetBodyOutput(OutChanProps outChan);

	/** @brief Scale the drag coefficients
	 * @param scaler The drag coefficients scale factor
	 */
	inline void scaleDrag(real scaler) { bodyCdA *= scaler; }

	/** @brief Set the simulation time
	 * @param time The simulation time
	 */
	inline void setTime(real time) { t = time; }

	/** @brief Called at the beginning of each coupling step to update the
	 * boundary conditions (body kinematics) for the proceeding time steps
	 * @param r The output position
	 * @param rd The output velocity
	 * @param time The simulation time
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::COUPLED or moordyn::Body::FIXED
	 */
	void initiateStep(vec6 r, vec6 rd, real time);

	/** @brief Sets the kinematics
	 *
	 * This function is meant only for coupled or fixed bodies
	 * @param time The simulation time
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::COUPLED or moordyn::Body::FIXED
	 */
	void updateFairlead(real time);

	/** @brief Set the latest states to the body
	 * @param r The position
	 * @param rd The velocity
	 * @param time The simulation time
	 */
	void setState(vec6 r, vec6 rd, real time);

	/** @brief calculate the forces and state derivatives of the body
	 *
	 * This function is only meant for free bodies
	 * @param return The states derivatives, i.e. the velocity (first) and the
	 * acceleration (second)
	 * @throw moordyn::invalid_value_error If the body is of type
	 * moordyn::Body::FREE
	 */
	std::pair<vec6, vec6> getStateDeriv();

	/** @brief calculates the forces on the body
	 * @throw moordyn::invalid_value_error If the body is of type
	 * moordyn::Body::FREE
	 */
	void doRHS();

	void Output(real time);

	/** @brief Produce the packed data to be saved
	 *
	 * The produced data can be used afterwards to restore the saved information
	 * afterwards calling Deserialize(void).
	 *
	 * Thus, this function is not processing the information that is extracted
	 * from the definition file
	 * @return The packed data
	 */
	virtual std::vector<uint64_t> Serialize(void);

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	virtual uint64_t* Deserialize(const uint64_t* data);

#ifdef USEGL
	void drawGL(void);
#endif
};

} // ::moordyn
