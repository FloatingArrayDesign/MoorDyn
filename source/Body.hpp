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
 *
 * Body objects provide generic 6DOF rigid-body representations based on a
 * lumped-parameter model of translational and rotational properties. Rod
 * and Line objects may be added to body objects at any location. This allows
 * the user to simulate a variety of submerged structures.
 */

#pragma once

#include "Misc.hpp"
#include "IO.hpp"
#include "Util/CFL.hpp"
#include <vector>
#include <utility>

#ifdef USE_VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#endif

using namespace std;

namespace moordyn {

class Waves;
typedef std::shared_ptr<Waves> WavesRef;

class Point;
class Rod;

/** @class Body Body.hpp
 * @brief A rigid body
 *
 * Some really basic dynamics are implemented for rigid bodies out of the box,
 * which can be extended through the usage of coupled ones.
 *
 * In the configuration file the options are:
 *
 * Name/ID, X0, Y0, Z0, Xcg, Ycg, Zcg, M, V, IX, IY, IZ, CdA-x,y,z Ca-x,y,z
 *
 * moordyn::Body extends the io::IO class, allowing it to perform input/output
 * in a consistent manner.
 */
class Body final : public io::IO, public SuperCFL
{
  public:
	/** @brief Costructor
	 * @param log Logging handler defining where/how results should be logged.
	 * @param id U nique identifier of this body
	 */
	Body(moordyn::Log* log, size_t id);

	/** @brief Destructor
	 */
	~Body();

	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCondRef env;
	/// global Waves object
	WavesRef waves;

	// unique to Body:
	/// Points attached to this body
	std::vector<moordyn::Point*> attachedP;
	/// Rods attached to this body
	std::vector<Rod*> attachedR;

	/// Attachment points of each point
	std::vector<vec> rPointRel;
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
	/// The product of the drag coefficient and the frontal area (m^2)
	vec6 bodyCdA;
	/// The added mass coefficients
	vec6 bodyCa;

	// degrees of freedom (or states)
	/// body 6dof position [x/y/z]
	XYZQuat r7;
	// vec6 r6;
	/// body 6dof velocity[x/y/z]
	vec6 v6;
	/// body quaternion position derivative
	XYZQuat dPos;
	/// body 6dof acceleration[x/y/z]
	vec6 a6;

	/// fairlead position for coupled bodies [x/y/z]
	vec6 r_ves;
	/// fairlead velocity for coupled bodies [x/y/z]
	vec6 rd_ves;
	/// fairlead acceleration for coupled bodies [x/y/z]
	vec6 rdd_ves;

	/// total force and moment vector on body
	vec6 F6net;

	/// total body mass + added mass matrix including all elements
	mat6 M;
	/// starting mass and added mass matrix (6x6) of body without any rod
	/// elements in inertial orientation
	mat6 M0;

	/// orientation matrix of body (rotation matrix that gets it to its current
	/// orientation)
	mat OrMat;

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
		/// Is coupled pinned, i.e. translational dof are controlled by the user
		CPLDPIN = 2,
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
			case CPLDPIN:
				return "CPLDPIN";	
			case FREE:
				return "FREE";
			case FIXED:
				return "FIXED";
		}
		return "UNKNOWN";
	}

	/// Body ID
	size_t bodyId;
	/// Body number
	int number;
	/// Type of body
	types type;

	/** @brief Setup/initialize a rigid body. Called after instantiating a new
	 * Body in MoorDyn2.cpp
	 * @param number Body number
	 * @param type Body type
	 * @param r6 6dof position
	 * @param rCG Center of gravity position
	 * @param M Mass
	 * @param V Volume
	 * @param I Inertia (diagonal matrix components)
	 * @param CdA Drag coefficient
	 * @param Ca Added mass coefficient
	 * @param env_in Global struct that holds environmental settings
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
	           EnvCondRef env_in,
	           shared_ptr<ofstream> outfile);

	/** @brief Attach a point to the body
	 * @param point The point
	 * @param coords The fixation point
	 * @throw moordyn::invalid_value_error If @p point is NULL
	 */
	void addPoint(moordyn::Point* point, vec coords);

	/** @brief Attach a rod to the body
	 * @param rod The rod
	 * @param coords vector indicating start (vals 0-2) and end (valse 3-5) of
	 * rod in the body reference frame
	 * @throw moordyn::invalid_value_error If @p rod is NULL
	 */
	void addRod(Rod* rod, vec6 coords);

	/** @brief Initialize the body that aren't free i.e. don't have states
	 *
	 * Those are the bodies with types moordyn::Body::COUPLED and
	 * moordyn::Body::FIXED
	 * @param r The position (6 dof)
	 * @param rd The velocity (6 dof)
	 * @param rdd The acceleration (6 dof)
	 * @throw moordyn::invalid_value_error If the body is of type
	 * moordyn::Body::FREE
	 */
	void initializeUnfreeBody(vec6 r = vec6::Zero(),
	                          vec6 rd = vec6::Zero(),
	                          vec6 rdd = vec6::Zero());

	/** @brief Get the last setted velocity for an unfree body
	 *
	 * For free bodies the behaviour is undetermined
	 *
	 * @return The velocity (6 dof)
	 */
	inline vec6 getUnfreeVel() const { return rd_ves; }

	/** @brief Initialize the FREE point state
	 * @return The 6-dof position (first) and the 6-dof velocity (second)
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::FREE
	 * @throws moordyn::output_file_error If an outfile has been provided, but
	 * it cannot be written
	 */
	std::pair<XYZQuat, vec6> initialize();

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
	void initializeBody(XYZQuat r = XYZQuat::Zero(), vec6 rd = vec6::Zero());

	/** @brief Set the environmental data
	 * @param waves_in Global Waves object
	 */
	inline void setWaves(moordyn::WavesRef waves_in) { waves = waves_in; }

	/** @brief set the states (positions and velocities) to all the attached
	 * entities
	 * @throws moordyn::invalid_value_error If a non FIXED point is tried
	 * to be edited
	 */
	void setDependentStates();

	/** @brief Get the body kinematics
	 * @param pos The output position
	 * @param vel The output velocity
	 */
	inline void getState(XYZQuat& pos, vec6& vel) const
	{
		pos = r7;
		vel = v6;
	}

	/** @brief Get the body kinematics
	 * @return Position and velocity
	 */
	inline std::pair<XYZQuat, vec6> getState() const
	{
		return std::make_pair(r7, v6);
	}

	/** @brief Get the body position
	 * @return The body position
	 */
	inline const vec getPosition() const { return r7.pos; }

	/** @brief Get the body Euler XYZ angles
	 * @return The body Euler XYZ angles
	 */
	inline const vec getAngles() const { return Quat2Euler(r7.quat); }
	// inline vec getAngles() const { return r6(Eigen::seqN(3, 3)); }

	/** @brief Get the body velocity
	 * @return The body velocity
	 */
	inline const vec getVelocity() const { return v6(Eigen::seqN(0, 3)); }

	/** @brief Get the body angular velocity
	 * @return The body angular velocity
	 */
	inline const vec getAngularVelocity() const { return v6(Eigen::seqN(3, 3)); }

	/** @brief Get the forces and moments exerted over the body
	 * @return The net force
	 */
	const vec6 getFnet() const;

	/** @brief Get the mass and intertia matrix
	 * @return The mass and inertia matrix
	 */
	inline const mat6& getM() const { return M; };

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

	/** @brief Called at the beginning of each coupling step to update the
	 * boundary conditions (body kinematics) for the proceeding time steps
	 * @param r The input position
	 * @param rd The input velocity
	 * @param rdd The input acceleration
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::COUPLED or moordyn::Body::FIXED
	 */
	void initiateStep(vec6 r, vec6 rd, vec6 rdd);

	/** @brief Sets the kinematics based on the position and velocity of the
	 * fairlead.
	 *
	 * This function is meant only for coupled or fixed bodies
	 * @param time Local time within the time step (from 0 to dt)
	 * @throw moordyn::invalid_value_error If the body is not of type
	 * moordyn::Body::COUPLED or moordyn::Body::FIXED
	 */
	void updateFairlead(real time);

	/** @brief Set the states to the body to the position r and velocity rd
	 * @param r The position
	 * @param rd The velocity
	 */
	void DECLDIR setState(XYZQuat r, vec6 rd);

	/** @brief calculate the forces and state derivatives of the body
	 *
	 * This function is only meant for free bodies
	 * @return The states derivatives, i.e. the velocity (first) and the
	 * acceleration (second)
	 * @throw moordyn::invalid_value_error If the body is of type
	 * moordyn::Body::FREE
	 */
	std::pair<XYZQuat, vec6> getStateDeriv();

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
	std::vector<uint64_t> Serialize(void);

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	uint64_t* Deserialize(const uint64_t* data);

#ifdef USE_VTK
	/** @brief Produce a VTK object
	 * @return The new VTK object
	 */
	vtkSmartPointer<vtkPolyData> getVTK() const;

	/** @brief Use the provided VTK object as the representation
	 *
	 * Afterwards moordyn::Body::getVTK() will apply the appropriate
	 * transformations
	 * @param vtk_obj The VTK object
	 */
	inline void setVTK(vtkSmartPointer<vtkPolyData> vtk_obj)
	{
		vtk_body = vtk_obj;
	}

	/** @brief Save the body on a VTK (.vtp) file
	 * @param filename The output file name
	 * @throws output_file_error If VTK reports
	 * vtkErrorCode::FileNotFoundError, vtkErrorCode::CannotOpenFileError
	 * or vtkErrorCode::NoFileNameError
	 * @throws invalid_value_error If VTK reports
	 * vtkErrorCode::UnrecognizedFileTypeError or vtkErrorCode::FileFormatError
	 * @throws mem_error If VTK reports
	 * vtkErrorCode::OutOfDiskSpaceError
	 * @throws unhandled_error If VTK reports
	 * any other error
	 */
	void saveVTK(const char* filename) const;
#endif

  private:
#ifdef USE_VTK
	/// The 3D object that represents the body
	vtkSmartPointer<vtkPolyData> vtk_body;

	/** @brief Helper function to setup an initial body representation
	 */
	void defaultVTK();
#endif
};

} // ::moordyn
