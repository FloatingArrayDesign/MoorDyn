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
#include "IO.hpp"
#include "Seafloor.hpp"
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
class Rod final : public io::IO, public SuperCFL
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 * @param rodId Unique identifier of this rod
	 */
	Rod(moordyn::Log* log, size_t rodId);

	/** @brief Destructor
	 */
	~Rod();

  private:
	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCondRef env;
	/// global Waves object
	moordyn::WavesRef waves;
	/// Object containing the 3d seafloor info
	moordyn::SeafloorRef seafloor;

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
	/// Original direction vector
	vec q0;
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
	// values from input files
	moordyn::real CdEnd;
	// values from input files
	moordyn::real CaEnd;

	/// Rod 6dof position [x,y,z,u1,u2,u3] (end A coordinates and direction unit
	/// vector)
	// vec6 r6;
	XYZQuat r7;
	/// Rod 6dof velocity[vx,vy,vz,wx,wy,wz] (end A velocity and rotational
	/// velocities about unrotated axes)
	vec6 v6;
	/// Final 6dof rod velocity (for output)
	XYZQuat vel7;
	/// Final 6dof rod acceleration (for output)
	vec6 acc6;

	// kinematics
	/// node positions
	std::vector<vec> r;
	/// node velocities
	std::vector<vec> rd;
	/// unit tangent vector for the whole rod
	vec q;
	/// unstretched rod segment lengths
	std::vector<moordyn::real> l;

	/// added mass matrix (doesn't contain node masses)
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
	/// node buoyancy
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
	std::vector<moordyn::real> VOF; // TODO: This doesn't need to be a vector, can be a double reused for each node
	/// instantaneous axial submerged length [m]
	real h0;

	// time
	/// simulation time
	moordyn::real t;

	/// fairlead position for coupled rods [x/y/z]
	vec6 r_ves;
	/// fairlead velocity for coupled rods [x/y/z]
	vec6 rd_ves;
	/// fairlead acceleration for coupled rods [x/y/z]
	vec6 rdd_ves;

	// file stuff
	/// Pointer to moordyn::MoorDyn::outfileMain
	ofstream* outfile;
	/// A copy of moordyn::MoorDyn::outChans
	string channels;
	/// Flag for printing channels and units in rod outfile
	int openedoutfile;

	/** @brief Finds the depth of the water at some (x, y) point. Either using
	 * env->WtrDpth or the 3D seafloor if available
	 * @param x x coordinate
	 * @param y y coordinate
	 * @return A negative number representing the sea floor depth at the given
	 * location
	 */
	inline real getWaterDepth(real x, real y)
	{
		return seafloor ? seafloor->getDepthAt(x, y) : -env->WtrDpth;
	}

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
		POINT = FREE,
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
	size_t rodId;
	/// Rod number
	int number;
	/// Rod type
	types type;

	/// The roll angle (useful for output)
	real roll;
	/// The pitch angle (useful for output)
	real pitch;

	/** @brief Setup a rod
	 * @param number Rod ID
	 * @param type Rod type
	 * @param props Rod properties
	 * @param endCoords The coordinates of both end points
	 * @param n Number of segments
	 * @param env_in Global struct that holds environmental settings
	 * @param outfile The outfile where information shall be witten
	 * @param channels The channels/fields that shall be printed in the file
	 */
	void setup(int number,
	           types type,
	           RodProps* props,
	           vec6 endCoords,
	           unsigned int n,
	           EnvCondRef env_in,
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
	 * @param waves_in Global Waves object
	 * @param seafloor_in Global Seafloor object
	 */
	inline void setWaves(moordyn::WavesRef waves_in,
	                     moordyn::SeafloorRef seafloor_in)
	{
		waves = waves_in;
		seafloor = seafloor_in;
	}

	/** @brief Opens rod output file
	 */
	inline void openoutput();

	/** @brief Initialize the rod state
	 * @return The position and orientation angles (first) and the linear and
	 * angular velocity (second)
	 * @note moordyn::Rod::r6 and moordyn::Rod::v6 must already be set
	 * @note ground- or body-pinned rods have already had
	 * moordyn::Rod::setKinematics() called
	 * @note In the case of pinned rods, just the rod directions and angular
	 * velocites shall be considered
	 */
	std::pair<XYZQuat, vec6> initialize();

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
		if (isnan(r[i].sum())) {
			stringstream s;
			s << "NaN detected" << endl
			  << "Rod " << number << " node positions:" << endl;
			for (unsigned int j = 0; j <= N; j++)
				s << j << " : " << r[j] << ";" << endl;
			throw moordyn::nan_error(s.str().c_str());
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
		CdEnd = CdEnd * scaler;
	}

	/** @brief Set the line  simulation time
	 * @param time Simulation time
	 */
	inline void setTime(real time) { t = time; }

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
	 * @throws invalid_value_error If the rod is not of type FREE, CPLDPIN or
	 * PINNED
	 */
	void setState(XYZQuat pos, vec6 vel);

	/** @brief Called at the beginning of each coupling step to update the
	 * boundary conditions (rod kinematics) for the proceeding time steps
	 * @param r The input position
	 * @param rd The input velocity
	 * @param rdd The input acceleration
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 * @note If the rod is of type moordyn::Rod::CPLDPIN, then just 3 components
	 * of @p r and @p rd are considered
	 */
	void initiateStep(vec6 r, vec6 rd, vec6 rdd);

	/** @brief Get the last setted velocity for an unfree rod
	 *
	 * For free rods the behaviour is undetermined
	 *
	 * @return The velocity (6 dof)
	 */
	inline vec6 getUnfreeVel() const { return rd_ves; }

	/** @brief Sets the kinematics
	 *
	 * This function is meant only for coupled or fixed bodies
	 * @param time Local time within the time step (from 0 to dt)
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
	std::pair<XYZQuat, vec6> getStateDeriv();

	/** @brief Get the net force on rod (and possibly moment at end A if it's
	 * not pinned)
	 * @return The net force
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 */
	const vec6 getFnet() const;

	/** @brief Get the rod mass matrix
	 * @return The net force
	 * @throw moordyn::invalid_value_error If the rod is not of type
	 * moordyn::Rod::COUPLED or moordyn::Rod::CPLDPIN
	 */
	inline const mat6 getM() const { return M6net; }

	/** @brief Calculate the force and mass contributions of the point on the
	 * parent body
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 * @param rBody The body position
	 * @param vBody The body velocity
	 */
	void getNetForceAndMass(vec6& Fnet_out,
	                        mat6& M_out,
	                        vec rBody,
	                        vec6 vBody);

	/** @brief Calculate the force and mass contributions of the point on the
	 * parent body
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 */
	inline void getNetForceAndMass(vec6& Fnet_out, mat6& M_out)
	{
		getNetForceAndMass(Fnet_out, M_out, r[0], vec6::Zero());
	}

	/** @brief This is the big function that calculates the forces on the rod,
	 * including from attached lines
	 */
	void doRHS();

	void Output(real);

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

	/** @brief Save the rod on a VTK (.vtp) file
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
	/** @brief Calculate the centripetal force on a body
	 * @param r The body position
	 * @param w The body angular velocity
	 * @return Centripetal force on the body
	 */
	inline vec getCentripetalForce(vec r, vec w) const
	{
		if (!N)
			return vec::Zero();

		vec F = vec::Zero();
		for (unsigned int i = 0; i <= N; i++) {
			F -= M[i] * (w.cross(w.cross(this->r[i] - r)));
		}
		return F;
	}


};

} // ::moordyn
