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

/** @file Point.hpp
 * C++ API for the moordyn::Point object
 */

#pragma once

#include "Misc.hpp"
#include "IO.hpp"
#include "Seafloor.hpp"
#include "Util/CFL.hpp"
#include <utility>

#ifdef USE_VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#endif

using namespace std;

namespace moordyn {

class Line;
class Waves;
typedef std::shared_ptr<Waves> WavesRef;

/** @class Point Point.hpp
 * @brief A point for a line endpoint
 *
 * Each line must have 2 points at each endpoint, which are used to define
 * how those points are moving. There are 3 basic types of points:
 *
 *  - Fixed: The point is fixed, either to a unmovable point (i.e. an anchor
 *           or to a Body
 *  - Free: The point freely moves, with its own translational degrees of
 * freedom, to provide a point point between multiple mooring lines or an
 *          unconnected termination point of a Line, which could have a clump
 *          weight or float via the point's mass and volume parameters
 *  - Coupled: The point position and velocity is externally imposed
 */
class Point final : public io::IO, public SuperCFL
{
  public:
	/** @brief Constructor
	 * @param log Logging handler
	 * @param id Unique identifier of this instance
	 */
	Point(moordyn::Log* log, size_t id);

	/** @brief Destructor
	 */
	~Point();

	/// Attached lines to the point
	typedef struct _attachment
	{
		/// The attached line
		Line* line;
		/// The attachment end point
		EndPoints end_point;
	} attachment;

  private:
	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCondRef env;
	/// global Waves object
	moordyn::WavesRef waves;
	moordyn::SeafloorRef seafloor;

	/// Lines attached to this point node
	std::vector<attachment> attached;

	/** @defgroup point_constants Constants set at startup from input file
	 * @{
	 */

	/// Mass [kg]
	real pointM;
	/// Volume [m3]
	real pointV;
	/// Force [N]
	vec pointF;
	/// Drag coefficient
	real pointCdA;
	/// Added mass coefficient
	real pointCa;

	/**
	 * @}
	 */

	/** @defgroup point_common_line Common properties with line internal nodes
	 * @{
	 */

	/// node position [x/y/z]
	vec r;
	/// node velocity[x/y/z]
	vec rd;

	/**
	 * @}
	 */

	/// fairlead position for vessel/coupled node types [x/y/z]
	vec r_ves;
	/// fairlead velocity for vessel/coupled node types [x/y/z]
	vec rd_ves;

	/// total force on node
	vec Fnet;

	// time
	/// simulation time
	moordyn::real t;

	/// node mass + added mass matrices
	mat M;

	/// node acceleration
	vec acc;

  public:
	/** @brief Types of points
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
		FAIRLEAD = COUPLED,
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
			case FREE:
				return "FREE";
			case FIXED:
				return "FIXED";
		}
		return "UNKNOWN";
	}

	/// Point ID
	size_t pointId;

	/// Point number
	int number;
	/// Point type
	types type;

	/** @brief Set the line  simulation time
	 * @param time Simulation time
	 */
	inline void setTime(real time) { t = time; }

	/** @brief Setup the point
	 *
	 * Always call this function after the constructor
	 * @param number_in The point identifier. The identifiers starts at 1,
	 * not at 0.
	 * @param type_in One of COUPLED, FREE or FIXED
	 * @param r0_in The initial position
	 * @param M_in The mass
	 * @param V_in The volume
	 * @param F_in The initial force on the node
	 * @param CdA_in Product of drag coefficient and projected area
	 * @param Ca_in Added mass coefficient used along with V to calculate added
	 * mass on node
	 * @param env_in Global struct that holds environmental settings
	 */
	void setup(int number_in,
	           types type_in,
	           vec r0_in,
	           double M_in,
	           double V_in,
	           vec F_in,
	           double CdA_in,
	           double Ca_in,
	           EnvCondRef env_in);

	/** @brief Attach a line endpoint to this point
	 * @param theLine The line to be attached
	 * @param end_point The line endpoint
	 */
	void addLine(moordyn::Line* theLine, EndPoints end_point);

	/** @brief Detach a line
	 * @param line The line
	 * @return The line end point that was attached to the point
	 * @throws moordyn::invalid_value_error If there is no an attached line
	 * with the provided @p lineID
	 */
	EndPoints removeLine(Line* line);

	/** @brief Get the list of attachments
	 * @return The list of attachments
	 */
	inline std::vector<attachment> getLines() const { return attached; }

	/** @brief Initialize the FREE point state
	 * @return The position (first) and the velocity (second)
	 * @throws moordyn::invalid_value_error If it is not a FREE point
	 */
	std::pair<vec, vec> initialize();

	/** @brief Get the point position
	 * @return The position [x,y,z]
	 */
	inline const vec& getPosition() const { return r; }

	/** @brief Get the point velocity
	 * @return The velocity [x,y,z]
	 */
	inline const vec& getVelocity() const { return rd; }

	/** @brief Get the point state
	 * @param r_out The output position [x,y,z]
	 * @param rd_out The output velocity [x,y,z]
	 */
	inline void getState(vec& r_out, vec& rd_out)
	{
		r_out = r;
		rd_out = rd;
	};

	/** @brief Get the point state
	 * @return The position and velocity
	 */
	inline std::pair<vec, vec> getState() { return std::make_pair(r, rd); }

	/** @brief Get the force on the point
	 * @param Fnet_out The output force [x,y,z]
	 */
	inline void getFnet(vec& Fnet_out) const { Fnet_out = Fnet; }

	/** @brief Get the force on the point
	 * @return The output force [x,y,z]
	 */
	inline const vec& getFnet() const { return Fnet; }

	/** @brief Get the mass matrix
	 * @param M_out The output mass matrix
	 */
	inline void getM(mat& M_out) const { M_out = M; }

	/** @brief Get the mass matrix
	 * @return The mass matrix
	 */
	inline const mat& getM() const { return M; };

	/** @brief Get the output
	 * @param outChan The query
	 * @return The data, 0.0 if no such data can be found
	 */
	real GetPointOutput(OutChanProps outChan);

	/** @brief Set the environmental data
	 * @param waves_in Global Waves object
	 * @param seafloor_in Global 3D Seafloor object
	 */
	inline void setWaves(moordyn::WavesRef waves_in,
	                     moordyn::SeafloorRef seafloor_in)
	{
		waves = waves_in; // set pointer to Waves  object
		seafloor = seafloor_in;
	}

	/** @brief Multiply the drag by a factor
	 *
	 * function for boosting drag coefficients during IC generation
	 * @param scaler Drag factor
	 */
	inline void scaleDrag(real scaler) { pointCdA *= scaler; }

	/** @brief Initialize the time step integration
	 *
	 * Called at the beginning of each coupling step to update the boundary
	 * conditions (fairlead kinematics) for the proceeding line time steps
	 * @param rFairIn Fairlead position
	 * @param rdFairIn Fairlead velocity
	 * @throws moordyn::invalid_value_error If it is not a COUPLED point
	 */
	void initiateStep(vec rFairIn, vec rdFairIn);

	/** @brief Take the kinematics from the fairlead information
	 *
	 * Sets Point states and ends of attached lines ONLY if this Point
	 * is driven externally, i.e. type = COUPLED (otherwise shouldn't be called)
	 * @param time Local time within the time step (from 0 to dt)
	 * @throws moordyn::invalid_value_error If it is not a COUPLED point
	 */
	void updateFairlead(real time);

	/** @brief Take the kinematics from the fairlead information
	 *
	 * sets Point states and ends of attached lines ONLY if this Point
	 * is attached to a body, i.e. type = FIXED (otherwise shouldn't be called)
	 * @param r_in Position
	 * @param rd_in Velocity
	 * @throws moordyn::invalid_value_error If it is not a FIXED point
	 */
	void setKinematics(vec r_in, vec rd_in);

	/** @brief Set the state variables
	 *
	 * sets Point states and ends of attached lines ONLY if this Point
	 * is free, i.e. type = FREE (otherwise shouldn't be called)
	 * @param pos Position
	 * @param vel Velocity
	 * @throws moordyn::invalid_value_error If it is not a FREE point
	 */
	void setState(vec pos, vec vel);

	/** @brief Calculate the forces and state derivatives of the point
	 * @return The states derivatives, i.e. the velocity (first) and the
	 * acceleration (second)
	 * @throws moordyn::invalid_value_error If it is not a FREE point
	 */
	std::pair<vec, vec> getStateDeriv();

	/** @brief Calculate the force and mass contributions of the point on the
	 * parent body
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 * @param rBody The body position
	 * @param vBody The body velocity
	 */
	void getNetForceAndMass(vec6& Fnet_out,
	                        mat6& M_out,
	                        vec rBody = vec::Zero(),
	                        vec6 vBody = vec6::Zero());

	/** @brief Calculates the forces and mass on the point, including from
	 * attached lines
	 *
	 * @return MOORDYN_SUCCESS upon success, an error code otherwise
	 */
	moordyn::error_id doRHS();

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

	/** @brief Save the point on a VTK (.vtp) file
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
		return -M * (w.cross(w.cross(this->r - r)));
	}

};

} // ::moordyn
