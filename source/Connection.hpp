/*
 * Copyright (c) 2014 Matt Hall <mtjhall@alumni.uvic.ca>
 * 
 * This file is part of MoorDyn.  MoorDyn is free software: you can redistribute 
 * it and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 * 
 * MoorDyn is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MoorDyn.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "Misc.h"
#include "Log.hpp"

using namespace std;

class Line;
class Waves;

namespace moordyn
{

/** @class Connection Connection.hpp
 * @brief A connection for a line endpoint
 *
 * Each line must have 2 connections at each endpoint, which are used to define
 * how those points are moving. There are 3 basic types of connections:
 *
 *  - Fixed: The point is indeed fixed, either to a unmovable point (i.e. an
 *           anchor) or to another moving body
 *  - Free: The point freely moves, like the line endpoint is not attached to
 *          anything. This will be the connection generated when a line is
 *          failing
 *  - Coupled: The connection position and velocity is externally imposed
 */
class Connection : public LogUser
{
public:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	Connection(moordyn::Log *log);

	/** @brief Destructor
	 */
	~Connection();

private:
	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCond *env;
	/// global Waves object
	Waves *waves;
	
	// unique to Connection:
	/// Lines attached to this connection node
	Line* Attached[10];
	/// Which end of line are we attached to?
	/// 1 = top/fairlead(end B)
	/// 0 = bottom/anchor(end A)
	int Top[10];
	/// Number of attached lines
	int nAttached;

	/** @defgroup conn_constants Constants set at startup from input file
	*  @{
	*/

	double conX;
	double conY;
	double conZ;
	double conM;
	double conV;
	double conFX;
	double conFY;
	double conFZ;
	double conCdA;
	double conCa;

	/**
	* @}
	*/

	/** @defgroup conn_common_line Common properties with line internal nodes
	*  @{
	*/

	/// node position [x/y/z]
	double r[3];
	/// node velocity[x/y/z]
	double rd[3];

	/**
	* @}
	*/

	/// simulation time
	double t;
	/// simulation time current integration was started at (used for BC function)
	double t0;
	/// fairlead position for vessel node types [x/y/z]
	double r_ves[3];
	/// fairlead velocity for vessel node types [x/y/z]
	double rd_ves[3];

	/// total force on node
	double Fnet[3];

	/// node mass + added mass matrices
	double M[3][3];

	/** @defgroup conn_wave Wave data
	*  @{
	*/

	/// free surface elevation
	double zeta;
	/// dynamic pressure
	double PDyn;
	/// wave velocities
	double U [3];
	/// wave accelerations
	double Ud[3];

	/**
	* @}
	*/

public:
	/** @brief Types of connections
	 */
	typedef enum {
		/// Is coupled, i.e. is controlled by the user
		COUPLED = -1,
		/// Is free to move, controlled by MoorDyn
		FREE = 0,
		/// Is fixed, either to a location or to another moving entity
		FIXED = 1,
		// Some aliases
		VESSEL = COUPLED,
		FAIRLEAD = COUPLED,
		CONNECT = FREE,
		ANCHOR = FIXED,
	} types;

	/** @brief Return a string with the name of a type
	 *
	 * This tool is useful mainly for debugging
	 */
	static string TypeName(types t)
	{
		switch(t)
		{
		case COUPLED:
			return "COUPLED";
		case FREE:
			return "FREE";
		case FIXED:
			return "FIXED";
		}
		return "UNKNOWN";
	}

	/// Connection ID
	int number;
	/// Connection type
	types type;

	/** @brief flag indicating whether wave/current kinematics will be considered for
	 * this linec
	 *
	 * - 0: none, or use value set externally for each node of the object
	 * - 1: interpolate from stored
	 * - 2: call interpolation function from global Waves grid
	 */
	int WaterKin;

	/** @brief Setup the connection
	 *
	 * Always call this function after the construtor
	 * @param number_in The connection identifier. The identifiers starts at 1,
	 * not at 0.
	 * @param type_in One of COUPLED, FREE or FIXED
	 * @param r0_in The initial position
	 * @param M_in The mass
	 * @param V_in The volume
	 * @param F_in The initial force on the node
	 * @param CdA_in Product of drag coefficient and projected area
	 * @param Ca_in Added mass coefficient used along with V to calculate added
	 * mass on node
	 */
	void setup(int number_in, types type_in, const double r0_in[3], double M_in,
	           double V_in, const double F_in[3], double CdA_in, double Ca_in);

	/** @brief Attach a line endpoint to this connection
	 * @param theLine The line to be attached
	 * @param TopOfLine 1 for attachments at the last node of the line (top).
	 * 0 for attachments at the first node of the line (bottom)
	 */
	void addLineToConnect(Line *theLine, int TopOfLine);

	/** @brief Dettach a line endpoint from this connection
	 * @param lineID The line identifier
	 * @param TopOfLine Output flag to indicate whether the line was connected
	 * at the top end or at the bottom end. 1 for attachments at the last node
	 * of the line (top). 0 for attachments at the first node of the line
	 * (bottom)
	 * @param rEnd Output position of the node
	 * @param rdEnd Output velocity of the node
	 * @throws moordyn::invalid_value_error If there is no an attached line
	 * with the provided @p lineID
	 */
	void removeLineFromConnect(int lineID, int *TopOfLine,
	                           double rEnd[], double rdEnd[]);

	/** @brief Initialize the FREE connection state
	 * @param X The output state variables, i.e. the velocity [x,y,z] and
	 * position [x,y,z]
	 * @throws moordyn::invalid_value_error If it is not a FREE connection
	 */
	void initializeConnect(double X[6]);

	/** @brief Get the connection state
	 * @param r_out The output position [x,y,z]
	 * @param rd_out The output velocity [x,y,z]
	 */
	void getConnectState(vector<double> &r_out, vector<double> &rd_out);

	/** @brief Get the force on the connection
	 * @param Fnet_out The output force [x,y,z]
	 */
	void getFnet(double Fnet_out[3]);

	/** @brief Get the mass matrix
	 * @param M_out The output mass matrix
	 */
	void getM(double M_out[3][3]);

	/** @brief Get the output
	 * @param outChan The query
	 * @return The data, 0.0 if no such data can be found
	 */
	double GetConnectionOutput(OutChanProps outChan);

	/** @brief Set the environmental data
	 * @param env_in Global struct that holds environmental settings
	 * @param waves_in Global Waves object
	 */
	void setEnv(EnvCond *env_in, Waves *waves_in);

	/** @brief Multiply the drag by a factor
	 * @param scaler Drag factor
	 */
	void scaleDrag(double scaler);

	/** @brief Set the time stamp
	 * @param time Simulation time
	 */
	void setTime(double time);

	/** @brief Initialize the time step integration
	 *
	 * Called at the beginning of each coupling step to update the boundary
	 * conditions (fairlead kinematics) for the proceeding line time steps
	 * @param rFairIn Fairlead position, used only if type = COUPLED
	 * @param rdFairIn Fairlead velocity, used only if type = COUPLED
	 * @param time Simulation time
	 */
	void initiateStep(const double rFairIn[3],
	                  const double rdFairIn[3],
	                  double time);

	/** @brief Take the kinematics from the fairlead information
	 *
	 * Sets Connection states and ends of attached lines ONLY if this Connection
	 * is driven externally, i.e. type = COUPLED (otherwise shouldn't be called)
	 * @param time Simulation time
	 * @throws moordyn::invalid_value_error If it is not a COUPLED connection
	 */
	void updateFairlead(const double time);

	/** @brief Take the kinematics from the fairlead information
	 *
	 * sets Connection states and ends of attached lines ONLY if this Connection
	 * is attached to a body, i.e. type = FIXED (otherwise shouldn't be called)
	 * @param r_in Position
	 * @param rd_in Velocity
	 * @throws moordyn::invalid_value_error If it is not a FIXED connection
	 */
	void setKinematics(double *r_in, double *rd_in);

	/** @brief Set the state variables
	 *
	 * sets Connection states and ends of attached lines ONLY if this Connection
	 * is free, i.e. type = FREE (otherwise shouldn't be called)
	 * @param X State variables, containing the velocity [x,y,z] and position
	 * [x,y,z]
	 * @param time Simulation time
	 * @return MOORDYN_SUCCESS upon success, MOORDYN_INVALID_VALUE if it is not
	 * a FREE connection
	 */
	moordyn::error_id setState(const double X[6], const double time);

	/** @brief Calculate the forces and state derivatives of the connection
	 * @param Xd Output state variables derivatives, i.e. the acceleration
	 * [x,y,z] and the velocity [x,y,z]
	 * @return MOORDYN_SUCCESS upon success, MOORDYN_INVALID_VALUE if it is not
	 * a FREE connection
	 */
	moordyn::error_id getStateDeriv(double Xd[6]);

	/** @brief Calculate the force and mass contributions of the connect on the
	 * parent body
	 * @param rBody The body position. If NULL, {0, 0, 0} is considered
	 * @param Fnet_out Output Force about body ref point
	 * @param M_out Output Mass matrix about body ref point
	 * @return MOORDYN_SUCCESS upon success, an error code otherwise
	 */
	moordyn::error_id getNetForceAndMass(const double rBody[3],
	                                     double Fnet_out[6],
	                                     double M_out[6][6]);

	/** @brief Calculates the forces and mass on the connection, including from
	 * attached lines
	 *
	 * @return MOORDYN_SUCCESS upon success, an error code otherwise
	 */
	moordyn::error_id doRHS();

#ifdef USEGL
	void drawGL(void);
#endif

};

}  // ::moordyn