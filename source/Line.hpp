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

/** @file Line.hpp
 * C++ API for the moordyn::Line object
 */

#pragma once

#include "Misc.h"
#include "Log.hpp"
#include <utility>

using namespace std;

namespace moordyn {

class Waves;

/** @class Line Line.hpp
 * @brief A mooring line
 *
 * Each mooring line is divided on a set of nodes connected by segments, with
 * two connections at the end points
 *
 * [connect (node 0)] - seg 0 - [node 1] - ... - seg n-1 - [connect (node N)]
 *
 * Depending on the length of the line, \f$ l \f$, the number of segments,
 * \f$ n \f$, an the material density and Young's modulus, \f$ \rho \f$ &
 * \f$ E \f$, a natural scillation period can be defined:
 *
 * \f$ T = \frac{l}{\pi n} \sqrt{\frac{\rho}{E}} \f$
 *
 * Thus, the integration time step (moordyn::MoorDyn.dtM0) shall be smaller than
 * such natural period to avoid numerical instabilities
 */
class Line : public LogUser
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	Line(moordyn::Log* log);

	/** @brief Destructor
	 */
	~Line();

  private:
	/** @brief Get the non-linear Young's modulus
	 * @param l_stretched The actual length of the segment
	 * @param l_unstretched The unstretched length of the segment
	 */
	real getNonlinearE(real l_stretched, real l_unstretched);

	/** @brief Get the non-linear Damping coefficient
	 * @param ld_stretched The segment rate of stretch
	 * @param l_unstretched The unstretched length of the segment
	 */
	real getNonlinearC(real ld_stretched, real l_unstretched);

	/** @brief Get the non-linear bending stiffness
	 * @param curv The curvature
	 */
	real getNonlinearEI(real curv);

	// ENVIRONMENTAL STUFF
	/// Global struct that holds environmental settings
	EnvCond* env;
	/// global Waves object
	moordyn::Waves* waves;

	/// Number of line segments
	unsigned int N;
	/// Unstretched line length
	moordyn::real UnstrLen;
	/// line diameter
	moordyn::real d;
	/// line linear density
	moordyn::real rho;
	/// line elasticity modulus (Young's modulus) [Pa]
	moordyn::real E;
	/// line bending stiffness [Nm^2]
	moordyn::real EI;
	/** line axial internal damping coefficient [Ns]
	 *
	 * The provided value in moordyn::Line::props::c can be either the literal
	 * damping coefficient, or a negative value in which case it represents the
	 * fraction of the critical damping,
	 *
	 * \f$ C_{crit} = \frac{l}{n} \sqrt{\rho E} \f$
	 */
	moordyn::real c;
	/// normal added mass coefficient [-]
	/// with respect to line displacement
	moordyn::real Can;
	/// axial added mass coefficient [-]
	/// with respect to line displacement
	moordyn::real Cat;
	/// normal drag coefficient [-]
	/// with respect to frontal area, \f$ d l \f$
	moordyn::real Cdn;
	/// axial drag coefficient [-]
	/// with respect to surface area, \f$ \pi d l \f$
	moordyn::real Cdt;

	/// line axial internal damping coefficient (before proceessing) [Ns]
	moordyn::real BAin;
	/// line cross-sectional area to pre-compute [m2]
	moordyn::real A;
	/// number of values in stress-strain lookup table (0 for constant E)
	unsigned int nEApoints;
	/// x array for stress-strain lookup table
	std::vector<moordyn::real> stiffXs;
	/// y array for stress-strain lookup table
	std::vector<moordyn::real> stiffYs;
	/// number of values in bent stiffness lookup table (0 for constant EI)
	unsigned int nEIpoints;
	/// x array for bent stiffness lookup table
	std::vector<moordyn::real> bstiffXs;
	/// y array for bent stiffness lookup table
	std::vector<moordyn::real> bstiffYs;
	/// number of values in stress-strainrate lookup table (0 for constant c)
	unsigned int nCpoints;
	/// x array for stress-strainrate lookup table
	std::vector<moordyn::real> dampXs;
	/// y array for stress-strainrate lookup table
	std::vector<moordyn::real> dampYs;

	// kinematics
	/// node positions
	std::vector<vec> r;
	/// node velocities
	std::vector<vec> rd;
	/// unit tangent vectors for each node
	std::vector<vec> q;
	/// unit tangent vectors for each segment (used in bending calcs)
	std::vector<vec> qs;
	/// unstretched line segment lengths
	std::vector<moordyn::real> l;
	/// stretched segment lengths
	std::vector<moordyn::real> lstr;
	/// rate of stretch
	std::vector<moordyn::real> ldstr;
	/// curvatures at node points (1/m)
	std::vector<moordyn::real> Kurv;

	/// node mass + added mass matrix
	std::vector<mat> M;
	// line segment volumes
	std::vector<moordyn::real> V;

	// forces
	/// segment tensions
	std::vector<vec> T;
	/// segment damping forces
	std::vector<vec> Td;
	/// bending stiffness forces
	std::vector<vec> Bs;
	/// node weights
	std::vector<vec> W;
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

	// time
	/// simulation time
	moordyn::real t;

	// end conditions
	/** @brief Types of end points
	 */
	typedef enum
	{
		/// Pinned to Connection
		PINNED = 0,
		/// Cantilevered to Rod
		CANTILEVERED = 1,
		// Some aliases
		CONNECTION = PINNED,
		ROD = CANTILEVERED,
	} endTypes;

	/// type of connection at end A: 0=pinned to Connection, 1=cantilevered to
	/// Rod.
	endTypes endTypeA;
	/// type of connection at end B: 0=pinned to Connection, 1=cantilevered to
	/// Rod.
	endTypes endTypeB;
	/// moment at end A from bending, to be applied on attached Rod/Body
	vec endMomentA;
	/// moment at end A from bending, to be applied on attached Rod/Body
	vec endMomentB;

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
	/// Line ID
	int number;

	/** flag indicating whether wave/current kinematics will be considered
	 *
	 * Thisis not exactly a copy of EnvCond::WaveKin, but a compiled value
	 * dependig on both EnvCond::WaveKin and EnvCond::Current
	 */
	moordyn::waves_settings WaterKin;

	/** @brief Setup a line
	 * @param number Line ID
	 * @param props Line properties
	 * @param l Unstretched line length
	 * @param n Number of segments
	 * @param outfile The outfile where information shall be witten
	 * @param channels The channels/fields that shall be printed in the file
	 */
	void setup(int number,
	           LineProps* props,
	           real l,
	           unsigned int n,
	           shared_ptr<ofstream> outfile,
	           string channels);

	/** @brief Set the environmental data
	 * @param env_in Global struct that holds environmental settings
	 * @param waves_in Global Waves object
	 */
	void setEnv(EnvCond* env_in, moordyn::Waves* waves_in);

	/** @brief Compute the stationary Initial Condition (IC)
	 * @param X The output states vector, with 3*(n-1) velocity components and
	 * 3*(n-1) position components. It cannot be NULL
	 * @throws moordyn::output_file_error If an outfile has been provided, but
	 * it cannot be written
	 * @throws invalid_value_error If there is no enough water depth
	 */
	void initializeLine(double* X);

	/** @brief Number of segments
	 *
	 * The number of nodes can be computed as moordyn::Line::getN() + 1
	 * @return The number of segments, moordyn::Line::N
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
			LOGERR << "Asking node " << i << " of line " << number
			       << ", which only has " << N + 1 << " nodes" << std::endl;
			throw moordyn::invalid_value_error("Invalid node index");
		}
		return r[i];
	}

	/** @brief Get the tension in a node
	 *
	 * smart (selective) function to get tension at any node including fairlead
	 * or anchor (accounting for weight in these latter cases) (added Nov 15th)
	 * @param i The line node index
	 * @return The tension
	 * @throws invalid_value_error If the node index \p i is bigger than the
	 * number of nodes, moordyn::Line::N + 1
	 */
	inline vec getNodeTen(unsigned int i) const
	{
		if (i > N) {
			LOGERR << "Asking node " << i << " of line " << number
			       << ", which only has " << N + 1 << " nodes" << std::endl;
			throw moordyn::invalid_value_error("Invalid node index");
		}
		if ((i == 0) || (i == N))
			return (
			    Fnet[i] +
			    vec(0.0, 0.0, M[i](0, 0) * (-env->g))); // <<< update to use W

		// take average of tension in adjacent segments
		return (0.5 * (T[i] + T[i - 1]));
	};

	/** @brief Get the line curvature at a node position
	 * @param i The line node index
	 * @return The line curvature (1 / m)
	 * @throws invalid_value_error If the node index \p i is bigger than the
	 * number of nodes, moordyn::Line::N + 1
	 * @note The curvature is only computed if bending stiffness
	 * (moordyn::Line::EI) is not zero. Otherwise the curvature of every single
	 * node will be zero.
	 */
	inline real getNodeCurv(unsigned int i) const
	{
		if (i > N) {
			LOGERR << "Asking node " << i << " of line " << number
			       << ", which only has " << N + 1 << " nodes" << std::endl;
			throw moordyn::invalid_value_error("Invalid node index");
		}
		return Kurv[i];
	}

	/** @brief Get the array of coordinates of all nodes along the line
	 * @return The positions array
	 */
	inline std::vector<vec> getNodeCoordinates() const { return r; }

	/** @brief Set the water flow velocity and acceleration at the line nodes
	 *
	 * used just when these are provided externally, i.e. WaveKin=1
	 * @param U_in Velocities, should have (moordyn::Line::N + 1) * 3
	 *             components
	 * @param Ud_in Accelerations, should have (moordyn::Line::N + 1) * 3
	 *              components
	 */
	inline void setNodeWaveKin(double U_in[], double Ud_in[])
	{
		for (unsigned int i = 0; i <= N; i++) {
			moordyn::array2vec(&(U_in[3 * i]), U[i]);
			moordyn::array2vec(&(Ud_in[3 * i]), Ud[i]);
		}
	}

	/** @brief Get the tensions at the fairlead and anchor in a FASTv7 friendly
	 * way
	 * @param FairHTen Horizontal tension on the fairlead
	 * @param FairVTen Vertical tension on the fairlead
	 * @param AnchHTen Horizontal tension on the anchor
	 * @param AnchVTen Vertical tension on the anchor
	 */
	inline void getFASTtens(float* FairHTen,
	                        float* FairVTen,
	                        float* AnchHTen,
	                        float* AnchVTen) const
	{
		*FairHTen = (float)(Fnet[N](Eigen::seqN(0, 2)).norm());
		*FairVTen = (float)(Fnet[N][2] + M[N](0, 0) * (-env->g));
		*AnchHTen = (float)(Fnet[0](Eigen::seqN(0, 2)).norm());
		*AnchVTen = (float)(Fnet[0][2] + M[0](0, 0) * (-env->g));
	}

	/** @brief Get the force, moment and mass at the line endpoint
	 * @param Fnet_out The output force vector
	 * @param Moment_out The output moment vector
	 * @param M_out The output mass matrix
	 * @param end_point Either ENDPOINT_TOP or ENDPOINT_BOTTOM
	 * @throws invalid_value_error If @p end_point is not a valid end point
	 * qualifier
	 */
	inline void getEndStuff(vec& Fnet_out,
	                        vec& Moment_out,
	                        mat& M_out,
	                        EndPoints end_point) const
	{
		switch (end_point) {
			case ENDPOINT_TOP:
				Fnet_out = Fnet[N];
				Moment_out = endMomentB;
				M_out = M[N];
				break;
			case ENDPOINT_BOTTOM:
				Fnet_out = Fnet[0];
				Moment_out = endMomentA;
				M_out = M[0];
				break;
			default:
				LOGERR << "Invalid end point qualifier: " << end_point << endl;
				throw moordyn::invalid_value_error("Invalid end point");
		}
	}

	/** @brief Get line output
	 *
	 * This funtion is useful when outputs are set in the line properties
	 * @param outChan The output channel/field
	 * @return The output value, 0.0 if a non-valid field is set
	 */
	real GetLineOutput(OutChanProps outChan);

	/** @brief store wave/current kinematics time series for this line
	 *
	 * This is used when nodal approaches are selected, i.e. WaveKin = 4 or 5,
	 * Currents = 3 or 4
	 *
	 * @param nt Number of time steps
	 * @param dt Time step
	 * @param zeta_in The wave elevations
	 * @param f_in The fluid fractions
	 * @param u_in The flow velocity
	 * @param ud_in The flow acceleration
	 *
	 * @note Working in progress
	 */
	void storeWaterKin(unsigned int nt,
	                   real dt,
	                   const real** zeta_in,
	                   const real** f_in,
	                   const real*** u_in,
	                   const real*** ud_in);

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

	/** @brief Set the line state
	 * @param X State vector. It contains moordyn::Line::N - 1 velocities
	 * followed by moordyn::Line::N - 1 positions
	 * @param time Simulation time
	 * @note This method is not affecting the line end points
	 * @see moordyn::Line::setEndState
	 */
	void setState(const double* X, const double time);

	/** @brief Set the position and velocity of an end point
	 * @param r_in Position
	 * @param rd_in Velocity
	 * @param topOfLine 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	 * @deprecated This method is replaced by SetEndKinematics and will be
	 * removed in the future
	 */
	void DEPRECATED setEndState(double r_in[3], double rd_in[3], int topOfLine);

	/** @brief Set the position and velocity of an end point
	 * @param r Position
	 * @param rd Velocity
	 * @param end_point Either ENDPOINT_TOP or ENDPOINT_BOTTOM
	 * @throws invalid_value_error If @p end_point is not a valid end point
	 * qualifier
	 * @note This method replaces the deprecated setEndState
	 */
	void setEndKinematics(vec r, vec rd, EndPoints end_point);

	/** @brief set end node unit vector
	 *
	 * This method is called by an eventually attached Rod, only applicable for
	 * bending stiffness
	 * @param qin The direction unit vector
	 * @param topOfLine 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	 * @param rodEndB 1 = end B, 0 = end A
	 * @deprecated This method is replaced to use moordyn::vec instead, and will
	 * be removed in the future
	 */
	void DEPRECATED setEndOrientation(double* qin, int topOfLine, int rodEndB);

	/** @brief set end node unit vector
	 *
	 * This method is called by an eventually attached Rod, only applicable for
	 * bending stiffness
	 * @param q The direction unit vector
	 * @param end_point Either ENDPOINT_B or ENDPOINT_A
	 * @param rod_end_point Either ENDPOINT_B or ENDPOINT_A
	 * @throws invalid_value_error If either @p end_point or @p end_point are
	 * not valid end point qualifiers
	 */
	void setEndOrientation(vec q, EndPoints end_point, EndPoints rod_end_point);

	/** @brief Get the bending moment at the end point
	 *
	 * The bending moment is defined as
	 * \f$\bar{r} \frac{E I}{\vert \bar{r} \vert^2}\f$, with \f$\bar{r}\f$ the
	 * vector joining the endpoint and the next line node
	 *
	 * This method is already taking into account the line and rod end points
	 * to appropriately set the sign of the resulting moment
	 * @param q_EI_dl The output moment vector
	 * @param topOfLine 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	 * @param rodEndB 1 = end B, 0 = end A
	 * @deprecated This method is replaced by getEndSegmentMoment and will
	 * be removed in the future
	 */
	void DEPRECATED getEndSegmentInfo(double q_EI_dl[3],
	                                  int topOfLine,
	                                  int rodEndB);

	/** @brief Get the bending moment at the end point
	 *
	 * The bending moment is defined as
	 * \f$\bar{r} \frac{E I}{\vert \bar{r} \vert^2}\f$, with \f$\bar{r}\f$ the
	 * vector joining the endpoint and the next line node
	 *
	 * This method is already taking into account the line and rod end points
	 * to appropriately set the sign of the resulting moment
	 * @param end_point Either ENDPOINT_B or ENDPOINT_A
	 * @param rod_end_point Either ENDPOINT_B or ENDPOINT_A
	 * @return The moment vector
	 * @throws invalid_value_error If either @p end_point or @p end_point are
	 * not valid end point qualifiers
	 */
	vec getEndSegmentMoment(EndPoints end_point, EndPoints rod_end_point) const;

	/** @brief Calculate forces and get the derivative of the line's states
	 * @param Xd The output line states
	 * @param dt The time step, unused
	 * @throws nan_error If nan values are detected in any node position
	 */
	void getStateDeriv(double* Xd, const double dt);

	// void initiateStep(vector<double> &rFairIn, vector<double> &rdFairIn,
	// double time);

	void Output(double);

#ifdef USEGL
	void drawGL(void);
	void drawGL2(void);
#endif
};

} // ::moordyn
