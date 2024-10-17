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

/** @file Line.h
 * C API for the moordyn::Line object
 */

#ifndef MOORDYN_LINE_H
#define MOORDYN_LINE_H

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup new_c_api
	 *  @{
	 */

	/// A mooring line instance
	typedef struct __MoorDynLine* MoorDynLine;

	/** @brief Get the line identifier
	 * @param l The Moordyn line
	 * @param id The output id
	 * @return MOORDYN_INVALID_VALUE if a NULL point is provided,
	 * MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineID(MoorDynLine l, int* id);

	/** @brief Get the line number of segments
	 *
	 * The number of nodes is equal to this value plus 1
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineN(MoorDynLine l, unsigned int* n);

	/** @brief Get the line number of nodes
	 * @param l The Moordyn line
	 * @param n The output number of nodes
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineN()
	 */
	int DECLDIR MoorDyn_GetLineNumberNodes(MoorDynLine l, unsigned int* n);

	/** @brief Get the line unstretched length
	 * @param l The Moordyn line
	 * @param ul The output length
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_SetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_GetLineUnstretchedLength(MoorDynLine l, double* ul);

	/** @brief Set the line unstretched length
	 * @param l The Moordyn line
	 * @param ul The new length
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_SetLineUnstretchedLength(MoorDynLine l, double ul);

	/** @brief Set the line unstretched length rate of change
	 * @param l The Moordyn line
	 * @param v The rate of change
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineUnstretchedLength()
	 * @see MoorDyn_SetLineUnstretchedLength()
	 */
	int DECLDIR MoorDyn_SetLineUnstretchedLengthVel(MoorDynLine l, double v);

	/** @brief Get whether the line is governed by a non-linear stiffness or a
	 * constant one
	 * @param l The Moordyn line
	 * @param b 1 if the stiffness of the line is constant, 0 if a
	 * non-linear stiffness has been set
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_GetLineConstantEA()
	 * @see MoorDyn_SetLineConstantEA()
	 */
	int DECLDIR MoorDyn_IsLineConstantEA(MoorDynLine l, int* b);

	/** @brief Get the constant stiffness of the line
	 *
	 * This value is useless if non-linear stiffness is considered
	 * @param l The Moordyn line
	 * @param EA The constant stiffness EA value
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_IsLineConstantEA()
	 * @see MoorDyn_SetLineConstantEA()
	 */
	int DECLDIR MoorDyn_GetLineConstantEA(MoorDynLine l, double* EA);

	/** @brief Set the constant stiffness of the line
	 *
	 * This value is useless if non-linear stiffness is considered
	 * @param l The Moordyn line
	 * @param EA The constant stiffness EA value
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_IsLineConstantEA()
	 * @see MoorDyn_GetLineConstantEA()
	 */
	int DECLDIR MoorDyn_SetLineConstantEA(MoorDynLine l, double EA);

	/** @brief Get whether the line pressure bending is considered or not
	 * @param l The Moordyn line
	 * @param b 1 if the pressure bending of the line is enabled, 0 otherwise
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_SetLinePressBend()
	 * @see MoorDyn_SetLinePressInt()
	 */
	int DECLDIR MoorDyn_IsLinePressBend(MoorDynLine l, int* b);

	/** @brief Set whether the line pressure bending is considered or not
	 * @param l The Moordyn line
	 * @param b 1 if the pressure bending of the line shall be considered, 0
	 * otherwise
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_IsLinePressBend()
	 * @see MoorDyn_SetLinePressInt()
	 */
	int DECLDIR MoorDyn_SetLinePressBend(MoorDynLine l, int b);

	/** @brief Set the line internal pressure values at the nodes
	 * @param l The Moordyn line
	 * @param p Pressure values, and array with MoorDyn_GetLineN() + 1
	 * values
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 * @see MoorDyn_IsLinePressBend()
	 * @see MoorDyn_SetLinePressBend()
	 */
	int DECLDIR MoorDyn_SetLinePressInt(MoorDynLine l, const double* p);

	/** @brief Get a line node position
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param pos The output node position
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodePos(MoorDynLine l,
	                                   unsigned int i,
	                                   double pos[3]);

	/** @brief Get a line node velocity
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param vel The output node velocity
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeVel(MoorDynLine l,
	                                   unsigned int i,
	                                   double vel[3]);

	/** @brief Get a line node force
	 *
	 * To get the components of the force use MoorDyn_GetLineNodeTen() ,
	 * MoorDyn_GetLineNodeBendStiff(), MoorDyn_GetLineNodeWeight() ,
	 * MoorDyn_GetLineNodeDrag() , MoorDyn_GetLineNodeFroudeKrilov() and
	 * MoorDyn_GetLineNodeSeaBedForce()
	 * @note The net force is \b not the sum of all the components that you
	 * cat extract from the API. For instance, the tension contribution on the
	 * internal nodes is the difference between the tensions of the adjacent
	 * segments, while MoorDyn_GetLineNodeTen() is returning the averaged
	 * value.
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param f The output node net force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeForce(MoorDynLine l,
	                                     unsigned int i,
	                                     double f[3]);


	/** @brief Get a line node tension
	 *
	 * The node tension is the sum of the axial stiffness plus the internal
	 * damping
	 *
	 * \f[
	 *     \bar{T}(i) = E A \left(
	 *         \frac{l(i) - l_0}{l_0}
	 *         - \frac{\beta A}{l_0} \frac{\mathrm{d}l(i)}{\mathrm{d}t}
	 *     \right) \bar{q}(i)
	 * \f]
	 *
	 * with \f$l(i)\f$ and \f$l_0\f$ the stretched and unstretched segment
	 * lengths respectively, \f$E\f$ the Young's modulus, \f$A\f$ the
	 * transversal area, \f$\beta\f$ the internal damping coefficient and
	 * \f$\bar{q}(i)\f$ the direction verctor of the segment.
	 *
	 * As can be appreciated, the node tension is computed at each segment. So
	 * to get the node one the surrounding segments are averaged (in case of
	 * line-ends the associated ending segment tension is returned)
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param t The output node tension
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeTen(MoorDynLine l,
	                                   unsigned int i,
	                                   double t[3]);

	/** @brief Get a line node bending stiffness force
	 *
	 * The bending stiffness is computed at each segment as
	 *
	 * \f[
	 *     \bar{Bs}(i) = E I \frac{\bar{k}(i)}{l_0}
	 * \f]
	 *
	 * with \f$l_0\f$ the unstretched segment length, \f$E\f$ the Young's
	 * modulus, \f$I\f$ the segment normal axis inertia, and
	 * \f$\bar{k}\f$ the curvature vector.
	 *
	 * To get the node bending stiffness forces the surrounding segments forces
	 * are accumulated
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param t The output node force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeBendStiff(MoorDynLine l,
	                                         unsigned int i,
	                                         double t[3]);

	/** @brief Get a line node weight and bouyancy
	 *
	 * This is computed at each segment as
	 *
	 * \f[
	 *     \bar{W}(i) = A l_0 \left(\rho - F(i) \rho_w \right) \bar{g}
	 * \f]
	 *
	 * with \f$l_0\f$ the unstretched segment length, \f$A\f$ the
	 * transversal area, \f$\rho\f$ the line material density, \f$\rho_w\f$
	 * the water density, \f$F(i)\f$ the portion of the segment submerged and
	 * \f$\bar{g}\f$ the gravity acceleration.
	 *
	 * The weight and buoyancy force at any internal node is computed as the
	 * average of the surrounding segments, while on the line-ends \b half of
	 * the associated ending segment weight force is returned.
	 *
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param f The output node force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeWeight(MoorDynLine l,
	                                      unsigned int i,
	                                      double f[3]);

	/** @brief Get a line node drag force
	 *
	 * This is computed at each segment as
	 *
	 * \f[
	 *     \bar{D}(i) = \frac{1}{2} l_0 F(i) \rho_w \left(
	 *        Cd_t \pi d \vert \bar{v}_t(i) \vert \bar{v}_t(i)
	 *        + Cd_n d \vert \bar{v}_n(i) \vert \bar{v}_n(i)
	 *     \right)
	 * \f]
	 *
	 * with \f$l_0\f$ the unstretched segment length, \f$F(i)\f$ the portion of
	 * the segment submerged, \f$\rho_w\f$ the water density, \f$d\f$ the line
	 * diameter, \f$Cd_t\f$ and \f$Cd_n\f$ the normal and tangential drag
	 * coefficients, and \f$\bar{v}_t\f$ and \f$\bar{v}_n\f$ the tangential and
	 * normal velocities:
	 *
	 * \f[
	 *     \bar{v}_t = (\bar{v} - \bar{U}) \cdot \bar{q}
	 * \f]
	 * \f[
	 *     \bar{v}_n = (\bar{v} - \bar{U}) - \bar{v}_t
	 * \f]
	 * 
	 * with \f$\bar{v}\f$ the velocity, \f$\bar{U}\f$ the flow velocity and
	 * \f$\bar{q}\f$ the direction verctor of the segment
	 *
	 * The drag at any internal node is computed as the average of the
	 * surrounding segments, while on the line-ends \b half of the associated
	 * ending segment drag force is returned.
	 *
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param f The output node force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeDrag(MoorDynLine l,
	                                    unsigned int i,
	                                    double f[3]);

	/** @brief Get a line node Froude Krilov force
	 *
	 * This is computed at each segment as
	 *
	 * \f[
	 *     \bar{Fk}(i) = V(i) F(i) \rho_w \left(
	 *        (1 + Ca_t) \frac{\mathrm{d} \bar{v}_t}{\mathrm{d} t}
	 *        + (1 + Ca_n)  \frac{\mathrm{d} \bar{v}_n}{\mathrm{d} t}
	 *     \right)
	 * \f]
	 *
	 * with \f$V(i)\f$ the segment volume, \f$F(i)\f$ the portion of
	 * the segment submerged, \f$\rho_w\f$ the water density,
	 * \f$Cd_t\f$ and \f$Cd_n\f$ the normal and tangential drag coefficients,
	 * and \f$\frac{\mathrm{d} \bar{v}_t}{\mathrm{d} t}\f$ and
	 * \f$\frac{\mathrm{d} \bar{v}_n}{\mathrm{d} t}\f$ the tangential and
	 * normal accelerations:
	 *
	 * \f[
	 *     \frac{\mathrm{d} \bar{v}_t}{\mathrm{d} t} = 
	 *          \frac{\mathrm{d} \bar{v}}{\mathrm{d} t} \cdot \bar{q}(i)
	 * \f]
	 * \f[
	 *     \frac{\mathrm{d} \bar{v}_n}{\mathrm{d} t} =
	 *          \frac{\mathrm{d} \bar{v}}{\mathrm{d} t} -
	 *          \frac{\mathrm{d} \bar{v}_t}{\mathrm{d} t}
	 * \f]
	 * 
	 * with \f$\bar{q}(i)\f$ the direction verctor of the segment
	 *
	 * The force at any internal node is computed as the average of the
	 * surrounding segments, while on the line-ends \b half of the associated
	 * ending segment Froude-Krylov force is returned.
	 *
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param f The output node force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeFroudeKrilov(MoorDynLine l,
	                                            unsigned int i,
	                                            double f[3]);

	/** @brief Get a line node seabed reaction
	 *
	 * This is computed at each segment as
	 *
	 * \f[
	 *     \bar{B}(i) = l_0 d \left(
	 *         Kb (\bar{r}_b - \bar{r}(i)) \cdot \bar{k}
	 *         Cb \bar{v} \cdot \bar{k} +
	 *     \right) \bar{k}
	 * \f]
	 *
	 * with \f$l_0\f$ the unstretched segment length, \f$d\f$ the line diameter
	 * \f$Kb\f$ and \f$Cb\f$ the bottom stiff and drag coefficients,
	 * \f$\bar{r}_b\f$ the seabed position, \f$\bar{r}\f$ the position,
	 * \f$\bar{v}\f$ the velocity and \f$\bar{k}\f$ the upward direction
	 * vector.
	 *
	 * If bottom friction is configured, the force is added to this magnitude
	 * as well.
	 *
	 * The reaction force at any internal node is computed as the
	 * average of the surrounding segments, while on the line-ends \b half of
	 * the associated ending segment weight force is returned.
	 *
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param f The output node force
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeSeabedForce(MoorDynLine l,
	                                           unsigned int i,
	                                           double f[3]);

	/** @brief Get a line curvature at a node
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param c The output line curvature
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 * @note The curvature is only computed if bending stiffness
	 * (moordyn::Line::EI) is not zero. Otherwise the curvature of every single
	 * node will be zero.
	 */
	int DECLDIR MoorDyn_GetLineNodeCurv(MoorDynLine l,
	                                    unsigned int i,
	                                    double* c);

	/** @brief Get a line node mass matrix
	 *
	 * The mass matrix includes the node mass as well as the added mass
	 * @param l The Moordyn line
	 * @param i The node index
	 * @param m The output mass matrix
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided or if the node
	 * index is bigger than the number of segments, MOORDYN_SUCCESS otherwise
	 */
	int DECLDIR MoorDyn_GetLineNodeM(MoorDynLine l,
	                                 unsigned int i,
	                                 double m[3][3]);

	/** @brief Get the tension module at the end point B (the fairlead)
	 * @param l The Moordyn line
	 * @param t The output node tension module
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineFairTen(MoorDynLine l, double* t);

	/** @brief Get the maximum tension module
	 * @param l The Moordyn line
	 * @param t The output maximum tension module
	 * @return MOORDYN_INVALID_VALUE if a NULL line is provided, MOORDYN_SUCCESS
	 * otherwise
	 */
	int DECLDIR MoorDyn_GetLineMaxTen(MoorDynLine l, double* t);

	/** @brief Save the line to a VTK (.vtp) file
	 * @param l The Moordyn line
	 * @param filename The output maximum tension module
	 * @return MOORDYN_SUCCESS if the file is correctly written, an error code
	 * otherwise
	 * @note If MoorDyn has been built without VTK support, this function will
	 * return a MOORDYN_NON_IMPLEMENTED error, but it will be still available
	 * anyway
	 */
	int DECLDIR MoorDyn_SaveLineVTK(MoorDynLine l, const char* filename);

	/**
	 * @}
	 */

#ifdef __cplusplus
}
#endif

#endif
