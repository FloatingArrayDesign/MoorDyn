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

/** @file QSlines.hpp
 * C/C++ translation and adaptation of the Fortran subroutine "Catenary" written
 * by Jason Jonkman.
 *
 * The original function is contained in the National Renewable Energy
 * Laboratory's FAST version 7 source code, available at
 * https://nwtc.nrel.gov/FAST7 and with a disclaimer at
 * http://wind.nrel.gov/designcodes/disclaimer.html.
 *
 * This quasi-static analysis is used as a starting point for getting the
 * initial line profiles.
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <iostream>
#include <vector>

using namespace std;

/// switch to turn on excessive output for locating crashes
int longwinded = 0;

/** @brief Positions and tensions of a single mooring line
 *
 * A quasi-static approach used as a starting point for getting the initial line
 * profiles
 * @param XF Horizontal distance between anchor and fairlead (meters)
 * @param ZF Vertical distance between anchor and fairlead (meters)
 * @param L Unstretched length of line (meters)
 * @param EA Extensional stiffness of line (N)
 * @param W Weight of line in fluid per unit length (N/m)
 * @param CB Coefficient of seabed static friction drag (a negative value
 *           indicates no seabed) (-)
 * @param Tol Convergence tolerance within Newton-Raphson iteration specified as
 *            a fraction of tension (-)
 * @param HFout Output effective horizontal tension in line at the fairlead (N)
 * @param VFout Output effective vertical tension in line at the fairlead (N)
 * @param HAout Output effective horizontal tension in line at the anchor (N)
 * @param VAout Output effective vertical tension in line at the anchor (N)
 * @param Nnodes Number of nodes where the line position and tension can be
 *               output (-)
 * @param s Unstretched arc distance along line from anchor to each node where
 *          the line position and tension can be output (meters)
 * @param X Output horizontal locations of each line node relative to the anchor
 *          (meters)
 * @param Z Output vertical locations of each line node relative to the anchor
 *          (meters)
 * @param Te Output effective line tensions at each node (N)
 * @return 1 if the quasi-static equilibrium is found, -1 otherwise
 */
template<typename T>
int
Catenary(T XF,
         T ZF,
         T L,
         T EA,
         T W,
         T CB,
         T Tol,
         T* HFout,
         T* VFout,
         T* HAout,
         T* VAout,
         unsigned int Nnodes,
         vector<T>& s,
         vector<T>& X,
         vector<T>& Z,
         vector<T>& Te)
{
	if (longwinded == 1)
		cout << "In Catenary.  XF is " << XF << " and ZF is " << ZF << endl;

	// these are temporary and put to the pointers above with the "out" suffix
	T HF, VF, HA, VA;

	if (longwinded == 1)
		cout << "hi" << endl;

	// Checking inverted points at line ends: from catenary.py in MoorPy
	bool reverseFlag;
	if ( ZF <  0.0 ){ // True if the fairlead has passed below its anchor
        ZF = -ZF;
        reverseFlag = true;
		if (longwinded == 1)
     		cout << " Warning from catenary: "
			 	 << "Anchor point is above the fairlead point" << endl;
    } else reverseFlag = false;

	// Maximum stretched length of the line with seabed interaction beyond which
	// the line would have to double-back on itself; here the line forms an "L"
	// between the anchor and fairlead (i.e. it is horizontal along the seabed
	// from the anchor, then vertical to the fairlead) (meters)
	T LMax;

	if (W > 0.0) // .TRUE. when the line will sink in fluid
	{
		// Compute the maximum stretched length of the line with seabed
		// interaction beyond which the line would have to double-back on
		// itself; here the line forms an "L" between the anchor and fairlead
		// (i.e. it is horizontal along the seabed from the anchor, then
		// vertical to the fairlead)
		LMax = XF - EA / W + sqrt((EA / W) * (EA / W) + 2.0 * ZF * EA / W);
		if ((L >= LMax) && (CB >= 0.0)) {
			// .TRUE. if the line is as long or longer than its maximum possible
			// value with seabed interaction
			if (longwinded == 1)
				cout << "Warning from Catenary: "
				     << "Unstretched line length too large." << endl
				     << "       d (horiz) is " << XF << " and h (vert) is "
				     << ZF << " and L is " << L << endl;
			return -1;
		}
	}

	// Initialize some commonly used terms that don't depend on the iteration:
	T WL = W * L;
	T WEA = W * EA;
	T LOvrEA = L / EA;
	T CBOvrEA = CB / EA;
	// Smaller tolerances may take more iterations, so choose a maximum
	// inversely proportional to the tolerance
	unsigned int MaxIter = (unsigned int)(1.0 / Tol);

	// more initialization
	bool FirstIter = true; // Initialize iteration flag

	// Partial derivative of the calculated horizontal distance with respect to
	// the horizontal fairlead tension (m/N): dXF(HF,VF)/dHF
	T dXFdHF;
	// Partial derivative of the calculated horizontal distance with respect to
	// the vertical   fairlead tension (m/N): dXF(HF,VF)/dVF
	T dXFdVF;
	// Partial derivative of the calculated vertical distance with respect to
	// the horizontal fairlead tension (m/N): dZF(HF,VF)/dHF
	T dZFdHF;
	// Partial derivative of the calculated vertical distance with respect to
	// the vertical fairlead tension (m/N): dZF(HF,VF)/dVF
	T dZFdVF;
	// Error function between calculated and known horizontal distance (meters):
	// XF(HF,VF) - XF
	T EXF;
	// Error function between calculated and known vertical distance (meters):
	// ZF(HF,VF) - ZF
	T EZF;

	// Determinant of the Jacobian matrix (m^2/N^2)
	T DET;
	// Increment in HF predicted by Newton-Raphson (N)
	T dHF;
	// Increment in VF predicted by Newton-Raphson (N)
	T dVF;

	// = HF/W
	T HFOvrW;
	// = HF/WEA
	T HFOvrWEA;
	// Catenary parameter used to generate the initial guesses of the horizontal
	// and vertical tensions at the fairlead for the Newton-Raphson
	// iteration (-)
	T Lamda0;
	// = L - VF/W
	T LMinVFOvrW;
	// = s[I]/EA
	T sOvrEA;
	// = SQRT( 1.0 + VFOvrHF2 )
	T SQRT1VFOvrHF2;
	// = SQRT( 1.0 + VFMinWLOvrHF2 )
	T SQRT1VFMinWLOvrHF2;
	// = SQRT( 1.0 + VFMinWLsOvrHF*VFMinWLsOvrHF )
	T SQRT1VFMinWLsOvrHF2;
	// = VF - WL
	T VFMinWL;
	// = VFMinWL/HF
	T VFMinWLOvrHF;
	// = VFMinWLOvrHF*VFMinWLOvrHF
	T VFMinWLOvrHF2;
	// = VFMinWL + Ws
	T VFMinWLs;
	// = VFMinWLs/HF
	T VFMinWLsOvrHF;
	// = VF/HF
	T VFOvrHF;
	// = VFOvrHF*VFOvrHF
	T VFOvrHF2;
	// = VF/WEA
	T VFOvrWEA;
	// = W*s[I]
	T Ws;
	// = XF*XF
	T XF2;
	// = ZF*ZF
	T ZF2;

	// insertion - to get HF and VF initial guesses (FAST normally uses previous
	// time step)
	XF2 = XF * XF;
	ZF2 = ZF * ZF;

	if (L <= sqrt(XF2 + ZF2)) {
		//.TRUE. if the current mooring line is taut
		Lamda0 = 0.2;
	} else {
		// The current mooring line must be slack and not vertical
		Lamda0 = sqrt(3.0 * ((L * L - ZF2) / XF2 - 1.0));
	}

	HF = abs(0.5 * W * XF / Lamda0);
	VF = 0.5 * W * (ZF / tanh(Lamda0) + L);

	/*
	! To avoid an ill-conditioned situation, ensure that the initial guess for
	!   HF is not less than or equal to zero.  Similarly, avoid the problems
	!   associated with having exactly vertical (so that HF is zero) or exactly
	!   horizontal (so that VF is zero) lines by setting the minimum values
	!   equal to the tolerance.  This prevents us from needing to implement
	!   the known limiting solutions for vertical or horizontal lines (and thus
	!   complicating this routine):
	*/

	HF = max(HF, Tol);
	XF = max(XF, Tol);
	ZF = max(ZF, Tol);

	// Solve the analytical, static equilibrium equations for a catenary (or
	// taut) mooring line with seabed interaction:

	// Begin Newton-Raphson iteration:
	for (unsigned int I = 1; I <= MaxIter; I++) {
		// Initialize some commonly used terms that depend on HF and VF:
		HFOvrWEA = HF / WEA;
		VFOvrWEA = VF / WEA;
		VFOvrHF = VF / HF;
		VFOvrHF2 = VFOvrHF * VFOvrHF;
		SQRT1VFOvrHF2 = sqrt(1.0 + VFOvrHF2);
		// These variables below need to be located in for loop, otherwise
		// catenary solver fails
		VFMinWL = VF - WL;
		HFOvrW = HF / W;
		LMinVFOvrW = L - VF / W;
		VFMinWLOvrHF = VFMinWL / HF;
		VFMinWLOvrHF2 = VFMinWLOvrHF * VFMinWLOvrHF;
		SQRT1VFMinWLOvrHF2 = sqrt(1.0 + VFMinWLOvrHF2);

		// Compute the error functions (to be zeroed) and the Jacobian matrix
		//   (these depend on the anticipated configuration of the mooring
		//   line):
		if ((CB < 0.0) || (W < 0.0) || (VFMinWL > 0.0)) {
			// .TRUE. when no portion of the line rests on the seabed
			EXF = (log(VFOvrHF + SQRT1VFOvrHF2) -
			       log(VFMinWLOvrHF + SQRT1VFMinWLOvrHF2)) *
			          HFOvrW +
			      LOvrEA * HF - XF;
			EZF = (SQRT1VFOvrHF2 - SQRT1VFMinWLOvrHF2) * HFOvrW +
			      LOvrEA * (VF - 0.5 * WL) - ZF;
			dXFdHF = (log(VFOvrHF + SQRT1VFOvrHF2) -
			          log(VFMinWLOvrHF + SQRT1VFMinWLOvrHF2)) /
			             W -
			         ((VFOvrHF + VFOvrHF2 / SQRT1VFOvrHF2) /
			              (VFOvrHF + SQRT1VFOvrHF2) -
			          (VFMinWLOvrHF + VFMinWLOvrHF2 / SQRT1VFMinWLOvrHF2) /
			              (VFMinWLOvrHF + SQRT1VFMinWLOvrHF2)) /
			             W +
			         LOvrEA;
			dXFdVF =
			    ((1.0 + VFOvrHF / SQRT1VFOvrHF2) / (VFOvrHF + SQRT1VFOvrHF2) -
			     (1.0 + VFMinWLOvrHF / SQRT1VFMinWLOvrHF2) /
			         (VFMinWLOvrHF + SQRT1VFMinWLOvrHF2)) /
			    W;
			dZFdHF = (SQRT1VFOvrHF2 - SQRT1VFMinWLOvrHF2) / W -
			         (VFOvrHF2 / SQRT1VFOvrHF2 -
			          VFMinWLOvrHF2 / SQRT1VFMinWLOvrHF2) /
			             W;
			dZFdVF =
			    (VFOvrHF / SQRT1VFOvrHF2 - VFMinWLOvrHF / SQRT1VFMinWLOvrHF2) /
			        W +
			    LOvrEA;
		} else if (-CB * VFMinWL < HF) {
			// .TRUE. when a portion of the line rests on the seabed and the
			// anchor tension is nonzero
			EXF = log(VFOvrHF + SQRT1VFOvrHF2) * HFOvrW -
			      0.5 * CBOvrEA * W * LMinVFOvrW * LMinVFOvrW + LOvrEA * HF +
			      LMinVFOvrW - XF;
			EZF = (SQRT1VFOvrHF2 - 1.0) * HFOvrW + 0.5 * VF * VFOvrWEA - ZF;

			dXFdHF = log(VFOvrHF + SQRT1VFOvrHF2) / W -
			         ((VFOvrHF + VFOvrHF2 / SQRT1VFOvrHF2) /
			          (VFOvrHF + SQRT1VFOvrHF2)) /
			             W +
			         LOvrEA;
			dXFdVF =
			    ((1.0 + VFOvrHF / SQRT1VFOvrHF2) / (VFOvrHF + SQRT1VFOvrHF2)) /
			        W +
			    CBOvrEA * LMinVFOvrW - 1.0 / W;
			dZFdHF = (SQRT1VFOvrHF2 - 1.0 - VFOvrHF2 / SQRT1VFOvrHF2) / W;
			dZFdVF = (VFOvrHF / SQRT1VFOvrHF2) / W + VFOvrWEA;
		} else {
			// A portion of the line must rest on the seabed and the anchor
			// tension is zero
			EXF =
			    log(VFOvrHF + SQRT1VFOvrHF2) * HFOvrW -
			    0.5 * CBOvrEA * W *
			        (LMinVFOvrW * LMinVFOvrW -
			         (LMinVFOvrW - HFOvrW / CB) * (LMinVFOvrW - HFOvrW / CB)) +
			    LOvrEA * HF + LMinVFOvrW - XF;
			EZF = (SQRT1VFOvrHF2 - 1.0) * HFOvrW + 0.5 * VF * VFOvrWEA - ZF;
			dXFdHF = log(VFOvrHF + SQRT1VFOvrHF2) / W -
			         ((VFOvrHF + VFOvrHF2 / SQRT1VFOvrHF2) /
			          (VFOvrHF + SQRT1VFOvrHF2)) /
			             W +
			         LOvrEA - (LMinVFOvrW - HFOvrW / CB) / EA;
			dXFdVF =
			    ((1.0 + VFOvrHF / SQRT1VFOvrHF2) / (VFOvrHF + SQRT1VFOvrHF2)) /
			        W +
			    HFOvrWEA - 1.0 / W;
			dZFdHF = (SQRT1VFOvrHF2 - 1.0 - VFOvrHF2 / SQRT1VFOvrHF2) / W;
			dZFdVF = (VFOvrHF / SQRT1VFOvrHF2) / W + VFOvrWEA;
		}

		// Compute the determinant of the Jacobian matrix and the incremental
		//   tensions predicted by Newton-Raphson:
		DET = dXFdHF * dZFdVF - dXFdVF * dZFdHF;

		// This is the incremental change in horizontal tension at the fairlead
		// as predicted by Newton-Raphson
		dHF = (-dZFdVF * EXF + dXFdVF * EZF) / DET;
		// This is the incremental change in vertical tension at the fairlead as
		// predicted by Newton-Raphson
		dVF = (dZFdHF * EXF - dXFdHF * EZF) / DET;

		// ! Reduce dHF by factor (between 1 at I = 1 and 0 at I = MaxIter) that
		// reduces linearly with iteration count to ensure that we converge on a
		// solution even in the case were we obtain a nonconvergent cycle about
		// the correct solution (this happens, for example, if we jump to
		// quickly between a taut and slack catenary)
		dHF = dHF * (1.0 - Tol * I);
		// ! Reduce dHF by factor (between 1 at I = 1 and 0 at I = MaxIter) that
		// reduces linearly with iteration count to ensure that we converge on a
		// solution even in the case were we obtain a nonconvergent cycle about
		// the correct solution (this happens, for example, if we jump to
		// quickly between a taut and slack catenary)
		dVF = dVF * (1.0 - Tol * I);

		//! To avoid an ill-conditioned situation, make sure HF does not go less
		// than or equal to zero by having a lower limit of Tol*HF [NOTE: the
		// value of dHF = ( Tol - 1.0 )*HF comes from: HF = HF + dHF = Tol*HF
		// when dHF = ( Tol - 1.0 )*HF]
		dHF = max(dHF, (T)(Tol - 1.0) * HF);

		// Check if we have converged on a solution, or restart the iteration,
		// or abort if we cannot find a solution:
		if ((abs(dHF) <= abs(Tol * HF)) && (abs(dVF) <= abs(Tol * VF))) {
			// .TRUE. if we have converged; stop iterating!
			// [The converge tolerance, Tol, is a fraction of tension]
			break;
		}

		else if ((I == MaxIter) && FirstIter) {
			// .TRUE. if we've iterated MaxIter-times for the first time, try a
			// new set of ICs;
			/*
			! Perhaps we failed to converge because our initial guess was too
			far off. !   (This could happen, for example, while linearizing a
			model via large !   pertubations in the DOFs.)  Instead, use
			starting values documented in: !   Peyrot, Alain H. and Goulois, A.
			M., "Analysis Of Cable Structures," !   Computers & Structures, Vol.
			10, 1979, pp. 805-813: ! NOTE: We don't need to check if the current
			mooring line is exactly !       vertical (i.e., we don't need to
			check if XF == 0.0), because XF is !       limited by the tolerance
			above.
			*/

			XF2 = XF * XF;
			ZF2 = ZF * ZF;

			if (L <= sqrt(XF2 + ZF2)) {
				//.TRUE. if the current mooring line is taut
				Lamda0 = 0.2;
			} else {
				// The current mooring line must be slack and not vertical
				Lamda0 = sqrt(3.0 * ((L * L - ZF2) / XF2 - 1.0));
			}

			// ! As above, set the lower limit of the guess value of HF to the
			// tolerance
			HF = max((T)abs(0.5 * W * XF / Lamda0), Tol);
			VF = 0.5 * W * (ZF / tanh(Lamda0) + L);

			// Restart Newton-Raphson iteration:
			I = 0;
			FirstIter = false;
			dHF = 0.0;
			dVF = 0.0;
		}

		else if ((I == MaxIter) && (!FirstIter)) {
			// .TRUE. if we've iterated as much as we can take without finding a
			// solution; Abort
			if (longwinded == 1)
				cout << "Reached max iterations without finding solution, "
				     << "aborting catenary solver ..." << endl;
			return -1;
		}

		// Increment fairlead tensions and iterate...
		HF = HF + dHF;
		VF = VF + dVF;
	}

	/*
	! We have found a solution for the tensions at the fairlead!
	! Now compute the tensions at the anchor and the line position and tension
	!   at each node (again, these depend on the configuration of the mooring
	!   line):
	*/

	if ((CB < 0.0) || (W < 0.0) || (VFMinWL > 0.0)) {
		// .TRUE. when no portion of the line rests on the seabed
		// Anchor tensions:
		HA = HF;
		VA = VFMinWL;

		//! Line position and tension at each node:
		for (unsigned int I = 0; I < Nnodes; I++) {
			if ((s[I] < 0.0) || (s[I] > L)) {
				if (longwinded == 1)
					cout << "Warning from Catenary: "
					     << "All line nodes must be located between the anchor "
					     << "and fairlead (inclusive) in routine Catenary()"
					     << endl;
				return -1;
			}

			// Initialize some commonly used terms that depend on s[I]
			Ws = W * s[I];
			VFMinWLs = VFMinWL + Ws;
			VFMinWLsOvrHF = VFMinWLs / HF;
			sOvrEA = s[I] / EA;
			SQRT1VFMinWLsOvrHF2 = sqrt(1.0 + VFMinWLsOvrHF * VFMinWLsOvrHF);

			X[I] = (log(VFMinWLsOvrHF + SQRT1VFMinWLsOvrHF2) -
			        log(VFMinWLOvrHF + SQRT1VFMinWLOvrHF2)) *
			           HFOvrW +
			       sOvrEA * HF;
			Z[I] = (SQRT1VFMinWLsOvrHF2 - SQRT1VFMinWLOvrHF2) * HFOvrW +
			       sOvrEA * (VFMinWL + 0.5 * Ws);
			Te[I] = sqrt(HF * HF + VFMinWLs * VFMinWLs);
		}
	} else if (-CB * VFMinWL < HF) {
		// .TRUE. when a portion of the line rests on the seabed and the anchor
		// tension is nonzero

		// Anchor tensions:
		HA = HF + CB * VFMinWL;
		VA = 0.0;

		// Line position and tension at each node:
		for (unsigned int I = 0; I < Nnodes; I++) {
			if ((s[I] < 0.0) || (s[I] > L)) {
				if (longwinded == 1)
					cout << "Warning from Catenary: "
					     << "All line nodes must be located between the anchor "
					     << "and fairlead (inclusive) in routine Catenary()"
					     << endl;
				return -1;
			}

			// Initialize some commonly used terms that depend on s[I]
			Ws = W * s[I];
			VFMinWLs = VFMinWL + Ws;
			VFMinWLsOvrHF = VFMinWLs / HF;
			sOvrEA = s[I] / EA;
			SQRT1VFMinWLsOvrHF2 = sqrt(1.0 + VFMinWLsOvrHF * VFMinWLsOvrHF);

			if (s[I] <= LMinVFOvrW) {
				// .TRUE. if this node rests on the seabed and the tension is
				// nonzero
				X[I] = s[I] + sOvrEA * (HF + CB * VFMinWL + 0.5 * Ws * CB);
				Z[I] = 0.0;
				Te[I] = HF + CB * VFMinWLs;
			} else {
				// LMinVFOvrW < s <= L ! This node must be above the seabed
				X[I] = log(VFMinWLsOvrHF + SQRT1VFMinWLsOvrHF2) * HFOvrW +
				       sOvrEA * HF + LMinVFOvrW -
				       0.5 * CB * VFMinWL * VFMinWL / WEA;
				Z[I] = (-1.0 + SQRT1VFMinWLsOvrHF2) * HFOvrW +
				       sOvrEA * (VFMinWL + 0.5 * Ws) +
				       0.5 * VFMinWL * VFMinWL / WEA;
				Te[I] = sqrt(HF * HF + VFMinWLs * VFMinWLs);
			}
		}

	} else {
		// 0.0 <  HF  <= -CB*VFMinWL   ! A  portion of the line must rest on the
		// seabed and the anchor tension is    zero

		// Anchor tensions:
		HA = 0.0;
		VA = 0.0;

		// Line position and tension at each node:
		for (unsigned int I = 0; I < Nnodes; I++) {
			if ((s[I] < 0.0) || (s[I] > L)) {
				if (longwinded == 1)
					cout << "Warning from Catenary: "
					     << "All line nodes must be located between the anchor "
					     << "and fairlead (inclusive) in routine Catenary()"
					     << endl;
				return -1;
			}

			// Initialize some commonly used terms that depend on s[I]
			Ws = W * s[I];
			VFMinWLs = VFMinWL + Ws;
			VFMinWLsOvrHF = VFMinWLs / HF;
			sOvrEA = s[I] / EA;
			SQRT1VFMinWLsOvrHF2 = sqrt(1.0 + VFMinWLsOvrHF * VFMinWLsOvrHF);

			if (s[I] <= LMinVFOvrW - HFOvrW / CB) {
				// .TRUE. if this node rests on the seabed and the tension is
				// zero
				X[I] = s[I];
				Z[I] = 0.0;
				Te[I] = 0.0;
			} else if (s[I] <= LMinVFOvrW) {
				// .TRUE. if this node rests on the seabed and the tension is
				// nonzero
				X[I] = s[I] - (LMinVFOvrW - 0.5 * HFOvrW / CB) * HF / EA +
				       sOvrEA * (HF + CB * VFMinWL + 0.5 * Ws * CB) +
				       0.5 * CB * VFMinWL * VFMinWL / WEA;
				Z[I] = 0.0;
				Te[I] = HF + CB * VFMinWLs;
			} else {
				// LMinVFOvrW < s <= L ! This node must be above the seabed
				X[I] = log(VFMinWLsOvrHF + SQRT1VFMinWLsOvrHF2) * HFOvrW +
				       sOvrEA * HF + LMinVFOvrW -
				       (LMinVFOvrW - 0.5 * HFOvrW / CB) * HF / EA;
				Z[I] = (-1.0 + SQRT1VFMinWLsOvrHF2) * HFOvrW +
				       sOvrEA * (VFMinWL + 0.5 * Ws) +
				       0.5 * VFMinWL * VFMinWL / WEA;
				Te[I] = sqrt(HF * HF + VFMinWLs * VFMinWLs);
			}
		}
	}

	*HFout = HF;
	*VFout = VF;
	*HAout = HA;
	*VAout = VA;

	if (reverseFlag) { // return values to normal
		reverse(s.begin(), s.end());
		reverse(X.begin(), X.end());
		reverse(Z.begin(), Z.end());
		reverse(Te.begin(), Te.end());
		for (unsigned int I = 0; I < Nnodes; I++){
			s[I] = L - s[I];
			X[I] = XF - X[I];
			Z[I] = Z[I] - ZF;
		}
		ZF = -ZF;
	}

	return 1;
}
