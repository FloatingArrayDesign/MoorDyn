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

#include "Line.hpp"
#include "Line.h"
#include "Waves.hpp"
#include "QSlines.hpp"
#include <tuple>

#ifdef USE_VTK
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLPolyDataWriter.h>
#endif

using namespace std;

namespace moordyn {

Line::Line(moordyn::Log* log)
  : io::IO(log)
{
}

Line::~Line() {}

real
Line::getNonlinearE(real l_stretched, real l_unstretched)
{
	if (!nEApoints)
		return EI;

	real Xi = l_stretched / l_unstretched - 1.0; // strain rate based on inputs
	if (Xi < 0.0) {
		// if negative strain (compression), zero stress
		return 0.0;
	}

	real Yi = interp(stiffXs, stiffYs, Xi);

	// calculate equivalent elasticity (since that's what MoorDyn works with)
	return Yi / Xi;
}

real
Line::getNonlinearEI(real curv)
{
	if (!nEIpoints)
		return EI;

	real Xi = curv;
	real Yi = interp(bstiffXs, bstiffYs, Xi);

	// calculate equivalent bending stiffness (since that's what MoorDyn works
	// with)
	return Yi / Xi;
}

real
Line::getNonlinearC(real ld_stretched, real l_unstretched)
{
	if (!nCpoints)
		return c;

	real Xi = ld_stretched / l_unstretched; // stretching/compressing rate
	real Yi = 0.0;

	// find stress based on strain rate
	if (dampXs[0] < 0) {
		// first check if lookup table includes compressing
		Yi = interp(dampXs, dampYs, Xi);
	} else {
		// if no compressing data given, we'll flip-mirror so stretching and
		// compressing are same
		real Xsign = 1.0;
		if (Xi < 0) {
			Xsign = -1.0;
			Xi = -Xi;
		}
		Yi = interp(dampXs, dampYs, Xi);
		Yi *= Xsign;
	}

	// calculate equivalent damping coefficient (since that's what MoorDyn works
	// with)
	return Yi / Xi;
}

void
Line::setup(int number_in,
            LineProps* props,
            real UnstrLen_in,
            unsigned int NumSegs,
            shared_ptr<ofstream> outfile_pointer,
            string channels_in)
{
	number = number_in;
	// Note, this is a temporary value that will be processed depending on sign
	// during initializeLine
	UnstrLen = UnstrLen_in;
	UnstrLend = 0.0;
	// assign number of nodes to line
	N = NumSegs;

	// store passed line properties (and convert to numbers)
	d = props->d;
	A = pi / 4. * d * d;
	rho = props->w / A;
	E = props->EA / A;
	EI = props->EI;
	BAin = props->c;
	Can = props->Can;
	Cat = props->Cat;
	Cdn = props->Cdn;
	Cdt = props->Cdt;

	// copy in nonlinear stress-strain data if applicable
	stiffXs.clear();
	stiffYs.clear();
	nEApoints = props->nEApoints;
	for (unsigned int I = 0; I < nEApoints; I++) {
		stiffXs.push_back(props->stiffXs[I]);
		stiffYs.push_back(props->stiffYs[I] / A);
	}

	// copy in nonlinear bent stiffness data if applicable
	bstiffXs.clear();
	bstiffYs.clear();
	nEIpoints = props->nEIpoints;
	for (unsigned int I = 0; I < nEIpoints; I++) {
		bstiffXs.push_back(props->bstiffXs[I]);
		bstiffYs.push_back(props->bstiffYs[I]);
	}

	// copy in nonlinear stress-strainrate data if applicable
	dampXs.clear();
	dampYs.clear();
	nCpoints = props->nCpoints;
	for (unsigned int I = 0; I < nCpoints; I++) {
		dampXs.push_back(props->dampXs[I]);
		dampYs.push_back(props->dampYs[I]);
	}

	// ------------------------- size vectors -------------------------

	r.assign(N + 1, vec(0., 0., 0.));  // node positions [i][x/y/z]
	rd.assign(N + 1, vec(0., 0., 0.)); // node positions [i][x/y/z]
	q.assign(N + 1, vec(0., 0., 0.));  // unit tangent vectors for each node
	qs.assign(N, vec(0., 0., 0.));     // unit tangent vectors for each segment
	l.assign(N, 0.0);                  // line unstretched segment lengths
	lstr.assign(N, 0.0);               // stretched lengths
	ldstr.assign(N, 0.0);              // rate of stretch
	Kurv.assign(N + 1, 0.0);           // curvatures at node points (1/m)

	M.assign(N + 1, mat()); // mass matrices (3x3) for each node
	V.assign(N, 0.0);       // segment volume?

	// forces
	T.assign(N, vec(0., 0., 0.));        // segment tensions
	Td.assign(N, vec(0., 0., 0.));       // segment damping forces
	Bs.assign(N + 1, vec(0., 0., 0.));   // bending stiffness forces
	W.assign(N + 1, vec(0., 0., 0.));    // node weights
	Dp.assign(N + 1, vec(0., 0., 0.));   // node drag (transverse)
	Dq.assign(N + 1, vec(0., 0., 0.));   // node drag (axial)
	Ap.assign(N + 1, vec(0., 0., 0.));   // node added mass forcing (transverse)
	Aq.assign(N + 1, vec(0., 0., 0.));   // node added mass forcing (axial)
	B.assign(N + 1, vec(0., 0., 0.));    // node bottom contact force
	Fnet.assign(N + 1, vec(0., 0., 0.)); // total force on node

	// wave things
	F.assign(N + 1, 0.0); // VOF scaler for each NODE (mean of two half adjacent
	                      // segments) (1 = fully submerged, 0 = out of water)
	zeta.assign(N + 1, 0.0);           // wave elevation above each node
	PDyn.assign(N + 1, 0.0);           // dynamic pressure
	U.assign(N + 1, vec(0., 0., 0.));  // wave velocities
	Ud.assign(N + 1, vec(0., 0., 0.)); // wave accelerations

	// ensure end moments start at zero
	endMomentA = vec(0., 0., 0.);
	endMomentB = vec(0., 0., 0.);

	// set the number of preset wave kinematic time steps to zero (flagging
	// disabled) to start with
	ntWater = 0;

	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); // make outfile point to the right place
	channels = channels_in;          // copy string of output channels to object

	LOGDBG << "   Set up Line " << number << ". " << endl;
};

void
Line::setEnv(EnvCond* env_in, moordyn::Waves* waves_in)
{
	env = env_in;
	waves = waves_in;
}

std::pair<std::vector<vec>, std::vector<vec>>
Line::initialize()
{
	LOGMSG << "  - Line" << number << ":" << endl
	       << "    ID: " << number << endl
	       << "    UnstrLen: " << UnstrLen << endl
	       << "    N   : " << N << endl
	       << "    d   : " << d << endl
	       << "    rho : " << rho << endl
	       << "    E   : " << E << endl
	       << "    EI  : " << EI << endl
	       << "    BAin: " << BAin << endl
	       << "    Can : " << Can << endl
	       << "    Cat : " << Cat << endl
	       << "    Cdn : " << Cdn << endl
	       << "    Cdt : " << Cdt << endl
	       << "    ww_l: " << ((rho - env->rho_w) * (pi / 4. * d * d)) * 9.81
	       << endl;

	if (outfile) {
		if (!outfile->is_open()) {
			LOGERR << "Unable to write file Line" << number << ".out" << endl;
			throw moordyn::output_file_error("Invalid line file");
		}

		// Write the header
		// 1st line with the fields

		// output time
		*outfile << "Time"
		         << "\t ";

		// output positions
		if (channels.find("p") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "px \t Node" << i << "py \t Node"
				         << i << "pz \t ";
			}
		}
		// output curvatures
		if (channels.find("K") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "Ku \t ";
			}
		}
		// output velocities
		if (channels.find("v") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "vx \t Node" << i << "vy \t Node"
				         << i << "vz \t ";
			}
		}
		// output wave velocities
		if (channels.find("U") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "Ux \t Node" << i << "Uy \t Node"
				         << i << "Uz \t ";
			}
		}
		// output hydro force
		if (channels.find("D") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "Dx \t Node" << i << "Dy \t Node"
				         << i << "Dz \t ";
			}
		}
		// output internal damping force
		if (channels.find("c") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "cx \t Node" << i << "cy \t Node" << i
				         << "cz \t ";
			}
		}
		// output segment tensions
		if (channels.find("t") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "Te \t ";
			}
		}
		// output segment strains
		if (channels.find("s") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "St \t ";
			}
		}
		// output segment strain rates
		if (channels.find("d") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "dSt \t ";
			}
		}
		// output seabed contact forces
		if (channels.find("b") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "bx \t Node" << i << "by \t Node"
				         << i << "bz \t ";
			}
		}

		*outfile << "\n";

		if (env->WriteUnits > 0) {
			// 2nd line with the units

			// output time
			*outfile << "(s)"
			         << "\t ";

			// output positions
			if (channels.find("p") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(m) \t";
			}
			// output curvatures?
			if (channels.find("K") != string::npos) {
				for (unsigned int i = 0; i <= N; i++) {
					*outfile << "(1/m) \t ";
				}
			}
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(m/s) \t";
			}
			// output wave velocities?
			if (channels.find("U") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(m/s) \t";
			}
			// output hydro force
			if (channels.find("D") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(N) \t";
			}
			// output internal damping force?
			if (channels.find("c") != string::npos) {
				for (unsigned int i = 0; i < N; i++)
					*outfile << "(N) \t";
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (unsigned int i = 0; i < N; i++)
					*outfile << "(N) \t";
			}
			// output segment strains?
			if (channels.find("s") != string::npos) {
				for (unsigned int i = 0; i < N; i++)
					*outfile << "(-) \t";
			}
			// output segment strain rates?
			if (channels.find("d") != string::npos) {
				for (unsigned int i = 0; i < N; i++)
					*outfile << "(-/s) \t";
			}
			// output seabed contact force?
			if (channels.find("D") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(N) \t";
			}

			*outfile << "\n";
		}
	}

	// The end node kinematics should already have been set by the
	// corresponding Connection or Rod objects calling "setEndState",
	// so now we can proceed with figuring out the positions of the nodes along
	// the line.

	if (-env->WtrDpth > r[0][2]) {
		LOGERR << "Water depth is shallower than Line " << number << " anchor"
		       << endl;
		throw moordyn::invalid_value_error("Invalid water depth");
	}

	// set water kinematics flag based on global wave and current settings
	// (for now)
	if ((env->WaveKin == WAVES_FFT_GRID) || (env->WaveKin == WAVES_GRID) ||
	    (env->WaveKin == WAVES_KIN) || (env->Current == CURRENTS_STEADY_GRID) ||
	    (env->Current == CURRENTS_DYNAMIC_GRID)) {
		// water kinematics to be considered through precalculated global grid
		// stored in Waves object
		WaterKin = WAVES_GRID;
	} else if ((env->WaveKin == WAVES_FFT_NODE) ||
	           (env->WaveKin == WAVES_NODE) ||
	           (env->Current == CURRENTS_STEADY_NODE) ||
	           (env->Current == CURRENTS_DYNAMIC_NODE)) {
		// water kinematics to be considered through precalculated time series
		// for each node
		WaterKin = WAVES_EXTERNAL;
	} else {
		// no water kinematics to be considered (or to be set externally on each
		// node)
		WaterKin = WAVES_NONE;
		U.assign(N + 1, vec(0.0, 0.0, 0.0));
		Ud.assign(N + 1, vec(0.0, 0.0, 0.0));
		F.assign(N + 1, 1.0);
	}

	// process unstretched line length input
	vec dir = r[N] - r[0];
	if (UnstrLen < 0) {
		// Interpret as scaler relative to distance between initial line end
		// points (which have now been set by the relevant Connection objects)
		UnstrLen = -UnstrLen * dir.norm();
		LOGMSG << "Line " << number << " unstretched length set to " << UnstrLen
		       << " m" << endl;
	}
	UnstrLen0 = UnstrLen;

	// now that line length is known, assign length and volume properties
	for (unsigned int i = 0; i < N; i++) {
		// distribute line length evenly over segments
		l[i] = UnstrLen / double(N);
		V[i] = l[i] * A;
	}

	if (nEApoints > 0) {
		// For the sake of the following initialization steps, if using a
		// nonlinear stiffness model, set the stiffness based on the last
		// entries in the lookup table
		E = stiffYs.back() / stiffXs.back();
	}

	// process internal damping input
	if (BAin < 0) {
		// automatic internal damping option (if negative BA provided (stored as
		// BAin), then -BAin indicates desired damping ratio)
		c = -BAin * UnstrLen / N * sqrt(E * rho); // rho = w/A
		LOGMSG << "Line " << number << " damping set to " << c
		       << " Ns = " << c * A << " Pa-s, based on input of " << BAin
		       << endl;
	} else {
		// otherwise it's the regular internal damping coefficient, which should
		// be divided by area to get a material coefficient
		c = BAin / A;
	}

	// initialize line node positions as distributed linearly between the
	// endpoints
	for (unsigned int i = 1; i < N; i++) {
		r[i] = r[0] + dir * (i / (real)N);
	}

	// if conditions are ideal, try to calculate initial line profile using
	// catenary routine (from FAST v.7)
	if (-r[0][0] == env->WtrDpth) {
		real XF = dir(Eigen::seqN(0, 2)).norm(); // horizontal spread
		real ZF = dir[2];
		real LW = ((rho - env->rho_w) * A) * env->g;
		real CB = 0.;
		real Tol = 0.00001;

		if ((XF > 0.0) && (ZF > 0.0)) {
			// locations of line nodes along line length - evenly distributed
			// here
			vector<real> snodes(N + 1, 0.0);
			for (unsigned int i = 1; i <= N; i++)
				snodes[i] = snodes[i - 1] + l[i - 1];
			// double check to ensure the last node does not surpass the line
			// length
			snodes[N] = UnstrLen;

			// output variables
			real HF, VF, HA, VA, COSPhi, SINPhi;
			vector<real> Xl(N + 1, 0.0);
			vector<real> Zl(N + 1, 0.0);
			vector<real> Te(N + 1, 0.0);

			COSPhi = (r[N][0] - r[0][0]) / XF;
			SINPhi = (r[N][1] - r[0][1]) / XF;

			int success = Catenary(XF,
			                       ZF,
			                       UnstrLen,
			                       E * A,
			                       LW,
			                       CB,
			                       Tol,
			                       &HF,
			                       &VF,
			                       &HA,
			                       &VA,
			                       N,
			                       snodes,
			                       Xl,
			                       Zl,
			                       Te);

			if (success >= 0) {
				// the catenary solve is successful, update the node positions
				LOGDBG << "Catenary initial profile available for Line "
				       << number << endl;
				for (unsigned int i = 1; i < N; i++) {
					vec l(Xl[i] * COSPhi, Xl[i] * SINPhi, Zl[i]);
					r[i] = r[0] + l;
				}
			} else {
				LOGWRN << "Catenary initial profile failed for Line " << number
				       << endl;
			}
		}
	}

	LOGMSG << "Initialized Line " << number << endl;

	// also assign the resulting internal node positions to the integrator
	// initial state vector! (velocities leave at 0)
	std::vector<vec> vel(N - 1, vec::Zero());
	return std::make_pair(vector_slice(r, 1, N - 1), vel);
};

real
Line::GetLineOutput(OutChanProps outChan)
{
	if (outChan.QType == PosX)
		return r[outChan.NodeID][0];
	else if (outChan.QType == PosY)
		return r[outChan.NodeID][1];
	else if (outChan.QType == PosZ)
		return r[outChan.NodeID][2];
	else if (outChan.QType == VelX)
		return rd[outChan.NodeID][0];
	else if (outChan.QType == VelY)
		return rd[outChan.NodeID][1];
	else if (outChan.QType == VelZ)
		return rd[outChan.NodeID][2];
	else if (outChan.QType == Ten)
		return getNodeTen(outChan.NodeID).norm();
	else if (outChan.QType == FX)
		return Fnet[outChan.NodeID][0];
	else if (outChan.QType == FY)
		return Fnet[outChan.NodeID][1];
	else if (outChan.QType == FZ)
		return Fnet[outChan.NodeID][2];
	LOGWRN << "Unrecognized output channel " << outChan.QType << endl;
	return 0.0;
}

void
Line::storeWaterKin(real dt,
                    std::vector<std::vector<moordyn::real>> zeta_in,
                    std::vector<std::vector<moordyn::real>> f_in,
                    std::vector<std::vector<vec>> u_in,
                    std::vector<std::vector<vec>> ud_in)
{
	if ((zeta_in.size() != N + 1) || (f_in.size() != N + 1) ||
	    (u_in.size() != N + 1) || (ud_in.size() != N + 1)) {
		LOGERR << "Invalid input length" << endl;
		throw moordyn::invalid_value_error("Invalid input size");
	}

	ntWater = zeta_in[0].size();
	dtWater = dt;

	LOGDBG << "Setting up wave variables for Line " << number
	       << "!  ---------------------" << endl
	       << "   nt=" << ntWater << ", and WaveDT=" << dtWater
	       << ", env->WtrDpth=" << env->WtrDpth << endl;

	// resize the new time series vectors
	zetaTS.assign(N + 1, std::vector<moordyn::real>(ntWater, 0.0));
	FTS.assign(N + 1, std::vector<moordyn::real>(ntWater, 0.0));
	UTS.assign(N + 1, std::vector<vec>(ntWater, vec(0.0, 0.0, 0.0)));
	UdTS.assign(N + 1, std::vector<vec>(ntWater, vec(0.0, 0.0, 0.0)));

	for (unsigned int i = 0; i < N + 1; i++) {
		if ((zeta_in[i].size() != N + 1) || (f_in[i].size() != N + 1) ||
		    (u_in[i].size() != N + 1) || (ud_in[i].size() != N + 1)) {
			LOGERR << "Invalid input length" << endl;
			throw moordyn::invalid_value_error("Invalid input size");
		}
		zetaTS[i] = zeta_in[i];
		FTS[i] = f_in[i];
		u_in[i] = UTS[i];
		ud_in[i] = UdTS[i];
	}
};

void
Line::setState(std::vector<vec> pos, std::vector<vec> vel)
{
	if ((pos.size() != N - 1) || (vel.size() != N - 1)) {
		LOGERR << "Invalid input size" << endl;
		throw moordyn::invalid_value_error("Invalid input size");
	}

	// set interior node positions and velocities based on state vector
	for (unsigned int i = 1; i < N; i++) {
		r[i] = pos[i - 1];
		rd[i] = vel[i - 1];
	}
}

void
Line::setEndKinematics(vec pos, vec vel, EndPoints end_point)
{
	switch (end_point) {
		case ENDPOINT_TOP:
			endTypeB = PINNED; // indicate pinned
			r[N] = pos;
			rd[N] = vel;
			break;
		case ENDPOINT_BOTTOM:
			endTypeA = PINNED; // indicate pinned
			r[0] = pos;
			rd[0] = vel;
			break;
		default:
			LOGERR << "Invalid end point qualifier: " << end_point << endl;
			throw moordyn::invalid_value_error("Invalid end point");
	}
}

void
Line::setEndOrientation(vec qin, EndPoints end_point, EndPoints rod_end_point)
{
	if ((rod_end_point != ENDPOINT_A) && (rod_end_point != ENDPOINT_B)) {
		LOGERR << "Invalid rod end point qualifier: " << rod_end_point << endl;
		throw moordyn::invalid_value_error("Invalid end point");
	}
	switch (end_point) {
		case ENDPOINT_TOP:
			endTypeB = CANTILEVERED; // indicate pinned
			q[N] = qin;              // -----line----->[A==ROD==>B]
			if (rod_end_point == ENDPOINT_B)
				q[N] *= -1.0; // -----line----->[B<==ROD==A]
			break;
		case ENDPOINT_BOTTOM:
			endTypeA = CANTILEVERED; // indicate pinned
			q[0] = qin;              // [A==ROD==>B]-----line----->
			if (rod_end_point == ENDPOINT_A)
				q[0] *= -1.0; // [B<==ROD==A]-----line----->
			break;
		default:
			LOGERR << "Invalid end point qualifier: " << end_point << endl;
			throw moordyn::invalid_value_error("Invalid end point");
	}
}

vec
Line::getEndSegmentMoment(EndPoints end_point, EndPoints rod_end_point) const
{
	real dlEnd, EIEnd;
	vec qEnd;

	if ((rod_end_point != ENDPOINT_A) && (rod_end_point != ENDPOINT_B)) {
		LOGERR << "Invalid rod end point qualifier: " << rod_end_point << endl;
		throw moordyn::invalid_value_error("Invalid end point");
	}
	switch (end_point) {
		case ENDPOINT_TOP:
			// unit vector of last line segment
			dlEnd = unitvector(qEnd, r[N - 1], r[N]);
			if (rod_end_point == ENDPOINT_A) {
				// -----line----->[A==ROD==>B]
				EIEnd = EI;
			} else {
				// -----line----->[B==ROD==>A]
				EIEnd = -EI;
			}
			break;
		case ENDPOINT_BOTTOM:
			// unit vector of last line segment
			dlEnd = unitvector(qEnd, r[N - 1], r[N]);
			if (rod_end_point == ENDPOINT_A) {
				// <----line-----[A==ROD==>B]
				EIEnd = -EI;
			} else {
				// <----line-----[B==ROD==>A]
				EIEnd = EI;
			}
			break;
		default:
			LOGERR << "Invalid end point qualifier: " << end_point << endl;
			throw moordyn::invalid_value_error("Invalid end point");
	}

	return qEnd * EIEnd / dlEnd;
}

std::pair<std::vector<vec>, std::vector<vec>>
Line::getStateDeriv()
{
	// NOTE:
	// Jose Luis Cercos-Pita: This is by far the most consuming function of the
	// whole library, just because it is called every single time substep and
	// it shall makecomputations in every single line node. Thus it is worthy
	// to invest effort on keeping it optimized.

	// attempting error handling <<<<<<<<
	for (unsigned int i = 0; i <= N; i++) {
		if (isnan(r[i].sum())) {
			stringstream s;
			LOGERR << "NaN detected" << endl << "Line " << number << endl;
			LOGMSG << "node positions:" << endl;
			for (unsigned int j = 0; j <= N; j++)
				LOGMSG << j << " : [" << r[j].transpose() << "]" << endl;
			throw moordyn::nan_error("NaN in node positions");
		}
	}

	// dt is possibly used for stability tricks...

	// -------------------- calculate various kinematic quantities
	// ---------------------------
	for (unsigned int i = 0; i < N; i++) {
		// calculate current (Stretched) segment lengths and unit tangent
		// vectors (qs) for each segment (this is used for bending calculations)
		lstr[i] = unitvector(qs[i], r[i], r[i + 1]);

		// this is the denominator of how the stretch rate equation was
		// formulated
		const double ldstr_top = (r[i + 1] - r[i]).dot(rd[i + 1] - rd[i]);
		ldstr[i] = ldstr_top / lstr[i]; // strain rate of segment

		V[i] = A * l[i]; // volume attributed to segment
	}

	// calculate unit tangent vectors (q) for each internal node. note: I've
	// reversed these from pointing toward 0 rather than N. Check sign of wave
	// loads. <<<<
	for (unsigned int i = 1; i < N; i++)
		unitvector(
		    q[i],
		    r[i - 1],
		    r[i + 1]); // compute unit vector q ... using adjacent two nodes!

	// calculate unit tangent vectors for either end node if the line has no
	// bending stiffness of if either end is pinned (otherwise it's already been
	// set via setEndStateFromRod)
	if ((endTypeA == PINNED) || (EI == 0))
		unitvector(q[0], r[0], r[1]);
	if ((endTypeB == PINNED) || (EI == 0))
		unitvector(q[N], r[N - 1], r[N]);

	//============================================================================================
	// --------------------------------- apply wave kinematics
	// -----------------------------

	if (WaterKin == WAVES_EXTERNAL) // wave kinematics time series set
	                                // internally for each node
	{
		// =========== obtain (precalculated) wave kinematics at current time
		// instant ============ get precalculated wave kinematics at
		// previously-defined node positions for time instant t

		// get interpolation constant and wave time step index
		int it = floor(t / dtWater);
		double frac = remainder(t, dtWater) / dtWater;

		// loop through nodes
		for (unsigned int i = 0; i <= N; i++) {
			zeta[i] =
			    zetaTS[i][it] + frac * (zetaTS[i][it + 1] - zetaTS[i][it]);
			F[i] = 1.0; // FTS[i][it] + frac*(FTS[i][it+1] - FTS[i][it]);

			U[i] = UTS[i][it] + frac * (UTS[i][it + 1] - UTS[i][it]);
			Ud[i] = UdTS[i][it] + frac * (UdTS[i][it + 1] - UdTS[i][it]);
		}
	} else if (WaterKin == WAVES_GRID) {
		// wave kinematics interpolated from global grid in Waves object
		for (unsigned int i = 0; i <= N; i++) {
			// call generic function to get water velocities
			waves->getWaveKin(
			    r[i][0], r[i][1], r[i][2], U[i], Ud[i], zeta[i], PDyn[i]);

			F[i] = 1.0; // set VOF value to one for now (everything submerged -
			            // eventually this should be element-based!!!) <<<<
		}
	} else if (WaterKin !=
	           WAVES_NONE) // Hopefully WaterKin is set to zero, meaning no
	                       // waves or set externally, otherwise it's an error
		cout << "ERROR: We got a problem with WaterKin not being 0,1,2."
		     << endl;

	//============================================================================================

	// calculate mass matrix
	for (unsigned int i = 0; i <= N; i++) {
		real m_i; // node mass
		real v_i; // node submerged volume

		if (i == 0) {
			m_i = pi / 8. * d * d * l[0] * rho;
			v_i = 1. / 2. * F[i] * V[i];
		} else if (i == N) {
			m_i = pi / 8. * d * d * l[N - 2] * rho;
			v_i = 1. / 2. * F[i - 1] * V[i - 1];
		} else {
			m_i = pi / 8. * (d * d * rho * (l[i] + l[i - 1]));
			v_i = 1. / 2. * (F[i - 1] * V[i - 1] + F[i] * V[i]);
		}

		// Make node mass matrix
		const mat I = mat::Identity();
		const mat Q = q[i] * q[i].transpose();
		M[i] = m_i * I + env->rho_w * v_i * (Can * (I - Q) + Cat * Q);
	}

	// ============  CALCULATE FORCES ON EACH NODE
	// ===============================

	// loop through the segments
	for (unsigned int i = 0; i < N; i++) {
		// line tension
		if (nEApoints > 0)
			E = getNonlinearE(lstr[i], l[i]);

		if (lstr[i] / l[i] > 1.0) {
			T[i] = E * A * (1. / l[i] - 1. / lstr[i]) * (r[i + 1] - r[i]);
		} else {
			// cable can't "push" ...
			// or can it, if bending stiffness is nonzero? <<<<<<<<<
			T[i] = vec(0.0, 0.0, 0.0);
		}

		// line internal damping force
		if (nCpoints > 0)
			c = getNonlinearC(ldstr[i], l[i]);

		Td[i] = c * A * (ldstr[i] / l[i]) * (r[i + 1] - r[i]) / lstr[i];
	}

	// Bending loads
	// first zero out the forces from last run
	for (unsigned int i = 0; i <= N; i++)
		Bs[i] = vec(0.0, 0.0, 0.0);

	// and now compute them (if possible)
	if (EI > 0) {
		// loop through all nodes to calculate bending forces
		for (unsigned int i = 0; i <= N; i++) {
			moordyn::real Kurvi = 0.0;
			vec pvec;
			vec Mforce_im1(0.0, 0.0, 0.0);
			vec Mforce_ip1(0.0, 0.0, 0.0);
			vec Mforce_i;

			// calculate force on each node due to bending stiffness!

			// end node A case (only if attached to a Rod, i.e. a cantilever
			// rather than pinned connection)
			if (i == 0) {
				if (endTypeA == CANTILEVERED) // if attached to Rod i.e.
				                              // cantilever connection
				{
					// curvature <<< check if this approximation works for an
					// end (assuming rod angle is node angle which is middle of
					// if there was a segment -1/2
					Kurvi = GetCurvature(lstr[i], q[i], qs[i]);

					// get direction of bending radius axis
					pvec = q[0].cross(qs[i]);

					// get direction of resulting force from bending to apply on
					// node i+1
					Mforce_ip1 = qs[i].cross(pvec);

					// record bending moment at end for potential application to
					// attached object   <<<< do double check this....
					scalevector(pvec, Kurvi * EI, endMomentA);

					// scale force direction vectors by desired moment force
					// magnitudes to get resulting forces on adjacent nodes
					scalevector(Mforce_ip1, Kurvi * EI / lstr[i], Mforce_ip1);

					// set force on node i to cancel out forces on adjacent
					// nodes
					Mforce_i = -Mforce_ip1;

					// apply these forces to the node forces
					Bs[i] = Mforce_i;
					Bs[i + 1] = Mforce_ip1;
				}
			}
			// end node A case (only if attached to a Rod, i.e. a cantilever
			// rather than pinned connection)
			else if (i == N) {
				if (endTypeB == CANTILEVERED) // if attached to Rod i.e.
				                              // cantilever connection
				{
					// curvature <<< check if this approximation
					// works for an end (assuming rod angle is node
					// angle which is middle of if there was a
					// segment -1/2
					Kurvi = GetCurvature(lstr[i - 1], qs[i - 1], q[i]);

					// get direction of bending radius axis
					pvec = qs[i - 1].cross(q[N]);

					// get direction of resulting force from bending to apply on
					// node i-1
					Mforce_im1 = qs[i - 1].cross(pvec);

					// record bending moment at end for potential application to
					// attached object   <<<< do double check this....
					scalevector(
					    pvec,
					    -Kurvi * EI,
					    endMomentB); // note end B is oposite sign as end A

					// scale force direction vectors by desired moment force
					// magnitudes to get resulting forces on adjacent nodes
					scalevector(
					    Mforce_im1, Kurvi * EI / lstr[i - 1], Mforce_im1);

					// set force on node i to cancel out forces on adjacent
					// nodes
					Mforce_i = -Mforce_im1;

					// apply these forces to the node forces
					Bs[i - 1] = Mforce_im1;
					Bs[i] = Mforce_i;
				}
			} else // internal node
			{
				// curvature <<< remember to check
				// sign, or just take abs
				Kurvi = GetCurvature(lstr[i - 1] + lstr[i], qs[i - 1], qs[i]);

				// get direction of bending radius axis
				pvec = qs[i - 1].cross(q[i]);

				// get direction of resulting force from bending to apply on
				// node i-1
				Mforce_im1 = qs[i - 1].cross(pvec);
				// get direction of resulting force from bending to apply on
				// node i+1
				Mforce_ip1 = qs[i].cross(pvec);

				// scale force direction vectors by desired moment force
				// magnitudes to get resulting forces on adjacent nodes
				scalevector(Mforce_im1, Kurvi * EI / lstr[i - 1], Mforce_im1);
				scalevector(Mforce_ip1, Kurvi * EI / lstr[i], Mforce_ip1);

				// set force on node i to cancel out forces on adjacent nodes
				Mforce_i = -Mforce_im1 - Mforce_ip1;

				// apply these forces to the node forces
				Bs[i - 1] = Mforce_im1;
				Bs[i] = Mforce_i;
				Bs[i + 1] = Mforce_ip1;
			}

			// check for NaNs <<<<<<<<<<<<<<< temporary measure <<<<<<<
			if (isnan(Bs[i][0]) || isnan(Bs[i][1]) || isnan(Bs[i][2])) {
				cout << "   Error: NaN value detected in bending force at Line "
				     << number << " node " << i << endl;
				cout << lstr[i - 1] + lstr[i] << endl;
				cout << sqrt(0.5 * (1 - qs[i - 1].dot(qs[i]))) << endl;

				cout << Bs[i - 1] << endl;
				cout << Bs[i] << endl;
				cout << Bs[i + 1] << endl;
				cout << Mforce_im1 << endl;
				cout << Mforce_i << endl;
				cout << Mforce_ip1 << endl;
			}

			// record curvature at node!!
			Kurv[i] = Kurvi;

			// any damping forces for bending? I hope not...

			// get normal component at each adjacent node

			// trace along line to find torsion at each segment
			/*
			if (torsion)
			{
			    // assume first segment's twist coordinate is fixed
			    if (i==0)
			        s[0] = {0,0,1}; //or something
			    else  // i =1..N-1
			    {
			        // R x = u(u . x) + cos(alpha) (u cross x) cross u +
			sin(alpha) (u cross x)
			        // \uvec{s}_\iplus = \uvec{p}_i(\uvec{p}_i \cdot
			\uvec{s}_\iminus)
			        // + (\uvec{q}_\iplus \cdot \uvec{q}_\iminus) (\uvec{p}_i
			\times \uvec{s}_\iminus) \times \uvec{p}_i
			        // + \sin(\alpha_i) (\uvec{p}_i \times \uvec{s}_\iminus)

			        double p_p_dot_s[3];
			        scalevector( pvec, dotprod(pvec, s[i-1]), p_p_dot_s)  //
			\uvec{p}_i(\uvec{p}_i \cdot \uvec{s}_\iminus)

			        double p_cross_s[3];
			        crossprod(pvec, s[i-1], p_cross_s)// \uvec{p}_i \times
			\uvec{s}_\iminus

			        double p_cross_s_cross_p[3];
			        crossprod(p_cross_s, pvec, p_cross_s_cross_p)// (\uvec{p}_i
			\times \uvec{s}_\iminus) \times \uvec{p}_i

			        for j in  <3)   // \uvec{s}_\iplus = \uvec{p}_i(\uvec{p}_i
			\cdot \uvec{s}_\iminus) + (\uvec{q}_\iplus \cdot \uvec{q}_\iminus)
			pcx \times \uvec{p}_i + \sin(\alpha_i) pcx
			        {
			            s[i][j] = p_p_dot_s[j] + cos_alpha*p_cross_s_cross_p[j]
			+ sin_alpha*p_cross_s
			        }
			    }
			    // compare end alignment with torsion calculation to figure out
			actual cable twist, and distribute it uniformly along the cable ..
			but how?

			}
			*/
		} // for i=0,N (looping through nodes)
	}     // if EI > 0

	// loop through the nodes
	for (unsigned int i = 0; i <= N; i++) {
		W[i][0] = W[i][1] = 0.0;
		// submerged weight (including buoyancy)
		if (i == 0)
			W[i][2] = 0.5 * A * (l[i] * (rho - F[i] * env->rho_w)) * (-env->g);
		else if (i == N)
			W[i][2] = 0.5 * A * (l[i - 1] * (rho - F[i - 1] * env->rho_w)) *
			          (-env->g); // missing the "W[i][2] =" previously!
		else
			W[i][2] = 0.5 * A *
			          (l[i] * (rho - F[i] * env->rho_w) +
			           l[i - 1] * (rho - F[i - 1] * env->rho_w)) *
			          (-env->g);

		// relative flow velocity over node
		const vec vi = U[i] - rd[i];
		// tangential relative flow component
		// <<<<<<< check sign since I've reversed q
		const moordyn::real vql = vi.dot(q[i]);
		const vec vq = vql * q[i];
		// transverse relative flow component
		const vec vp = vi - vq;

		const moordyn::real vq_mag = vq.norm();
		const moordyn::real vp_mag = vp.norm();

		// transverse drag
		if (i == 0)
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d * (F[i] * l[i]) * vp;
		else if (i == N)
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d *
			        (F[i - 1] * l[i - 1]) * vp;
		else
			Dp[i] = 0.25 * vp_mag * env->rho_w * Cdn * d *
			        (F[i] * l[i] + F[i - 1] * l[i - 1]) * vp;

		// tangential drag
		if (i == 0)
			Dq[i] =
			    0.25 * vq_mag * env->rho_w * Cdt * pi * d * (F[i] * l[i]) * vq;
		else if (i == N)
			Dq[i] = 0.25 * vq_mag * env->rho_w * Cdt * pi * d *
			        (F[i - 1] * l[i - 1]) * vq;
		else
			Dq[i] = 0.25 * vq_mag * env->rho_w * Cdt * pi * d *
			        (F[i] * l[i] + F[i - 1] * l[i - 1]) * vq;

		// tangential component of fluid acceleration
		// <<<<<<< check sign since I've reversed q
		const moordyn::real aql = Ud[i].dot(q[i]);
		const vec aq = aql * q[i];
		// normal component of fluid acceleration
		const vec ap = Ud[i] - aq;

		// transverse Froude-Krylov force
		if (i == 0)
			Ap[i] = env->rho_w * (1. + Can) * 0.5 * (V[i]) * ap;
		else if (i == N)
			Ap[i] = env->rho_w * (1. + Can) * 0.5 * (V[i - 1]) * ap;
		else
			Ap[i] = env->rho_w * (1. + Can) * 0.5 * (V[i] + V[i - 1]) * ap;
		// tangential Froude-Krylov force
		if (i == 0)
			Aq[i] = 0.5 * env->rho_w * (1. + Cat) * 0.5 * (V[i]) * aq;
		else if (i == N)
			Aq[i] = 0.5 * env->rho_w * (1. + Cat) * 0.5 * (V[i - 1]) * aq;
		else
			Aq[i] = env->rho_w * (1. + Cat) * 0.5 * (V[i] + V[i - 1]) * aq;

		// bottom contact (stiffness and damping, vertical-only for now) -
		// updated for general case of potentially anchor or fairlead end in
		// contact
		if (r[i][2] < -env->WtrDpth) {
			if (i == 0)
				B[i][2] =
				    ((-env->WtrDpth - r[i][2]) * env->kb - rd[i][2] * env->cb) *
				    0.5 * d * (l[i]);
			else if (i == N)
				B[i][2] =
				    ((-env->WtrDpth - r[i][2]) * env->kb - rd[i][2] * env->cb) *
				    0.5 * d * (l[i - 1]);
			else
				B[i][2] =
				    ((-env->WtrDpth - r[i][2]) * env->kb - rd[i][2] * env->cb) *
				    0.5 * d * (l[i - 1] + l[i]);

			// new rough-draft addition of seabed friction
			real FrictionMax =
			    abs(B[i][2]) *
			    env->FrictionCoefficient; // dynamic friction force saturation
			                              // level based on bottom contact force

			// saturated damping approach to applying friction, for now
			real BottomVel =
			    sqrt(rd[i][0] * rd[i][0] +
			         rd[i][1] * rd[i][1]); // velocity of node along sea bed
			real FrictionForce =
			    BottomVel * env->FrictionCoefficient *
			    env->FricDamp; // some arbitrary damping scaling thing at end
			if (FrictionForce > env->StatDynFricScale * FrictionMax)
				FrictionForce =
				    FrictionMax; // saturate (quickly) to static/dynamic
				                 // friction force level

			if (BottomVel == 0.0) { // check for zero velocity, in which case
				                    // friction force is zero
				B[i][0] = 0.0;
				B[i][1] = 0.0;
			} else { // otherwise, apply friction force in correct
				     // direction(opposing direction of motion)
				B[i][0] = -FrictionForce * rd[i][0] / BottomVel;
				B[i][1] = -FrictionForce * rd[i][1] / BottomVel;
			}
		} else
			B[i] = vec(0.0, 0.0, 0.0);

		// total forces
		if (i == 0)
			Fnet[i] = T[i] + Td[i];
		else if (i == N)
			Fnet[i] = -T[i - 1] - Td[i - 1];
		else
			Fnet[i] = T[i] - T[i - 1] + Td[i] - Td[i - 1];
		Fnet[i] += W[i] + (Dp[i] + Dq[i] + Ap[i] + Aq[i]) + B[i] + Bs[i];
	}

	//	if (t > 5)
	//	{
	//		cout << " in getStateDeriv of line " << number << endl;
	//
	//		B[0][0] = 0.001; // meaningless
	//	}

	// loop through internal nodes and compute the accelerations
	std::vector<vec> u, a;
	u.reserve(N - 1);
	a.reserve(N - 1);
	for (unsigned int i = 1; i < N; i++) {
		//	double M_out[9];
		//	double F_out[3];
		//	for (int I=0; I<3; I++)
		//	{	F_out[I] = Fnet[i][I];
		//		for (int J=0; J<3; J++) M_out[3*I + J] = M[i][I][J];
		//	}

		// solve for accelerations in [M]{a}={f} using LU decomposition
		//	double LU[9];                        // serialized matrix that will
		// hold LU matrices combined 	Crout(3, M_out, LU);                  //
		// perform LU decomposition on mass matrix 	double acc[3]; //
		// acceleration vector to solve for 	solveCrout(3, LU, F_out, acc);
		// // solve for acceleration vector

		//	LUsolve3(M[i], acc, Fnet[i]);

		//	Solve3(M[i], acc, (const double*)Fnet[i]);

		// For small systems it is usually faster to compute the inverse
		// of the matrix. See
		// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		a.push_back(M[i].inverse() * Fnet[i]);
		u.push_back(rd[i]);
	}

	return make_pair(u, a);
};

// write output file for line  (accepts time parameter since retained time value
// (t) will be behind by one line time step
void
Line::Output(real time)
{
	// run through output flags
	// if channel is flagged for output, write to file.
	// Flags changed to just be one character (case sensitive) per output flag.
	// To match FASTv8 version.

	if (outfile) // if not a null pointer (indicating no output)
	{
		if (!outfile->is_open()) {
			LOGWRN << "Unable to write to output file " << endl;
			return;
		}
		// output time
		*outfile << time << "\t ";

		// output positions?
		// if (find(channels.begin(), channels.end(), "position") !=
		// channels.end())
		if (channels.find("p") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) // loop through nodes
			{
				for (unsigned int J = 0; J < 3; J++)
					*outfile << r[i][J] << "\t ";
			}
		}
		// output curvatures?
		if (channels.find("K") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << Kurv[i] << "\t ";
			}
		}
		// output velocities?
		if (channels.find("v") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << rd[i][J] << "\t ";
			}
		}
		// output wave velocities?
		if (channels.find("U") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << U[i][J] << "\t ";
			}
		}
		// output hydro drag force?
		if (channels.find("D") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << Dp[i][J] + Dq[i][J] + Ap[i][J] + Aq[i][J]
					         << "\t ";
			}
		}
		// output segment tensions?
		if (channels.find("t") != string::npos) {
			for (unsigned int i = 0; i < N; i++) {
				*outfile << T[i].norm() << "\t ";
				// >>> preparation below for switching to outputs at nodes
				// <<< note that tension of end nodes will need weight and
				// buoyancy adjustment
				// if (i==0)
				//      *outfile << (T[i] + W[i]).norm() << "\t ";
				// else if (i==N)
				//      *outfile << (T[i] - W[i]).norm() << "\t ";
				// else
				//	*outfile << T[i].norm() << "\t ";
			}
		}
		// output internal damping force?
		if (channels.find("c") != string::npos) {
			for (unsigned int i = 0; i < N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << Td[i][J] + Td[i][J] + Td[i][J] << "\t ";
			}
		}
		// output segment strains?
		if (channels.find("s") != string::npos) {
			for (unsigned int i = 0; i < N; i++) {
				*outfile << lstr[i] / l[i] - 1.0 << "\t ";
			}
		}
		// output segment strain rates?
		if (channels.find("d") != string::npos) {
			for (unsigned int i = 0; i < N; i++) {
				*outfile << ldstr[i] / l[i] << "\t ";
			}
		}
		// output seabed contact forces?
		if (channels.find("b") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << B[i][J] << "\t ";
			}
		}

		*outfile << "\n";
	}
	return;
};

std::vector<uint64_t>
Line::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	data.push_back(io::IO::Serialize(t));
	subdata = io::IO::Serialize(r);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(q);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(qs);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(l);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(lstr);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(ldstr);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Kurv);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(M);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(V);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(T);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Td);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Bs);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(W);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Dp);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Dq);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Ap);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Aq);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(B);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Fnet);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(F);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(zeta);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(PDyn);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(U);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Ud);
	data.insert(data.end(), subdata.begin(), subdata.end());

	return data;
}

uint64_t*
Line::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	ptr = io::IO::Deserialize(ptr, t);
	ptr = io::IO::Deserialize(ptr, r);
	ptr = io::IO::Deserialize(ptr, rd);
	ptr = io::IO::Deserialize(ptr, q);
	ptr = io::IO::Deserialize(ptr, qs);
	ptr = io::IO::Deserialize(ptr, l);
	ptr = io::IO::Deserialize(ptr, lstr);
	ptr = io::IO::Deserialize(ptr, ldstr);
	ptr = io::IO::Deserialize(ptr, Kurv);
	ptr = io::IO::Deserialize(ptr, M);
	ptr = io::IO::Deserialize(ptr, V);
	ptr = io::IO::Deserialize(ptr, T);
	ptr = io::IO::Deserialize(ptr, Td);
	ptr = io::IO::Deserialize(ptr, Bs);
	ptr = io::IO::Deserialize(ptr, W);
	ptr = io::IO::Deserialize(ptr, Dp);
	ptr = io::IO::Deserialize(ptr, Dq);
	ptr = io::IO::Deserialize(ptr, Ap);
	ptr = io::IO::Deserialize(ptr, Aq);
	ptr = io::IO::Deserialize(ptr, B);
	ptr = io::IO::Deserialize(ptr, Fnet);
	ptr = io::IO::Deserialize(ptr, F);
	ptr = io::IO::Deserialize(ptr, zeta);
	ptr = io::IO::Deserialize(ptr, PDyn);
	ptr = io::IO::Deserialize(ptr, U);
	ptr = io::IO::Deserialize(ptr, Ud);

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkPolyData>
Line::getVTK() const
{
	auto points = vtkSmartPointer<vtkPoints>::New();
	auto line = vtkSmartPointer<vtkPolyLine>::New();
	// Node fields, i.e. r.size() number of tuples
	auto vtk_rd = io::vtk_farray("rd", 3, r.size());
	auto vtk_Kurv = io::vtk_farray("Kurv", 1, r.size());
	auto vtk_M = io::vtk_farray("M", 9, r.size());
	auto vtk_Fnet = io::vtk_farray("Fnet", 3, r.size());
	// Segment fields, i.e. r.size()-1 number of tuples
	auto vtk_lstr = io::vtk_farray("lstr", 1, r.size() - 1);
	auto vtk_ldstr = io::vtk_farray("ldstr", 1, r.size() - 1);
	auto vtk_V = io::vtk_farray("V", 1, r.size() - 1);

	line->GetPointIds()->SetNumberOfIds(r.size());
	for (unsigned int i = 0; i < r.size(); i++) {
		points->InsertNextPoint(r[i][0], r[i][1], r[i][2]);
		line->GetPointIds()->SetId(i, i);
		vtk_rd->SetTuple3(i, rd[i][0], rd[i][1], rd[i][2]);
		vtk_Kurv->SetTuple1(i, Kurv[i]);
		vtk_M->SetTuple9(i,
		                 M[i](0, 0),
		                 M[i](0, 1),
		                 M[i](0, 2),
		                 M[i](1, 0),
		                 M[i](1, 1),
		                 M[i](1, 2),
		                 M[i](2, 0),
		                 M[i](2, 1),
		                 M[i](2, 2));
		vtk_Fnet->SetTuple3(i, Fnet[i][0], Fnet[i][1], Fnet[i][2]);
		if (i == r.size() - 1)
			continue;
		vtk_lstr->SetTuple1(i, lstr[i]);
		vtk_ldstr->SetTuple1(i, ldstr[i]);
		vtk_V->SetTuple1(i, V[i]);
	}
	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(line);

	auto out = vtkSmartPointer<vtkPolyData>::New();
	out->SetPoints(points);
	out->SetLines(cells);

	out->GetCellData()->AddArray(vtk_lstr);
	out->GetCellData()->AddArray(vtk_ldstr);
	out->GetCellData()->AddArray(vtk_V);
	out->GetCellData()->SetActiveScalars("ldstr");

	out->GetPointData()->AddArray(vtk_rd);
	out->GetPointData()->AddArray(vtk_Kurv);
	out->GetPointData()->AddArray(vtk_M);
	out->GetPointData()->AddArray(vtk_Fnet);
	out->GetPointData()->SetActiveVectors("Fnet");

	return out;
}

void
Line::saveVTK(const char* filename) const
{
	auto obj = this->getVTK();
	auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	writer->SetFileName(filename);
	writer->SetInputData(obj);
	writer->SetDataModeToBinary();
	writer->Update();
	writer->Write();
	auto err = io::vtk_error(writer->GetErrorCode());
	if (err != MOORDYN_SUCCESS) {
		LOGERR << "VTK reported an error while writing the VTP file '"
		       << filename << "'" << endl;
		MOORDYN_THROW(err, "vtkXMLPolyDataWriter reported an error");
	}
}
#endif

// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void
Line::drawGL(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i = 0; i <= N; i++) {
		double newTen = getNodeTen(i).norm();
		if (newTen > maxTen)
			maxTen = newTen;
	}

	glColor3f(0.5, 0.5, 1.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= N; i++) {
		glVertex3d(r[i][0], r[i][1], r[i][2]);
		if (i < N) {
			normTen = getNodeTen(i).norm() / maxTen;
			ColorMap(normTen, rgb);
			glColor3d(rgb[0], rgb[1], rgb[2]);
		}
	}
	glEnd();
};

void
Line::drawGL2(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i = 0; i <= N; i++) {
		double newTen = getNodeTen(i).norm();
		if (newTen > maxTen)
			maxTen = newTen;
	}

	// line
	for (unsigned int i = 0; i < N; i++) {
		normTen = 0.2 + 0.8 * pow(getNodeTen(i).norm() / maxTen, 4.0);
		ColorMap(normTen, rgb);
		glColor3d(rgb[0], rgb[1], rgb[2]);

		Cylinder(r[i][0],
		         r[i][1],
		         r[i][2],
		         r[i + 1][0],
		         r[i + 1][1],
		         r[i + 1][2],
		         27,
		         0.5);
	}
	// velocity vectors
	for (int i = 0; i <= N; i++) {
		glColor3d(0.0, 0.2, 0.8);
		double vscal = 5.0;

		Arrow(r[i][0],
		      r[i][1],
		      r[i][2],
		      vscal * rd[i][0],
		      vscal * rd[i][1],
		      vscal * rd[i][2],
		      0.1,
		      0.7);
	}
};
#endif

} // ::moordyn

// =============================================================================
//
//                     ||                     ||
//                     ||        C API        ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

/// Check that the provided line is not Null
#define CHECK_LINE(s)                                                          \
	if (!s) {                                                                  \
		cerr << "Null line received in " << __FUNC_NAME__ << " ("              \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetLineID(MoorDynLine l, int* id)
{
	CHECK_LINE(l);
	*id = ((moordyn::Line*)l)->number;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineN(MoorDynLine l, unsigned int* n)
{
	CHECK_LINE(l);
	*n = ((moordyn::Line*)l)->getN();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineNumberNodes(MoorDynLine l, unsigned int* n)
{
	int err = MoorDyn_GetLineN(l, n);
	if (err != MOORDYN_SUCCESS)
		return err;
	*n += 1;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineUnstretchedLength(MoorDynLine l, double* ul)
{
	CHECK_LINE(l);
	*ul = ((moordyn::Line*)l)->getUnstretchedLength();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLineUnstretchedLength(MoorDynLine l, double ul)
{
	CHECK_LINE(l);
	((moordyn::Line*)l)->setUnstretchedLength((const moordyn::real)ul);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLineUnstretchedLengthVel(MoorDynLine l, double v)
{
	CHECK_LINE(l);
	((moordyn::Line*)l)->setUnstretchedLengthVel((const moordyn::real)v);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineNodePos(MoorDynLine l, unsigned int i, double pos[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec r = ((moordyn::Line*)l)->getNodePos(i);
		moordyn::vec2array(r, pos);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeTen(MoorDynLine l, unsigned int i, double ten[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec t = ((moordyn::Line*)l)->getNodeTen(i);
		moordyn::vec2array(t, ten);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeCurv(MoorDynLine l, unsigned int i, double* curv)
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const auto c = ((moordyn::Line*)l)->getNodeCurv(i);
		*curv = c;
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineFairTen(MoorDynLine l, double* t)
{
	CHECK_LINE(l);
	*t = ((moordyn::Line*)l)->getNodeTen(((moordyn::Line*)l)->getN()).norm();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineMaxTen(MoorDynLine l, double* t)
{
	CHECK_LINE(l);
	double t_max = 0.0;
	for (unsigned int i = 0; i < ((moordyn::Line*)l)->getN(); i++) {
		const double t_candidate = ((moordyn::Line*)l)->getNodeTen(i).norm();
		if (t_candidate > t_max)
			t_max = t_candidate;
	}
	*t = t_max;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SaveLineVTK(MoorDynLine l, const char* filename)
{
#ifdef USE_VTK
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::Line*)l)->saveVTK(filename);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
#else
	cerr << "MoorDyn has been built without VTK support, so " << __FUNC_NAME__
	     << " (" << XSTR(__FILE__) << ":" << __LINE__
	     << ") cannot save the file '" << filename << "'" << endl;
	return MOORDYN_NON_IMPLEMENTED;
#endif
}
