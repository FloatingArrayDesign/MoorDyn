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
#include "Util/Interp.hpp"
#include <tuple>
// #include <random>

#ifdef USE_VTK
#include "Util/VTK_Util.hpp"
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLPolyDataWriter.h>
#endif

using namespace std;

namespace moordyn {
using namespace waves;

Line::Line(moordyn::Log* log, size_t lineId)
  : io::IO(log)
  , lineId(lineId)
  , isPb(false)
{
}

Line::~Line() {}

real
Line::getNonlinearEA(real l_stretched, real l_unstretched) const
{
	if (!nEApoints)
		return EA;

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
Line::getNonlinearEI(real curv) const
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
Line::getNonlinearBA(real ld_stretched, real l_unstretched) const
{
	if (!nBApoints)
		return BA;

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
            EnvCondRef env_in,
            shared_ptr<ofstream> outfile_pointer,
            string channels_in,
			real dtM0)
{
	env = env_in; // set pointer to environment settings object
	number = number_in;
	// Note, this is a temporary value that will be processed depending on sign
	// during initializeLine
	UnstrLen = UnstrLen_in;
	UnstrLend = 0.0;
	// assign number of nodes to line
	N = NumSegs;
	// save the moordyn internal timestep
	dtm = dtM0;

	// store passed line properties (and convert to numbers)
	d = props->d;
	A = pi / 4. * d * d;
	rho = props->w / A;
	ElasticMod = props->ElasticMod;
	EA = props->EA;
	EA_D = props->EA_D;
	alphaMBL = props->alphaMBL;
	vbeta = props->vbeta;
	EI = props->EI;
	BAin = props->BA;
	BA_D = props->BA_D;
	Can = props->Can;
	Cat = props->Cat;
	Cdn = props->Cdn;
	Cdt = props->Cdt;
	Cl = props->Cl;
	dF = props->dF;
	cF = props->cF;

	// copy in nonlinear stress-strain data if applicable
	stiffXs.clear();
	stiffYs.clear();
	nEApoints = props->nEApoints;
	for (unsigned int I = 0; I < nEApoints; I++) {
		stiffXs.push_back(props->stiffXs[I]);
		stiffYs.push_back(props->stiffYs[I]);
	}

	// Use the last entry on the lookup table. see Line::initialize()
	const real EA = nEApoints ? stiffYs.back() / stiffXs.back() : props->EA;
	NatFreqCFL::length(UnstrLen / N);
	NatFreqCFL::stiffness(EA * N / UnstrLen);
	NatFreqCFL::mass(props->w * UnstrLen / N);

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
	nBApoints = props->nBApoints;
	for (unsigned int I = 0; I < nBApoints; I++) {
		dampXs.push_back(props->dampXs[I]);
		dampYs.push_back(props->dampYs[I]);
	}

	// Initialize API provided info
	isPb = false;

	// ------------------------- size vectors -------------------------

	pin.assign(N + 1, 0.0);           // Internal pressure at node points (Pa)

	r.assign(N + 1, vec::Zero());     // node positions [i][x/y/z]
	rd.assign(N + 1, vec::Zero());    // node velocities [i][x/y/z]
	rdd_old.assign(N + 1, vec::Zero()); // node accelerations previous iteration [i][x/y/z]
	Misc.assign(N+1, vec::Zero()); // node misc states [i][viv phase/viscoelastic/unused]
	Miscd.assign(N+1, vec::Zero()); // node misc state derivatives [i][viv phase/viscoelastic/unused]
	q.assign(N + 1, vec::Zero());     // unit tangent vectors for each node
	pvec.assign(N + 1, vec::Zero());  // unit normal vectors for each node
	qs.assign(N, vec::Zero());        // unit tangent vectors for each segment
	l.assign(N, 0.0);                 // line unstretched segment lengths
	lstr.assign(N, 0.0);              // stretched lengths
	ldstr.assign(N, 0.0);             // rate of stretch
	Kurv.assign(N + 1, 0.0);          // curvatures at node points (1/m)

	M.assign(N + 1, mat::Zero());     // mass matrices (3x3) for each node
	V.assign(N, 0.0);                 // segment volume?

	// forces
	T.assign(N, vec::Zero());        // segment tensions
	Td.assign(N, vec::Zero());       // segment damping forces
	Bs.assign(N + 1, vec::Zero());   // bending stiffness forces
	Pb.assign(N + 1, vec::Zero());   // Pressure bending forces
	W.assign(N + 1, vec::Zero());    // node weights
	Dp.assign(N + 1, vec::Zero());   // node drag (transverse)
	Dq.assign(N + 1, vec::Zero());   // node drag (axial)
	Ap.assign(N + 1, vec::Zero());   // node added mass forcing (transverse)
	Aq.assign(N + 1, vec::Zero());   // node added mass forcing (axial)
	B.assign(N + 1, vec::Zero());    // node bottom contact force
	Lf.assign(N + 1, vec::Zero());    // viv crossflow lift force
	Fnet.assign(N + 1, vec::Zero()); // total force on node

	// wave things
	F.assign(N + 1, 0.0); // VOF scaler for each NODE (mean of two half adjacent
	                      // segments) (1 = fully submerged, 0 = out of water)

	// Back indexing things for VIV (amplitude disabled, only needed if lift coefficient table is added)
	// A_int_old.assign(N + 1, 0.0); // running amplitude total, from previous zero crossing of yd
	// Amp.assign(N + 1, 0.0); // VIV Amplitude updated every zero crossing of crossflow velcoity
	yd_rms_old.assign(N + 1, 0.0); // node old yd_rms
	ydd_rms_old.assign(N + 1, 0.0);// node old ydd_rms

	// ensure end moments start at zero
	endMomentA = vec::Zero();
	endMomentB = vec::Zero();

	// set the number of preset wave kinematic time steps to zero (flagging
	// disabled) to start with
	ntWater = 0;

	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); // make outfile point to the right place
	channels = channels_in;          // copy string of output channels to object

	LOGDBG << "   Set up Line " << number << ". " << endl;
};

std::tuple<std::vector<vec>, std::vector<vec>, std::vector<vec>>
Line::initialize()
{
	LOGMSG << "  - Line" << number << ":" << endl
	       << "    ID      : " << number << endl
	       << "    UnstrLen: " << UnstrLen << endl
	       << "    N       : " << N << endl
	       << "    d       : " << d << endl
	       << "    rho     : " << rho << endl
		   << "    EAMod   : " << ElasticMod << endl
	       << "    EA      : " << EA << endl
		   << "    BA      : " << BAin << endl
	       << "    EI      : " << EI << endl
	       << "    Can     : " << Can << endl
	       << "    Cat     : " << Cat << endl
	       << "    Cdn     : " << Cdn << endl
	       << "    Cdt     : " << Cdt << endl
		   << "    Cl      : " << Cl << endl
		   << "    dF      : " << dF << endl
		   << "    cF      : " << cF << endl
	       << "    ww_l    : " << ((rho - env->rho_w) * (pi / 4. * d * d)) * 9.81
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
		// output VIV lift force
		if (channels.find("V") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "Vx \t Node" << i << "Vy \t Node"
				         << i << "Vz \t ";
			}
		}
		// output segment tensions
		if (channels.find("t") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "Te \t ";
			}
		}
		// output internal damping force
		if (channels.find("c") != string::npos) {
			for (unsigned int i = 1; i <= N; i++) {
				*outfile << "Seg" << i << "cx \t Seg" << i << "cy \t Seg" << i
				         << "cz \t ";
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
			// output VIV force
			if (channels.find("V") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(N) \t";
			}
			// output segment tensions?
			if (channels.find("t") != string::npos) {
				for (unsigned int i = 0; i < N; i++)
					*outfile << "(N) \t";
			}
			// output internal damping force?
			if (channels.find("c") != string::npos) {
				for (unsigned int i = 0; i < 3 * N; i++)
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
			if (channels.find("b") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(N) \t";
			}

			*outfile << "\n";
		}
	}

	// The end node kinematics should already have been set by the
	// corresponding Point or Rod objects calling "setEndState",
	// so now we can proceed with figuring out the positions of the nodes along
	// the line.
	if (getWaterDepth(r[0][0], r[0][1]) > r[0][2]) {
		LOGERR << "Water depth is shallower than Line " << number << " anchor"
		       << endl;
		throw moordyn::invalid_value_error("Invalid water depth");
	}

	// TODO - determine if F should be for each segment or for each node,
	// currently it's actually for segments
	F.assign(N + 1, 1.0);

	// process unstretched line length input
	vec dir = r[N] - r[0];
	if (UnstrLen < 0) {
		// Interpret as scaler relative to distance between initial line end
		// points (which have now been set by the relevant Point objects)
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
		EA = stiffYs.back() / stiffXs.back();
	}

	// process internal damping input
	if (BAin < 0) {
		// automatic internal damping option (if negative BA provided (stored as
		// BAin), then -BAin indicates desired damping ratio [unitless])
		BA = -BAin * UnstrLen / N * sqrt(EA * rho * A); // rho = w/A. Units: no unit * m * sqrt(kg-m/s^2 *kg/m) = Ns 
		LOGMSG << "Line " << number << " damping set to " << BA / A
		       << " Pa-s = " << BA << " Ns, based on input of " << BAin
		       << endl;
	} else {
		// otherwise it's the regular internal damping coefficient, which should
		// be divided by area to get a material coefficient
		BA = BAin;
	}

	// initialize line node positions as distributed linearly between the
	// endpoints
	for (unsigned int i = 1; i < N; i++) {
		r[i] = r[0] + dir * (i / (real)N);
	}

	// if conditions are ideal, try to calculate initial line profile using
	// catenary routine (from FAST v.7)

	real XF = dir(Eigen::seqN(0, 2)).norm(); // horizontal spread
	if (XF > 0.0001) { // tolerance for calculation of XF when points are not along x or y axis
		// Check if the line touches the seabed, so we are modelling it. Just
		// the end points are checked
		const real Tol = 1e-5;
		real CB = -1.0;
		for (unsigned int i = 0; i <= N; i += N) {
			const real waterDepth = getWaterDepth(r[i][0], r[i][1]);
			if(r[i][2] <= waterDepth * (1.0 - Tol))
				CB = 0.0;
		}
		const real ZF = dir[2];
		const real LW = ((rho - env->rho_w) * A) * env->g;

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
		                       EA,
		                       LW,
		                       CB,
		                       Tol,
		                       &HF,
		                       &VF,
		                       &HA,
		                       &VA,
		                       N+1,
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
			       << ", initalizing as linear " << endl;
		}
	} else {
		LOGDBG << "Vertical linear initial profile for Line " << number << endl;
	}

	// also assign the resulting internal node positions to the integrator
	// initial state vector! (velocities leave at 0)
	std::vector<vec> vel(N - 1, vec::Zero());
	// inital viv state vector (phase,amplitude,smoothed amplitude)
	std::vector<vec> misc(N+1, vec::Zero());
	/// give a random distribution between 0 and 2pi for inital phase of lift force to avoid initial transient
	if (Cl > 0.0) {
		std::vector<moordyn::real> phase_range(N+1);
		for (unsigned int i = 0; i < N+1; i++) phase_range[i] = (i/moordyn::real(N))*2*pi;
		// shuffle(phase_range.begin(),phase_range.end(),random_device());
		for (unsigned int i = 0; i < N+1; i++) misc[i][0] = phase_range[i];
	}

	LOGMSG << "Initialized Line " << number << endl;

	return std::make_tuple(vector_slice(r, 1, N - 1), vel, misc);
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
	else if (outChan.QType == Ten) {
		if ((outChan.NodeID == 0) || (outChan.NodeID == (int)N))
			return getNodeForce(outChan.NodeID).norm();
		return getNodeTen(outChan.NodeID).norm();
	}
	else if (outChan.QType == TenA)
		return getNodeForce(0).norm();
	else if (outChan.QType == TenB)
		return getNodeForce(N).norm();
	else if (outChan.QType == FX)
		return getNodeForce(outChan.NodeID)[0];
	else if (outChan.QType == FY)
		return getNodeForce(outChan.NodeID)[1];
	else if (outChan.QType == FZ)
		return getNodeForce(outChan.NodeID)[2];
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

	ntWater = static_cast<unsigned int>(zeta_in[0].size());
	dtWater = dt;

	LOGDBG << "Setting up wave variables for Line " << number
	       << "!  ---------------------" << endl
	       << "   nt=" << ntWater << ", and WaveDT=" << dtWater
	       << ", average water depth=" << avgWaterDepth() << endl;

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
}

real
Line::calcSubSeg(unsigned int firstNodeIdx,
                 unsigned int secondNodeIdx,
                 real surface_height)
{
	const real firstNodeZ = r[firstNodeIdx][2] - surface_height;
	const real secondNodeZ = r[secondNodeIdx][2] - surface_height;
	if (firstNodeZ <= 0.0 && secondNodeZ < 0.0) {
		return 1.0; // Both nodes below water; segment must be too
	} else if (firstNodeZ > 0.0 && secondNodeZ > 0.0) {
		return 0.0; // Both nodes above water; segment must be too
	} else if (firstNodeZ == -secondNodeZ) {
		return 0.5; // Segment halfway submerged
	} else {
		// Segment partially submerged - figure out which node is above water
		vec lowerEnd = firstNodeZ < 0.0 ? r[firstNodeIdx] : r[secondNodeIdx];
		vec upperEnd = firstNodeZ < 0.0 ? r[secondNodeIdx] : r[firstNodeIdx];
		lowerEnd.z() -= surface_height;
		upperEnd.z() -= surface_height;

		// segment submergence is calculated by calculating submergence of
		// hypotenuse across segment from upper corner to lower corner
		// To calculate this, we need the coordinates of these corners.
		// first step is to get vector from lowerEnd to upperEnd
		vec segmentAxis = upperEnd - lowerEnd;

		// Next, find normal vector in z-plane, i.e. the normal vecto that
		// points "up" the most. See the following stackexchange:
		// https://math.stackexchange.com/questions/2283842/
		vec upVec(0, 0, 1); // the global up-unit vector
		vec normVec = segmentAxis.cross(upVec.cross(segmentAxis));
		normVec.normalize();

		// make sure normal vector has length equal to radius of segment
		real radius = d / 2;
		scalevector(normVec, radius, normVec);

		// Calculate and return submerged ratio:
		lowerEnd = lowerEnd - normVec;
		upperEnd = upperEnd + normVec;

		return fabs(lowerEnd[2]) / (fabs(lowerEnd[2]) + upperEnd[2]);
	}
}

void
Line::setPin(std::vector<real> p)
{
	if (p.size() != pin.size()) {
		LOGERR << "Invalid input size" << endl;
		throw moordyn::invalid_value_error("Invalid input size");
	}
	pin = p;
}

void
Line::setState(const std::vector<vec>& pos, const std::vector<vec>& vel, const std::vector<vec>& misc)
{
	if ((pos.size() != N - 1) || (vel.size() != N - 1) || (misc.size() != N + 1)) {
		LOGERR << "Invalid input size" << endl;
		throw moordyn::invalid_value_error("Invalid input size");
	}

	// set interior node positions and velocities and all misc states based on state vector
	std::copy(pos.begin(), pos.end(), r.begin() + 1);
	std::copy(vel.begin(), vel.end(), rd.begin() + 1);
	std::copy(misc.begin(), misc.end(), Misc.begin() + 1); // Misc[i][0,1,2] correspond to viv phase, viscoelastic, unused.
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
	real dlEnd, non_linear_EI, EIEnd;
	vec qEnd;

	if ((rod_end_point != ENDPOINT_A) && (rod_end_point != ENDPOINT_B)) {
		LOGERR << "Invalid rod end point qualifier: " << rod_end_point << endl;
		throw moordyn::invalid_value_error("Invalid end point");
	}
	switch (end_point) {
		case ENDPOINT_TOP:
			// unit vector of last line segment
			dlEnd = unitvector(qEnd, r[N - 1], r[N]);
			non_linear_EI = nEIpoints ? getNonlinearEI(Kurv[N]) : EI;
			if (rod_end_point == ENDPOINT_A) {
				// -----line----->[A==ROD==>B]
				EIEnd = non_linear_EI;
			} else {
				// -----line----->[B==ROD==>A]
				EIEnd = -non_linear_EI;
			}
			break;
		case ENDPOINT_BOTTOM:
			// unit vector of first line segment
			dlEnd = unitvector(qEnd, r[0], r[1]);
			non_linear_EI = nEIpoints ? getNonlinearEI(Kurv[0]) : EI;
			if (rod_end_point == ENDPOINT_A) {
				// <----line-----[A==ROD==>B]
				EIEnd = -non_linear_EI;
			} else {
				// <----line-----[B==ROD==>A]
				EIEnd = non_linear_EI;
			}
			break;
		default:
			LOGERR << "Invalid end point qualifier: " << end_point << endl;
			throw moordyn::invalid_value_error("Invalid end point");
	}

	return qEnd * EIEnd / dlEnd;
}

void
Line::getStateDeriv(std::vector<vec>& vel, std::vector<vec>& acc, std::vector<vec>& misc)
{
	// NOTE:
	// Jose Luis Cercos-Pita: This is by far the most consuming function of the
	// whole library, just because it is called every single time substep and
	// it shall make computations in every single line node. Thus it is worthy
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

	// ======= calculate various kinematic quantities and stiffness forces =======

	// calculate unit tangent vectors (q) for each internal node. note: I've
	// reversed these from pointing toward 0 rather than N. Check sign of wave
	// loads. <<<<
	for (unsigned int i = 1; i < N; i++)
		unitvector(
		    q[i],
		    r[i - 1],
		    r[i + 1]); // compute unit vector q ... using adjacent two nodes!

	// calculate unit tangent vectors for either end node if the line has no
	// bending stiffness of if either end is pinned (otherwise it's already
	// been set via ::setEndOrientation())
	const bool isEI = (EI > 0) || (nEIpoints > 0);
	if ((endTypeA == PINNED) || !isEI)
		unitvector(q[0], r[0], r[1]);
	if ((endTypeB == PINNED) || !isEI)
		unitvector(q[N], r[N - 1], r[N]);

	// loop through the segments for stiffness forces and segment lengths
	for (unsigned int i = 0; i < N; i++) {
		// calculate current (Stretched) segment lengths and unit tangent
		// vectors (qs) for each segment (this is used for bending and stiffness calculations)
		lstr[i] = unitvector(qs[i], r[i], r[i + 1]);

		ldstr[i] = qs[i].dot(rd[i + 1] - rd[i]); // strain rate of segment

		// V[i] = A * l[i]; // volume attributed to segment

		// Calculate segment stiffness
		if (ElasticMod == 1) {
			// line tension
			if (nEApoints > 0)
				EA = getNonlinearEA(lstr[i], l[i]);

			if (lstr[i] / l[i] > 1.0) {
				T[i] = EA * (lstr[i] - l[i]) / l[i] * qs[i];
			} else {
				// cable can't "push" ...
				// or can it, if bending stiffness is nonzero? <<<<<<<<<
				T[i] = vec::Zero();
			}

			// line internal damping force
			if (nBApoints > 0)
				BA = getNonlinearBA(ldstr[i], l[i]);
			Td[i] = BA * ldstr[i] / l[i] * qs[i];

		} else if (ElasticMod > 1){ // viscoelastic model from https://asmedigitalcollection.asme.org/OMAE/proceedings/IOWTC2023/87578/V001T01A029/1195018 
			// note that Misc[i][1] is the same as Line%dl_1 in MD-F. This is the deltaL of the first static spring k1.
               
			if (ElasticMod == 2) {
               // constant dynamic stiffness
               EA_2 = EA_D;

            } else if (ElasticMod == 3){
            	if (Misc[i][1] >= 0.0) // spring k1 is in tension
                	// Mean load dependent dynamic stiffness: from combining eqn. 2 and eqn. 10 from original MD viscoelastic paper, taking mean load = k1 delta_L1 / MBL, and solving for k_D using WolframAlpha with following conditions: k_D > k_s, (MBL,alpha,beta,unstrLen,delta_L1) > 0
                	EA_2 = 0.5 * ((alphaMBL) + (vbeta*Misc[i][1]*(EA / l[i])) + EA + sqrt((alphaMBL * alphaMBL) + (2*alphaMBL*(EA / l[i]) * (vbeta*Misc[i][1] - l[i])) + ((EA / l[i])*(EA / l[i]) * (vbeta*Misc[i][1] + l[i])*(vbeta*Misc[i][1] + l[i]))));
            	
				else // spring k1 is in compression
                	EA_2 = alphaMBL; // mean load is considered to be 0 in this case. The second term in the above equation is not valid for delta_L1 < 0.
            }

            if (EA_2 == 0.0) { // Make sure EA_2 != 0 or else nans, also make sure EA != EA_D or else nans. 
               LOGERR << "Viscoelastic model: Dynamic stiffness cannot equal zero" << endl;
			   throw moordyn::invalid_value_error("Viscoelastic model: Dynamic stiffness cannot equal zero");
			} else if (EA_2 == EA) {
               LOGERR << "Viscoelastic model: Dynamic stiffness cannot equal static stiffness" << endl;
			   throw moordyn::invalid_value_error("Viscoelastic model: Dynamic stiffness cannot equal static stiffness");
			}
         
            const real EA_1 = EA_2*EA/(EA_2 - EA); // calculated EA_1 which is the stiffness in series with EA_D that will result in the desired static stiffness of EA_S. 
         
            const real dl = lstr[i] - l[i]; // delta l of this segment
         
            const real ld_1 = (EA_2*dl - (EA_2 + EA_1)*Misc[i][1] + BA_D*ldstr[i]) /( BA_D + BA); // rate of change of static stiffness portion [m/s]

            if (dl >= 0.0) // if both spring 1 (the spring dashpot in parallel) and the whole segment are not in compression
               T[i]  = (EA_1*Misc[i][1] / l[i]) * qs[i];  // compute tension based on static portion (dynamic portion would give same). See eqn. 14 in paper
            else 
               T[i] = vec::Zero(); // cable can't "push"

            Td[i] = BA*ld_1 / l[i] * qs[i];

            // update state derivative for static stiffness stretch
			Miscd[i][1] = ld_1;
		}
	}

	// calculate the curvatures and normal vectors (just if needed)
	if (isEI || isPb) {
		for (unsigned int i = 0; i <= N; i++) {
			if (i == 0) {
				// end node A case (only if attached to a Rod, i.e. a
				// cantilever rather than pinned point)
				Kurv[i] = (endTypeA == CANTILEVERED) ?
					GetCurvature(lstr[0], q[0], qs[0]) : 0.0;
			} else if (i == N) {
				// end node B case (only if attached to a Rod, i.e. a
				// cantilever rather than pinned point)
				Kurv[i] = (endTypeB == CANTILEVERED) ?
					GetCurvature(lstr[i - 1], qs[i - 1], q[i]) : 0.0;
			} else {
				// internal node
				// curvature <<< remember to check sign, or just take abs
				Kurv[i] = GetCurvature(lstr[i - 1] + lstr[i], qs[i - 1], qs[i]);
			}
			if (EqualRealNos(Kurv[i], 0.0)) {
				pvec[i] = vec::Zero();
				continue;
			}

			if (i == 0)
				pvec[i] = q[0].cross(qs[0]);
			else if (i == N)
				pvec[i] = qs[i - 1].cross(q[N]);
			else
				pvec[i] = qs[i - 1].cross(qs[i]);
			const real l_pvec = pvec[i].norm();
			// We can renormalize it for afterwards simplicity
			if (!EqualRealNos(l_pvec, 0.0))
				pvec[i] /= l_pvec;
		}
	}

	//============================================================================================
	// --------------------------------- apply wave kinematics
	// -----------------------------
	auto [zeta, U, Ud, pdyn] = waves->getWaveKinLine(lineId);

	// If in still water, iterate over all the segments and calculate
	// volume of segment submerged. This is later used to calculate
	// v_i, the *nodal* submerged volumes, which is then used to
	// to calculate buoyancy.
	for (unsigned int i = 0; i < N; i++) {
		// TODO - figure out the best math to do here
		// Averaging the surface heights at the two nodes is probably never
		// correct
		auto surface_height = 0.5 * (zeta[i] + zeta[i + 1]);
		F[i] = calcSubSeg(i, i + 1, surface_height);
	}
	//============================================================================================

	// ============  CALCULATE FORCES ON EACH NODE  ===============================

	// Bending loads
	// first zero out the forces from last run
	for (unsigned int i = 0; i <= N; i++) {
		Bs[i] = vec::Zero();
		Pb[i] = vec::Zero();
	}

	// and now compute them (if possible)
	if (isEI) {
		// loop through all nodes to calculate bending forces
		for (unsigned int i = 0; i <= N; i++) {
			const real Kurvi = Kurv[i];
			if (EqualRealNos(Kurvi, 0.0))
				continue;

			// calculate force on each node due to bending stiffness!

			if (i == 0) {
				// end node A case (only if attached to a Rod, i.e. a
				// cantilever rather than pinned point)
				if (endTypeA == CANTILEVERED)
				{
					if (nEIpoints > 0)
						EI = getNonlinearEI(Kurvi);

					// get direction of resulting force from bending to apply
					// on node i+1
					vec Mforce_ip1 = qs[0].cross(pvec[i]);

					// scale force direction vectors by desired moment force
					// magnitudes to get resulting forces on adjacent nodes
					Mforce_ip1 *= Kurvi * EI / lstr[i];

					// set force on node i to cancel out forces on adjacent
					// nodes
					vec Mforce_i = -Mforce_ip1;

					// apply these forces to the node forces
					Bs[i] += Mforce_i;
					Bs[i + 1] += Mforce_ip1;
				}
			}
			// end node A case (only if attached to a Rod, i.e. a cantilever
			// rather than pinned point)
			else if (i == N) {
				if (endTypeB == CANTILEVERED) // if attached to Rod i.e.
				                              // cantilever point
				{
					// curvature <<< check if this approximation
					// works for an end (assuming rod angle is node
					// angle which is middle of if there was a
					// segment -1/2
					if (nEIpoints > 0)
						EI = getNonlinearEI(Kurvi);

					// get direction of resulting force from bending to apply on
					// node i-1
					vec Mforce_im1 = qs[i - 1].cross(pvec[i]);

					// scale force direction vectors by desired moment force
					// magnitudes to get resulting forces on adjacent nodes
					Mforce_im1 *= Kurvi * EI / lstr[i - 1];

					// set force on node i to cancel out forces on adjacent
					// nodes
					vec Mforce_i = -Mforce_im1;

					// apply these forces to the node forces
					Bs[i - 1] += Mforce_im1;
					Bs[i] += Mforce_i;
				}
			}
			// internal node
			else {
				// curvature <<< remember to check
				// sign, or just take abs
				if (nEIpoints > 0)
					EI = getNonlinearEI(Kurvi);

				// get direction of resulting force from bending to apply on
				// node i-1
				vec Mforce_im1 = qs[i - 1].cross(pvec[i]);
				// get direction of resulting force from bending to apply on
				// node i+1
				vec Mforce_ip1 = qs[i].cross(pvec[i]);

				// scale force direction vectors by desired moment force
				// magnitudes to get resulting forces on adjacent nodes
				Mforce_im1 *= Kurvi * EI / lstr[i - 1];
				Mforce_ip1 *= Kurvi * EI / lstr[i];

				// set force on node i to cancel out forces on adjacent nodes
				vec Mforce_i = -Mforce_im1 - Mforce_ip1;

				// apply these forces to the node forces
				Bs[i - 1] += Mforce_im1;
				Bs[i] += Mforce_i;
				Bs[i + 1] += Mforce_ip1;
			}

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

	if (isPb) {
		// loop through all nodes to calculate bending forces
		for (unsigned int i = 0; i <= N; i++) {
			const real Kurvi = Kurv[i];
			if (EqualRealNos(Kurvi, 0.0))
				continue;
			// The driving pressure
			const real h = (std::max)(0.0, zeta[i] - r[i].z());
			const real p = pin[i] - (env->rho_w * env->g * h + pdyn[i]);
			// The direction
			const vec nvec = q[i].cross(pvec[i]);
			// So we can compute the force
			if (i == 0)
				Pb[i] = 0.5 * A * l[i] * p * Kurvi * nvec;
			else if (i == N)
				Pb[i] = 0.5 * A * l[i - 1] * p * Kurvi * nvec;
			else
				Pb[i] = 0.5 * A * (l[i] + l[i - 1]) * p * Kurvi * nvec;
		}
	}

	// This mostly just simplifies the code, but also sometimes helps the
	// compiler avoid repeated pointer lookup.
	const real rho_w = env->rho_w;
	const real g = env->g;

	// loop through the nodes
	for (unsigned int i = 0; i <= N; i++) {
		// Nodes are considered to have a line length of half the length of the
		// line on either side.
		const real length_left = i == 0 ? 0.0 : l[i - 1];
		const real length_right = i == N ? 0.0 : l[i];
		const real length = 0.5 * (length_left + length_right);
		const real submergence_left = i == 0 ? 0.0 : F[i - 1];
		const real submergence_right = i == N ? 0.0 : F[i];
		// Submerged length is half the submerged length of the neighboring
		// segments
		const real submerged_length = 0.5 * (length_left * submergence_left +
		                                     length_right * submergence_right);

		const real submerged_volume = submerged_length * A;

		const real m_i = A * length * rho; // node mass
		const real v_i = submerged_volume; // node submerged volume

		// Make node mass matrix
		const mat I = mat::Identity();
		const mat Q = q[i] * q[i].transpose();
		// calculate mass matrix
		M[i] = m_i * I + rho_w * v_i * (Can * (I - Q) + Cat * Q);

		// submerged weight (including buoyancy)

		// submerged weight (including buoyancy)
		W[i][0] = W[i][1] = 0.0;
		W[i][2] = -g * (m_i - v_i * rho_w);


		// magnitude of current
		const real Ui_mag = U[i].norm();
		// Unit vector of current flow
		const vec Ui_hat = U[i].normalized();
		// relative flow velocity over node
		const vec vi = U[i] - rd[i];
		// tangential relative flow component
		const real vql = vi.dot(q[i]);
		const vec vq = vql * q[i];
		// transverse relative flow component
		const vec vp = vi - vq;

		// magnitudes of relative flow
		const real vq_mag = vq.norm();
		const real vp_mag = vp.norm();

		const real shared_part = 0.5 * rho_w * d * submerged_length;
		// transverse drag
		Dp[i] = vp_mag * Cdn * shared_part * vp;
		// tangential drag
		Dq[i] = vq_mag * Cdt * pi * shared_part * vq;

		// Vortex Induced Vibration (VIV) cross-flow lift force
		if ((Cl > 0.0) && (!IC_gen)) { // If non-zero lift coefficient and not during IC_gen then VIV to be calculated

			// ----- The Synchronization Model ------
			// Crossflow velocity
			const real yd = rd[i].dot(q[i].cross(vp.normalized()));
			const real ydd = rdd_old[i].dot(q[i].cross(vp.normalized()));

			// Rolling RMS calculation
			const real yd_rms = sqrt((((n_m-1)*yd_rms_old[i]*yd_rms_old[i])+(yd*yd))/n_m); // RMS approximation from Thorsen
			const real ydd_rms = sqrt((((n_m-1)*ydd_rms_old[i]*ydd_rms_old[i])+(ydd*ydd))/n_m);

			if ((t >= t_old + dtm) || (t == 0.0)) { // Update the stormed RMS vaues
				// update back indexing one moordyn time step (regardless of time integration scheme). T_old is handled at end of getStateDeriv when rdd_old is updated.
				yd_rms_old[i] = yd_rms; // for rms back indexing (one moordyn timestep back)
				ydd_rms_old[i] = ydd_rms; // for rms back indexing (one moordyn timestep back)
			}

			if ((yd_rms==0.0) || (ydd_rms == 0.0)) phi_yd = atan2(-ydd, yd); // To avoid divide by zero
			else phi_yd = atan2(-ydd/ydd_rms, yd/yd_rms); 
			
			if (phi_yd < 0) phi_yd = 2*pi + phi_yd; // atan2 to 0-2Pi range

			// Note: amplitude calculations and states commented out. Would be needed if a Cl vs A lookup table was ever implemented

			const real phi = Misc[i][0] - (2 * pi * floor(Misc[i][0] / (2*pi))); // Map integrated phase to 0-2Pi range. Is this necessary? sin (a-b) is the same if b is 100 pi or 2pi
			// const real A_int = Misc[i][1];
			// const real As = Misc[i][2];

			// non-dimensional frequency
			const real f_hat = cF + dF *sin(phi_yd - phi); // phi is integrated from state deriv phi_dot
			// frequency of lift force (rad/s)
			const real phi_dot = 2*pi*f_hat*vp_mag / d;// to be added to state

			// ----- The rest of the model -----

			// // Oscillation amplitude 
			// const real A_int_dot = abs(yd);
			// // Note: Check if this actually measures zero crossings
			// if ((yd * yd_old[i]) < 0) { // if sign changed, i.e. a zero crossing
			// 	Amp[i] = A_int-A_int_old[i]; // amplitude calculation since last zero crossing
			// 	A_int_old[i] = A_int; // stores amplitude of previous zero crossing for finding Amp
			// }
			// // Careful with integrating smoothed amplitude, as 0.1 was a calibarated value based on a very simple integration method
			// const real As_dot = (0.1/dtm)*(Amp[i]-As); // As to be variable integrated from the state. stands for amplitude smoothed

			// // Lift coefficient from lookup table
			// const real C_l = cl_lookup(x = As/d); // create function in Line.hpp that uses lookup table 

			// The Lift force
			Lf[i] = 0.5 * env->rho_w * d * vp_mag * Cl * cos(phi) * q[i].cross(vp) * submerged_length;

			// Prep for returning VIV state derivatives
			Miscd[i][0] = phi_dot;
			// Miscd[i][2] = As_dot; // unused state that could be used for future amplitude calculations

		}	

		// tangential component of fluid acceleration
		// <<<<<<< check sign since I've reversed q
		const real aql = Ud[i].dot(q[i]);
		const vec aq = aql * q[i];
		// normal component of fluid acceleration
		const vec ap = Ud[i] - aq;

		// transverse Froude-Krylov force
		Ap[i] = rho_w * (1. + Can) * submerged_volume * ap;
		// tangential Froude-Krylov force
		Aq[i] = rho_w * (1. + Cat) * submerged_volume * aq;

		// bottom contact (stiffness and damping, vertical-only for now) -
		// updated for general case of potentially anchor or fairlead end in
		// contact
		const real waterDepth = getWaterDepth(r[i][0], r[i][1]);
		if (r[i][2] < waterDepth) {
			B[i][2] = ((waterDepth - r[i][2]) * env->kb - rd[i][2] * env->cb) *
			          d * (length);

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
		Fnet[i] += W[i] + (Dp[i] + Dq[i] + Ap[i] + Aq[i]) + B[i] +
			Bs[i] + Lf[i] + Pb[i];
	}

	// loop through internal nodes and compute the accelerations
	vel.reserve(N - 1);
	acc.reserve(N - 1);
	for (unsigned int i = 1; i < N; i++) {
		// For small systems it is usually faster to compute the inverse
		// of the matrix. See
		// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		acc[i - 1] = M[i].inverse() * Fnet[i];
		vel[i - 1] = rd[i];
	}

	if ((t >= t_old + dtm) || (t == 0.0)) { // update back indexing one moordyn time step (regardless of time integration scheme)
		t_old = t; // for updating back indexing if statements 
	}
	rdd_old = acc; // saving the acceleration for VIV RMS calculation

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
			auto [_z, U, _ud, _pdyn] = waves->getWaveKinLine(lineId);
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

		// output VIV force (only CF for now)
		if (channels.find("V") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << Lf[i][J]
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
					*outfile << Td[i][J] << "\t ";
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

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkPolyData>
Line::getVTK() const
{
	auto points = vtkSmartPointer<vtkPoints>::New();
	auto line = vtkSmartPointer<vtkPolyLine>::New();

	auto num_points = this->N + 1;
	auto num_cells = this->N;

	// Node fields, i.e. r.size() number of tuples
	auto vtk_rd = vector_to_vtk_array("rd", this->rd);
	auto vtk_Kurv = vector_to_vtk_array("Kurv", this->Kurv);
	auto vtk_Fnet = vector_to_vtk_array("Fnet", this->Fnet);
	auto vtk_M = io::vtk_farray("M", 9, num_points);
	auto vtk_D = io::vtk_farray("Drag", 3, num_points);
	auto [_z, U, _ud, _pdyn] = waves->getWaveKinLine(lineId);
	auto vtk_U = vector_to_vtk_array("U", U);

	// Segment fields, i.e. r.size()-1 number of tuples
	auto vtk_lstr = vector_to_vtk_array("lstr", this->lstr);
	auto vtk_ldstr = vector_to_vtk_array("ldstr", this->ldstr);
	auto vtk_V = vector_to_vtk_array("V", this->V);
	auto vtk_T = vector_to_vtk_array("T", this->T);
	auto vtk_F = vector_to_vtk_array("F", this->F);

	line->GetPointIds()->SetNumberOfIds(num_points);

	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->AllocateExact(num_cells, num_cells * 2);

	for (unsigned int i = 0; i < num_points; i++) {
		points->InsertNextPoint(r[i][0], r[i][1], r[i][2]);
		line->GetPointIds()->SetId(i, i);
		auto drag = Dq[i] + Dp[i];
		vtk_D->SetTuple3(i, drag[0], drag[1], drag[2]);
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

		std::array<vtkIdType, 2> cell_points{ i, i + 1 };
		cells->InsertNextCell(cell_points.size(), cell_points.data());
	}

	auto out = vtkSmartPointer<vtkPolyData>::New();
	out->SetPoints(points);
	out->SetLines(cells);

	out->GetCellData()->AddArray(vtk_lstr);
	out->GetCellData()->AddArray(vtk_ldstr);
	out->GetCellData()->AddArray(vtk_V);
	out->GetCellData()->AddArray(vtk_T);
	out->GetCellData()->AddArray(vtk_F);
	out->GetCellData()->SetActiveScalars("ldstr");

	out->GetPointData()->AddArray(vtk_rd);
	out->GetPointData()->AddArray(vtk_Kurv);
	out->GetPointData()->AddArray(vtk_M);
	out->GetPointData()->AddArray(vtk_Fnet);
	out->GetPointData()->AddArray(vtk_D);
	out->GetPointData()->AddArray(vtk_U);
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
MoorDyn_IsLineConstantEA(MoorDynLine l, int* b)
{
	CHECK_LINE(l);
	*b = (int)((moordyn::Line*)l)->isConstantEA();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetLineConstantEA(MoorDynLine l, double* EA)
{
	CHECK_LINE(l);
	*EA = ((moordyn::Line*)l)->getConstantEA();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLineConstantEA(MoorDynLine l, double EA)
{
	CHECK_LINE(l);
	((moordyn::Line*)l)->setConstantEA(EA);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_IsLinePressBend(MoorDynLine l, int* b)
{
	CHECK_LINE(l);
	*b = (int)((moordyn::Line*)l)->enabledPb();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLinePressBend(MoorDynLine l, int b)
{
	CHECK_LINE(l);
	if (b)
		((moordyn::Line*)l)->enablePb();
	else
		((moordyn::Line*)l)->disablePb();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLinePressInt(MoorDynLine l, const double* p)
{
	CHECK_LINE(l);
	unsigned int n = ((moordyn::Line*)l)->getN() + 1;
	std::vector<moordyn::real> pin(n);
	for (unsigned int i = 0; i < n; i++)
		pin[i] = (moordyn::real)p[i];
	((moordyn::Line*)l)->setPin(pin);
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
MoorDyn_GetLineNodeVel(MoorDynLine l, unsigned int i, double vel[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec rd = ((moordyn::Line*)l)->getNodeVel(i);
		moordyn::vec2array(rd, vel);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeForce(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeForce(i);
		moordyn::vec2array(force, f);
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
MoorDyn_GetLineNodeBendStiff(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeBendStiff(i);
		moordyn::vec2array(force, f);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeWeight(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeWeight(i);
		moordyn::vec2array(force, f);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeDrag(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeDrag(i);
		moordyn::vec2array(force, f);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeFroudeKrilov(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeFroudeKrilov(i);
		moordyn::vec2array(force, f);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_GetLineNodeSeabedForce(MoorDynLine l, unsigned int i, double f[3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec force = ((moordyn::Line*)l)->getNodeSeabedForce(i);
		moordyn::vec2array(force, f);
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
MoorDyn_GetLineNodeM(MoorDynLine l, unsigned int i, double m[3][3])
{
	CHECK_LINE(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::mat mass = ((moordyn::Line*)l)->getNodeM(i);
		moordyn::mat2array(mass, m);
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
