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

#include "Rod.hpp"
#include "Rod.h"
#include "Line.hpp"
#include "Waves.hpp"
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

// here is the new numbering scheme (N segments per line)

//   [connect (node 0)]  --- segment 0 --- [ node 1 ] --- seg 1 --- [node2] ---
//   ... --- seg n-2 --- [node n-1] --- seg n-1 ---  [connect (node N)]

Rod::Rod(moordyn::Log* log)
  : io::IO(log)
{
}

Rod::~Rod() {}

void
Rod::setup(int number_in,
           types type_in,
           RodProps* props,
           vec6 endCoords,
           unsigned int NumSegs,
           shared_ptr<ofstream> outfile_pointer,
           string channels_in)
{
	// ================== set up properties ===========
	number = number_in;
	type = type_in;

	N = NumSegs;

	LOGDBG << "Setting up Rod " << number << " (type " << type << ") with " << N
	       << " segments." << endl;

	d = props->d;
	rho = props->w / (pi / 4. * d * d);
	Can = props->Can;
	Cat = props->Cat;
	Cdn = props->Cdn;
	Cdt = props->Cdt;

	t = 0.;

	attachedA.clear();
	attachedB.clear();

	//	WaveKin = 0;  // start off with wave kinematics disabled.  Can be
	// enabled after initial conditions are found and wave kinematics are
	// calculated

	// ------------------------- size vectors -------------------------

	r.assign(N + 1, vec(0., 0., 0.));  // node positions [i][x/y/z]
	rd.assign(N + 1, vec(0., 0., 0.)); // node positions [i][x/y/z]
	l.assign(N, 0.0);                  // line unstretched segment lengths

	M.assign(N + 1, mat()); // mass matrices (3x3) for each node
	V.assign(N, 0.0);       // segment volume?

	// forces
	W.assign(N + 1, vec(0., 0., 0.));    // node weights
	Bo.assign(N + 1, vec(0., 0., 0.));   // node boyancy
	Pd.assign(N + 1, vec(0., 0., 0.));   // dynamic pressure
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

	// get Rod axis direction vector and Rod length
	UnstrLen = unitvector(
	    q, endCoords(Eigen::seqN(0, 3)), endCoords(Eigen::seqN(3, 3)));

	if (N == 0) {
		// special case of zero-length rod, which is denoted by numsegs=0 in the
		// intput file
		l.assign(1, 0.); // line unstretched segment lengths
		V.assign(1, 0.); // segment volume?
		UnstrLen = 0.0;  // set Rod length to zero
	} else {
		// normal finite-length case
		const real lseg =
		    UnstrLen / N;  // distribute line length evenly over segments
		l.assign(N, lseg); // line unstretched segment lengths
		V.assign(N, lseg * 0.25 * pi * d * d); // segment volume?
	}

	// ------------------------- set starting kinematics
	// -------------------------

	// set Rod positions if applicable
	if (type == FREE) {
		// For an independent rod, set the position right off the bat
		r6(Eigen::seqN(0, 3)) = endCoords(Eigen::seqN(0, 3));
		r6(Eigen::seqN(3, 3)) = q;
		v6 = vec6::Zero();
	} else if ((type == PINNED) || (type == CPLDPIN)) {
		// for a pinned rod, just set the orientation (position will be set
		// later by parent object)
		r6(Eigen::seqN(3, 3)) = q;
		v6(Eigen::seqN(3, 3)) = vec6::Zero();
	}
	// otherwise (for a fixed rod) the positions will be set by the parent body
	// or via coupling

	// Initialialize some variables (Just a bunch of them would be used, but we
	// better initialize everything just in case Output methods are called, so
	// no memory errors are triggered)
	const vec org = endCoords(Eigen::seqN(0, 3));
	const vec dst = endCoords(Eigen::seqN(3, 3));
	for (int i = 0; i <= N; i++) {
		const real f = i / (real)N;
		r[i] = org + f * (dst - org);
		rd[i] = vec::Zero();
	}

	// set the number of preset wave kinematic time steps to zero (flagging
	// disabled) to start with
	ntWater = 0;

	// record output file pointer and channel key-letter list
	outfile = outfile_pointer.get(); // make outfile point to the right place
	channels = channels_in;          // copy string of output channels to object

	LOGDBG << "   Set up Rod " << number << ", type '" << TypeName(type)
	       << "'. " << endl;
};

void
Rod::addLine(Line* l, EndPoints l_end_point, EndPoints end_point)
{
	LOGDBG << "L" << l->number << end_point_name(l_end_point) << "->R" << number
	       << end_point_name(end_point);
	const attachment a = { l, l_end_point };
	switch (end_point) {
		case ENDPOINT_A:
			attachedA.push_back(a);
			break;
		case ENDPOINT_B:
			attachedB.push_back(a);
			break;
		default:
			LOGERR << "Rod only has end points 'A' or 'B'" << endl;
			throw moordyn::invalid_value_error("Invalid end point");
	}
}

EndPoints
Rod::removeLine(EndPoints end_point, Line* line)
{
	EndPoints line_end_point;
	std::vector<attachment>* lines =
	    (end_point == ENDPOINT_A) ? &attachedA : &attachedB;
	// look through attached lines
	for (auto it = std::begin(*lines); it != std::end(*lines); ++it) {
		if (it->line != line)
			continue;
		// This is the line's entry in the attachment list
		line_end_point = it->end_point;
		lines->erase(it);

		LOGMSG << "Detached line " << line->number << " from rod " << number
		       << end_point_name(end_point) << endl;
		return line_end_point;
	}

	// line not found
	LOGERR << "Error: failed to find the line " << line->number
	       << " to remove from rod " << number << end_point_name(end_point)
	       << endl;
	throw moordyn::invalid_value_error("Invalid line");
};

std::pair<vec6, vec6>
Rod::initialize()
{
	LOGDBG << "Initializing Rod " << number << " (type '" << TypeName(type)
	       << "') now." << endl;

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
		// output velocities
		if (channels.find("v") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "vx \t Node" << i << "vy \t Node"
				         << i << "vz \t ";
			}
		}
		// output net node force
		if (channels.find("f") != string::npos) {
			for (unsigned int i = 0; i <= N; i++) {
				*outfile << "Node" << i << "Fx \t Node" << i << "Fy \t Node"
				         << i << "Fz \t ";
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
			// output velocities?
			if (channels.find("v") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(m/s) \t";
			}
			// output net node force?
			if (channels.find("f") != string::npos) {
				for (unsigned int i = 0; i <= 3 * N + 2; i++)
					*outfile << "(N) \t";
			}

			*outfile << "\n";
		}
	}

	// if (-env->WtrDpth > r[0][2]) {
	//	cout << "   Error: water depth is shallower than Line " << number << "
	// anchor." << endl; 	return;

	// set water kinematics flag based on global wave and current settings (for
	// now)
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

	// the r6 and v6 vectors should have already been set
	// r and rd of ends have already been set by setup function or by parent
	// object   <<<<< right? <<<<<

	// Pass kinematics to any attached lines (this is just like what a
	// Connection does, except for both ends) so that they have the correct
	// initial positions at this initialization stage.
	if (type != COUPLED) {
		// don't call this for type -2 coupled Rods as it's already been called
		setDependentStates();
	}

	// assign the resulting kinematics to its part of the state vector (only
	// matters if it's an independent Rod)

	// copy over state values for potential use during derivative calculations
	vec6 pos = vec6::Zero();
	vec6 vel = vec6::Zero();
	if (type == FREE)
		pos(Eigen::seqN(0, 3)) = r[0];
	pos(Eigen::seqN(3, 3)) = q;

	// otherwise this was only called to make the rod an output file and set its
	// dependent line end kinematics...

	LOGMSG << "Initialized Rod " << number << endl;
	return std::make_pair(pos, vel);
};

real
Rod::GetRodOutput(OutChanProps outChan)
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
		// for Rods, this option provides the net force applied by attached
		// lines at end A (if 0) or end B (if >0)
		if (outChan.NodeID > 0)
			return FextB.norm();
		return FextA.norm();
	} else if (outChan.QType == FX)
		return Fnet[outChan.NodeID][0];
	else if (outChan.QType == FY)
		return Fnet[outChan.NodeID][1];
	else if (outChan.QType == FZ)
		return Fnet[outChan.NodeID][2];
	LOGWRN << "Unrecognized output channel " << outChan.QType << endl;
	return 0.0;
}

void
Rod::storeWaterKin(real dt,
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

	LOGDBG << "Setting up wave variables for Rod " << number
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
Rod::setState(vec6 pos, vec6 vel)
{
	// copy over state values for potential use during derivative calculations
	if (type == FREE) {
		// end A coordinates & Rod direction unit vector
		r6(Eigen::seqN(0, 3)) = pos(Eigen::seqN(0, 3));
		r6(Eigen::seqN(3, 3)) = pos(Eigen::seqN(3, 3)).normalized();
		// end A velocityes & rotational velocities about unrotated axes
		v6 = vel;
	} else if ((type == CPLDPIN) || (type == PINNED)) {
		r6(Eigen::seqN(3, 3)) = pos(Eigen::seqN(3, 3)).normalized();
		v6(Eigen::seqN(3, 3)) = vel(Eigen::seqN(3, 3));
	} else {
		LOGERR << "Invalid rod type: " << TypeName(type) << endl;
		throw moordyn::invalid_value_error("Invalid rod type");
	}
	setDependentStates();

	if (N == 0) {
		// for zero-length Rod case, set orientation stuff to zero
		// (maybe not necessary...)
		r6(Eigen::seqN(3, 3)) = vec::Zero();
		v6(Eigen::seqN(3, 3)) = vec::Zero();
	}

	// update Rod direction unit vector (simply equal to last three entries of
	// r6)
	q = r6(Eigen::seqN(3, 3));
}

void
Rod::initiateStep(vec6 rFairIn, vec6 rdFairIn)
{
	if (type == COUPLED) {
		// set Rod kinematics based on BCs (linear model for now)
		r_ves = rFairIn;
		rd_ves = rdFairIn;

		// since this rod has no states and all DOFs have been set, pass its
		// kinematics to dependent Lines
		setDependentStates();
	} else if (type == CPLDPIN) {
		// set Rod *end A only* kinematics based on BCs (linear model for now)
		r_ves(Eigen::seqN(0, 3)) = rFairIn(Eigen::seqN(0, 3));
		rd_ves(Eigen::seqN(0, 3)) = rdFairIn(Eigen::seqN(0, 3));
	} else {
		LOGERR << "Invalid rod type: " << TypeName(type) << endl;
		throw moordyn::invalid_value_error("Invalid rod type");
	}
};

// updates kinematics for Rods ONLY if they are driven externally (otherwise
// shouldn't be called)
void
Rod::updateFairlead(real time)
{
	if (type == COUPLED) {
		// set Rod kinematics based on BCs (linear model for now)
		r6 = r_ves + rd_ves * time;
		v6 = rd_ves;
		// enforce direction vector to be a unit vector
		r6(Eigen::seqN(3, 3)) = r6(Eigen::seqN(3, 3)).normalized();

		// since this rod has no states and all DOFs have been set, pass its
		// kinematics to dependent Lines
		setDependentStates();
	} else if (type == CPLDPIN) {
		// set Rod *end A only* kinematics based on BCs (linear model for now)
		r6(Eigen::seqN(0, 3)) =
		    r_ves(Eigen::seqN(0, 3)) + rd_ves(Eigen::seqN(0, 3)) * time;
		v6(Eigen::seqN(0, 3)) = rd_ves(Eigen::seqN(0, 3));

		// Rod is pinned so only end A is specified, rotations are left alone
		// and will be handled, along with passing kinematics to dependent
		// lines, by separate call to setState
	} else {
		LOGERR << "Invalid rod type: " << TypeName(type) << endl;
		throw moordyn::invalid_value_error("Invalid rod type");
	}
}

void
Rod::setKinematics(vec6 r_in, vec6 rd_in)
{
	if (type == FIXED) {
		r6 = r_in;
		v6 = rd_in;

		// enforce direction vector to be a unit vector
		r6(Eigen::seqN(3, 3)) = r6(Eigen::seqN(3, 3)).normalized();

		// since this rod has no states and all DOFs have been set, pass its
		// kinematics to dependent Lines
		setDependentStates();
	} else if (type == PINNED) // rod end A pinned to a body
	{
		// set Rod *end A only* kinematics based on BCs (linear model for now)
		r6(Eigen::seqN(0, 3)) = r_in(Eigen::seqN(0, 3));
		v6(Eigen::seqN(0, 3)) = rd_in(Eigen::seqN(0, 3));

		// Rod is pinned so only end A is specified, rotations are left alone
		// and will be handled, along with passing kinematics to dependent
		// lines, by separate call to setState
	} else {
		LOGERR << "Invalid rod type: " << TypeName(type) << endl;
		throw moordyn::invalid_value_error("Invalid rod type");
	}

	// update Rod direction unit vector (simply equal to last three entries of
	// r6, presumably these were set elsewhere for pinned Rods)
	q = r6(Eigen::seqN(3, 3));
}

void
Rod::setDependentStates()
{
	// TODO: add bool initialization flag that will skip the N==0 block if true,
	// to avoid calling uninitialized lines during initialization <<<

	// from state values, set positions of end nodes
	r[0] = r6(Eigen::seqN(0, 3));
	rd[0] = v6(Eigen::seqN(0, 3));

	if (N > 0) {
		// set end B nodes only if the rod isn't zero length
		const vec rRel = UnstrLen * r6(Eigen::seqN(3, 3));
		r[N] = r[0] + rRel;
		const vec w = v6(Eigen::seqN(3, 3));
		rd[N] = rd[0] + w.cross(rRel);
	}

	// pass end node kinematics to any attached lines (this is just like what a
	// Connection does, except for both ends)
	for (auto attached : attachedA)
		attached.line->setEndKinematics(r[0], rd[0], attached.end_point);
	for (auto attached : attachedB)
		attached.line->setEndKinematics(r[N], rd[N], attached.end_point);

	// if this is a zero-length Rod, get bending moment-related information from
	// attached lines and compute Rod's equilibrium orientation
	if (N == 0) {
		//		double qEnd[3];       // unit vector of attached line end
		// segment, following same direction convention as Rod's q vector
		// double EIend; // bending stiffness of attached line end segment
		// double dlEnd; // stretched length of attached line end segment
		vec qMomentSum; // summation of qEnd*EI/dl_stretched (with correct sign)
		                // for each attached line

		qMomentSum = vec::Zero();
		for (auto attached : attachedA)
			qMomentSum += attached.line->getEndSegmentMoment(attached.end_point,
			                                                 ENDPOINT_A);
		for (auto attached : attachedB)
			qMomentSum += attached.line->getEndSegmentMoment(attached.end_point,
			                                                 ENDPOINT_B);

		// solve for line unit vector that balances all moments (unit vector of
		// summation of qEnd*EI/dl_stretched over each line)
		q = qMomentSum.normalized();
		r6(Eigen::seqN(3, 3)) = q; // set orientation angles (maybe not used)
	}

	// pass Rod orientation to any attached lines
	for (auto attached : attachedA)
		attached.line->setEndOrientation(q, attached.end_point, ENDPOINT_A);
	for (auto attached : attachedB)
		attached.line->setEndOrientation(q, attached.end_point, ENDPOINT_B);
}

std::pair<vec6, vec6>
Rod::getStateDeriv()
{
	// attempting error handling <<<<<<<<
	for (unsigned int i = 0; i <= N; i++) {
		if (isnan(r[i].sum())) {
			stringstream s;
			s << "NaN detected" << endl
			  << "Line " << number << " node positions:" << endl;
			for (unsigned int j = 0; j <= N; j++)
				s << j << " : " << r[j] << ";" << endl;
			throw moordyn::nan_error(s.str().c_str());
		}
	}

	// calculate forces and added mass for each node (including those from lines
	// attached to ends)
	vec6 Fnet_out; // total force vector
	mat6 M_out6;   // total mass matrix

	getNetForceAndMass(
	    Fnet_out,
	    M_out6); // call doRHS and sum each node's contributions about end A

	// >>>>>>>>> should the below only be done locally for free/pinned Rods?
	// >>>>>>>>>>>>>>>

	// supplement mass matrix with rotational inertia terms for axial rotation
	// of rod (this is based on assigning Jaxial * cos^2(theta) to each axis...
	const vec q2 = r6(Eigen::seqN(3, 3)).cwiseProduct(r6(Eigen::seqN(3, 3)));
	M_out6(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) +=
	    rho * d * d * d * d / 64.0 * q2.asDiagonal();

	// solve for accelerations in [M]{a}={f}, then fill in state derivatives
	vec6 vel6, acc6;
	if (type == FREE) {
		if (N == 0) {
			// special zero-length Rod case, orientation is not an actual state

			// For small systems it is usually faster to compute the inverse
			// of the matrix. See
			// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
			const vec Fnet_out3 = Fnet_out(Eigen::seqN(0, 3));
			const mat M_out3 = M_out6(Eigen::seqN(0, 3), Eigen::seqN(0, 3));
			const vec acc = M_out3.inverse() * Fnet_out3;

			// dxdt = V   (velocities)
			vel6(Eigen::seqN(0, 3)) = v6(Eigen::seqN(0, 3));
			vel6(Eigen::seqN(3, 3)) = vec::Zero();
			// dVdt = a   (accelerations)
			acc6(Eigen::seqN(0, 3)) = acc;
			acc6(Eigen::seqN(3, 3)) = vec::Zero();
		} else {
			// For small systems, which are anyway larger than 4x4, we can use
			// the ColPivHouseholderQR algorithm, which is working with every
			// single matrix, retaining a very good accuracy, and becoming yet
			// faster See:
			// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
			Eigen::ColPivHouseholderQR<mat6> solver(M_out6);
			acc6 = solver.solve(Fnet_out);

			// dxdt = V   (velocities)
			vel6(Eigen::seqN(0, 3)) = v6(Eigen::seqN(0, 3));
			// rate of change of unit vector components!!  CHECK!   <<<<<
			const vec v3 = v6(Eigen::seqN(3, 3));
			const vec r3 = r6(Eigen::seqN(3, 3));
			vel6(Eigen::seqN(3, 3)) = v3.cross(r3);
		}
	} else {
		// For small systems, which are anyway larger than 4x4, we can use the
		// ColPivHouseholderQR algorithm, which is working with every single
		// matrix, retaining a very good accuracy, and becoming yet faster
		// See:
		// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		Eigen::ColPivHouseholderQR<mat6> solver(M_out6);
		acc6 = solver.solve(Fnet_out);

		// dxdt = V   (velocities)
		vel6(Eigen::seqN(0, 3)) = vec::Zero();
		vel6(Eigen::seqN(3, 3)) = v6(Eigen::seqN(3, 3));
		// dVdt = a   (accelerations)
		acc6(Eigen::seqN(0, 3)) = vec::Zero();
	}

	return std::make_pair(vel6, acc6);
}

vec6
Rod::getFnet()
{
	// >>>>>>>>>>>>> do I want to leverage getNetForceAndMass or a saved global
	// to save comp time and code?  >>>>>>>> this function likely not used
	// >>>>>>>>>>>

	// Fnet_out is assumed to point to a size-3 array if the rod is pinned and
	// size-6 array if the rod is fixed

	int nDOF = 0;

	if (type == CPLDPIN) // if coupled pinned
		nDOF = 3;
	else if (type == COUPLED) // if coupled rigidly
		nDOF = 6;
	else {
		LOGERR << "Invalid rod type: " << TypeName(type) << endl;
		throw moordyn::invalid_value_error("Invalid rod type");
	}

	// this assumes doRHS() has already been called

	// make sure Fnet_out is zeroed first
	vec6 Fnet_out = vec6::Zero();

	// now go through each node's contributions, put them in body ref frame, and
	// sum them
	for (unsigned int i = 0; i <= N; i++) {
		// position of a given node relative to the body reference point (global
		// orientation frame)
		vec rRel = r[i] - r[0];

		vec6 F6_i; // 6dof force-moment from rod about body ref point (but
		           // global orientation frame of course)
		// convert segment net force into 6dof force about body ref point
		F6_i(Eigen::seqN(0, 3)) = Fnet[i];
		F6_i(Eigen::seqN(3, 3)) = rRel.cross(Fnet[i]);

		Fnet_out(Eigen::seqN(0, nDOF)) += F6_i(Eigen::seqN(0, nDOF));
	}

	// add any moments applied from lines at either end (might be zero)
	Fnet_out(Eigen::seqN(3, 3)) += Mext;

	//  this is where we'd add inertial loads for coupled rods! <<<<<<<<<<<<

	return Fnet_out;
}

// calculate the aggregate 6DOF rigid-body force and mass data of the rod
void
Rod::getNetForceAndMass(vec6& Fnet_out, mat6& M_out, vec rRef)
{
	// rBody is the location of the body reference point. A NULL pointer value
	// means the end A coordinates should be used instead.

	// question: do I really want to neglect the rotational inertia/drag/etc
	// across the length of each segment?

	doRHS(); // do calculations of forces and masses on each rod node

	// make sure Fnet_out and M_out are zeroed first
	Fnet_out = vec6::Zero();
	M_out = mat6::Zero();

	// NEW APPROACH:  (6 DOF properties now already summed in doRHS)

	// shift everything from end A reference to rRef reference point
	vec rRel = r[0] - rRef; // position of a given node relative to the body
	                        // reference point (global orientation frame)

	Fnet_out(Eigen::seqN(0, 3)) = F6net(Eigen::seqN(0, 3));
	// shift net forces and add the existing moments
	const vec f3net = F6net(Eigen::seqN(0, 3));
	Fnet_out(Eigen::seqN(3, 3)) = F6net(Eigen::seqN(3, 3)) + rRel.cross(f3net);

	// shift mass matrix to be about ref point
	M_out = translateMass6(rRel, M6net);

	// >>> do we need to ensure zero moment is passed if it's pinned? <<<
	// if (abs(Rod%typeNum)==1) then
	//   Fnet_out(4:6) = 0.0_DbKi
	// end if

	// >>> OLD APPROACH:
	/*
	// now go through each node's contributions, put them in body ref frame, and
	sum them for (int i=0; i<=N; i++)
	{
	    double rRel[3];     // position of a given node relative to the body
	reference point (global orientation frame) double Fnet_dub[3]; double
	M_dub[3][3]; double F6_i[6];     // 6dof force-moment from rod about body
	ref point (but global orientation frame of course) double M6_i[6][6];  //
	mass matrix of each rod to be added

	    for (int J=0; J<3; J++)
	    {	rRel[J] = r[i][J] - rRef[J];   // vector from reference point to
	node Fnet_dub[J] = Fnet[i][J];      // convert to array for passing
	<<<<<<<<< this intermediate array can be skipped for (int K=0; K<3; K++)
	M_dub[J][K] = M[i][J][K];  // <<<<<<<<< this intermediate array can be
	skipped
	    }

	    // convert segment net force into 6dof force about body ref point
	    translateForce3to6DOF(rRel, Fnet_dub, F6_i);

	    // convert segment mass matrix to 6by6 mass matrix about body ref point
	    translateMass3to6DOF(rRel, M_dub, M6_i);

	    for (int J=0; J<6; J++)
	    {
	        Fnet_out[J] += F6_i[J]; // add force to total force vector

	        for (int K=0; K<6; K++)
	            M_out[J][K] += M6_i[J][K];  // add element mass matrix to body
	mass matrix
	    }
	}

	// add any moments applied from lines at either end (might be zero)
	for (int J=0; J<3; J++)
	    Fnet_out[J+3] = Fnet_out[J+3] + Mext[J];

	*/
}

void
Rod::doRHS()
{
	// ---------------------------- initial rod and node calculations
	// ------------------------

	// calculate some orientation information for the Rod as a whole
	const auto angles = orientationAngles(q);
	const real phi = angles.first;
	const real beta = angles.second;
	const real sinPhi = sin(phi);
	const real cosPhi = cos(phi);
	const real tanPhi = tan(phi);
	const real sinBeta = sin(beta);
	const real cosBeta = cos(beta);

	// save to internal roll and pitch variables for use in output <<< should
	// check these, make Euler angles isntead of independent <<<
	roll = -180.0 / pi * phi * sinBeta;
	pitch = 180.0 / pi * phi * cosBeta;

	// set interior node positions and velocities (stretch the nodes between the
	// endpoints linearly) (skipped for zero-length Rods)
	for (int i = 1; i < N; i++) {
		const real f = i / (real)N;
		r[i] = r[0] + f * (r[N] - r[0]);
		rd[i] = rd[0] + f * (rd[N] - rd[0]);
		V[i] = 0.25 * pi * d * d * l[i]; // volume attributed to segment
	}

	//	// stretched length of entire rod and strain rate
	//	double lstr_squared = 0.0;
	//	for (int J=0; J<3; J++) lstr_squared += (r[N][J] - r[0][J])*(r[N][J] -
	// r[0][J]); 	lstr = sqrt(lstr_squared); 	// stretched rod length
	//
	//	double ldstr_top = 0.0;   // this is the denominator of how the stretch
	// rate equation was formulated 	for (int J=0; J<3; J++) ldstr_top +=
	// (r[N][J]
	//- r[0][J])*(rd[N][J] - rd[0][J]); 	ldstr = ldstr_top/lstr; 	// rate
	// of rod stretch (m/s)

	// note: Rod's q unit tangent vector is already set by setState-type
	// functions (and is same as last three entries of r6)

	//============================================================================================
	// --------------------------------- apply wave kinematics
	// ------------------------------------

	if (WaterKin == WAVES_EXTERNAL) {
		// wave kinematics time series set internally for each node

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
			PDyn[i] = 0.0; // this option is out of date
			               // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			F[i] = 1.0;    // FTS[i][it] + frac*(FTS[i][it+1] - FTS[i][it]);

			U[i] = UTS[i][it] + frac * (UTS[i][it + 1] - UTS[i][it]);
			Ud[i] = UdTS[i][it] + frac * (UdTS[i][it + 1] - UdTS[i][it]);
		}
	} else if (WaterKin == WAVES_FFT_GRID) {
		// wave kinematics interpolated from global grid in Waves object
		for (unsigned int i = 0; i <= N; i++) {
			waves->getWaveKin(
			    r[i][0],
			    r[i][1],
			    r[i][2],
			    U[i],
			    Ud[i],
			    zeta[i],
			    PDyn[i]); // call generic function to get water velocities

			// >>> add Pd variable for dynamic pressure, which will be applied
			// on Rod surface

			F[i] = 1.0; // set VOF value to one for now (everything submerged -
			            // eventually this should be element-based!!!) <<<<
		}
	} else if (WaterKin != WAVES_NONE) {
		// Hopefully WaterKin is set to zero, meaning no waves or set
		// externally, otherwise it's an error
		LOGERR << "We got a problem with WaterKin?!?!?" << endl;
		throw moordyn::unhandled_error("Invalid WaterKin");
	}

	// >>> remember to check for violated conditions, if there are any... <<<

	real zeta_i = zeta[N]; // just use the wave elevation computed at the
	                       // location of the top node for now

	if ((r[0][2] < zeta_i) && (r[N][2] > zeta_i)) {
		// the water plane is crossing the rod
		// (should also add some limits to avoid near-horizontals at some point)
		h0 = (zeta_i - r[0][2]) / q[2];
	} else if (r[0][2] < zeta_i) {
		// fully submerged case
		h0 = UnstrLen;
	} else {
		// fully unsubmerged case (ever applicable?)
		h0 = 0.0;
	}

	// -------------------------- loop through all the nodes
	// -----------------------------------
	real Lsum = 0.0;
	for (unsigned int i = 0; i <= N; i++) {
		// calculate mass matrix   <<<< can probably simplify/eliminate this...
		real dL;  // segment length corresponding to the node
		real m_i; // node mass
		real v_i; // node submerged volume
		const real Area = 0.25 * pi * d * d;

		if (i == 0) {
			dL = 0.5 * l[i];
			m_i = Area * dL * rho; //  (will be zero for zero-length Rods)
			v_i = 0.5 * F[i] * V[i];
		} else if (i == N) {
			dL = 0.5 * l[i - 1];
			m_i = Area * dL * rho;
			v_i = 0.5 * F[i - 1] * V[i - 1];
		} else {
			dL = 0.5 * (l[i - 1] + l[i]);
			m_i = Area * dL * rho;
			v_i = 0.5 *
			      (F[i - 1] * V[i - 1] +
			       F[i] * V[i]); // <<< remove F term here and above 2 cases??
		}

		// get scalar for submerged portion
		double VOF;
		if (Lsum + dL <= h0) // if fully submerged
			VOF = 1.0;
		else if (Lsum < h0) // if partially below waterline
			VOF = (h0 - Lsum) / dL;
		else // must be out of water
			VOF = 0.0;

		Lsum = Lsum + dL; // add length attributed to this node to the total

		// make node mass matrix  (will be zero for zero-length Rods)
		const mat I = mat::Identity();
		const mat Q = q * q.transpose();
		M[i] = m_i * I + env->rho_w * v_i * (Can * (I - Q) + Cat * Q);

		// mass matrices will be summed up before inversion, near end of this
		// function

		// ============  CALCULATE FORCES ON EACH NODE
		// ===============================

		// relative flow velocity over node
		const vec vi = U[i] - rd[i];
		// tangential relative flow component
		const vec vq = vi.dot(q) * q;
		// transverse relative flow component
		const vec vp = vi - vq;

		const real vq_mag = vq.norm();
		const real vp_mag = vp.norm();

		// fluid acceleration components for current node
		const vec aq =
		    Ud[i].dot(q) * q;      // tangential component of fluid acceleration
		const vec ap = Ud[i] - aq; // normal component of fluid acceleration

		if (N > 0) {
			// this is only nonzero for finite-length rods (skipped for
			// zero-length Rods)

			// note: no nodal axial structural loads calculated since it's
			// assumed rigid, but should I calculate tension/compression due to
			// other loads?

			// weight (now only the dry weight)
			W[i][0] = W[i][1] = 0.0;
			W[i][2] = m_i * (-env->g);

			// buoyance (now calculated based on outside pressure, for submerged
			// portion only) radial buoyancy force from sides
			real Ftemp = -VOF * Area * dL * env->rho_w * env->g * sinPhi;
			Bo[i] = Ftemp * vec(cosBeta * cosPhi, sinBeta * cosPhi, sinPhi);

			// transverse and tangential drag
			Dp[i] = VOF * 0.5 * env->rho_w * Cdn * d * dL * vp_mag * vp;
			Dq[i] = VOF * 0.5 * env->rho_w * Cdt * pi * d * dL * vq_mag * vq;

			// transverse and axial Froude-Krylov force
			Ap[i] = VOF * env->rho_w * (1. + Can) * v_i * ap;
			Aq[i] = vec::Zero(); // VOF * env->rho_w*(1.+Cat)* v_i * aq[J]; <<<
			                     // should anything here be included?

			// dynamic pressure
			Pd[i] = vec::Zero(); // assuming zero for sides for now, until taper
			                     // comes into play

			// seabed contact (stiffness and damping, vertical-only for now) -
			// updated for general case of potentially anchor or fairlead end in
			// contact
			B[i][0] = 0.0;
			B[i][1] = 0.0;
			if (r[i][2] < -env->WtrDpth)
				B[i][2] =
				    ((-env->WtrDpth - r[i][2]) * env->kb - rd[i][2] * env->cb) *
				    d * dL;
			else {
				B[i][2] = 0.0;
			}
		} else { // for zero-length rods, make sure various forces are zero
			W[i] = vec::Zero();
			Bo[i] = vec::Zero();
			Dp[i] = vec::Zero();
			Dq[i] = vec::Zero();
			Ap[i] = vec::Zero();
			Aq[i] = vec::Zero();
			Pd[i] = vec::Zero();
			B[i] = vec::Zero();
		}

		// ------ now add forces, moments, and added mass from Rod end effects
		// (these can exist even if N==0) -------

		// end A
		if ((i == 0) && (h0 > 0.0)) // if this is end A and it is submerged
		{
			// >>> eventually should consider a VOF approach for the ends hTilt
			// = 0.5*Rod%d/cosPhi <<<

			// buoyancy force
			real Ftemp = -VOF * Area * env->rho_w * env->g * r[i][2];
			Bo[i] += Ftemp * vec(cosBeta * sinPhi, sinBeta * sinPhi, cosPhi);

			// buoyancy moment
			real Mtemp =
			    -VOF / 64.0 * pi * d * d * d * d * env->rho_w * env->g * sinPhi;
			Mext += Mtemp * vec(sinBeta, -cosBeta, 0.0);

			// axial drag
			Dq[i] += VOF * Area * env->rho_w * Cdt * vq_mag * vq;

			// Froud-Krylov force
			const real V_temp = 2.0 / 3.0 * pi * d * d * d / 8.0;
			Aq[i] += VOF * env->rho_w * (1.0 + Cat) * V_temp * aq;

			// dynamic pressure force
			Pd[i] += VOF * Area * PDyn[i] * q;

			// added mass
			const mat Q = q * q.transpose();
			M[i] = VOF * env->rho_w * V_temp * Cat * Q;
		}

		if ((i == N) && (h0 >= UnstrLen)) {
			// if this end B and it is submerged
			// NOTE: if N=0, both this and previous if statement are true

			// buoyancy force
			real Ftemp = VOF * Area * env->rho_w * env->g * r[i][2];
			Bo[i] += Ftemp * vec(cosBeta * sinPhi, sinBeta * sinPhi, cosPhi);

			// buoyancy moment
			real Mtemp =
			    VOF / 64.0 * pi * d * d * d * d * env->rho_w * env->g * sinPhi;
			Mext += Mtemp * vec(sinBeta, -cosBeta, 0.0);

			// axial drag
			Dq[i] += VOF * Area * env->rho_w * Cdt * vq_mag * vq;

			// Froud-Krylov force
			const real V_temp = 2.0 / 3.0 * pi * d * d * d / 8.0;
			Aq[i] += VOF * env->rho_w * (1.0 + Cat) * V_temp * aq;

			// dynamic pressure force
			Pd[i] += -VOF * Area * PDyn[i] * q;

			// added mass
			const mat Q = q * q.transpose();
			M[i] = VOF * env->rho_w * V_temp * Cat * Q;
		}

		// ----------------- total forces for this node --------------------
		Fnet[i] = W[i] + Bo[i] + Dp[i] + Dq[i] + Ap[i] + Aq[i] + Pd[i] + B[i];
	} // i - done looping through nodes

	// ----- add waterplane moment of inertia moment if applicable -----
	if ((r[0][2] < zeta_i) && (r[N][2] > zeta_i)) {
		// the water plane is crossing the rod
		real Mtemp = 1.0 / 16.0 * pi * d * d * d * d * env->rho_w * env->g *
		             sinPhi * (1.0 + 0.5 * tanPhi * tanPhi);
		Mext += Mtemp * vec(sinBeta, -cosBeta, 0.0);
	}

	// ============ now add in forces on end nodes from attached lines
	// =============

	// zero the external force/moment sums (important!)
	FextA = vec::Zero();
	FextB = vec::Zero();
	// BUG: Then why we did the computations in line 1146???
	Mext = vec::Zero();

	for (auto attached : attachedA) {
		vec Fnet_i, Mnet_i;
		mat M_i;

		// get quantities
		attached.line->getEndStuff(Fnet_i, Mnet_i, M_i, attached.end_point);

		// Process outline for line failure, similar to as done for connections
		// (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if
		// failure time has elapsed for line
		// 2. create new massless connect with same instantaneous kinematics as
		// current Rod end
		// 3. disconnect line end from current Rod end and instead attach to new
		// connect The above may require rearrangement of connection indices,
		// expansion of state vector, etc.

		// sum quantitites
		Fnet[0] += Fnet_i; // forces
		FextA += Fnet_i;   // a copy for outputting totalled line loads
		Mext += Mnet_i;    // moments
		M[0] += M_i;       // mass matrix
	}

	for (auto attached : attachedB) {
		vec Fnet_i, Mnet_i;
		mat M_i;

		// get quantities
		attached.line->getEndStuff(Fnet_i, Mnet_i, M_i, attached.end_point);

		// Process outline for line failure, similar to as done for connections
		// (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if
		// failure time has elapsed for line
		// 2. create new massless connect with same instantaneous kinematics as
		// current Rod end
		// 3. disconnect line end from current Rod end and instead attach to new
		// connect The above may require rearrangement of connection indices,
		// expansion of state vector, etc.

		// sum quantitites
		Fnet[N] += Fnet_i; // forces
		FextB += Fnet_i;   // a copy for outputting totalled line loads
		Mext += Mnet_i;    // moments
		M[N] += M_i;       // mass matrix
	}

	// ---------------- now lump everything in 6DOF about end A
	// -----------------------------

	// question: do I really want to neglect the rotational inertia/drag/etc
	// across the length of each segment?

	// make sure 6DOF quantiaties are zeroed before adding them up
	F6net = vec6::Zero();
	M6net = mat6::Zero();

	// now go through each node's contributions, put them about end A, and sum
	// them
	for (unsigned int i = 0; i <= N; i++) {
		// position of a given node relative to end A node
		const vec rRel = r[i] - r[0];
		// 6dof force-moment from rod about body ref point (but global
		// orientation frame of course)
		vec6 F6_i;
		F6_i(Eigen::seqN(0, 3)) = Fnet[i];
		F6_i(Eigen::seqN(3, 3)) = rRel.cross(Fnet[i]);
		// mass matrix of each rod to be added
		const mat6 M6_i =
		    translateMass(rRel, M[i](Eigen::seqN(0, 3), Eigen::seqN(0, 3)));

		// sum contributions
		F6net += F6_i;
		M6net += M6_i;
	}

	// ------------- Calculate some items for the Rod as a whole here
	// -----------------

	// >>> could some of these be precalculated just once? <<<

	// rod total mass, used to help with making generic inertia coefficients
	real mass = UnstrLen * 0.25 * pi * pi * rho;

	// add inertia terms for the Rod assuming it is uniform density (radial
	// terms add to existing matrix which contains parallel-axis-theorem
	// components only)
	mat Imat_l = mat::Zero();
	if (N > 0) {
		real I_l = 0.125 * mass * d * d; // axial moment of inertia
		// summed radial moment of inertia for each segment individually
		real I_r = mass / 12.0 * (0.75 * d * d + pow(UnstrLen / N, 2)) * N;

		Imat_l(0, 0) = I_r;
		Imat_l(1, 1) = I_r;
		Imat_l(2, 2) = I_l;
	}

	// get rotation matrix to put things in global rather than rod-axis
	// orientations
	const mat OrMat = RotXYZ(phi, beta, 0.0);
	const mat Imat = rotateMass(Imat_l, OrMat);
	// these supplementary inertias can then be added the matrix (these are the
	// terms ASIDE from the parallel axis terms)
	M6net(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) += Imat;

	// now add centripetal and gyroscopic forces/moments, and that should be
	// everything
	/*
	double h_c = 0.5*UnstrLen;          // distance to center of mass
	double r_c[3];
	for (int J=0; J<3; J++)
	    r_c[J] = h_c*q[J];                 // vector to center of mass

	// note that Rod%v6(4:6) is the rotational velocity vector, omega
	for (int J=0; J<3; J++)
	{	Fcentripetal[J] = 0.0; //<<<TEMP<<< -cross_product(Rod%v6(4:6),
	cross_product(Rod%v6(4:6), r_c ))*Rod%mass <<< Mcentripetal[J] = 0.0;
	//<<<TEMP<<< cross_product(r_c, Fcentripetal) - cross_product(Rod%v6(4:6),
	MATMUL(Imat,Rod%v6(4:6)))
	}
	*/

	// add centripetal force/moment, gyroscopic moment, and any moments applied
	// from lines at either end (might be zero)
	// F6net(Eigen::seqN(0, 3)) += Fcentripetal
	F6net(Eigen::seqN(3, 3)) += Mext; // + Mcentripetal

	// NOTE: F6net saves the Rod's net forces and moments (excluding inertial
	// ones) for use in later output
	//       (this is what the rod will apply to whatever it's attached to, so
	//       should be zero moments if pinned). M6net saves the rod's mass
	//       matrix.
}

// write output file for line  (accepts time parameter since retained time value
// (t) will be behind by one line time step
void
Rod::Output(real time)
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
		if (channels.find("p") != string::npos) {
			for (int i = 0; i <= N; i++) // loop through nodes
			{
				for (int J = 0; J < 3; J++)
					*outfile << r[i][J] << "\t ";
			}
		}
		// output velocities?
		if (channels.find("v") != string::npos) {
			for (int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << rd[i][J] << "\t ";
			}
		}
		// output net node forces?
		if (channels.find("f") != string::npos) {
			for (int i = 0; i <= N; i++) {
				for (int J = 0; J < 3; J++)
					*outfile << Fnet[i][J] << "\t ";
			}
		}

		*outfile << "\n";
	}
	return;
}

std::vector<uint64_t>
Rod::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	data.push_back(io::IO::Serialize(t));
	subdata = io::IO::Serialize(r6);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(v6);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(r);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(q);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(l);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(M);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(V);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(FextA);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(FextB);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Mext);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(F6net);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(M6net);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(W);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Bo);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Pd);
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
	data.push_back(io::IO::Serialize(h0));
	subdata = io::IO::Serialize(r_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(zetaTS);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(FTS);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(UTS);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(UdTS);
	data.insert(data.end(), subdata.begin(), subdata.end());
	data.push_back(io::IO::Serialize(dtWater));

	return data;
}

uint64_t*
Rod::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	ptr = io::IO::Deserialize(ptr, t);
	ptr = io::IO::Deserialize(ptr, r6);
	ptr = io::IO::Deserialize(ptr, v6);
	ptr = io::IO::Deserialize(ptr, r);
	ptr = io::IO::Deserialize(ptr, rd);
	ptr = io::IO::Deserialize(ptr, q);
	ptr = io::IO::Deserialize(ptr, l);
	ptr = io::IO::Deserialize(ptr, M);
	ptr = io::IO::Deserialize(ptr, V);
	ptr = io::IO::Deserialize(ptr, FextA);
	ptr = io::IO::Deserialize(ptr, FextB);
	ptr = io::IO::Deserialize(ptr, Mext);
	ptr = io::IO::Deserialize(ptr, F6net);
	ptr = io::IO::Deserialize(ptr, M6net);
	ptr = io::IO::Deserialize(ptr, W);
	ptr = io::IO::Deserialize(ptr, Bo);
	ptr = io::IO::Deserialize(ptr, Pd);
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
	ptr = io::IO::Deserialize(ptr, h0);
	ptr = io::IO::Deserialize(ptr, r_ves);
	ptr = io::IO::Deserialize(ptr, rd_ves);
	ptr = io::IO::Deserialize(ptr, zetaTS);
	ptr = io::IO::Deserialize(ptr, FTS);
	ptr = io::IO::Deserialize(ptr, UTS);
	ptr = io::IO::Deserialize(ptr, UdTS);
	ptr = io::IO::Deserialize(ptr, dtWater);
	ntWater = zetaTS.size();

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkPolyData>
Rod::getVTK() const
{
	auto points = vtkSmartPointer<vtkPoints>::New();
	auto line = vtkSmartPointer<vtkPolyLine>::New();
	auto vtk_rd = io::vtk_farray("rd", 3, r.size());
	auto vtk_Fnet = io::vtk_farray("Fnet", 3, r.size());

	line->GetPointIds()->SetNumberOfIds(r.size());
	for (unsigned int i = 0; i < r.size(); i++) {
		points->InsertNextPoint(r[i][0], r[i][1], r[i][2]);
		line->GetPointIds()->SetId(i, i);
		vtk_rd->SetTuple3(i, rd[i][0], rd[i][1], rd[i][2]);
		vtk_Fnet->SetTuple3(i, Fnet[i][0], Fnet[i][1], Fnet[i][2]);
	}
	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(line);

	auto out = vtkSmartPointer<vtkPolyData>::New();
	out->SetPoints(points);
	out->SetLines(cells);

	out->GetPointData()->AddArray(vtk_rd);
	out->GetPointData()->AddArray(vtk_Fnet);
	out->GetPointData()->SetActiveVectors("Fnet");

	return out;
}

void
Rod::saveVTK(const char* filename) const
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
Rod::drawGL(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i = 0; i <= N; i++) {
		double newTen = getNodeTen(i);
		if (newTen > maxTen)
			maxTen = newTen;
	}

	glColor3f(0.5, 0.5, 1.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= N; i++) {
		glVertex3d(r[i][0], r[i][1], r[i][2]);
		if (i < N) {
			normTen = getNodeTen(i) / maxTen;
			ColorMap(normTen, rgb);
			glColor3d(rgb[0], rgb[1], rgb[2]);
		}
	}
	glEnd();
};

void
Rod::drawGL2(void)
{
	double maxTen = 0.0;
	double normTen;
	double rgb[3];
	for (int i = 0; i <= N; i++) {
		double newTen = getNodeTen(i);
		if (newTen > maxTen)
			maxTen = newTen;
	}

	// line
	for (int i = 0; i < N; i++) {
		normTen = 0.2 + 0.8 * pow(getNodeTen(i) / maxTen, 4.0);
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

/// Check that the provided system is not Null
#define CHECK_ROD(r)                                                           \
	if (!r) {                                                                  \
		cerr << "Null rod received in " << __FUNC_NAME__ << " ("               \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetRodID(MoorDynRod rod, int* id)
{
	CHECK_ROD(rod);
	*id = ((moordyn::Rod*)rod)->number;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetRodType(MoorDynRod rod, int* t)
{
	CHECK_ROD(rod);
	*t = ((moordyn::Rod*)rod)->type;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetRodN(MoorDynRod rod, unsigned int* n)
{
	CHECK_ROD(rod);
	*n = ((moordyn::Rod*)rod)->getN();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetRodNumberNodes(MoorDynRod rod, unsigned int* n)
{
	int err = MoorDyn_GetRodN(rod, n);
	if (err != MOORDYN_SUCCESS)
		return err;
	*n += 1;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetRodNodePos(MoorDynRod rod, unsigned int i, double pos[3])
{
	CHECK_ROD(rod);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const moordyn::vec r = ((moordyn::Rod*)rod)->getNodePos(i);
		moordyn::vec2array(r, pos);
	}
	MOORDYN_CATCHER(err, err_msg);
	return err;
}

int DECLDIR
MoorDyn_SaveRodVTK(MoorDynRod l, const char* filename)
{
#ifdef USE_VTK
	CHECK_ROD(l);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::Rod*)l)->saveVTK(filename);
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
