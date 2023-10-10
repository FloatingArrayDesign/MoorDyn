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

// This is version 2.a5, 2021-03-16

#include "MoorDyn2.h"
#include "Misc.hpp"
#include "MoorDyn2.hpp"
#include "Rod.hpp"
#include <limits>

#ifdef LINUX
#include <cmath>
#include <ctype.h>

// contributed by Yi-Hsiang Yu at NREL
#define isnan(x) std::isnan(x)
#endif

#ifdef USE_VTK
#include <vtkXMLMultiBlockDataWriter.h>
#endif

using namespace std;

/**
 * @brief A helper function for getting the size of a vector as an unsigned int
 *
 * @tparam T
 * @param a Vector to get size of
 * @return unsigned int Size of vector (truncated to lower sizeof(unsigned int)
 * bytes)
 */
template<typename T>
unsigned int
ui_size(const std::vector<T>& a)
{
	return static_cast<unsigned int>(a.size());
}

namespace moordyn {

/// The list of units for the output
const char* UnitList[] = {
	"(s)       ", "(m)       ", "(m)       ", "(m)       ", "(deg)     ",
	"(deg)     ", "(deg)     ", "(m/s)     ", "(m/s)     ", "(m/s)     ",
	"(deg/s)   ", "(deg/s)   ", "(deg/s)   ", "(m/s2)    ", "(m/s2)    ",
	"(m/s2)    ", "(deg/s2)  ", "(deg/s2)  ", "(deg/s2)  ", "(N)       ",
	"(N)       ", "(N)       ", "(N)       ", "(Nm)      ", "(Nm)      ",
	"(Nm)      ", "(frac)    "
};

moordyn::MoorDyn::MoorDyn(const char* infilename, int log_level)
  : io::IO(NULL)
  , _filepath("Mooring/lines.txt")
  , _basename("lines")
  , _basepath("Mooring/")
  , ICDfac(5.0)
  , ICdt(1.0)
  , ICTmax(120.0)
  , ICthresh(0.001)
  , WaveKinTemp(waves::WAVES_NONE)
  , dtM0(0.001)
  , dtOut(0.0)
  , _t_integrator(NULL)
  , env(std::make_shared<EnvCond>())
  , GroundBody(NULL)
  , waves(nullptr)
  , seafloor(nullptr)
  , nX(0)
  , nXtra(0)
  , npW(0)
{
	SetLogger(new Log(log_level));

	if (infilename && (strlen(infilename) > 0)) {
		_filepath = infilename;
		const std::size_t lastSlash = _filepath.find_last_of("/\\");
		const std::size_t lastDot = _filepath.find_last_of('.');
		_basename = _filepath.substr(lastSlash + 1, lastDot - lastSlash - 1);
		_basepath = _filepath.substr(0, lastSlash + 1);
	}

	LOGMSG << "\n Running MoorDyn (v2.0.0, 2023-09-18)" << endl
	       << "         MoorDyn v2 has significant ongoing input file changes "
	          "from v1."
	       << endl
	       << "   Copyright: (C) 2023 National Renewable Energy Laboratory, "
	          "(C) 2014-2019 Matt Hall"
	       << endl
	       << "   This program is released under the  BSD 3-Clause license."
	       << endl;

	LOGMSG << "The filename is " << _filepath << endl;
	LOGDBG << "The basename is " << _basename << endl;
	LOGDBG << "The basepath is " << _basepath << endl;

	env->g = 9.80665;
	env->WtrDpth = 0.;
	env->rho_w = 1025.;
	env->kb = 3.0e6;
	env->cb = 3.0e5;
	env->waterKinOptions = waves::WaterKinOptions();
	env->WriteUnits = 1; // by default, write units line
	env->writeLog = 0;   // by default, don't write out a log file
	env->FrictionCoefficient = 0.0;
	env->FricDamp = 200.0;
	env->StatDynFricScale = 1.0;

	waves = std::make_shared<moordyn::Waves>(_log);

	const moordyn::error_id err = ReadInFile();
	MOORDYN_THROW(err, "Exception while reading the input file");

	LOGDBG << "MoorDyn is expecting " << NCoupledDOF()
	       << " coupled degrees of freedom" << endl;

	if (!nX) {
		LOGWRN << "WARNING: MoorDyn has no state variables."
		       << " (Is there a mooring sytem?)" << endl;
	}

	nXtra = nX + 6 * 2 * ui_size(LineList);
}

moordyn::MoorDyn::~MoorDyn()
{
	if (outfileMain.is_open())
		outfileMain.close();
	for (auto outfile : outfiles) // int l=0; l<nLines; l++)
		if (outfile && outfile->is_open())
			outfile->close();

	delete _t_integrator;

	delete GroundBody;
	for (auto obj : LinePropList)
		delete obj;
	for (auto obj : RodPropList)
		delete obj;
	for (auto obj : FailList)
		delete obj;
	for (auto obj : BodyList)
		delete obj;
	for (auto obj : RodList)
		delete obj;
	for (auto obj : PointList)
		delete obj;
	for (auto obj : LineList)
		delete obj;

	delete _log;
}

moordyn::error_id
moordyn::MoorDyn::Init(const double* x, const double* xd, bool skip_ic)
{
	if (NCoupledDOF() && !x) {
		LOGERR << "ERROR: "
		       << "MoorDyn::Init received a Null position vector, "
		       << "but " << NCoupledDOF() << "components are required" << endl;
	}

	// <<<<<<<<< need to add bodys

	// Allocate past line fairlead tension array, which is used for convergence
	// test during IC gen
	const unsigned int convergence_iters = 10;
	vector<real> FairTensLast_col(convergence_iters, 0.0);
	for (unsigned int i = 0; i < convergence_iters; i++)
		FairTensLast_col[i] = 1.0 * i;
	vector<vector<real>> FairTensLast(LineList.size(), FairTensLast_col);

	// ------------------ do static bodies and lines ---------------------------

	LOGMSG << "Creating mooring system..." << endl;

	// call ground body to update all the fixed things...
	GroundBody->initializeUnfreeBody();

	// intialize fixed bodies and attached objects
	for (auto l : FixedBodyIs){
		BodyList[l]->initializeUnfreeBody(BodyList[l]->body_r6, vec6::Zero());
	}

	// initialize coupled objects based on passed kinematics
	int ix = 0;

	for (auto l : CpldBodyIs) {
		LOGMSG << "Initializing coupled Body " << l << " in " << x[ix] << ", "
		       << x[ix + 1] << ", " << x[ix + 2] << "..." << endl;
		// this calls initiateStep and updateFairlead, then initializes
		// dependent Rods
		// BUG: These conversions will not be needed in the future
		vec6 r, rd;
		moordyn::array2vec6(x + ix, r);
		moordyn::array2vec6(xd + ix, rd);
		BodyList[l]->initializeUnfreeBody(r, rd);
		ix += 6;
	}

	for (auto l : CpldRodIs) {
		LOGMSG << "Initializing coupled Rod " << l << " in " << x[ix] << ", "
		       << x[ix + 1] << ", " << x[ix + 2] << "..." << endl;
		vec6 r, rd;
		if (RodList[l]->type == Rod::COUPLED) {
			moordyn::array2vec6(x + ix, r);
			moordyn::array2vec6(xd + ix, rd);
			ix += 6; // for cantilevered rods 6 entries will be taken
		} else {
			vec3 r3, rd3;
			moordyn::array2vec(x + ix, r3);
			r(Eigen::seqN(0, 3)) = r3;
			moordyn::array2vec(xd + ix, rd3);
			rd(Eigen::seqN(0, 3)) = rd3;
			ix += 3; // for pinned rods 3 entries will be taken
		}
		RodList[l]->initiateStep(r, rd);
		RodList[l]->updateFairlead(0.0);
		// call this just to set up the output file header
		RodList[l]->initialize();
	}

	for (auto l : CpldPointIs) {
		LOGMSG << "Initializing coupled Point " << l+1 << " at " << x[ix] << ", "
		       << x[ix + 1] << ", " << x[ix + 2] << "..." << endl;
		vec r, rd;
		moordyn::array2vec(x + ix, r);
		moordyn::array2vec(xd + ix, rd);
		PointList[l]->initiateStep(r, rd);

		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			PointList[l]->updateFairlead(0.0);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGERR << "Error initializing coupled point" << l << ": " << err_msg
			       << endl;
			return err;
		}
		// call this just to set WaterKin (may also set up output file in
		// future)
		PointList[l]->initialize();
		ix += 3;
	}

	// Initialize the system state
	_t_integrator->init();

	// ------------------ do dynamic relaxation IC gen --------------------

	if (!skip_ic) {
		LOGMSG << "Finalizing ICs using dynamic relaxation (" << ICDfac
		       << "X normal drag)" << endl;
	}

	// boost drag coefficients to speed static equilibrium convergence
	for (auto obj : LineList)
		obj->scaleDrag(ICDfac);
	for (auto obj : PointList)
		obj->scaleDrag(ICDfac);
	for (auto obj : RodList)
		obj->scaleDrag(ICDfac);
	for (auto obj : BodyList)
		obj->scaleDrag(ICDfac);

	// vector to store tensions for analyzing convergence
	vector<real> FairTens(LineList.size(), 0.0);

	unsigned int iic = 0;
	real t = 0;
	bool converged = true;
	real max_error = 0.0;
	unsigned int max_error_line = 0;
	// The function is enclosed in parenthesis to avoid Windows min() and max()
	// macros break it
	// See
	// https://stackoverflow.com/questions/1825904/error-c2589-on-stdnumeric-limitsdoublemin
	real best_score = (std::numeric_limits<real>::max)();
	real best_score_t = 0.0;
	unsigned int best_score_line = 0;
	while ((t < ICTmax) && (!skip_ic)) {
		// Integrate one ICD timestep (ICdt)
		real t_target = ICdt;
		real dt;
		_t_integrator->Next();
		while ((dt = t_target) > 0.0) {
			if (dtM0 < dt)
				dt = dtM0;
			moordyn::error_id err = MOORDYN_SUCCESS;
			string err_msg;
			try {
				_t_integrator->Step(dt);
				t = _t_integrator->GetTime();
				t_target -= dt;
			}
			MOORDYN_CATCHER(err, err_msg);
			if (err != MOORDYN_SUCCESS) {
				LOGERR << "t = " << t << " s" << endl;
				return err;
			}
		}

		// Roll previous fairlead tensions for comparison
		for (unsigned int lf = 0; lf < LineList.size(); lf++) {
			for (int pt = convergence_iters - 1; pt > 0; pt--)
				FairTensLast[lf][pt] = FairTensLast[lf][pt - 1];
			FairTensLast[lf][0] = FairTens[lf];
		}

		// go through points to get fairlead forces
		for (unsigned int lf = 0; lf < LineList.size(); lf++)
			FairTens[lf] =
			    LineList[lf]->getNodeTen(LineList[lf]->getN()).norm();

		// check for convergence (compare current tension at each fairlead with
		// previous convergence_iters-1 values)
		if (iic > convergence_iters) {
			// check for any non-convergence, and continue to the next time step
			// if any occurs
			converged = true;
			max_error = 0.0;
			for (unsigned int lf = 0; lf < LineList.size(); lf++) {
				for (unsigned int pt = 0; pt < convergence_iters; pt++) {
					const real error =
					    abs(FairTens[lf] / FairTensLast[lf][pt] - 1.0);
					if (error > max_error) {
						max_error = error;
						max_error_line = LineList[lf]->number;
					}
				}
			}
			if (max_error < best_score) {
				best_score = max_error;
				best_score_t = t;
				best_score_line = max_error_line;
			}
			if (max_error > ICthresh) {
				converged = false;
				LOGDBG << "Dynamic relaxation t = " << t << "s (time step "
				       << iic << "), error = " << 100.0 * max_error
				       << "% on line " << max_error_line << "     \r";
			}

			if (converged)
				break;
		}

		iic++;
	}

	if (!skip_ic) {
		if (converged) {
			LOGMSG << "Fairlead tensions converged" << endl;
		} else {
			LOGWRN << "Fairlead tensions did not converged" << endl;
		}
		LOGMSG << "Remaining error after " << t << " s = " << 100.0 * max_error
		       << "% on line " << max_error_line << endl;
		if (!converged) {
			LOGMSG << "Best score at " << best_score_t
			       << " s = " << 100.0 * best_score << "% on line "
			       << best_score_line << endl;
		}
	}

	// restore drag coefficients to normal values and restart time counter of
	// each object
	_t_integrator->SetTime(0.0);
	for (auto obj : LineList) {
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : PointList)
		obj->scaleDrag(1.0 / ICDfac);
	for (auto obj : RodList) {
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : BodyList)
		obj->scaleDrag(1.0 / ICDfac);

	// store passed WaveKin value to enable waves in simulation if applicable
	// (they're not enabled during IC gen)
	env->waterKinOptions.waveMode = WaveKinTemp;
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		// TODO - figure out how i want to do this better
		// because this is horrible. the solution is probably to move EnvCond
		// to its own .hpp and .cpp file so that it can contain the Seafloor and
		// can itself be queries about the seafloor in general
		real tmp = env->WtrDpth;
		if (seafloor) {
			env->WtrDpth = -seafloor->getAverageDepth();
		}
		waves->setup(env, seafloor, _t_integrator, _basepath.c_str());
		env->WtrDpth = tmp;
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS)
		return err;

	// @mth: new approach to be implemented
	// ------------------------- calculate wave time series if needed
	// -------------------
	// 	if (env->WaveKin == 2)
	// 	{
	// 		for (int l=0; l<LineList.size(); l++)
	// 			LineList[l]->makeWaveKinematics( 0.0 );
	// 	}

	// -------------------------- start main output file
	// --------------------------------

	stringstream oname;
	oname << _basepath << _basename << ".out";

	outfileMain.open(oname.str());
	if (!outfileMain.is_open()) {
		LOGERR << "ERROR: Unable to write to main output file " << oname.str()
		       << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}

	// --- channel titles ---
	outfileMain << "Time"
	            << "\t ";
	for (auto channel : outChans)
		outfileMain << channel.Name << "\t ";
	outfileMain << endl;

	if (env->WriteUnits > 0) {
		// --- units ---
		outfileMain << "(s)"
		            << "\t ";
		for (auto channel : outChans)
			outfileMain << channel.Units << "\t ";
		outfileMain << "\n";
	}

	// write t=0 output
	return AllOutput(0.0, 0.0);
}

moordyn::error_id DECLDIR
moordyn::MoorDyn::Step(const double* x,
                       const double* xd,
                       double* f,
                       double& t,
                       double& dt)
{
	// should check if wave kinematics have been set up if expected!
	LOGDBG << "t = " << t << "s     \r";

	if (dt <= 0) {
		// Nothing to do, just recover the forces if there are coupled DOFs
		if (NCoupledDOF())
			return GetForces(f);
		else
			return MOORDYN_SUCCESS;
	}

	if (NCoupledDOF() && (!x || !xd || !f)) {
		LOGERR << "Null Pointer received in " << __FUNC_NAME__ << " ("
		       << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;
	}

	unsigned int ix = 0;

	// ---------------- set positions and velocities -----------------------
	// ... of any coupled bodies, rods, and points at this instant, to be
	// used later for extrapolating motions
	for (auto l : CpldBodyIs) {
		// BUG: These conversions will not be needed in the future
		vec6 r, rd;
		moordyn::array2vec6(x + ix, r);
		moordyn::array2vec6(xd + ix, rd);
		BodyList[l]->initiateStep(r, rd);
		ix += 6;
	}
	for (auto l : CpldRodIs) {
		vec6 r, rd;
		if (RodList[l]->type == Rod::COUPLED) {
			// for cantilevered rods 6 entries will be taken
			moordyn::array2vec6(x + ix, r);
			moordyn::array2vec6(xd + ix, rd);
			ix += 6;
		} else {
			// for pinned rods 3 entries will be taken
			vec3 r3, rd3;
			moordyn::array2vec(x + ix, r3);
			r(Eigen::seqN(0, 3)) = r3;
			moordyn::array2vec(xd + ix, rd3);
			rd(Eigen::seqN(0, 3)) = rd3;
			ix += 3;
		}
		RodList[l]->initiateStep(r, rd);
	}
	for (auto l : CpldPointIs) {
		vec r, rd;
		moordyn::array2vec(x + ix, r);
		moordyn::array2vec(xd + ix, rd);
		PointList[l]->initiateStep(r, rd);
		ix += 3;
	}

	// -------------------- do time stepping -----------------------
	real t_target = dt;
	real dt_step;
	_t_integrator->Next();
	while ((dt_step = t_target) > 0.0) {
		if (dtM0 < dt_step)
			dt_step = dtM0;
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			_t_integrator->Step(dt_step);
			t = _t_integrator->GetTime();
			t_target -= dt_step;
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGERR << "t = " << t << " s: " << err_msg << endl;
			return err;
		}
	}

	// --------------- check for line failures (detachments!) ----------------
	// step 1: check for time-triggered failures
	for (unsigned int l = 0; l < FailList.size(); l++) {
		auto failure = FailList[l];
		if (failure->status || (failure->time < t))
			continue;
		LOGMSG << "Failure number " << l + 1 << " triggered at time " << t
		       << " s" << endl;
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			detachLines(failure);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
			return err;
	}

	// step 2: check for tension-triggered failures (this will require
	// specifying max tension things)

	// ------------------------ write outputs --------------------------
	const moordyn::error_id err = AllOutput(t, dt);
	if (err != MOORDYN_SUCCESS)
		return err;

	// recover the forces if there are coupled DOFs
	if (NCoupledDOF())
		return GetForces(f);
	else
		return MOORDYN_SUCCESS;
}

std::vector<uint64_t>
MoorDyn::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	data.push_back(io::IO::Serialize((uint64_t)npW));

	// Ask to save the data off all the subinstances
	subdata = _t_integrator->Serialize();
	data.insert(data.end(), subdata.begin(), subdata.end());

	for (auto body : BodyList) {
		subdata = body->Serialize();
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	for (auto rod : RodList) {
		subdata = rod->Serialize();
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	for (auto point : PointList) {
		subdata = point->Serialize();
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	for (auto line : LineList) {
		subdata = line->Serialize();
		data.insert(data.end(), subdata.begin(), subdata.end());
	}

	return data;
}

uint64_t*
MoorDyn::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	uint64_t n;
	ptr = io::IO::Deserialize(ptr, n);
	npW = n;

	// Load the children data also
	ptr = _t_integrator->Deserialize(ptr);
	for (auto body : BodyList) {
		ptr = body->Deserialize(ptr);
	}
	for (auto rod : RodList) {
		ptr = rod->Deserialize(ptr);
	}
	for (auto point : PointList) {
		ptr = point->Deserialize(ptr);
	}
	for (auto line : LineList) {
		ptr = line->Deserialize(ptr);
	}

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkMultiBlockDataSet>
MoorDyn::getVTK() const
{
	auto out = vtkSmartPointer<vtkMultiBlockDataSet>::New();
	out->SetNumberOfBlocks(static_cast<unsigned int>(
	    RodList.size() + PointList.size() + LineList.size()));
	unsigned int n = 0;
	for (unsigned int i = 0; i < BodyList.size(); i++)
		out->SetBlock(n + i, BodyList[i]->getVTK());
	n += ui_size(BodyList);
	for (unsigned int i = 0; i < PointList.size(); i++)
		out->SetBlock(n + i, PointList[i]->getVTK());
	n += ui_size(PointList);
	for (unsigned int i = 0; i < RodList.size(); i++)
		out->SetBlock(n + i, RodList[i]->getVTK());
	n += ui_size(RodList);
	for (unsigned int i = 0; i < LineList.size(); i++)
		out->SetBlock(n + i, LineList[i]->getVTK());
	return out;
}

void
MoorDyn::saveVTK(const char* filename) const
{
	auto obj = this->getVTK();
	auto writer = vtkSmartPointer<vtkXMLMultiBlockDataWriter>::New();
	writer->SetFileName(filename);
	writer->SetInputData(obj);
	writer->SetDataModeToBinary();
	writer->Update();
	writer->Write();
	auto err = io::vtk_error(writer->GetErrorCode());
	if (err != MOORDYN_SUCCESS) {
		LOGERR << "VTK reported an error while writing the VTM file '"
		       << filename << "'" << endl;
		MOORDYN_THROW(err, "vtkXMLMultiBlockDataWriter reported an error");
	}
}
#endif

moordyn::error_id
moordyn::MoorDyn::ReadInFile()
{
	unsigned int i = 0;

	// We are really interested in looking for the writeLog option, to start
	// logging as soon as possible
	vector<string> in_txt;
	if (readFileIntoBuffers(in_txt) != MOORDYN_SUCCESS) {
		// BREAK
		return MOORDYN_INVALID_INPUT_FILE;
	}
	// Skip until we find the options header line
	if ((i = findStartOfSection(in_txt, { "OPTIONS" })) != -1) {
		LOGDBG << "   Reading options:" << endl;
		// Parse options until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');
			if (entries.size() < 2) {
				i++;
				continue;
			}
			const string value = entries[0];
			const string name = entries[1];

			if (name == "writeLog") {
				env->writeLog = atoi(entries[0].c_str());
				const moordyn::error_id err = SetupLog();
				if (err != MOORDYN_SUCCESS)
					return err;
				i++;

			} else {
				readOptionsLine(in_txt, i);
				i++;
			}
		}
	}

	// make a "ground body" that will be the parent of all fixed objects
	// (points and rods)
	LOGDBG << "Creating the ground body of type " << Body::TypeName(Body::FIXED)
	       << "..." << endl;
	// GroundBody always get id of zero
	GroundBody = new Body(_log, 0);
	GroundBody->setup(0,
	                  Body::FIXED,
	                  vec6::Zero(),
	                  vec::Zero(),
	                  0.0,
	                  0.0,
	                  vec::Zero(),
	                  vec6::Zero(),
	                  vec6::Zero(),
	                  env,
	                  NULL);

	// Make sure the state vector counter starts at zero
	// This will be conveniently incremented as each object is added
	nX = 0;

	// Now we can parse the whole input file
	if ((i = findStartOfSection(in_txt, { "LINE DICTIONARY", "LINE TYPES" })) !=
	    -1) {
		// look for the section header line
		LOGDBG << "   Reading line types:" << endl;

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			LineProps* obj = readLineProps(in_txt[i]);
			if (obj)
				LinePropList.push_back(obj);
			else {
				delete obj; // Skips error lines... TODO update this
				return MOORDYN_INVALID_INPUT;
			}
			i++;
		}
	}

	if ((i = findStartOfSection(in_txt, { "ROD DICTIONARY", "ROD TYPES" })) !=
	    -1) {
		LOGDBG << "   Reading rod types:" << endl;

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			RodProps* obj = readRodProps(in_txt[i]);

			if (obj)
				RodPropList.push_back(obj);
			else {
				delete obj;
				return MOORDYN_INVALID_INPUT;
			}
			i++;
		}
	}

	if ((i = findStartOfSection(
	         in_txt, { "BODIES", "BODY LIST", "BODY PROPERTIES" })) != -1) {
		LOGDBG << "   Reading body list:" << endl;

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			Body* obj = readBody(in_txt[i]);

			if (obj) {
				BodyList.push_back(obj);
			} else {
				delete obj;
				return MOORDYN_INVALID_INPUT;
			}
			i++;
		}
	}

	if ((i = findStartOfSection(in_txt,
	                            { "POINTS",
	                              "POINT LIST",
	                              "POINT PROPERTIES",
	                              "CONNECTION PROPERTIES",
	                              "NODE PROPERTIES" })) != -1) {
		LOGDBG << "   Reading point list:" << endl;

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');
			if (entries.size() < 9) {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "9 fields are required, but just " << entries.size()
				       << " are provided" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			int number = atoi(entries[0].c_str());
			double M = atof(entries[5].c_str());
			double V = atof(entries[6].c_str());
			double CdA;
			double Ca;
			vec r0;
			vec F = vec::Zero();
			if (entries.size() >=
			    12) // case with optional force inputs (12 total entries)
			{
				for (int I = 0; I < 3; I++) {
					r0[I] = atof(entries[2 + I].c_str());
					F[I] = atof(entries[7 + I].c_str());
				}
				CdA = atof(entries[10].c_str());
				Ca = atof(entries[11].c_str());
			} else // case without optional force inputs (9 total entries)
			{
				for (int I = 0; I < 3; I++)
					r0[I] = atof(entries[2 + I].c_str());

				CdA = atof(entries[7].c_str());
				Ca = atof(entries[8].c_str());
			}

			Point::types type;
			std::string let1, num1, let2, num2, let3;
			// divided outWord into letters and numbers
			str::decomposeString(entries[1], let1, num1, let2, num2, let3);
			if (str::isOneOf(let1, { "ANCHOR", "FIXED", "FIX" })) {
				// it is fixed  (this would just be used if someone wanted
				// to temporarly fix a body that things were attached to)
				type = Point::FIXED;
			} else if (let1 == "BODY") {
				type = Point::FIXED;
				if (num1.empty()) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "no number provided for Rod " << number
					       << " Body attachment" << endl;
					return MOORDYN_INVALID_INPUT;
				}
				unsigned int bodyID = atoi(num1.c_str());
				if (!bodyID || (bodyID > BodyList.size())) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "There is not " << bodyID << " bodies" << endl;
					return MOORDYN_INVALID_INPUT;
				}
			} else if (str::isOneOf(let1,
			                        { "FAIRLEAD",
			                          "VESSEL",
			                          "VES",
			                          "COUPLED",
			                          "CPLD" })) {
				// if a fairlead, add to list and add
				type = Point::COUPLED;
				CpldPointIs.push_back(ui_size(PointList));
			} else if (str::isOneOf(let1,
			                        { "POINT", "CONNECT", "CON", "FREE" })) {
				// if a point, add to list and add states for it
				type = Point::FREE;
				FreePointIs.push_back(ui_size(PointList));
				PointStateIs.push_back(
				    nX); // assign start index of this point's states
				nX += 6; // add 6 state variables for each point
			} else {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "Unrecognized point type '" << let1 << "'" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			// make default water depth at least the depth of the lowest
			// node (so water depth input is optional)
			// TODO - this probably doesn't care about 3d seafloor?
			if (r0[2] < -env->WtrDpth)
				env->WtrDpth = -r0[2];

			LOGDBG << "\t'" << number << "'"
			       << " - of type " << Point::TypeName(type) << " with id "
			       << PointList.size() << endl;

			// now make Point object!
			Point* obj = new Point(_log, PointList.size());
			obj->setup(number, type, r0, M, V, F, CdA, Ca, env);
			PointList.push_back(obj);

			// depending on type, assign the Point to its respective
			// parent body
			if (str::isOneOf(let1, { "ANCHOR", "FIXED", "FIX" }))
				GroundBody->addPoint(obj, r0);
			else if (let1 == "BODY") {
				int bodyID = stoi(num1);
				BodyList[bodyID - 1]->addPoint(obj, r0);
			}
			LOGDBG << endl;

			i++;
		}
	}

	if ((i = findStartOfSection(
	         in_txt, { "RODS", "ROD LIST", "ROD PROPERTIES" })) != -1) {
		LOGDBG << "   Reading rod list:" << endl;

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			Rod* obj = readRod(in_txt[i]);
			RodList.push_back(obj);

			i++;
		}
	}

	if ((i = findStartOfSection(
	         in_txt, { "LINES", "LINE LIST", "LINE PROPERTIES" })) != -1) {
		LOGDBG << "   Reading line list: " << endl;

		if (!LinePropList.size()) {
			LOGERR << "Reading lines without defined line types" << endl;
			return MOORDYN_INVALID_INPUT;
		}
		if (!PointList.size()) {
			LOGERR << "Reading lines without defined points" << endl;
			return MOORDYN_INVALID_INPUT;
		}

		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');
			if (entries.size() < 7) {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "7 fields are required, but only " << entries.size()
				       << " are provided" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			int number = atoi(entries[0].c_str());
			string type = entries[1];
			double UnstrLen = atof(entries[4].c_str());
			int NumSegs = atoi(entries[5].c_str()); // addition vs. MAP
			string outchannels = entries[6];

			int TypeNum = -1;
			for (unsigned int J = 0; J < LinePropList.size(); J++) {
				if (LinePropList[J]->type == type)
					TypeNum = J;
			}
			if (TypeNum == -1) {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "Unrecognized line type " << type << endl;
				return MOORDYN_INVALID_INPUT;
			}

			// Make the output file (if queried)
			if ((outchannels.size() > 0) &&
			    (strcspn(outchannels.c_str(), "pvUDctsd") <
			     strlen(outchannels.c_str()))) {
				// if 1+ output flag chars are given and they're valid
				stringstream oname;
				oname << _basepath << _basename << "_Line" << number << ".out";
				outfiles.push_back(make_shared<ofstream>(oname.str()));
				if (!outfiles.back()->is_open()) {
					LOGERR << "Cannot create the output file '" << oname.str()
					       << endl;
					return MOORDYN_INVALID_OUTPUT_FILE;
				}
			} else
				outfiles.push_back(NULL);

			LOGDBG << "\t'" << number << "'"
			       << " - of class " << type << " (" << TypeNum << ")"
			       << " with id " << LineList.size() << endl;

			Line* obj = new Line(_log, LineList.size());
			obj->setup(number,
			           LinePropList[TypeNum],
			           UnstrLen,
			           NumSegs,
			           env,
			           outfiles.back(),
			           outchannels);
			LineList.push_back(obj);
			LineStateIs.push_back(
			    nX);                 // assign start index of this Line's states
			nX += 6 * (NumSegs - 1); // add 6 state variables for each
			                         // internal node of this line

			for (unsigned int I = 0; I < 2; I++) {
				const EndPoints end_point = I == 0 ? ENDPOINT_A : ENDPOINT_B;
				std::string let1, num1, let2, num2, let3;
				// divided outWord into letters and numbers
				str::decomposeString(
				    entries[2 + I], let1, num1, let2, num2, let3);

				if (num1.empty()) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "No number provided for the 1st point index"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}
				unsigned int id = atoi(num1.c_str());

				if (str::isOneOf(let1, { "R", "ROD" })) {
					if (!id || id > RodList.size()) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There are not " << id << " rods" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					if (let2 == "A")
						RodList[id - 1]->addLine(obj, end_point, ENDPOINT_A);
					else if (let2 == "B")
						RodList[id - 1]->addLine(obj, end_point, ENDPOINT_B);
					else {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "Rod end (A or B) must be specified" << endl;
						return MOORDYN_INVALID_INPUT;
					}
				} else if (let1.empty() ||
				           str::isOneOf(let1, { "C", "CON", "P", "POINT" })) {
					if (!id || id > PointList.size()) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There are not " << id << " points" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					PointList[id - 1]->addLine(obj, end_point);
				} else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Unrecognized point type " << let1 << endl;
					return MOORDYN_INVALID_INPUT;
				}
			}
			LOGDBG << endl;

			i++;
		}
	}

	if ((i = findStartOfSection(in_txt, { "FAILURE" })) != -1) {
		LOGDBG << "   Reading failure conditions:" << endl;
		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');
			if (entries.size() < 4) {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "4 fields are required, but just " << entries.size()
				       << " are provided" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			FailProps* obj = new FailProps();
			obj->rod = NULL;
			obj->point = NULL;
			obj->status = false;
			FailList.push_back(obj);

			std::string let1, num1, let2, num2, let3;
			// divided outWord into letters and numbers
			str::decomposeString(entries[0], let1, num1, let2, num2, let3);

			if (num1.empty()) {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "No number provided for Node Failure" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			const unsigned int id = atoi(num1.c_str());
			if (str::isOneOf(let1, { "R", "ROD" })) {
				if (!id || id > RodList.size()) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "There are not " << id << " rods" << endl;
					return MOORDYN_INVALID_INPUT;
				}
				obj->rod = RodList[id - 1];
				if (let2 == "A")
					obj->rod_end_point = ENDPOINT_A;
				else if (let2 == "B")
					obj->rod_end_point = ENDPOINT_B;
				else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Failure end (A or B) must be specified" << endl;
					return MOORDYN_INVALID_INPUT;
				}
			} else if (let1.empty() ||
			           str::isOneOf(let1, { "C", "CON", "P", "POINT" })) {
				if (!id || id > PointList.size()) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "There are not " << id << " points" << endl;
					return MOORDYN_INVALID_INPUT;
				}
				obj->point = PointList[id - 1];
				;
			} else {
				LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
				       << endl
				       << "'" << in_txt[i] << "'" << endl
				       << "Unrecognized point type " << let1 << endl;
				return MOORDYN_INVALID_INPUT;
			}

			vector<string> lineNums = moordyn::str::split(entries[1], ',');
			obj->lines.reserve(lineNums.size());
			obj->line_end_points.reserve(lineNums.size());
			for (unsigned int il = 0; il < lineNums.size(); il++) {
				const unsigned int line_id = atoi(lineNums[il].c_str());
				if (!line_id || line_id > LineList.size()) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "There are not " << line_id << " lines" << endl;
					return MOORDYN_INVALID_INPUT;
				}
				obj->lines.push_back(LineList[line_id - 1]);
				obj->line_end_points.push_back(ENDPOINT_A);
			}

			obj->time = atof(entries[2].c_str());
			obj->ten = atof(entries[3].c_str());

			LOGDBG << "fail time is " << obj->time << " s" << endl;
			LOGDBG << "fail ten is " << obj->ten << " N" << endl;

			i++;
		}
	}

	// Options read in at start

	if ((i = findStartOfSection(in_txt, { "OUTPUT" })) != -1) {
		LOGDBG << "   Reading output options:" << endl;
		// parse until the next header or the end of the file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');

			for (unsigned int j = 0; j < entries.size();
			     j++) // loop through each word on each line
			{
				std::string let1, num1, let2, num2, let3;
				// divided outWord into letters and numbers
				str::decomposeString(entries[j], let1, num1, let2, num2, let3);

				// declare dummy struct to be copied onto end of vector (and
				// filled in later);
				OutChanProps dummy;
				dummy.Name = entries[j];

				// figure out what type of output it is and process
				// accordingly
				// TODO: add checks of first char of num1,2, let1,2,3 not
				// being NULL to below and handle errors (e.g. invalid line
				// number)

				// fairlead tension case (changed to just be for single
				// line, not all connected lines)
				if (let1 == "FAIRTEN") {
					dummy.OType = 1;
					dummy.QType = Ten;
					dummy.Units = moordyn::UnitList[Ten];
					dummy.ObjID = atoi(num1.c_str());
					dummy.NodeID = LineList[dummy.ObjID - 1]->getN();
				}
				// achor tension case (changed to just be for single line,
				// not all connected lines)
				else if (let1 == "ANCHTEN") {
					dummy.OType = 1;
					dummy.QType = Ten;
					dummy.Units = moordyn::UnitList[Ten];
					dummy.ObjID = atoi(num1.c_str());
					dummy.NodeID = 0;
				}
				// more general case
				else {

					// object number
					dummy.ObjID = atoi(num1.c_str());

					// get object type and node number if applicable
					// Line case:  L?N?xxxx
					if (str::isOneOf(let1, { "L", "LINE" })) {
						dummy.OType = 1;
						if (let3.empty()) {
							if (let2.substr(0, 2) == "NA") {
								dummy.NodeID = 0;
								let2.erase(0, 2);
							} else if (let2.substr(0, 2) == "NB") {
								dummy.NodeID =
								    LineList[dummy.ObjID - 1]->getN();
								let2.erase(0, 2);
							} else if (num2.empty())
								dummy.NodeID = 0;
							else {
								LOGWRN << "Warning in " << _filepath << ":"
								       << i + 1 << "..." << endl
								       << "'" << in_txt[i] << "'" << endl
								       << "invalid output specifier: " << let1
								       << ".  Line ID or Node ID missing."
								       << endl;
								dummy.OType = -1;
							}
						} else
							dummy.NodeID = atoi(num2.c_str());
					}
					// Point case:   P?xxx or Point?xxx
					else if (str::isOneOf(let1, { "P", "POINT" })) {
						dummy.OType = 2;
						dummy.NodeID = -1;
					}
					// Rod case:   R?xxx or Rod?xxx
					else if (str::isOneOf(let1, { "R", "ROD" })) {
						dummy.OType = 3;
						if (let3.empty()) {
							if (let2.substr(0, 2) == "NA") {
								dummy.NodeID = 0;
								let2.erase(0, 2);
							} else if (let2.substr(0, 2) == "NB") {
								dummy.NodeID = RodList[dummy.ObjID - 1]->getN();
								let2.erase(0, 2);
							} else
								dummy.NodeID = -1;
						} else if (!num2.empty())
							dummy.NodeID = atoi(num2.c_str());
						else {
							LOGWRN << "Warning in " << _filepath << ":" << i + 1
							       << "..." << endl
							       << "'" << in_txt[i] << "'" << endl
							       << "invalid output specifier: " << let1
							       << ".  Rod ID or Node ID missing." << endl;
							dummy.OType = -1;
						}
					}
					// Body case:   B?xxx or Body?xxx
					else if (str::isOneOf(let1, { "B", "BODY" })) {
						dummy.OType = 4;
						dummy.NodeID = -1;
					}
					// should do fairlead option also!
					else {
						LOGWRN
						    << "Warning in " << _filepath << ":" << i + 1
						    << "..." << endl
						    << "'" << in_txt[i] << "'" << endl
						    << "invalid output specifier: " << let1
						    << ".  Type must be oneof L/Line, P/Point, R/Rod, "
						       "or B/Body"
						    << endl;
						dummy.OType = -1;
						continue;
					}

					if (let3.empty())
						let3 = let2;

					if (let3 == "PX") {
						dummy.QType = PosX;
						dummy.Units = moordyn::UnitList[PosX];
					} else if (let3 == "PY") {
						dummy.QType = PosY;
						dummy.Units = moordyn::UnitList[PosY];
					} else if (let3 == "PZ") {
						dummy.QType = PosZ;
						dummy.Units = moordyn::UnitList[PosZ];
					} else if (let3 == "RX") {
						dummy.QType = RX;
						dummy.Units = moordyn::UnitList[RX];
					} else if (let3 == "RY") {
						dummy.QType = RY;
						dummy.Units = moordyn::UnitList[RY];
					} else if (let3 == "RZ") {
						dummy.QType = RZ;
						dummy.Units = moordyn::UnitList[RZ];
					} else if (let3 == "VX") {
						dummy.QType = VelX;
						dummy.Units = moordyn::UnitList[VelX];
					} else if (let3 == "VY") {
						dummy.QType = VelY;
						dummy.Units = moordyn::UnitList[VelY];
					} else if (let3 == "VZ") {
						dummy.QType = VelZ;
						dummy.Units = moordyn::UnitList[VelZ];
					} else if (let3 == "RVX") {
						dummy.QType = RVelX;
						dummy.Units = moordyn::UnitList[RVelX];
					} else if (let3 == "RVY") {
						dummy.QType = RVelY;
						dummy.Units = moordyn::UnitList[RVelY];
					} else if (let3 == "RVZ") {
						dummy.QType = RVelZ;
						dummy.Units = moordyn::UnitList[RVelZ];
					} else if (let3 == "AX") {
						dummy.QType = AccX;
						dummy.Units = moordyn::UnitList[AccX];
					} else if (let3 == "Ay") {
						dummy.QType = AccY;
						dummy.Units = moordyn::UnitList[AccY];
					} else if (let3 == "AZ") {
						dummy.QType = AccZ;
						dummy.Units = moordyn::UnitList[AccZ];
					} else if (let3 == "RAX") {
						dummy.QType = RAccX;
						dummy.Units = moordyn::UnitList[RAccX];
					} else if (let3 == "RAY") {
						dummy.QType = RAccY;
						dummy.Units = moordyn::UnitList[RAccY];
					} else if (let3 == "RAZ") {
						dummy.QType = RAccZ;
						dummy.Units = moordyn::UnitList[RAccZ];
					} else if (let3 == "T" || let3 == "TEN") {
						dummy.QType = Ten;
						dummy.Units = moordyn::UnitList[Ten];
					} else if (let3 == "TA" || let3 == "TENA") {
						dummy.QType = TenA;
						dummy.Units = moordyn::UnitList[Ten];
					} else if (let3 == "TB" || let3 == "TENB") {
						dummy.QType = TenB;
						dummy.Units = moordyn::UnitList[Ten];
					} else if (let3 == "FX") {
						dummy.QType = FX;
						dummy.Units = moordyn::UnitList[FX];
					} else if (let3 == "FY") {
						dummy.QType = FY;
						dummy.Units = moordyn::UnitList[FY];
					} else if (let3 == "FZ") {
						dummy.QType = FZ;
						dummy.Units = moordyn::UnitList[FZ];
					} else if (let3 == "MX") {
						dummy.QType = MX;
						dummy.Units = moordyn::UnitList[MX];
					} else if (let3 == "MY") {
						dummy.QType = MY;
						dummy.Units = moordyn::UnitList[MY];
					} else if (let3 == "MZ") {
						dummy.QType = MZ;
						dummy.Units = moordyn::UnitList[MZ];
					} else if (let3 == "SUB") {
						dummy.QType = Sub;
						dummy.Units = moordyn::UnitList[Sub];
					} else {
						LOGWRN << "Warning in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "invalid quantity specifier: " << let3
						       << endl;
						dummy.OType = -1;
						continue;
					}
				}

				// some name adjusting for special cases (maybe should
				// handle this elsewhere...)
				if ((dummy.OType == 3) && (dummy.QType == Ten)) {
					if (dummy.NodeID > 0) {
						dummy.Name = "TenEndB";
					} else {
						dummy.Name = "TenEndA";
					}
				}

				if ((dummy.OType > 0) && (dummy.QType > 0))
					outChans.push_back(dummy);
			}

			i++;
		}
	}

	// do some input validity checking?
	// should there be a flag in the input file that clearly distingiushes
	// the coupling type?
	// <<<<< I guess it's implied by whether bodies are coupled or not??

	// TODO: make sure things are consistent for only ONE coupling type
	// (body centric or fairlead centric)
	// <<<<<<<<<<<<<<<< also do checks when time step function is called...

	LOGMSG << "Generated entities:" << endl
	       << "\tnLineTypes  = " << LinePropList.size() << endl
	       << "\tnRodTypes   = " << RodPropList.size() << endl
	       << "\tnPoints     = " << PointList.size() << endl
	       << "\tnBodies     = " << BodyList.size() << endl
	       << "\tnRods       = " << RodList.size() << endl
	       << "\tnLines      = " << LineList.size() << endl
	       << "\tnFails      = " << FailList.size() << endl
	       << "\tnFreeBodies = " << FreeBodyIs.size() << endl
	       << "\tnFreeRods   = " << FreeRodIs.size() << endl
	       << "\tnFreePoints  = " << FreePointIs.size() << endl
	       << "\tnCpldBodies = " << CpldBodyIs.size() << endl
	       << "\tnCpldRods   = " << CpldRodIs.size() << endl
	       << "\tnCpldPoints = " << CpldPointIs.size() << endl;

	// write system description
	LOGDBG << "----- MoorDyn Model Summary (to be written) -----" << endl;

	// Setup the time integrator
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	if (!_t_integrator) {
		try {
			_t_integrator = create_time_scheme("RK2", _log, waves);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGERR << err_msg << endl;
			return err;
		}
	}
	LOGMSG << "Time integrator = " << _t_integrator->GetName() << endl;
	_t_integrator->SetGround(GroundBody);
	for (auto obj : BodyList)
		_t_integrator->AddBody(obj);
	for (auto obj : RodList)
		_t_integrator->AddRod(obj);
	for (auto obj : PointList)
		_t_integrator->AddPoint(obj);
	for (auto obj : LineList)
		_t_integrator->AddLine(obj);

	// Setup the waves and populate them
	try {
		// TODO - figure out how i want to do this better
		// because this is horrible. the solution is probably to move EnvCond
		// to its own .hpp and .cpp file so that it can contain the Seafloor and
		// can itself be queries about the seafloor in general
		real tmp = env->WtrDpth;
		if (seafloor) {
			env->WtrDpth = -seafloor->getAverageDepth();
		}
		waves->setup(env, seafloor, _t_integrator, _basepath.c_str());
		env->WtrDpth = tmp;
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS)
		return err;

	GroundBody->setWaves(waves);
	waves->addBody(GroundBody);
	for (auto obj : BodyList) {
		obj->setWaves(waves);
		waves->addBody(obj);
	}
	for (auto obj : RodList) {
		obj->setWaves(waves, seafloor);
		waves->addRod(obj);
	}
	for (auto obj : PointList) {
		obj->setWaves(waves, seafloor);
		waves->addPoint(obj);
	}
	for (auto obj : LineList) {
		obj->setWaves(waves, seafloor);
		waves->addLine(obj);
	}

	return MOORDYN_SUCCESS;
}

moordyn::error_id
moordyn::MoorDyn::readFileIntoBuffers(vector<string>& in_txt)
{
	ifstream in_file(_filepath);
	if (!in_file.is_open()) {
		LOGERR << "Error: unable to open file '" << _filepath << "'\n";
		return MOORDYN_INVALID_INPUT_FILE;
	}

	while (in_file.good()) {
		string line_txt;
		getline(in_file, line_txt);
		moordyn::str::rtrim(line_txt);
		in_txt.push_back(line_txt);
	}
	in_file.close();
	return MOORDYN_SUCCESS;
}

int
moordyn::MoorDyn::findStartOfSection(vector<string>& in_txt,
                                     vector<string> sectionName)
{
	int i = 0;
	while (i < in_txt.size() &&
	       ((in_txt[i].find("---") == string::npos) ||
	        !moordyn::str::has(moordyn::str::upper(in_txt[i]), sectionName))) {
		i++;
	}

	if (i == in_txt.size())
		return -1; // indicates section not found
	if (sectionName[0] == "OPTIONS" || sectionName[0] == "OUTPUT")
		i++; // Increment once to get to the options
	else
		i += 3; // need to also skip label line and unit line

	return i;
}
LineProps*
moordyn::MoorDyn::readLineProps(string inputText)
{
	vector<string> entries = moordyn::str::split(inputText, ' ');

	if (!checkNumberOfEntriesInLine(entries, 10)) {
		return nullptr;
	}

	LineProps* obj = new LineProps();

	obj->type = entries[0];
	obj->d = atof(entries[1].c_str());
	obj->w = atof(entries[2].c_str());
	obj->Cdn = atof(entries[6].c_str());
	obj->Can = atof(entries[7].c_str());
	obj->Cdt = atof(entries[8].c_str());
	obj->Cat = atof(entries[9].c_str());

	moordyn::error_id err;
	err = read_curve(entries[3].c_str(),
	                 &(obj->EA),
	                 &(obj->nEApoints),
	                 obj->stiffXs,
	                 obj->stiffYs);
	if (err)
		return nullptr;
	err = read_curve(entries[4].c_str(),
	                 &(obj->c),
	                 &(obj->nCpoints),
	                 obj->dampXs,
	                 obj->dampYs);
	if (err)
		return nullptr;
	err = read_curve(entries[5].c_str(),
	                 &(obj->EI),
	                 &(obj->nEIpoints),
	                 obj->bstiffXs,
	                 obj->bstiffYs);
	if (err)
		return nullptr;

	LOGDBG << "\t'" << obj->type << "'"
	       << " - with id " << LinePropList.size() << endl
	       << "\t\td   : " << obj->d << endl
	       << "\t\tw   : " << obj->w << endl
	       << "\t\tCdn : " << obj->Cdn << endl
	       << "\t\tCan : " << obj->Can << endl
	       << "\t\tCdt : " << obj->Cdt << endl
	       << "\t\tCat : " << obj->Cat << endl;
	return obj;
}

RodProps*
moordyn::MoorDyn::readRodProps(string inputText)
{
	vector<string> entries = moordyn::str::split(inputText, ' ');
	if (!checkNumberOfEntriesInLine(entries, 7)) {
		return nullptr;
	}

	RodProps* obj = new RodProps();
	obj->type = entries[0];
	obj->d = atof(entries[1].c_str());
	obj->w = atof(entries[2].c_str());
	obj->Cdn = atof(entries[3].c_str());
	obj->Can = atof(entries[4].c_str());
	obj->CdEnd = atof(entries[5].c_str());
	obj->CaEnd = atof(entries[6].c_str());
	obj->Cdt = 0.0;
	obj->Cat = 0.0;

	LOGDBG << "\t'" << obj->type << "'"
	       << " - with id " << RodPropList.size() << endl
	       << "\t\td   : " << obj->d << endl
	       << "\t\tw   : " << obj->w << endl
	       << "\t\tCdn : " << obj->Cdn << endl
	       << "\t\tCan : " << obj->Can << endl
	       << "\t\tCdEnd : " << obj->CdEnd << endl
	       << "\t\tCaEnd : " << obj->CaEnd << endl;
	return obj;
}

Body*
moordyn::MoorDyn::readBody(string inputText)
{
	vector<string> entries = moordyn::str::split(inputText, ' ');
	if (!checkNumberOfEntriesInLine(entries, 14)) {
		LOGERR << "Error in " << _filepath << ":" << '\n';
		return nullptr;
	}

	const int number = atoi(entries[0].c_str());
	Body::types type;
	vec6 r6;
	for (unsigned int I = 0; I < 6; I++)
		r6[I] = atof(entries[2 + I].c_str());
	double M = atof(entries[8].c_str());
	double V = atof(entries[11].c_str());

	vec rCG, Inert;
	vec6 CdA = vec6::Zero();
	vec6 Ca = vec6::Zero();

	vector<string> entries_rCG = moordyn::str::split(entries[9], '|');
	if (entries_rCG.size() == 1) {
		// if only one entry, it is the z coordinate
		rCG[0] = 0.0;
		rCG[1] = 0.0;
		rCG[2] = atof(entries_rCG[0].c_str());
	} else if (entries_rCG.size() == 3) {
		rCG[0] = atof(entries_rCG[0].c_str());
		rCG[1] = atof(entries_rCG[1].c_str());
		rCG[2] = atof(entries_rCG[2].c_str());
	} else {
		LOGERR << "Error in " << _filepath << ":" << endl
		       << "'" << inputText << "'" << endl
		       << "CG entry (col 10) must have 1 or 3 numbers" << endl;
		return nullptr;
	}

	vector<string> entries_I = moordyn::str::split(entries[10], '|');
	if (entries_I.size() == 1) {
		// if only one entry, use it for all directions
		Inert[0] = atof(entries_I[0].c_str());
		Inert[1] = Inert[0];
		Inert[2] = Inert[0];
	} else if (entries_I.size() == 3) {
		Inert[0] = atof(entries_I[0].c_str());
		Inert[1] = atof(entries_I[1].c_str());
		Inert[2] = atof(entries_I[2].c_str());
	} else {
		LOGERR << "Error in " << _filepath << endl
		       << "'" << inputText << "'" << endl
		       << "Inertia entry (col 11) must have 1 or 3 numbers" << endl;
		return nullptr;
	}

	vector<string> entries_CdA = moordyn::str::split(entries[12], '|');
	if (entries_CdA.size() == 1) {
		// if only one entry, use it for all directions
		CdA[0] = atof(entries_CdA[0].c_str());
		for (unsigned int i = 1; i < 6; i++)
			CdA[i] = CdA[0];
	} else if (entries_CdA.size() == 2) {
		CdA[0] = atof(entries_CdA[0].c_str());
		CdA[3] = atof(entries_CdA[1].c_str());
		for (unsigned int i = 1; i < 3; i++) {
			CdA[i] = CdA[0];
			CdA[i + 3] = CdA[3];
		}
	} else if (entries_CdA.size() == 3) {
		for (unsigned int i = 1; i < 3; i++) {
			CdA[i] = atof(entries_CdA[i].c_str());
			CdA[i + 3] = CdA[i];
		}
	} else if (entries_CdA.size() == 6) {
		for (unsigned int i = 1; i < 6; i++) {
			CdA[i] = atof(entries_CdA[i].c_str());
		}
	} else {
		LOGERR << "Error in " << _filepath << endl
		       << "'" << inputText << "'" << endl
		       << "CdA entry (col 13) must have 1, 2, 3 or 6 numbers" << endl;
		return nullptr;
	}

	vector<string> entries_Ca = moordyn::str::split(entries[13], '|');
	if (entries_Ca.size() == 1) {
		// if only one entry, use it for all directions
		Ca[0] = atof(entries_Ca[0].c_str());
		Ca[1] = Ca[0];
		Ca[2] = Ca[0];
	} else if (entries_Ca.size() == 3) {
		Ca[0] = atof(entries_Ca[0].c_str());
		Ca[1] = atof(entries_Ca[1].c_str());
		Ca[2] = atof(entries_Ca[2].c_str());
	} else {
		LOGERR << "Error in " << _filepath << endl
		       << "'"
		       << "'" << endl
		       << "Ca entry (col 14) must have 1 or 3 numbers" << endl;
		return nullptr;
	}

	std::string let1, num1, let2, num2, let3;
	// divided outWord into letters and numbers
	str::decomposeString(entries[1], let1, num1, let2, num2, let3);

	if (str::isOneOf(let1, { "ANCHOR", "FIXED", "FIX" })) {
		// it is fixed  (this would just be used if someone wanted
		// to temporarly fix a body that things were attached to)
		type = Body::FIXED;
		FixedBodyIs.push_back(ui_size(BodyList));
	} else if (str::isOneOf(let1, { "VESSEL", "VES", "COUPLED", "CPLD" })) {
		// it is coupled - controlled from outside
		type = Body::COUPLED;
		CpldBodyIs.push_back(ui_size(BodyList));
	} else {
		// it is free - controlled by MoorDyn
		type = Body::FREE;
		FreeBodyIs.push_back(ui_size(BodyList));
		BodyStateIs.push_back(nX); // assign start index of this body's states
		nX += 12;                  // add 12 state variables for the body
	}
	stringstream oname;
	oname << _basepath << _basename << "_Body" << number << ".out";
	outfiles.push_back(make_shared<ofstream>(oname.str()));
	if (!outfiles.back()->is_open()) {
		LOGERR << "Cannot create the output file '" << oname.str() << endl;
		return nullptr;
	}

	// id = size + 1 because of ground body, which has an Id of zero
	Body* obj = new Body(_log, BodyList.size() + 1);
	LOGDBG << "\t'" << number << "'"
	       << " - of type " << Body::TypeName(type) << " with id "
	       << BodyList.size() << endl;
	obj->setup(
	    number, type, r6, rCG, M, V, Inert, CdA, Ca, env, outfiles.back());
	return obj;
}

Rod*
moordyn::MoorDyn::readRod(string inputText)
{

	vector<string> entries = moordyn::str::split(inputText, ' ');
	if (!checkNumberOfEntriesInLine(entries, 11)) {
		return nullptr;
	}

	int number = atoi(entries[0].c_str());
	string RodType = entries[1];
	vec6 endCoords;
	for (unsigned int I = 0; I < 6; I++)
		endCoords[I] = atof(entries[3 + I].c_str());
	int NumSegs = atoi(entries[9].c_str());
	string outchannels = entries[10];

	std::string let1, num1, let2, num2, let3;
	// divided outWord into letters and numbers
	str::decomposeString(entries[2], let1, num1, let2, num2, let3);

	Rod::types type;
	if (str::isOneOf(let1, { "ANCHOR", "FIXED", "FIX" })) {
		// it is fixed  (this would just be used if someone wanted
		// to temporarly fix a body that things were attached to)
		type = Rod::FIXED;
	} else if (str::isOneOf(let1, { "PINNED", "PIN" })) {
		// it is pinned
		type = Rod::PINNED;
		FreeRodIs.push_back(
		    ui_size(RodList));    // add this pinned rod to the free
		                          // list because it is half free
		RodStateIs.push_back(nX); // assign start index of this rod's states
		nX += 6;                  // add 6 state variables for each pinned rod
	} else if (let1 == "BODY") {
		if (num1.empty()) {
			LOGERR << "Error in " << _filepath << ":"
			       << "'" << inputText << "'" << endl
			       << "no number provided for Rod " << number
			       << " Body attachment" << endl;
			return nullptr;
		}
		unsigned int bodyID = atoi(num1.c_str());
		if (!bodyID || (bodyID > BodyList.size())) {
			LOGERR << "Error in " << _filepath << ":"
			       << "'" << inputText << "'" << endl
			       << "There is not " << bodyID << " bodies" << endl;
			return nullptr;
		}

		if (str::isOneOf(let2, { "PINNED", "PIN" })) {
			// it is pinned
			type = Rod::PINNED;
			FreeRodIs.push_back(
			    ui_size(RodList));    // add this pinned rod to the free
			                          // list because it is half free
			RodStateIs.push_back(nX); // assign start index of this rod's states
			nX += 6; // add 6 state variables for each pinned rod
		} else {
			type = Rod::FIXED;
		}
	} else if (str::isOneOf(let1, { "VESSEL", "VES", "COUPLED", "CPLD" })) {
		// if a rigid fairlead, add to list and add
		type = Rod::COUPLED;
		CpldRodIs.push_back(
		    ui_size(RodList)); // index of fairlead in RodList vector
	} else if (str::isOneOf(let1, { "VESPIN", "CPLDPIN" })) {
		// if a pinned fairlead, add to list and add
		type = Rod::CPLDPIN;
		CpldRodIs.push_back(
		    ui_size(RodList)); // index of fairlead in RodList vector
		FreeRodIs.push_back(
		    ui_size(RodList));    // also add this pinned rod to the free
		                          // list because it is half free
		RodStateIs.push_back(nX); // assign start index of this rod's states
		nX += 6;                  // add 6 state variables for each pinned rod
	} else if (str::isOneOf(let1, { "POINT", "CON", "FREE" })) {
		type = Rod::FREE;
		FreeRodIs.push_back(
		    ui_size(RodList));    // add this free rod to the free list
		RodStateIs.push_back(nX); // assign start index of this rod's states
		nX += 12;                 // add 12 state variables for each free rod
	} else {
		LOGERR << "Error in " << _filepath << ":"
		       << "'" << inputText << "'" << endl
		       << "Unrecognized point type '" << let1 << "'" << endl;
		return nullptr;
	}

	int TypeNum = -1;
	for (unsigned int J = 0; J < RodPropList.size(); J++) {
		if (RodPropList[J]->type == RodType)
			TypeNum = J;
	}
	if (TypeNum == -1) {
		LOGERR << "Error in " << _filepath << ":"
		       << "'" << inputText << "'" << endl
		       << "Unrecognized rod type " << RodType << endl;
		return nullptr;
	}

	// Make the output file (if queried)
	if ((outchannels.size() > 0) && (strcspn(outchannels.c_str(), "pvUDctsd") <
	                                 strlen(outchannels.c_str()))) {
		// if 1+ output flag chars are given and they're valid
		stringstream oname;
		oname << _basepath << _basename << "_Rod" << number << ".out";
		outfiles.push_back(make_shared<ofstream>(oname.str()));
		if (!outfiles.back()->is_open()) {
			LOGERR << "Cannot create the output file '" << oname.str() << endl;
			return nullptr;
		}
	} else
		outfiles.push_back(NULL);

	LOGDBG << "\t'" << number << "'"
	       << " - of class " << RodType << " (" << TypeNum << ")"
	       << " and type " << Rod::TypeName(type) << " with id "
	       << RodList.size() << endl;

	Rod* obj = new Rod(_log, RodList.size());
	obj->setup(number,
	           type,
	           RodPropList[TypeNum],
	           endCoords,
	           NumSegs,
	           env,
	           outfiles.back(),
	           outchannels);

	// depending on type, assign the Rod to its respective parent
	// body
	if (str::isOneOf(let1, { "ANCHOR", "FIXED", "FIX" }))
		GroundBody->addRod(obj, endCoords);
	if (str::isOneOf(let1, { "PINNED", "PIN" }))
		GroundBody->addRod(obj, endCoords);
	else if (let1 == "BODY") {
		unsigned int bodyID = stoi(num1);
		BodyList[bodyID - 1]->addRod(obj, endCoords);
	}
	LOGDBG << endl;

	return obj;
}

void
moordyn::MoorDyn::readOptionsLine(vector<string>& in_txt, int i)
{
	vector<string> entries = moordyn::str::split(in_txt[i], ' ');
	if (entries.size() < 2) {
		LOGWRN << "Ignoring option line " << i
		       << " due to unspecified value or option type" << endl;
		return;
	}

	LOGDBG << "\t" << entries[1] << " = " << entries[0] << endl;
	const string value = entries[0];
	const string name = entries[1];

	// DT is old way, should phase out
	if ((name == "dtM") || (name == "DT"))
		dtM0 = atof(entries[0].c_str());
	else if (name == "writeLog") {
		// This was actually already did, so we do not need to do that again
		// But we really want to have this if to avoid showing a warning for
		// Unrecognized option writeLog
		// env->writeLog = atoi(entries[0].c_str());
	} else if (name == "tScheme") {
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			_t_integrator = create_time_scheme(entries[0], _log, waves);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGWRN << "Defaulting to RK2 time integration";
			LOGERR << err_msg << endl;
		}
	} else if ((name == "g") || (name == "gravity"))
		env->g = atof(entries[0].c_str());
	else if ((name == "Rho") || (name == "rho") || (name == "WtrDnsty"))
		env->rho_w = atof(entries[0].c_str());
	else if (name == "WtrDpth")
		env->WtrDpth = atof(entries[0].c_str());
	else if ((name == "kBot") || (name == "kb"))
		env->kb = atof(entries[0].c_str());
	else if ((name == "cBot") || (name == "cb"))
		env->cb = atof(entries[0].c_str());
	else if ((name == "dtIC") || (name == "ICdt"))
		ICdt = atof(entries[0].c_str());
	else if ((name == "TmaxIC") || (name == "ICTmax"))
		ICTmax = atof(entries[0].c_str());
	else if ((name == "CdScaleIC") || (name == "ICDfac"))
		ICDfac = atof(entries[0].c_str());
	else if ((name == "threshIC") || (name == "ICthresh"))
		ICthresh = atof(entries[0].c_str());
	else if (name == "WaveKin") {
		WaveKinTemp = (waves::waves_settings)stoi(entries[0]);
		if ((WaveKinTemp < waves::WAVES_NONE) ||
		    (WaveKinTemp > waves::WAVES_SUM_COMPONENTS_NODE))
			LOGWRN << "Unknown WaveKin option value " << WaveKinTemp << endl;
	} else if (name == "dtWave")
		env->waterKinOptions.dtWave = stof(entries[0]);
	else if (name == "Currents") {
		auto current_mode = (waves::currents_settings)stoi(entries[0]);
		env->waterKinOptions.currentMode = current_mode;
		if ((current_mode < waves::CURRENTS_NONE) ||
		    (current_mode > waves::CURRENTS_4D))
			LOGWRN << "Unknown Currents option value " << current_mode << endl;
	} else if (name == "UnifyCurrentGrid") {
		if (entries[0] == "1") {
			env->waterKinOptions.unifyCurrentGrid = true;
		} else if (entries[0] == "0") {
			env->waterKinOptions.unifyCurrentGrid = false;
		} else {
			LOGWRN << "Unrecognized UnifyCurrentGrid value "
			       << std::quoted(entries[1]) << ". Should be 0 or 1" << endl;
		}
	} else if (name == "WriteUnits")
		env->WriteUnits = atoi(entries[0].c_str());
	else if (name == "FrictionCoefficient")
		env->FrictionCoefficient = atof(entries[0].c_str());
	else if (name == "FricDamp")
		env->FricDamp = atof(entries[0].c_str());
	else if (name == "StatDynFricScale")
		env->StatDynFricScale = atof(entries[0].c_str());
	// output writing period (0 for at every call)
	else if (name == "dtOut")
		dtOut = atof(entries[0].c_str());
	else if (name == "SeafloorFile") {
		env->SeafloorMode = seafloor_settings::SEAFLOOR_3D;
		this->seafloor = make_shared<moordyn::Seafloor>(_log);
		std::string filepath = entries[0];
		this->seafloor->setup(env, filepath);
	} else
		LOGWRN << "Warning: Unrecognized option '" << name << "'" << endl;
}

bool
moordyn::MoorDyn::checkNumberOfEntriesInLine(vector<string> entries,
                                             int supposedNumberOfEntries)
{
	if (entries.size() < supposedNumberOfEntries) {
		LOGERR << "Error in " << _filepath << ":" << endl
		       << supposedNumberOfEntries << " fields are required, but just "
		       << entries.size() << " are provided" << endl;
		return false;
	}

	return true;
}

void
moordyn::MoorDyn::detachLines(FailProps* failure)
{
	failure->status = true;
	if (failure->rod && failure->point) {
		LOGERR << "The failure criteria considers both a rod and a point"
		       << endl;
		throw moordyn::unhandled_error("Invalid failure data");
	} else if (!failure->rod && !failure->point) {
		LOGERR << "The failure criteria is missing either a rod or a "
		          "point"
		       << endl;
		throw moordyn::unhandled_error("Invalid failure data");
	}

	// create new massless point for detached end(s) of line(s)
	const real M = 0.0;
	const real V = 0.0;
	const vec r0 = vec::Zero();
	const vec F = vec::Zero();
	const real CdA = 0.0;
	const real Ca = 0.0;
	const Point::types type = Point::FREE;

	nX += 6; // add 6 state variables for each point

	// add point to list of free ones and add states for it
	FreePointIs.push_back(ui_size(PointList));
	// assign start index of this point's states
	PointStateIs.push_back(nX);

	// now make Point object!
	Point* obj = new Point(_log, PointList.size());
	obj->setup(static_cast<int>(PointList.size() + 1),
	           type,
	           r0,
	           M,
	           V,
	           F,
	           CdA,
	           Ca,
	           env);
	obj->setWaves(waves, seafloor);
	PointList.push_back(obj);

	// Kinematics of old attachment point
	vec pos, vel;
	if (failure->rod) {
		const unsigned int node =
		    (failure->rod_end_point == ENDPOINT_A) ? 0 : failure->rod->getN();
		pos = failure->rod->getNodePos(node);
		vel = failure->rod->getNodeVel(node);
	} else {
		std::tie(pos, vel) = failure->point->getState();
	}

	// detach lines from old Rod or Point, and get kinematics of the
	// old attachment point
	for (unsigned int i = 0; i < failure->lines.size(); i++) {
		if (failure->rod)
			failure->line_end_points[i] = failure->rod->removeLine(
			    failure->rod_end_point, failure->lines[i]);
		else
			failure->line_end_points[i] =
			    failure->point->removeLine(failure->lines[i]);

		obj->addLine(failure->lines[i], failure->line_end_points[i]);
	}

	// update point kinematics to match old line attachment point
	// kinematics and set positions of attached line ends
	obj->setState(pos, vel);
}

moordyn::error_id
moordyn::MoorDyn::AllOutput(double t, double dt)
{
	// if (env->writeLog == 0)
	// 	return MOORDYN_SUCCESS;

	if (dtOut > 0)
		if (t < (floor((t - dt) / dtOut) + 1.0) * dtOut)
			return MOORDYN_SUCCESS;

	// write to master output file
	if (!outfileMain.is_open()) {
		LOGERR << "Error: Unable to write to main output file " << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}
	outfileMain << t << "\t "; // output time
	for (auto channel : outChans) {
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			outfileMain << GetOutput(channel) << "\t ";
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGERR << "Error handling an output channel:" << err_msg << endl;
			return err;
		}
	}
	outfileMain << endl;

	// write individual output files
	for (auto obj : LineList)
		obj->Output(t);
	for (auto obj : RodList)
		obj->Output(t);
	for (auto obj : BodyList)
		obj->Output(t);

	return MOORDYN_SUCCESS;
}

} // ::moordyn

// =============================================================================
//
//                     ||                     ||
//                     ||        C API        ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

MoorDyn DECLDIR
MoorDyn_Create(const char* infilename)
{
	// ---------------------------- MoorDyn title message
	// ----------------------------

	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	moordyn::MoorDyn* instance = NULL;
	try {
		instance = new moordyn::MoorDyn(infilename);
	}
	MOORDYN_CATCHER(err, err_msg);

	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err
		     << ") during the Mooring System creation:" << endl
		     << err_msg << endl;
	}
	return (MoorDyn)instance;
}

/// Check that the provided system is not Null
#define CHECK_SYSTEM(s)                                                        \
	if (!s) {                                                                  \
		cerr << "Null system received in " << __FUNC_NAME__ << " ("            \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_SetVerbosity(MoorDyn system, int verbosity)
{
	CHECK_SYSTEM(system);
	((moordyn::MoorDyn*)system)->GetLogger()->SetVerbosity(verbosity);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLogFile(MoorDyn system, const char* log_path)
{
	CHECK_SYSTEM(system);
	((moordyn::MoorDyn*)system)->GetLogger()->SetFile(log_path);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SetLogLevel(MoorDyn system, int verbosity)
{
	CHECK_SYSTEM(system);
	((moordyn::MoorDyn*)system)->GetLogger()->SetLogLevel(verbosity);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_Log(MoorDyn system, int level, const char* msg)
{
	CHECK_SYSTEM(system);
	((moordyn::MoorDyn*)system)->GetLogger()->Cout(level) << msg;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_NCoupledDOF(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ((moordyn::MoorDyn*)system)->NCoupledDOF();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_Init(MoorDyn system, const double* x, const double* xd)
{
	CHECK_SYSTEM(system);
	return ((moordyn::MoorDyn*)system)->Init(x, xd);
}

int DECLDIR
MoorDyn_Init_NoIC(MoorDyn system, const double* x, const double* xd)
{
	CHECK_SYSTEM(system);
	return ((moordyn::MoorDyn*)system)->Init(x, xd, true);
}

int DECLDIR
MoorDyn_Step(MoorDyn system,
             const double* x,
             const double* xd,
             double* f,
             double* t,
             double* dt)
{
	CHECK_SYSTEM(system);
	return ((moordyn::MoorDyn*)system)->Step(x, xd, f, *t, *dt);
}

int DECLDIR
MoorDyn_Close(MoorDyn system)
{
	CHECK_SYSTEM(system);
	delete ((moordyn::MoorDyn*)system);
	return MOORDYN_SUCCESS;
}

MoorDynWaves DECLDIR
MoorDyn_GetWaves(MoorDyn system)
{
	if (!system)
		return NULL;
	return (MoorDynWaves)(((moordyn::MoorDyn*)system)->GetWaves().get());
}

MoorDynSeafloor DECLDIR
MoorDyn_GetSeafloor(MoorDyn system)
{
	if (!system)
		return NULL;
	return (MoorDynSeafloor)(((moordyn::MoorDyn*)system)->GetSeafloor().get());
}

int DECLDIR
MoorDyn_ExternalWaveKinInit(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);

	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		*n = ((moordyn::MoorDyn*)system)->ExternalWaveKinInit();
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():" << endl
		     << err_msg << endl;
	}
	return err;
}

int DECLDIR
MoorDyn_ExternalWaveKinGetN(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ((moordyn::MoorDyn*)system)->ExternalWaveKinGetN();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_ExternalWaveKinGetCoordinates(MoorDyn system, double* r)
{
	CHECK_SYSTEM(system);

	auto r_list = ((moordyn::MoorDyn*)system)->ExternalWaveKinGetPoints();
	for (unsigned int i = 0; i < r_list.size(); i++) {
		const auto r_i = r_list[i];
		for (unsigned int j = 0; j < 3; j++) {
			r[3 * i + j] = r_i[j];
		}
	}

	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_ExternalWaveKinSet(MoorDyn system,
                           const double* U,
                           const double* Ud,
                           double t)
{
	CHECK_SYSTEM(system);

	std::vector<moordyn::vec> u, ud;
	unsigned int n;
	MoorDyn_ExternalWaveKinGetN(system, &n);
	if (!n) {
		cerr << "Error: There is not wave kinematics to set "
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return MOORDYN_INVALID_VALUE;
	}
	u.reserve(n);
	ud.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		u.push_back(moordyn::vec(U[3 * i], U[3 * i + 1], U[3 * i + 2]));
		ud.push_back(moordyn::vec(Ud[3 * i], Ud[3 * i + 1], Ud[3 * i + 2]));
	}

	((moordyn::MoorDyn*)system)->ExternalWaveKinSet(u, ud, t);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetNumberBodies(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ui_size(((moordyn::MoorDyn*)system)->GetBodies());
	return MOORDYN_SUCCESS;
}

MoorDynBody DECLDIR
MoorDyn_GetBody(MoorDyn system, unsigned int b)
{
	if (!system)
		return NULL;
	auto bodies = ((moordyn::MoorDyn*)system)->GetBodies();
	if (!b || (b > bodies.size())) {
		cerr << "Error: There is not such body " << b << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return NULL;
	}
	return (MoorDynBody)(bodies[b - 1]);
}

int DECLDIR
MoorDyn_GetNumberRods(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ui_size(((moordyn::MoorDyn*)system)->GetRods());
	return MOORDYN_SUCCESS;
}

MoorDynRod DECLDIR
MoorDyn_GetRod(MoorDyn system, unsigned int l)
{
	if (!system)
		return NULL;
	auto rods = ((moordyn::MoorDyn*)system)->GetRods();
	if (!l || (l > rods.size())) {
		cerr << "Error: There is not such rod " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return NULL;
	}
	return (MoorDynRod)(rods[l - 1]);
}

int DECLDIR
MoorDyn_GetNumberPoints(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ui_size(((moordyn::MoorDyn*)system)->GetPoints());
	return MOORDYN_SUCCESS;
}

MoorDynPoint DECLDIR
MoorDyn_GetPoint(MoorDyn system, unsigned int l)
{
	if (!system)
		return NULL;
	auto points = ((moordyn::MoorDyn*)system)->GetPoints();
	if (!l || (l > points.size())) {
		cerr << "Error: There is not such point " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return NULL;
	}
	return (MoorDynPoint)(points[l - 1]);
}

int DECLDIR
MoorDyn_GetNumberLines(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ui_size(((moordyn::MoorDyn*)system)->GetLines());
	return MOORDYN_SUCCESS;
}

MoorDynLine DECLDIR
MoorDyn_GetLine(MoorDyn system, unsigned int l)
{
	if (!system)
		return NULL;
	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if (!l || (l > lines.size())) {
		cerr << "Error: There is not such line " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return NULL;
	}
	return (MoorDynLine)(lines[l - 1]);
}

int DECLDIR
MoorDyn_GetFASTtens(MoorDyn system,
                    const int* numLines,
                    float FairHTen[],
                    float FairVTen[],
                    float AnchHTen[],
                    float AnchVTen[])
{
	CHECK_SYSTEM(system);

	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if ((unsigned int)(*numLines) > lines.size()) {
		cerr << "Error: There is not " << *numLines << " lines" << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return MOORDYN_INVALID_VALUE;
	}

	for (int l = 0; l < *numLines; l++)
		lines[l]->getFASTtens(
		    FairHTen + l, FairVTen + l, AnchHTen + l, AnchVTen + l);

	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_Serialize(MoorDyn system, size_t* size, uint64_t* data)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const std::vector<uint64_t> backup =
		    ((moordyn::MoorDyn*)system)->Serialize();
		if (size)
			*size = backup.size() * sizeof(uint64_t);
		if (data)
			memcpy(data, backup.data(), backup.size() * sizeof(uint64_t));
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():" << endl
		     << err_msg << endl;
	}
	return err;
}

int DECLDIR
MoorDyn_Deserialize(MoorDyn system, const uint64_t* data)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	if (!data) {
		cerr << "Error: No data has been provided to " << __FUNC_NAME__ << "()"
		     << endl;
		return MOORDYN_INVALID_VALUE;
	}
	string err_msg;
	try {
		((moordyn::MoorDyn*)system)->Deserialize(data);
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():" << endl
		     << err_msg << endl;
	}
	return err;
}

int DECLDIR
MoorDyn_Save(MoorDyn system, const char* filepath)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::MoorDyn*)system)->Save(filepath);
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():" << endl
		     << err_msg << endl;
	}
	return err;
}

int DECLDIR
MoorDyn_Load(MoorDyn system, const char* filepath)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::MoorDyn*)system)->Load(filepath);
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():" << endl
		     << err_msg << endl;
	}
	return err;
}

int DECLDIR
MoorDyn_DrawWithGL(MoorDyn system)
{
	CHECK_SYSTEM(system);

#ifdef USEGL
	// draw the mooring system with OpenGL commands (assuming a GL context has
	// been created by the calling program)
	for (auto line : ((moordyn::MoorDyn*)system)->GetLines())
		line->drawGL2();
	for (auto point : ((moordyn::MoorDyn*)system)->GetPoints())
		point->drawGL();
#endif
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SaveVTK(MoorDyn system, const char* filename)
{
#ifdef USE_VTK
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::MoorDyn*)system)->saveVTK(filename);
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
