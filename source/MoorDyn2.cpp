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
#include "MoorDyn2.hpp"
#include "Rod.hpp"

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

namespace moordyn {

/// The list of units for the output
const char* UnitList[] = { "(s)     ", "(m)     ", "(m)     ", "(m)     ",
	                       "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
	                       "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
	                       "(N)     ", "(N)     " };

moordyn::MoorDyn::MoorDyn(const char* infilename)
  : io::IO(NULL)
  , _filepath("Mooring/lines.txt")
  , _basename("lines")
  , _basepath("Mooring/")
  , ICDfac(5.0)
  , ICdt(1.0)
  , ICTmax(120.0)
  , ICthresh(0.001)
  , WaveKinTemp(WAVES_NONE)
  , dtM0(0.001)
  , dtOut(0.0)
  , _t_integrator(NULL)
  , GroundBody(NULL)
  , waves(NULL)
  , nX(0)
  , nXtra(0)
  , npW(0)
  , tW_1(0.0)
  , tW_2(0.0)
{
	SetLogger(new Log());

	if (infilename && (strlen(infilename) > 0)) {
		_filepath = infilename;
		const int lastSlash = _filepath.find_last_of("/\\");
		const int lastDot = _filepath.find_last_of('.');
		_basename = _filepath.substr(lastSlash + 1, lastDot - lastSlash - 1);
		_basepath = _filepath.substr(0, lastSlash + 1);
	}

	LOGMSG << "\n Running MoorDyn (v2.a10, 2022-01-01)" << endl
	       << "   NOTE: This is an alpha version of MoorDyn v2, intended for "
	          "testing and debugging."
	       << endl
	       << "         MoorDyn v2 has significant ongoing input file changes "
	          "from v1."
	       << endl
	       << "   Copyright: (C) 2021 National Renewable Energy Laboratory, "
	          "(C) 2014-2019 Matt Hall"
	       << endl
	       << "   This program is released under the GNU General Public "
	          "License v3."
	       << endl;

	LOGMSG << "The filename is " << _filepath << endl;
	LOGDBG << "The basename is " << _basename << endl;
	LOGDBG << "The basepath is " << _basepath << endl;

	env.g = 9.8;
	env.WtrDpth = 0.;
	env.rho_w = 1025.;
	env.kb = 3.0e6;
	env.cb = 3.0e5;
	env.WaveKin = moordyn::WAVES_NONE;
	env.Current = moordyn::CURRENTS_NONE;
	env.dtWave = 0.25;
	env.WriteUnits = 1; // by default, write units line
	env.writeLog = 0;   // by default, don't write out a log file
	env.FrictionCoefficient = 0.0;
	env.FricDamp = 200.0;
	env.StatDynFricScale = 1.0;

	const moordyn::error_id err = ReadInFile();
	MOORDYN_THROW(err, "Exception while reading the input file");

	LOGDBG << "MoorDyn is expecting " << NCoupledDOF()
	       << " coupled degrees of freedom" << endl;

	if (!nX) {
		LOGWRN << "WARNING: MoorDyn has no state variables."
		       << " (Is there a mooring sytem?)" << endl;
	}

	nXtra = nX + 6 * 2 * LineList.size();
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
	delete waves;
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
	for (auto obj : ConnectionList)
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

	for (auto l : CpldConIs) {
		LOGMSG << "Initializing coupled Connection " << l << " in " << x[ix]
		       << ", " << x[ix + 1] << ", " << x[ix + 2] << "..." << endl;
		vec r, rd;
		moordyn::array2vec(x + ix, r);
		moordyn::array2vec(xd + ix, rd);
		ConnectionList[l]->initiateStep(r, rd);

		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try {
			ConnectionList[l]->updateFairlead(0.0);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS) {
			LOGERR << "Error initializing coupled connection" << l << ": "
			       << err_msg << endl;
			return err;
		}
		// call this just to set WaterKin (may also set up output file in
		// future)
		ConnectionList[l]->initialize();
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
	for (auto obj : ConnectionList)
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

		// go through connections to get fairlead forces
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
					if (error > max_error)
						max_error = error;
					if (max_error > ICthresh) {
						converged = false;
						if ((_log->GetVerbosity() > MOORDYN_DBG_LEVEL) ||
						    (_log->GetLogLevel() > MOORDYN_DBG_LEVEL)) {
							// We are not reporting the error, so it is enough
							// to know that the criteria convergence has not
							// been met
							break;
						}
					}
				}
				if (!converged) {
					LOGDBG << "Dynamic relaxation t = " << t << "s (time step "
					       << iic << "), error = " << 100.0 * max_error
					       << "%     \r";
					break;
				}
			}

			if (converged)
				break;
		}

		iic++;
	}

	if (converged) {
		LOGMSG << "Fairlead tensions converged" << endl;
	} else {
		LOGWRN << "Fairlead tensions did not converged" << endl;
	}
	LOGMSG << "Remaining error after " << t << " s = " << 100.0 * max_error
	       << "%" << endl;

	// restore drag coefficients to normal values and restart time counter of
	// each object
	_t_integrator->SetTime(0.0);
	for (auto obj : LineList) {
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : ConnectionList)
		obj->scaleDrag(1.0 / ICDfac);
	for (auto obj : RodList) {
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : BodyList)
		obj->scaleDrag(1.0 / ICDfac);

	// store passed WaveKin value to enable waves in simulation if applicable
	// (they're not enabled during IC gen)
	env.WaveKin = WaveKinTemp;
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		waves->setup(&env, _t_integrator, _basepath.c_str());
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS)
		return err;

	// @mth: new approach to be implemented
	// ------------------------- calculate wave time series if needed
	// -------------------
	// 	if (env.WaveKin == 2)
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

	if (env.WriteUnits > 0) {
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

moordyn::error_id
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
	// ... of any coupled bodies, rods, and connections at this instant, to be
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
	for (auto l : CpldConIs) {
		vec r, rd;
		moordyn::array2vec(x + ix, r);
		moordyn::array2vec(xd + ix, rd);
		ConnectionList[l]->initiateStep(r, rd);
		ix += 3;
	}

	// -------------------- do time stepping -----------------------
	if (env.WaveKin == WAVES_EXTERNAL) {
		// extrapolate velocities from accelerations
		// (in future could extrapolote from most recent two points,
		// (U_1 and U_2)
		_t_integrator->SetExtWaves(tW_1, U_1, Ud_1);
	}
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
			LOGERR << "t = " << t << " s" << endl;
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
	data.push_back(io::IO::Serialize(tW_1));
	subdata = io::IO::Serialize(U_1);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Ud_1);
	data.insert(data.end(), subdata.begin(), subdata.end());
	data.push_back(io::IO::Serialize(tW_2));
	subdata = io::IO::Serialize(U_2);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Ud_2);
	data.insert(data.end(), subdata.begin(), subdata.end());

	// Ask to save the data off all the subinstances
	subdata = _t_integrator->Serialize();
	data.insert(data.end(), subdata.begin(), subdata.end());

	return data;
}

uint64_t*
MoorDyn::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	uint64_t n;
	ptr = io::IO::Deserialize(ptr, n);
	npW = n;
	ptr = io::IO::Deserialize(ptr, tW_1);
	ptr = io::IO::Deserialize(ptr, U_1);
	ptr = io::IO::Deserialize(ptr, Ud_1);
	ptr = io::IO::Deserialize(ptr, tW_2);
	ptr = io::IO::Deserialize(ptr, U_2);
	ptr = io::IO::Deserialize(ptr, Ud_2);

	// Load the children data also
	ptr = _t_integrator->Deserialize(ptr);

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkMultiBlockDataSet>
MoorDyn::getVTK() const
{
	auto out = vtkSmartPointer<vtkMultiBlockDataSet>::New();
	out->SetNumberOfBlocks(RodList.size() + ConnectionList.size() +
	                       LineList.size());
	unsigned int n = 0;
	for (unsigned int i = 0; i < BodyList.size(); i++)
		out->SetBlock(n + i, BodyList[i]->getVTK());
	n += BodyList.size();
	for (unsigned int i = 0; i < ConnectionList.size(); i++)
		out->SetBlock(n + i, ConnectionList[i]->getVTK());
	n += ConnectionList.size();
	for (unsigned int i = 0; i < RodList.size(); i++)
		out->SetBlock(n + i, RodList[i]->getVTK());
	n += RodList.size();
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
	ifstream in_file(_filepath);
	vector<string> in_txt;
	string line;
	if (!in_file.is_open()) {
		LOGERR << "Error: unable to open file '" << _filepath << "'" << endl;
		return MOORDYN_INVALID_INPUT_FILE;
	}
	while (in_file.good()) {
		string line_txt;
		getline(in_file, line_txt);
		in_txt.push_back(line_txt);
	}
	in_file.close();
	while (i < in_txt.size()) {
		// Skip until we find the options header line
		if ((in_txt[i].find("---") == string::npos) ||
		    !moordyn::str::has(moordyn::str::upper(in_txt[i]), { "OPTIONS" })) {
			i++;
			continue;
		}

		i++;
		// Look for writeLog option entries until the end of section or file
		while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size())) {
			vector<string> entries = moordyn::str::split(in_txt[i], ' ');
			if (entries.size() < 2) {
				i++;
				continue;
			}
			const string value = entries[0];
			const string name = entries[1];
			if (name != "writeLog") {
				i++;
				continue;
			}
			env.writeLog = atoi(entries[0].c_str());
			const moordyn::error_id err = SetupLog();
			if (err != MOORDYN_SUCCESS)
				return err;

			i++;
		}
	}

	// factor by which to boost drag coefficients during dynamic relaxation IC
	// generation
	ICDfac = 5.0;
	// convergence analysis time step for IC generation
	ICdt = 1.0;
	// max time for IC generation
	ICTmax = 120;
	// threshold for relative change in tensions to call it converged
	ICthresh = 0.001;
	// temporary wave kinematics flag used to store input value while keeping
	// env.WaveKin=0 for IC gen
	WaveKinTemp = WAVES_NONE;
	// assume no wave kinematics points are passed in externally, unless
	// ExernalWaveKinInit is called later
	npW = 0;
	// default value for desired mooring model time step
	dtM0 = 0.001;
	// default time integration scheme
	string t_integrator_name = "RK2";

	// string containing which channels to write to output
	vector<string> outchannels;

	// make a "ground body" that will be the parent of all fixed objects
	// (connections and rods)
	LOGDBG << "Creating the ground body of type " << Body::TypeName(Body::FIXED)
	       << "..." << endl;
	GroundBody = new Body(_log);
	GroundBody->setup(0,
	                  Body::FIXED,
	                  vec6::Zero(),
	                  vec::Zero(),
	                  0.0,
	                  0.0,
	                  vec::Zero(),
	                  vec6::Zero(),
	                  vec6::Zero(),
	                  NULL);

	// Make sure the state vector counter starts at zero
	// This will be conveniently incremented as each object is added
	nX = 0;

	// Now we can parse the whole input file
	i = 0;
	while (i < in_txt.size()) {
		// look for the section header line
		if (in_txt[i].find("---") == string::npos) {
			i++;
			continue;
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "LINE DICTIONARY", "LINE TYPES" })) {
			LOGDBG << "   Reading line types:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 10) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "10 fields are required, but just "
					       << entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
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
					return err;
				err = read_curve(entries[4].c_str(),
				                 &(obj->c),
				                 &(obj->nCpoints),
				                 obj->dampXs,
				                 obj->dampYs);
				if (err)
					return err;
				err = read_curve(entries[5].c_str(),
				                 &(obj->EI),
				                 &(obj->nEIpoints),
				                 obj->bstiffXs,
				                 obj->bstiffYs);
				if (err)
					return err;

				LOGDBG << "\t'" << obj->type << "'"
				       << " - with id " << LinePropList.size() << endl
				       << "\t\td   : " << obj->d << endl
				       << "\t\tw   : " << obj->w << endl
				       << "\t\tCdn : " << obj->Cdn << endl
				       << "\t\tCan : " << obj->Can << endl
				       << "\t\tCdt : " << obj->Cdt << endl
				       << "\t\tCat : " << obj->Cat << endl;

				LinePropList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "ROD DICTIONARY", "ROD TYPES" })) {
			LOGDBG << "   Reading rod types:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 7) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "7 fields are required, but just "
					       << entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				RodProps* obj = new RodProps();
				obj->type = entries[0];
				obj->d = atof(entries[1].c_str());
				obj->w = atof(entries[2].c_str());
				obj->Cdn = atof(entries[3].c_str());
				obj->Can = atof(entries[4].c_str());
				obj->Cdt = atof(entries[5].c_str());
				obj->Cat = atof(entries[6].c_str());

				LOGDBG << "\t'" << obj->type << "'"
				       << " - with id " << RodPropList.size() << endl
				       << "\t\td   : " << obj->d << endl
				       << "\t\tw   : " << obj->w << endl
				       << "\t\tCdn : " << obj->Cdn << endl
				       << "\t\tCan : " << obj->Can << endl
				       << "\t\tCdt : " << obj->Cdt << endl
				       << "\t\tCat : " << obj->Cat << endl;

				RodPropList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "BODIES", "BODY LIST", "BODY PROPERTIES" })) {
			LOGDBG << "   Reading body list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 14) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "14 fields are required, but just "
					       << entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
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

				vector<string> entries_rCG =
				    moordyn::str::split(entries[9], '|');
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
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "CG entry (col 10) must have 1 or 3 numbers"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_I =
				    moordyn::str::split(entries[10], '|');
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
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Inertia entry (col 11) must have 1 or 3 numbers"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_CdA =
				    moordyn::str::split(entries[12], '|');
				if (entries_CdA.size() == 1) {
					// if only one entry, use it for all directions
					CdA[0] = atof(entries_CdA[0].c_str());
					CdA[1] = CdA[0];
					CdA[2] = CdA[0];
				} else if (entries_CdA.size() == 3) {
					CdA[0] = atof(entries_CdA[0].c_str());
					CdA[1] = atof(entries_CdA[1].c_str());
					CdA[2] = atof(entries_CdA[2].c_str());
				} else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "CdA entry (col 13) must have 1 or 3 numbers"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_Ca =
				    moordyn::str::split(entries[13], '|');
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
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Ca entry (col 14) must have 1 or 3 numbers"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				char let1[10], num1[10], let2[10], num2[10], let3[10];
				char typeWord[10];
				strncpy(typeWord, entries[1].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				str::decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") ||
				    !strcmp(let1, "FIX")) {
					// it is fixed  (this would just be used if someone wanted
					// to temporarly fix a body that things were attached to)
					type = Body::FIXED;
				} else if (!strcmp(let1, "COUPLED") ||
				           !strcmp(let1, "VESSEL") || !strcmp(let1, "VES") ||
				           !strcmp(let1, "CPLD")) {
					// it is coupled - controlled from outside
					type = Body::COUPLED;
					CpldBodyIs.push_back(BodyList.size());
				} else {
					// it is free - controlled by MoorDyn
					type = Body::FREE;
					FreeBodyIs.push_back(BodyList.size());
					BodyStateIs.push_back(
					    nX);  // assign start index of this body's states
					nX += 12; // add 12 state variables for the body
				}
				stringstream oname;
				oname << _basepath << _basename << "_Body" << number << ".out";
				outfiles.push_back(make_shared<ofstream>(oname.str()));
				if (!outfiles.back()->is_open()) {
					LOGERR << "Cannot create the output file '" << oname.str()
					       << endl;
					return MOORDYN_INVALID_OUTPUT_FILE;
				}

				Body* obj = new Body(_log);
				LOGDBG << "\t'" << number << "'"
				       << " - of type " << Body::TypeName(type) << " with id "
				       << BodyList.size() << endl;
				obj->setup(number,
				           type,
				           r6,
				           rCG,
				           M,
				           V,
				           Inert,
				           CdA,
				           Ca,
				           outfiles.back());
				BodyList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "RODS", "ROD LIST", "ROD PROPERTIES" })) {
			LOGDBG << "   Reading rod list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 8) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "8 fields are required, but just "
					       << entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				int number = atoi(entries[0].c_str());
				string RodType = entries[1];
				vec6 endCoords;
				for (unsigned int I = 0; I < 6; I++)
					endCoords[I] = atof(entries[3 + I].c_str());
				int NumSegs = atoi(entries[9].c_str());
				string outchannels = entries[10];

				char let1[10], num1[10], let2[10], num2[10], let3[10];
				char typeWord[10];
				strncpy(typeWord, entries[2].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				str::decomposeString(typeWord, let1, num1, let2, num2, let3);

				Rod::types type;
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") ||
				    !strcmp(let1, "FIX")) {
					// it is fixed  (this would just be used if someone wanted
					// to temporarly fix a body that things were attached to)
					type = Rod::FIXED;
				} else if (!strcmp(let1, "PINNED") || !strcmp(let1, "PIN")) {
					// it is pinned
					type = Rod::PINNED;
					FreeRodIs.push_back(
					    RodList.size()); // add this pinned rod to the free list
					                     // because it is half free
					RodStateIs.push_back(
					    nX); // assign start index of this rod's states
					nX += 6; // add 6 state variables for each pinned rod
				} else if (!strcmp(let1, "BODY")) {
					if (!strlen(num1)) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "no number provided for Rod " << number
						       << " Body attachment" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int bodyID = atoi(num1);
					if (!bodyID || (bodyID > BodyList.size())) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There is not " << bodyID << " bodies"
						       << endl;
						return MOORDYN_INVALID_INPUT;
					}

					if (!strcmp(let2, "PINNED") || !strcmp(let2, "PIN")) {
						// it is pinned
						type = Rod::PINNED;
						FreeRodIs.push_back(
						    RodList.size()); // add this pinned rod to the free
						                     // list because it is half free
						RodStateIs.push_back(
						    nX); // assign start index of this rod's states
						nX += 6; // add 6 state variables for each pinned rod
					} else {
						type = Rod::FIXED;
					}
				} else if (!strcmp(let1, "VESSEL") || !strcmp(let1, "VES") ||
				           !strcmp(let1, "COUPLED") || !strcmp(let1, "CPLD")) {
					// if a rigid fairlead, add to list and add
					type = Rod::COUPLED;
					CpldRodIs.push_back(
					    RodList.size()); // index of fairlead in RodList vector
				} else if (!strcmp(let1, "VESPIN") ||
				           !strcmp(let1, "CPLDPIN")) {
					// if a pinned fairlead, add to list and add
					type = Rod::CPLDPIN;
					CpldRodIs.push_back(
					    RodList.size()); // index of fairlead in RodList vector
					FreeRodIs.push_back(
					    RodList.size()); // also add this pinned rod to the free
					                     // list because it is half free
					RodStateIs.push_back(
					    nX); // assign start index of this rod's states
					nX += 6; // add 6 state variables for each pinned rod
				} else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") ||
				           !strcmp(let1, "FREE")) {
					type = Rod::FREE;
					FreeRodIs.push_back(
					    RodList.size()); // add this free rod to the free list
					RodStateIs.push_back(
					    nX);  // assign start index of this rod's states
					nX += 12; // add 12 state variables for each free rod
				} else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Unrecognized connection type '" << let1 << "'"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				int TypeNum = -1;
				for (unsigned int J = 0; J < RodPropList.size(); J++) {
					if (RodPropList[J]->type == RodType)
						TypeNum = J;
				}
				if (TypeNum == -1) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Unrecognized rod type " << RodType << endl;
					return MOORDYN_INVALID_INPUT;
				}

				// Make the output file (if queried)
				if ((outchannels.size() > 0) &&
				    (strcspn(outchannels.c_str(), "pvUDctsd") <
				     strlen(outchannels.c_str()))) {
					// if 1+ output flag chars are given and they're valid
					stringstream oname;
					oname << _basepath << _basename << "_Rod" << number
					      << ".out";
					outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!outfiles.back()->is_open()) {
						LOGERR << "Cannot create the output file '"
						       << oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				} else
					outfiles.push_back(NULL);

				LOGDBG << "\t'" << number << "'"
				       << " - of class " << RodType << " (" << TypeNum << ")"
				       << " and type " << Rod::TypeName(type) << " with id "
				       << RodList.size() << endl;

				Rod* obj = new Rod(_log);
				obj->setup(number,
				           type,
				           RodPropList[TypeNum],
				           endCoords,
				           NumSegs,
				           outfiles.back(),
				           outchannels);
				RodList.push_back(obj);

				// depending on type, assign the Rod to its respective parent
				// body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") ||
				    !strcmp(let1, "FIX"))
					GroundBody->addRod(obj, endCoords);
				else if (!strcmp(let1, "PINNED") || !strcmp(let1, "PIN"))
					GroundBody->addRod(obj, endCoords);
				else if (!strcmp(let1, "BODY")) {
					unsigned int bodyID = atoi(num1);
					BodyList[bodyID - 1]->addRod(obj, endCoords);
				}
				LOGDBG << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "POINTS",
		                        "POINT LIST",
		                        "CONNECTION PROPERTIES",
		                        "NODE PROPERTIES" })) {
			LOGDBG << "   Reading point list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 9) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "9 fields are required, but just "
					       << entries.size() << " are provided" << endl;
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

				Connection::types type;
				char let1[10], num1[10], let2[10], num2[10], let3[10];
				char typeWord[10];
				strncpy(typeWord, entries[1].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				str::decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") ||
				    !strcmp(let1, "FIX")) {
					// it is fixed  (this would just be used if someone wanted
					// to temporarly fix a body that things were attached to)
					type = Connection::FIXED;
				} else if (!strcmp(let1, "BODY")) {
					type = Connection::FIXED;
					if (!strlen(num1)) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "no number provided for Rod " << number
						       << " Body attachment" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int bodyID = atoi(num1);
					if (!bodyID || (bodyID > BodyList.size())) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There is not " << bodyID << " bodies"
						       << endl;
						return MOORDYN_INVALID_INPUT;
					}
				} else if (!strcmp(let1, "FAIRLEAD") ||
				           !strcmp(let1, "VESSEL") || !strcmp(let1, "VES") ||
				           !strcmp(let1, "COUPLED") || !strcmp(let1, "CPLD")) {
					// if a fairlead, add to list and add
					type = Connection::COUPLED;
					CpldConIs.push_back(ConnectionList.size());
				} else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") ||
				           !strcmp(let1, "FREE")) {
					// if a connect, add to list and add states for it
					type = Connection::FREE;
					FreeConIs.push_back(ConnectionList.size());
					ConnectStateIs.push_back(
					    nX); // assign start index of this connect's states
					nX += 6; // add 6 state variables for each connect
				} else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Unrecognized connection type '" << let1 << "'"
					       << endl;
					return MOORDYN_INVALID_INPUT;
				}

				// make default water depth at least the depth of the lowest
				// node (so water depth input is optional)
				if (r0[2] < -env.WtrDpth)
					env.WtrDpth = -r0[2];

				LOGDBG << "\t'" << number << "'"
				       << " - of type " << Connection::TypeName(type)
				       << " with id " << ConnectionList.size() << endl;

				// now make Connection object!
				Connection* obj = new Connection(_log);
				obj->setup(number, type, r0, M, V, F, CdA, Ca);
				ConnectionList.push_back(obj);

				// depending on type, assign the Connection to its respective
				// parent body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") ||
				    !strcmp(let1, "FIX"))
					GroundBody->addConnection(obj, r0);
				else if (!strcmp(let1, "BODY")) {
					int bodyID = atoi(num1);
					BodyList[bodyID - 1]->addConnection(obj, r0);
				}
				LOGDBG << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      { "LINES", "LINE LIST", "LINE PROPERTIES" })) {
			LOGDBG << "   Reading line list: " << endl;

			if (!LinePropList.size()) {
				LOGERR << "Reading lines without defined line types" << endl;
				return MOORDYN_INVALID_INPUT;
			}
			if (!ConnectionList.size()) {
				LOGERR << "Reading lines without defined connections" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 7) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "7 fields are required, but just "
					       << entries.size() << " are provided" << endl;
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
					oname << _basepath << _basename << "_Line" << number
					      << ".out";
					outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!outfiles.back()->is_open()) {
						LOGERR << "Cannot create the output file '"
						       << oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				} else
					outfiles.push_back(NULL);

				LOGDBG << "\t'" << number << "'"
				       << " - of class " << type << " (" << TypeNum << ")"
				       << " with id " << LineList.size() << endl;

				Line* obj = new Line(_log);
				obj->setup(number,
				           LinePropList[TypeNum],
				           UnstrLen,
				           NumSegs,
				           outfiles.back(),
				           outchannels);
				LineList.push_back(obj);
				LineStateIs.push_back(
				    nX); // assign start index of this Line's states
				nX += 6 * (NumSegs - 1); // add 6 state variables for each
				                         // internal node of this line

				for (unsigned int I = 0; I < 2; I++) {
					const EndPoints end_point =
					    I == 0 ? ENDPOINT_A : ENDPOINT_B;
					char let1[10], num1[10], let2[10], num2[10], let3[10];
					char typeWord[10];
					strncpy(typeWord, entries[2 + I].c_str(), 9);
					typeWord[9] = '\0';
					// divided outWord into letters and numbers
					str::decomposeString(
					    typeWord, let1, num1, let2, num2, let3);

					if (!strlen(num1)) {
						LOGERR
						    << "Error in " << _filepath << ":" << i + 1 << "..."
						    << endl
						    << "'" << in_txt[i] << "'" << endl
						    << "No number provided for the 1st connection index"
						    << endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int id = atoi(num1);

					if (!strcmp(let1, "R") || !strcmp(let1, "ROD")) {
						if (!id || id > RodList.size()) {
							LOGERR << "Error in " << _filepath << ":" << i + 1
							       << "..." << endl
							       << "'" << in_txt[i] << "'" << endl
							       << "There are not " << id << " rods" << endl;
							return MOORDYN_INVALID_INPUT;
						}
						if (!strcmp(let2, "A"))
							RodList[id - 1]->addLine(
							    obj, end_point, ENDPOINT_A);
						else if (!strcmp(let2, "B"))
							RodList[id - 1]->addLine(
							    obj, end_point, ENDPOINT_B);
						else {
							LOGERR << "Error in " << _filepath << ":" << i + 1
							       << "..." << endl
							       << "'" << in_txt[i] << "'" << endl
							       << "Rod end (A or B) must be specified"
							       << endl;
							return MOORDYN_INVALID_INPUT;
						}
					} else if (!strlen(let1) || !strcmp(let1, "C") ||
					           !strcmp(let1, "CON")) {
						if (!id || id > ConnectionList.size()) {
							LOGERR << "Error in " << _filepath << ":" << i + 1
							       << "..." << endl
							       << "'" << in_txt[i] << "'" << endl
							       << "There are not " << id << " connections"
							       << endl;
							return MOORDYN_INVALID_INPUT;
						}
						ConnectionList[id - 1]->addLine(obj, end_point);
					} else {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "Unrecognized connection type " << let1
						       << endl;
						return MOORDYN_INVALID_INPUT;
					}
				}
				LOGDBG << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]), { "FAILURE" })) {
			LOGDBG << "   Reading failure conditions:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 4) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "4 fields are required, but just "
					       << entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				FailProps* obj = new FailProps();
				obj->rod = NULL;
				obj->conn = NULL;
				obj->status = false;
				FailList.push_back(obj);

				char let1[10], num1[10], let2[10], num2[10], let3[10];
				char typeWord[10];
				strncpy(typeWord, entries[0].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				str::decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strlen(num1)) {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "No number provided for Node Failure" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				const unsigned int id = atoi(num1);
				if (!strcmp(let1, "R") || !strcmp(let1, "ROD")) {
					if (!id || id > RodList.size()) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There are not " << id << " rods" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					obj->rod = RodList[id - 1];
					if (!strcmp(let2, "A"))
						obj->rod_end_point = ENDPOINT_A;
					else if (!strcmp(let2, "B"))
						obj->rod_end_point = ENDPOINT_B;
					else {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "Failure end (A or B) must be specified"
						       << endl;
						return MOORDYN_INVALID_INPUT;
					}
				} else if (!strlen(let1) || !strcmp(let1, "C") ||
				           !strcmp(let1, "CON")) {
					if (!id || id > ConnectionList.size()) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There are not " << id << " connections"
						       << endl;
						return MOORDYN_INVALID_INPUT;
					}
					obj->conn = ConnectionList[id - 1];
					;
				} else {
					LOGERR << "Error in " << _filepath << ":" << i + 1 << "..."
					       << endl
					       << "'" << in_txt[i] << "'" << endl
					       << "Unrecognized connection type " << let1 << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> lineNums = moordyn::str::split(entries[1], ',');
				obj->lines.reserve(lineNums.size());
				obj->line_end_points.reserve(lineNums.size());
				for (unsigned int il = 0; il < lineNums.size(); il++) {
					const unsigned int line_id = atoi(lineNums[il].c_str());
					if (!line_id || line_id > LineList.size()) {
						LOGERR << "Error in " << _filepath << ":" << i + 1
						       << "..." << endl
						       << "'" << in_txt[i] << "'" << endl
						       << "There are not " << line_id << " lines"
						       << endl;
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

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]), { "OPTIONS" })) {
			LOGDBG << "   Reading options:" << endl;

			i++;
			// Parse options until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 2) {
					i++;
					LOGWRN << "Ignoring option line " << i << endl;
					continue;
				}

				LOGDBG << "\t" << entries[1] << " = " << entries[0] << endl;
				const string value = entries[0];
				const string name = entries[1];
				if (name == "writeLog") {
					// Already registered
					i++;
					continue;
				}
				// DT is old way, should phase out
				else if ((name == "dtM") || (name == "DT"))
					dtM0 = atof(entries[0].c_str());
				else if (name == "tScheme")
					t_integrator_name = entries[0];
				else if ((name == "g") || (name == "gravity"))
					env.g = atof(entries[0].c_str());
				else if ((name == "Rho") || (name == "rho") ||
				         (name == "WtrDnsty"))
					env.rho_w = atof(entries[0].c_str());
				else if (name == "WtrDpth")
					env.WtrDpth = atof(entries[0].c_str());
				else if ((name == "kBot") || (name == "kb"))
					env.kb = atof(entries[0].c_str());
				else if ((name == "cBot") || (name == "cb"))
					env.cb = atof(entries[0].c_str());
				else if ((name == "dtIC") || (name == "ICdt"))
					ICdt = atof(entries[0].c_str());
				else if ((name == "TmaxIC") || (name == "ICTmax"))
					ICTmax = atof(entries[0].c_str());
				else if ((name == "CdScaleIC") || (name == "ICDfac"))
					ICDfac = atof(entries[0].c_str());
				else if ((name == "threshIC") || (name == "ICthresh"))
					ICthresh = atof(entries[0].c_str());
				else if (name == "WaveKin") {
					WaveKinTemp =
					    (moordyn::waves_settings)atoi(entries[0].c_str());
					if ((WaveKinTemp < WAVES_NONE) || (WaveKinTemp > WAVES_KIN))
						LOGWRN << "Unknown WaveKin option value " << WaveKinTemp
						       << endl;
				} else if (name == "dtWave")
					env.dtWave = atoi(entries[0].c_str());
				else if (name == "Currents") {
					env.Current =
					    (moordyn::currents_settings)atoi(entries[0].c_str());
					if ((env.Current < CURRENTS_NONE) ||
					    (env.Current > CURRENTS_DYNAMIC_NODE))
						LOGWRN << "Unknown Currents option value "
						       << env.Current << endl;
				} else if (name == "WriteUnits")
					env.WriteUnits = atoi(entries[0].c_str());
				else if (name == "FrictionCoefficient")
					env.FrictionCoefficient = atof(entries[0].c_str());
				else if (name == "FricDamp")
					env.FricDamp = atof(entries[0].c_str());
				else if (name == "StatDynFricScale")
					env.StatDynFricScale = atof(entries[0].c_str());
				// output writing period (0 for at every call)
				else if (name == "dtOut")
					dtOut = atof(entries[0].c_str());
				else
					LOGWRN << "Warning: Unrecognized option '" << name << "'"
					       << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]), { "OUTPUT" })) {
			LOGDBG << "   Reading output options:" << endl;

			i++;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) &&
			       (i < in_txt.size())) {
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');

				for (unsigned int j = 0; j < entries.size();
				     j++) // loop through each word on each line
				{
					char let1[10], num1[10], let2[10], num2[10], let3[10];
					char typeWord[10];
					typeWord[9] = '\0';
					strncpy(typeWord, entries[j].c_str(), 9);
					// divided outWord into letters and numbers
					str::decomposeString(
					    typeWord, let1, num1, let2, num2, let3);

					// declare dummy struct to be copied onto end of vector (and
					// filled in later);
					OutChanProps dummy;
					strncpy(dummy.Name, typeWord, 10);

					// figure out what type of output it is and process
					// accordingly
					// TODO: add checks of first char of num1,2, let1,2,3 not
					// being NULL to below and handle errors (e.g. invalid line
					// number)

					// The length of dummy.Units is hardcoded to 10 in
					// Misc.h
					const int UnitsSize = 9;
					dummy.Units[UnitsSize] = '\0';

					// fairlead tension case (changed to just be for single
					// line, not all connected lines)
					if (!strcmp(let1, "FAIRTEN")) {
						dummy.OType = 1;
						dummy.QType = Ten;
						strncpy(dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						dummy.ObjID = atoi(num1);
						dummy.NodeID = LineList[dummy.ObjID - 1]->getN();
					}
					// achor tension case (changed to just be for single line,
					// not all connected lines)
					else if (!strcmp(let1, "ANCHTEN")) {
						dummy.OType = 1;
						dummy.QType = Ten;
						strncpy(dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						dummy.ObjID = atoi(num1);
						dummy.NodeID = 0;
					}
					// more general case
					else {
						// get object type and node number if applicable
						// Line case:  L?N?xxxx
						if (!strcmp(let1, "L")) {
							dummy.OType = 1;
							dummy.NodeID = atoi(num2);
						}
						// Connect case:   C?xxx or Con?xxx
						else if (!strcmp(let1, "C") || !strcmp(let1, "CON")) {
							dummy.OType = 2;
							dummy.NodeID = -1;
							strncpy(let3, let2, 10);
						}
						// Rod case:   R?xxx or Rod?xxx
						else if (!strcmp(let1, "R") || !strcmp(let1, "ROD")) {
							dummy.OType = 3;
							dummy.NodeID = atoi(num2);
						}
						// should do fairlead option also!

						else {
							LOGWRN << "Warning in " << _filepath << ":" << i + 1
							       << "..." << endl
							       << "'" << in_txt[i] << "'" << endl
							       << "invalid output specifier: " << let1
							       << ".  Type must be oneof L, C/Con or R/Rod"
							       << endl;
							dummy.OType = -1;
							continue;
						}

						// object number
						dummy.ObjID = atoi(num1);

						if (!strcmp(let3, "PX")) {
							// cout << "SETTING QTYPE to " << PosX << endl;
							dummy.QType = PosX;
							strncpy(dummy.Units,
							        moordyn::UnitList[PosX],
							        UnitsSize);
						} else if (!strcmp(let3, "PY")) {
							dummy.QType = PosY;
							strncpy(dummy.Units,
							        moordyn::UnitList[PosY],
							        UnitsSize);
						} else if (!strcmp(let3, "PZ")) {
							dummy.QType = PosZ;
							strncpy(dummy.Units,
							        moordyn::UnitList[PosZ],
							        UnitsSize);
						} else if (!strcmp(let3, "VX")) {
							dummy.QType = VelX;
							strncpy(dummy.Units,
							        moordyn::UnitList[VelX],
							        UnitsSize);
						} else if (!strcmp(let3, "VY")) {
							dummy.QType = VelY;
							strncpy(dummy.Units,
							        moordyn::UnitList[VelY],
							        UnitsSize);
						} else if (!strcmp(let3, "VZ")) {
							dummy.QType = VelZ;
							strncpy(dummy.Units,
							        moordyn::UnitList[VelZ],
							        UnitsSize);
						} else if (!strcmp(let3, "AX")) {
							dummy.QType = AccX;
							strncpy(dummy.Units,
							        moordyn::UnitList[AccX],
							        UnitsSize);
						} else if (!strcmp(let3, "Ay")) {
							dummy.QType = AccY;
							strncpy(dummy.Units,
							        moordyn::UnitList[AccY],
							        UnitsSize);
						} else if (!strcmp(let3, "AZ")) {
							dummy.QType = AccZ;
							strncpy(dummy.Units,
							        moordyn::UnitList[AccZ],
							        UnitsSize);
						} else if (!strcmp(let3, "T") || !strcmp(let3, "TEN")) {
							dummy.QType = Ten;
							strncpy(
							    dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						} else if (!strcmp(let3, "FX")) {
							dummy.QType = FX;
							strncpy(
							    dummy.Units, moordyn::UnitList[FX], UnitsSize);
						} else if (!strcmp(let3, "FY")) {
							dummy.QType = FY;
							strncpy(
							    dummy.Units, moordyn::UnitList[FY], UnitsSize);
						} else if (!strcmp(let3, "FZ")) {
							dummy.QType = FZ;
							strncpy(
							    dummy.Units, moordyn::UnitList[FZ], UnitsSize);
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
						if (dummy.NodeID > 0)
							strncpy(dummy.Name, "TenEndB", 10);
						else
							strncpy(dummy.Name, "TenEndA", 10);
					}

					if ((dummy.OType > 0) && (dummy.QType > 0))
						outChans.push_back(dummy);
				}

				i++;
			}
		}

		i++;
	}

	// do some input validity checking?
	// should there be a flag in the input file that clearly distingiushes the
	// coupling type?
	// <<<<< I guess it's implied by whether bodies are coupled or not??

	// TODO: make sure things are consistent for only ONE coupling type (body
	// centric or fairlead centric)
	// <<<<<<<<<<<<<<<< also do checks when time step function is called...

	LOGMSG << "Generated entities:" << endl
	       << "\tnLineTypes  = " << LinePropList.size() << endl
	       << "\tnRodTypes   = " << RodPropList.size() << endl
	       << "\tnPoints     = " << ConnectionList.size() << endl
	       << "\tnBodies     = " << BodyList.size() << endl
	       << "\tnRods       = " << RodList.size() << endl
	       << "\tnLines      = " << LineList.size() << endl
	       << "\tnFails      = " << FailList.size() << endl
	       << "\tnFreeBodies = " << FreeBodyIs.size() << endl
	       << "\tnFreeRods   = " << FreeRodIs.size() << endl
	       << "\tnFreePonts  = " << FreeConIs.size() << endl
	       << "\tnCpldBodies = " << CpldBodyIs.size() << endl
	       << "\tnCpldRods   = " << CpldRodIs.size() << endl
	       << "\tnCpldPoints = " << CpldConIs.size() << endl;

	// write system description
	LOGDBG << "----- MoorDyn Model Summary (to be written) -----" << endl;

	// Setup the time integrator
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		_t_integrator = create_time_scheme(t_integrator_name, _log);
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS) {
		LOGERR << err_msg << endl;
		return err;
	}
	LOGMSG << "Time integrator = " << _t_integrator->GetName() << endl;
	_t_integrator->SetGround(GroundBody);
	for (auto obj : BodyList)
		_t_integrator->AddBody(obj);
	for (auto obj : RodList)
		_t_integrator->AddRod(obj);
	for (auto obj : ConnectionList)
		_t_integrator->AddConnection(obj);
	for (auto obj : LineList)
		_t_integrator->AddLine(obj);

	// Setup the waves and populate them
	waves = new moordyn::Waves(_log);
	try {
		waves->setup(&env, _t_integrator, _basepath.c_str());
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS)
		return err;

	GroundBody->setEnv(&env, waves);
	for (auto obj : BodyList)
		obj->setEnv(&env, waves);
	for (auto obj : RodList)
		obj->setEnv(&env, waves);
	for (auto obj : ConnectionList)
		obj->setEnv(&env, waves);
	for (auto obj : LineList)
		obj->setEnv(&env, waves);

	return MOORDYN_SUCCESS;
}

void
moordyn::MoorDyn::detachLines(FailProps* failure)
{
	failure->status = true;
	if (failure->rod && failure->conn) {
		LOGERR << "The failure criteria considers both a rod and a connection"
		       << endl;
		throw moordyn::unhandled_error("Invalid failure data");
	} else if (!failure->rod && !failure->conn) {
		LOGERR << "The failure criteria is missing either a rod or a connection"
		       << endl;
		throw moordyn::unhandled_error("Invalid failure data");
	}

	// create new massless connection for detached end(s) of line(s)
	const real M = 0.0;
	const real V = 0.0;
	const vec r0 = vec::Zero();
	const vec F = vec::Zero();
	const real CdA = 0.0;
	const real Ca = 0.0;
	const Connection::types type = Connection::FREE;

	nX += 6; // add 6 state variables for each connect

	// add connect to list of free ones and add states for it
	FreeConIs.push_back(ConnectionList.size());
	// assign start index of this connect's states
	ConnectStateIs.push_back(nX);

	// now make Connection object!
	Connection* obj = new Connection(_log);
	obj->setup(ConnectionList.size() + 1, type, r0, M, V, F, CdA, Ca);
	obj->setEnv(&env, waves);
	ConnectionList.push_back(obj);

	// Kinematics of old attachment point
	vec pos, vel;
	if (failure->rod) {
		const unsigned int node =
		    (failure->rod_end_point == ENDPOINT_A) ? 0 : failure->rod->getN();
		pos = failure->rod->getNodePos(node);
		vel = failure->rod->getNodeVel(node);
	} else {
		std::tie(pos, vel) = failure->conn->getState();
	}

	// detach lines from old Rod or Connection, and get kinematics of the old
	// attachment point
	for (unsigned int i = 0; i < failure->lines.size(); i++) {
		if (failure->rod)
			failure->line_end_points[i] = failure->rod->removeLine(
			    failure->rod_end_point, failure->lines[i]);
		else
			failure->line_end_points[i] =
			    failure->conn->removeLine(failure->lines[i]);

		obj->addLine(failure->lines[i], failure->line_end_points[i]);
	}

	// update connection kinematics to match old line attachment point
	// kinematics and set positions of attached line ends
	obj->setState(pos, vel);
}

moordyn::error_id
moordyn::MoorDyn::AllOutput(double t, double dt)
{
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
	return (MoorDynWaves)(((moordyn::MoorDyn*)system)->GetWaves());
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
	*n = ((moordyn::MoorDyn*)system)->GetBodies().size();
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
	*n = ((moordyn::MoorDyn*)system)->GetRods().size();
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
MoorDyn_GetNumberConnections(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ((moordyn::MoorDyn*)system)->GetConnections().size();
	return MOORDYN_SUCCESS;
}

MoorDynConnection DECLDIR
MoorDyn_GetConnection(MoorDyn system, unsigned int l)
{
	if (!system)
		return NULL;
	auto conns = ((moordyn::MoorDyn*)system)->GetConnections();
	if (!l || (l > conns.size())) {
		cerr << "Error: There is not such connection " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return NULL;
	}
	return (MoorDynConnection)(conns[l - 1]);
}

int DECLDIR
MoorDyn_GetNumberLines(MoorDyn system, unsigned int* n)
{
	CHECK_SYSTEM(system);
	*n = ((moordyn::MoorDyn*)system)->GetLines().size();
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

int DECLDIR MoorDyn_Serialize(MoorDyn system,
	                          size_t* size,
	                          uint64_t* data)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		const std::vector<uint64_t> backup = ((moordyn::MoorDyn*)system)->Serialize();
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

int DECLDIR MoorDyn_Deserialize(MoorDyn system,
	                            const uint64_t* data)
{
	CHECK_SYSTEM(system);
	moordyn::error_id err = MOORDYN_SUCCESS;
	if (!data) {
		cerr << "Error: No data has been provided to "
	         << __FUNC_NAME__ << "()" << endl;
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
	for (auto conn : ((moordyn::MoorDyn*)system)->GetConnections())
		conn->drawGL();
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
