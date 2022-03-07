/*
 * Copyright (c) 2019 Matt Hall <mtjhall@alumni.uvic.ca>
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

 // This is version 2.a5, 2021-03-16
 
#include "Misc.h"
#include "Waves.h"
#include "MoorDyn2.h"
#include "MoorDyn2.hpp"
#include "Line.h"
#include "Connection.h" 
#include "Rod.h" 
#include "Body.h"

#ifdef LINUX
	#include <cmath>
	#include <ctype.h>

	// contributed by Yi-Hsiang Yu at NREL
	#define isnan(x) std::isnan(x)
#endif

using namespace std;

namespace moordyn
{

/// The list of units for the output
const char *UnitList[] = {"(s)     ", "(m)     ", "(m)     ", "(m)     ", 
                          "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
                          "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
                          "(N)     ", "(N)     "};


moordyn::MoorDyn::MoorDyn(const char *infilename, const int verbosity)
	: Log(verbosity)
	, _filepath("Mooring/lines.txt")
	, _basename("lines")
	, _basepath("Mooring/")
	, ICDfac(5.0)
	, ICdt(1.0)
	, ICTmax(120.0)
	, ICthresh(0.001)
	, WaveKinTemp(0)
	, dtM0(0.001)
	, dtOut(0.0)
	, GroundBody(NULL)
	, waves(NULL)
	, nX(0)
	, nXtra(0)
	, states(NULL)
	, xt(NULL)
	, f0(NULL)
	, f1(NULL)
	, npW(0)
	, tW_1(0.0)
	, U_1(NULL)
	, Ud_1(NULL)
	, tW_2(0.0)
	, U_2(NULL)
	, Ud_2(NULL)
{
	if (infilename && (strlen(infilename) > 0))
	{
		_filepath = infilename;
		const int lastSlash = _filepath.find_last_of("/\\");
		const int lastDot = _filepath.find_last_of('.');
		_basename = _filepath.substr(lastSlash + 1, lastDot - lastSlash - 1);
		_basepath = _filepath.substr(0, lastSlash + 1);
	}

	Cout(MOORDYN_MSG_LEVEL)
		<< "\n Running MoorDyn (v2.a10, 2022-01-01)" << endl
		<< "   NOTE: This is an alpha version of MoorDyn v2, intended for testing and debugging." << endl
		<< "         MoorDyn v2 has significant ongoing input file changes from v1." << endl
		<< "   Copyright: (C) 2021 National Renewable Energy Laboratory, (C) 2014-2019 Matt Hall" << endl
		<< "   This program is released under the GNU General Public License v3." << endl;

	Cout(MOORDYN_MSG_LEVEL) << "The filename is " << _filepath << endl;
	Cout(MOORDYN_DBG_LEVEL) << "The basename is " << _basename << endl;
	Cout(MOORDYN_DBG_LEVEL) << "The basepath is " << _basepath << endl;

	env.g = 9.8;
	env.WtrDpth = 0.;
	env.rho_w = 1025.;
	env.kb = 3.0e6;
	env.cb = 3.0e5;
	env.WaveKin = 0;                // 0=none
	env.Current = 0;                // 0=none
	env.dtWave = 0.25;
	env.WriteUnits = 1;             // by default, write units line
	env.writeLog = 0;               // by default, don't write out a log file
	env.FrictionCoefficient = 0.0;
	env.FricDamp = 200.0;
	env.StatDynFricScale = 1.0;

	const moordyn::error_id err = ReadInFile();
	MOORDYN_THROW(err, "Exception while reading the input file");

	Cout(MOORDYN_DBG_LEVEL) << "MoorDyn is expecting " << NCoupedDOF()
	                        << " coupled degrees of freedom" << endl;

	if (!nX)
	{
		Cout(MOORDYN_WRN_LEVEL) << "WARNING: MoorDyn has no state variables."
		                        << " (Is there a mooring sytem?)" << endl;
	}

	nXtra = nX + 6 * 2 * LineList.size();
	Cout(MOORDYN_DBG_LEVEL) << "Creating state vectors of size "
	                        << nXtra << endl;
	states = (double*) malloc(nXtra * sizeof(double));
	xt = (double*) malloc(nXtra * sizeof(double));
	f0 = (double*) malloc(nXtra * sizeof(double));
	f1 = (double*) malloc(nXtra * sizeof(double));
	if (!states || !f0 || !f1 || !xt)
	{
		MOORDYN_THROW(MOORDYN_MEM_ERROR,
		              "Error allocating memory for state variables");
	}

	memset(states, 0.0, nXtra * sizeof(double));
	memset(xt, 0.0, nXtra * sizeof(double));
	memset(f0, 0.0, nXtra * sizeof(double));
	memset(f1, 0.0, nXtra * sizeof(double));
}

moordyn::MoorDyn::~MoorDyn()
{
	if (outfileLog.is_open())
		outfileLog.close();
	if (outfileMain.is_open())
		outfileMain.close();
	for (auto outfile : outfiles)  // int l=0; l<nLines; l++)
		if (outfile && outfile->is_open())
			outfile->close();

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

	free(states);
	free(xt);
	free(f0);
	free(f1);
}

moordyn::error_id moordyn::MoorDyn::Init(const double *x, const double *xd)
{
	if (NCoupedDOF() && !x)
	{
		Cout(MOORDYN_ERR_LEVEL) << "ERROR: "
			<< "MoorDyn::Init received a Null position vector, "
			<< "but " << NCoupedDOF() << "components are required" << endl;
	}

	// <<<<<<<<< need to add bodys
 
	// Allocate past line fairlead tension array, which is used for convergence
	// test during IC gen
	const unsigned int convergence_iters = 10;
	double **FairTensLast = make2Darray(LineList.size(), convergence_iters);
	for (unsigned int i = 0; i < LineList.size(); i++)
		for (unsigned int j = 0; j < convergence_iters; j++)
			FairTensLast[i][j] = 1.0 * j;

	// ------------------ do static bodies and lines ---------------------------

	Cout(MOORDYN_MSG_LEVEL) << "Creating mooring system..." << endl;

	// call ground body to update all the fixed things...
	GroundBody->initializeUnfreeBody(NULL, NULL, 0.0);

	// initialize coupled objects based on passed kinematics
	int ix = 0;

	for (auto l : CpldBodyIs)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Body " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		// this calls initiateStep and updateFairlead, then initializes
		// dependent Rods
		BodyList[l]->initializeUnfreeBody(x + ix, xd + ix, 0.0);
		ix += 6;
	}

	for (auto l : CpldRodIs)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Rod " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		RodList[l]->initiateStep(x + ix, xd + ix, 0.0);
		RodList[l]->updateFairlead(0.0);
		RodList[l]->initializeRod(NULL);  // call this just to set up the output file header
		
		if (RodList[l]->type == -2)
			ix += 6;  // for cantilevered rods 6 entries will be taken
		else
			ix += 3;  // for pinned rods 3 entries will be taken
	}

	for (auto l : CpldConIs)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Connection " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		ConnectionList[l]->initiateStep(x + ix, xd + ix, 0.0);

		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			ConnectionList[l]->updateFairlead(0.0);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error initializing coupled connection"
				<< l << ": " << err_msg << endl;
			return err;
		}
		ConnectionList[l]->initializeConnect(NULL);  // call this just to set WaterKin (may also set up output file in future)
		ix += 3;
	}

	// initialize objects with states, writing their initial states to the
	//  master state vector (states)

	// Go through Bodys and write the coordinates to the state vector
	for (unsigned int l = 0; l < FreeBodyIs.size(); l++)
		BodyList[FreeBodyIs[l]]->initializeBody(states + BodyStateIs[l]);

	// Go through independent (including pinned) Rods and write the coordinates to the state vector
	for (unsigned int l = 0; l < FreeRodIs.size(); l++)
		RodList[FreeRodIs[l]]->initializeRod(states + RodStateIs[l]);

	// Go through independent connections (Connects) and write the coordinates to 
	// the state vector and set positions of attached line ends
	for (unsigned int l = 0; l < FreeConIs.size(); l++)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			ConnectionList[FreeConIs[l]]->initializeConnect(
				states + ConnectStateIs[l]);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error initializing free connection"
				<< FreeConIs[l] << ": " << err_msg << endl;
			return err;
		}
	}


	// Lastly, go through lines and initialize internal node positions using quasi-static model
	for (unsigned int l = 0; l < LineList.size(); l++)
		LineList[l]->initializeLine(states + LineStateIs[l]);

	// ------------------ do dynamic relaxation IC gen --------------------
	
	Cout(MOORDYN_MSG_LEVEL) << "Finalizing ICs using dynamic relaxation ("
	                        << ICDfac << "X normal drag)" << endl;
	
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
	vector<double> FairTens(LineList.size(), 0.0);

	unsigned int iic = 0;
	double t = 0;
	bool converged = true;
	double max_error = 0.0;
	while (t < ICTmax)
	{
		// Integrate one ICD timestep (ICdt)
		double t_target = t + ICdt;
		double dt;
		while ((dt = t_target - t) > 0.0)
		{
			if (dtM0 < dt)
				dt = dtM0;
			const moordyn::error_id err = RK2(states, t, dt);
			if (err != MOORDYN_SUCCESS)
			{
				free2Darray(FairTensLast, LineList.size());
				return err;
			}
		}

		// check for NaNs (actually it was already done in RK2)
		for (unsigned int i = 0; i < nX; i++)
		{
			if (isnan(states[i]))
			{
				Cout(MOORDYN_ERR_LEVEL) << "Error: NaN value detected "
				    << "in MoorDyn state at dynamic relaxation time "
					<< t << " s." << endl;
				free2Darray(FairTensLast, LineList.size());
				return MOORDYN_NAN_ERROR;
			}
		}

		// Roll previous fairlead tensions for comparison
		for (unsigned int lf = 0; lf < LineList.size(); lf++)
		{
			for (int pt = convergence_iters - 1; pt > 0; pt--)
				FairTensLast[lf][pt] = FairTensLast[lf][pt - 1];
			FairTensLast[lf][0] = FairTens[lf];
		}

		// go through connections to get fairlead forces 
		for (unsigned int lf = 0; lf < LineList.size(); lf++)
			FairTens[lf] = LineList[lf]->getNodeTen(LineList[lf]->getN());

		// check for convergence (compare current tension at each fairlead with
		// previous convergence_iters-1 values)
		if (iic > convergence_iters)
		{
			// check for any non-convergence, and continue to the next time step
			// if any occurs
			converged = true;
			max_error = 0.0;
			for (unsigned int lf = 0; lf < LineList.size(); lf++)
			{
				for (unsigned int pt = 0; pt < convergence_iters; pt++)
				{
					const double error = abs(
						FairTens[lf] / FairTensLast[lf][pt] - 1.0);
					if (error > max_error)
						max_error = error;
					if (max_error > ICthresh)
					{
						converged = false;
						break;
					}
				}
				if (!converged)
				{
					Cout(MOORDYN_DBG_LEVEL) << "Dynamic relaxation t = "
						<< t << "s (time step " << iic << "), error = "
						<< 100.0 * max_error << "%     \r";
					break;
				}
			}

			if (converged)
				break;
		}

		iic++;
	}

	if (converged) {
		Cout(MOORDYN_MSG_LEVEL) << "Fairlead tensions converged" << endl;
	} else {
		Cout(MOORDYN_WRN_LEVEL) << "Fairlead tensions did not converged" << endl;
	}
	Cout(MOORDYN_MSG_LEVEL) << "Remaining error after " << t << " s = "
	                        << 100.0 * max_error << "%" << endl;

	free2Darray(FairTensLast, LineList.size());

	// restore drag coefficients to normal values and restart time counter of each object
	for (auto obj : LineList)
	{
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : ConnectionList)
	{
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : RodList)
	{
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}
	for (auto obj : BodyList)
	{
		obj->scaleDrag(1.0 / ICDfac);
		obj->setTime(0.0);
	}

	// store passed WaveKin value to enable waves in simulation if applicable
	// (they're not enabled during IC gen)
	env.WaveKin = WaveKinTemp;

	// @mth: new approach to be implemented
	// ------------------------- calculate wave time series if needed -------------------
// 	if (env.WaveKin == 2)
// 	{
// 		for (int l=0; l<LineList.size(); l++)
// 			LineList[l]->makeWaveKinematics( 0.0 );
// 	}

	// -------------------------- start main output file --------------------------------

	stringstream oname;
	oname << _basepath << _basename << ".out";

	outfileMain.open(oname.str());
	if (!outfileMain.is_open())
	{
		Cout(MOORDYN_ERR_LEVEL) << "ERROR: Unable to write to main output file "
			<< oname.str() << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}

	// --- channel titles ---
	outfileMain << "Time" << "\t ";
	for (auto channel : outChans)
		outfileMain << channel.Name << "\t ";
	outfileMain << endl;

	if (env.WriteUnits > 0)
	{
		// --- units ---
		outfileMain << "(s)" << "\t ";
		for (auto channel : outChans)
			outfileMain << channel.Units << "\t ";
		outfileMain << "\n";
	}

	// write t=0 output
	return AllOutput(0.0, 0.0);
}

moordyn::error_id moordyn::MoorDyn::Step(const double *x,
                                         const double *xd,
                                         double *f,
                                         double &t,
                                         double &dt)
{
	// should check if wave kinematics have been set up if expected!
	Cout(MOORDYN_DBG_LEVEL) << "t = " << t << "s     \r";

	if (dt <= 0)
	{
		// Nothing to do, just recover the forces
		return GetForces(f);
	}

	if (NCoupedDOF() && (!x || !xd || !f))
	{
		Cout(MOORDYN_ERR_LEVEL) << "Null Pointer received in "
			<< __FUNC_NAME__
			<< " (" << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;
	}

	unsigned int ix = 0;

	// ---------------- set positions and velocities -----------------------
	// ... of any coupled bodies, rods, and connections at this instant, to be
	// used later for extrapolating motions
	for (auto l : CpldBodyIs)
	{
		BodyList[l]->initiateStep(x + ix, xd + ix, t);
		ix += 6;
	}
	for (auto l : CpldRodIs)
	{
		RodList[l]->initiateStep(x + ix, xd + ix, t);
		if (RodList[CpldRodIs[l]]->type == -2)
			ix += 6;  // for cantilevered rods 6 entries will be taken
		else
			ix += 3;  // for pinned rods 3 entries will be taken
	}
	for (auto l : CpldConIs)
	{
		ConnectionList[l]->initiateStep(x + ix, xd + ix, t);
		ix += 3;
	}

	// -------------------- do time stepping -----------------------
	double t_target = t + dt;
	double dt_step;
	while ((dt_step = t_target - t) > 0.0)
	{
		if (dtM0 < dt_step)
			dt_step = dtM0;
		const moordyn::error_id err = RK2(states, t, dt_step);
		if (err != MOORDYN_SUCCESS)
			return err;
	}
	t = t_target;

	// check for NaNs (actually it was already done in RK2)
	for (unsigned int i = 0; i < nX; i++)
	{
		if (isnan(states[i]))
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: NaN value detected "
			                        << "in MoorDyn state " << i
			                        << " at time " << t << " s" << endl;
			return MOORDYN_NAN_ERROR;
		}
	}

	// --------------- check for line failures (detachments!) ----------------
	// step 1: check for time-triggered failures
	for (unsigned int l = 0; l < FailList.size(); l++)
	{
		if (FailList[l]->failStatus)
			continue;
		if (t >= FailList[l]->failTime)
		{
			Cout(MOORDYN_MSG_LEVEL) << "Failure number " << l + 1
				<< " triggered at time " << t << endl;
			FailList[l]->failStatus = 1;
			const moordyn::error_id err = detachLines(
				FailList[l]->attachID,
				FailList[l]->isRod,
				FailList[l]->lineIDs,
				FailList[l]->lineTops,
				FailList[l]->nLinesToDetach,
				t);
			if (err)
				return err;
		}
	}

	// step 2: check for tension-triggered failures (this will require specifying max tension things)

	// ------------------------ write outputs --------------------------
	const moordyn::error_id err = AllOutput(t, dt);
	if (err != MOORDYN_SUCCESS)
		return err;

	return GetForces(f);
}

moordyn::error_id moordyn::MoorDyn::ReadInFile()
{
	unsigned int i=0;

	// factor by which to boost drag coefficients during dynamic relaxation IC generation
	ICDfac = 5.0;
	// convergence analysis time step for IC generation
	ICdt = 1.0;
	// max time for IC generation
	ICTmax = 120;
	// threshold for relative change in tensions to call it converged
	ICthresh = 0.001;
	// temporary wave kinematics flag used to store input value while keeping env.WaveKin=0 for IC gen
	WaveKinTemp = 0;
	// assume no wave kinematics points are passed in externally, unless ExernalWaveKinInit is called later
	npW = 0;
	// default value for desired mooring model time step
	dtM0 = 0.001;

	// string containing which channels to write to output
	vector<string> outchannels;

	// make a "ground body" that will be the parent of all fixed objects (connections and rods)
	Cout(MOORDYN_DBG_LEVEL) << "Creating the ground body of type "
		<< Body::TypeName(Body::FIXED) << "..." << endl;
	GroundBody = new Body();
	GroundBody->setup(0, Body::FIXED, NULL, NULL, 0.0, 0.0, NULL, NULL, NULL, NULL);

	// Make sure the state vector counter starts at zero
	// This will be conveniently incremented as each object is added
	nX = 0;

	// Load the mooring lines input file in lines of text
	ifstream in_file(_filepath);
	vector<string> in_txt;
	string line;
	if (!in_file.is_open())
	{
		Cout(MOORDYN_ERR_LEVEL) << "Error: unable to open file '"
		                        << _filepath << "'" << endl;
		return MOORDYN_INVALID_INPUT_FILE;
	}
	while (in_file.good())
	{
		string line_txt;
		getline(in_file, line_txt);
		in_txt.push_back(line_txt);
	}
	in_file.close();

	// Parse the input file
	i=0;
	while (i < in_txt.size())
	{
		// look for the section header line
		if (in_txt[i].find("---") == string::npos)
		{
			i++;
			continue;
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"LINE DICTIONARY", "LINE TYPES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading line types:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 10)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "10 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				LineProps *obj = new LineProps();

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

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << obj->type << "'"
					<< " - with id " << LinePropList.size() << endl;
				if (env.writeLog > 1)
				{
					outfileLog << "  - LineType" << LinePropList.size() << ":"
					          << endl
					          << "    name: " << obj->type << endl
					          << "    d   : " << obj->d    << endl
					          << "    w   : " << obj->w    << endl
					          << "    Cdn : " << obj->Cdn  << endl
					          << "    Can : " << obj->Can  << endl
					          << "    Cdt : " << obj->Cdt  << endl
					          << "    Cat : " << obj->Cat  << endl;
				}

				LinePropList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"ROD DICTIONARY", "ROD TYPES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading rod types:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 7)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "7 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				RodProps *obj = new RodProps();
				obj->type = entries[0];
				obj->d = atof(entries[1].c_str());
				obj->w = atof(entries[2].c_str());
				obj->Cdn = atof(entries[3].c_str());
				obj->Can = atof(entries[4].c_str());
				obj->Cdt = atof(entries[5].c_str());
				obj->Cat = atof(entries[6].c_str());

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << obj->type << "'"
					<< " - with id " << RodPropList.size() << endl;
				if (env.writeLog > 1)
				{
					outfileLog << "  - RodType" << RodPropList.size() << ":"
					          << endl
					          << "    name: " << obj->type << endl
					          << "    d   : " << obj->d    << endl
					          << "    w   : " << obj->w    << endl
					          << "    Cdn : " << obj->Cdn  << endl
					          << "    Can : " << obj->Can  << endl
					          << "    Cdt : " << obj->Cdt  << endl
					          << "    Cat : " << obj->Cat  << endl;
				}

				RodPropList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"BODIES", "BODY LIST", "BODY PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading body list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 14)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "14 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				const int number = atoi(entries[0].c_str());
				Body::types type;
				double r6[6];
				for (unsigned int I=0; I<6; I++) 
					r6[I] = atof(entries[2 + I].c_str());
				double M = atof(entries[8].c_str());
				double V = atof(entries[11].c_str());

				double rCG[3];
				double Inert[3];
				double CdA[3];
				double Ca[3];

				vector<string> entries_rCG = moordyn::str::split(entries[9],
				                                                 '|');
				if (entries_rCG.size() == 1)
				{
					// if only one entry, it is the z coordinate
					rCG[0] = 0.0;
					rCG[1] = 0.0;
					rCG[2] = atof(entries_rCG[0].c_str());
				}
				else if (entries_rCG.size() == 3)
				{
					rCG[0] = atof(entries_rCG[0].c_str());
					rCG[1] = atof(entries_rCG[1].c_str());
					rCG[2] = atof(entries_rCG[2].c_str());
				}
				else {
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "CG entry (col 10) must have 1 or 3 numbers" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_I = moordyn::str::split(entries[10],
				                                               '|');
				if (entries_I.size() == 1)
				{
					// if only one entry, use it for all directions
					Inert[0] = atof(entries_I[0].c_str());
					Inert[1] = Inert[0];
					Inert[2] = Inert[0];
				}
				else if (entries_I.size() == 3)
				{
					Inert[0] = atof(entries_I[0].c_str());
					Inert[1] = atof(entries_I[1].c_str());
					Inert[2] = atof(entries_I[2].c_str());
				}
				else {
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Inertia entry (col 11) must have 1 or 3 numbers" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_CdA = moordyn::str::split(entries[12],
				                                                 '|');
				if (entries_CdA.size() == 1)
				{
					// if only one entry, use it for all directions
					CdA[0] = atof(entries_CdA[0].c_str());
					CdA[1] = CdA[0];
					CdA[2] = CdA[0];
				}
				else if (entries_CdA.size() == 3)
				{
					CdA[0] = atof(entries_CdA[0].c_str());
					CdA[1] = atof(entries_CdA[1].c_str());
					CdA[2] = atof(entries_CdA[2].c_str());
				}
				else {
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "CdA entry (col 13) must have 1 or 3 numbers" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> entries_Ca = moordyn::str::split(entries[13],
				                                                '|');
				if (entries_Ca.size() == 1)
				{
					// if only one entry, use it for all directions
					Ca[0] = atof(entries_Ca[0].c_str());
					Ca[1] = Ca[0];
					Ca[2] = Ca[0];
				}
				else if (entries_Ca.size() == 3)
				{
					Ca[0] = atof(entries_Ca[0].c_str());
					Ca[1] = atof(entries_Ca[1].c_str());
					Ca[2] = atof(entries_Ca[2].c_str());
				}
				else {
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Ca entry (col 14) must have 1 or 3 numbers" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				char let1[10], num1[10], let2[10], num2[10], let3[10]; 
				char typeWord[10];
				strncpy(typeWord, entries[1].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
				{
					// it is fixed  (this would just be used if someone wanted to temporarly fix a body that things were attached to)
					type = Body::FIXED;
				}
				else if (!strcmp(let1, "COUPLED") || !strcmp(let1, "VESSEL") || !strcmp(let1, "VES") || !strcmp(let1, "CPLD"))
				{
					// it is coupled - controlled from outside
					type = Body::COUPLED;
					CpldBodyIs.push_back(BodyList.size());
				}
				else 
				{
					// it is free - controlled by MoorDyn
					type = Body::FREE;
					FreeBodyIs.push_back(BodyList.size());
					BodyStateIs.push_back(nX);       // assign start index of this body's states
					nX += 12;                        // add 12 state variables for the body
				}
				stringstream oname;
				oname << _basepath << _basename << "_Body" << number << ".out";
				outfiles.push_back(make_shared<ofstream>(oname.str()));
				if (!outfiles.back()->is_open())
				{
					Cout(MOORDYN_ERR_LEVEL) << "Cannot create the output file '"
						<< oname.str() << endl;
					return MOORDYN_INVALID_OUTPUT_FILE;
				}

				Body *obj = new Body();
				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of type " << Body::TypeName(type)
					<< " with id " << BodyList.size() << endl;
				obj->setup(number, type, r6, rCG, M, V, Inert, CdA, Ca,
				           outfiles.back());
				BodyList.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"RODS", "ROD LIST", "ROD PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading rod list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 8)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "8 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				int number = atoi(entries[0].c_str());
				string RodType = entries[1];
				double endCoords[6]; 
				for (unsigned int I=0; I<6; I++)
					endCoords[I] = atof(entries[3 + I].c_str());
				int NumSegs = atoi(entries[9].c_str());
				string outchannels = entries[10];

				char let1[10], num1[10], let2[10], num2[10], let3[10]; 
				char typeWord[10];
                strncpy(typeWord, entries[2].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				decomposeString(typeWord, let1, num1, let2, num2, let3);

				Rod::types type;
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
				{
					// it is fixed  (this would just be used if someone wanted to temporarly fix a body that things were attached to)
					type = Rod::FIXED;
				}
				else if (!strcmp(let1, "PINNED") || !strcmp(let1, "PIN"))
				{
					// it is pinned
					type = Rod::PINNED;
					FreeRodIs.push_back(RodList.size());  // add this pinned rod to the free list because it is half free
					RodStateIs.push_back(nX);       // assign start index of this rod's states
					nX += 6;                       // add 6 state variables for each pinned rod
				}
				else if (!strcmp(let1, "BODY"))
				{
					if (!strlen(num1))
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "no number provided for Rod " << number
							<< " Body attachment" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int bodyID = atoi(num1);
					if (!bodyID || (bodyID > BodyList.size()))
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "There is not " << bodyID << " bodies" << endl;
						return MOORDYN_INVALID_INPUT;
					}

					if (!strcmp(let2, "PINNED") || !strcmp(let2, "PIN"))
					{
						// it is pinned
						type = Rod::PINNED;
						FreeRodIs.push_back(RodList.size());  // add this pinned rod to the free list because it is half free
						RodStateIs.push_back(nX);       // assign start index of this rod's states
						nX += 6;                       // add 6 state variables for each pinned rod
					}
					else
					{
						type = Rod::FIXED;
					}
				}
				else if (!strcmp(let1, "VESSEL") || !strcmp(let1, "VES") || !strcmp(let1, "COUPLED") || !strcmp(let1, "CPLD"))
				{
					// if a rigid fairlead, add to list and add 
					type = Rod::COUPLED;
					CpldRodIs.push_back(RodList.size());  // index of fairlead in RodList vector
				}
				else if (!strcmp(let1, "VESPIN") || !strcmp(let1, "CPLDPIN"))
				{
					// if a pinned fairlead, add to list and add 
					type = Rod::CPLDPIN;
					CpldRodIs.push_back(RodList.size());  // index of fairlead in RodList vector
					FreeRodIs.push_back(RodList.size());     // also add this pinned rod to the free list because it is half free
					RodStateIs.push_back(nX);          // assign start index of this rod's states
					nX += 6;                          // add 6 state variables for each pinned rod
				}
				else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") || !strcmp(let1, "FREE"))
				{
					type = Rod::FREE;
					FreeRodIs.push_back(RodList.size());  // add this free rod to the free list
					RodStateIs.push_back(nX);       // assign start index of this rod's states
					nX += 12;                      // add 12 state variables for each free rod
				}
				else 
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Unrecognized connection type '" << let1 << "'"
						<< endl;
					return MOORDYN_INVALID_INPUT;
				}

				int TypeNum = -1;
				for (unsigned int J = 0; J < RodPropList.size(); J++)  {
					if (RodPropList[J]->type == RodType)
						TypeNum = J;
				}
				if (TypeNum == -1)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Unrecognized rod type " << RodType << endl;
					return MOORDYN_INVALID_INPUT;
				}

				// Make the output file (if queried)
				if ((outchannels.size() > 0) &&
					(strcspn(outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))
				{
					// if 1+ output flag chars are given and they're valid
					stringstream oname;
					oname << _basepath << _basename << "_Rod" << number << ".out";
					outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!outfiles.back()->is_open())
					{
						Cout(MOORDYN_ERR_LEVEL)
							<< "Cannot create the output file '"
							<< oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				}
				else
					outfiles.push_back(NULL);

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of class " << RodType << " (" << TypeNum << ")"
					<< " and type " << Rod::TypeName(type)
					<< " with id " << RodList.size() << endl;

				Rod *obj = new Rod();
				obj->setup(number, type, RodPropList[TypeNum], endCoords,
				           NumSegs, outfiles.back(), outchannels);
				RodList.push_back(obj);

				// depending on type, assign the Rod to its respective parent body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
					GroundBody->addRodToBody(obj, endCoords);
				else if (!strcmp(let1, "PINNED") || !strcmp(let1, "PIN"))
					GroundBody->addRodToBody(obj, endCoords);
				else if (!strcmp(let1, "BODY"))
				{
					unsigned int bodyID = atoi(num1);
					BodyList[bodyID - 1]->addRodToBody(obj, endCoords);
				}
				Cout(MOORDYN_DBG_LEVEL) << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"POINTS", "POINT LIST", "CONNECTION PROPERTIES",
		                       "NODE PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading point list:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 9)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "9 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				int number=atoi(entries[0].c_str());
				double M = atof(entries[5].c_str());
				double V = atof(entries[6].c_str());
				double CdA;
				double Ca;
				double r0[3];
				double F[3] = {0.0, 0.0, 0.0};
				if (entries.size() >= 12) // case with optional force inputs (12 total entries)
				{
					for (int I=0; I<3; I++)
					{
						r0[I] = atof(entries[2+I].c_str());
						F[ I] = atof(entries[7+I].c_str());
					}
					CdA  = atof(entries[10].c_str());
					Ca   = atof(entries[11].c_str());
				}
				else // case without optional force inputs (9 total entries)
				{
					for (int I=0; I<3; I++)
						r0[I] = atof(entries[2+I].c_str());
				
					CdA  = atof(entries[7].c_str());
					Ca   = atof(entries[8].c_str());
				}

				Connection::types type;
				char let1[10], num1[10], let2[10], num2[10], let3[10]; 
				char typeWord[10];
                strncpy(typeWord, entries[1].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
				{
					// it is fixed  (this would just be used if someone wanted to temporarly fix a body that things were attached to)
					type = Connection::FIXED;
				}
				else if (!strcmp(let1, "BODY"))
				{
					type = Connection::FIXED;
					if (!strlen(num1))
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "no number provided for Rod " << number
							<< " Body attachment" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int bodyID = atoi(num1);
					if (!bodyID || (bodyID > BodyList.size()))
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "There is not " << bodyID << " bodies" << endl;
						return MOORDYN_INVALID_INPUT;
					}
				}
				else if (!strcmp(let1, "FAIRLEAD") || !strcmp(let1, "VESSEL") || !strcmp(let1, "VES") || !strcmp(let1, "COUPLED") || !strcmp(let1, "CPLD"))
				{
					// if a fairlead, add to list and add 
					type = Connection::COUPLED;
					CpldConIs.push_back(ConnectionList.size());
				}
				else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") || !strcmp(let1, "FREE"))
				{
					// if a connect, add to list and add states for it
					type = Connection::FREE;
					FreeConIs.push_back(ConnectionList.size());
					ConnectStateIs.push_back(nX);  // assign start index of this connect's states
					nX += 6;                         // add 6 state variables for each connect
				}
				else 
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Unrecognized connection type '" << let1 << "'"
						<< endl;
					return MOORDYN_INVALID_INPUT;
				}

				// make default water depth at least the depth of the lowest node (so water depth input is optional)
				if (r0[2] < -env.WtrDpth)
					env.WtrDpth = -r0[2];

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of type " << Connection::TypeName(type)
					<< " with id " << ConnectionList.size() << endl;

				// now make Connection object!
				Connection *obj = new Connection(this);
				obj->setup(number, type, r0, M, V, F, CdA, Ca);
				ConnectionList.push_back(obj);

				// depending on type, assign the Connection to its respective parent body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
					GroundBody->addConnectionToBody(obj, r0);
				else if (!strcmp(let1, "BODY"))
				{
					int bodyID = atoi(num1);
					BodyList[bodyID - 1]->addConnectionToBody(obj, r0);
				}
				Cout(MOORDYN_DBG_LEVEL) << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"LINES", "LINE LIST", "LINE PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading line list: " << endl;

			if (!LinePropList.size())
			{
				Cout(MOORDYN_ERR_LEVEL)
					<< "Reading lines without defined line types" << endl;
				return MOORDYN_INVALID_INPUT;
			}
			if (!ConnectionList.size())
			{
				Cout(MOORDYN_ERR_LEVEL)
					<< "Reading lines without defined connections" << endl;
				return MOORDYN_INVALID_INPUT;
			}

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 7)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
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
				for (unsigned int J = 0; J < LinePropList.size(); J++)  {
					if (LinePropList[J]->type == type)
						TypeNum = J;
				}
				if (TypeNum == -1)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Unrecognized line type " << type << endl;
					return MOORDYN_INVALID_INPUT;
				}

				// Make the output file (if queried)
				if ((outchannels.size() > 0) &&
					(strcspn(outchannels.c_str(), "pvUDctsd") < strlen(outchannels.c_str())))
				{
					// if 1+ output flag chars are given and they're valid
					stringstream oname;
					oname << _basepath << _basename << "_Line" << number << ".out";
					outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!outfiles.back()->is_open())
					{
						Cout(MOORDYN_ERR_LEVEL)
							<< "Cannot create the output file '"
							<< oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				}
				else
					outfiles.push_back(NULL);

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of class " << type << " (" << TypeNum << ")"
					<< " with id " << LineList.size() << endl;

				Line *obj = new Line();
				obj->setup(number, LinePropList[TypeNum], UnstrLen, NumSegs,
				           outfiles.back(), outchannels);
				LineList.push_back(obj);
				LineStateIs.push_back(nX);  // assign start index of this Line's states
				nX += 6 * (NumSegs - 1);   // add 6 state variables for each internal node of this line

				for (unsigned int I = 0; I < 2; I++)
				{
					char let1[10], num1[10], let2[10], num2[10], let3[10]; 
					char typeWord[10];
					strncpy(typeWord, entries[2 + I].c_str(), 9);
					typeWord[9] = '\0';
					// divided outWord into letters and numbers
					decomposeString(typeWord, let1, num1, let2, num2, let3);

					if (!strlen(num1))
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "No number provided for the 1st connection index"
							<< endl;
						return MOORDYN_INVALID_INPUT;
					}
					unsigned int id = atoi(num1);

					if (!strcmp(let1, "R") || !strcmp(let1, "ROD"))
					{
						if (!id || id > RodList.size())
						{
							Cout(MOORDYN_ERR_LEVEL) << "Error in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "There are not " << id << " rods" << endl;
							return MOORDYN_INVALID_INPUT;
						}
						if (!strcmp(let2, "A")) 
							RodList[id - 1]->addLineToRodEndA(obj, I);
						else if (!strcmp(let2, "B")) 
							RodList[id - 1]->addLineToRodEndB(obj, I);
						else
						{
							Cout(MOORDYN_ERR_LEVEL) << "Error in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "Rod end (A or B) must be specified" << endl;
							return MOORDYN_INVALID_INPUT;
						}
					}
					else if (!strlen(let1) || !strcmp(let1, "C") || !strcmp(let1, "CON"))
					{
						if (!id || id > ConnectionList.size())
						{
							Cout(MOORDYN_ERR_LEVEL) << "Error in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "There are not " << id << " connections" << endl;
							return MOORDYN_INVALID_INPUT;
						}
						ConnectionList[id - 1]->addLineToConnect(obj, I);
					}
					else 
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "Unrecognized connection type " << let1 << endl;
						return MOORDYN_INVALID_INPUT;
					}
				}
				Cout(MOORDYN_DBG_LEVEL) << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"FAILURE"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading failure conditions:" << endl;

			// skip following two lines (label line and unit line)
			i += 3;
			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 4)
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "4 fields are required, but just "
						<< entries.size() << " are provided" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				FailProps *obj = new FailProps();
				FailList.push_back(obj);

				char let1[10], num1[10], let2[10], num2[10], let3[10]; 
				char typeWord[10];
				strncpy(typeWord, entries[0].c_str(), 9);
				typeWord[9] = '\0';
				// divided outWord into letters and numbers
				decomposeString(typeWord, let1, num1, let2, num2, let3);

				if (!strlen(num1))
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "No number provided for Node Failure" << endl;
					return MOORDYN_INVALID_INPUT;
				}

				unsigned int id = atoi(num1);
				obj->attachID = id;
				if (!strcmp(let1, "R") || !strcmp(let1, "ROD"))
				{
					if (!id || id > RodList.size())
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "There are not " << id << " rods" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					if (!strcmp(let2, "A")) 
						obj->isRod = 1;
					else if (!strcmp(let2, "B")) 
						obj->isRod = 2;
					else
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "Failure end (A or B) must be specified" << endl;
						return MOORDYN_INVALID_INPUT;
					}
				}
				else if (!strlen(let1) || !strcmp(let1, "C") || !strcmp(let1, "CON"))
				{
					if (!id || id > ConnectionList.size())
					{
						Cout(MOORDYN_ERR_LEVEL) << "Error in "
							<< _filepath << ":" << i + 1 << "..." << endl
							<< "'" << in_txt[i] << "'" << endl
							<< "There are not " << id << " connections" << endl;
						return MOORDYN_INVALID_INPUT;
					}
					obj->isRod = 0;
				}
				else 
				{
					Cout(MOORDYN_ERR_LEVEL) << "Error in "
						<< _filepath << ":" << i + 1 << "..." << endl
						<< "'" << in_txt[i] << "'" << endl
						<< "Unrecognized connection type " << let1 << endl;
					return MOORDYN_INVALID_INPUT;
				}

				vector<string> lineNums = moordyn::str::split(entries[1], ',');
				obj->nLinesToDetach = lineNums.size();
				for (unsigned int il = 0; il < lineNums.size(); il++)
					obj->lineIDs[il] = atoi(lineNums[il].c_str());  

				obj->failTime = atof(entries[2].c_str());
				obj->failTen  = atof(entries[3].c_str());

				Cout(MOORDYN_DBG_LEVEL) << "failTime is "<< obj->failTime << endl;
				// initialize as unfailed, of course
				obj->failStatus  = 0;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"OPTIONS"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading options:" << endl;

			i++;
			// Parse options until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');
				if (entries.size() < 2)
				{
					i++;
					Cout(MOORDYN_WRN_LEVEL) << "Ignoring option line "
					                        << i << endl;
					continue;
				}

				Cout(MOORDYN_DBG_LEVEL) << "\t"
					<< entries[1] << " = " << entries[0] << endl;
				const string value = entries[0];
				const string name = entries[1];
				if (name == "writeLog")
				{
					env.writeLog = atoi(entries[0].c_str());
					const moordyn::error_id err = SetupLog();
					if (err != MOORDYN_SUCCESS)
						return err;
				}
				// DT is old way, should phase out
				else if ((name == "dtM") || (name == "DT"))
					dtM0 = atof(entries[0].c_str());
				else if ((name == "g") || (name == "gravity"))
					env.g  = atof(entries[0].c_str());
				else if ((name =="Rho") || (name=="rho") || (name=="WtrDnsty"))
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
					ICDfac   = atof(entries[0].c_str());
				else if ((name == "threshIC") || (name == "ICthresh"))
					ICthresh = atof(entries[0].c_str());
				else if (name == "WaveKin")
					WaveKinTemp = atoi(entries[0].c_str());
				else if (name == "Currents")
					env.Current = atoi(entries[0].c_str());
				else if (name == "WriteUnits")
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
				// >>>>>>>>>> add dtWave...
				else
					Cout(MOORDYN_WRN_LEVEL) << "Warning: Unrecognized option '"
					                        << name << "'" << endl;

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"OUTPUT"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "   Reading output options:" << endl;

			// parse until the next header or the end of the file
			while ((in_txt[i].find("---") == string::npos) && (i < in_txt.size()))
			{ 
				vector<string> entries = moordyn::str::split(in_txt[i], ' ');

				for (unsigned int j=0; j<entries.size(); j++)  //loop through each word on each line
				{
					char let1[10], num1[10], let2[10], num2[10], let3[10]; 
					char typeWord[10];
					typeWord[9] = '\0';
					strncpy(typeWord, entries[j].c_str(), 9);
					// divided outWord into letters and numbers
					decomposeString(typeWord, let1, num1, let2, num2, let3);

					// declare dummy struct to be copied onto end of vector (and filled in later);
					OutChanProps dummy;
					strncpy(dummy.Name, typeWord, 10);

					// figure out what type of output it is and process accordingly 
					// TODO: add checks of first char of num1,2, let1,2,3 not being NULL to below and handle errors (e.g. invalid line number)

					// The length of dummy.Units is hardcoded to 10 in
					// Misc.h
					const int UnitsSize = 9;
					dummy.Units[UnitsSize] = '\0';

					// fairlead tension case (changed to just be for single line, not all connected lines)
					if (!strcmp(let1, "FAIRTEN"))
					{
						dummy.OType = 1;
						dummy.QType = Ten;
						strncpy(dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						dummy.ObjID = atoi(num1);
						dummy.NodeID = LineList[dummy.ObjID-1]->getN();
					}
					// achor tension case (changed to just be for single line, not all connected lines)
					else if (!strcmp(let1, "ANCHTEN")) 
					{
						dummy.OType = 1;
						dummy.QType = Ten;
						strncpy(dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						dummy.ObjID = atoi(num1);
						dummy.NodeID = 0;
					}
					// more general case
					else
					{
						// get object type and node number if applicable
						// Line case:  L?N?xxxx
						if (!strcmp(let1, "L"))
						{
							dummy.OType = 1;
							dummy.NodeID = atoi(num2);
						}
						// Connect case:   C?xxx or Con?xxx
						else if(!strcmp(let1, "C") || !strcmp(let1, "CON"))
						{
							dummy.OType = 2;
							dummy.NodeID = -1;
							strncpy(let3, let2, 10);
						}
						// Rod case:   R?xxx or Rod?xxx
						else if(!strcmp(let1, "R") || !strcmp(let1, "ROD"))
						{
							dummy.OType = 3;
							dummy.NodeID = atoi(num2);
						}
						// should do fairlead option also!
						
						else
						{
							Cout(MOORDYN_WRN_LEVEL) << "Warning in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "invalid output specifier: " << let1
								<< ".  Type must be oneof L, C/Con or R/Rod"
								<< endl;
							dummy.OType = -1;
							continue;
						}

						// object number
						dummy.ObjID =  atoi(num1);

						if (!strcmp(let3, "PX")) {
							//cout << "SETTING QTYPE to " << PosX << endl;
							dummy.QType = PosX;
							strncpy(dummy.Units, moordyn::UnitList[PosX], UnitsSize);
						}
						else if (!strcmp(let3, "PY"))  {
							dummy.QType = PosY;
							strncpy(dummy.Units, moordyn::UnitList[PosY], UnitsSize);
						}
						else if (!strcmp(let3, "PZ"))  {
							dummy.QType = PosZ;
							strncpy(dummy.Units, moordyn::UnitList[PosZ], UnitsSize);
						}
						else if (!strcmp(let3, "VX"))  {
							dummy.QType = VelX;
							strncpy(dummy.Units, moordyn::UnitList[VelX], UnitsSize);
						}
						else if (!strcmp(let3, "VY"))  {
							dummy.QType = VelY;
							strncpy(dummy.Units, moordyn::UnitList[VelY], UnitsSize);
						}
						else if (!strcmp(let3, "VZ"))  {
							dummy.QType = VelZ;
							strncpy(dummy.Units, moordyn::UnitList[VelZ], UnitsSize);
						}
						else if (!strcmp(let3, "AX"))  {
							dummy.QType = AccX;
							strncpy(dummy.Units, moordyn::UnitList[AccX], UnitsSize);
						}
						else if (!strcmp(let3, "Ay"))  {
							dummy.QType = AccY;
							strncpy(dummy.Units, moordyn::UnitList[AccY], UnitsSize);
						}
						else if (!strcmp(let3, "AZ"))  {
							dummy.QType = AccZ;
							strncpy(dummy.Units, moordyn::UnitList[AccZ], UnitsSize);
						}
						else if (!strcmp(let3, "T") || !strcmp(let3, "TEN")) {
							dummy.QType = Ten;
							strncpy(dummy.Units, moordyn::UnitList[Ten], UnitsSize);
						}
						else if (!strcmp(let3, "FX"))  {
							dummy.QType = FX;
							strncpy(dummy.Units, moordyn::UnitList[FX], UnitsSize);
						}
						else if (!strcmp(let3, "FY"))  {
							dummy.QType = FY;
							strncpy(dummy.Units, moordyn::UnitList[FY], UnitsSize);
						}
						else if (!strcmp(let3, "FZ"))  {
							dummy.QType = FZ;
							strncpy(dummy.Units, moordyn::UnitList[FZ], UnitsSize);
						}
						else
						{
							Cout(MOORDYN_WRN_LEVEL) << "Warning in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "invalid quantity specifier: " << let3 << endl;
							dummy.OType = -1;
							continue;
						}

					}

					// some name adjusting for special cases (maybe should
					// handle this elsewhere...)
					if ((dummy.OType==3) && (dummy.QType==Ten))
					{
						if (dummy.NodeID > 0)
							strncpy(dummy.Name,"TenEndB", 10);
						else
							strncpy(dummy.Name,"TenEndA", 10);
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

	Cout(MOORDYN_MSG_LEVEL) << "Generated entities:" << endl
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
	
	// write system description to log file
	if (env.writeLog > 0)
	{
		outfileLog << "----- MoorDyn Model Summary (to be written) -----"
		          << endl;
	}

	// Setup the waves and populate them
	waves = new Waves();
	waves->setup(&env);

	GroundBody->setEnv( &env, waves);
	for (auto obj : BodyList)
		obj->setEnv( &env, waves);
	for (auto obj : RodList)
		obj->setEnv( &env, waves);
	for (auto obj : ConnectionList)
		obj->setEnv( &env, waves);
	for (auto obj : LineList)
		obj->setEnv( &env, waves);

	return MOORDYN_SUCCESS;
}

moordyn::error_id moordyn::MoorDyn::CalcStateDeriv(double *x,  double *xd,
                                                   const double t,
                                                   const double dt)
{
	// call ground body to update all the fixed things...
	GroundBody->updateFairlead(t);

	// couple things...

	// extrapolate instantaneous positions of any coupled bodies (type -1)
	for (auto l : CpldBodyIs)
		BodyList[l]->updateFairlead(t);

	// extrapolate instantaneous positions of any coupled or fixed rods (type -1 or -2)
	for (auto l : CpldRodIs)
		RodList[l]->updateFairlead(t);

	// extrapolate instantaneous positions of any coupled or fixed connections (type -1)
	for (auto l : CpldConIs)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			ConnectionList[l]->updateFairlead(t);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error updating connection: "
				<< err_msg << endl;
			return err;
		}
	}

	// update wave kinematics if applicable
	if (env.WaveKin == 1)
	{
		// extrapolate velocities from accelerations
		// (in future could extrapolote from most recent two points,
		// (U_1 and U_2)

		double t_delta = t - tW_1;

		vector<double> U_extrap;
		for (unsigned int i = 0; i < npW * 3; i++)
			U_extrap.push_back(U_1[i] + Ud_1[i] * t_delta);

		// distribute to the appropriate objects
		unsigned int i = 0;
		for (auto line : LineList)
		{
			line->setNodeWaveKin(U_extrap.data() + 3 * i, Ud_1 + 3 * i);
			i += 3 * line->getN() + 3;
		}
	}

	// independent or semi-independent things with their own states...

	// give Bodies latest state variables (kinematics will also be assigned to dependent connections and rods, and thus line ends)
	for (unsigned int l = 0; l < FreeBodyIs.size(); l++)
		BodyList[FreeBodyIs[l]]->setState((x + BodyStateIs[l]), t);

	// give independent or pinned rods' latest state variables (kinematics will also be assigned to attached line ends)
	for (unsigned int l = 0; l < FreeRodIs.size(); l++)
		RodList[FreeRodIs[l]]->setState((x + RodStateIs[l]), t);

	// give Connects (independent connections) latest state variable values (kinematics will also be assigned to attached line ends)
	for (unsigned int l = 0; l < FreeConIs.size(); l++)
		ConnectionList[FreeConIs[l]]->setState((x + ConnectStateIs[l]), t);

	// give Lines latest state variable values for internal nodes
	for (unsigned int l = 0; l < LineList.size(); l++)
		LineList[l]->setState((x + LineStateIs[l]), t);

	// calculate dynamics of free objects (will also calculate forces (doRHS())
	// from any child/dependent objects)...

	// calculate line dynamics (and calculate line forces and masses attributed to connections)
	for (unsigned int l = 0; l < LineList.size(); l++)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			LineList[l]->getStateDeriv((xd + LineStateIs[l]), dt);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Exception detected at " << t << " s: "
			                        << err_msg << endl;
			return err;
		}
	}

	// calculate connect dynamics (including contributions from attached lines
	// as well as hydrodynamic forces etc. on connect object itself if applicable)
	for (unsigned int l = 0; l < FreeConIs.size(); l++)
		ConnectionList[FreeConIs[l]]->getStateDeriv((xd + ConnectStateIs[l]));

	// calculate dynamics of independent Rods 
	for (unsigned int l = 0; l < FreeRodIs.size(); l++)
		RodList[FreeRodIs[l]]->getStateDeriv((xd + RodStateIs[l]));

	// calculate dynamics of Bodies
	for (unsigned int l = 0; l < FreeBodyIs.size(); l++)
		BodyList[FreeBodyIs[l]]->getStateDeriv((xd + BodyStateIs[l]));

	// get dynamics/forces (doRHS()) of coupled objects, which weren't addressed in above calls

	for (unsigned int l = 0; l < CpldConIs.size(); l++)
		ConnectionList[CpldConIs[l]]->doRHS();

	for (unsigned int l = 0; l < CpldRodIs.size(); l++)
		RodList[CpldRodIs[l]]->doRHS();

	for (unsigned int l = 0; l < CpldBodyIs.size(); l++)
		BodyList[CpldBodyIs[l]]->doRHS();

	// call ground body to update all the fixed things
	// GroundBody->doRHS();
	GroundBody->setDependentStates();  // (not likely needed) <<<

	return MOORDYN_SUCCESS;
}

moordyn::error_id moordyn::MoorDyn::RK2(double *x, double &t, const double dt)
{
	moordyn::error_id err;

	if (env.writeLog > 2)
		outfileLog << "\n----- RK2 predictor call to CalcStateDeriv at time "
		          << t << " s -----\n";

	// get derivatives at t0. f0 = f(t0, x0);
	err = CalcStateDeriv(x, f0, t, dt);
	if (err != MOORDYN_SUCCESS)
		return err;

	// integrate to t0 + dt/2. x1 = x0 + dt*f0/2.0;
	for (unsigned int i = 0; i < nX; i++)
		xt[i] = x[i] + 0.5 * dt * f0[i];

	if (env.writeLog > 2)
		outfileLog << "\n----- RK2 corrector call to CalcStateDeriv at time "
		          << t + 0.5 * dt << " s -----\n";

	// get derivatives at t0 + dt/2. f1 = f(t1, x1);
	err = CalcStateDeriv(xt, f1, t + 0.5 * dt, dt);
	if (err != MOORDYN_SUCCESS)
		return err;

	// integrate states to t0 + dt
	for (unsigned int i = 0; i < nX; i++)
		x[i] = x[i] + dt * f1[i];

	// update time
	t = t + dt;

	// <<<<<<<<< maybe should check/force all rod unit vectors to be unit
	// vectors here?

	return MOORDYN_SUCCESS;
}

moordyn::error_id moordyn::MoorDyn::detachLines(int attachID, int isRod,
	const int* lineIDs, int* lineTops, int nLinesToDetach, double time)
{
	// create new massless connection for detached end(s) of line(s)
	double M = 0.0;
	double V = 0.0;
	double r0[3] = {0.0, 0.0, 0.0};
	double F[3] = {0.0, 0.0, 0.0};
	double CdA  = 0.0;
	double Ca   = 0.0;
	Connection::types type = Connection::FREE;

	nX += 6;  // add 6 state variables for each connect

	// check to make sure we haven't gone beyond the extra size allotted to the
	// state arrays or the connections list <<<< really should throw an error
	// here
	if (nX > nXtra)
	{
		Cout(MOORDYN_ERR_LEVEL) << "Error: nX = " << nX
			<< " is bigger than nXtra = " << nXtra << endl;
		return MOORDYN_MEM_ERROR;
	}

	// add connect to list of free ones and add states for it
	FreeConIs.push_back(ConnectionList.size());
	// assign start index of this connect's states
	ConnectStateIs.push_back(nX);

	// now make Connection object!
	Connection *obj = new Connection(this);
	obj->setup(ConnectionList.size() + 1, type, r0, M, V, F, CdA, Ca);
	obj->setEnv(&env, waves);
	ConnectionList.push_back(obj);

	// dummy state array to hold kinematics of old attachment point (format in
	// terms of part of connection state vector:
	// r[J]  = X[3 + J]; rd[J] = X[J]; )
	double dummyConnectState[6];

	// detach lines from old Rod or Connection, and get kinematics of the old attachment point
	for (int l = 0; l < nLinesToDetach; l++)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		if (isRod == 1)
			RodList[attachID-1]->removeLineFromRodEndA(lineIDs[l],
			                                         lineTops + l,
			                                         dummyConnectState + 3,
			                                         dummyConnectState);
		else if (isRod == 2)
			RodList[attachID-1]->removeLineFromRodEndB(lineIDs[l],
			                                         lineTops + l,
			                                         dummyConnectState + 3,
			                                         dummyConnectState);
		else if (isRod == 0)
		{
			try
			{
				ConnectionList[attachID-1]->removeLineFromConnect(
					lineIDs[l], lineTops + l,
					dummyConnectState + 3, dummyConnectState);
			}
			MOORDYN_CATCHER(err, err_msg);
		}
		else
		{
			err_msg = "Error: Failure does not have a valid isRod value";
			err = MOORDYN_INVALID_VALUE;
		}

		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error detaching line: "
				<< err_msg << endl;
			return err;
		}
	}

	// attach lines to new connection
	for (int l = 0; l < nLinesToDetach; l++)
		obj->addLineToConnect(LineList[lineIDs[l] - 1], lineTops[l]);

	// update connection kinematics to match old line attachment point
	// kinematics and set positions of attached line ends
	obj->setState(dummyConnectState, time);

	// now make the state vector up to date!
	for (unsigned int J = 0; J < 6; J++)
		states[ConnectStateIs.back() + J] = dummyConnectState[J];

	return MOORDYN_SUCCESS;
}

moordyn::error_id moordyn::MoorDyn::AllOutput(double t, double dt)
{
	if (dtOut > 0)
		if (t < (floor((t - dt) / dtOut) + 1.0) * dtOut)
			return MOORDYN_SUCCESS;
	
	// write to master output file
	if (!outfileMain.is_open())
	{
		Cout(MOORDYN_ERR_LEVEL) << "Error: Unable to write to main output file "
		                        << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}
	outfileMain << t << "\t "; 		// output time
	for (auto channel : outChans)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			outfileMain << GetOutput(channel) << "\t ";
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error handling an output channel:"
				<< err_msg << endl;
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

}  // ::moordyn


// =============================================================================
//
//                     ||                     ||
//                     ||        C API        ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

MoorDyn DECLDIR MoorDyn_Create(const char *infilename)
{
	// ---------------------------- MoorDyn title message ----------------------------

	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	moordyn::MoorDyn *instance = NULL;
	try
	{
		instance = new moordyn::MoorDyn(infilename);
	}
	MOORDYN_CATCHER(err, err_msg);

	if (err != MOORDYN_SUCCESS)
	{
		cerr << "Error (" << err << ") during the Mooring System creation:"
		      << endl << err_msg << endl;
	}
	return (void*)instance;

}

/// Check that the provided system is not Null
#define CHECK_SYSTEM(s)                                                         \
	if (!s)                                                                     \
	{                                                                           \
		cerr << "Null system received in " << __FUNC_NAME__                     \
		     << " (" << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;       \
		return MOORDYN_INVALID_VALUE;                                           \
	}

int DECLDIR MoorDyn_SetVerbosity(MoorDyn system, int verbosity)
{
	CHECK_SYSTEM(system);
	((moordyn::MoorDyn*)system)->SetVerbosity(verbosity);
	return MOORDYN_SUCCESS;
}

unsigned int DECLDIR MoorDyn_NCoupledDOF(MoorDyn system)
{
	if (!system)
		return 0;
	return ((moordyn::MoorDyn*)system)->NCoupedDOF();
}

int DECLDIR MoorDyn_Init(MoorDyn system, const double *x, const double *xd)
{
	CHECK_SYSTEM(system);
	return ((moordyn::MoorDyn*)system)->Init(x, xd);
}

int DECLDIR MoorDyn_Step(MoorDyn system, const double *x, const double *xd,
                         double *f, double *t, double *dt)
{
	CHECK_SYSTEM(system);
	return ((moordyn::MoorDyn*)system)->Step(x, xd, f, *t, *dt);
}

int DECLDIR MoorDyn_Close(MoorDyn system)
{
	CHECK_SYSTEM(system);
	delete((moordyn::MoorDyn*)system);
	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_InitExtWaves(MoorDyn system, unsigned int *n)
{
	CHECK_SYSTEM(system);

	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try
	{
		*n = ((moordyn::MoorDyn*)system)->ExternalWaveKinInit();
	}
	MOORDYN_CATCHER(err, err_msg);
	if (err != MOORDYN_SUCCESS)
	{
		cerr << "Error (" << err << ") at " << __FUNC_NAME__ << "():"
		      << endl << err_msg << endl;
	}
	return err;
}

int DECLDIR MoorDyn_GetWavesCoords(MoorDyn system, double *r)
{
	CHECK_SYSTEM(system);

	return ((moordyn::MoorDyn*)system)->GetWaveKinCoordinates(r);
}

int DECLDIR MoorDyn_SetWaves(MoorDyn system, const double *U,
                                             const double *Ud,
                                             double t)
{
	CHECK_SYSTEM(system);

	return ((moordyn::MoorDyn*)system)->SetWaveKin(U, Ud, t);
}

unsigned int DECLDIR MoorDyn_GetNumberBodies(MoorDyn system)
{
	if (!system)
		return 0;
	return ((moordyn::MoorDyn*)system)->GetBodies().size();
}

unsigned int DECLDIR MoorDyn_GetNumberRods(MoorDyn system)
{
	if (!system)
		return 0;
	return ((moordyn::MoorDyn*)system)->GetRods().size();
}

unsigned int DECLDIR MoorDyn_GetNumberConnections(MoorDyn system)
{
	if (!system)
		return 0;
	return ((moordyn::MoorDyn*)system)->GetConnections().size();
}

unsigned int DECLDIR MoorDyn_GetNumberLines(MoorDyn system)
{
	if (!system)
		return 0;
	return ((moordyn::MoorDyn*)system)->GetLines().size();
}

unsigned int DECLDIR MoorDyn_GetNumberLineNodes(MoorDyn system,
                                                unsigned int line)
{
	if (!system)
		return 0;

	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if (!line || (line > lines.size()))
	{
		cerr << "Error: There is not such line " << line << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return 0;
	}

	return lines[line - 1]->getN() + 1;
}

double DECLDIR MoorDyn_GetFairTen(MoorDyn system, unsigned int line)
{
	CHECK_SYSTEM(system);

	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if (!line || (line > lines.size()))
	{
		cerr << "Error: There is not such line " << line << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return -1;
	}

	return lines[line - 1]->getNodeTen(lines[line - 1]->getN());
}

int DECLDIR MoorDyn_GetFASTtens(MoorDyn system, const int* numLines,
                                float FairHTen[], float FairVTen[],
                                float AnchHTen[], float AnchVTen[])
{
	CHECK_SYSTEM(system);

	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if ((unsigned int)(*numLines) > lines.size())
	{
		cerr << "Error: There is not " << *numLines << " lines" << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return MOORDYN_INVALID_VALUE;
	}

	for (int l = 0; l < *numLines; l++)
		lines[l]->getFASTtens(FairHTen + l, FairVTen + l,
		                      AnchHTen + l, AnchVTen + l);

	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_GetConnectPos(MoorDyn system, unsigned int l, double pos[3])
{
	CHECK_SYSTEM(system);

	auto conns = ((moordyn::MoorDyn*)system)->GetConnections();
	if (!l || (l > conns.size()))
	{
		cerr << "Error: There is not such connection " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return MOORDYN_INVALID_VALUE;
	}
	vector<double> rs(3);
	vector<double> rds(3);
	conns[l - 1]->getConnectState(rs, rds);
	for (unsigned int i = 0; i < 3; i++)
		pos[i] = rs[i];
	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_GetConnectForce(MoorDyn system, unsigned int l, double f[3])
{
	CHECK_SYSTEM(system);

	auto conns = ((moordyn::MoorDyn*)system)->GetConnections();
	if (!l || (l > conns.size()))
	{
		cerr << "Error: There is not such connection " << l << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return MOORDYN_INVALID_VALUE;
	}
	conns[l - 1]->getFnet(f);
	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_GetNodePos(MoorDyn system,
                               unsigned int LineNum,
                               unsigned int NodeNum,
                               double pos[3])
{
	CHECK_SYSTEM(system);

	auto lines = ((moordyn::MoorDyn*)system)->GetLines();
	if (!LineNum || (LineNum > lines.size()))
	{
		cerr << "Error: There is not such line " << LineNum << endl
		     << "while calling " << __FUNC_NAME__ << "()" << endl;
		return -1;
	}
	if (lines[LineNum - 1]->getNodePos(NodeNum, pos))
		return MOORDYN_INVALID_VALUE;
	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_DrawWithGL(MoorDyn system)
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
