/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
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

#include "MoorDynAPI.h"
#include "Log.hpp"
#include "Misc.h"

#include "Waves.h"
#include "MoorDyn.h"
#include "Line.h"
#include "Connection.h" 
#include "Rod.h" 
#include "Body.h"

/// MoorDyn2 C++ API namespace
namespace moordyn
{

/** @class MoorDyn
 * @brief A Mooring system
 *
 * This class contains everything required to hold a whole mooring system,
 * making everything thread-friendly easier
 */
class MoorDyn : public Log
{
public:
	/** @brief Constructor
	 *
	 * Remember to call Init() to initialize the mooring system
	 *
	 * @param infilename The input file, if either NULL or "", then
	 * "Mooring/lines.txt" will be considered
	 * @param verbosity The verbosity level (see @ref moordyn_log)
	 */
	MoorDyn(const char *infilename=NULL,
            const int verbosity=MOORDYN_DBG_LEVEL);

	/** @brief Destuctor
	 */    
	~MoorDyn();

	/** @brief Initializes Moordyn, reading the input file and setting up the
	 * mooring lines
	 * @param x Position vector
	 * @param xd Velocity vector
	 * @note You can know the number of components required for \p x and \p xd
	 * with the function MoorDyn::NCoupedDOF()
	 * @return MOORDYN_SUCCESS If the mooring system is correctly initialized,
	 * an error code otherwise (see @ref moordyn_errors)
	 */
	moordyn::error_id Init(const double *x, const double *xd);

	/** @brief Runs a time step of the MoorDyn system
	 * @param x Position vector
	 * @param xd Velocity vector
	 * @param f Output forces
	 * @return MOORDYN_SUCCESS If the mooring system is correctly evolved,
	 * an error code otherwise (see @ref moordyn_errors)
	 * @note You can know the number of components required for \p x, \p xd and
	 * \p f with the function MoorDyn::NCoupedDOF()
	 */
	moordyn::error_id Step(const double *x, const double *xd, double *f,
	                       double &t, double &dt);

	/** @brief Get the connections
	 */
	inline vector<Body*> GetBodies() const { return BodyList; }

	/** @brief Get the connections
	 */
	inline vector<Rod*> GetRods() const { return RodList; }

	/** @brief Get the connections
	 */
	inline vector<Connection*> GetConnections() const { return ConnectionList; }

	/** @brief Get the lines
	 */
	inline vector<Line*> GetLines() const { return LineList; }

	/** @brief Return the number of coupled Degrees Of Freedom (DOF)
	 *
	 * This should match with the number of components of the positions and
	 * velocities passed to MoorDyn::Init() and MoorDyn::Step()
	 *
	 * The number of coupled DOF is computed as
	 *
	 * \f$n_{dof} = 6 * n_{body} + 3 * n_{conn} + 6 * n_{rod} + 3 * n_{pinn}\f$
	 *
	 * with \f$n_{body}\f$ the number of coupled bodies, \f$n_{conn}\f$ the
	 * number of coupled connections, \f$n_{rod}\f$ the number of cantilevered
	 * coupled rods, and \f$n_{pinn}\f$ the number of pinned coupled rods
	 *
	 * @return The number of coupled DOF
	 */
	inline unsigned int NCoupedDOF() const
	{
		unsigned int n = 6 * CpldBodyIs.size() +
		                 3 * CpldConIs.size();
		for (auto rodi : CpldRodIs)
		{
			if (RodList[rodi]->type == -2)
				n += 6;  // cantilevered rods
			else
				n += 3;  // pinned rods
		}
		return n;
	}

	/** @brief Initializes the external Wave kinetics
	 *
	 * This is only used if env.WaveKin > 0
	 * @return The number of points where the wave kinematics shall be provided
	 */
	inline unsigned int ExternalWaveKinInit()
	{
		npW = 0;
		
		for (auto line : LineList)
			npW += line->getN() + 1;

		// allocate arrays to hold data that could be passed in
		U_1      = make1Darray(3 * npW);
		Ud_1     = make1Darray(3 * npW);
		U_2      = make1Darray(3 * npW);
		Ud_2     = make1Darray(3 * npW);

		// initialize with zeros for safety
		tW_1 = 0.0;
		tW_2 = 0.0;
		memset(U_1, 0.0, 3 * npW * sizeof(double));
		memset(Ud_1, 0.0, 3 * npW * sizeof(double));
		memset(U_2, 0.0, 3 * npW * sizeof(double));
		memset(Ud_2, 0.0, 3 * npW * sizeof(double));

		return npW;
	}

	/** @brief Get the points where the waves kinematics shall be provided
	 * @param r The output coordinates
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise  (see @ref moordyn_errors)
	 * @see MoorDyn::ExternalWaveKinInit()
	 */
	inline moordyn::error_id GetWaveKinCoordinates(double *r) const
	{
		unsigned int i = 0;
		for (auto line : LineList)
		{
			line->getNodeCoordinates(r + 3 * i);
			i += line->getN() + 1;
		}
		return MOORDYN_SUCCESS;
	}

	/** @brief Set the kinematics of the waves
	*
	* Use this function if WaveKin option is set in the input file
	* @param U The velocities at the points (3 components per point)
	* @param Ud The accelerations at the points (3 components per point)
	* @return MOORDYN_SUCCESS If the data is correctly set, an error code
	* otherwise (see @ref moordyn_errors)
	* @see MoorDyn_InitExtWaves()
	* @see MoorDyn_GetWavesCoords()
	*/
	inline moordyn::error_id SetWaveKin(const double *U,
                                        const double *Ud,
                                        double t)
	{
		if (!U || !Ud)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: Received a Null pointer in "
				<< "MoorDyn::SetWaveKin()" << endl
				<< "Both velocity and acceleration arrays are needed"
				<< endl;
			return MOORDYN_INVALID_VALUE;
		}

		if (!U_1 || !U_2 || !Ud_1 || !Ud_2)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: Memory not allocated."
				<< " Have you called MoorDyn::ExternalWaveKinInit()?"
				<< endl;
			return MOORDYN_MEM_ERROR;
		}

		// set time stamp
		tW_2 = tW_1;
		tW_1 = t;

		for (unsigned int i = 0; i < 3 * npW; i++)
		{
			U_2[i] = U_1[i];
			Ud_2[i] = Ud_1[i];
			U_1[i] = U[i];
			Ud_1[i] = Ud[i];
		}
		return MOORDYN_SUCCESS;
	}

protected:
	/** @brief Read the input file, setting up all the requird objects and their
	 * relationships
	 *
	 * This function is called from the constructor, so this information is
	 * ready when MoorDyn::Init() is called
	 *
	 * @return MOORDYN_SUCCESS If the input file is correctly loaded and all
	 * the objects are consistently set, an error code otherwise
	 * (see @ref moordyn_errors)
	 */
	moordyn::error_id ReadInFile();

	/** @brief Get the forces
	 * @param f The forces array
	 * @return MOORDYN_SUCCESS If the forces are correctly set, an error code
	 * otherwise (see @ref moordyn_errors)
	 * @note You can know the number of components required for \p f with the
	 * function MoorDyn::NCoupedDOF()
	 */
	inline moordyn::error_id GetForces(double *f) const
	{
		if (!NCoupedDOF())
		{
			if (f)
				Cout(MOORDYN_WRN_LEVEL) << "Warning: Forces have been asked on "
					<< "the coupled entities, but there are no such entities"
					<< endl;
			return MOORDYN_SUCCESS;
		}
		if (NCoupedDOF() && !f)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: " << __PRETTY_FUNC_NAME__
				<< " called with a NULL forces pointer, but there are "
				<< NCoupedDOF() << " coupled Degrees Of Freedom" << endl;
			return MOORDYN_INVALID_VALUE;
		}
		unsigned int ix = 0;
		for (auto l : CpldBodyIs)
		{
			BodyList[l]->getFnet(f + ix);
			ix += 6;
		}
		for (auto l : CpldRodIs)
		{
			RodList[l]->getFnet(f + ix);
			if (RodList[l]->type == -2)
				ix += 6;  // for cantilevered rods 6 entries will be taken
			else
				ix += 3;  // for pinned rods 3 entries will be taken
		}
		for (auto l : CpldConIs)
		{
			ConnectionList[l]->getFnet(f + ix);
			ix += 3;
		}
		return MOORDYN_SUCCESS;
	}

private:
	/// The input file
	string _filepath;
	/// The input file basename
	string _basename;
	/// The input file directory
	string _basepath;

	// factor by which to boost drag coefficients during dynamic relaxation IC generation
	double ICDfac;
	// convergence analysis time step for IC generation
	double ICdt;
	// max time for IC generation
	double ICTmax;
	// threshold for relative change in tensions to call it converged
	double ICthresh;
	// temporary wave kinematics flag used to store input value while keeping env.WaveKin=0 for IC gen
	int WaveKinTemp;
	/// (s) desired mooring line model time step
	double dtM0;
	/// (s) desired output interval (the default zero value provides output at
	/// every call to MoorDyn)
	double dtOut;

	/// General options of the Mooryng system
	EnvCond env;
	/// The ground body, which is unique
	Body* GroundBody;
	/// Waves object that will be created to hold water kinematics info
	Waves *waves = NULL;

	/// array of pointers to hold line library types
	vector<LineProps*> LinePropList;
	/// array of pointers to hold rod library types
	vector<RodProps*> RodPropList;
	/// array of pointers to hold failure condition structs
	vector<FailProps*> FailList;
	/// array of pointers to connection objects (line joints or ends)
	vector<Body*> BodyList;
	/// array of pointers to Rod objects
	vector<Rod*> RodList;
	/// array of pointers to connection objects (line joints or ends)
	vector<Connection*> ConnectionList;
	/// array of pointers to line objects
	vector<Line*> LineList;

	/// array of starting indices for Lines in "states" array
	vector<int> LineStateIs;
	/// array of starting indices for indendent Connections in "states" array
	vector<int> ConnectStateIs;
	/// array of starting indices for independent Rods in "states" array
	vector<int> RodStateIs;
	/// array of starting indices for Bodies in "states" array
	vector<int> BodyStateIs;

	/// vector of free body indices in BodyList vector
	vector<int> FreeBodyIs;
	/// vector of coupled/fairlead body indices in BodyList vector
	vector<int> CpldBodyIs;

	/// vector of free rod indices in RodList vector (this includes pinned rods
	/// because they are partially free and have states)
	vector<int> FreeRodIs;
	/// vector of coupled/fairlead rod indices in RodList vector
	vector<int> CpldRodIs;

	/// vector of free connection indices in ConnectionList vector
	vector<int> FreeConIs;
	/// vector of coupled/fairlead connection indices in ConnectionList vector
	vector<int> CpldConIs;

	/// Number of used state vector components
	unsigned int nX;
	/// full size of state vector array including extra space for detaching up
	/// to all line ends, each which could get its own 6-state connect
	/// (nXtra = nX + 6 * 2 * LineList.size())
	unsigned int nXtra;

	/// Global state vector
	double* states;
	/// State vector at midpoint in the RK-2 integration scheme 
	double* xt;
	/// Drivatives computed in the first step of the RK-2 integration scheme 
	double* f0;
	/// Drivatives computed in the second step of the RK-2 integration scheme 
	double* f1;

	/// number of points that wave kinematics are input at
	/// (if using env.WaveKin=1)
	unsigned int npW;
	/// time corresponding to the wave kinematics data
	double tW_1;
	/// array of wave velocity at each of the npW points at time tW_1
	double* U_1;
	/// array of wave acceleration at each of the npW points
	double* Ud_1;
	/// time corresponding to the wave kinematics data
	double tW_2;
	/// array of wave velocity at each of the npW points at time tW_2
	double* U_2;
	/// array of wave acceleration at each of the npW points
	double* Ud_2;

	/// log output file
	ofstream outfileLog;

	/// main output file
	ofstream outfileMain;

	/// a vector to hold ofstreams for each body, line or rod
	vector<shared_ptr<ofstream>> outfiles;

	/// list of structs describing selected output channels for main out file
	vector<OutChanProps> outChans;

	/** @brief Create the log file if queried, close it otherwise
	 *
	 * Depending on the value of env.writeLog value, a Log file stream will
	 * be open or closed
	 */
	inline moordyn::error_id SetupLog()
	{
		if (env.writeLog > 0)
		{
			stringstream oname;
			oname << _basepath << _basename << ".log";
			Cout(MOORDYN_DBG_LEVEL) << "Creating a log file: '"
				<< oname.str() << "'" << endl;
			outfileLog.open(oname.str());
			if (!outfileLog.is_open())
			{
				Cout(MOORDYN_ERR_LEVEL) << "Unable to create the log file '"
				                        << oname.str() << "'" << endl;
				return MOORDYN_INVALID_OUTPUT_FILE;
			}
			outfileLog << "MoorDyn v2 log file with output level "
			          << env.writeLog << endl
			          << "Note: options above the writeLog line in the input "
					  << "file will not be recorded" << endl;
			// get pointer to outfile for MD objects to use
			env.outfileLogPtr = & outfileLog;
			return MOORDYN_SUCCESS;
		}

		if (outfileLog.is_open())
		{
			env.outfileLogPtr = NULL;
			outfileLog.close();
		}

		return MOORDYN_SUCCESS;
	}

	/** @brief Helper to read a nonlinear curve data from text
	 *
	 * This function can also read single values
	 *
	 * @param entry Either the value or the file where the curve can be read
	 *              from
	 * @param x Array of x values, 0 for single values
	 * @param y Array of y values, or the single value
	 * @return MOORDYN_SUCCESS if the value/curve is correctly read, an error
	 * code otherwise
	 */
	moordyn::error_id read_curve(const char *entry,
	                             vector<double> &x,
	                             vector<double> &y)
	{
		if (strpbrk(
				entry,
				"abcdfghijklmnopqrstuvwxyzABCDFGHIJKLMNOPQRSTUVWXYZ") == NULL)
		{
			// "eE" are exluded as they're used for scientific notation!
			x.push_back(0.0);
			y.push_back(atof(entry));
			return MOORDYN_SUCCESS;
		}

		string fpath = _basepath + entry;
		Cout(MOORDYN_MSG_LEVEL) << "Loading a curve from '"
		                        << fpath << "'..." << endl;
		ifstream f(fpath);
		if (!f.is_open())
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: Cannot read the file" << endl;
			return MOORDYN_INVALID_INPUT_FILE;
		}

		vector<string> flines;
		while (f.good())
		{
			string fline;
			getline (f, fline);
			flines.push_back(fline);
		}
		f.close();

		for (auto fline : flines)
		{
			vector<string> entries = moordyn::str::split(fline, ' ');
			if (entries.size() < 2)
			{
				Cout(MOORDYN_ERR_LEVEL) << "Error: Bad curve point" << endl
					<< "\t'" << fline << "'" << endl
					<< "\t2 fields required, but just " << entries.size()
					<< " are provided" << endl;
				return MOORDYN_INVALID_INPUT;
			}
			x.push_back(atof(entries[0].c_str()));
			y.push_back(atof(entries[0].c_str()));
			Cout(MOORDYN_DBG_LEVEL) << "(" << x.back() << ", "
				<< y.back() << ")" << endl;
		}

		Cout(MOORDYN_MSG_LEVEL) << "OK" << endl;
		return MOORDYN_SUCCESS;
	}

	/** @brief Helper to read a nonlinear curve data from text
	 *
	 * This function can also read single values. This function is prepared
	 * for working with the old API
	 *
	 * @param entry Either the value or the file where the curve can be read
	 *              from
	 * @param c The value, if it is just a coefficient
	 * @param n Number of points in the curves
	 * @param x Array of x values
	 * @param y Array of y values
	 * @return MOORDYN_SUCCESS if the value/curve is correctly read, an error
	 * code otherwise
	 */
	moordyn::error_id read_curve(const char *entry,
	                             double *c,
	                             int *n,
	                             double *x,
	                             double *y)
	{
		vector<double> xv, yv;
		const moordyn::error_id error = read_curve(entry, xv, yv);
		if (error != MOORDYN_SUCCESS)
			return error;

		if (xv.size() == 1)
		{
			*c = yv.back();
			n = 0;
			return MOORDYN_SUCCESS;
		}

		if (xv.size() > nCoef)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: Too much points in the curve"
				<< endl << "\t" << xv.size() << " points given, but just "
				<< nCoef << " are accepted" << endl;
			return MOORDYN_INVALID_INPUT;
		}

		*c = 0.0;
		*n = xv.size();
		memcpy(x, xv.data(), xv.size() * sizeof(double));
		memcpy(y, yv.data(), yv.size() * sizeof(double));

		return MOORDYN_SUCCESS;
	}

	/** @brief Get the value of a specific output channel
	 *
	 * This function might trhow moordyn::invalid_value_error exceptions
	 * @param channel Output channel
	 * @return The corresponding value
	 */
	inline double GetOutput(const OutChanProps channel) const
	{
		if (channel.OType == 1)
			return LineList[channel.ObjID - 1]->GetLineOutput(channel);
		else if (channel.OType == 2)
			return ConnectionList[channel.ObjID - 1]->GetConnectionOutput(channel);
		else if (channel.OType == 3)
			return RodList[channel.ObjID - 1]->GetRodOutput(channel);
		stringstream s;
		s << "Error: output type of " << channel.Name
			<< " does not match a supported object type";
		MOORDYN_THROW(MOORDYN_INVALID_VALUE, s.str().c_str());
		return 0.0;
	}

	/** @brief Compute the state derivatives
	 * @param x Array of states (input)
	 * @param xd Array of state derivatives (output)
	 * @param t The time instant
	 * @param dt Desired time step
	 * @return MOORDYN_SUCCESS if the system was correctly integrated, an error
	 * code otherwise
	 */
	moordyn::error_id CalcStateDeriv(double *x,  double *xd,
                                     const double t, const double dt);

	/** @brief Carry out an integration step
	 *
	 * 2nd order Runge-Kutta time integrator is considered
	 * @param x Array of states which will be integrated
	 * @param t The time instant. It will be modified
	 * @param dt Desired time step
	 * @return MOORDYN_SUCCESS if the system was correctly integrated, an error
	 * code otherwise
	 */
	moordyn::error_id RK2(double *x, double &t, const double dt);

	/** @brief Detach lines from a failed connection
	 * @param attachID ID of connection or Rod the lines are attached to (index
	 * is -1 this value)
	 * @param isRod 1 Rod end A, 2 Rod end B, 0 if connection
	 * @param lineIDs Array of one or more lines to detach (starting from 1...)
	 * @param lineTops An array that will be FILLED IN to return which end of
	 * each line was disconnected ...
	 * 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	 * @param nLinesToDetach how many lines to dettach
	 */
	moordyn::error_id detachLines(int attachID, int isRod,
	                              const int* lineIDs, int* lineTops,
	                              int nLinesToDetach, double time);

	/** @brief Print the output files
	 *
	 * Ths function is indeed chcking before that the output should be printed
	 * @param t Time instant
	 * @param dt Time step
	 * @return MOORDYN_SUCCESS if the output is correctly printed, an error
	 * code otherwise
	 */
	moordyn::error_id AllOutput(double t, double dt);
};

}  // ::moordyn
