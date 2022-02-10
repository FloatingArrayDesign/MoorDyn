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
#include "MoorDyn.h"
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

/** @brief Null buffer to avoid printing on screen
* @see moordyn::cnul
*/
class null_out_buf : public std::streambuf
{
public:
	virtual std::streamsize xsputn(const char PARAM_UNUSED *s, std::streamsize n)
	{
		return n;
	}
	virtual int overflow (int c)
	{
		return c;
	}
};

/// The buffer to nowhere
null_out_buf __cnul_buff;

/// Stream to nowhere
std::ostream __cnul(&__cnul_buff);

/// Stream to nowhere, used when verbosity is not large enough to print the
/// message
std::ostream& cnul = __cnul;

// The list of units for the output
const char *UnitList[] = {"(s)     ", "(m)     ", "(m)     ", "(m)     ", 
                          "(m/s)   ", "(m/s)   ", "(m/s)   ", "(m/s2)  ",
                          "(m/s2)  ", "(m/s2)  ", "(N)     ", "(N)     ",
                          "(N)     ", "(N)     "};

}  // ::moordyn

/** @class _MoorDyn
 * @brief A Mooring system
 *
 * This class contains everything required to hold a whole mooring system,
 * making everything thread-friendly easier
 */
class MoorDynSystem
{
public:
	/** @brief Constructor
	 *
	 * Remember to call Init() to initialize the mooring system
	 *
	 * @param infilename The input file, if either NULL or "", then
	 * "Mooring/lines.txt" will be considered
	 */
	MoorDynSystem(const char *infilename=NULL,
	              const int verbosity=MOORDYN_DBG_LEVEL);

	/** @brief Destuctor
	 */    
	~MoorDynSystem();

	/** @brief Get a stream to log data
	 * 
	 * Whether the message is logged, and where, depends on the verbosity level
	 *
	 * @param level Message level
	 */
	inline std::ostream& Cout(const int level=MOORDYN_MSG_LEVEL) const
	{
		if (level < _verbosity)
			return moordyn::cnul;
		if (level >= MOORDYN_ERR_LEVEL)
			return cerr;
		return cout;
	}

	/** @brief Initializes Moordyn, reading the input file and setting up the
	 * mooring lines
	 * @param x Position vector
	 * @param xd Velocity vector
	 * @note You can know the number of components required for \p x and \p xd
	 * with the function MoorDynSystem::NCoupedDOF()
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
	 * \p f with the function MoorDynSystem::NCoupedDOF()
	 */
	moordyn::error_id Step(const double *x, const double *xd, double *f,
	                       double &t, double &dt);

	/** @brief Get the connections
	 */
	inline vector<Body*> GetBodies() const { return _bodies; }

	/** @brief Get the connections
	 */
	inline vector<Rod*> GetRods() const { return _rods; }

	/** @brief Get the connections
	 */
	inline vector<Connection*> GetConnections() const { return _connections; }

	/** @brief Get the lines
	 */
	inline vector<Line*> GetLines() const { return _lines; }


	/** @brief Return the number of coupled Degrees Of Freedom (DOF)
	 *
	 * This should match with the number of components of the positions and
	 * velocities passed to MoorDynSystem::Init() and MoorDynSystem::Step()
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
		unsigned int n = 6 * _bodies_coupled_map.size() +
		                 3 * _connections_coupled_map.size();
		for (auto rodi : _rods_coupled_map)  
		{
			if (_rods[rodi]->type == -2)
				n += 6;  // cantilevered rods
			else
				n += 3;  // pinned rods
		}
		return n;
	}

	/** @brief Initializes the external Wave kinetics
	 *
	 * This is only used if _env.WaveKin > 0
	 * @return The number of points where the wave kinematics shall be provided
	 */
	inline unsigned int ExternalWaveKinInit()
	{
		_n_points_W = 0;
		
		for (auto line : _lines)
			_n_points_W += line->getN() + 1;

		// allocate arrays to hold data that could be passed in
		_U1      = make1Darray(3 * _n_points_W);
		_Ud1     = make1Darray(3 * _n_points_W);
		_U2      = make1Darray(3 * _n_points_W);
		_Ud2     = make1Darray(3 * _n_points_W);

		// initialize with zeros for safety
		_t_W1 = 0.0;
		_t_W2 = 0.0;
		memset(_U1, 0.0, 3 * _n_points_W * sizeof(double));
		memset(_Ud1, 0.0, 3 * _n_points_W * sizeof(double));
		memset(_U2, 0.0, 3 * _n_points_W * sizeof(double));
		memset(_Ud2, 0.0, 3 * _n_points_W * sizeof(double));

		return _n_points_W;
	}

	/** @brief Get the points where the waves kinematics shall be provided
	 * @param r The output coordinates
	 * @return MOORDYN_SUCCESS If the data is correctly set, an error code
	 * otherwise  (see @ref moordyn_errors)
	 * @see MoorDynSystem::ExternalWaveKinInit()
	 */
	inline moordyn::error_id GetWaveKinCoordinates(double *r) const
	{
		unsigned int i = 0;
		for (auto line : _lines)
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
				<< "MoorDynSystem::SetWaveKin()" << endl
				<< "Both velocity and acceleration arrays are needed"
				<< endl;
			return MOORDYN_INVALID_VALUE;
		}

		if (!_U1 || !_U2 || !_Ud1 || !_Ud2)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: Memory not allocated."
				<< " Have you called MoorDynSystem::ExternalWaveKinInit()?"
				<< endl;
			return MOORDYN_MEM_ERROR;
		}

		// set time stamp
		_t_W2 = _t_W1;
		_t_W1 = t;

		for (unsigned int i = 0; i < 3 * _n_points_W; i++)
		{
			_U2[i] = _U1[i];
			_Ud2[i] = _Ud1[i];
			_U1[i] = U[i];
			_Ud1[i] = Ud[i];
		}
		return MOORDYN_SUCCESS;
	}

protected:
	/** @brief Read the input file, setting up all the requird objects and their
	 * relationships
	 *
	 * This function is called from the constructor, so this information is
	 * ready when MoorDynSystem::Init() is called
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
	 * function MoorDynSystem::NCoupedDOF()
	 */
	inline moordyn::error_id GetForces(double *f) const
	{
		if (!f)
			return MOORDYN_INVALID_VALUE;
		unsigned int ix = 0;
		for (auto l : _bodies_coupled_map)
		{
			_bodies[l]->getFnet(f + ix);
			ix += 6;
		}
		for (auto l : _rods_coupled_map)
		{
			_rods[l]->getFnet(f + ix);
			if (_rods[l]->type == -2)
				ix += 6;  // for cantilevered rods 6 entries will be taken
			else
				ix += 3;  // for pinned rods 3 entries will be taken
		}
		for (auto l : _connections_coupled_map)
		{
			_connections[l]->getFnet(f + ix);
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
	/// Verbosity level
	int _verbosity;

	// factor by which to boost drag coefficients during dynamic relaxation IC generation
	double _ICD_factor = 5.0;
	// convergence analysis time step for IC generation
	double _ICD_dt = 1.0;
	// max time for IC generation
	double _ICD_t_max = 120;
	// threshold for relative change in tensions to call it converged
	double _ICD_threshold = 0.001;
	// temporary wave kinematics flag used to store input value while keeping env.WaveKin=0 for IC gen
	int _wave_kin_temp = 0;
	/// number of points that wave kinematics are input at
	/// (if using env.WaveKin=1)
	unsigned int _n_points_wave;
	/// (s) desired mooring line model time step
	double _dt;
	/// (s) desired output interval (the default zero value provides output at
	/// every call to MoorDyn)
	double _dt_out;

	/// General options of the Mooryng system
	EnvCond _env;
	/// The ground body, which is unique
	Body* _ground;
	/// Waves object that will be created to hold water kinematics info
	Waves *_waves = NULL;

	/// array of pointers to hold line library types
	vector<LineProps*> _line_props;
	/// array of pointers to hold rod library types
	vector<RodProps*> _rod_props;
	/// array of pointers to hold failure condition structs
	vector<FailProps*> _fail_props;
	/// array of pointers to connection objects (line joints or ends)
	vector<Body*> _bodies;
	/// array of pointers to Rod objects
	vector<Rod*> _rods;
	/// array of pointers to connection objects (line joints or ends)
	vector<Connection*> _connections;
	/// array of pointers to line objects
	vector<Line*> _lines;

	/// array of starting indices for Lines in "states" array
	vector<int> _lines_map;
	/// array of starting indices for indendent Connections in "states" array
	vector<int> _connections_map;
	/// array of starting indices for independent Rods in "states" array
	vector<int> _rods_map;
	/// array of starting indices for Bodies in "states" array
	vector<int> _bodies_map;

	/// vector of free body indices in _bodies vector
	vector<int> _bodies_free_map;
	/// vector of coupled/fairlead body indices in _bodies vector
	vector<int> _bodies_coupled_map;

	/// vector of free rod indices in _rods vector (this includes pinned rods
	/// because they are partially free and have states)
	vector<int> _rods_free_map;
	/// vector of coupled/fairlead rod indices in _rods vector
	vector<int> _rods_coupled_map;

	/// vector of free connection indices in _connections vector
	vector<int> _connections_free_map;
	/// vector of coupled/fairlead connection indices in _connections vector
	vector<int> _connections_coupled_map;

	/// Number of used state vector components
	unsigned int _n_states;
	/// full size of state vector array including extra space for detaching up
	/// to all line ends, each which could get its own 6-state connect
	/// (_n_states_extra = _n_states + 6 * 2 * _lines.size())
	unsigned int _n_states_extra;

	/// Global state vector
	double* _states;
	/// State vector at midpoint in the RK-2 integration scheme 
	double* _x_temp;
	/// Drivatives computed in the first step of the RK-2 integration scheme 
	double* _f0;
	/// Drivatives computed in the second step of the RK-2 integration scheme 
	double* _f1;

	/// number of points that wave kinematics are input at
	/// (if using env.WaveKin=1)
	unsigned int _n_points_W;
	/// time corresponding to the wave kinematics data
	double _t_W1;
	/// array of wave velocity at each of the _n_points_W points at time _t_W1
	double* _U1;
	/// array of wave acceleration at each of the _n_points_W points
	double* _Ud1;
	/// time corresponding to the wave kinematics data
	double _t_W2;
	/// array of wave velocity at each of the _n_points_W points at time _t_W2
	double* _U2;
	/// array of wave acceleration at each of the _n_points_W points
	double* _Ud2;

	/// log output file
	ofstream _log_file;

	/// main output file
	ofstream _out_file;

	/// a vector to hold ofstreams for each body, line or rod
	vector<shared_ptr<ofstream>> _outfiles;

	/// list of structs describing selected output channels for main out file
	vector<OutChanProps> _out_channels;

	/** @brief Create the log file if queried, close it otherwise
	 *
	 * Depending on the value of _env.writeLog value, a Log file stream will
	 * be open or closed
	 */
	inline moordyn::error_id SetupLog()
	{
		if (_env.writeLog > 0)
		{
			stringstream oname;
			oname << _basepath << _basename << ".log";
			Cout(MOORDYN_DBG_LEVEL) << "Creating a log file: '"
				<< oname.str() << "'" << endl;
			_log_file.open(oname.str());
			if (!_log_file.is_open())
			{
				Cout(MOORDYN_ERR_LEVEL) << "Unable to create the log file '"
				                        << oname.str() << "'" << endl;
				return MOORDYN_INVALID_OUTPUT_FILE;
			}
			_log_file << "MoorDyn v2 log file with output level "
			          << _env.writeLog << endl
			          << "Note: options above the writeLog line in the input "
					  << "file will not be recorded" << endl;
			// get pointer to outfile for MD objects to use
			_env.outfileLogPtr = & _log_file;
			return MOORDYN_SUCCESS;
		}

		if (_log_file.is_open())
		{
			_env.outfileLogPtr = NULL;
			_log_file.close();
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
			return _lines[channel.ObjID - 1]->GetLineOutput(channel);
		else if (channel.OType == 2)
			return _connections[channel.ObjID - 1]->GetConnectionOutput(channel);
		else if (channel.OType == 3)
			return _rods[channel.ObjID - 1]->GetRodOutput(channel);
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

MoorDynSystem::MoorDynSystem(const char *infilename, const int verbosity)
	: _filepath("Mooring/lines.txt")
	, _basename("lines")
	, _basepath("Mooring/")
	, _verbosity(verbosity)
	, _ICD_factor(5.0)
	, _ICD_dt(1.0)
	, _ICD_t_max(120.0)
	, _ICD_threshold(0.001)
	, _wave_kin_temp(0)
	, _n_points_wave(0)
	, _dt(0.001)
	, _dt_out(0.0)
	, _ground(NULL)
	, _waves(NULL)
	, _n_states(0)
	, _n_states_extra(0)
	, _states(NULL)
	, _x_temp(NULL)
	, _f0(NULL)
	, _f1(NULL)
	, _n_points_W(0)
	, _t_W1(0.0)
	, _U1(NULL)
	, _Ud1(NULL)
	, _t_W2(0.0)
	, _U2(NULL)
	, _Ud2(NULL)
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

	_env.g = 9.8;
	_env.WtrDpth = 0.;
	_env.rho_w = 1025.;
	_env.kb = 3.0e6;
	_env.cb = 3.0e5;
	_env.WaveKin = 0;                // 0=none
	_env.Current = 0;                // 0=none
	_env.dtWave = 0.25;
	_env.WriteUnits = 1;             // by default, write units line
	_env.writeLog = 0;               // by default, don't write out a log file
	_env.FrictionCoefficient = 0.0;
	_env.FricDamp = 200.0;
	_env.StatDynFricScale = 1.0;

	const moordyn::error_id err = ReadInFile();
	MOORDYN_THROW(err, "Exception while reading the input file");

	Cout(MOORDYN_DBG_LEVEL) << "MoorDyn is expecting " << NCoupedDOF()
	                        << " coupled degrees of freedom" << endl;

	if (!_n_states)
	{
		Cout(MOORDYN_WRN_LEVEL) << "WARNING: MoorDyn has no state variables."
		                        << " (Is there a mooring sytem?)" << endl;
	}

	_n_states_extra = _n_states + 6 * 2 * _lines.size();
	Cout(MOORDYN_DBG_LEVEL) << "Creating state vectors of size "
	                        << _n_states_extra << endl;
	_states = (double*) malloc(_n_states_extra * sizeof(double));
	_x_temp = (double*) malloc(_n_states_extra * sizeof(double));
	_f0 = (double*) malloc(_n_states_extra * sizeof(double));
	_f1 = (double*) malloc(_n_states_extra * sizeof(double));
	if (!_states || !_f0 || !_f1 || !_x_temp)
	{
		MOORDYN_THROW(MOORDYN_MEM_ERROR,
		              "Error allocating memory for state variables");
	}

	memset(_states, 0.0, _n_states_extra * sizeof(double));
	memset(_x_temp, 0.0, _n_states_extra * sizeof(double));
	memset(_f0, 0.0, _n_states_extra * sizeof(double));
	memset(_f1, 0.0, _n_states_extra * sizeof(double));
}

MoorDynSystem::~MoorDynSystem()
{
	if (_log_file.is_open())
		_log_file.close();
	if (_out_file.is_open())
		_out_file.close();
	for (auto outfile : _outfiles)  // int l=0; l<nLines; l++) 
		if (outfile && outfile->is_open())
			outfile->close();

	delete _ground;
	delete _waves;
	for (auto obj : _line_props)
		delete obj;
	for (auto obj : _rod_props)
		delete obj;
	for (auto obj : _fail_props)
		delete obj;
	for (auto obj : _bodies)
		delete obj;
	for (auto obj : _rods)
		delete obj;
	for (auto obj : _connections)
		delete obj;
	for (auto obj : _lines)
		delete obj;

	free(_states);
	free(_x_temp);
	free(_f0);
	free(_f1);
}

moordyn::error_id MoorDynSystem::Init(const double *x, const double *xd)
{
	if (NCoupedDOF() && !x)
	{
		Cout(MOORDYN_ERR_LEVEL) << "ERROR: "
			<< "MoorDynSystem::Init received a Null position vector, "
			<< "but " << NCoupedDOF() << "components are required" << endl;
	}

	// <<<<<<<<< need to add bodys
 
	// Allocate past line fairlead tension array, which is used for convergence
	// test during IC gen
	const unsigned int convergence_iters = 10;
	double **FairTensLast = make2Darray(_lines.size(), convergence_iters);
	for (unsigned int i = 0; i < _lines.size(); i++)
		for (unsigned int j = 0; j < convergence_iters; j++)
			FairTensLast[i][j] = 1.0 * j;

	// ------------------ do static bodies and lines ---------------------------

	Cout(MOORDYN_MSG_LEVEL) << "Creating mooring system..." << endl;

	// call ground body to update all the fixed things...
	_ground->initializeUnfreeBody(NULL, NULL, 0.0);

	// initialize coupled objects based on passed kinematics
	int ix = 0;

	for (auto l : _bodies_coupled_map)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Body " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		// this calls initiateStep and updateFairlead, then initializes
		// dependent Rods
		_bodies[l]->initializeUnfreeBody(x + ix, xd + ix, 0.0);
		ix += 6;
	}

	for (auto l : _rods_coupled_map)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Rod " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		_rods[l]->initiateStep(x + ix, xd + ix, 0.0);
		_rods[l]->updateFairlead(0.0);
		_rods[l]->initializeRod(NULL);  // call this just to set up the output file header
		
		if (_rods[l]->type == -2)
			ix += 6;  // for cantilevered rods 6 entries will be taken
		else
			ix += 3;  // for pinned rods 3 entries will be taken
	}

	for (auto l : _connections_coupled_map)
	{
		Cout(MOORDYN_MSG_LEVEL) << "Initializing coupled Connection " << l
			<< " in " << x[ix] << ", " << x[ix + 1] << ", " << x[ix + 2]
			<< "..." << endl;
		_connections[l]->initiateStep(x + ix, xd + ix, 0.0);
		_connections[l]->updateFairlead(0.0);
		ix += 3;
	}

	// initialize objects with states, writing their initial states to the
	//  master state vector (states)

	// Go through Bodys and write the coordinates to the state vector
	for (unsigned int l = 0; l < _bodies_free_map.size(); l++)  
		_bodies[_bodies_free_map[l]]->initializeBody(_states + _bodies_map[l]);

	// Go through independent (including pinned) Rods and write the coordinates to the state vector
	for (unsigned int l = 0; l < _rods_free_map.size(); l++)  
		_rods[_rods_free_map[l]]->initializeRod(_states + _rods_map[l]);

	// Go through independent connections (Connects) and write the coordinates to 
	// the state vector and set positions of attached line ends
	for (unsigned int l = 0; l < _connections_free_map.size(); l++)  
		_connections[_connections_free_map[l]]->initializeConnect(_states + _connections_map[l]);


	// Lastly, go through lines and initialize internal node positions using quasi-static model
	for (unsigned int l = 0; l < _lines.size(); l++)
		_lines[l]->initializeLine(_states + _lines_map[l]);

	// ------------------ do dynamic relaxation IC gen --------------------
	
	Cout(MOORDYN_MSG_LEVEL) << "Finalizing ICs using dynamic relaxation ("
	                        << _ICD_factor << "X normal drag)" << endl;
	
	// boost drag coefficients to speed static equilibrium convergence
	for (auto obj : _lines)
		obj->scaleDrag(_ICD_factor); 
	for (auto obj : _connections)
		obj->scaleDrag(_ICD_factor);
	for (auto obj : _rods)
		obj->scaleDrag(_ICD_factor);
	for (auto obj : _bodies)
		obj->scaleDrag(_ICD_factor);

	// vector to store tensions for analyzing convergence
	vector<double> FairTens(_lines.size(), 0.0);

	unsigned int iic = 0;
	double t = 0;
	bool converged = true;
	double max_error = 0.0;
	while (t < _ICD_t_max)
	{
		// Integrate one ICD timestep (_ICD_dt)
		double t_target = t + _ICD_dt;
		double dt;
		while ((dt = t_target - t) > 0.0)
		{
			if (_dt < dt)
				dt = _dt;
			const moordyn::error_id err = RK2(_states, t, dt);
			if (err != MOORDYN_SUCCESS)
			{
				free2Darray(FairTensLast, _lines.size());
				return err;
			}
		}

		// check for NaNs (actually it was already done in RK2)
		for (unsigned int i = 0; i < _n_states; i++)
		{
			if (isnan(_states[i]))
			{
				Cout(MOORDYN_ERR_LEVEL) << "Error: NaN value detected "
				    << "in MoorDyn state at dynamic relaxation time "
					<< t << " s." << endl;
				free2Darray(FairTensLast, _lines.size());
				return MOORDYN_NAN_ERROR;
			}
		}

		// Roll previous fairlead tensions for comparison
		for (unsigned int lf = 0; lf < _lines.size(); lf++) 
		{
			for (int pt = convergence_iters - 1; pt > 0; pt--)
				FairTensLast[lf][pt] = FairTensLast[lf][pt - 1];
			FairTensLast[lf][0] = FairTens[lf];
		}

		// go through connections to get fairlead forces 
		for (unsigned int lf = 0; lf < _lines.size(); lf++) 
			FairTens[lf] = _lines[lf]->getNodeTen(_lines[lf]->getN());

		// check for convergence (compare current tension at each fairlead with
		// previous convergence_iters-1 values)
		if (iic > convergence_iters)
		{
			// check for any non-convergence, and continue to the next time step
			// if any occurs
			converged = true;
			max_error = 0.0;
			for (unsigned int lf = 0; lf < _lines.size(); lf++) 
			{
				for (unsigned int pt = 0; pt < convergence_iters; pt++)
				{
					const double error = abs(
						FairTens[lf] / FairTensLast[lf][pt] - 1.0);
					if (error > max_error)
						max_error = error;
					if (max_error > _ICD_threshold)
					{
						converged = false;
						break;
					}
				}
				if (!converged)
					break;
			}

			if (converged)
				break;
		}

		iic++;
	}

	if (converged) {
		Cout(MOORDYN_MSG_LEVEL) << "Fairlead tensions converged" << endl;
	} else {
		Cout(MOORDYN_MSG_LEVEL) << "Fairlead tensions did not converged" << endl;
	}
	Cout(MOORDYN_MSG_LEVEL) << "Remaining error after " << t << " s = "
	                        << 100.0 * max_error << "%" << endl;

	free2Darray(FairTensLast, _lines.size());

	// restore drag coefficients to normal values and restart time counter of each object
	for (auto obj : _lines)
	{
		obj->scaleDrag(1.0 / _ICD_factor);
		obj->setTime(0.0);
	}
	for (auto obj : _connections)
	{
		obj->scaleDrag(1.0 / _ICD_factor);
		obj->setTime(0.0);
	}
	for (auto obj : _rods)
	{
		obj->scaleDrag(1.0 / _ICD_factor);
		obj->setTime(0.0);
	}
	for (auto obj : _bodies)
	{
		obj->scaleDrag(1.0 / _ICD_factor);
		obj->setTime(0.0);
	}

	// store passed WaveKin value to enable waves in simulation if applicable
	// (they're not enabled during IC gen)
	_env.WaveKin = _wave_kin_temp;

	// @mth: new approach to be implemented
	// ------------------------- calculate wave time series if needed -------------------
// 	if (_env.WaveKin == 2)
// 	{
// 		for (int l=0; l<_lines.size(); l++) 
// 			_lines[l]->makeWaveKinematics( 0.0 );
// 	}

	// -------------------------- start main output file --------------------------------

	stringstream oname;
	oname << _basepath << _basename << ".out";

	_out_file.open(oname.str());
	if (!_out_file.is_open())
	{
		Cout(MOORDYN_ERR_LEVEL) << "ERROR: Unable to write to main output file "
			<< oname.str() << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}

	// --- channel titles ---
	_out_file << "Time" << "\t ";
	for (auto channel : _out_channels)
		_out_file << channel.Name << "\t ";
	_out_file << endl;

	if (_env.WriteUnits > 0)
	{
		// --- units ---
		_out_file << "(s)" << "\t ";
		for (auto channel : _out_channels)
			_out_file << channel.Units << "\t ";
		_out_file << "\n";
	}

	// write t=0 output
	return AllOutput(0.0, 0.0);
}

moordyn::error_id MoorDynSystem::Step(const double *x,
                                      const double *xd,
                                      double *f,
                                      double &t,
                                      double &dt)
{
	// should check if wave kinematics have been set up if expected!

	if (dt <= 0)
	{
		// Nothing to do, just recover the forces
		return GetForces(f);
	}

	if (!x || !xd || !f)
	{
		Cout(MOORDYN_ERR_LEVEL) << "Null Pointer received in "
			<< __FUNC_NAME__
			<< " (" << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;
	}

	unsigned int ix = 0;

	// ---------------- set positions and velocities -----------------------
	// ... of any coupled bodies, rods, and connections at this instant, to be
	// used later for extrapolating motions
	for (auto l : _bodies_coupled_map)
	{
		_bodies[l]->initiateStep(x + ix, xd + ix, t);
		ix += 6;
	}
	for (auto l : _rods_coupled_map)
	{
		_rods[l]->initiateStep(x + ix, xd + ix, t);
		if (_rods[_rods_coupled_map[l]]->type == -2)
			ix += 6;  // for cantilevered rods 6 entries will be taken
		else
			ix += 3;  // for pinned rods 3 entries will be taken
	}
	for (auto l : _connections_coupled_map)
	{
		_connections[l]->initiateStep(x + ix, xd + ix, t);
		ix += 3;
	}

	// -------------------- do time stepping -----------------------
	double t_target = t + dt;
	double dt_step;
	while ((dt_step = t_target - t) > 0.0)
	{
		if (_dt < dt_step)
			dt_step = _dt;
		const moordyn::error_id err = RK2(_states, t, dt_step);
		if (err != MOORDYN_SUCCESS)
			return err;
	}
	t = t_target;

	// check for NaNs (actually it was already done in RK2)
	for (unsigned int i = 0; i < _n_states; i++)
	{
		if (isnan(_states[i]))
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error: NaN value detected "
			                        << "in MoorDyn state " << i
			                        << " at time " << t << " s" << endl;
			return MOORDYN_NAN_ERROR;
		}
	}

	// --------------- check for line failures (detachments!) ----------------
	// step 1: check for time-triggered failures
	for (unsigned int l = 0; l < _fail_props.size(); l++)
	{
		if (_fail_props[l]->failStatus)
			continue;
		if (t >= _fail_props[l]->failTime)
		{
			Cout(MOORDYN_MSG_LEVEL) << "Failure number " << l + 1
				<< " triggered at time " << t << endl;
			_fail_props[l]->failStatus = 1;
			const moordyn::error_id err = detachLines(
				_fail_props[l]->attachID,
				_fail_props[l]->isRod,
				_fail_props[l]->lineIDs,
				_fail_props[l]->lineTops,
				_fail_props[l]->nLinesToDetach,
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

moordyn::error_id MoorDynSystem::ReadInFile()
{
	unsigned int i=0;

	// factor by which to boost drag coefficients during dynamic relaxation IC generation
	_ICD_factor = 5.0;
	// convergence analysis time step for IC generation
	_ICD_dt = 1.0;
	// max time for IC generation
	_ICD_t_max = 120;
	// threshold for relative change in tensions to call it converged
	_ICD_threshold = 0.001;
	// temporary wave kinematics flag used to store input value while keeping env.WaveKin=0 for IC gen
	_wave_kin_temp = 0;
	// assume no wave kinematics points are passed in externally, unless ExernalWaveKinInit is called later
	_n_points_wave = 0;
	// default value for desired mooring model time step
	_dt = 0.001;

	// string containing which channels to write to output
	vector<string> outchannels;

	// make a "ground body" that will be the parent of all fixed objects (connections and rods)
	Cout(MOORDYN_DBG_LEVEL) << "Creating the ground body of type "
		<< Body::TypeName(Body::FIXED) << "..." << endl;
	_ground = new Body(); 
	_ground->setup(0, Body::FIXED, NULL, NULL, 0.0, 0.0, NULL, NULL, NULL, NULL);  

	// Make sure the state vector counter starts at zero
	// This will be conveniently incremented as each object is added
	_n_states = 0;

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
			Cout(MOORDYN_DBG_LEVEL) << "Reading line types..." << endl;

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
					<< " - with id " << _line_props.size() << endl;
				if (_env.writeLog > 1)
				{
					_log_file << "  - LineType" << _line_props.size() << ":"
					          << endl
					          << "    name: " << obj->type << endl
					          << "    d   : " << obj->d    << endl
					          << "    w   : " << obj->w    << endl
					          << "    Cdn : " << obj->Cdn  << endl
					          << "    Can : " << obj->Can  << endl
					          << "    Cdt : " << obj->Cdt  << endl
					          << "    Cat : " << obj->Cat  << endl;
				}

				_line_props.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"ROD DICTIONARY", "ROD TYPES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading rod types..." << endl;

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
					<< " - with id " << _rod_props.size() << endl;
				if (_env.writeLog > 1)
				{
					_log_file << "  - RodType" << _rod_props.size() << ":"
					          << endl
					          << "    name: " << obj->type << endl
					          << "    d   : " << obj->d    << endl
					          << "    w   : " << obj->w    << endl
					          << "    Cdn : " << obj->Cdn  << endl
					          << "    Can : " << obj->Can  << endl
					          << "    Cdt : " << obj->Cdt  << endl
					          << "    Cat : " << obj->Cat  << endl;
				}

				_rod_props.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"BODIES", "BODY LIST", "BODY PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading bodies..." << endl;

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
					_bodies_coupled_map.push_back(_bodies.size() - 1);
				}
				else 
				{
					// it is free - controlled by MoorDyn
					type = Body::FREE;
					_bodies_free_map.push_back(_bodies.size() - 1);
				}
				stringstream oname;
				oname << _basepath << _basename << "_Body" << number << ".out";
				_outfiles.push_back(make_shared<ofstream>(oname.str()));
				if (!_outfiles.back()->is_open())
				{
					Cout(MOORDYN_ERR_LEVEL) << "Cannot create the output file '"
						<< oname.str() << endl;
					return MOORDYN_INVALID_OUTPUT_FILE;
				}

				Body *obj = new Body();
				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of type " << Body::TypeName(type)
					<< " with id " << _bodies.size() << endl;
				obj->setup(number, type, r6, rCG, M, V, Inert, CdA, Ca,
				           _outfiles.back());
				_bodies.push_back(obj);
				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"RODS", "ROD LIST", "ROD PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading rods..." << endl;

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
					_rods_free_map.push_back(_rods.size());  // add this pinned rod to the free list because it is half free
					_rods_map.push_back(_n_states);       // assign start index of this rod's states
					_n_states += 6;                       // add 6 state variables for each pinned rod
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
					if (!bodyID || (bodyID > _bodies.size()))
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
						_rods_free_map.push_back(_rods.size());  // add this pinned rod to the free list because it is half free
						_rods_map.push_back(_n_states);       // assign start index of this rod's states
						_n_states += 6;                       // add 6 state variables for each pinned rod	
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
					_rods_coupled_map.push_back(_rods.size());  // index of fairlead in _rods vector
				}
				else if (!strcmp(let1, "VESPIN") || !strcmp(let1, "CPLDPIN"))
				{
					// if a pinned fairlead, add to list and add 
					type = Rod::CPLDPIN;
					_rods_coupled_map.push_back(_rods.size());  // index of fairlead in _rods vector
					_rods_free_map.push_back(_rods.size());     // also add this pinned rod to the free list because it is half free
					_rods_map.push_back(_n_states);          // assign start index of this rod's states
					_n_states += 6;                          // add 6 state variables for each pinned rod	
				}
				else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") || !strcmp(let1, "FREE"))
				{
					type = Rod::FREE;
					_rods_free_map.push_back(_rods.size());  // add this free rod to the free list
					_rods_map.push_back(_n_states);       // assign start index of this rod's states
					_n_states += 12;                      // add 12 state variables for each free rod
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
				for (unsigned int J = 0; J < _rod_props.size(); J++)  {
					if (_rod_props[J]->type == RodType)
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
					_outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!_outfiles.back()->is_open())
					{
						Cout(MOORDYN_ERR_LEVEL)
							<< "Cannot create the output file '"
							<< oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				}
				else
					_outfiles.push_back(NULL);

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of class " << RodType << " (" << TypeNum << ")"
					<< " and type " << Rod::TypeName(type)
					<< " with id " << _rods.size() << endl;

				Rod *obj = new Rod();
				obj->setup(number, type, _rod_props[TypeNum], endCoords,  
				           NumSegs, _outfiles.back(), outchannels);
				_rods.push_back(obj);

				// depending on type, assign the Rod to its respective parent body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
					_ground->addRodToBody(obj, endCoords);  
				else if (!strcmp(let1, "PINNED") || !strcmp(let1, "PIN"))
					_ground->addRodToBody(obj, endCoords);  
				else if (!strcmp(let1, "BODY"))
				{
					unsigned int bodyID = atoi(num1);
					_bodies[bodyID - 1]->addRodToBody(obj, endCoords); 
				}

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"POINTS", "POINT LIST", "CONNECTION PROPERTIES",
		                       "NODE PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading connections..." << endl;

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
					if (!bodyID || (bodyID > _bodies.size()))
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
					_connections_coupled_map.push_back(_connections.size());
				}
				else if (!strcmp(let1, "CONNECT") || !strcmp(let1, "CON") || !strcmp(let1, "FREE"))
				{
					// if a connect, add to list and add states for it
					type = Connection::FREE;
					_connections_free_map.push_back(_connections.size());
					_connections_map.push_back(_n_states);  // assign start index of this connect's states
					_n_states += 6;                         // add 6 state variables for each connect
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
				if (r0[2] < -_env.WtrDpth)
					_env.WtrDpth = -r0[2];

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of type " << Connection::TypeName(type)
					<< " with id " << _connections.size() << endl;

				// now make Connection object!
				Connection *obj = new Connection();
				obj->setup(number, type, r0, M, V, F, CdA, Ca);
				_connections.push_back(obj);

				// depending on type, assign the Connection to its respective parent body
				if (!strcmp(let1, "ANCHOR") || !strcmp(let1, "FIXED") || !strcmp(let1, "FIX"))
					_ground->addConnectionToBody(obj, r0); 
				else if (!strcmp(let1, "BODY"))
				{
					int bodyID = atoi(num1);
					_bodies[bodyID - 1]->addConnectionToBody(obj, r0);
				}

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"LINES", "LINE LIST", "LINE PROPERTIES"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading lines..." << endl;

			if (!_line_props.size())
			{
				Cout(MOORDYN_ERR_LEVEL)
					<< "Reading lines without defined line types" << endl;
				return MOORDYN_INVALID_INPUT;
			}
			if (!_connections.size())
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
				for (unsigned int J = 0; J < _line_props.size(); J++)  {
					if (_line_props[J]->type == type)
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
					_outfiles.push_back(make_shared<ofstream>(oname.str()));
					if (!_outfiles.back()->is_open())
					{
						Cout(MOORDYN_ERR_LEVEL)
							<< "Cannot create the output file '"
							<< oname.str() << endl;
						return MOORDYN_INVALID_OUTPUT_FILE;
					}
				}
				else
					_outfiles.push_back(NULL);

				Cout(MOORDYN_DBG_LEVEL) << "\t'" << number << "'"
					<< " - of class " << type << " (" << TypeNum << ")"
					<< " with id " << _lines.size() << endl;

				Line *obj = new Line();
				obj->setup(number, _line_props[TypeNum], UnstrLen, NumSegs, 
				           _outfiles.back(), outchannels);
				_lines.push_back(obj);
				_lines_map.push_back(_n_states);  // assign start index of this Line's states
				_n_states += 6 * (NumSegs - 1);   // add 6 state variables for each internal node of this line

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
						if (!id || id > _rods.size())
						{
							Cout(MOORDYN_ERR_LEVEL) << "Error in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "There are not " << id << " rods" << endl;
							return MOORDYN_INVALID_INPUT;
						}
						if (!strcmp(let2, "A")) 
							_rods[id - 1]->addLineToRodEndA(obj, I);
						else if (!strcmp(let2, "B")) 
							_rods[id - 1]->addLineToRodEndB(obj, I);
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
						if (!id || id > _connections.size())
						{
							Cout(MOORDYN_ERR_LEVEL) << "Error in "
								<< _filepath << ":" << i + 1 << "..." << endl
								<< "'" << in_txt[i] << "'" << endl
								<< "There are not " << id << " connections" << endl;
							return MOORDYN_INVALID_INPUT;
						}
						_connections[id - 1]->addLineToConnect(obj, I);
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

				i++;
			}
		}

		if (moordyn::str::has(moordyn::str::upper(in_txt[i]),
		                      {"FAILURE"}))
		{
			Cout(MOORDYN_DBG_LEVEL) << "Reading failure conditions..." << endl;

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
				_fail_props.push_back(obj);

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
					if (!id || id > _rods.size())
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
					if (!id || id > _connections.size())
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
			Cout(MOORDYN_DBG_LEVEL) << "Reading options..." << endl;

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
					_env.writeLog = atoi(entries[0].c_str());
					const moordyn::error_id err = SetupLog();
					if (err != MOORDYN_SUCCESS)
						return err;
				}
				// DT is old way, should phase out
				else if ((name == "dtM") || (name == "DT"))
					_dt = atof(entries[0].c_str());
				else if ((name == "g") || (name == "gravity"))
					_env.g  = atof(entries[0].c_str()); 
				else if ((name =="Rho") || (name=="rho") || (name=="WtrDnsty"))
					_env.rho_w = atof(entries[0].c_str()); 
				else if (name == "WtrDpth")
					_env.WtrDpth = atof(entries[0].c_str()); 
				else if ((name == "kBot") || (name == "kb"))
					_env.kb = atof(entries[0].c_str());
				else if ((name == "cBot") || (name == "cb"))
					_env.cb = atof(entries[0].c_str());
				else if ((name == "dtIC") || (name == "_ICD_dt"))
					_ICD_dt = atof(entries[0].c_str());
				else if ((name == "TmaxIC") || (name == "_ICD_t_max"))
					_ICD_t_max = atof(entries[0].c_str());
				else if ((name == "CdScaleIC") || (name == "_ICD_factor"))
					_ICD_factor   = atof(entries[0].c_str());
				else if ((name == "threshIC") || (name == "_ICD_threshold"))
					_ICD_threshold = atof(entries[0].c_str());
				else if (name == "WaveKin")
					_wave_kin_temp = atoi(entries[0].c_str());
				else if (name == "Currents")
					_env.Current = atoi(entries[0].c_str());
				else if (name == "WriteUnits")
					_env.WriteUnits = atoi(entries[0].c_str());
				else if (name == "FrictionCoefficient")
					_env.FrictionCoefficient = atof(entries[0].c_str());
				else if (name == "FricDamp")
					_env.FricDamp = atof(entries[0].c_str());
				else if (name == "StatDynFricScale")
					_env.StatDynFricScale = atof(entries[0].c_str());
				// output writing period (0 for at every call)
				else if (name == "dtOut")
					_dt_out = atof(entries[0].c_str());
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
			Cout(MOORDYN_DBG_LEVEL) << "Reading output options..." << endl;

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
						dummy.NodeID = _lines[dummy.ObjID-1]->getN();
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
						_out_channels.push_back(dummy);
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
		<< "\tnLineTypes  = " << _line_props.size() << endl
		<< "\tnRodTypes   = " << _rod_props.size() << endl
		<< "\tnPoints     = " << _connections.size() << endl
		<< "\tnBodies     = " << _bodies.size() << endl
		<< "\tnRods       = " << _rods.size() << endl
		<< "\tnLines      = " << _lines.size() << endl
		<< "\tnFails      = " << _fail_props.size() << endl
		<< "\tnFreeBodies = " << _bodies_free_map.size() << endl
		<< "\tnFreeRods   = " << _rods_free_map.size() << endl
		<< "\tnFreePonts  = " << _connections_free_map.size() << endl
		<< "\tnCpldBodies = " << _bodies_coupled_map.size() << endl
		<< "\tnCpldRods   = " << _rods_coupled_map.size() << endl
		<< "\tnCpldPoints = " << _connections_coupled_map.size() << endl;
	
	// write system description to log file
	if (_env.writeLog > 0)
	{
		_log_file << "----- MoorDyn Model Summary (to be written) -----"
		          << endl;
	}

	// Setup the waves and populate them
	_waves = new Waves();
	_waves->setup(&_env);

	_ground->setEnv( &_env, _waves); 
	for (auto obj : _bodies)
		obj->setEnv( &_env, _waves); 
	for (auto obj : _rods)
		obj->setEnv( &_env, _waves); 
	for (auto obj : _connections)
		obj->setEnv( &_env, _waves); 
	for (auto obj : _lines)
		obj->setEnv( &_env, _waves);

	return MOORDYN_SUCCESS;
}

moordyn::error_id MoorDynSystem::CalcStateDeriv(double *x,  double *xd,
                                                const double t, const double dt)
{
	// call ground body to update all the fixed things...
	_ground->updateFairlead(t); 

	// couple things...

	// extrapolate instantaneous positions of any coupled bodies (type -1)
	for (auto l : _bodies_coupled_map)  
		_bodies[l]->updateFairlead(t);

	// extrapolate instantaneous positions of any coupled or fixed rods (type -1 or -2)
	for (auto l : _rods_coupled_map)  
		_rods[l]->updateFairlead(t);

	// extrapolate instantaneous positions of any coupled or fixed connections (type -1)
	for (auto l : _connections_coupled_map)  
		_connections[l]->updateFairlead(t); 

	// update wave kinematics if applicable
	if (_env.WaveKin == 1)
	{
		// extrapolate velocities from accelerations
		// (in future could extrapolote from most recent two points,
		// (_U1 and _U2)

		double t_delta = t - _t_W1;

		vector<double> U_extrap;
		for (unsigned int i = 0; i < _n_points_W * 3; i++)
			U_extrap.push_back(_U1[i] + _Ud1[i] * t_delta);

		// distribute to the appropriate objects
		unsigned int i = 0;
		for (auto line : _lines) 	
		{
			line->setNodeWaveKin(U_extrap.data() + 3 * i, _Ud1 + 3 * i);
			i += 3 * line->getN() + 3;
		}
	}

	// independent or semi-independent things with their own states...

	// give Bodies latest state variables (kinematics will also be assigned to dependent connections and rods, and thus line ends)
	for (unsigned int l = 0; l < _bodies_free_map.size(); l++)  
		_bodies[_bodies_free_map[l]]->setState((x + _bodies_map[l]), t);

	// give independent or pinned rods' latest state variables (kinematics will also be assigned to attached line ends)
	for (unsigned int l = 0; l < _rods_free_map.size(); l++)  
		_rods[_rods_free_map[l]]->setState((x + _rods_map[l]), t);

	// give Connects (independent connections) latest state variable values (kinematics will also be assigned to attached line ends)
	for (unsigned int l = 0; l < _connections_free_map.size(); l++)  
		_connections[_connections_free_map[l]]->setState((x + _connections_map[l]), t);

	// give Lines latest state variable values for internal nodes
	for (unsigned int l = 0; l < _lines.size(); l++)
		_lines[l]->setState((x + _lines_map[l]), t);

	// calculate dynamics of free objects (will also calculate forces (doRHS())
	// from any child/dependent objects)...

	// calculate line dynamics (and calculate line forces and masses attributed to connections)
	for (unsigned int l = 0; l < _lines.size(); l++)
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			_lines[l]->getStateDeriv((xd + _lines_map[l]), dt);
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Exception detected at " << t << " s: "
			                        << err_msg << endl;
		}
	}

	// calculate connect dynamics (including contributions from attached lines
	// as well as hydrodynamic forces etc. on connect object itself if applicable)
	for (unsigned int l = 0; l < _connections_free_map.size(); l++)  
		_connections[_connections_free_map[l]]->getStateDeriv((xd + _connections_map[l]));

	// calculate dynamics of independent Rods 
	for (unsigned int l = 0; l < _rods_free_map.size(); l++)  
		_rods[_rods_free_map[l]]->getStateDeriv((xd + _rods_map[l]));

	// calculate dynamics of Bodies
	for (unsigned int l = 0; l < _bodies_free_map.size(); l++)  
		_bodies[_bodies_free_map[l]]->getStateDeriv((xd + _bodies_map[l]));

	// get dynamics/forces (doRHS()) of coupled objects, which weren't addressed in above calls

	for (unsigned int l = 0; l < _connections_coupled_map.size(); l++)  
		_connections[_connections_coupled_map[l]]->doRHS();

	for (unsigned int l = 0; l < _rods_coupled_map.size(); l++)  
		_rods[_rods_coupled_map[l]]->doRHS();

	for (unsigned int l = 0; l < _bodies_coupled_map.size(); l++)  
		_bodies[_bodies_coupled_map[l]]->doRHS();

	// call ground body to update all the fixed things
	// _ground->doRHS();
	_ground->setDependentStates();  // (not likely needed) <<<

	return MOORDYN_SUCCESS;
}

moordyn::error_id MoorDynSystem::RK2(double *x, double &t, const double dt)
{
	moordyn::error_id err;

	if (_env.writeLog > 2)
		_log_file << "\n----- RK2 predictor call to CalcStateDeriv at time "
		          << t << " s -----\n";

	// get derivatives at t0. f0 = f(t0, x0);
	err = CalcStateDeriv(x, _f0, t, dt);
	if (err)
		return err;

	// integrate to t0 + dt/2. x1 = x0 + dt*f0/2.0;
	for (unsigned int i = 0; i < _n_states; i++)
		_x_temp[i] = x[i] + 0.5 * dt * _f0[i];

	if (_env.writeLog > 2)
		_log_file << "\n----- RK2 corrector call to CalcStateDeriv at time "
		          << t + 0.5 * dt << " s -----\n";

	// get derivatives at t0 + dt/2. f1 = f(t1, x1);
	err = CalcStateDeriv(_x_temp, _f1, t + 0.5 * dt, dt);
	if (err)
		return err;

	// integrate states to t0 + dt
	for (unsigned int i = 0; i < _n_states; i++)
		x[i] = x[i] + dt * _f1[i];

	// update time
	t = t + dt;

	// <<<<<<<<< maybe should check/force all rod unit vectors to be unit
	// vectors here?

	return MOORDYN_SUCCESS;
}

moordyn::error_id MoorDynSystem::detachLines(int attachID, int isRod,
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

	_n_states += 6;  // add 6 state variables for each connect

	// check to make sure we haven't gone beyond the extra size allotted to the
	// state arrays or the connections list <<<< really should throw an error
	// here
	if (_n_states > _n_states_extra)
	{
		Cout(MOORDYN_ERR_LEVEL) << "Error: _n_states = " << _n_states
			<< " is bigger than _n_states_extra = " << _n_states_extra << endl;
		return MOORDYN_MEM_ERROR;
	}

	// add connect to list of free ones and add states for it
	_connections_free_map.push_back(_connections.size());
	// assign start index of this connect's states
	_connections_map.push_back(_n_states);

	// now make Connection object!
	Connection *obj = new Connection();
	obj->setup(_connections.size() + 1, type, r0, M, V, F, CdA, Ca);
	obj->setEnv(&_env, _waves);
	_connections.push_back(obj);

	// dummy state array to hold kinematics of old attachment point (format in
	// terms of part of connection state vector:
	// r[J]  = X[3 + J]; rd[J] = X[J]; )
	double dummyConnectState[6];

	// detach lines from old Rod or Connection, and get kinematics of the old attachment point
	for (int l = 0; l < nLinesToDetach; l++)
	{
		if (isRod == 1)
			_rods[attachID-1]->removeLineFromRodEndA(lineIDs[l],
			                                         lineTops + l,
			                                         dummyConnectState + 3,
			                                         dummyConnectState);
		else if (isRod == 2)
			_rods[attachID-1]->removeLineFromRodEndB(lineIDs[l],
			                                         lineTops + l,
			                                         dummyConnectState + 3,
			                                         dummyConnectState);
		else if (isRod == 0)
			_connections[attachID-1]->removeLineFromConnect(lineIDs[l],
			                                                lineTops + l,
			                                                dummyConnectState + 3,
			                                                dummyConnectState);
		else
		{
			Cout(MOORDYN_ERR_LEVEL)
				<< "Error: Failure does not have a valid isRod value" << endl;
			return MOORDYN_INVALID_VALUE;
		}
	}

	// attach lines to new connection
	for (int l = 0; l < nLinesToDetach; l++)
		obj->addLineToConnect(_lines[lineIDs[l] - 1], lineTops[l]);

	// update connection kinematics to match old line attachment point
	// kinematics and set positions of attached line ends
	obj->setState(dummyConnectState, time);

	// now make the state vector up to date!
	for (unsigned int J = 0; J < 6; J++)
		_states[_connections_map.back() + J] = dummyConnectState[J];

	return MOORDYN_SUCCESS;
}

moordyn::error_id MoorDynSystem::AllOutput(double t, double dt)
{
	if (_dt_out > 0)
		if (t < (floor((t - dt) / _dt_out) + 1.0) * _dt_out)
			return MOORDYN_SUCCESS;
	
	// write to master output file
	if (!_out_file.is_open())
	{
		Cout(MOORDYN_ERR_LEVEL) << "Error: Unable to write to main output file "
		                        << endl;
		return MOORDYN_INVALID_OUTPUT_FILE;
	}
	_out_file << t << "\t "; 		// output time
	for (auto channel : _out_channels)   
	{
		moordyn::error_id err = MOORDYN_SUCCESS;
		string err_msg;
		try
		{
			_out_file << GetOutput(channel) << "\t ";
		}
		MOORDYN_CATCHER(err, err_msg);
		if (err != MOORDYN_SUCCESS)
		{
			Cout(MOORDYN_ERR_LEVEL) << "Error handling an output channel:"
				<< err_msg << endl;
			return err;
		}
	}
	_out_file << endl;

	// write individual output files
	for (auto obj : _lines)
		obj->Output(t);
	for (auto obj : _rods)
		obj->Output(t);
	for (auto obj : _bodies)
		obj->Output(t);

	return MOORDYN_SUCCESS;
}


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
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	MoorDynSystem *instance = NULL;
	try
	{
		instance = new MoorDynSystem(infilename);
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

unsigned int DECLDIR MoorDyn_NCoupledDOF(MoorDyn system)
{
	if (!system)
		return 0;
	return ((MoorDynSystem*)system)->NCoupedDOF();
}

int DECLDIR MoorDyn_Init(MoorDyn system, const double *x, const double *xd)
{
	CHECK_SYSTEM(system);
	return ((MoorDynSystem*)system)->Init(x, xd);
}

int DECLDIR MoorDyn_Step(MoorDyn system, const double *x, const double *xd,
                         double *f, double *t, double *dt)
{
	CHECK_SYSTEM(system);
	return ((MoorDynSystem*)system)->Step(x, xd, f, *t, *dt);
}

int DECLDIR MoorDyn_Close(MoorDyn system)
{
	CHECK_SYSTEM(system);
	delete((MoorDynSystem*)system);
	return MOORDYN_SUCCESS;
}

int DECLDIR MoorDyn_InitExtWaves(MoorDyn system, unsigned int *n)
{
	CHECK_SYSTEM(system);

	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try
	{
		*n = ((MoorDynSystem*)system)->ExternalWaveKinInit();
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

	return ((MoorDynSystem*)system)->GetWaveKinCoordinates(r);
}

int DECLDIR MoorDyn_SetWaves(MoorDyn system, const double *U,
                                             const double *Ud,
                                             double t)
{
	CHECK_SYSTEM(system);

	return ((MoorDynSystem*)system)->SetWaveKin(U, Ud, t);
}

unsigned int DECLDIR MoorDyn_GetNumberBodies(MoorDyn system)
{
	if (!system)
		return 0;
	return ((MoorDynSystem*)system)->GetBodies().size();
}

unsigned int DECLDIR MoorDyn_GetNumberRods(MoorDyn system)
{
	if (!system)
		return 0;
	return ((MoorDynSystem*)system)->GetRods().size();
}

unsigned int DECLDIR MoorDyn_GetNumberConnections(MoorDyn system)
{
	if (!system)
		return 0;
	return ((MoorDynSystem*)system)->GetConnections().size();
}

unsigned int DECLDIR MoorDyn_GetNumberLines(MoorDyn system)
{
	if (!system)
		return 0;
	return ((MoorDynSystem*)system)->GetLines().size();
}

unsigned int DECLDIR MoorDyn_GetNumberLineNodes(MoorDyn system,
                                                unsigned int line)
{
	if (!system)
		return 0;

	auto lines = ((MoorDynSystem*)system)->GetLines();
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

	auto lines = ((MoorDynSystem*)system)->GetLines();
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

	auto lines = ((MoorDynSystem*)system)->GetLines();
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

	auto conns = ((MoorDynSystem*)system)->GetConnections();
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

	auto conns = ((MoorDynSystem*)system)->GetConnections();
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

	auto lines = ((MoorDynSystem*)system)->GetLines();
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
	for (auto line : ((MoorDynSystem*)system)->GetLines())
		line->drawGL2();  
	for (auto conn : ((MoorDynSystem*)system)->GetConnections())
		conn->drawGL();  
#endif
	return MOORDYN_SUCCESS;
}


// =============================================================================
//
//                     ||                     ||
//                     ||      Old C API      ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

/** @defgroup cmd Output console handling
 *  @{
 */

#ifdef WIN32

/// Console handle
int hConHandle;
/// Std output handle
intptr_t lStdHandle;

/// pointer to be made to environment variable PROMPT
char const* PromptPtr;
/// 0 if the system console is used, 1 if the console has been created by us
int OwnConsoleWindow = 0;

#endif

/**
 * @}
 */

/** \addtogroup old_api
 *  @{
 */

/// The singleton of the very only MoorDyn instance that this process might hold
/// This only applies if the old API is cosidered. See @ref old_api
MoorDyn md_singleton = NULL;

/**
 * @}
 */

int DECLDIR MoorDynInit(const double x[], const double xd[], const char *infilename)
{
#ifdef WIN32
	// ------------ create console window for messages if none already available -----------------
	// adapted from Andrew S. Tucker, "Adding Console I/O to a Win32 GUI App" in Windows Developer Journal, December 1997. source code at http://dslweb.nwnexus.com/~ast/dload/guicon.htm

	FILE *fp;
	// get pointer to environment variable "PROMPT" (NULL if not in console)
	PromptPtr = getenv("PROMPT");
	
	//TODO: simplify this to just keep the output parts I need

	HWND consoleWnd = GetConsoleWindow();
	if (!consoleWnd)
	{
		// if not in console, create our own
		OwnConsoleWindow = 1;

		// allocate a console for this app
		AllocConsole();

		// set the screen buffer to be big enough to let us scroll text
	    static const WORD MAX_CONSOLE_LINES = 500;
	    CONSOLE_SCREEN_BUFFER_INFO coninfo;
		GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
		coninfo.dwSize.Y = MAX_CONSOLE_LINES;
		SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

		// redirect unbuffered STDOUT to the console
		//lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
		lStdHandle = (intptr_t)GetStdHandle(STD_OUTPUT_HANDLE);
		hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
		fp = _fdopen( hConHandle, "w" );
		*stdout = *fp;
		setvbuf(stdout, NULL, _IONBF, 0);

		// redirect unbuffered STDERR to the console
		lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
		hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
		fp = _fdopen( hConHandle, "w" );
		*stderr = *fp;
		setvbuf( stderr, NULL, _IONBF, 0 );

		// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
		// point to console as well
		ios::sync_with_stdio();
		
		cout << "(MoorDyn-initiated console window)" << endl;
	}
#endif

	MoorDyn instance = MoorDyn_Create(infilename);
	if (!instance)
		return MOORDYN_UNHANDLED_ERROR;

	int err = MoorDyn_Init(instance, x, xd);
	if (err)
		return err;

	if (md_singleton)
		MoorDyn_Close(md_singleton);  // We do not care if this fails
	md_singleton = instance;

	return MOORDYN_SUCCESS;
}

// This is the original time stepping function, for platform-centric coupling, but now it has capabilities for multiple 6DOF coupled bodies
int DECLDIR MoorDynStep(const double x[], const double xd[], double f[], double* t_in, double* dt_in) 
{
	if (!md_singleton)
		return MOORDYN_INVALID_VALUE;

	return MoorDyn_Step(md_singleton, x, xd, f, t_in, dt_in);
}

int DECLDIR MoorDynClose(void)
{
	if (!md_singleton)
		return MOORDYN_INVALID_VALUE;

	int err = MoorDyn_Close(md_singleton);
	if (err)
		return err;

	md_singleton = NULL;
	cout << "   MoorDyn closed." << endl;

#ifdef WIN32
	if (OwnConsoleWindow == 1)  {
		cout << "press enter to close: " << endl;
		cin.get();
		FreeConsole();
	}
#endif

	return 0;
}

int DECLDIR externalWaveKinInit()
{
	if (!md_singleton)
		return 0;

	unsigned int n;
	int err = MoorDyn_InitExtWaves(md_singleton, &n);
	if (err)
		return 0;

	return (int)n;
}

// returns array providing coordinates of all points that will be receiving wave kinematics
void DECLDIR getWaveKinCoordinates(double r_out[])
{
	if (!md_singleton)
		return;
	MoorDyn_GetWavesCoords(md_singleton, r_out);
}

// receives arrays containing U and Ud for each point at which wave kinematics will be applied (and the time they are calculated at)
void DECLDIR setWaveKin(const double U_in[], const double Ud_in[], double t_in)
{
	if (!md_singleton)
		return;
	MoorDyn_SetWaves(md_singleton, U_in, Ud_in, t_in);
}


double DECLDIR GetFairTen(int l)
{
	if (!md_singleton)
		return -1;
	return MoorDyn_GetFairTen(md_singleton, l);
}



int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] )
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetFASTtens(
		md_singleton, numLines, FairHTen, FairVTen, AnchHTen, AnchVTen);
}

int DECLDIR GetConnectPos(int l, double pos[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetConnectPos(md_singleton, (unsigned int)l, pos);
}

int DECLDIR GetConnectForce(int l, double force[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetConnectForce(md_singleton, (unsigned int)l, force);
}

int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetNodePos(
		md_singleton, (unsigned int)LineNum, (unsigned int)NodeNum, pos);
}


int DECLDIR DrawWithGL()
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_DrawWithGL(md_singleton);
}
