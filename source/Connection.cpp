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

#include "Connection.hpp"
#include "Connection.h"
#include "Line.hpp"
#include "Waves.hpp"
#include <tuple>

#ifdef USE_VTK
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkVertex.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLPolyDataWriter.h>
#endif

namespace moordyn {

Connection::Connection(moordyn::Log* log, size_t id)
  : io::IO(log)
  , seafloor(nullptr)
  , connId(id)
{
}

Connection::~Connection() {}

void
Connection::setup(int number_in,
                  types type_in,
                  vec r0_in,
                  double M_in,
                  double V_in,
                  vec F_in,
                  double CdA_in,
                  double Ca_in)
{
	// props contains:
	// Node, Type, X, Y, Z, M, V, FX, FY, FZ, CdA, Ca

	number = number_in;
	type = type_in;

	// store passed rod properties  >>>(and convert to numbers)<<<
	conM = M_in;
	conV = V_in;
	conF = F_in;
	conCdA = CdA_in;
	conCa = Ca_in;

	// beta = 0.0;

	// Start off at position specified in input file (will be overwritten for
	// fairleads). This is the starting point for connects and the permanent
	// location of anchors.
	r = r0_in;
	rd = { 0.0, 0.0, 0.0 };

	LOGDBG << "   Set up Connection " << number << ", type '" << TypeName(type)
	       << "'. " << endl;
}

// this function handles assigning a line to a connection node
void
Connection::addLine(Line* theLine, EndPoints end_point)
{
	LOGDBG << "L" << theLine->number << end_point_name(end_point) << "->P"
	       << number << " ";

	attachment a = { theLine, end_point };
	attached.push_back(a);
};

EndPoints
Connection::removeLine(Line* line)
{
	EndPoints end_point;
	// look through attached lines
	for (auto it = std::begin(attached); it != std::end(attached); ++it) {
		if (it->line != line)
			continue;
		// This is the line's entry in the attachment list
		end_point = it->end_point;
		attached.erase(it);

		LOGMSG << "Detached line " << line->number << " from Connection "
		       << number << endl;
		return end_point;
	}

	// line not found
	LOGERR << "Error: failed to find line to remove during "
	       << __PRETTY_FUNC_NAME__ << " call to connection " << number
	       << ". Line " << line->number << endl;
	throw moordyn::invalid_value_error("Invalid line");
};

std::pair<vec, vec>
Connection::initialize()
{
	// the default is for no water kinematics to be considered (or to be set
	// externally on each node)

	vec pos = vec::Zero();
	vec vel = vec::Zero();

	if (type == FREE) {
		// pass kinematics to any attached lines so they have initial positions
		// at this initialization stage
		for (auto a : attached)
			a.line->setEndKinematics(r, rd, a.end_point);

		// assign initial node kinematics to state vector
		pos = r;
		vel = rd;

		const real waterDepth = seafloor ? seafloor->getDepthAt(r[0], r[1]) : -env->WtrDpth;
		if (waterDepth > r[2]) {
			LOGERR << "Error: water depth is shallower than Point " << number
			       << "." << endl;
			throw moordyn::invalid_value_error("Invalid water depth");
		}
	}

	LOGDBG << "   Initialized Connection " << number << endl;

	return std::make_pair(pos, vel);
};

// function to return net force on connection (just to allow public reading of
// Fnet)
void
Connection::getFnet(vec& Fnet_out)
{
	Fnet_out = Fnet;
};

// function to return mass matrix of connection
void
Connection::getM(mat& M_out)
{
	M_out = M;
};

real
Connection::GetConnectionOutput(OutChanProps outChan)
{
	if (outChan.QType == PosX)
		return r[0];
	else if (outChan.QType == PosY)
		return r[1];
	else if (outChan.QType == PosZ)
		return r[2];
	else if (outChan.QType == VelX)
		return rd[0];
	else if (outChan.QType == VelY)
		return rd[1];
	else if (outChan.QType == VelZ)
		return rd[2];
	else if (outChan.QType == Ten)
		return Fnet.squaredNorm();
	else if (outChan.QType == FX)
		return Fnet[0]; // added Oct 20
	else if (outChan.QType == FY)
		return Fnet[1];
	else if (outChan.QType == FZ)
		return Fnet[2];
	else {
		return 0.0;
	}
}

void
Connection::setEnv(EnvCondRef env_in,
                   moordyn::WavesRef waves_in,
                   moordyn::SeafloorRef seafloor_in)
{
	env = env_in;     // set pointer to environment settings object
	waves = waves_in; // set pointer to Waves  object
	seafloor = seafloor_in;
}

void
Connection::initiateStep(vec rFairIn, vec rdFairIn)
{
	if (type != COUPLED) {
		LOGERR << "Invalid Connection " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid connection type");
	}

	// update values to fairlead position and velocity functions
	// (function of time)
	r_ves = rFairIn;
	rd_ves = rdFairIn;

	// do I want to get precalculated values here at each FAST time step or at
	// each line time step?
};

void
Connection::updateFairlead(real time)
{
	if (type != COUPLED) {
		LOGERR << "Invalid Connection " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid connection type");
	}

	// set fairlead position and velocity based on BCs (linear model for now)
	r = r_ves + rd_ves * time;
	rd = rd_ves;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

void
Connection::setKinematics(vec r_in, vec rd_in)
{
	if (type != FIXED) {
		LOGERR << "Invalid Connection " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid connection type");
	}

	// set position and velocity
	r = r_in;
	rd = rd_in;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

void
Connection::setState(vec pos, vec vel)
{
	// the kinematics should only be set with this function of it's an
	// independent/free connection
	if (type != FREE) {
		LOGERR << "Invalid Connection " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid connection type");
	}

	// from state values, get r and rdot values
	r = pos;
	rd = vel;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

std::pair<vec, vec>
Connection::getStateDeriv()
{
	// the RHS is only relevant (there are only states to worry about) if it is
	// a Connect type of Connection
	if (type != FREE) {
		LOGERR << "Invalid Connection " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid connection type");
	}

	// cout << "ConRHS: m: " << M[0][0] << ", f: " << Fnet[0] << " " <<
	// Fnet[1] << " " << Fnet[2] << endl;
	doRHS();

	// solve for accelerations in [M]{a}={f}
	const vec acc = M.inverse() * Fnet;

	// update states
	return std::make_pair(rd, acc);
};

void
Connection::getNetForceAndMass(vec6& Fnet_out, mat6& M_out, vec rBody)
{
	doRHS();

	// position of connection relative to the body reference point (global
	// orientation frame)
	const vec rRel = r - rBody;

	// convert segment net force into 6dof force about body ref point
	Fnet_out(Eigen::seqN(0, 3)) = Fnet;
	Fnet_out(Eigen::seqN(3, 3)) = rRel.cross(Fnet);

	// convert segment mass matrix to 6by6 mass matrix about body ref point
	M_out = translateMass(rRel, M);
}

moordyn::error_id
Connection::doRHS()
{
	// start with the Connection's own forces including buoyancy and weight, and
	// its own mass
	Fnet = conF + vec(0.0, 0.0, env->g * (conV * env->rho_w - conM));

	// start with physical mass
	M = conM * mat::Identity();

	// loop through attached lines, adding force and mass contributions
	for (auto a : attached) {
		vec Fnet_i, Moment_dummy;
		mat M_i;

		// get quantities
		a.line->getEndStuff(Fnet_i, Moment_dummy, M_i, a.end_point);

		// Process outline for line failure (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if
		// failure time has elapsed for line
		// 2. create new massless connect with same instantaneous kinematics as
		// current connection
		// 3. disconnect line end from current connection and instead attach to
		// new connect The above may require rearrangement of connection
		// indices, expansion of state vector, etc.

		// sum quantitites
		Fnet += Fnet_i;
		M += M_i;
	}

	// --------------------------------- apply wave kinematics
	// ------------------------------------

	auto [zeta, U, Ud] = waves->getWaveKinConn(connId);
	// env->waves->getU(r, t, U); // call generic function to get water
	// velocities  <<<<<<<<<<<<<<<< all needs updating

	// --------------------------------- hydrodynamic loads
	// ----------------------------------

	// viscous drag calculation
	const vec vi = U[0] - rd; // relative water velocity
	const vec dir = vi.normalized();
	Fnet += 0.5 * env->rho_w * dir * vi.squaredNorm() * conCdA;

	// TODO <<<<<<<<< add Ud to inertia force calcuation!!

	// if (abs(r[0]) > 40)
	//{
	//	cout <<"Connection going crazy at t=" << t << endl;
	//	cout << r << endl;
	//	cout << Fnet << endl;
	//
	//	double r2 = r[0]+1;
	//	cout << r2 << endl;
	// }

	// added mass calculation
	M += conV * env->rho_w * conCa * mat::Identity();

	return MOORDYN_SUCCESS;
}

std::vector<uint64_t>
Connection::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	subdata = io::IO::Serialize(r);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(r_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(Fnet);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(M);
	data.insert(data.end(), subdata.begin(), subdata.end());

	return data;
}

uint64_t*
Connection::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	ptr = io::IO::Deserialize(ptr, r);
	ptr = io::IO::Deserialize(ptr, rd);
	ptr = io::IO::Deserialize(ptr, r_ves);
	ptr = io::IO::Deserialize(ptr, rd_ves);
	ptr = io::IO::Deserialize(ptr, Fnet);
	ptr = io::IO::Deserialize(ptr, M);

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkPolyData>
Connection::getVTK() const
{
	auto points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(r[0], r[1], r[2]);
	auto vertex = vtkSmartPointer<vtkVertex>::New();
	vertex->GetPointIds()->SetId(0, 0);
	// Node fields, i.e. r.size() number of tuples
	auto vtk_rd = io::vtk_farray("rd", 3, 1);
	vtk_rd->SetTuple3(0, rd[0], rd[1], rd[2]);
	auto vtk_M = io::vtk_farray("M", 9, 1);
	vtk_M->SetTuple9(0,
	                 M(0, 0),
	                 M(0, 1),
	                 M(0, 2),
	                 M(1, 0),
	                 M(1, 1),
	                 M(1, 2),
	                 M(2, 0),
	                 M(2, 1),
	                 M(2, 2));
	auto vtk_Fnet = io::vtk_farray("Fnet", 3, 1);
	vtk_Fnet->SetTuple3(0, Fnet[0], Fnet[1], Fnet[2]);
	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(vertex);

	auto out = vtkSmartPointer<vtkPolyData>::New();
	out->SetPoints(points);
	out->SetVerts(cells);

	out->GetPointData()->AddArray(vtk_rd);
	out->GetPointData()->AddArray(vtk_M);
	out->GetPointData()->AddArray(vtk_Fnet);
	out->GetPointData()->SetActiveVectors("Fnet");

	return out;
}

void
Connection::saveVTK(const char* filename) const
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
Connection::drawGL(void)
{
	double radius = pow(conV / (4 / 3 * pi), 0.33333); // conV
	Sphere(r[0], r[1], r[2], radius);
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
#define CHECK_CONNECTION(c)                                                    \
	if (!c) {                                                                  \
		cerr << "Null connection received in " << __FUNC_NAME__ << " ("        \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetConnectID(MoorDynConnection conn, int* id)
{
	CHECK_CONNECTION(conn);
	*id = ((moordyn::Connection*)conn)->number;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectType(MoorDynConnection conn, int* t)
{
	CHECK_CONNECTION(conn);
	*t = ((moordyn::Connection*)conn)->type;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectPos(MoorDynConnection conn, double pos[3])
{
	CHECK_CONNECTION(conn);
	moordyn::vec r, rd;
	((moordyn::Connection*)conn)->getState(r, rd);
	moordyn::vec2array(r, pos);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectVel(MoorDynConnection conn, double v[3])
{
	CHECK_CONNECTION(conn);
	moordyn::vec r, rd;
	((moordyn::Connection*)conn)->getState(r, rd);
	moordyn::vec2array(rd, v);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectForce(MoorDynConnection conn, double f[3])
{
	CHECK_CONNECTION(conn);
	moordyn::vec fnet;
	((moordyn::Connection*)conn)->getFnet(fnet);
	moordyn::vec2array(fnet, f);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectNAttached(MoorDynConnection conn, unsigned int* n)
{
	CHECK_CONNECTION(conn);
	*n = ((moordyn::Connection*)conn)->getLines().size();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetConnectAttached(MoorDynConnection conn,
                           unsigned int i,
                           MoorDynLine* l,
                           int* e)
{
	CHECK_CONNECTION(conn);
	auto attached = ((moordyn::Connection*)conn)->getLines();
	if (i >= attached.size()) {
		cerr << "Invalid line index " << i << ", just " << attached.size()
		     << " are available" << __FUNC_NAME__ << " (" << XSTR(__FILE__)
		     << ":" << __LINE__ << ")" << endl;
		return MOORDYN_INVALID_VALUE;
	}
	*l = (MoorDynLine)(attached[i].line);
	*e = (int)(attached[i].end_point);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SaveConnectVTK(MoorDynConnection conn, const char* filename)
{
#ifdef USE_VTK
	CHECK_CONNECTION(conn);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::Connection*)conn)->saveVTK(filename);
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
