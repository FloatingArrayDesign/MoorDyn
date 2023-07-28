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

#include "Point.hpp"
#include "Point.h"
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

Point::Point(moordyn::Log* log, size_t id)
  : io::IO(log)
  , seafloor(nullptr)
  , pointId(id)
{
}

Point::~Point() {}

void
Point::setup(int number_in,
             types type_in,
             vec r0_in,
             double M_in,
             double V_in,
             vec F_in,
             double CdA_in,
             double Ca_in,
             EnvCondRef env_in)
{
	// props contains:
	// Node, Type, X, Y, Z, M, V, FX, FY, FZ, CdA, Ca, env

	env = env_in; // set pointer to environment settings object
	number = number_in;
	type = type_in;

	// store passed rod properties  >>>(and convert to numbers)<<<
	pointM = M_in;
	pointV = V_in;
	pointF = F_in;
	pointCdA = CdA_in;
	pointCa = Ca_in;

	// beta = 0.0;

	// Start off at position specified in input file (will be overwritten for
	// fairleads). This is the starting point for points and the permanent
	// location of anchors.
	r = r0_in;
	rd = vec::Zero();
	r_ves = r;
	rd_ves = rd;

	// Start also the variables that will be serialized, just to avoid floating
	// point exceptions
	Fnet = vec::Zero();
	M = pointM * mat::Identity();

	LOGDBG << "   Set up Point " << number << ", type '" << TypeName(type)
	       << "'. " << endl;
}

// this function handles assigning a line to a point node
void
Point::addLine(Line* theLine, EndPoints end_point)
{
	LOGDBG << "L" << theLine->number << end_point_name(end_point) << "->P"
	       << number << " ";

	attachment a = { theLine, end_point };
	attached.push_back(a);
};

EndPoints
Point::removeLine(Line* line)
{
	EndPoints end_point;
	// look through attached lines
	for (auto it = std::begin(attached); it != std::end(attached); ++it) {
		if (it->line != line)
			continue;
		// This is the line's entry in the attachment list
		end_point = it->end_point;
		attached.erase(it);

		LOGMSG << "Detached line " << line->number << " from Point " << number
		       << endl;
		return end_point;
	}

	// line not found
	LOGERR << "Error: failed to find line to remove during "
	       << __PRETTY_FUNC_NAME__ << " call to point " << number << ". Line "
	       << line->number << endl;
	throw moordyn::invalid_value_error("Invalid line");
};

std::pair<vec, vec>
Point::initialize()
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

		const real waterDepth =
		    seafloor ? seafloor->getDepthAt(r[0], r[1]) : -env->WtrDpth;
		if (waterDepth > r[2]) {
			LOGERR << "Error: water depth is shallower than Point " << number
			       << "." << endl;
			throw moordyn::invalid_value_error("Invalid water depth");
		}
	}

	LOGDBG << "   Initialized Point " << number << endl;

	return std::make_pair(pos, vel);
};

// function to return net force on point (just to allow public reading of
// Fnet)
void
Point::getFnet(vec& Fnet_out)
{
	Fnet_out = Fnet;
};

// function to return mass matrix of point
void
Point::getM(mat& M_out)
{
	M_out = M;
};

real
Point::GetPointOutput(OutChanProps outChan)
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
	else if (outChan.QType == AccX)
		return acc[0];
	else if (outChan.QType == AccY)
		return acc[1];
	else if (outChan.QType == AccZ)
		return acc[2];
	else if (outChan.QType == Ten)
		return Fnet.norm();
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
Point::initiateStep(vec rFairIn, vec rdFairIn)
{
	if (type != COUPLED) {
		LOGERR << "Invalid Point " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid point type");
	}

	// update values to fairlead position and velocity functions
	// (function of time)
	r_ves = rFairIn;
	rd_ves = rdFairIn;

	// do I want to get precalculated values here at each FAST time step or at
	// each line time step?
};

void
Point::updateFairlead(real time)
{
	if (type != COUPLED) {
		LOGERR << "Invalid Point " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid point type");
	}

	// set fairlead position and velocity based on BCs (linear model for now)
	r = r_ves + rd_ves * time;
	rd = rd_ves;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

void
Point::setKinematics(vec r_in, vec rd_in)
{
	if (type != FIXED) {
		LOGERR << "Invalid Point " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid point type");
	}

	// set position and velocity
	r = r_in;
	rd = rd_in;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

void
Point::setState(vec pos, vec vel)
{
	// the kinematics should only be set with this function of it's an
	// independent/free point
	if (type != FREE) {
		LOGERR << "Invalid Point " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid point type");
	}

	// from state values, get r and rdot values
	r = pos;
	rd = vel;

	// pass latest kinematics to any attached lines
	for (auto a : attached)
		a.line->setEndKinematics(r, rd, a.end_point);
}

std::pair<vec, vec>
Point::getStateDeriv()
{
	// the RHS is only relevant (there are only states to worry about) if it is
	// a Point type of Point
	if (type != FREE) {
		LOGERR << "Invalid Point " << number << " type " << TypeName(type)
		       << endl;
		throw moordyn::invalid_value_error("Invalid point type");
	}

	// cout << "PointRHS: m: " << M[0][0] << ", f: " << Fnet[0] << " " <<
	// Fnet[1] << " " << Fnet[2] << endl;
	doRHS();

	// solve for accelerations in [M]{a}={f}
	acc = M.inverse() * Fnet;

	// update states
	return std::make_pair(rd, acc);
};

void
Point::getNetForceAndMass(vec6& Fnet_out, mat6& M_out, vec rBody)
{
	doRHS();

	// position of point relative to the body reference point (global
	// orientation frame)
	const vec rRel = r - rBody;

	// convert segment net force into 6dof force about body ref point
	Fnet_out(Eigen::seqN(0, 3)) = Fnet;
	Fnet_out(Eigen::seqN(3, 3)) = rRel.cross(Fnet);

	// convert segment mass matrix to 6by6 mass matrix about body ref point
	M_out = translateMass(rRel, M);
}

moordyn::error_id
Point::doRHS()
{
	// start with the Point's own forces including buoyancy and weight, and
	// its own mass
	Fnet = pointF + vec(0.0, 0.0, env->g * (pointV * env->rho_w - pointM));

	// start with physical mass
	M = pointM * mat::Identity();

	// loop through attached lines, adding force and mass contributions
	for (auto a : attached) {
		vec Fnet_i, Moment_dummy;
		mat M_i;

		// get quantities
		a.line->getEndStuff(Fnet_i, Moment_dummy, M_i, a.end_point);

		// Process outline for line failure (yet to be coded):
		// 1. check if tension (of Fnet_i) exceeds line's breaking limit or if
		// failure time has elapsed for line
		// 2. create new massless point with same instantaneous kinematics as
		// current point
		// 3. disconnect line end from current point and instead attach to
		// new point The above may require rearrangement of point
		// indices, expansion of state vector, etc.

		// sum quantitites
		Fnet += Fnet_i;
		M += M_i;
	}

	// --------------------------------- apply wave kinematics
	// ------------------------------------

	auto [zeta, U, Ud] = waves->getWaveKinPoint(pointId);
	// env->waves->getU(r, t, U); // call generic function to get water
	// velocities  <<<<<<<<<<<<<<<< all needs updating

	// --------------------------------- hydrodynamic loads
	// ----------------------------------

	// viscous drag calculation
	const vec vi = U[0] - rd; // relative water velocity
	const vec dir = vi.normalized();
	Fnet += 0.5 * env->rho_w * dir * vi.squaredNorm() * pointCdA;

	// TODO <<<<<<<<< add Ud to inertia force calcuation!!

	// if (abs(r[0]) > 40)
	//{
	//	cout <<"Point going crazy at t=" << t << endl;
	//	cout << r << endl;
	//	cout << Fnet << endl;
	//
	//	double r2 = r[0]+1;
	//	cout << r2 << endl;
	// }

	// added mass calculation
	M += pointV * env->rho_w * pointCa * mat::Identity();

	return MOORDYN_SUCCESS;
}

std::vector<uint64_t>
Point::Serialize(void)
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
Point::Deserialize(const uint64_t* data)
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
Point::getVTK() const
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
Point::saveVTK(const char* filename) const
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
Point::drawGL(void)
{
	double radius = pow(pointV / (4 / 3 * pi), 0.33333); // pointV
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
#define CHECK_POINT(c)                                                         \
	if (!c) {                                                                  \
		cerr << "Null point received in " << __FUNC_NAME__ << " ("             \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetPointID(MoorDynPoint point, int* id)
{
	CHECK_POINT(point);
	*id = ((moordyn::Point*)point)->number;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointType(MoorDynPoint point, int* t)
{
	CHECK_POINT(point);
	*t = ((moordyn::Point*)point)->type;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointPos(MoorDynPoint point, double pos[3])
{
	CHECK_POINT(point);
	moordyn::vec r, rd;
	((moordyn::Point*)point)->getState(r, rd);
	moordyn::vec2array(r, pos);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointVel(MoorDynPoint point, double v[3])
{
	CHECK_POINT(point);
	moordyn::vec r, rd;
	((moordyn::Point*)point)->getState(r, rd);
	moordyn::vec2array(rd, v);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointForce(MoorDynPoint point, double f[3])
{
	CHECK_POINT(point);
	moordyn::vec fnet;
	((moordyn::Point*)point)->getFnet(fnet);
	moordyn::vec2array(fnet, f);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointNAttached(MoorDynPoint point, unsigned int* n)
{
	CHECK_POINT(point);
	*n = ((moordyn::Point*)point)->getLines().size();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetPointAttached(MoorDynPoint point,
                         unsigned int i,
                         MoorDynLine* l,
                         int* e)
{
	CHECK_POINT(point);
	auto attached = ((moordyn::Point*)point)->getLines();
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
MoorDyn_SavePointVTK(MoorDynPoint point, const char* filename)
{
#ifdef USE_VTK
	CHECK_POINT(point);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::Point*)point)->saveVTK(filename);
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
