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

#include "Body.hpp"
#include "Body.h"
#include "Point.hpp"
#include "Rod.hpp"
#include "Waves.hpp"
#include <tuple>

#ifdef USE_VTK
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkCellData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSTLReader.h>
#endif

using namespace std;

namespace moordyn {

Body::Body(moordyn::Log* log, size_t id)
  : io::IO(log)
  , bodyId(id)
{
#ifdef USE_VTK
	defaultVTK();
#endif
}

Body::~Body() {}

void
Body::setup(int number_in,
            types type_in,
            vec6 r6_in,
            vec rCG_in,
            real M_in,
            real V_in,
            vec I_in,
            vec6 CdA_in,
            vec6 Ca_in,
            EnvCondRef env_in,
            shared_ptr<ofstream> outfile_pointer)
{
	env = env_in; // set pointer to environment settings object
	number = number_in;
	type = type_in;
	outfile = outfile_pointer.get(); // make outfile point to the right place

	if (type == FREE) {
		bodyM = M_in;
		bodyV = V_in;

		body_r6.head<3>() = r6_in.head<3>();
		body_r6.tail<3>() = deg2rad * r6_in.tail<3>();
		body_rCG = rCG_in;
		bodyI = I_in;
		bodyCdA = CdA_in;
		bodyCa = Ca_in;
	} else if (type == FIXED){ // fixed bodies have no need for these variables other than position...
		bodyM = 0.0;
		bodyV = 0.0;
		body_r6.head<3>() = r6_in.head<3>();
		body_r6.tail<3>() = deg2rad * r6_in.tail<3>();
		bodyI = vec::Zero();
		bodyCdA = vec6::Zero();
		bodyCa = vec6::Zero();
    } else // coupled bodies have no need for these variables...
	{
		bodyM = 0.0;
		bodyV = 0.0;

		body_r6 = vec6::Zero();
		body_rCG = vec::Zero();
		bodyI = vec::Zero();
		bodyCdA = vec6::Zero();
		bodyCa = vec6::Zero();
	}

	attachedP.clear();
	attachedR.clear();
	rPointRel.clear();
	r6RodRel.clear();

	// set up body initial mass matrix (excluding any rods or attachements)
	mat6 Mtemp = mat6::Zero();
	Mtemp(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = mat::Identity() * bodyM;
	Mtemp(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = bodyI.asDiagonal();
	// account for potential CG offset <<< is the direction right? <<<
	M0 = translateMass6(body_rCG, Mtemp);

	// add added mass in each direction about ref point (so only diagonals)
	M0 += bodyV * bodyCa.asDiagonal() *
	      env->rho_w; // Values are only non-zero if free body

	// --------------- if this is an independent body (not coupled) ----------
	// set initial position and orientation of body from input file
	r7.pos = body_r6.head<3>();
	r7.quat = Euler2Quat(body_r6.tail<3>());

	v6 = vec6::Zero();

	// calculate orientation matrix based on latest angles
	OrMat = r7.quat.toRotationMatrix();

	LOGDBG << "Set up Body " << number << ", type " << TypeName(type) << ". "
	       << endl;
};

void
Body::addPoint(moordyn::Point* point, vec coords)
{
	LOGDBG << "P" << point->number << "->B" << number << " " << endl;

	// store Point address
	attachedP.push_back(point);

	// store Point relative location
	rPointRel.push_back(coords);
};

void
Body::addRod(Rod* rod, vec6 coords)
{
	LOGDBG << "R" << rod->number << "->B" << number << " " << endl;

	// store Rod address
	attachedR.push_back(rod);

	// store Rod end A relative position and unit vector from end A to B
	vec tempUnitVec;
	unitvector(
	    tempUnitVec, coords(Eigen::seqN(0, 3)), coords(Eigen::seqN(3, 3)));
	vec6 r6Rod;
	r6Rod(Eigen::seqN(0, 3)) = coords(Eigen::seqN(0, 3));
	r6Rod(Eigen::seqN(3, 3)) = tempUnitVec;
	r6RodRel.push_back(r6Rod);
};

void
Body::initializeUnfreeBody(vec6 r6_in, vec6 v6_in)
{
	if (type == FREE) {
		LOGERR << "Invalid initializator for a FREE body" << endl;
		throw moordyn::invalid_value_error("Invalid body type");
	}
	initiateStep(r6_in, v6_in);
	updateFairlead(0.0);

	// If any Rod is fixed to the body (not pinned), initialize it now because
	// otherwise it won't be initialized
	for (auto attached : attachedR)
		if (attached->type == Rod::FIXED)
			attached->initialize();
	// If there's an attached Point, initialize it now because it won't be
	// initialized otherwise
	for (auto attached : attachedP)
		attached->initialize();
}

std::pair<XYZQuat, vec6>
Body::initialize()
{
	if (type != FREE) {
		LOGERR << "Invalid initializator for a non FREE body ("
		       << TypeName(type) << ")" << endl;
		throw moordyn::invalid_value_error("Invalid body type");
	}

	// set positions of any dependent points and rods now (before they are
	// initialized)
	setDependentStates();

	// If any Rod is fixed to the body (not pinned), initialize it now because
	// otherwise it won't be initialized
	for (auto attached : attachedR)
		if (attached->type == Rod::FIXED)
			attached->initialize();
	// If there's an attached Point, initialize it now because it won't be
	// initialized otherwise
	for (auto attached : attachedP)
		attached->initialize();

	// create output file for writing output (and write channel header and units
	// lines) if applicable

	if (outfile) // check it's not null.  Null signals no individual line output
	             // files
	{
		if (!outfile->is_open()) {
			LOGERR << "Unable to write file Body" << number << ".out" << endl;
			throw moordyn::output_file_error("Invalid line file");
		}
		// ------------- write channel names line --------------------

		// output time
		*outfile << "Time"
		         << "\t ";

		*outfile << "x\ty\tz\troll\tpitch\tyaw";

		*outfile << "\n";

		// ----------- write units line ---------------

		if (env->WriteUnits > 0) {
			// output time
			*outfile << "(s)"
			         << "\t ";

			*outfile << "(m)\t(m)\t(m)\t(deg)\t(deg)\t(deg)";

			*outfile << "\n"; // should also write units at some point!
		}
	}

	LOGDBG << "Initialized Body " << number << endl;

	return std::make_pair(r7, v6);
};

void
Body::setDependentStates()
{
	// set kinematics of any dependent points (this is relevant for the
	// dependent lines, yeah?)
	for (unsigned int i = 0; i < attachedP.size(); i++) {
		// this is making a "fake" state vector for the point, describing its
		// position (rPoint) and velocity (rdPoint)
		vec rPoint, rdPoint;

		// Get point location from rPointRel for ith connected Point
		// and calculate the kinematics of that point based on the
		// kinematics of the Body:
		transformKinematics(rPointRel[i],
		                    OrMat,
		                    r7.pos,
		                    v6,
		                    rPoint,
		                    rdPoint); //<<< should double check this function

		// pass above to the point and get it to calculate the forces
		try {
			attachedP[i]->setKinematics(rPoint, rdPoint);
		} catch (moordyn::invalid_value_error& exception) {
			// Just rethrow the exception
			throw;
		}
	}

	// set kinematics of any dependent Rods
	for (unsigned int i = 0; i < attachedR.size(); i++) {
		// calculate displaced coordinates/orientation and velocities of each
		// rod <<<<<<<<<<<<<
		// this is making a "fake" state vector for the rod, describing its
		// position and velocity
		vec6 rRod, rdRod;

		// do 3d details of Rod ref point
		vec tmpr, tmprd;
		transformKinematics(r6RodRel[i](Eigen::seqN(0, 3)),
		                    OrMat,
		                    r7.pos,
		                    v6,
		                    tmpr,
		                    tmprd); // set first three entires (end A
		                            // translation) of rRod and rdRod
		// does the above function need to take in all 6 elements of r6RodRel??
		rRod(Eigen::seqN(0, 3)) = tmpr;
		rdRod(Eigen::seqN(0, 3)) = tmprd;

		// rotate rod relative unit vector by OrMat to get unit vec in reference
		// coords
		rRod(Eigen::seqN(3, 3)) = OrMat * r6RodRel[i](Eigen::seqN(3, 3));

		// do rotational stuff
		// is this okay as is?
		rdRod(Eigen::seqN(3, 3)) = v6(Eigen::seqN(3, 3));

		// pass above to the rod and get it to calculate the forces
		// rRod = [baseX, baseY, baseZ, dirX, dirY, dirZ]
		// rdRod = [baseDX, baseDY, baseDZ, endDX, endDY, endDZ]
		try {
			attachedR[i]->setKinematics(rRod, rdRod);
		} catch (moordyn::invalid_value_error& exception) {
			// Just rethrow the exception
			throw;
		}
	}
}

real
Body::GetBodyOutput(OutChanProps outChan)
{

	vec3 rotations = rad2deg * Quat2Euler(r7.quat);

	if (outChan.QType == PosX)
		return r7.pos.x();
	else if (outChan.QType == PosY)
		return r7.pos.y();
	else if (outChan.QType == PosZ)
		return r7.pos.z();
	else if (outChan.QType == RX)
		return rotations[0];
	else if (outChan.QType == RY)
		return rotations[1];
	else if (outChan.QType == RZ)
		return rotations[2];
	else if (outChan.QType == VelX)
		return v6[0];
	else if (outChan.QType == VelY)
		return v6[1];
	else if (outChan.QType == VelZ)
		return v6[2];
	else if (outChan.QType == RVelX)
		return v6[3] * 180.0 / pi;
	else if (outChan.QType == RVelY)
		return v6[4] * 180.0 / pi;
	else if (outChan.QType == RVelZ)
		return v6[5] * 180.0 / pi;
	else if (outChan.QType == AccX)
		return a6[0];
	else if (outChan.QType == AccY)
		return a6[1];
	else if (outChan.QType == AccZ)
		return a6[2];
	else if (outChan.QType == RAccX)
		return a6[3] * 180.0 / pi;
	else if (outChan.QType == RAccY)
		return a6[4] * 180.0 / pi;
	else if (outChan.QType == RAccZ)
		return a6[5] * 180.0 / pi;
	else if (outChan.QType == Ten)
		return sqrt(F6net[0] * F6net[0] + F6net[1] * F6net[1] +
		            F6net[2] * F6net[2]);
	else if (outChan.QType == FX)
		return F6net[0];
	else if (outChan.QType == FY)
		return F6net[1];
	else if (outChan.QType == FZ)
		return F6net[2];
	else if (outChan.QType == MX)
		return F6net[3];
	else if (outChan.QType == MY)
		return F6net[4];
	else if (outChan.QType == MZ)
		return F6net[5];
	else {
		LOGWRN << "Unrecognized output channel " << outChan.QType << endl;
		return 0.0;
	}
}

// called at the beginning of each coupling step to update the boundary
// conditions (body kinematics) for the proceeding time steps
void
Body::initiateStep(vec6 r, vec6 rd)
{
	if (type == COUPLED) // if coupled, update boundary conditions
	{
		r_ves = r;
		rd_ves = rd;
		return;
	}
	if (type == FIXED) // if fixed body, set the BCs to stationary
	{
		if (bodyId == 0) r_ves = vec6::Zero(); // special ground body case
		else r_ves = r;
		rd_ves = vec6::Zero();
		return;
	}
	LOGERR << "Body " << number << "is not of type COUPLED or FIXED." << endl;
	throw moordyn::invalid_value_error("Invalid body type");
}

void
Body::updateFairlead(real time)
{
	if ((type == COUPLED) || (type == FIXED)) // if coupled OR GROUND BODY
	{
		// set Body kinematics based on BCs (linear model for now)
		r7 = XYZQuat::fromVec6(r_ves + rd_ves * time);
		v6 = rd_ves;

		// calculate orientation matrix based on latest angles
		OrMat = r7.quat.toRotationMatrix();

		// set positions of any dependent points and rods
		setDependentStates();

		return;
	}
	LOGERR << "The body is not a coupled/fixed one" << endl;
	throw moordyn::invalid_value_error("Invalid body type");
}

void
Body::setState(XYZQuat pos, vec6 vel)
{
	// set position and velocity vectors from state vector
	r7 = pos;
	v6 = vel;

	// calculate orientation matrix based on latest angles
	OrMat = r7.quat.toRotationMatrix();

	// set positions of any dependent points and rods
	setDependentStates();
}

std::pair<XYZQuat, vec6>
Body::getStateDeriv()
{
	if (type != FREE) {
		LOGERR << "The body is not a free one" << endl;
		throw moordyn::invalid_value_error("Invalid body type");
	}

	// Get contributions from attached points (and lines attached to
	// them) and store in FNet:
	doRHS();

	// solve for accelerations in [M]{a}={f}
	a6 = solveMat6(M, F6net);

	// NOTE; is the above still valid even though it includes rotational DOFs?
	dPos.pos = v6.head<3>();
	// this assumes that the angular velocity is about the global coordinates
	// which is true for bodies
	dPos.quat = 0.5 * (quaternion(0.0, v6[3], v6[4], v6[5]) * r7.quat).coeffs();
	return std::make_pair(dPos, a6);
};

//  this is the big function that calculates the forces on the body
void
Body::doRHS()
{
	// TODO: somewhere should check for extreme orientation changes, i.e.
	// "winding" close to 2pi, and maybe prevent it if it risks compromising
	// angle assumptions <<<<<<

	// clear Mass and Force matrix before re-summing
	F6net = vec6::Zero();
	M = mat6::Zero();

	// First, the body's own mass matrix must be adjusted based on its
	// orientation so that we have a mass matrix in the global orientation frame
	M = rotateMass6(OrMat, M0);

	// gravity forces and moments about body ref point given CG location
	const vec body_rCGrotated = OrMat * body_rCG;
	// weight+buoyancy vector
	const vec Fgrav =
	    vec(0.0, 0.0, bodyV * env->rho_w * env->g - bodyM * env->g);
	F6net(Eigen::seqN(0, 3)) = Fgrav;
	F6net(Eigen::seqN(3, 3)) = body_rCGrotated.cross(Fgrav);

	// --------------------------------- apply wave kinematics
	// ------------------------------------

	// env->waves->getU(r6, t, U); // call generic function to get water
	// velocities <<<<<<<<< all needs updating
	auto [zeta, U, Ud] = waves->getWaveKinBody(bodyId);

	// ------------------------------------------------------------------------------------------

	// viscous drag calculation (on core body)

	// relative water velocity
	// for rotation, this is just the negative of the body's rotation for now
	// (not allowing flow rotation)
	vec6 vi = -v6;
	vi(Eigen::seqN(0, 3)) += U[0];

	// Rotational DOFs drag coefficients are also defined on bodyCdA
	vec6 cda;
	cda(Eigen::seqN(0, 3)) = OrMat.transpose() * bodyCdA.head<3>();
	cda(Eigen::seqN(3, 3)) = OrMat.transpose() * bodyCdA.tail<3>();
	F6net +=
	    0.5 * env->rho_w * vi.cwiseProduct(vi.cwiseAbs()).cwiseProduct(cda);

	// Get contributions from any points attached to the body
	for (auto attached : attachedP) {
		// get net force and mass from Point on body ref point (global
		// orientation)
		vec6 F6_i;
		mat6 M6_i;
		attached->getNetForceAndMass(F6_i, M6_i, r7.pos);

		// sum quantitites
		F6net += F6_i;
		M += M6_i;
	}

	// Get contributions from any rods that are part of the body
	for (auto attached : attachedR) {
		// get net force and mass from Rod on body ref point (global
		// orientation)
		vec6 F6_i;
		mat6 M6_i;
		attached->getNetForceAndMass(F6_i, M6_i, r7.pos);

		//			// calculate relative location of rod about body center in
		// global orientation 			double rRod_i[3];
		// rotateVector3(r6Rod[c], OrMat, rRod_i);   // this will only consider
		// 3d position of rod (not orientation)
		//
		//			// convert force into 6dof force based on rod position
		//			double F6_i;  // 6dof force-moment from rod about body ref
		// point (but global orientation frame of course)
		// translateForce6DOF(rRod_i, Fnet_i, F6_i);
		//
		//			// transform mass matrix to 6dof one about body center
		//			double M6_i[6][6];
		//			translateMassInertia3to6DOF(M_i, I_i, rRod_i, M6_i);
		//<<<<<<<<<

		// sum quantitites
		F6net += F6_i;
		M += M6_i;
	}

	return;
}

// write output file for body
void
Body::Output(real time)
{
	if (outfile) // if not a null pointer (indicating no output)
	{
		if (!outfile->is_open()) {
			LOGWRN << "Unable to write to output file " << endl;
			return;
		}
		// output time
		*outfile << time << "\t ";

		for (int J = 0; J < 3; J++)
			*outfile << r7.pos[J] << "\t ";

		vec3 angles = rad2deg * Quat2Euler(r7.quat);
		*outfile << angles[0] << "\t " << angles[1] << "\t " << angles[2]
		         << "\n";
	}
	return;
};

std::vector<uint64_t>
Body::Serialize(void)
{
	std::vector<uint64_t> data, subdata;

	subdata = io::IO::Serialize(r7);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(v6);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(r_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(rd_ves);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(F6net);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(M);
	data.insert(data.end(), subdata.begin(), subdata.end());
	subdata = io::IO::Serialize(OrMat);
	data.insert(data.end(), subdata.begin(), subdata.end());

	return data;
}

uint64_t*
Body::Deserialize(const uint64_t* data)
{
	uint64_t* ptr = (uint64_t*)data;
	ptr = io::IO::Deserialize(ptr, r7);
	ptr = io::IO::Deserialize(ptr, v6);
	ptr = io::IO::Deserialize(ptr, r_ves);
	ptr = io::IO::Deserialize(ptr, rd_ves);
	ptr = io::IO::Deserialize(ptr, F6net);
	ptr = io::IO::Deserialize(ptr, M);
	ptr = io::IO::Deserialize(ptr, OrMat);

	return ptr;
}

#ifdef USE_VTK
vtkSmartPointer<vtkPolyData>
Body::getVTK() const
{
	auto transform = vtkSmartPointer<vtkTransform>::New();
	// default behavior is for vtkTransform to internally concatenate transform
	// like M' = M * A where A is the additional transform specified, which is
	// the opposite order from what we expect (M' = A * M, so A happens last)
	transform->PostMultiply();
	// The VTK object is already centered on 0,0,0, so we can rotate it
	// Going through angle axis is both faster from quaternion, and avoids
	// any issues of euler angle conventions
	Eigen::AngleAxis<real> angleAxis(r7.quat);
	// vtk uses degrees
	real angle = rad2deg * angleAxis.angle();
	transform->RotateWXYZ(angle, angleAxis.axis().data());

	// And then we can move it to the appropriate position
	vec3 pos = r7.pos;
	transform->Translate(pos.x(), pos.y(), pos.z());

	auto transformer = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformer->SetInputData(vtk_body);
	transformer->SetTransform(transform);
	transformer->Update();

	vtkSmartPointer<vtkPolyData> out = transformer->GetOutput();

	return out;
}

void
Body::saveVTK(const char* filename) const
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

void
Body::defaultVTK()
{
	vtk_body = vtkSmartPointer<vtkPolyData>::New();
	auto points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0, 0, 0);
	points->InsertNextPoint(1, 0, 0);
	points->InsertNextPoint(0, 1, 0);
	points->InsertNextPoint(0, 0, 1);
	auto x_axis = vtkSmartPointer<vtkLine>::New();
	x_axis->GetPointIds()->SetId(0, 0);
	x_axis->GetPointIds()->SetId(1, 1);
	auto y_axis = vtkSmartPointer<vtkLine>::New();
	y_axis->GetPointIds()->SetId(0, 0);
	y_axis->GetPointIds()->SetId(1, 2);
	auto z_axis = vtkSmartPointer<vtkLine>::New();
	z_axis->GetPointIds()->SetId(0, 0);
	z_axis->GetPointIds()->SetId(1, 3);

	auto axes = io::vtk_carray("axis", 1, 3);
	axes->SetTuple1(0, 'x');
	axes->SetTuple1(1, 'y');
	axes->SetTuple1(2, 'z');

	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(x_axis);
	cells->InsertNextCell(y_axis);
	cells->InsertNextCell(z_axis);

	vtk_body->SetPoints(points);
	vtk_body->SetLines(cells);

	vtk_body->GetCellData()->AddArray(axes);
	vtk_body->GetCellData()->SetActiveScalars("axis");
}
#endif

// new function to draw instantaneous line positions in openGL context
#ifdef USEGL
void
Body::drawGL(void)
{
	double radius = pow(BodyV / (4 / 3 * pi), 0.33333); // pointV
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

/// Check that the provided body is not Null
#define CHECK_BODY(s)                                                          \
	if (!s) {                                                                  \
		cerr << "Null body received in " << __FUNC_NAME__ << " ("              \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetBodyID(MoorDynBody b, int* id)
{
	CHECK_BODY(b);
	*id = ((moordyn::Body*)b)->number;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetBodyType(MoorDynBody b, int* t)
{
	CHECK_BODY(b);
	*t = ((moordyn::Body*)b)->type;
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetBodyState(MoorDynBody b, double r[6], double rd[6])
{
	CHECK_BODY(b);
	moordyn::XYZQuat pos;
	moordyn::vec6 vel;
	std::tie(pos, vel) = ((moordyn::Body*)b)->getState();
	moordyn::vec62array(pos.toVec6(), r);
	moordyn::vec62array(vel, rd);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_SaveBodyVTK(MoorDynBody b, const char* filename)
{
#ifdef USE_VTK
	CHECK_BODY(b);
	moordyn::error_id err = MOORDYN_SUCCESS;
	string err_msg;
	try {
		((moordyn::Body*)b)->saveVTK(filename);
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

int DECLDIR
MoorDyn_UseBodyVTK(MoorDynBody b, const char* filename)
{
#ifdef USE_VTK
	CHECK_BODY(b);

	vtkSmartPointer<vtkPolyData> model;
	std::string ext =
	    moordyn::str::lower(moordyn::str::split(filename, '.').back());
	moordyn::error_id err = MOORDYN_SUCCESS;
	if (ext == "vtp") {
		auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
		reader->SetFileName(filename);
		reader->Update();
		err = moordyn::io::vtk_error(reader->GetErrorCode());
		if (err == MOORDYN_SUCCESS)
			model = reader->GetOutput();
	} else if (ext == "stl") {
		auto reader = vtkSmartPointer<vtkSTLReader>::New();
		reader->SetFileName(filename);
		reader->Update();
		err = moordyn::io::vtk_error(reader->GetErrorCode());
		if (err == MOORDYN_SUCCESS)
			model = reader->GetOutput();
	} else {
		cerr << "Unrecognized file format in " << __FUNC_NAME__ << " ("
		     << XSTR(__FILE__) << ":" << __LINE__ << "). Cannot load the file '"
		     << filename << "'" << endl;
		return MOORDYN_INVALID_INPUT_FILE;
	}

	if (err != MOORDYN_SUCCESS) {
		cerr << "VTK reported an error while reading the file '" << filename
		     << "'in " << __FUNC_NAME__ << " (" << XSTR(__FILE__) << ":"
		     << __LINE__ << ")" << endl;
		return err;
	}

	string err_msg;
	try {
		((moordyn::Body*)b)->setVTK(model);
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
