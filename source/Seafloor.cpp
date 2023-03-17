#include "Seafloor.hpp"

namespace moordyn {

Seafloor::Seafloor(moordyn::Log* log)
	: LogUser(log)
{
}

Seafloor::~Seafloor() {}

void Seafloor::setup(EnvCond* env, const char* folder)
{
	// Initialize grid lengths:
	nx = 0;
	ny = 0;

	if ((env->SeafloorMode == moordyn::SEAFLOOR_FLAT)) {
		LOGMSG << "No seafloor file\n";
		return;
	}
}

real
Seafloor::getDepthAt(real x, real y)
{
	real fx, fy;

	auto ix = interp_factor(px, x, fx);
	auto iy = interp_factor(py, y, fy);

	return interp2(depthGrid, ix, iy, fx, fy);
}

}
