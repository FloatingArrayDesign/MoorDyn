#include "Seafloor.hpp"
#include "Seafloor.h"
#include "Util/Interp.hpp"
#include <limits>

namespace moordyn {

Seafloor::Seafloor(moordyn::Log* log)
  : LogUser(log)
  , averageDepth(0)
  , minDepth(-std::numeric_limits<real>::infinity())
{
}

Seafloor::~Seafloor() {}

unsigned int
calcInsertIndex(std::vector<real>& list, real value)
{
	for (unsigned int i = 0; i < list.size(); i++) {
		if (list[i] > value) {
			return i == 0 ? 0 : i - 1;
		}
	}
	return (unsigned int)(list.size() - 1);
}
void
Seafloor::setup(EnvCondRef env, const string& filepath)
{
	// Initialize grid lengths:
	nx = 0;
	ny = 0;

	if (env->SeafloorMode == moordyn::SEAFLOOR_FLAT) {
		// In the case where we have a flat seafloor, we shouldn't
		// actually build this object. In case we attempt to, the
		// setup function will simply return
		LOGMSG << "No seafloor file required. Assuming flat seafloor.\n";
		return;
	}

	if (env->SeafloorMode == moordyn::SEAFLOOR_3D) {
		LOGDBG << "Seafloor set to 3D mode.";
		// const string SeafloorFilename =
		//     (string)folder + "seafloor_profile_3d.txt";
		LOGMSG << "Reading seafloor from " << filepath << '\n';

		vector<string> fLines; // Buffer to load file into line-by-line
		string fLine;          // Buffer to process each line of input file

		try {
			fLines = moordyn::fileIO::fileToLines(filepath);

		} catch (std::exception) {
			LOGERR << "Cannot read the file " << filepath << '\n';
			throw moordyn::input_file_error("Failure reading depths file");
		}

		if (fLines.size() < 4) {
			// Basic input checking. Input file should contain
			// 1st line indicating dimensions of x/y axes,
			// 2nd and 3rd line denoting axis tick vals,
			// and at least one line specifying a particular xyz val
			LOGERR << "The file '" << filepath
			       << "' should have at least 4 lines\n";
			throw moordyn::input_file_error("Invalid file format");
		}

		// Setup nx, ny
		vector<string> entries = moordyn::str::split(fLines[0]);
		nx = stoi(entries[0]);
		ny = stoi(entries[1]);

		// Setup px (x-axis ticks):
		entries = moordyn::str::split(fLines[1]);
		if (entries.size() != nx) {
			LOGERR << "There should be " << nx << " entries in line 2"
			       << " of the input file.\n";
			throw moordyn::input_file_error("Invalid no. of x ticks");
		}
		for (string entry : entries) {
			px.push_back(stof(entry));
		}

		// Setup py (y-axis ticks):
		entries = moordyn::str::split(fLines[2]);
		if (entries.size() != ny) {
			LOGERR << "There should be " << ny << " entries in line 3"
			       << " of the input file.\n";
			throw moordyn::input_file_error("Invalid no. of y ticks");
		}
		for (string entry : entries) {
			py.push_back(stof(entry));
		}

		// initialize depths grid (full of zeroes to begin with):
		depthGrid =
		    std::vector<std::vector<real>>(nx, std::vector<real>(ny, 0.0));

		real depthTotal = 0.0;
		for (unsigned int i = 3; i < fLines.size(); i++) {
			// This loop iterates all the (x,y,z) entries in the input file
			entries = moordyn::str::split(fLines[i]);
			if (entries.size() != 3) {
				LOGERR << "Line " << i + 1 << " of '" << filepath
				       << "' should have 3 entries (x, y, z)\n";
				throw moordyn::input_file_error("Invalid file format");
			}

			real xPos = stof(entries[0]);
			real yPos = stof(entries[1]);
			real depth = stof(entries[2]);
			unsigned int xIdx = calcInsertIndex(px, xPos);
			unsigned int yIdx = calcInsertIndex(py, yPos);
			depthGrid[xIdx][yIdx] = depth;
			depthTotal += depth;
			if (depth > minDepth) {
				minDepth = depth;
			}
		}
		averageDepth = depthTotal / (real)(nx * ny);

	} else {
		// Handle case where we specified an inappropriate flag
		// for seafloor depths:
		LOGERR << "ERROR: Invalid Seafloor depth mode specified. "
		       << "EXITING.";
		throw moordyn::invalid_value_error("Invalid depth mode\n.");
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

#define CHECK_SEAFLOOR(s)                                                      \
	if (!s) {                                                                  \
		cerr << "Null seafloor instance received in " << __FUNC_NAME__ << " (" \
		     << XSTR(__FILE__) << ":" << __LINE__ << ")" << endl;              \
		return MOORDYN_INVALID_VALUE;                                          \
	}

int DECLDIR
MoorDyn_GetDepthAt(MoorDynSeafloor seafloor, double x, double y, double* depth)
{
	CHECK_SEAFLOOR(seafloor);
	*depth = ((moordyn::Seafloor*)seafloor)->getDepthAt(x, y);
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetAverageDepth(MoorDynSeafloor seafloor, double* avgDepth)
{
	CHECK_SEAFLOOR(seafloor);
	*avgDepth = ((moordyn::Seafloor*)seafloor)->getAverageDepth();
	return MOORDYN_SUCCESS;
}

int DECLDIR
MoorDyn_GetMinDepth(MoorDynSeafloor seafloor, double* minDepth)
{
	CHECK_SEAFLOOR(seafloor);
	*minDepth = ((moordyn::Seafloor*)seafloor)->getMinimumDepth();
	return MOORDYN_SUCCESS;
}
