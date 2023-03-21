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
		// In the case where we have a flat seafloor, we shouldn't
		// actually build this object. In case we attempt to, the
		// setup function will simply return
		LOGMSG << "No seafloor file required. Assuming flat seafloor.\n";
		return;
	}

	if ((env->SeafloorMode) == moordyn::SEAFLOOR_3D) {
		LOGDBG << "Seafloor set to 3D mode.";
		const string SeafloorFilename =
		    (string)folder + "/seafloor_profile_3d.txt";
		LOGMSG << "Reading seafloor from " << SeafloorFilename << '\n';

		vector<string> fLines;  // Buffer to load file into line-by-line 
		string fLine;  // Buffer to process each line of input file 

		// Read file into buffers:
		ifstream f(SeafloorFilename);
		if (!f.is_open()) {
			LOGERR << "Cannot read the file " << SeafloorFilename << '\n';
			throw moordyn::input_file_error("Failure reading depths file");
		}

		while (getline(f, fLine)) {
			fLines.push_back(fLine);
		}
		f.close();

		if (fLines.size() < 4) {
			// Basic input checking. Input file should contain
			// 1st line indicating dimensions of x/y axes,
			// 2nd and 3rd line denoting axis tick vals,
			// and at least one line specifying a particular xyz val
			LOGERR << "The file '" << SeafloorFilename
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
		depthGrid = std::vector<std::vector<real>>(nx, std::vector<real>(ny, 0.0));

		for (unsigned int i = 3; i < fLines.size(); i++) {
			// This loop iterates all the (x,y,z) entries in the input file
			entries = moordyn::str::split(fLines[i]);
			if (entries.size() != 3) {
				LOGERR << "Line " << i + 1 << " of '" << SeafloorFilename
				       << "' should have 3 entries (x, y, z)\n";
				throw moordyn::input_file_error("Invalid file format");
			}

		}

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
