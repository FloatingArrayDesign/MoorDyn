/** @file wave_surface.cpp
 * An example of running a model and saving the structures and wave surface to
 * vtk files
 *
 * Expects there to be a vtk_out/ folder in the base MoorDyn folder
 */
#include "MoorDyn2.h"
#include "MoorDyn2.hpp"
#include "MoorDynAPI.h"
#include "Waves.hpp"
#include "Misc.hpp"
#include <cstddef>
#include <string.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include "Waves/SpectrumKin.hpp"
#include "Waves/WaveSpectrum.hpp"
#include "util.h"

#include <sstream>
#include <filesystem>
#include <string_view>
#include <unordered_map>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkXMLDataSetWriter.h>
#include <vtkRectilinearGrid.h>

using namespace std;

bool
write_individual_vtk(moordyn::MoorDyn& system,
                     double time,
                     SeriesWriter& series_writer)
{
	int body_i = 1;
	for (auto& body : system.GetBodies()) {
		std::string filepath = "../../vtk_out/";
		std::stringstream filename;
		std::stringstream element_name;
		element_name << "vtk_body_" << body_i;
		auto& vtp_series = series_writer.getSeries(element_name.str());
		auto step_num = vtp_series.time_steps.size();

		filename << element_name.str() << "." << step_num << ".vtp";
		std::string full_path = filepath + filename.str();
		// std::cout << "***     Saving on '" << full_path << "'..." <<
		// std::endl;
		vtp_series.time_steps.push_back({ filename.str(), time });
		body->saveVTK(full_path.c_str());
		body_i++;
	}

	int rod_i = 1;
	for (auto& rod : system.GetRods()) {
		std::string filepath = "../../vtk_out/";
		std::stringstream filename;
		std::stringstream element_name;
		element_name << "vtk_rod_" << rod_i;
		auto& vtp_series = series_writer.getSeries(element_name.str());
		auto step_num = vtp_series.time_steps.size();

		filename << element_name.str() << "." << step_num << ".vtp";
		std::string full_path = filepath + filename.str();
		// std::cout << "***     Saving on '" << full_path << "'..." <<
		// std::endl;
		vtp_series.time_steps.push_back({ filename.str(), time });

		rod->saveVTK(full_path.c_str());
		rod_i++;
	}

	int point_i = 1;
	for (auto& point : system.GetPoints()) {
		std::string filepath = "../../vtk_out/";
		std::stringstream filename;
		std::stringstream element_name;
		element_name << "vtk_point_" << point_i;
		auto& vtp_series = series_writer.getSeries(element_name.str());
		auto step_num = vtp_series.time_steps.size();

		filename << element_name.str() << "." << step_num << ".vtp";
		std::string full_path = filepath + filename.str();
		// std::cout << "***     Saving on '" << full_path << "'..." <<
		// std::endl;
		vtp_series.time_steps.push_back({ filename.str(), time });
		point->saveVTK(full_path.c_str());
		point_i++;
	}

	int line_i = 1;
	for (auto& line : system.GetLines()) {
		std::string filepath = "../../vtk_out/";
		std::stringstream filename;
		std::stringstream element_name;
		element_name << "vtk_line_" << line_i;
		auto& vtp_series = series_writer.getSeries(element_name.str());
		auto step_num = vtp_series.time_steps.size();

		filename << element_name.str() << "." << step_num << ".vtp";
		std::string full_path = filepath + filename.str();
		// std::cout << "***     Saving on '" << full_path << "'..." <<
		// std::endl;
		vtp_series.time_steps.push_back({ filename.str(), time });
		line->saveVTK(full_path.c_str());
		line_i++;
	}
	return true;
}

vtkSmartPointer<vtkDataSet>
getWaveSurfaceVTK(Eigen::ArrayXd x_grid,
                  Eigen::ArrayXd y_grid,
                  moordyn::Waves* waves)
{
	auto xcoords = vtkSmartPointer<vtkFloatArray>::New();
	for (auto x : x_grid) {
		xcoords->InsertNextValue(x);
	}
	auto ycoords = vtkSmartPointer<vtkFloatArray>::New();
	for (auto y : y_grid) {
		ycoords->InsertNextValue(y);
	}

	auto zcoord = vtkSmartPointer<vtkFloatArray>::New();
	zcoord->InsertNextValue(0.0);

	auto nx = x_grid.size();
	auto ny = y_grid.size();
	auto grid = vtkSmartPointer<vtkRectilinearGrid>::New();
	grid->SetDimensions(nx, ny, 1);
	grid->SetXCoordinates(xcoords);
	grid->SetYCoordinates(ycoords);
	grid->SetZCoordinates(zcoord);

	auto vtk_zeta = moordyn::io::vtk_farray("Zeta", 1, nx * ny);
	for (unsigned int z = 0; z < 1; z++) {
		for (unsigned int y = 0; y < ny; y++) {
			for (unsigned int x = 0; x < nx; x++) {
				auto tupleIdx = x + y * nx + z * (nx * ny);
				vtk_zeta->SetTuple1(tupleIdx,
				                    waves->getWaveHeightPoint(
				                        moordyn::vec2(x_grid[x], y_grid[y])));
			}
		}
	}

	grid->GetPointData()->AddArray(vtk_zeta);
	return grid;
}

void
save_wave_vtp(moordyn::Waves* waves, double time, SeriesWriter& series_writer)
{
	auto x_grid = Eigen::ArrayXd::LinSpaced(111, -100.0, 10.0);
	auto y_grid = Eigen::ArrayXd::LinSpaced(101, -50.0, 50.0);
	auto obj = getWaveSurfaceVTK(x_grid, y_grid, waves);
	std::string filepath = "../../vtk_out/";
	std::stringstream filename;
	std::stringstream element_name;
	element_name << "wave_elev";
	auto& vtp_series = series_writer.getSeries(element_name.str());
	auto step_num = vtp_series.time_steps.size();

	filename << element_name.str() << "." << step_num << ".vtr";
	std::string full_path = filepath + filename.str();
	// std::cout << "***     Saving on '" << full_path << "'..." <<
	// std::endl;
	vtp_series.time_steps.push_back({ filename.str(), time });
	auto writer = vtkSmartPointer<vtkXMLDataSetWriter>::New();
	writer->SetFileName(full_path.c_str());
	writer->SetInputData(obj);
	writer->SetDataModeToBinary();
	writer->Update();
	writer->Write();
	auto err = moordyn::io::vtk_error(writer->GetErrorCode());
	if (err != MOORDYN_SUCCESS) {
		std::cerr << "VTK reported an error while writing the VTP file '"
		          << filename.str() << "'" << endl;
		MOORDYN_THROW(err, "vtkXMLPolyDataWriter reported an error");
	}
}
/** @brief Runs a simulation
 *
 * @return true if the test is passed, false if problems are detected
 */
bool
api()
{
	moordyn::MoorDyn system("Mooring/wavekin_2/wavekin_2.txt");

	unsigned int n_dof;
	n_dof = system.NCoupledDOF();

	std::vector<double> x(n_dof, 0.0);
	std::vector<double> dx(n_dof, 0.0);
	auto err = system.Init(x.data(), dx.data());
	if (err != MOORDYN_SUCCESS) {
		cout << "failed to init surface waves" << endl;
		return false;
	}

	SeriesWriter series_writer;
	if (!write_individual_vtk(system, 0.0, series_writer)) {
		return false;
	}
	save_wave_vtp(system.GetWaves().get(), 0.0, series_writer);

	// Integrate in time
	const double t_max = 30;
	double t = 0.0, dt = 0.1;
	double f[3];
	while (t < t_max) {
		system.Step(x.data(), dx.data(), f, t, dt);

		if (!write_individual_vtk(system, t, series_writer)) {
			return false;
		}
		save_wave_vtp(system.GetWaves().get(), t, series_writer);
		cout << "Time: " << t << endl;
	}

	series_writer.writeJson("../../vtk_out/");

	return true;
}

/** @brief Runs all the test
 * @return 0 if the tests have ran just fine, 1 otherwise
 */
int
main(int, char**)
{
	if (!api())
		return 1;

	return 0;
}
