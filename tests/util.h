#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include "Misc.hpp"

#ifdef USE_VTK

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
namespace fs = std::filesystem;

struct VtpSeries
{
	std::string version = "1.0";
	std::vector<std::pair<std::string, double>> time_steps;

	std::string getJson() const
	{
		std::stringstream ss;
		ss << "{";
		ss << "\"file-series-version\": "
		   << "\"" << version << "\",";
		ss << "\"files\": [";

		for (const auto& step : time_steps) {
			ss << "{\"name\": \"" << step.first
			   << "\", \"time\":" << step.second << "},";
		}
		ss << "]";
		ss << "}";

		return ss.str();
	}
};

struct SeriesWriter
{
	std::unordered_map<std::string, VtpSeries> series;
	VtpSeries& getSeries(std::string s)
	{
		if (series.count(s) == 0) {
			series[s] = VtpSeries{};
		}
		return series[s];
	}

	void writeJson(std::string path)
	{

		for (const auto& [element_name, series] : series) {
			auto example_filename = series.time_steps.front().first;
			auto ext = fs::path(example_filename).extension().string();
			std::string filename = path + element_name + "." + ext + ".series";
			std::ofstream file;
			file.open(filename);
			file << series.getJson();
			file.close();
		}
	}
};

#endif

template<typename T>
std::vector<size_t>
get_shape(T* data)
{
	std::vector<size_t> a;
	return a;
}
template<typename T>
std::vector<size_t>
get_shape(std::vector<T>* data)
{
	std::vector<size_t> sub_shape = get_shape(data->data());
	sub_shape.insert(sub_shape.cbegin(), data->size());
	return sub_shape;
}

template<typename T>
void
save_vec(std::ofstream& file, std::vector<T>* data)
{
	auto count = data->size() * sizeof(T);
	file.write(reinterpret_cast<const char*>(data->data()), count);
}
template<typename T>
void
save_vec(std::ofstream& file, std::vector<std::vector<T>>* data)
{
	for (int i = 0; i < data->size(); i++) {
		std::vector<T>* entry = &data->at(i);
		save_vec(file, entry);
	}
}

template<typename T>
void
multidim_arr_to_file(std::string filename, T* thing)
{
	std::vector<size_t> shape = get_shape(thing);

	// cout << "shape of arr is: " << moordyn::write_to_text(shape) << endl;
	std::ofstream file;
	file.open(filename, std::ios::out | std::ios::binary);
	if (!file) {
		std::cout << "Cannot open file " << filename << std::endl;
	}
	size_t size = shape.size();
	file.write(reinterpret_cast<const char*>(&size), sizeof(size_t));
	for (size_t dim : shape) {
		file.write(reinterpret_cast<const char*>(&dim), sizeof(size_t));
		// cout << "write dimension = " << dim << "\n";
	}
	save_vec(file, thing);
	file.close();
}
// template<typename T>
// void save_to_file(std::string filename, T* thing) {
//     std::ofstream file;
// 	file.open(filename);
// 	file << moordyn::write_to_text(*thing);
// 	file.close();

// }

template<typename DerivedA, typename DerivedB>
bool
allclose(const Eigen::DenseBase<DerivedA>& a,
         const Eigen::DenseBase<DerivedB>& b,
         const typename DerivedA::RealScalar& rtol =
             Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
         const typename DerivedA::RealScalar& atol =
             Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
	return ((a.derived() - b.derived()).array().abs() <=
	        (atol + rtol * b.derived().array().abs()))
	    .all();
}

bool
isclose(const double& a,
        const double& b,
        const double& rtol = 0.0,
        const double& atol = std::numeric_limits<double>::epsilon())
{
	return std::abs(a - b) <= (atol + rtol * std::abs(b));
}

#ifdef USE_CATCH

#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_tostring.hpp"
#include "catch2/matchers/catch_matchers_templated.hpp"

namespace Catch {

template<typename T, int N>
struct StringMaker<Eigen::Vector<T, N>>
{
	static std::string convert(const Eigen::Vector<T, N>& value)
	{
		Eigen::IOFormat testFmt(4, Eigen::DontAlignCols, ", ", "\n", "[", "]");
		std::stringstream ss;
		ss << (value.transpose()).format(testFmt);
		return ss.str();
	}
};

template<typename DerivedA>
struct IsCloseMatcher : Catch::Matchers::MatcherGenericBase
{
	IsCloseMatcher(
	    const Eigen::Ref<const DerivedA> a,
	    const typename DerivedA::RealScalar rtol =
	        Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
	    const typename DerivedA::RealScalar atol =
	        Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
	  : a(a)
	  , rtol(rtol)
	  , atol(atol)
	{
	}

	template<typename DerivedB>
	bool match(const Eigen::DenseBase<DerivedB>& b) const
	{
		return ((a.derived() - b.derived()).array().abs() <=
		        (atol + rtol * b.derived().array().abs()))
		    .all();
	}

	std::string describe() const override
	{
		std::stringstream ss;
		ss << "Is close to: " << StringMaker<DerivedA>::convert(a)
		   << "\nrtol = " << rtol << ", atol = " << atol;
		return ss.str();
	}

  private:
	const Eigen::Ref<const DerivedA> a;
	const typename DerivedA::RealScalar rtol;
	const typename DerivedA::RealScalar atol;
};

template<typename T>
IsCloseMatcher<T>
IsClose(T value)
{
	return IsCloseMatcher<T>(value, 1e-10, 1e-12);
}

}  // namespace Catch

#endif
