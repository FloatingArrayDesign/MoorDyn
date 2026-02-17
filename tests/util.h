#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include "Misc.hpp"

#include <sstream>
#include <filesystem>
#include <string_view>
#include <unordered_map>

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
