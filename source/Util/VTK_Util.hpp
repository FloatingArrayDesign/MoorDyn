#pragma once

#ifdef USE_VTK

#include <vtkFloatArray.h>
#include <vtkSmartPointer.h>
#include <Eigen/Dense>

/**
 * @brief Converts a vector of doubles to a vtkFloatArray
 *
 * @param name The desired name of the vtkFloatArray
 * @param data The std::vector of data
 * @return vtkSmartPointer<vtkFloatArray>
 */
vtkSmartPointer<vtkFloatArray>
vector_to_vtk_array(const char* name, const std::vector<double>& data);

/**
 * @brief Returns the number of rows in an Eigen::Matrix that has 1 column
 *
 * @tparam T
 * @tparam N
 * @param mat
 * @return constexpr unsigned int
 */
template<typename T, int N>
constexpr int
get_vec_size(const Eigen::Matrix<T, N, 1>& mat)
{
	static_assert(N >= 0,
	              "cannot get_vec_size with dynamic sized Eigen vector");
	return N;
}

/**
 * @brief Converts a vector of Eigen Vectors to a vtkFloatArray
 * This function is not well constrained to taking only things that are
 * Eigen::Matrix<double, N, 1> but calling this function with something other
 * than those will probably result in template errors. THe number of components
 * of the vtkFloatArray will be equal to the dimension of the vector.
 * @tparam T
 * @param name Desired name of the float array
 * @param data The std::vector of Eigen Vectors of floating point values
 * @return vtkSmartPointer<vtkFloatArray>
 */
template<typename T>
vtkSmartPointer<vtkFloatArray>
vector_to_vtk_array(const char* name, const std::vector<T>& data)
{

	vtkSmartPointer<vtkFloatArray> a = vtkSmartPointer<vtkFloatArray>::New();
	a->SetName(name);
	const int num_comps = get_vec_size(data.front());
	a->SetNumberOfComponents(num_comps);
	a->SetNumberOfTuples(data.size());

	for (vtkIdType i = 0; i < static_cast<vtkIdType>(data.size()); i++) {
		for (int j = 0; j < static_cast<int>(num_comps); j++) {
			a->SetTypedComponent(i, j, data[i][j]);
		}
	}
	return a;
}
#endif
