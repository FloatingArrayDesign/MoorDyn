#include "VTK_Util.hpp"

#ifdef USE_VTK

vtkSmartPointer<vtkFloatArray>
vector_to_vtk_array(const char* name, const std::vector<double>& data)
{

	vtkSmartPointer<vtkFloatArray> a = vtkSmartPointer<vtkFloatArray>::New();
	a->SetName(name);
	a->SetNumberOfComponents(1);
	a->SetNumberOfTuples(data.size());

	for (vtkIdType i = 0; i < static_cast<vtkIdType>(data.size()); i++) {
		a->SetValue(i, data[i]);
	}
	return a;
}

#endif
