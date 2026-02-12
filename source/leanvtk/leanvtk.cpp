#include <leanvtk.hpp>
#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
std::string a_library_function(){
    return std::string("a function specified in the source code");
}

namespace leanvtk {

static const int VTK_EMPTY_CELL = 0;
static const int VTK_VERTEX = 1;
static const int VTK_LINE = 3;
static const int VTK_POLY_LINE = 4;
static const int VTK_TRIANGLE = 5;
static const int VTK_POLYGON = 7;
static const int VTK_QUAD = 9;
static const int VTK_TETRA = 10;
static const int VTK_HEXAHEDRON = 12;

inline static int VTKTagVolume(const int n_vertices) {
  switch (n_vertices) {
  case 4:
    return VTK_TETRA;
  case 8:
    return VTK_HEXAHEDRON;
  default:
    // element type not supported. To add it
    // (http://www.vtk.org/VTK/img/file-formats.pdf)
    cerr << n_vertices << " not supported, " << endl;
    assert(false);
    return -1;
  }
}

inline static int VTKTagPlanar(const int n_vertices) {
  switch (n_vertices) {
  case 1:
    return VTK_VERTEX;
  case 2:
    return VTK_POLYGON;
  case 3:
    return VTK_TRIANGLE;
  case 4:
    return VTK_QUAD;
  default:
    // element type not supported. To add it
    // (http://www.vtk.org/VTK/img/file-formats.pdf)
    cerr << "{} not supported, " << n_vertices << endl;
    assert(false);
    return -1;
  }
}

VTUWriter::~VTUWriter() {
  for (auto node : point_data_)
    delete node;
  for (auto node : cell_data_)
    delete node;
}

void VTUWriter::write_point_data(std::ostream &os) {
  if (current_scalar_point_data_.empty() && current_vector_point_data_.empty())
    return;

  os << "<PointData ";
  if (!current_scalar_point_data_.empty())
    os << "Scalars=\"" << current_scalar_point_data_ << "\" ";
  if (!current_vector_point_data_.empty())
    os << "Vectors=\"" << current_vector_point_data_ << "\" ";
  os << ">\n";

  for (auto node : point_data_) {
    node->set_binary(is_binary());
    node->write(os);
  }

  os << "</PointData>\n";
}

void VTUWriter::write_header(const size_t n_vertices, const size_t n_elements,
                             std::ostream &os) {
  os << "<?xml version=\"1.0\"?>\n";
  os << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\"";
  if (binary_) {
    os << " header_type=\"UInt64\"";
    int n = 1;
    if(*(char *)&n == 1) {
      os << " byte_order=\"LittleEndian\"";
    } else {
      os << " byte_order=\"BigEndian\"";
    }
  }
  os << ">\n";
  os << "<UnstructuredGrid>\n";
  os << "<Piece NumberOfPoints=\"" << n_vertices << "\" NumberOfCells=\""
     << n_elements << "\">\n";
}

void VTUWriter::write_footer(std::ostream &os) {
  os << "</Piece>\n";
  os << "</UnstructuredGrid>\n";
  os << "</VTKFile>\n";
}

void VTUWriter::write_points(std::ostream &os,
                             const size_t num_points,
                             const vector<double> &points,
                             bool is_volume_mesh) {
  os << "<Points>\n";
  os << "<DataArray type=\"Float64\" NumberOfComponents=\"3\" "
        "format=\"" << (binary_ ? "binary" : "ascii") << "\">\n";
  assert(!(points.size() % num_points));
  size_t dim = points.size() / num_points;

  vector<double> pts;
  if (!is_volume_mesh && dim == 2) {
    pts.resize(num_points * 3);
    for (size_t d = 0; d < num_points; ++d) {
      for (size_t i = 0; i < 2; ++i) {
        const size_t idx = _index(dim, d, i);
        pts[_index(3, d, i)] = points[_index(dim, d, i)];
      }
      pts[_index(3, d, 2)] = 0;
    }
    dim = 3;
  } else {
    pts = points;
  }

  if (binary_) {
    uint64_t data_bytes = sizeof(double) * pts.size();
    os << base64::encode((unsigned char*)(&data_bytes), sizeof(uint64_t))
       << base64::encode((unsigned char*)pts.data(),
                         data_bytes)
       << "\n";
  } else {
    for (int d = 0; d < num_points; ++d) {
      for (int i = 0; i < dim; ++i) {
        int idx = _index(dim, d, i); 
        os << pts.at(idx);
        if (i < dim - 1) {
          os << " ";
        }
      }
      os << "\n";
    }
  }

  os << "</DataArray>\n";
  os << "</Points>\n";
}

void VTUWriter::write_cells(std::ostream &os,
                            const size_t n_vertices,
                            const vector<size_t> &tets,
                            bool is_volume_mesh) {
  const int n_cells = tets.size() / n_vertices;
  os << "<Cells>\n";
  /////////////////////////////////////////////////////////////////////////////
  // List vertex id's i=0, ..., n_vertices associated with each cell c
  os << "<DataArray type=\"UInt" << 8 * sizeof(size_t) << "\" "
     << "Name=\"connectivity\" "
     << "format=\"" << (binary_ ? "binary" : "ascii") << "\">\n";
  if (binary_) {
    size_t data_bytes = sizeof(size_t) * tets.size();
    os << base64::encode((unsigned char*)(&data_bytes), sizeof(size_t))
       << base64::encode((unsigned char*)tets.data(),
                         data_bytes)
       << "\n";
  } else {
    for (int c = 0; c < n_cells; ++c) {
      for (int i = 0; i < n_vertices; ++i) {
        int idx = _index(n_vertices, c, i);
        const int v_index = tets.at(idx);
        os << v_index;
        if (i < n_vertices - 1) {
          os << " ";
        }
      }
      os << "\n";
    }
  }

  os << "</DataArray>\n";
  /////////////////////////////////////////////////////////////////////////////
  // List the VTK cell type for each mesh element.
  // This assumes a uniform cell type the entire mesh; to generalize, pass
  // or compute the number of vertices per cell and recompute the cell type
  int cell_type =
      is_volume_mesh ? VTKTagVolume(n_vertices) : VTKTagPlanar(n_vertices);

  os << "<DataArray type=\"Int8\" Name=\"types\" format=\"ascii\" "
        "RangeMin=\""
     << cell_type << "\" RangeMax=\"" << cell_type << "\">\n";
  for (int i = 0; i < n_cells; ++i) {
    os << cell_type << "\n";
  }
  os << "</DataArray>\n";

  /////////////////////////////////////////////////////////////////////////////
  // List offsets to access the vertex indices of the ith cell. Non-trivial
  // if the mesh is a general polyognal mesh.
  os << "<DataArray type=\"UInt" << 8 * sizeof(size_t) << "\" "
     << "Name=\"offsets\" "
     << "format=\"" << (binary_ ? "binary" : "ascii") << "\" "
     << "RangeMin=\"" << n_vertices << "\" "
     << "RangeMax=\"" << n_cells * n_vertices << "\">\n";

  vector<size_t> offsets(n_cells);
  for (unsigned int i = 0; i < n_cells; ++i) {
    offsets[i] = n_vertices * (i + 1);
  }
  if (binary_) {
    size_t data_bytes = sizeof(size_t) * offsets.size();
    os << base64::encode((unsigned char*)(&data_bytes), sizeof(size_t))
       << base64::encode((unsigned char*)offsets.data(),
                         data_bytes)
       << "\n";
  } else {
    for (auto offset : offsets) {
      os << offset << "\n";
    }
  }

  os << "</DataArray>\n";
  /////////////////////////////////////////////////////////////////////////////
  os << "</Cells>\n";
}

void VTUWriter::write_cell_data(std::ostream &os) {
  if (current_scalar_cell_data_.empty() && current_vector_cell_data_.empty())
    return;

  os << "<CellData ";
  if (!current_scalar_cell_data_.empty())
    os << "Scalars=\"" << current_scalar_cell_data_ << "\" ";
  if (!current_vector_cell_data_.empty())
    os << "Vectors=\"" << current_vector_cell_data_ << "\" ";
  os << ">\n";

  for (auto node : cell_data_) {
    node->set_binary(is_binary());
    node->write(os);
  }

  os << "</CellData>\n";
}
void VTUWriter::clear() {
  for (auto node : point_data_)
    delete node;
  for (auto node : cell_data_)
    delete node;
  point_data_.clear();
  cell_data_.clear();
  current_scalar_point_data_.clear();
  current_vector_point_data_.clear();
  current_scalar_cell_data_.clear();
  current_vector_cell_data_.clear();
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<double> &data) {
  vector<double> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  point_data_.push_back(make_data_node(name, tmp));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<float> &data) {
  vector<float> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  point_data_.push_back(make_data_node(name, tmp));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<int8_t> &data) {
  point_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<int16_t> &data) {
  point_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<int32_t> &data) {
  point_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<int64_t> &data) {
  point_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<uint8_t> &data) {
  point_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<uint16_t> &data) {
  point_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<uint32_t> &data) {
  point_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_scalar_field(const std::string &name,
                                 const vector<uint64_t> &data) {
  point_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<double> &data,
                                 const int &dimension) {
  vector<double> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  point_data_.push_back(make_data_node(name, tmp, "Float", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<float> &data,
                                 const int &dimension) {
  vector<float> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  point_data_.push_back(make_data_node(name, tmp, "Float", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<int8_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<int16_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<int32_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<int64_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<uint8_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<uint16_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<uint32_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_vector_field(const std::string &name,
                                 const vector<uint64_t> &data,
                                 const int &dimension) {
  point_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_point_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<double> &data) {
  vector<double> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  cell_data_.push_back(make_data_node(name, tmp, "Float"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<float> &data) {
  vector<float> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  cell_data_.push_back(make_data_node(name, tmp, "Float"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<int8_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<int16_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<int32_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<int64_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "Int"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<uint8_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<uint16_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<uint32_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_scalar_field(const std::string &name,
                                      const vector<uint64_t> &data) {
  cell_data_.push_back(make_data_node(name, data, "UInt"));
  current_scalar_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<double> &data,
                                 const int &dimension) {
  vector<double> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  cell_data_.push_back(make_data_node(name, tmp, "Float", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<float> &data,
                                 const int &dimension) {
  vector<float> tmp(data.size());
  for (long i = 0; i < data.size(); ++i)
    tmp[i] = std::abs(data[i]) < 1e-16 ? 0 : data[i];
  cell_data_.push_back(make_data_node(name, tmp, "Float", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<int8_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<int16_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<int32_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<int64_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "Int", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<uint8_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<uint16_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<uint32_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_cell_data_ = name;
}

template<>
void VTUWriter::add_cell_vector_field(const std::string &name,
                                 const vector<uint64_t> &data,
                                 const int &dimension) {
  cell_data_.push_back(make_data_node(name, data, "UInt", dimension));
  current_vector_cell_data_ = name;
}

bool VTUWriter::write_mesh(std::ostream &os,
                           const size_t dim,
                           const size_t cell_size,
                           const vector<double> &points,
                           const vector<size_t> &tets,
                           bool is_volume_mesh){
  assert(dim > 1);
  assert(cell_size > 1);

  int num_points = points.size() / dim;
  int num_cells = tets.size() / cell_size;

  write_header(num_points, num_cells, os);
  write_points(os, num_points, points, is_volume_mesh);
  write_point_data(os);
  write_cells(os, cell_size, tets, is_volume_mesh);
  write_cell_data(os);
  write_footer(os);
  clear();
  path_ = "";
  return true;
}

bool VTUWriter::write_mesh(const std::string &path,
                           const size_t dim,
                           const size_t cell_size,
                           const vector<double> &points,
                           const vector<size_t> &tets,
                           bool is_volume_mesh) {
  std::ofstream os;
  os.open(path.c_str());
  if (!os.good()) {
    os.close();
    return false;
  }

  write_mesh(os, dim, cell_size, points, tets, is_volume_mesh);

  os.close();

  path_ = path;
  return true;
}

bool VTUWriter::write_surface_mesh(const std::string &path,
                                   const size_t dim,
                                   const size_t cell_size,
                                   const vector<double> &points,
                                   const vector<size_t> &tets) {
  
    return write_mesh(path, dim, cell_size, points, tets, false);
}

bool VTUWriter::write_volume_mesh(const std::string &path,
                                  const size_t dim,
                                  const size_t cell_size,
                                  const vector<double> &points,
                                  const vector<size_t> &tets) {
  
    return write_mesh(path, dim, cell_size, points, tets, true);
}

bool VTUWriter::write_surface_mesh(std::ostream &os,
                                   const size_t dim,
                                   const size_t cell_size,
                                   const vector<double> &points,
                                   const vector<size_t> &tets) {
  
    return write_mesh(os, dim, cell_size, points, tets, false);
}

bool VTUWriter::write_volume_mesh(std::ostream &os,
                                  const size_t dim,
                                  const size_t cell_size,
                                  const vector<double> &points,
                                  const vector<size_t> &tets) {
  
    return write_mesh(os, dim, cell_size, points, tets, true);
}

bool VTUWriter::write_point_cloud(std::ostream &os,
                                  const size_t dim,
                                  const vector<double> &points) {
  vector<size_t> tets;
  tets.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i)
      tets[i] = i;
  return write_surface_mesh(os, dim, 1, points, tets);
}

bool VTUWriter::write_point_cloud(const std::string &path,
                                  const size_t dim,
                                  const vector<double> &points) {
  vector<size_t> tets;
  tets.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i)
      tets[i] = i;
  return write_surface_mesh(path, dim, 1, points, tets);
}

bool DECLDIR write_vtm(const std::string &path,
                       std::vector<VTUWriter> vtus)
{
  std::ofstream os;
  os.open(path.c_str());
  if (!os.good()) {
    os.close();
    return false;
  }

  bool status = write_vtm(os, vtus);

  os.close();
  return status;
}

bool DECLDIR write_vtm(std::ostream &os,
                       std::vector<VTUWriter> vtus)
{
  for (auto vtu : vtus)
    assert(vtu.filepath() != "");
  os << "<?xml version=\"1.0\"?>\n"
     << "<VTKFile type=\"vtkMultiBlockDataSet\" version=\"1.0\">\n"
     << "<vtkMultiBlockDataSet>\n";
  for (size_t i = 0; i < vtus.size(); i++) {
    os << "<DataSet index=\"" << i
       << "\" file=\"" << vtus[i].filepath() << "\"/>\n";
  }
  os << "</vtkMultiBlockDataSet>\n"
     << "</VTKFile>\n";
  return true;
}

} // namespace leanvtk
