
#ifndef LEANVTK_HPP
#define LEANVTK_HPP

#include <string>
#include <cassert>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>

#ifdef LeanVTK_EXPORTS
#ifdef WIN32
#define DECLDIR __declspec(dllexport)
#else
#define DECLDIR
#endif
#else
#ifdef WIN32
#define DECLDIR __declspec(dllimport)
#else
#define DECLDIR
#endif
#endif

namespace leanvtk {

inline size_t _index(size_t N, size_t i, size_t j) {
  assert(N > 0);
  return i * N + j;
}

namespace base64 {
static const unsigned char encode_table[65] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "abcdefghijklmnopqrstuvwxyz"
  "0123456789+/";

inline static unsigned char encode_char(unsigned char c)
{
  assert(c < 65);
  return encode_table[c];
}

inline void encode_triplet(
  unsigned char i0, unsigned char i1, unsigned char i2,
  unsigned char* o0, unsigned char* o1, unsigned char* o2, unsigned char* o3)
{
  *o0 = encode_char((i0 >> 2) & 0x3F);
  *o1 = encode_char(((i0 << 4) & 0x30) | ((i1 >> 4) & 0x0F));
  *o2 = encode_char(((i1 << 2) & 0x3C) | ((i2 >> 6) & 0x03));
  *o3 = encode_char(i2 & 0x3F);
}

inline void encode_pair(
  unsigned char i0, unsigned char i1,
  unsigned char* o0, unsigned char* o1, unsigned char* o2, unsigned char* o3)
{
  *o0 = encode_char((i0 >> 2) & 0x3F);
  *o1 = encode_char(((i0 << 4) & 0x30) | ((i1 >> 4) & 0x0F));
  *o2 = encode_char(((i1 << 2) & 0x3C));
  *o3 = '=';
}

inline void encode_single(
  unsigned char i0,
  unsigned char* o0, unsigned char* o1, unsigned char* o2, unsigned char* o3)
{
  *o0 = encode_char((i0 >> 2) & 0x3F);
  *o1 = encode_char(((i0 << 4) & 0x30));
  *o2 = '=';
  *o3 = '=';
}

inline unsigned long encoder(const unsigned char* input,
                             unsigned long length,
                             unsigned char* output,
                             int mark_end=0)
{
  const unsigned char* ptr = input;
  const unsigned char* end = input + length;
  unsigned char* optr = output;

  // Encode complete triplet

  while ((end - ptr) >= 3)
  {
    encode_triplet(
      ptr[0], ptr[1], ptr[2], &optr[0], &optr[1], &optr[2], &optr[3]);
    ptr += 3;
    optr += 4;
  }

  // Encodes a 2-byte ending into 3 bytes and 1 pad byte and writes.

  if (end - ptr == 2)
  {
    encode_pair(ptr[0], ptr[1], &optr[0], &optr[1], &optr[2], &optr[3]);
    optr += 4;
  }

  // Encodes a 1-byte ending into 2 bytes and 2 pad bytes

  else if (end - ptr == 1)
  {
    encode_single(ptr[0], &optr[0], &optr[1], &optr[2], &optr[3]);
    optr += 4;
  }

  // Do we need to mark the end

  else if (mark_end)
  {
    optr[0] = optr[1] = optr[2] = optr[3] = '=';
    optr += 4;
  }

  return optr - output;
}

inline std::string encode(const unsigned char *data,
                          size_t input_length) {
    size_t output_length = input_length / 3 * 4 + 4;
    std::string encoded_data;
    encoded_data.resize(output_length, ' ');
    output_length = encoder(data,
                            input_length,
                            (unsigned char*)encoded_data.data());
    encoded_data.resize(output_length);
    return encoded_data;
}

} // namespace base64

class VTKDataNodeBase {
public:
  VTKDataNodeBase(const std::string &name="")
      : name_(name)
      , binary_(false)
  {}

  virtual void write(std::ostream &os) const
  {
  };

  /// Set the format to binary
  inline void set_binary() { binary_ = true; }

  /// Set the format to ASCII
  inline void set_ascii() { binary_ = false; }

  /// Set whether binary format is considered or not
  inline void set_binary(bool enable) { binary_ = enable; }

  /// Get if binary format is enabled
  inline bool is_binary() const { return binary_; }
protected:
  std::string name_;
  bool binary_;
};

template <typename T>
class VTKDataNode : public VTKDataNodeBase {

public:
  VTKDataNode()
      : VTKDataNodeBase()
  {}

  VTKDataNode(const std::string &name, const std::string &numeric_type,
              const std::vector<T> &data = std::vector<double>(),
              const int n_components = 1)
      : VTKDataNodeBase(name)
      , numeric_type_(numeric_type)
      , data_(data)
      , n_components_(n_components)
  {}

  inline std::vector<T> &data() { return data_; }

  void initialize(const std::string &name, const std::string &numeric_type,
                  const std::vector<T> &data, const int n_components = 1) {
    name_ = name;
    numeric_type_ = numeric_type;
    data_ = data;
    n_components_ = n_components;
  }

  void write(std::ostream &os) const override {
    os << "<DataArray type=\"" << numeric_type_ << "\" Name=\"" << name_
       << "\" NumberOfComponents=\"" << n_components_
       << "\" format=\"" << (binary_ ? "binary" : "ascii") << "\">\n";
    if (binary_) {
      uint64_t data_bytes = sizeof(T) * data_.size();
      os << base64::encode((unsigned char*)(&data_bytes), sizeof(uint64_t))
         << base64::encode((unsigned char*)data_.data(),
                           data_bytes)
         << "\n";
    } else {
      const int num_points = data_.size() / n_components_;
      for (int d = 0; d < num_points; ++d) {
        for (int i = 0; i < n_components_; ++i) {
          int idx = _index(n_components_, d, i); 
          os << data_.at(idx);
          if (i < n_components_ - 1) {
            os << " ";
          }
        }
        os << "\n";
      }
    }

    os << "</DataArray>\n";
  }

  inline bool empty() const { return data_.size() <= 0; }

private:
  std::string numeric_type_;
  std::vector<T> data_;
  int n_components_;
};

class DECLDIR VTUWriter {
public:
  VTUWriter()
      : binary_(false)
      , path_("")
  {}

  ~VTUWriter();

  /**
   * Write surface mesh to a file
   * const string& path             filename to store vtk mesh (ending with .vtu)
   * const int dim                  ambient dimension (2D or 3D)
   * const int cell_size            number of vertices per cell 
   *                                (3 for triangles, 4 for quads and tets, 8
   *                                for hexes)
   * const vector<double>& points   list of point locations. Format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and 
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   * const vector<int >& elements   list of point indices per cell. Format  of the
   *                                vector is:
   *                                  [c_{1,1}, c_{1,2},..., c_{1, cell_size}, 
   *                                  ...  
   *                                  c_{cell_size,1}, c_{cell_size,2},..., c_{cell_size, cell_size}]
   *                                  (i.e. index c*i corresponds to the ith
   *                                  vertex in the cth cell in the mesh
   */
  bool write_surface_mesh(const std::string &path,
                          const size_t dim,
                          const size_t cell_size,
                          const std::vector<double> &points,
                          const std::vector<size_t> &elements);

  /**
   * Write surface mesh to an output stream
   * ostream &os                    output stream where to write vtk mesh (ending with .vtu)
   * const int dim                  ambient dimension (2D or 3D)
   * const int cell_size            number of vertices per cell
   *                                (3 for triangles, 4 for quads and tets, 8
   *                                for hexes)
   * const vector<double>& points   list of point locations. Format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   * const vector<int >& elements   list of point indices per cell. Format  of the
   *                                vector is:
   *                                  [c_{1,1}, c_{1,2},..., c_{1, cell_size},
   *                                  ...
   *                                  c_{cell_size,1}, c_{cell_size,2},..., c_{cell_size, cell_size}]
   *                                  (i.e. index c*i corresponds to the ith
   *                                  vertex in the cth cell in the mesh
   */
  bool write_surface_mesh(std::ostream &os,
                          const size_t dim,
                          const size_t cell_size,
                          const std::vector<double> &points,
                          const std::vector<size_t> &elements);
  /**
   * Write volume mesh to a file
   *
   * const string& path             filename to store vtk mesh (ending with .vtu)
   * const int dim                  ambient dimension (2D or 3D)
   * const int cell_size            number of vertices per cell 
   *                                (3 for triangles, 4 for quads and tets, 8
   *                                for hexes)
   * const vector<double>& points   list of point locations. If there are 
   *                                n points in the mesh, the format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and 
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   * const vector<int >& elements   list of point indices per cell. Format  of the
   *                                vector is:
   *                                  [c_{1,1}, c_{1,2},..., c_{1, cell_size}, 
   *                                  ...  
   *                                  c_{m,1}, c_{m,2},..., c_{m, cell_size}]
   *                                if there are m cells
   *                                (i.e. index c*i corresponds to the ith
   *                                vertex in the cth cell in the mesh
   */
  bool write_volume_mesh(const std::string &path, 
                         const size_t dim,
                         const size_t cell_size, 
                         const std::vector<double> &points,
                         const std::vector<size_t> &elements);

  /**
   * Write volume mesh to an output stream
   *
   * ostream &os                    output stream where to write vtk mesh (ending with .vtu)
   * const int dim                  ambient dimension (2D or 3D)
   * const int cell_size            number of vertices per cell
   *                                (3 for triangles, 4 for quads and tets, 8
   *                                for hexes)
   * const vector<double>& points   list of point locations. If there are
   *                                n points in the mesh, the format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   * const vector<int >& elements   list of point indices per cell. Format  of the
   *                                vector is:
   *                                  [c_{1,1}, c_{1,2},..., c_{1, cell_size},
   *                                  ...
   *                                  c_{m,1}, c_{m,2},..., c_{m, cell_size}]
   *                                if there are m cells
   *                                (i.e. index c*i corresponds to the ith
   *                                vertex in the cth cell in the mesh
   */
  bool write_volume_mesh(std::ostream &os,
                         const size_t dim,
                         const size_t cell_size,
                         const std::vector<double> &points,
                         const std::vector<size_t> &elements);

  /**
   * Write point cloud to a file
   *
   * const string& path             filename to store vtk mesh (ending with .vtu)
   * const int dim                  ambient dimension (2D or 3D)
   * const vector<double>& points   list of point locations. If there are 
   *                                n points in the mesh, the format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and 
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   */
  bool write_point_cloud(const std::string &path,
                         const size_t dim,
                         const std::vector<double> &points); 

  /**
   * Write point cloud to a file
   *
   * ostream &os                    output stream where to write vtk mesh (ending with .vtp)
   * const int dim                  ambient dimension (2D or 3D)
   * const vector<double>& points   list of point locations. If there are 
   *                                n points in the mesh, the format  of the
   *                                vector is:
   *                                  [x_1, y_1, x_2, y_2, ..., x_n, y_n]
   *                                for 2D and 
   *                                  [x_1, y_1, z_1, ..., x_n, y_n, z_n]
   *                                for 3D.
   */
  bool write_point_cloud(std::ostream &os,
                         const size_t dim,
                         const std::vector<double> &points); 

  /**
   * Add a general field to the mesh
   * const string& name             name of the field to store vtk mesh 
   * const vector<double>& data     list of field values. There must be dimension 
   *                                values for each point in the mesh to be written.
   *                                Format of the vector is 
   *                                  [f_{1,1}, f_{1,2},..., f_{1, dimension}, 
   *                                  ...  
   *                                  f_{n,1}, f_{n,2},..., f_{n, dimension}]
   *                                if there are n points in the mesh
   * const int dimension            ambient dimension (2D or 3D)
   */
  template <typename T>
  inline void add_field(const std::string &name, 
                        const std::vector<T> &data,
                        const int &dimension)
  {
    if (dimension == 1)
      add_scalar_field<T>(name, data);
    else
      add_vector_field<T>(name, data, dimension);
  }

  /**
   * Add a general cell/element field to the mesh
   * const string& name             name of the field to store vtk mesh
   * const vector<double>& data     list of field values. There must be dimension
   *                                values for each cell in the mesh to be written.
   *                                Format of the vector is
   *                                  [f_{1,1}, f_{1,2},..., f_{1, dimension},
   *                                  ...
   *                                  f_{m,1}, f_{m,2},..., f_{m, dimension}]
   *                                if there are m cells in the mesh
   * const int dimension            ambient dimension (2D or 3D)
   */
  template <typename T>
  void add_cell_field(const std::string &name,
                 const std::vector<T> &data,
                 const int &dimension)
  {
    if (dimension == 1)
      add_cell_scalar_field<T>(name, data);
    else
      add_cell_vector_field<T>(name, data, dimension);
  }

  /**
   * Add a scalar field to the mesh
   * const string& name             name of the field to store vtk mesh
   * const vector<double>& data     list of field values. There must be one
   *                                value for each point in the mesh to be written.
   *                                Format of the vector is
   *                                  [f_1, f_2,..., f_n]
   *                                if there are n points in the mesh
   */
  template <typename T>
  void add_scalar_field(const std::string &name,
                        const std::vector<T> &data);

  /**
   * Add a scalar field to cells/elements of the mesh
   * const string& name             name of the field to store vtk mesh
   * const vector<double>& data     list of field values. There must be one
   *                                value for each cell in the mesh to be written.
   *                                Format of the vector is
   *                                  [f_1, f_2,..., f_m]
   *                                if there are m cells in the mesh
   */
  template <typename T>
  void add_cell_scalar_field(const std::string &name,
                        const std::vector<T> &data);

  /**
   * Add a vector field to the mesh
   * const string& name             name of the field to store vtk mesh 
   * const vector<double>& data     list of field values. There must be dimension 
   *                                values for each point in the mesh to be written.
   *                                Format of the vector is 
   *                                  [f_{1,1}, f_{1,2},..., f_{1, dimension}, 
   *                                  ...  
   *                                  f_{n,1}, f_{n,2},..., f_{n, dimension}]
   *                                if there are n points in the mesh
   * const int dimension            ambient dimension (2D or 3D)
   */
  template <typename T>
  void add_vector_field(const std::string &name,
                        const std::vector<T> &data, 
                        const int &dimension);

  /**
   * Add a vector field to cells/elements of the mesh
   * const string& name             name of the field to store vtk mesh
   * const vector<double>& data     list of field values. There must be dimension
   *                                values for each cell in the mesh to be written.
   *                                Format of the vector is
   *                                  [f_{1,1}, f_{1,2},..., f_{1, dimension},
   *                                  ...
   *                                  f_{m,1}, f_{m,2},..., f_{m, dimension}]
   *                                if there are m bool binary = falsecells in the mesh
   * const int dimension            ambient dimension (2D or 3D)
   */
  template <typename T>
  void add_cell_vector_field(const std::string &name,
                        const std::vector<T> &data,
                        const int &dimension);

  // Remove all fields and initialized data from the writer.
  void clear();

  /// Set the format to binary
  inline void set_binary() { binary_ = true; }

  /// Set the format to ASCII
  inline void set_ascii() { binary_ = false; }

  /// Set whether binary format is considered or not
  inline void set_binary(bool enable) { binary_ = enable; }

  /// Get if binary format is enabled
  inline bool is_binary() { return binary_; }

  /// Get the last saved file path (empty string if there is no such a path)
  inline std::string filepath() { return path_; }
private:
  std::vector<VTKDataNodeBase*> point_data_;
  std::vector<VTKDataNodeBase*> cell_data_;
  std::string current_scalar_point_data_;
  std::string current_vector_point_data_;
  std::string current_scalar_cell_data_;
  std::string current_vector_cell_data_;
  bool binary_;
  /// Last saved file path. Not available when using std::ostream to save
  std::string path_;

  void write_point_data(std::ostream &os);

  void write_cell_data(std::ostream &os);

  void write_header(const size_t n_vertices, const size_t n_elements,
                    std::ostream &os);

  void write_footer(std::ostream &os);
  
  bool write_mesh(std::ostream &os,
                  const size_t dim,
                  const size_t cell_size,
                  const std::vector<double> &points,
                  const std::vector<size_t> &tets, 
                  bool is_volume_mesh=true);

  bool write_mesh(const std::string &path,
                  const size_t dim, const size_t cell_size,
                  const std::vector<double> &points,
                  const std::vector<size_t> &tets, 
                  bool is_volume_mesh=true);

  void write_points(std::ostream &os,
                    const size_t num_points,
                    const std::vector<double> &points,
                    bool is_volume_mesh = true);

  void write_cells(std::ostream &os,
                   const size_t n_vertices,
                   const std::vector<size_t> &tets,
                   bool is_volume_mesh = true);

  template <typename T>
  inline static VTKDataNode<T>* make_data_node(const std::string &name,
                                               const std::vector<T> &data,
                                               std::string num_type="Float",
                                               const int dimension=1)
  {
    VTKDataNode<T>* node = new VTKDataNode<T>(
      name,
      num_type + std::to_string(8 * sizeof(T)),
      data,
      dimension);
    return node;
  }
};

/** @brief Write a multiblock .vtm file on top of the already written VTUs
 * @param path The output file path
 * @param vtus The list of VTU files
 */
bool DECLDIR write_vtm(const std::string &path,
                       std::vector<VTUWriter> vtus);

/** @brief Write a multiblock .vtm file on top of the already written VTUs
 * @param path The output file path
 * @param vtus The list of VTU files
 */
bool DECLDIR write_vtm(std::ostream &os,
                       std::vector<VTUWriter> vtus);

} // namespace leanvtk

#endif  // LEANVTK_HPP
