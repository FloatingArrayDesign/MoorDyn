/*
 * Copyright (c) 2022, Jose Luis Cercos-Pita <jlc@core-marine.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file IO.hpp
 * Input/Output operations
 */

#pragma once

#include "Misc.hpp"
#include "Log.hpp"
#include <cstdint>
#include <string>
#include <vector>
#ifdef USE_VTK
#include <vtkSmartPointer.h>
#include <vtkCharArray.h>
#include <vtkFloatArray.h>
#include <vtkErrorCode.h>
#endif

namespace moordyn {

/** @namespace io Input/Output
 */
namespace io {

/** @class IO IO.hpp
 * @brief A base class for all the entities that must save/load data to/from
 * disk
 *
 * @note Since this class already publicly inherits moordyn::LogUser, all the
 * classes inheriting this class do not require inheriting also LogUser
 */
class IO : public LogUser
{
  public:
	/** @brief Costructor
	 * @param log Logging handler
	 */
	IO(moordyn::Log* log);

	/** @brief Destructor
	 */
	~IO();

	/** @brief Save the entity into a file
	 *
	 * It is of course possible to save each entity in a separate file. However,
	 * since this function is just redirecting the work to save(void), which
	 * might produce the data to save recursively, actually the whole system
	 * can be saved in the same file
	 * @param filepath The output file path
	 */
	void Save(const std::string filepath);

	/** @brief Loads the entity from a file
	 *
	 * It is the inverse of Save(filepath)
	 * @param filepath The output file path
	 */
	void Load(const std::string filepath);

	/** @brief Produce the packed data to be saved
	 *
	 * The produced data can be used afterwards to restore the saved information
	 * afterwards calling Deserialize(void).
	 *
	 * This is the function that each inherited class must implement
	 * @return The packed data
	 */
	virtual std::vector<uint64_t> Serialize(void) = 0;

	/** @brief Unpack the data to restore the Serialized information
	 *
	 * This is the function that each inherited class must implement, and should
	 * be the inverse of Serialize(void)
	 * @param data The packed data
	 * @return A pointer to the end of the file, for debugging purposes
	 */
	virtual uint64_t* Deserialize(const uint64_t* data) = 0;

  protected:
	/** @brief Pack an unsigned integer to make it writable
	 * @param i The unsigned integer number
	 * @return The packed number
	 */
	uint64_t Serialize(const uint64_t& i);

	/** @brief Pack an integer to make it writable
	 * @param i The integer number
	 * @return The packed number
	 */
	uint64_t Serialize(const int64_t& i);

	/** @brief Pack a float to make it writable
	 * @param f The float number
	 * @return The packed number
	 */
	uint64_t Serialize(const real& f);

	/** @brief Pack a 3D vector to make it writable
	 * @param m The matrix
	 * @return The packed matrix
	 */
	std::vector<uint64_t> Serialize(const vec& m);

	/** @brief Pack a 6D vector to make it writable
	 * @param m The matrix
	 * @return The packed matrix
	 */
	std::vector<uint64_t> Serialize(const vec6& m);

	/** @brief Pack a 3x3 matrix to make it writable
	 * @param m The matrix
	 * @return The packed matrix
	 */
	std::vector<uint64_t> Serialize(const mat& m);

	/** @brief Pack a 6x6 matrix to make it writable
	 * @param m The matrix
	 * @return The packed matrix
	 */
	std::vector<uint64_t> Serialize(const mat6& m);

	/** @brief Pack a quaternion to make it writable
	 * @param m The quaternion
	 * @return The packed quaternion
	 */
	std::vector<uint64_t> Serialize(const quaternion& m);

	/** @brief Pack an XYZQuat to make it writable
	 * @param m The XYZQuat
	 * @return The packed XYZQuat
	 */
	std::vector<uint64_t> Serialize(const XYZQuat& m);

	/** @brief Pack a list of floating point numbers to make it writable
	 * @param l The list
	 * @return The packed list
	 */
	std::vector<uint64_t> Serialize(const std::vector<real>& l);

	/** @brief Pack a list of 3D vectors to make it writable
	 * @param l The list
	 * @return The packed list
	 */
	std::vector<uint64_t> Serialize(const std::vector<vec>& l);

	/** @brief Pack a list of 6D vectors to make it writable
	 * @param l The list
	 * @return The packed list
	 */
	std::vector<uint64_t> Serialize(const std::vector<vec6>& l);

	/** @brief Pack a list of 3x3 matrices to make it writable
	 * @param l The list
	 * @return The packed list
	 */
	std::vector<uint64_t> Serialize(const std::vector<mat>& l);

	/** @brief Pack a list of 6x6 matrices to make it writable
	 * @param l The list
	 * @return The packed list
	 */
	std::vector<uint64_t> Serialize(const std::vector<mat6>& l);

	/** @brief Pack a list of lists to make it writable
	 * This function might act recursively
	 * @param l The list
	 * @return The packed list
	 */
	template<typename T>
	std::vector<uint64_t> Serialize(const std::vector<std::vector<T>>& l)
	{
		std::vector<uint64_t> data;
		const uint64_t n = l.size();
		data.push_back(Serialize(n));
		for (auto v : l) {
			auto subdata = Serialize(v);
			data.insert(data.end(), subdata.begin(), subdata.end());
		}
		return data;
	}

	/** @brief Unpack a loaded unsigned integer
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, uint64_t& out);

	/** @brief Unpack a loaded integer
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, int64_t& out);

	/** @brief Unpack a loaded floating point number
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, real& out);

	/** @brief Unpack a loaded 3D vector
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, vec& out);

	/** @brief Unpack a loaded 6D vector
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, vec6& out);

	/** @brief Unpack a loaded 3x3 matrix
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, mat& out);

	/** @brief Unpack a loaded 6x6 matrix
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, mat6& out);

	/** @brief Unpack a loaded quaternion
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, quaternion& out);

	/** @brief Unpack a loaded XYZQuat
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, XYZQuat& out);

	/** @brief Unpack a loaded list of floating point numbers
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, std::vector<real>& out);

	/** @brief Unpack a loaded list of 3D vectors
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, std::vector<vec>& out);

	/** @brief Unpack a loaded list of 6D vectors
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, std::vector<vec6>& out);

	/** @brief Unpack a loaded list of 3x3 matrices
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, std::vector<mat>& out);

	/** @brief Unpack a loaded list of 6x6 matrices
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	uint64_t* Deserialize(const uint64_t* in, std::vector<mat6>& out);

	/** @brief Unpack a loaded list of lists
	 *
	 * This function might works recursively
	 * @param in The pointer to the next unread value
	 * @param out The unpacked value
	 * @return The new pointer to the remaining data to be read
	 */
	template<typename T>
	uint64_t* Deserialize(const uint64_t* in, std::vector<std::vector<T>>& out)
	{
		uint64_t n;
		uint64_t* remaining = Deserialize(in, n);
		out.clear();
		out.reserve(n);
		for (unsigned int i = 0; i < n; i++) {
			std::vector<T> v;
			remaining = Deserialize(remaining, v);
			out.push_back(v);
		}
		return remaining;
	}

  private:
	/// endianess of the system
	bool _is_big_endian;
	/// The minimum major version of the file that can be read
	uint8_t _min_major_version;
	/// The minimum minor version of the file that can be read
	uint8_t _min_minor_version;
};

#ifdef USE_VTK
/** @brief Create a data array
 *
 * This can be used later as either pointdata arrays or celldata arrays
 * @param name The array name. It should be unique for each object
 * @param dim The number of components of each array element. Usually 1 or 3
 * @param len The number of elements on the array
 * @return The VTK array
 */
vtkSmartPointer<vtkFloatArray>
vtk_farray(const char* name, unsigned int dim, unsigned int len);

/** @brief Create a data array
 *
 * This can be used later as either pointdata arrays or celldata arrays
 * @param name The array name. It should be unique for each object
 * @param dim The number of components of each array element. Usually 1 or 3
 * @param len The number of elements on the array
 * @return The VTK array
 */
vtkSmartPointer<vtkCharArray>
vtk_carray(const char* name, unsigned int dim, unsigned int len);

/** @brief Convert a vtk error code to a moordyn error
 * @param err_code The VTK error code
 * @return The MoorDyn error code
 * @see moordyn_errors_c
 */
int
vtk_error(unsigned long err_code);
#endif

} // ::io

} // ::moordyn
