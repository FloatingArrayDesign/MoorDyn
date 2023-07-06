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

#include "IO.hpp"
#include <climits>
#include <iostream>
#include <fstream>
#include <stdlib.h>

namespace moordyn {

namespace io {

// NOTE: When building with the legacy system there is no major and minor
// definitions, so we are adding dummy ones. This will be removed in the future
#ifndef MOORDYN_MAJOR_VERSION
#define MOORDYN_MAJOR_VERSION 0
#endif
#ifndef MOORDYN_MINOR_VERSION
#define MOORDYN_MINOR_VERSION 0
#endif

/** @brief Pack a floating point number into an integer
 *
 * This operation can be reverted with unpack754(). However, the result is still
 * non-portable, since the resulting integer must have different endianness
 * @param f The floating point number
 * @param bits The number of bits of the mantissa
 * @param expbits The number of bits of the exponent
 * @return Packed number
 * @see unpack754()
 * @see is_big_endian()
 * @see swap_endian
 */
long long
pack754(long double f, unsigned bits, unsigned expbits)
{
	long double fnorm;
	int shift;
	long long sign, exp, significand;
	unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	if (f == 0.0)
		return 0; // get this special case out of the way

	// check sign and begin normalization
	if (f < 0) {
		sign = 1;
		fnorm = -f;
	} else {
		sign = 0;
		fnorm = f;
	}

	// get the normalized form of f and track the exponent
	shift = 0;
	while (fnorm >= 2.0) {
		fnorm /= 2.0;
		shift++;
	}
	while (fnorm < 1.0) {
		fnorm *= 2.0;
		shift--;
	}
	fnorm = fnorm - 1.0;

	// calculate the binary form (non-float) of the significand data
	significand = fnorm * ((1LL << significandbits) + 0.5f);

	// get the biased exponent
	exp = shift + ((1 << (expbits - 1)) - 1); // shift + bias

	// return the final answer
	return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
}

/** @brief Unpack a floating point number from an integer
 *
 * This operation is the inverse of pack754()
 * @param i The packed integer
 * @param bits The number of bits of the mantissa
 * @param expbits The number of bits of the exponent
 * @return The unpacket floating point number
 * @see pack754()
 * @see is_big_endian()
 * @see swap_endian
 */
long double
unpack754(long long i, unsigned bits, unsigned expbits)
{
	long double result;
	long long shift;
	unsigned bias;
	unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	if (i == 0)
		return 0.0;

	// pull the significand
	result = (i & ((1LL << significandbits) - 1)); // mask
	result /= (1LL << significandbits);            // convert back to float
	result += 1.0f;                                // add the one back on

	// deal with the exponent
	bias = (1 << (expbits - 1)) - 1;
	shift = ((i >> significandbits) & ((1LL << expbits) - 1)) - bias;
	while (shift > 0) {
		result *= 2.0;
		shift--;
	}
	while (shift < 0) {
		result /= 2.0;
		shift++;
	}

	// sign it
	result *= (i >> (bits - 1)) & 1 ? -1.0 : 1.0;

	return result;
}

/// Pack IEEE754 32bits float numbers
#define pack754_32(f) (pack754((f), 32, 8))
/// Pack IEEE754 64bits double numbers
#define pack754_64(f) (pack754((f), 64, 11))
/// Unpack IEEE754 32bits float numbers
#define unpack754_32(i) (unpack754((i), 32, 8))
/// Unpack IEEE754 64bits double numbers
#define unpack754_64(i) (unpack754((i), 64, 11))

/** @brief CHeck if the platform is big endian
 * @return true for big endian platforms, false otherwise
 */
bool
is_big_endian(void)
{
	union
	{
		uint32_t i;
		char c[4];
	} bint = { 0x01020304 };

	return bint.c[0] == 1;
}

/** @brief Swap the endianess
 * @param u The value to get transformed
 * @return the endian-swaped value
 */
template<typename T>
T
swap_endian(T u)
{
	static_assert(CHAR_BIT == 8, "CHAR_BIT != 8");

	union
	{
		T u;
		unsigned char u8[sizeof(T)];
	} source, dest;

	source.u = u;

	for (size_t k = 0; k < sizeof(T); k++)
		dest.u8[k] = source.u8[sizeof(T) - k - 1];

	return dest.u;
}

IO::IO(moordyn::Log* log)
  : LogUser(log)
  , _is_big_endian(false)
  , _min_major_version(2)
  , _min_minor_version(2)
{
	_is_big_endian = is_big_endian();
	if (_min_major_version <= MOORDYN_MAJOR_VERSION) {
		_min_major_version = MOORDYN_MAJOR_VERSION;
		if (_min_minor_version <= MOORDYN_MINOR_VERSION)
			_min_minor_version = MOORDYN_MINOR_VERSION;
	}
}

IO::~IO() {}

void
IO::Save(const std::string filepath)
{
	ofstream f(filepath, ios::out | ios::binary);
	if (!f) {
		LOGERR << "The file '" << filepath << "' cannot be written" << endl;
		throw moordyn::output_file_error("Invalid file");
	}
	// Write some magic number
	const uint8_t major = MOORDYN_MAJOR_VERSION;
	const uint8_t minor = MOORDYN_MINOR_VERSION;
	f.write("MoorDyn", 7 * sizeof(char));
	f.write((char*)&major, sizeof(uint8_t));
	f.write((char*)&minor, sizeof(uint8_t));
	// Produce the data
	std::vector<uint64_t> data = Serialize();
	// Save the total size, which is simplifying the reading process
	const uint64_t size = data.size();
	f.write((char*)&size, sizeof(uint64_t));
	for (auto v : data) {
		f.write((char*)&v, sizeof(uint64_t));
	}
	f.close();
}

void
IO::Load(const std::string filepath)
{
	ifstream f(filepath, ios::in | ios::binary);
	if (!f) {
		LOGERR << "The file '" << filepath << "' cannot be read" << endl;
		throw moordyn::input_file_error("Invalid file");
	}
	// Check that it is a MoorDyn file and the version is acceptable
	f.seekg(0, ios::end);
	const uint64_t fsize = f.tellg();
	f.seekg(0, ios::beg);
	const uint64_t header_size =
	    7 * sizeof(char) + 2 * sizeof(uint8_t) + sizeof(uint64_t);
	if (fsize < header_size) {
		LOGERR << "The file '" << filepath
		       << "' is too small to be a MoorDyn file" << endl;
		throw moordyn::input_file_error("Invalid file");
	}
	char magic[8];
	magic[7] = '\0';
	f.read(magic, 7 * sizeof(char));
	if (strcmp(magic, "MoorDyn")) {
		LOGERR << "The file '" << filepath << "' is not a MoorDyn file" << endl;
		throw moordyn::input_file_error("Invalid file");
	}
	uint8_t major, minor;
	f.read((char*)&major, sizeof(uint8_t));
	f.read((char*)&minor, sizeof(uint8_t));
	if ((major < _min_major_version) ||
	    ((major == _min_major_version) && (minor < _min_minor_version))) {
		LOGERR << "The file '" << filepath << "' was written by MoorDyn "
		       << major << "." << minor << ", but >= " << _min_major_version
		       << "." << _min_minor_version << " is required" << endl;
		throw moordyn::input_file_error("Invalid file");
	}
	// Check that the amount of information is correct
	uint64_t length;
	f.read((char*)&length, sizeof(uint64_t));
	const uint64_t size = length * sizeof(uint64_t);
	if (size != fsize - header_size) {
		LOGERR << fsize - header_size
		       << " bytes of data are available in file '" << filepath
		       << "' but " << size << " bytes are declared" << endl;
		throw moordyn::input_error("Invalid size");
	}

	// Read the data and close the file, which we do not need anymore
	uint64_t* data = (uint64_t*)malloc(size);
	if (!data) {
		LOGERR << "Failure allocating the " << size << " bytes to read '"
		       << filepath << "'" << endl;
		throw moordyn::mem_error("Allocation error");
	}
	f.read((char*)data, size);
	f.close();

	// So do the unpacking job
	const uint64_t* end = Deserialize(data);
	if (data + length != end) {
		const uint64_t l = end - data;
		LOGERR << l * sizeof(uint64_t) << " bytes (vs. " << size
		       << " bytes expected) unpacked from '" << filepath << "'" << endl;
		throw moordyn::mem_error("Allocation error");
	}

	free(data);
}

uint64_t
IO::Serialize(const uint64_t& i)
{
	// Almost everyone uses little endian nowadays
	if (_is_big_endian)
		return swap_endian(i);
	return i;
}

uint64_t
IO::Serialize(const int64_t& i)
{
	uint64_t* ui = (uint64_t*)&i;
	return Serialize((uint64_t)(*ui));
}

uint64_t
IO::Serialize(const real& f)
{
	return Serialize((int64_t)pack754_64((double)f));
}

std::vector<uint64_t>
IO::Serialize(const vec& m)
{
	std::vector<uint64_t> data;
	data.reserve(m.size());
	for (unsigned int i = 0; i < 3; i++)
		data.push_back(Serialize(m(i)));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const vec6& m)
{
	std::vector<uint64_t> data;
	data.reserve(m.size());
	for (unsigned int i = 0; i < 6; i++)
		data.push_back(Serialize(m(i)));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const mat& m)
{
	std::vector<uint64_t> data;
	data.reserve(m.size());
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
			data.push_back(Serialize(m(i, j)));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const mat6& m)
{
	std::vector<uint64_t> data;
	data.reserve(m.size());
	for (unsigned int i = 0; i < 6; i++)
		for (unsigned int j = 0; j < 6; j++)
			data.push_back(Serialize(m(i, j)));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const quaternion& m)
{
	std::vector<uint64_t> data;
	auto coeffs = m.coeffs();
	data.reserve(4);
	for (unsigned int i = 0; i < 4; i++)
		data.push_back(Serialize(coeffs(i)));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const XYZQuat& m)
{
	std::vector<uint64_t> data = Serialize(m.pos);
	auto subdata = Serialize(m.quat);
	data.insert(data.end(), subdata.begin(), subdata.end());
	return data;
}

std::vector<uint64_t>
IO::Serialize(const std::vector<real>& l)
{
	std::vector<uint64_t> data;
	const uint64_t n = l.size();
	data.reserve(1 + l.size());
	data.push_back(Serialize(n));
	for (auto v : l)
		data.push_back(Serialize(v));
	return data;
}

std::vector<uint64_t>
IO::Serialize(const std::vector<vec>& l)
{
	std::vector<uint64_t> data;
	const uint64_t n = l.size();
	data.reserve(1 + 3 * l.size());
	data.push_back(Serialize(n));
	for (auto v : l) {
		auto subdata = Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

std::vector<uint64_t>
IO::Serialize(const std::vector<vec6>& l)
{
	std::vector<uint64_t> data;
	const uint64_t n = l.size();
	data.reserve(1 + 6 * l.size());
	data.push_back(Serialize(n));
	for (auto v : l) {
		auto subdata = Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

std::vector<uint64_t>
IO::Serialize(const std::vector<mat>& l)
{
	std::vector<uint64_t> data;
	const uint64_t n = l.size();
	data.reserve(1 + 9 * l.size());
	data.push_back(Serialize(n));
	for (auto v : l) {
		auto subdata = Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

std::vector<uint64_t>
IO::Serialize(const std::vector<mat6>& l)
{
	std::vector<uint64_t> data;
	const uint64_t n = l.size();
	data.reserve(1 + 36 * l.size());
	data.push_back(Serialize(n));
	for (auto v : l) {
		auto subdata = Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
	}
	return data;
}

uint64_t*
IO::Deserialize(const uint64_t* in, uint64_t& out)
{
	if (_is_big_endian)
		out = swap_endian(*in);
	else
		out = *in;
	return (uint64_t*)in + 1;
}

uint64_t*
IO::Deserialize(const uint64_t* in, int64_t& out)
{
	uint64_t uout;
	uint64_t* remaining = Deserialize(in, uout);
	out = *((int64_t*)(&uout));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, real& out)
{
	uint64_t uout;
	uint64_t* remaining = Deserialize(in, uout);
	out = (real)unpack754_64(uout);
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, vec& out)
{
	uint64_t* remaining = (uint64_t*)in;
	for (unsigned int i = 0; i < 3; i++)
		remaining = Deserialize(remaining, out(i));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, vec6& out)
{
	uint64_t* remaining = (uint64_t*)in;
	for (unsigned int i = 0; i < 6; i++)
		remaining = Deserialize(remaining, out(i));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, mat& out)
{
	uint64_t* remaining = (uint64_t*)in;
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
			remaining = Deserialize(remaining, out(i, j));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, mat6& out)
{
	uint64_t* remaining = (uint64_t*)in;
	for (unsigned int i = 0; i < 6; i++)
		for (unsigned int j = 0; j < 6; j++)
			remaining = Deserialize(remaining, out(i, j));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, quaternion& out)
{
	uint64_t* remaining = (uint64_t*)in;
	for (unsigned int i = 0; i < 4; i++)
		remaining = Deserialize(remaining, out.coeffs()(i));
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, XYZQuat& out)
{
	uint64_t* remaining = Deserialize(in, out.pos);
	remaining = Deserialize(remaining, out.quat);
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, std::vector<real>& out)
{
	uint64_t n;
	uint64_t* remaining = Deserialize(in, n);
	out.clear();
	out.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		real v;
		remaining = Deserialize(remaining, v);
		out.push_back(v);
	}
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, std::vector<vec>& out)
{
	uint64_t n;
	uint64_t* remaining = Deserialize(in, n);
	out.clear();
	out.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		vec v;
		remaining = Deserialize(remaining, v);
		out.push_back(v);
	}
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, std::vector<vec6>& out)
{
	uint64_t n;
	uint64_t* remaining = Deserialize(in, n);
	out.clear();
	out.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		vec6 v;
		remaining = Deserialize(remaining, v);
		out.push_back(v);
	}
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, std::vector<mat>& out)
{
	uint64_t n;
	uint64_t* remaining = Deserialize(in, n);
	out.clear();
	out.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		mat v;
		remaining = Deserialize(remaining, v);
		out.push_back(v);
	}
	return remaining;
}

uint64_t*
IO::Deserialize(const uint64_t* in, std::vector<mat6>& out)
{
	uint64_t n;
	uint64_t* remaining = Deserialize(in, n);
	out.clear();
	out.reserve(n);
	for (unsigned int i = 0; i < n; i++) {
		mat6 v;
		remaining = Deserialize(remaining, v);
		out.push_back(v);
	}
	return remaining;
}

#ifdef USE_VTK
vtkSmartPointer<vtkFloatArray>
vtk_farray(const char* name, unsigned int dim, unsigned int len)
{
	vtkSmartPointer<vtkFloatArray> a = vtkSmartPointer<vtkFloatArray>::New();
	a->SetName(name);
	a->SetNumberOfComponents(dim);
	a->SetNumberOfTuples(len);
	return a;
}

vtkSmartPointer<vtkCharArray>
vtk_carray(const char* name, unsigned int dim, unsigned int len)
{
	vtkSmartPointer<vtkCharArray> a = vtkSmartPointer<vtkCharArray>::New();
	a->SetName(name);
	a->SetNumberOfComponents(dim);
	a->SetNumberOfTuples(len);
	return a;
}

int
vtk_error(unsigned long err_code)
{
	switch (err_code) {
		case vtkErrorCode::NoError:
			return MOORDYN_SUCCESS;
		case vtkErrorCode::FileNotFoundError:
		case vtkErrorCode::CannotOpenFileError:
		case vtkErrorCode::NoFileNameError:
			return MOORDYN_INVALID_OUTPUT_FILE;
		case vtkErrorCode::UnrecognizedFileTypeError:
		case vtkErrorCode::FileFormatError:
			return MOORDYN_INVALID_VALUE;
		case vtkErrorCode::OutOfDiskSpaceError:
			return MOORDYN_MEM_ERROR;
		default:
			return MOORDYN_UNHANDLED_ERROR;
	}
}
#endif

} // ::io

} // ::moordyn
