/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
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

/** @file io.cpp
 * Tests on the input/output interface, moordyn::io
 */

#include "IO.hpp"
#include <vector>

using namespace moordyn;

#define LISTS_LENGTH 32
#define COMPARE_LISTS(v1, v2)                                                  \
	if (v1.size() != v2.size())                                                \
		return false;                                                          \
	for (unsigned int i = 0; i < v1.size(); i++)                               \
		if (v1[i] != v2[i])                                                    \
			return false;

class IOTester : public io::IO
{
  public:
	IOTester(Log* log)
	  : IO(log)
	  , i(-3)
	  , ui(1024)
	  , r(37.431)
	  , v({ 0.0, 1.0, 2.0 })
	  , v6({ 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 })
	  , m({ { 3.0, 4.5, -8.0 },
	        { 10.0, -256.0, -1024.341 },
	        { 32.4, 55.7, 812309765.2 } })
	  , m6({ { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 },
	         { 3.0, 4.5, -8.0, 10.0, -256.0, -1024.341 } })
	{
		for (unsigned int i = 0; i < LISTS_LENGTH; i++) {
			lv.push_back(v);
			lv6.push_back(v6);
			lm.push_back(m);
			lm6.push_back(m6);
		}
	}

	~IOTester() {}

	void clear()
	{
		i = 0;
		ui = 0;
		r = 0.0;
		v = vec::Zero();
		v6 = vec6::Zero();
		m = mat::Zero();
		m6 = mat6::Zero();
		lv.clear();
		lv6.clear();
		lm.clear();
		lm6.clear();
	}

	virtual std::vector<uint64_t> Serialize(void)
	{
		std::vector<uint64_t> data, subdata;
		const int64_t ii = i;
		data.push_back(io::IO::Serialize((int64_t)i));
		data.push_back(io::IO::Serialize((uint64_t)ui));
		data.push_back(io::IO::Serialize(r));
		subdata = io::IO::Serialize(v);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(v6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(m);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(m6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lv);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lv6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lm);
		data.insert(data.end(), subdata.begin(), subdata.end());
		subdata = io::IO::Serialize(lm6);
		data.insert(data.end(), subdata.begin(), subdata.end());
		return data;
	}

	virtual uint64_t* Deserialize(const uint64_t* data)
	{
		uint64_t* ptr = (uint64_t*)data;
		int64_t ii;
		ptr = io::IO::Deserialize(ptr, ii);
		i = (int)ii;
		int64_t uii;
		ptr = io::IO::Deserialize(ptr, uii);
		ui = (unsigned int)ii;
		ptr = io::IO::Deserialize(ptr, r);
		ptr = io::IO::Deserialize(ptr, v);
		ptr = io::IO::Deserialize(ptr, v6);
		ptr = io::IO::Deserialize(ptr, m);
		ptr = io::IO::Deserialize(ptr, m6);
		ptr = io::IO::Deserialize(ptr, lv);
		ptr = io::IO::Deserialize(ptr, lv6);
		ptr = io::IO::Deserialize(ptr, lm);
		ptr = io::IO::Deserialize(ptr, lm6);
		return ptr;
	}

	IOTester& operator=(const IOTester& visitor)
	{
		i = visitor.i;
		ui = visitor.ui;
		r = visitor.r;
		v = visitor.v;
		v6 = visitor.v6;
		m = visitor.m;
		m6 = visitor.m6;
		lv = visitor.lv;
		lv6 = visitor.lv6;
		lm = visitor.lm;
		lm6 = visitor.lm6;
		return *this;
	}

	bool operator==(const IOTester& visitor)
	{
		if (i != visitor.i)
			return false;
		if (ui != visitor.ui)
			return false;
		if (r != visitor.r)
			return false;
		if (v != visitor.v)
			return false;
		if (v6 != visitor.v6)
			return false;
		if (m != visitor.m)
			return false;
		if (m6 != visitor.m6)
			return false;
		COMPARE_LISTS(lv, visitor.lv);
		COMPARE_LISTS(lv6, visitor.lv6);
		COMPARE_LISTS(lm, visitor.lm);
		COMPARE_LISTS(lm6, visitor.lm6);
		return true;
	}

  private:
	int i;
	unsigned int ui;
	moordyn::real r;
	moordyn::vec v;
	moordyn::vec6 v6;
	moordyn::mat m;
	moordyn::mat6 m6;
	std::vector<moordyn::vec> lv;
	std::vector<moordyn::vec6> lv6;
	std::vector<moordyn::mat> lm;
	std::vector<moordyn::mat6> lm6;
};

int
main(int, char**)
{
	// First basic test, check that we can serialize and deserialize
	cout << "Serialize -> Deserialize..." << endl;
	Log dummy_log;
	IOTester src(&dummy_log), dst(&dummy_log);
	dst.clear();
	if (src == dst) {
		cerr << "src == dst from the beggining?!?!?" << endl;
		return 1;
	}
	auto data = src.Serialize();
	dst.Deserialize(data.data());
	if (src == dst) {
		cerr << "The deserialized data does not match the original" << endl;
		return 1;
	}
	cout << "  OK!" << endl;

	return 0;
}
