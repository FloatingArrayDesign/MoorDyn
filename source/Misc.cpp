/*
 * Copyright (c) 2022, Matt Hall
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

#include "Misc.hpp"
#include <algorithm>
#include <fstream>

using namespace std;

namespace moordyn {

namespace str {

string
lower(const string& str)
{
	string out = str;
	transform(out.begin(), out.end(), out.begin(), ::tolower);
	return out;
}

string
upper(const string& str)
{
	string out = str;
	transform(out.begin(), out.end(), out.begin(), ::toupper);
	return out;
}

bool
startswith(const string& str, const string& prefix)
{
	return str.rfind(prefix, 0) == 0;
}

bool
has(const string& str, const vector<string> terms)
{
	for (auto term : terms) {
		if (str.find(term) != string::npos) {
			return true;
		}
	}
	return false;
}

vector<string>
split(const string& str, const char sep)
{
	stringstream spliter(str);
	string token;
	vector<string> words;
	while (std::getline(spliter, token, sep)) {
		if (token.size())
			words.push_back(token);
	}
	return words;
}

void
rtrim(std::string& s)
{
	s.erase(std::find_if(
	            s.rbegin(), s.rend(), [](char& c) { return !std::isspace(c); })
	            .base(),
	        s.end());
}

int
decomposeString(const std::string& outWord,
                std::string& let1,
                std::string& num1,
                std::string& let2,
                std::string& num2,
                std::string& let3)
{
	const std::string upperStr = upper(outWord);
	const auto end = upperStr.cend();
	std::array markers{ end, end, end, end, end };
	std::size_t marker_idx = 0;
	auto it = upperStr.cbegin();
	bool wasLastAlpha = true;
	while (it != upperStr.cend() && marker_idx < markers.size()) {
		bool isalpha = std::isalpha(*it);
		bool isnum = std::isdigit(*it);
		if (isalpha || isnum) {
			// if this char is alpha and the last one was not, or the last was
			// one alpha and this one is not
			if (isalpha != wasLastAlpha) {
				markers[marker_idx] = it;
				marker_idx++;
				wasLastAlpha = !wasLastAlpha;
			}
		}
		++it;
	}
	let1 = std::string(upperStr.cbegin(), markers[0]);
	num1 = std::string(markers[0], markers[1]);
	let2 = std::string(markers[1], markers[2]);
	num2 = std::string(markers[2], markers[3]);
	// using end here instead of markers[4] is to match the behavior of
	// decomposeString, which copies all remaining characters
	let3 = std::string(markers[3], end);
	return num1.empty() ? -1 : 0;
}

bool
isOneOf(const std::string& str,
        const std::initializer_list<const std::string> values)
{
	for (auto v : values) {
		if (str == v) {
			return true;
		}
	}
	return false;
}
} // ::moordyn::str

namespace fileIO {

std::vector<std::string>
fileToLines(const std::filesystem::path& path)
{
	std::vector<std::string> lines;
	std::ifstream file(path);
	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line)) {
			// remove any trailing whitespace from the line
			str::rtrim(line);
			lines.push_back(line);
		}
		file.close();

		return lines;
	} else {
		std::stringstream ss;
		ss << "Could not get lines of file: " << path;
		throw input_file_error(ss.str().c_str());
	}
}

} // ::moordyn::fileIO

XYZQuat
XYZQuat::operator+(const XYZQuat& visitor) const
{
	XYZQuat result;
	result.pos = this->pos + visitor.pos;
	result.quat = this->quat.coeffs() + visitor.quat.coeffs();
	return result;
}
XYZQuat
XYZQuat::operator-(const XYZQuat& visitor) const
{

	XYZQuat result;
	result.pos = this->pos - visitor.pos;
	result.quat = this->quat.coeffs() - visitor.quat.coeffs();
	return result;
}
XYZQuat
XYZQuat::operator*(const real& visitor) const
{

	XYZQuat result;
	result.pos = this->pos * visitor;
	result.quat = this->quat.coeffs() * visitor;
	return result;
}
vec6
solveMat6(const mat6& mat, const vec6& vec)
{
	// For small systems, which are larger than 4x4, we can use the
	// ColPivHouseholderQR algorithm, which is working with every single
	// matrix, retaining a very good accuracy, and becoming yet faster
	// See:
	// https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
	Eigen::ColPivHouseholderQR<mat6> solver(mat);
	return solver.solve(vec);
}

mat6
translateMass(vec r, mat M)
{
	// "anti-symmetric tensor components" from Sadeghi and Incecik
	mat H = getH(r);

	// break input matrix into 3x3 quadrants
	mat6 Mout;
	Mout.topLeftCorner<3, 3>() = M;

	// product of inertia matrix  [J'] = [m][H] + [J]
	const mat tempM1 = M * H;
	Mout.topRightCorner<3, 3>() = tempM1;
	Mout.bottomLeftCorner<3, 3>() = tempM1.transpose();

	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I]
	Mout.bottomRightCorner<3, 3>() = H * M * H.transpose();

	return Mout;
}

mat6
translateMass6(vec r, mat6 M)
{
	// "anti-symmetric tensor components" from Sadeghi and Incecik
	mat H = getH(r);

	// break input matrix into 3x3 quadrants
	mat6 Mout;
	const mat m = M.topLeftCorner<3, 3>();
	Mout.topLeftCorner<3, 3>() = m;
	const mat J = M.topRightCorner<3, 3>();
	const mat I = M.bottomRightCorner<3, 3>();

	// product of inertia matrix  [J'] = [m][H] + [J]
	const mat tempM1 = m * H + J;
	Mout.topRightCorner<3, 3>() = tempM1;
	Mout.bottomLeftCorner<3, 3>() = tempM1.transpose();

	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I]
	const mat tempM2 = H * m * H.transpose(); // [H][m][H]^T
	const mat tempM3 = J.transpose() * H;     // [J]^T[H]
	const mat tempM4 = H.transpose() * J;     // [H]^T[J]
	Mout.bottomRightCorner<3, 3>() = tempM2 + tempM3 + tempM4 + I;

	return Mout;
}

mat6
rotateMass6(mat R, mat6 M)
{
	// the process for each of the following is to
	// 1. copy out the relevant 3x3 matrix section,
	// 2. rotate it, and
	// 3. paste it into the output 6x6 matrix

	mat6 out;

	// mass matrix
	const mat m = M(Eigen::seqN(0, 3), Eigen::seqN(0, 3));
	const mat mrot = rotateMass(R, m);
	out(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = mrot;

	// product of inertia matrix
	const mat J = M(Eigen::seqN(3, 3), Eigen::seqN(0, 3));
	const mat Jrot = rotateMass(R, J);
	out(Eigen::seqN(3, 3), Eigen::seqN(0, 3)) = Jrot;
	out(Eigen::seqN(0, 3), Eigen::seqN(3, 3)) = Jrot.transpose();

	// moment of inertia matrix
	const mat I = M(Eigen::seqN(3, 3), Eigen::seqN(3, 3));
	const mat Irot = rotateMass(R, I);
	out(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = Irot;

	return out;
}

void
transformKinematics(const vec& rRelBody,
                    const mat& M,
                    const vec& r,
                    const vec6& rd,
                    vec& rOut,
                    vec& rdOut)
{
	// rd_in should be in global orientation frame
	// note: it's okay if r_out and rd_out are 6-size. Only the first 3 will be
	// written, and 4-6 will
	//       already be correct or can be assigned seperately from r_in and
	//       rd_in (assuming orientation frames are identical)

	// locations (unrotated reference frame) about platform reference point
	const vec rRel = M * rRelBody;

	// absolute locations
	rOut = rRel + r;

	// absolute velocities
	const vec v = rd(Eigen::seqN(0, 3));
	const vec w = rd(Eigen::seqN(3, 3));
	rdOut = v + w.cross(rRel);
}

std::pair<real, real>
orientationAngles(vec v)
{
	if (v.squaredNorm() < 1.e-12)
		throw nan_error("Supplied vector is near zero");
	real l = v(Eigen::seqN(0, 2)).norm();

	// inclination angle. pi/2 for horizontal vectors (v[2] = 0), growing in
	// counter-clockwise direction at the XZ plane
	const real phi = atan2(l, v[2]);
	// heading angle. 0.0 for vectors pointing towards x, growing in
	// counter-clockwise direction at the XY plane
	const real beta = (fabs(l) < 1.e-6) ? 0.0 : atan2(v[1], v[0]);

	return make_pair(phi, beta);
}

moordyn::real
GetCurvature(moordyn::real length, const vec& q1, const vec& q2)
{
	// note "length" here is combined from both segments

	auto q1_dot_q2 = q1.dot(q2);

	if (q1_dot_q2 >
	    1.0) // this is just a small numerical error, so set q1_dot_q2 to 1
		return 0.0; // this occurs when there's no curvature, so return zero
		            // curvature

	// else if (q1_dot_q2 < 0)   // this is a bend of more than 90 degrees, too
	// much, call an error! {	 //<<< maybe throwing an error is overkill,
	// could be fine?? <<<< 	throw string("Error: the angle between two
	// adjacent segments is greater than 90 degrees! (this could indicate
	// instability)"); 	return 0.0;
	// }

	// this is the normal curvature calculation
	return 4.0 / length * sqrt(0.5 * (1.0 - q1_dot_q2));
}

} // ::moordyn

/*

References

[1] K. Sadeghi and A. Incecik, “Tensor Properties of Added-mass and Damping
Coefficients,” Journal of Engineering Mathematics, vol. 52, no. 4, pp. 379–387,
Aug. 2005.

*/
