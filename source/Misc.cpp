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

int
decomposeString(char outWord[10],
                char let1[10],
                char num1[10],
                char let2[10],
                char num2[10],
                char let3[10])
{
	// convert to uppercase for string matching purposes
	for (int charIdx = 0; charIdx < 10; charIdx++) {
		if (outWord[charIdx] == '\0')
			break;
		outWord[charIdx] = toupper(outWord[charIdx]);
	}

	// int wordLength = strlen(outWord);  // get length of input word (based on
	// null termination) cout << "1";
	//! find indicies of changes in number-vs-letter in characters
	unsigned int in1 =
	    strcspn(outWord, "1234567890"); // scan( OutListTmp , '1234567890' ) !
	                                    // index of first number in the string
	strncpy(let1, outWord, in1); // copy up to first number as object type
	let1[in1] = '\0';            // add null termination

	if (in1 < strlen(outWord)) // if there is a first number
	{
		// >>>>>>> the below line seems redundant - could just use in1 right???
		// <<<<<<<
		char* outWord1 =
		    strpbrk(outWord, "1234567890"); // get pointer to first number
		unsigned int il1 = strcspn(
		    outWord1,
		    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"); // in1+verify( OutListTmp(in1+1:) ,
		                                   // '1234567890' )  ! second letter
		                                   // start (assuming first character is
		                                   // a letter, i.e. in1>1)
		strncpy(num1, outWord1, il1);      // copy number
		num1[il1] = '\0';                  // add null termination

		if (il1 < strlen(outWord1)) // if there is a second letter
		{
			char* outWord2 = strpbrk(outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
			// cout << "3 il1=" << il1 << ", " ;
			unsigned int in2 =
			    strcspn(outWord2,
			            "1234567890"); // il1+scan( OutListTmp(il1+1:) ,
			                           // '1234567890' ) ! second number start
			strncpy(let2, outWord2, in2); // copy chars
			let2[in2] = '\0';             // add null termination

			if (in2 < strlen(outWord2)) // if there is a second number
			{
				char* outWord3 = strpbrk(outWord2, "1234567890");
				// cout << "4";
				unsigned int il2 = strcspn(
				    outWord3,
				    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"); // in2+verify(
				                                   // OutListTmp(in2+1:) ,
				                                   // '1234567890' )  ! third
				                                   // letter start
				strncpy(num2, outWord3, il2);      // copy number
				num2[il2] = '\0';                  // add null termination

				if (il2 < strlen(outWord3)) // if there is a third letter
				{
					char* outWord4 =
					    strpbrk(outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
					// cout << "5";
					strncpy(let3,
					        outWord4,
					        9); // copy remaining chars (should be letters)  ??
					let3[9] = '\0'; // add null termination  (hopefully takes
					                // care of case where letter D.N.E.)
				} else
					let3[0] = '\0';

			} else {
				num2[0] = '\0';
				let3[0] = '\0';
			}
		} else {
			let2[0] = '\0';
			num2[0] = '\0';
			let3[0] = '\0';
		}

	} else {
		num1[0] = '\0';
		let2[0] = '\0';
		num2[0] = '\0';
		let3[0] = '\0';

		return -1; // indicate an error because there is no number in the string
	}

	return 0;
}

} // ::moordyn::str

mat6
translateMass(vec r, mat M)
{
	// "anti-symmetric tensor components" from Sadeghi and Incecik
	mat H = getH(r);

	// break input matrix into 3x3 quadrants
	mat6 Mout;
	const mat m = M(Eigen::seqN(0, 3), Eigen::seqN(0, 3));
	Mout(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = m;

	// product of inertia matrix  [J'] = [m][H] + [J]
	const mat tempM1 = m * H;
	Mout(Eigen::seqN(3, 3), Eigen::seqN(0, 3)) = tempM1;
	Mout(Eigen::seqN(0, 3), Eigen::seqN(3, 3)) = tempM1.transpose();

	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I]
	Mout(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = H * m * H.transpose();

	return Mout;
}

mat6
translateMass6(vec r, mat6 M)
{
	// "anti-symmetric tensor components" from Sadeghi and Incecik
	mat H = getH(r);

	// break input matrix into 3x3 quadrants
	mat6 Mout;
	const mat m = M(Eigen::seqN(0, 3), Eigen::seqN(0, 3));
	Mout(Eigen::seqN(0, 3), Eigen::seqN(0, 3)) = m;
	const mat J = M(Eigen::seqN(3, 3), Eigen::seqN(0, 3));
	const mat I = M(Eigen::seqN(3, 3), Eigen::seqN(3, 3));

	// product of inertia matrix  [J'] = [m][H] + [J]
	const mat tempM1 = m * H + J;
	Mout(Eigen::seqN(3, 3), Eigen::seqN(0, 3)) = tempM1;
	Mout(Eigen::seqN(0, 3), Eigen::seqN(3, 3)) = tempM1.transpose();

	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I]
	const mat tempM2 = H * m * H.transpose(); // [H][m][H]^T
	const mat tempM3 = J.transpose() * H;     // [J]^T[H]
	const mat tempM4 = H.transpose() * J;     // [H]^T[J]
	Mout(Eigen::seqN(3, 3), Eigen::seqN(3, 3)) = tempM2 + tempM3 + tempM4 + I;

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

	// incline angle
	const real phi = atan2(l, v[2]);
	// heading of incline
	const real beta = (phi < 1.e-6) ? 0.0 : atan2(v[1], v[0]);

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
