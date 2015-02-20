/*
 * Copyright (c) 2014 Matt Hall <mtjhall@alumni.uvic.ca>
 * 
 * This file is part of MoorDyn.  MoorDyn is free software: you can redistribute 
 * it and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 * 
 * MoorDyn is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MoorDyn.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Misc.h"

using namespace std;

// simple convenience function for identity matrix
double eye(int I, int J)
{
	if (I==J) return 1.0;
	else return 0.0;
}


// return unit vector (u) in direction from r1 to r2
void unitvector( vector< double > & u, vector< double > & r1, vector< double > & r2)
{
	double length_squared = 0.0;
	
	for (int J=0; J<3; J++) length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				

	double length = sqrt(length_squared);
	
	for (int J=0; J<3; J++) u[J] = (r2[J] - r1[J]) / length; // write to unit vector
	
	return;
}


void inverse3by3( vector< vector< double > > & minv, vector< vector< double > > & m)
{		
	// computes the inverse of a matrix m
	double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
			   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
			   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

	double invdet = 1 / det;
	//cout << endl << "inverted matrix: ";
	minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet; //cout << minv[0][0];
	minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet; //cout << minv[0][1];
	minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet; //cout << minv[0][2];
	minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet; //cout << minv[1][0];
	minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet; //cout << minv[1][1];
	minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet; //cout << minv[1][2];
	minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet; //cout << minv[2][0];
	minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet; //cout << minv[2][1];
	minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet; //cout << minv[2][2] << endl;
	
}


// create rotation matrix
void RotMat( double x2, double x1, double x3, double TransMat[])
{
	// note above swapping of x1 and x2 to deal with weird coordinate system from FAST copied algorithm

	float s1 = sin(x1); 
	float c1 = cos(x1);
	float s2 = sin(x2); 
	float c2 = cos(x2);
	float s3 = sin(x3); 
	float c3 = cos(x3);
	
	//rmat = transpose([ 1 0 0; 0 c1 -s1; 0 s1 c1] * [c2 0 s2; 0 1 0; -s2 0 c2] * [c3 s3 0; -s3 c3 0; 0 0 1]);

	//rmat = [1 rz -ry; -rz 1 rx; ry -rx 1];   % rotation matrix
	
	TransMat[0] =  c1*c3 + s1*s2*s3;
	TransMat[1] =  c3*s1*s2-c1*s3;
	TransMat[2] =  c2*s1;
	TransMat[3] =  c2*s3;
	TransMat[4] =  c2*c3;
	TransMat[5] =  -s2;
	TransMat[6] =  c1*s2*s3 - c3*s1;
	TransMat[7] =  s1*s3 + c1*c3*s2;
	TransMat[8] =  c1*c2;
	
}


// computes dot product
double dotprod( vector<double>& A, vector<double>& B)
{	
	double ans = 0.;
	for (int i=0; i<A.size(); i++) ans += A[i]*B[i];	
	return ans;
}

double dotprod( double A[], vector<double>& B)
{	
	double ans = 0.;
	for (int i=0; i<B.size(); i++) ans += A[i]*B[i];	
	return ans;
}


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
		if (!item.empty())  elems.push_back(item); // skip empty ones
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}