/*
 * Copyright (c) 2019 Matt Hall <mtjhall@alumni.uvic.ca>
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


int decomposeString(char outWord[10], char let1[10], 
     char num1[10], char let2[10], char num2[10], char let3[10])
{
	// convert to uppercase for string matching purposes
	for (int charIdx=0; charIdx<10; charIdx++) 
		outWord[charIdx] = toupper(outWord[charIdx]);

	//int wordLength = strlen(outWord);  // get length of input word (based on null termination)
		//cout << "1";	
	//! find indicies of changes in number-vs-letter in characters
	int in1 = strcspn( outWord, "1234567890");  // scan( OutListTmp , '1234567890' )              ! index of first number in the string
	strncpy(let1, outWord, in1);   // copy up to first number as object type
	let1[in1] = '\0';				// add null termination

	if (in1 < strlen(outWord))								// if there is a first number
	{	
	 // >>>>>>> the below line seems redundant - could just use in1 right??? <<<<<<<
		char *outWord1 = strpbrk(outWord, "1234567890");  		// get pointer to first number
		int il1 = strcspn( outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");  // in1+verify( OutListTmp(in1+1:) , '1234567890' )  ! second letter start (assuming first character is a letter, i.e. in1>1)
		strncpy(num1, outWord1, il1);   	// copy number
		num1[il1] = '\0';				// add null termination
		
		if (il1 < strlen(outWord1))							// if there is a second letter
		{	
			char *outWord2 = strpbrk(outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
			//cout << "3 il1=" << il1 << ", " ;	
			int in2 = strcspn( outWord2, "1234567890");  // il1+scan( OutListTmp(il1+1:) , '1234567890' )    ! second number start
			strncpy(let2, outWord2, in2);   // copy chars
			let2[in2] = '\0';				// add null termination
			
			if (in2 < strlen(outWord2))		// if there is a second number
			{
				char *outWord3 = strpbrk(outWord2, "1234567890");
				//cout << "4";	
				int il2 = strcspn( outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");  // in2+verify( OutListTmp(in2+1:) , '1234567890' )  ! third letter start
				strncpy(num2, outWord3, il2);   // copy number
				num2[il2] = '\0';				// add null termination
				
				if (il2 < strlen(outWord3))		// if there is a third letter
				{
					char *outWord4 = strpbrk(outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
					//cout << "5";	
					strncpy(let3, outWord4, strlen(outWord4));   // copy remaining chars (should be letters)  ??
					let3[strlen(outWord4)] = '\0';				// add null termination  (hopefully takes care of case where letter D.N.E.)
				}
				else
					let3[0] = '\0';
				
			}
			else
			{
				num2[0] = '\0';
				let3[0] = '\0';
			}
		}
		else
		{
			let2[0] = '\0';
			num2[0] = '\0';
			let3[0] = '\0';
		}

		
	}
	else
	{	
		num1[0] = '\0';
		let2[0] = '\0';
		num2[0] = '\0';
		let3[0] = '\0';	
		
		return -1;  // indicate an error because there is no number in the string
	}
	
	return 0;
}


// simple convenience function for identity matrix
double eye(int I, int J)
{
	if (I==J) return 1.0;
	else return 0.0;
}

// produce alternator matrix
void getH(double r[3], double H[3][3])
{
	H[0][1] =  r[2];
	H[1][0] = -r[2];
	H[0][2] = -r[1];
	H[2][0] =  r[1];
	H[1][2] =  r[0];
	H[2][1] = -r[0];
	for (int I=0; I<3; I++) H[I][I] = 0.0;
	
	return;
}
void getH(double r[3], double H[9])
{
	H[3*0 + 1] =  r[2];
	H[3*1 + 0] = -r[2];
	H[3*0 + 2] = -r[1];
	H[3*2 + 0] =  r[1];
	H[3*1 + 2] =  r[0];
	H[3*2 + 1] = -r[0];
	for (int I=0; I<3; I++) H[3*I + I] = 0.0;
	
	return;
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
void directionAndLength( double r1[3], double r2[3], double u[3], double* l)
{
	double length_squared = 0.0;
	
	for (int J=0; J<3; J++) length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				

	*l = sqrt(length_squared);
	
	for (int J=0; J<3; J++) u[J] = (r2[J] - r1[J]) / (*l); // write to unit vector
	
	return;
}

// transpose a 3x3 matrix
void transposeM3(double A[3][3], double Atrans[3][3])
{
	for (int I=0; I<3; I++)  
		for (int J=0; J<3; J++)  
			Atrans[I][J] = A[J][I];		
	return;
}
void transposeM3(double A[9], double Atrans[9])
{
	for (int I=0; I<3; I++)  
		for (int J=0; J<3; J++)  
			Atrans[3*I + J] = A[3*J + I];		
	return;
}

void addM6(double Min1[6][6], double Min2[6][6], double Mout[6][6])
{
	for (int I=0; I<6; I++)  
		for (int J=0; J<6; J++)  
			Mout[I][J] = Min1[I][J] + Min2[I][J];
		
	return;
}

// multiply two 3x3 matrices together: A*B = C
void multiplyM3(double A[3][3], double B[3][3], double C[3][3])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[I][J]=0.0;
			for (int k=0; k<3; k++)  
				C[I][J] += A[I][k]*B[k][J];
		}
	}
	return;	
}
void multiplyM3(double A[9], double B[9], double C[9])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[3*I + J]=0.0;
			for (int k=0; k<3; k++)  
				C[3*I + J] += A[3*I + k]*B[3*k + J];
		}
	}
	return;	
}
// multiply two 3x3 matrices, with first trasposed: [A^T]*[B] = [C]
void multiplyM3AtransB(double A[3][3], double B[3][3], double C[3][3])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[I][J]=0.0;
			for (int k=0; k<3; k++)  
				C[I][J] += A[k][I]*B[k][J];
		}
	}
	return;	
}
void multiplyM3AtransB(double A[9], double B[9], double C[9])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[3*I + J]=0.0;
			for (int k=0; k<3; k++)  
				C[3*I + J] += A[3*k + I]*B[3*k + J];
		}
	}
	return;	
}
// multiply two 3x3 matrices, with first trasposed: [A]*[B^T] = [C]
void multiplyM3ABtrans(double A[3][3], double B[3][3], double C[3][3])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[I][J]=0.0;
			for (int k=0; k<3; k++)  
				C[I][J] += A[I][k]*B[J][k];
		}
	}
	return;	
}
void multiplyM3ABtrans(double A[9], double B[9], double C[9])
{	
	for (int I=0; I<3; I++)  
	{	for (int J=0; J<3; J++)  
		{
			C[3*I + J]=0.0;
			for (int k=0; k<3; k++)  
				C[3*I + J] += A[3*I + k]*B[3*J + k];
		}
	}
	return;	
}


// return distance between double arrays r1 and r2   - also in misc.cpp for vector types
double distance3d( double* r1, double* r2)
{
	double length_squared = 0.0;
	
	for (int J=0; J<3; J++)
	//{
		length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				
	//	cout << "difference in dimension " << J << " is " << r2[J] << " - " << r1[J] << endl;
	//}
	//cout << " length is " << sqrt(length_squared) << endl;
	return sqrt(length_squared);
}


// computes dot product
double dotProd( vector<double>& A, vector<double>& B)
{	
	double ans = 0.;
	for (int i=0; i<A.size(); i++) ans += A[i]*B[i];	
	return ans;
}

double dotProd( double A[], vector<double>& B)
{	
	double ans = 0.;
	for (int i=0; i<B.size(); i++) ans += A[i]*B[i];	
	return ans;
}


void crossProd(double u[3], double v[3], double out[3])
{
	out[0] = u[1]*v[2] - u[2]*v[1];
	out[1] = u[2]*v[0] - u[0]*v[2];
	out[2] = u[0]*v[1] - u[1]*v[0];
	return;
}


void inverseM3( vector< vector< double > > & minv, vector< vector< double > > & m)
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


// LU Decomposition functions from http://www.sci.utah.edu/~wallstedt/LU.htm
// Crout uses unit diagonals for the upper triangle
void Crout(int d,double*S,double*D){
   for(int k=0;k<d;++k){
      for(int i=k;i<d;++i){
         double sum=0.;
         for(int p=0;p<k;++p)sum+=D[i*d+p]*D[p*d+k];
         D[i*d+k]=S[i*d+k]-sum; // not dividing by diagonals
      }
      for(int j=k+1;j<d;++j){
         double sum=0.;
         for(int p=0;p<k;++p)sum+=D[k*d+p]*D[p*d+j];
         D[k*d+j]=(S[k*d+j]-sum)/D[k*d+k];
      }
   }
}
void solveCrout(int d,double*LU,double*b,double*x){
   double y[d];
   for(int i=0;i<d;++i){
      double sum=0.;
      for(int k=0;k<i;++k)sum+=LU[i*d+k]*y[k];
      y[i]=(b[i]-sum)/LU[i*d+i];
   }
   for(int i=d-1;i>=0;--i){
      double sum=0.;
      for(int k=i+1;k<d;++k)sum+=LU[i*d+k]*x[k];
      x[i]=(y[i]-sum); // not dividing by diagonals
   }
}


// create rotation matrix  (row major order?)
void RotMat( double x2, double x1, double x3, double TransMat[]) // could rename to EulerToDCM
{
	// note above swapping of x1 and x2 to deal with weird coordinate system from FAST convention
	// ( x2 is roll, x1 is pitch, x3 is yaw )

	float s1 = sin(x1); 
	float c1 = cos(x1);
	float s2 = sin(x2); 
	float c2 = cos(x2);
	float s3 = sin(x3); 
	float c3 = cos(x3);
	
	//rmat = transpose([ 1 0 0; 0 c1 -s1; 0 s1 c1] * [c2 0 s2; 0 1 0; -s2 0 c2] * [c3 s3 0; -s3 c3 0; 0 0 1]);

	//rmat = [1 rz -ry; -rz 1 rx; ry -rx 1];   % rotation matrix
	
	// what follows is order Y X Z on https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
	TransMat[0] =  c1*c3 + s1*s2*s3; // [0][0]
	TransMat[1] =  c3*s1*s2-c1*s3;   // [0][1]
	TransMat[2] =  c2*s1;            // [0][2]
	TransMat[3] =  c2*s3;            // [1][0]
	TransMat[4] =  c2*c3;
	TransMat[5] =  -s2;
	TransMat[6] =  c1*s2*s3 - c3*s1;
	TransMat[7] =  s1*s3 + c1*c3*s2;
	TransMat[8] =  c1*c2;
	
}


// calculate a direction cosines matrix from a unit quaternion
void QuaternionToDCM(double q[4], double outMat[3][3])
{
	double Qi2 = q[1]*q[1];
	double Qj2 = q[2]*q[2];
	double Qk2 = q[3]*q[3];
	double QiQj = q[1]*q[2];
	double QiQk = q[1]*q[3];
	double QjQk = q[2]*q[3];
	double QrQi = q[0]*q[1];
	double QrQj = q[0]*q[2];
	double QrQk = q[0]*q[3];
	
	outMat[0][0] = 1 - 2*(Qj2+Qk2);
	outMat[0][1] = 2*(QiQj-QrQk);
	outMat[0][2] = 2*(QiQk+QrQj);
	outMat[1][0] = 2*(QiQj+QrQk);
	outMat[1][1] = 1 - 2*(Qi2+Qk2);
	outMat[1][2] = 2*(QjQk-QrQi);
	outMat[2][0] = 2*(QiQk-QrQj);
	outMat[2][1] = 2*(QjQk+QrQi);
	outMat[2][2] = 1 - 2*(Qi2+Qj2);
	return;
}

//// make a direction cosines matrix from a set of three perpendicular unit vector axes
//void AxesToDCM(double inVec[3], double outMat[3][3])
//{
//	for (int i=0; i<3; i++)
//		for (int i=0; i<3; i++)
//			outMat[i][j] = 
//}

// apply a rotation to a 3-by-3 mass matrix or any other second order tensor
void rotateM3(double Min[3][3], double rotMat[3][3], double outMat[3][3])
{
	// overall operation is [m'] = [a]*[m]*[a]^T
	
	double am[3][3] = {{0.0}};
	multiplyM3(rotMat, Min, am); // multiply rotation matrix by mass matrix
	double rotMat_trans[3][3] = {{0.0}};
	transposeM3(rotMat, rotMat_trans);
	multiplyM3(am, rotMat_trans, outMat);
	
	return;	
}
void rotateM3(double Min[9], double rotMat[9], double outMat[9])
{
	// overall operation is [m'] = [a]*[m]*[a]^T
	
	double am[9] = {0.0};
	multiplyM3(rotMat, Min, am); // multiply rotation matrix by mass matrix
	double rotMat_trans[9] = {0.0};
	transposeM3(rotMat, rotMat_trans);
	multiplyM3(am, rotMat_trans, outMat);
	
	return;	
}

// apply a rotation to a 6-by-6 mass/inertia tensor (see Sadeghi and Incecik 2005 for theory)
void rotateM6(double Min[6][6], double rotMat[3][3], double outMat[6][6])
{
	// the process for each of the following is to 
	// 1. copy out the relevant 3x3 matrix section,
	// 2. rotate it, and
	// 3. paste it into the output 6x6 matrix
		
	double tempM[3][3];
	double tempMrotated[3][3];
	
	// mass matrix
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) tempM[I][J] = Min[I][J];
	rotateM3(tempM, rotMat, tempMrotated);
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) outMat[I][J] = tempMrotated[I][J];
	
	// product of inertia matrix	
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) tempM[I][J] = Min[3+I][J];
	rotateM3(tempM, rotMat, tempMrotated);
	for (int I=0; I<3; I++) 
	{	for (int J=0; J<3; J++) 
		{	outMat[3+I][J] = tempMrotated[I][J];
			outMat[J][3+I] = tempMrotated[I][J];
		}
	}
	
	// moment of inertia matrix
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) tempM[I][J] = Min[3+I][3+J];
	rotateM3(tempM, rotMat, tempMrotated);
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) outMat[3+I][3+J] = tempMrotated[I][J];
	
	return;
}
void rotateM6(double Min[36], double rotMat[9], double outMat[36])
{
	// IMPORTANT NOTE: the 36 element vectors representing 6x6 matrics
	// are structured to go through four 3x3 quadrants in row-major order,
	// meaning the array can be split into quadrants by passing with shift
	// of 0, 9, 18, and 24.
		
	double tempM[3][3];
	double tempMrotated[3][3];
	
	// mass matrix
	rotateM3(Min, rotMat, outMat);
	
	// product of inertia matrix	
	rotateM3(Min+9, rotMat, outMat+9);
	// copy over second quadrant to third quadrant using transpose
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) outMat[18 + 3*I + J] = outMat[9 + 3*J + I];
	
	// moment of inertia matrix
	rotateM3(Min+24, rotMat, outMat+24);
	
	return;
}




void rotateVector3(double inVec[3], double rotMat[9], double outVec[3])
{
	
	// <<<<<<<<<<<<<<
	return;
}
void rotateVector6(double inVec[6], double rotMat[9], double outVec[6])
{
	
	// <<<<<<<<<<<<<<
	return;
}

// calculate position and velocity of point based on its position relative to moving 6DOF body
void transformKinematics(double rRelBody[3], double r_in[3], double TransMat[9], double rd_in[6], double rOut[3], double rdOut[3])
{
	// note: it's okay if r_out and rd_out are 6-size. Only the first 3 will be written, and 4-6 will
	//       already be correct or can be assigned seperately from r_in and rd_in (assuming orientation frames are identical)
	
	double rRel[3];
	
	// locations (unrotated reference frame) about platform reference point
	rRel[0] = TransMat[0]*rRelBody[0] + TransMat[1]*rRelBody[1] + TransMat[2]*rRelBody[2];	// x
	rRel[1] = TransMat[3]*rRelBody[0] + TransMat[4]*rRelBody[1] + TransMat[5]*rRelBody[2];	// y
	rRel[2] = TransMat[6]*rRelBody[0] + TransMat[7]*rRelBody[1] + TransMat[8]*rRelBody[2];	// z

	// absolute locations
	rOut[0] = rRel[0] + r_in[0];	// x
	rOut[1] = rRel[1] + r_in[1];	// y
	rOut[2] = rRel[2] + r_in[2];	// z

	// absolute velocities
	rdOut[0] =                        - rd_in[5]*rRel[1] + rd_in[4]*rRel[2] + rd_in[0];		// x   
	rdOut[1] =  rd_in[5]*rRel[0]	                        - rd_in[3]*rRel[2] + rd_in[1];		// y
	rdOut[2] = -rd_in[4]*rRel[0] + rd_in[3]*rRel[1]	                       + rd_in[2];		// z
		
	return;
}


void translateForce6DOF(double dx[3], double F[6], double Fout[6])
{
	double Moments[3];
	crossProd(dx, F, Moments);   // F is actually size 6 but crossProd only looks for the first 3 so this is ok
	for (int i=0; i<3; i++)
	{
		Fout[i] = F[i];
		Fout[i+3] = F[i+3] + Moments[i];  // add moments from translation to original moments
	}
	return;
}

// Takes in a position vector and a force vector (applied at the positon),
// and calculates the resulting 6-DOF force&moment vector.
void translateForce3to6DOF(double dx[3], double F[3], double Fout[6])
{	
	for (int i=0; i<3; i++)  Fout[i] = F[i];
	crossProd(dx, F, (Fout+3) ); 
	return;
}


void transformMass3to6DOF(double r[3], double TransMat[9], double Min[3][3], double Iin[3][3], double Mout[6][6])
{
	// r - vector from ref point to CG (center of Min and Iin) in UNROTATED ref frame?
	
	// rotate mass matrix
	
	// rotate inertia matrix
	
	

	// see https://aapt.scitation.org/doi/10.1119/1.4994835?? (not it's not very useful)

	//<<<<<<<<<<<<<<<<<<<<<<<<<
	
	return;
}

// turn a 3x3 mass matrix about CG to a 6x6 one about another point (-r away)
void translateMassInertia3to6DOF(double r[3], double Min[3][3], double Iin[3][3], double Mout[6][6])
{
	// Takes in mass and inertia matrices of object with relative CG position r, and 
	// calculates 6x6 mass/inertia matrix about current ref point (about which r was made).
	
	
	// <<<<<<<<< is this any good?
	
	translateMass3to6DOF(r, Min, Mout);
	
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[3+I][3+J] += Iin[I][J];
	
	/*
	double x = r[0];
	double y = r[1];
	double z = r[2];
	double xx = x*x;
	double yy = y*y;
	double zz = z*z;
	double xy = x*y;
	double yz = y*z;
	double zx = z*x;
		
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[I][J] = Min[I][J];
		
	Mout[3][3] = Iin[0][0] + Min[0][0]*( yy + zz );
	Mout[4][4] = Iin[1][1] + Min[1][1]*( zz + xx );
	Mout[5][5] = Iin[2][2] + Min[2][2]*( xx + yy );
	
	Mout[3][4] = Iin[0][1] + Min[0][0]*( -xy );
	Mout[3][5] = Iin[0][2] + Min[0][0]*( -zx );
	Mout[4][3] = Iin[1][0] + Min[0][0]*( -xy );
	Mout[4][5] = Iin[1][2] + Min[0][0]*( -yz );
	Mout[5][3] = Iin[2][0] + Min[0][0]*( -zx );
	Mout[5][4] = Iin[2][1] + Min[0][0]*( -yz );
	
	Mout[0][3] = 0.0;
	Mout[1][4] = 0.0;
	Mout[2][5] = 0.0;
	
	Mout[0][4] = Min[0][0]*( z );
	Mout[0][5] = Min[0][0]*(-y );
	Mout[1][3] = Min[0][0]*(-z );
	Mout[1][5] = Min[0][0]*( x );
	Mout[2][3] = Min[0][0]*( y );
	Mout[2][4] = Min[0][0]*(-x );
	
	Mout[3][0] = 0.0;
	Mout[4][1] = 0.0;
	Mout[5][2] = 0.0;
	
	Mout[3][1] = Min[0][0]*(-z );
	Mout[3][2] = Min[0][0]*( y );
	Mout[4][0] = Min[0][0]*( z );
	Mout[4][2] = Min[0][0]*(-x );
	Mout[5][0] = Min[0][0]*(-y );
	Mout[5][1] = Min[0][0]*( x );
	*/
	
	return;
}
	
	
// turn a 3x3 mass matrix about CG to a 6x6 one about another point (-r away)
/*
void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6])
{
	// Takes in mass matrix of object with relative CG position r, and 
	// calculates 6x6 mass/inertia matrix about current ref point (about which r was made).
	
	double x = r[0];
	double y = r[1];
	double z = r[2];
	double xx = x*x;
	double yy = y*y;
	double zz = z*z;
	double xy = x*y;
	double yz = y*z;
	double zx = z*x;
		
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[I][J] = Min[I][J];
		
	// should double check diagonal elements (if I've included everything)	<< I derived these intuitively...
	Mout[3][3] = Min[2][2]*yy + m[1][1]*zz
	Mout[4][4] = Min[0][0]*zz + m[2][2]*xx;
	Mout[5][5] = Min[1][1]*xx + m[0][0]*yy;
	
	Mout[3][4] = -Min[2][2]*xy -Min[0][1]*zz +Min[2][0]*yz +Min[1][2]*zx //xyzz <<< check signs
	Mout[4][5] =  Min[0][0]*yz -Min[1][2]*xx +Min[0][1]*zx +Min[2][0]*xy //xxyz <<< check signs
	Mout[5][3] = -Min[1][1]*zx -Min[2][0]*yy +Min[1][2]*xy +Min[0][1]*yz //xyyz 
	Mout[4][3] = Mout[3][4]
	Mout[5][4] = Mout[4][5]
	Mout[5][3] = Mout[5][3]
	
	
	Mout[0][3] =  Min[0][1]*z -Min[2][0]*y; // xyz
	Mout[1][4] = -Min[1][2]*x +Min[0][1]*z; // xyz
	Mout[2][5] = -Min[2][0]*y +Min[1][2]*x; // xyz
	
	Mout[0][4] =  Min[0][0]*z -Min[2][0]*x; // xxz
	Mout[1][5] =  Min[1][1]*x -Min[0][1]*y; // yyx
	Mout[2][3] =  Min[2][2]*y -Min[1][2]*z; // zzy
	
	Mout[0][5] = -Min[0][0]*y +Min[0][1]*x; // xxy
	Mout[1][3] = -Min[1][1]*z +Min[1][2]*y; // yyz
	Mout[2][4] = -Min[2][2]*x +Min[2][0]*z; // zzx
	
	
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[3+I][J] = Mout[J][3+I];	
	return;
}
*/


void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6])
{

	double H[3][3];      // "anti-symmetric tensor components" from Sadeghi and Incecik
	getH(r,H);
	
	double tempM[3][3];
	double tempM2[3][3];
	double Htrans[3][3];
	
	// mass matrix  [m'] = [m]
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[I][J] = Min[I][J];
		
	// product of inertia matrix  [J'] = [m][H] + [J]
	multiplyM3(Min, H, tempM);
	for (int I=0; I<3; I++) 
	{	for (int J=0; J<3; J++) 
		{	Mout[3+I][J] = tempM[I][J];
			Mout[J][3+I] = tempM[I][J];
		}
	}
	
	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T [H] + [H]^T [J] + [I]
	multiplyM3(H, Min, tempM);
	transposeM3(H,Htrans);
	multiplyM3(tempM, Htrans, tempM2);
	for (int I=0; I<3; I++) 
		for (int J=0; J<3; J++) 
			Mout[3+I][3+J] = tempM2[I][J];
		
	return;
	
}
void translateMass6to6DOF(double r[3], double Min[6][6], double Mout[6][6])
{
	
	double H[3][3];      // "anti-symmetric tensor components" from Sadeghi and Incecik
	getH(r,H);
	
	// break input matrix into 3x3 quadrants
	
	double Min3[3][3]; 
	double tempM[3][3];
	double tempM2[3][3];
	double tempM3[3][3];
	double tempM4[3][3];
	
	// mass matrix  [m'] = [m]
	for (int I=0; I<3; I++)
	{	for (int J=0; J<3; J++)
		{	Mout[I][J] = Min[I][J];
			Min3[I][J] = Min[I][J];
		}
	}
		
	// product of inertia matrix  [J'] = [m][H] + [J]
	multiplyM3(Min3, H, tempM);
	for (int I=0; I<3; I++) 
	{	for (int J=0; J<3; J++) 
		{	Mout[3+I][J] = tempM[I][J] + Min[3+I][J];
			Mout[J][3+I] = tempM[J][I] + Min[J][3+I];
		}
	}
	
	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I] 
	
	double mm[3][3];
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) mm[I][J] = Min[I][J]; 
	multiplyM3(H, mm, tempM);
	multiplyM3ABtrans(tempM, H, tempM2);   // [H][m][H]^T
	
	double mJ[3][3];
	for (int I=0; I<3; I++) for (int J=0; J<3; J++) mJ[I][J] = Min[3+I][J];
	multiplyM3AtransB(mJ, H, tempM3);   //  [J]^T[H]
	multiplyM3AtransB(H, mJ, tempM4);   //  [H]^T[J]	
	
	for (int I=0; I<3; I++) 
		for (int J=0; J<3; J++) 
			// [I']        = [H][m][H]^T  + [J]^T[H]    + [H]^T[J]    + [I] 
			Mout[3+I][3+J] = tempM2[I][J] +tempM3[I][J] +tempM4[I][J] + Min[3+I][3+J];
		
	return;
		
}
void translateMass6to6DOF(double r[3], double Min[36], double Mout[36])
{
	double H[9];      // "anti-symmetric tensor components" from Sadeghi and Incecik
	getH(r,H);
	
	double tempM[9];
	double tempM2[9];
	double tempM3[9];
	double tempM4[9];
	
	// mass matrix  [m'] = [m]
	for (int I=0; I<3; I++)
		for (int J=0; J<3; J++)
			Mout[3*I + J] = Min[3*I + J];

		
	// product of inertia matrix  [J'] = [m][H] + [J]
	multiplyM3(Min, H, tempM);
	for (int I=0; I<3; I++) 
	{	for (int J=0; J<3; J++) 
		{	Mout[9 + 3*I + J] = tempM[3*I + J] + Min[9 + 3*I + J];
			Mout[18+ 3*J + I] = tempM[3*J + I] + Min[18+ 3*J + I];
		}
	}
	
	// moment of inertia matrix  [I'] = [H][m][H]^T + [J]^T[H] + [H]^T[J] + [I]  
	multiplyM3(H, Min, tempM);
	multiplyM3ABtrans(tempM, H, tempM2);   // [H][m][H]^T
	multiplyM3AtransB(Min+9, H, tempM3);   //  [J]^T[H]
	multiplyM3AtransB(H, Min+9, tempM4);   //  [H]^T[J]	
	for (int I=0; I<3; I++) 
		for (int J=0; J<3; J++) 
			Mout[27 + 3*I + J] = tempM2[3*I + J] + tempM3[3*I + J] + tempM4[3*I + J] + Min[27 + 3*I + J];
		
	return;


}




//std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
//{
//    std::stringstream ss(s);
//    std::string item;
//    while (std::getline(ss, item, delim)) {
//		if (!item.empty())  elems.push_back(item); // skip empty ones
//    }
//    return elems;
//}
//
//std::vector<std::string> split(const std::string &s, char delim) 
//{
//    std::vector<std::string> elems;
//    split(s, delim, elems);
//    return elems;
//}


// new better string splitting function
vector<string> split(const string &s, char delim)  // TODO: remove delim arg, it's unused!
{
	vector<string> elems;  // the vector of words to return
    
	char str[200];  // this gives some memory for the char array
	strncpy(str, s.c_str(), 200);  // copy input string to str
	char * pch;
	pch = strtok (str," \t");  // give strtok the c string of s
	while (pch != NULL)
	{
		elems.push_back(pch);
		pch = strtok (NULL, " \t");  // split by spaces or tabs
	}
	return elems;
}



// ==================== some filtering functions used in OC4 study =============================

// flip or reverse function
void reverse(double* data, int datasize)
{
	int i;
	double temp;
	for (i = 0; i < floor(datasize/2); i++) 
	{
		temp = data[i];
		data[i] = data[datasize-1-i];
		data[datasize-1-i] = temp;
	}
	return;
}


// IIR filter function
void doIIR(double* in, double* out, int dataSize, double* a, double* b, int kernelSize)
{	// kernelSize is size of vectors a and b, which contain the coefficients.  Normally a[0]=1.

	int i, k;
	
	// filtering from out[0] to out[kernelSize-2]
	for(i = 0; i < kernelSize - 1; ++i)
	{
		out[i] = in[kernelSize-1];					// maybe this hack will work...
	
//		out[i] = in[i]*b[0];                             // using just part of a filter kernel doesn't really work
//		for(k = 1; k < i+1; k++)
//			out[i] += in[i-k]*b[k] - out[i-k]*a[k];
//		out[i] = out[i]/a[0];
	}
	
	// filtering from out[kernelSize-1] to out[dataSize-1] (last)
	for(i = kernelSize-1; i < dataSize; i++)
	{
		out[i] = in[i]*b[0];                             
		for(k = 1; k < kernelSize; k++)
			out[i] += in[i-k]*b[k] - out[i-k]*a[k];
		out[i] = out[i]/a[0];
	}
	return;
}


// State Space filter function - a good reference is https://ccrma.stanford.edu/~jos/fp/Converting_State_Space_Form_Hand.html
void doSSfilter(double* in, double* out, int dataSize, double* a, double* beta, double b0, int kernelSize)
{	// kernelSize is size of vectors a and beta (exluding 0th entry!), which contain the coefficients.

	int i, k;
	double* fstates = (double*) malloc( dataSize*sizeof(double) ); // create state vector
	
	// filtering from out[0] to out[kernelSize-2]
	for(i = 0; i < kernelSize; ++i)
	{
		out[i] = in[kernelSize];			// maybe this hack will work...
		fstates[i] = 0.0;
	}
	
	// filtering from out[kernelSize] to out[dataSize-1] (last)
	for(i = kernelSize; i < dataSize; i++)
	{ 
		fstates[i] = in[i-1];
		out[i] = b0*in[i];
		for(k = 0; k < kernelSize; k++)
			fstates[i] += -1.0*a[k]*fstates[i-1-k];
			
		for(k = 0; k < kernelSize; k++)
			out[i] += beta[k]*fstates[i-k];
	}
	
	free(fstates);
	return;
}


// 2D double array creation and destruction functions
double** make2Darray(int arraySizeX, int arraySizeY) 
{
	double** theArray;  
	theArray = (double**) malloc(arraySizeX*sizeof(double*));  
	for (int i = 0; i < arraySizeX; i++)  
		theArray[i] = (double*) malloc(arraySizeY*sizeof(double));  
	return theArray;  
}   

void free2Darray(double** theArray, int arraySizeX)
{
	for (int i = 0; i < arraySizeX; i++)
		free(theArray[i]);
	free(theArray);
}

/*

References

[1] K. Sadeghi and A. Incecik, “Tensor Properties of Added-mass and Damping Coefficients,” 
Journal of Engineering Mathematics, vol. 52, no. 4, pp. 379–387, Aug. 2005.


*/

