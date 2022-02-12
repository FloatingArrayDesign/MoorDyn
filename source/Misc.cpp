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


// interpolate an array of data, x values must be increasing
void interpArray(int ndata, int n, double *xdata, double *ydata, double *xin, double *yout)
{
	int id = 0; // counter for where we're at in the x and y data arrays
	
	for (int i=0; i<n; i++)
	{
		while (xin[i] > xdata[id])
			id++;
		
		if (id > ndata)
			yout[i] = ydata[ndata-1]; // quick fix: for now if we go past the data just use the last value
		else
			yout[i] = ydata[id-1] + ( xin[i] - xdata[id-1] )/( xdata[id] - xdata[id-1] )*( ydata[id] - ydata[id-1] );
		
		// should also check for things like non-ordered data! <<<<<<<<<
	}
	return;
}
void interpArray(int ndata, int n, vector<double> xdata, vector<double> ydata, vector<double> xin, vector<double> &yout)
{
	int id = 0; // counter for where we're at in the x and y data arrays
	
	for (int i=0; i<n; i++)
	{
		while (xin[i] > xdata[id])
			id++;
		
		if (id > ndata)
			yout[i] = ydata[ndata-1]; // quick fix: for now if we go past the data just use the last value
		else
			yout[i] = ydata[id-1] + ( xin[i] - xdata[id-1] )/( xdata[id] - xdata[id-1] )*( ydata[id] - ydata[id-1] );
		
		// should also check for things like non-ordered data! <<<<<<<<<
	}
	return;
}







double calculate4Dinterpolation(double**** f, int ix0, int iy0, int iz0, int it0, double fx, double fy, double fz, double ft)
{		
	int ix1, iy1, iz1, it1;
	
	// handle end case conditions
	if (fx == 0) ix1 = ix0;
	else         ix1 = ix0+1;
	if (fy == 0) iy1 = iy0;
	else         iy1 = iy0+1;
	if (fz == 0) iz1 = iz0;
	else         iz1 = iz0+1;
	if (ft == 0) it1 = it0;
	else         it1 = it0+1;
	
	
	double c000 = f[ix0][iy0][iz0][it0]*(1-ft) + f[ix0][iy0][iz0][it1]*ft;
	double c001 = f[ix0][iy0][iz1][it0]*(1-ft) + f[ix0][iy0][iz1][it1]*ft;
	double c010 = f[ix0][iy1][iz0][it0]*(1-ft) + f[ix0][iy1][iz0][it1]*ft;
	double c011 = f[ix0][iy1][iz1][it0]*(1-ft) + f[ix0][iy1][iz1][it1]*ft;
	double c100 = f[ix1][iy0][iz0][it0]*(1-ft) + f[ix1][iy0][iz0][it1]*ft;
	double c101 = f[ix1][iy0][iz1][it0]*(1-ft) + f[ix1][iy0][iz1][it1]*ft;
	double c110 = f[ix1][iy1][iz0][it0]*(1-ft) + f[ix1][iy1][iz0][it1]*ft;
	double c111 = f[ix1][iy1][iz1][it0]*(1-ft) + f[ix1][iy1][iz1][it1]*ft;
	
	double c00 = c000*(1-fx) + c100*fx; 
	double c01 = c001*(1-fx) + c101*fx; 
	double c10 = c010*(1-fx) + c110*fx; 
	double c11 = c011*(1-fx) + c111*fx; 

	double c0  = c00 *(1-fy) + c10 *fy;
	double c1  = c01 *(1-fy) + c11 *fy;

	double c   = c0  *(1-fz) + c1  *fz;
	
	return c;
}	


double calculate3Dinterpolation(double*** f, int ix0, int iy0, int iz0, double fx, double fy, double fz)
{		
	// note that "z" could also be "t" - dimension names are arbitrary

	int ix1, iy1, iz1;
	
	if (fx == 0) ix1 = ix0;
	else         ix1 = ix0+1;
	if (fy == 0) iy1 = iy0;
	else         iy1 = iy0+1;
	if (fz == 0) iz1 = iz0;
	else         iz1 = iz0+1;
		

	double c000 = f[ix0][iy0][iz0];
	double c001 = f[ix0][iy0][iz1];
	double c010 = f[ix0][iy1][iz0];
	double c011 = f[ix0][iy1][iz1];
	double c100 = f[ix1][iy0][iz0];
	double c101 = f[ix1][iy0][iz1];
	double c110 = f[ix1][iy1][iz0];
	double c111 = f[ix1][iy1][iz1];
	
	double c00 = c000*(1-fx) + c100*fx; 
	double c01 = c001*(1-fx) + c101*fx; 
	double c10 = c010*(1-fx) + c110*fx; 
	double c11 = c011*(1-fx) + c111*fx; 

	double c0  = c00 *(1-fy) + c10 *fy;
	double c1  = c01 *(1-fy) + c11 *fy;

	double c   = c0  *(1-fz) + c1  *fz;
	
	return c;
}	


double calculate2Dinterpolation(double** f, int ix0, int iy0, double fx, double fy)
{	
	int ix1, iy1;
	
	if (fx == 0) ix1 = ix0;
	else         ix1 = ix0+1;
	if (fy == 0) iy1 = iy0;
	else         iy1 = iy0+1;
	
	
	double c00 = f[ix0][iy0]; 
	double c01 = f[ix0][iy1]; 
	double c10 = f[ix1][iy0]; 
	double c11 = f[ix1][iy1]; 

	double c0  = c00 *(1-fx) + c10 *fx;
	double c1  = c01 *(1-fx) + c11 *fx;

	double c   = c0  *(1-fy) + c1  *fy;
	
	return c;
}	


/*
interpLayer(double frac_in, int n_in, double* xs_in, double* ys_out, int dim_in, int dim_max      )
{
	
	for (int ts=0; ts<Nt-1; ts++)			// loop through precalculated wave time series time steps  (start ts at ts0 to save time)
	{	if (tTS[ts+1] > t) 				// moving precalculated time bracked "up".  Stop once upper end of bracket is greater than current time t.
		{
			ts0 = ts;
			frac = ( t - tTS[ts] )/( tTS[ts+1] - tTS[ts] );
			break;
		}
	}
	
	for (int i=0; i<n_in; i++)  // loop through passed coordinate values, get functions values for each
	{
		
	}
}

template <typename ptr>
interpLayer( ptr ys1[], ptr ys2[], int nin, int n, double *xs, double **pxs
{
	// inputs:  nin    number of dimensions in (ys1/2 arrays size is 2^nin)
	//          n      total number of dimensions
	//          ys1  n-d array slice(s) of y data to be passed to next lower level
	//          ys2  n-d array slice(s) of y data to be passed to next lower level  <<< eventually this should be a list of pointers to ALL data needing interpolation
	//          *xs  x values of remaining dimensions
	//          *pxs array of pointers to x data arrays (px, py, pz etc)
	
	// from x_, find i_, f_
	
	i1 = 
	f  =
	
	if f==0,	i2=i1
	
	// call next layer   with n-1, x+1
	interpLayer( ys1[i1 nin+1, xs+1, 
	
	// compute y_

}
	
	
	
	
	
template <typename ptr>
interpLayer(  int n, int *is, double *fs, 
{
	// inputs:  n    dimension we're at
	//          *is  interpolation indices of dimensions
	//          *fs  interpolation fractions of dimensions
	//          d
	//
	// outputs: f
	//          d
	//          d
	//          d
	
	
	// from x_, find i_, f_
	
	// call next layer 
	interpLayer( n1, is+1, fs+1 
	
	// compute y_

}
*/	


// calculate the lower index and the fraction to use for interpolating
int getInterpNums(double *xlist, int nx, double xin, double *fout)
{
	// Parameters: list of x values, number of x values, x value to be interpolated, fraction to return
	// Returns the lower index to interpolate from.  such that  y* = y[i] + fout*(y[i+1]-y[i])
	int i;
	
	if (xin <= xlist[0])            // below lowest data point
	{	i = 0;
		*fout = 0.0;
	}
	else if (xin >= xlist[nx-1])    // above higher data point
	{	i = nx-1;
		*fout = 0.0;
	}
	else                            // within the data range
	{	for (i=0; i<nx-1; i++)
		{	if (xlist[i+1] > xin)
			{
				*fout = (xin - xlist[i] )/( xlist[i+1] - xlist[i] );
				break;
			}
		}		
	}		
	
	return i;
}
void getInterpNums(double *xlist, int nx, double xin, double fout[2], int iout[2])
{
	// This more general/versatile version writes two indices (i, i+1) and two scalers (1-f, f), thereby handling bounds in one step
	
	// Parameters: list of x values, number of x values, x value to be interpolated, fractions to return, indices to return
	// Such that  y* = y[iout[0]]*fout[0] + y[iout[1]]*fout[1]
	
	if (xin <= xlist[0])
	{	iout[0] = 0;     iout[1] = 0;
		fout[0] = 1.0;   fout[1] = 0.0;
	}
	else if (xin >= xlist[nx-1])
	{	iout[0] = nx-1;  iout[1] = nx-1;
		fout[0] = 1.0;   fout[1] = 0.0;
	}
	else
	{	for (int i=0; i<nx-1; i++)
		{	if (xlist[i+1] > xin)
			{
				iout[0] = i;  iout[1] = i+1;
				
				fout[1] = (xin - xlist[i] )/( xlist[i+1] - xlist[i] );
				fout[0] = 1.0 - fout[1];
				break;
			}
		}		
	}		
	
	return;
}






// convenience function to calculate curvature based on adjacent segments' direction vectors and their combined length
double GetCurvature(double length, double q1[3], double q2[3])
{
	// note "length" here is combined from both segments
	
	double q1_dot_q2 = dotProd( q1, q2 );
	
	if (q1_dot_q2 > 1.0)    // this is just a small numerical error, so set q1_dot_q2 to 1
		return 0.0;          // this occurs when there's no curvature, so return zero curvature

	//else if (q1_dot_q2 < 0)   // this is a bend of more than 90 degrees, too much, call an error!
	//{	 //<<< maybe throwing an error is overkill, could be fine?? <<<<
	//	throw string("Error: the angle between two adjacent segments is greater than 90 degrees! (this could indicate instability)");
	//	return 0.0;
	//}
	else                        // normal case
		return 4.0/length * sqrt(0.5*(1.0 - q1_dot_q2));   // this is the normal curvature calculation
}


// calculate orientation angles of a cylindrical object
void GetOrientationAngles(double p1[3], double p2[3], double* phi, double* sinPhi, double* cosPhi, 
                          double* tanPhi, double* beta, double* sinBeta, double* cosBeta)
{
    // p1 and p2 are the end A and end B coordinate of the rod
	
    double vec[3];
	double vecLen;
	double vecLen2D;
	double k_hat[3]; // output unit vector no longer used <<<

	// calculate isntantaneous incline angle and heading, and related trig values
	// the first and last NodeIndx values point to the corresponding Joint nodes idices which are at the start of the Mesh
	for (int i=0; i<3; i++)  vec[i] = p2[i] - p1[i];   
	
	vecLen   = sqrt(dotProd(vec,vec));
	vecLen2D = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
	if ( vecLen < 0.000001 )  {
		cout << "ERROR in GetOrientationAngles in MoorDyn" << endl;
		cout << p1 << endl;
		cout << p2 << endl;
		k_hat[0] = 1.0/0.0;
	}
	else  { 
		for (int i=0; i<3; i++)  k_hat[i] = vec[i] / vecLen; 
		*phi   = atan2(vecLen2D, vec[2]);   // incline angle   
	}
	if ( *phi < 0.000001)  
		*beta = 0.0;
	else
		*beta = atan2(vec[1], vec[0]);    // heading of incline     
	
	*sinPhi  = sin(*phi);
	*cosPhi  = cos(*phi);
	*tanPhi  = tan(*phi); 
	*sinBeta = sin(*beta);
	*cosBeta = cos(*beta);
		
	return;
}


// split a string into separate letter strings and integers
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

// calculate unit vector (u) in direction from r1 to r2. Also returns distance between points. 
double unitvector( vector< double > & u, vector< double > & r1, vector< double > & r2)
{
	double length_squared = 0.0;	
	for (int J=0; J<3; J++) length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				

	double length = sqrt(length_squared);	
	for (int J=0; J<3; J++) u[J] = (r2[J] - r1[J]) / length; // write to unit vector
	
	return length;
}
double unitvector( double u[3], vector< double > & r1, vector< double > & r2)
{
	double length_squared = 0.0;	
	for (int J=0; J<3; J++) length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				

	double length = sqrt(length_squared);	
	for (int J=0; J<3; J++) u[J] = (r2[J] - r1[J]) / length; // write to unit vector
	
	return length;
}
double unitvector( double u[3], double r1[3], double r2[3])
{
	double length_squared = 0.0;	
	for (int J=0; J<3; J++) length_squared += (r2[J] - r1[J])*(r2[J] - r1[J]);				

	double length = sqrt(length_squared);	
	for (int J=0; J<3; J++) u[J] = (r2[J] - r1[J]) / length; // write to unit vector
	
	return length;
}

// scale vector to length
void scalevector( vector< double > & u, double newlength, vector< double > & y)
{
	double length_squared = 0.0;	
	for (int J=0; J<3; J++) length_squared += u[J]*u[J];				

	double scaler;
	if (length_squared > 0)
		scaler = newlength/sqrt(length_squared);	
	else                   // if original vector is zero, return zero
		scaler = 0.0;
	
	for (int J=0; J<3; J++) y[J] = u[J] * scaler;
	
	return;
}
void scalevector( double u[3], double newlength, double y[3])
{
	double length_squared = 0.0;	
	for (int J=0; J<3; J++) length_squared += u[J]*u[J];				

	double scaler;
	if (length_squared > 0)
		scaler = newlength/sqrt(length_squared);	
	else                   // if original vector is zero, return zero
		scaler = 0.0;
	
	for (int J=0; J<3; J++) 
	{
		double temp = u[J] * scaler;   // not sure this temp switching is necessary (was just if u and y were the same)
		y[J] = temp;
	}
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
double dotProd( double A[3], double B[3])
{	
	double ans = 0.;
	for (int i=0; i<3; i++) ans += A[i]*B[i];	
	return ans;
}


void crossProd(double u[3], double v[3], double out[3])
{
	out[0] = u[1]*v[2] - u[2]*v[1];
	out[1] = u[2]*v[0] - u[0]*v[2];
	out[2] = u[0]*v[1] - u[1]*v[0];
	return;
}
void crossProd(vector<double>& u, vector<double>& v, double out[3])
{
	out[0] = u[1]*v[2] - u[2]*v[1];
	out[1] = u[2]*v[0] - u[0]*v[2];
	out[2] = u[0]*v[1] - u[1]*v[0];
	return;
}
void crossProd(vector<double>& u, double v[3], double out[3])
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

// One-function implementation of Crout LU Decomposition with 2D array inputs
// adapted from http://www.sci.utah.edu/~wallstedt/LU.htm .
/*
void LUsolve(int n, double **A,double **LU, double*b, double *y, double*x)
{
	// Solves Ax=b for x
	// LU contains LU matrices, y is a temporary vector
	// all dimensions are n
	
   for(int k=0; k<n; ++k)
	{
      for(int i=k; i<n; ++i)
		{
         double sum=0.;
			
         for(int p=0; p<k; ++p)
				sum += LU[i][p]*LU[p][k];
			
         LU[i][k] = A[i][k]-sum; // not dividing by diagonals
      }
      for(int j=k+1;j<n;++j)
		{
         double sum=0.;
         for(int p=0;p<k;++p)
				sum += LU[k][p]*LU[p][j];
         
			LU[k][j] = (A[k][j]-sum)/LU[k][k];
      }
   }
	
   for(int i=0; i<n; ++i)
	{
      double sum=0.;
      for(int k=0; k<i; ++k)
			sum += LU[i][k]*y[k];
      
		y[i] = (b[i]-sum)/LU[i][i];
   }
   for(int i=n-1; i>=0; --i)
	{
      double sum=0.;
      for(int k=i+1; k<n; ++k)
			sum += LU[i][k]*x[k];
      
		x[i] = (y[i]-sum); // not dividing by diagonals
   }
}
*/
void LUsolve3(double A[3][3], double x[3], double b[3])
{
	// Solves Ax=b for x, with size 3
		
	double LU[3][3];
	double y[3];
		
   for(int k=0; k<3; ++k)
	{
      for(int i=k; i<3; ++i)
		{
         double sum=0.;
			
         for(int p=0; p<k; ++p)
				sum += LU[i][p]*LU[p][k];
			
         LU[i][k] = A[i][k]-sum; // not dividing by diagonals
      }
		
      for(int j=k+1;j<3;++j)
		{
         double sum=0.;
         for(int p=0;p<k;++p)
				sum += LU[k][p]*LU[p][j];
         
			LU[k][j] = (A[k][j]-sum)/LU[k][k];
      }
   }
	
   for(int i=0; i<3; ++i)
	{
      double sum=0.;
      for(int k=0; k<i; ++k)
			sum += LU[i][k]*y[k];
      
		y[i] = (b[i]-sum)/LU[i][i];
   }
	
   for(int i=3-1; i>=0; --i)
	{
      double sum=0.;
      for(int k=i+1; k<3; ++k)
			sum += LU[i][k]*x[k];
      
		x[i] = (y[i]-sum); // not dividing by diagonals
   }
}
void LUsolve6(double A[6][6], double x[6], double b[6])
{
	// Solves Ax=b for x, with size 6
	
	double LU[6][6];
	double y[6];
		
   for(int k=0; k<6; ++k)
	{
      for(int i=k; i<6; ++i)
		{
         double sum=0.;
			
         for(int p=0; p<k; ++p)
				sum += LU[i][p]*LU[p][k];
			
         LU[i][k] = A[i][k]-sum; // not dividing by diagonals
      }
		
      for(int j=k+1;j<6;++j)
		{
         double sum=0.;
         for(int p=0;p<k;++p)
				sum += LU[k][p]*LU[p][j];
         
			LU[k][j] = (A[k][j]-sum)/LU[k][k];
      }
   }
	
   for(int i=0; i<6; ++i)
	{
      double sum=0.;
      for(int k=0; k<i; ++k)
			sum += LU[i][k]*y[k];
      
		y[i] = (b[i]-sum)/LU[i][i];
   }
	
   for(int i=6-1; i>=0; --i)
	{
      double sum=0.;
      for(int k=i+1; k<6; ++k)
			sum += LU[i][k]*x[k];
      
		x[i] = (y[i]-sum); // not dividing by diagonals
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
	for (int I=0; I<3; I++)  
	{
		outVec[I] = 0.0;
		
		for (int J=0; J<3; J++)  
			outVec[I] += rotMat[3*I+J]*inVec[J];  // <<< check index order
	}
	return;
}

void rotateVector6(double inVec[6], double rotMat[9], double outVec[6])
{
	for (int I=0; I<3; I++)  
	{
		outVec[  I] = 0.0;
		outVec[3+I] = 0.0;
		
		for (int J=0; J<3; J++)  
		{	outVec[   I] += rotMat[3*I+J]*inVec[  J];  // <<< check index order
			outVec[3+I] += rotMat[3*I+J]*inVec[3+J];
		}
	}
	return;
}

// calculate position and velocity of point based on its position relative to moving 6DOF body
void transformKinematics(double rRelBody[3], double r_in[3], double TransMat[9], double rd_in[6], double rOut[3], double rdOut[3])
{
	// rd_in should be in global orientation frame
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


// calculate position and velocity of point based on its position relative to moving 6DOF body
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], vector< double > &rOut, vector< double > &rdOut)
{
	// rA is coordinate of end A, u is unit vector and L is length along it to end B (which is where outputs are calculated)
	// rd_in should be in global orientation frame
	// note: it's okay if r_out and rd_out are 6-size. Only the first 3 will be written, and 4-6 will
	//       already be correct or can be assigned seperately from r_in and rd_in (assuming orientation frames are identical)
		
	// locations (unrotated reference frame)
	double rRel[3];
	for (int j=0; j<3; j++)
	{
		rRel[j] = L*u[j];             // relative location of point B from point A
		rOut[j] = rRel[j] + rA[j];	// absolute location of point B
	}

	// absolute velocities
	rdOut[0] =                        - rd_in[5]*rRel[1] + rd_in[4]*rRel[2] + rd_in[0];		// x   
	rdOut[1] =  rd_in[5]*rRel[0]	                        - rd_in[3]*rRel[2] + rd_in[1];		// y
	rdOut[2] = -rd_in[4]*rRel[0] + rd_in[3]*rRel[1]	                       + rd_in[2];		// z
		
	return;
}
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], double rOut[3], double rdOut[3])
{
	// rA is coordinate of end A, u is unit vector and L is length along it to end B (which is where outputs are calculated)
	// rd_in should be in global orientation frame
	// note: it's okay if r_out and rd_out are 6-size. Only the first 3 will be written, and 4-6 will
	//       already be correct or can be assigned seperately from r_in and rd_in (assuming orientation frames are identical)
		
	// locations (unrotated reference frame)
	double rRel[3];
	for (int j=0; j<3; j++)
	{
		rRel[j] = L*u[j];             // relative location of point B from point A
		rOut[j] = rRel[j] + rA[j];	// absolute location of point B
	}

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
	multiplyM3(H, Min, tempM);    // could just do a transpose of old tempM here intead.. <<<
	transposeM3(H,Htrans);           // is this also bypassable? <<<
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
	{	for (int J=0; J<3; J++) 
		{	// [I']        = [H][m][H]^T  + [J]^T[H]    + [H]^T[J]    + [I] 
			Mout[3+I][3+J] = tempM2[I][J] +tempM3[I][J] +tempM4[I][J] + Min[3+I][3+J];
		}
	}
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
vector<string> split(const string &s)
{
	vector<string> elems;  // the vector of words to return
    
	char str[200];  // this gives some memory for the char array
	strncpy(str, s.c_str(), 200);  // copy input string to str (to avoid strtok modifying the input string)
	char * pch;
	pch = strtok (str, " \t");  // give strtok the c string of s
	while (pch != NULL)
	{
		elems.push_back(pch);
		pch = strtok (NULL, " \t");  // split by spaces or tabs
	}
	return elems;
}
vector<string> splitBar(const string &s)
{
	vector<string> elems;  // the vector of words to return
    
	char str[200];  // this gives some memory for the char array
	strncpy(str, s.c_str(), 200);  // copy input string to str (to avoid strtok modifying the input string)
	char * pch;
	pch = strtok (str, "|");  // give strtok the c string of s
	while (pch != NULL)
	{
		elems.push_back(pch);
		pch = strtok (NULL, "|");  // split by spaces or tabs
	}
	return elems;
}

vector<string> splitComma(const string &s)  // this one splits at commas
{
	vector<string> elems;  // the vector of words to return
    
	char str[200];  // this gives some memory for the char array
	strncpy(str, s.c_str(), 200);  // copy input string to str (to avoid strtok modifying the input string)
	char * pch;
	pch = strtok (str, ",");  // give strtok the c string of s
	while (pch != NULL)
	{
		elems.push_back(pch);
		pch = strtok (NULL, ",");  // split by spaces or tabs
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


// 2D,3D,4D double array creation and destruction functions (these could easily be nested) <<< should  inittialize all with zeros!
double** make2Darray(int n1, int n2) 
{
	double** theArray;  
	theArray = (double**) malloc(n1*sizeof(double*));  
	for (int i1 = 0; i1 < n1; i1++)  
		theArray[i1] = (double*) malloc(n2*sizeof(double));  
	return theArray;  
}
double*** make3Darray(int n1, int n2, int n3) 
{
	double*** theArray;  
	theArray = (double***) malloc(n1*sizeof(double**));  
	for (int i1 = 0; i1 < n1; i1++)  
	{	theArray[i1] = (double**) malloc(n2*sizeof(double*));
		for (int i2 = 0; i2 < n2; i2++)  
			theArray[i1][i2] = (double*) malloc(n3*sizeof(double));	
	}	
	return theArray;  
}
double**** make4Darray(int n1, int n2, int n3, int n4) 
{
	double**** theArray;  
	theArray = (double****) malloc(n1*sizeof(double***));  
	for (int i1 = 0; i1 < n1; i1++)  
	{	theArray[i1] = (double***) malloc(n2*sizeof(double**));
		for (int i2 = 0; i2 < n2; i2++)  
		{	theArray[i1][i2] = (double**) malloc(n3*sizeof(double*));	
			for (int i3 = 0; i3 < n3; i3++)  
				theArray[i1][i2][i3] = (double*) malloc(n4*sizeof(double));	
		}
	}
	return theArray;  
}  
double* make1Darray(int n1)  // because I'm forgetful and this is easier than remembering malloc 
{
	double* theArray = (double*) malloc(n1*sizeof(double));  
	return theArray;  
} 

void free2Darray(double** theArray, int n1)
{
	for (int i1 = 0; i1 < n1; i1++)
		free(theArray[i1]);
	free(theArray);
}
void free3Darray(double*** theArray, int n1, int n2)
{
	for (int i1 = 0; i1 < n1; i1++)		
	{	for (int i2 = 0; i2 < n2; i2++)
		{
			free(theArray[i1][i2]);
		}
		free(theArray[i1]);
	}
	free(theArray);
}
void free4Darray(double**** theArray, int n1, int n2, int n3)
{
	for (int i1 = 0; i1 < n1; i1++)		
	{	for (int i2 = 0; i2 < n2; i2++)
		{	for (int i3 = 0; i3 < n3; i3++)
			{	
				free(theArray[i1][i2][i3]);
			}
			free(theArray[i1][i2]);
		}
		free(theArray[i1]);
	}
	free(theArray);
}

/*

References

[1] K. Sadeghi and A. Incecik, “Tensor Properties of Added-mass and Damping Coefficients,” 
Journal of Engineering Mathematics, vol. 52, no. 4, pp. 379–387, Aug. 2005.


*/

