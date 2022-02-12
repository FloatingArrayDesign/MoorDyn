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

#ifndef MISC_H
#define MISC_H

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <complex>


#include <fstream>
#include <sstream>
#include <cstring>

#include <memory>

//#ifdef USEGL
// #include <GL/gl.h>  // for openGL drawing option
// #include <GL/glu.h> // used in arrow function
//#endif

#include "kiss_fftr.h"  // used for any wave kinematics functions

#ifdef OSX
 #include <sys/uio.h>
#elif defined LINUX

#else
 #include <windows.h>  // these are for guicon function RedirectIOToConsole
 #include <io.h>
#endif

#include <stdio.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

// note: this file contains the struct definitions for environmental and line/connect properties


// from Iñaki Zabala
#ifdef _MSC_VER
template<typename T> static inline T round(T val) {return floor(val + 0.5);}
#endif

using namespace std;

typedef complex<double> doubleC; 		// make shorthand for complex double type
typedef complex<float> floatC; 		// make shorthand for complex float type

const double pi = 3.14159265;
const double rad2deg = 57.29577951;

const doubleC i1(0., 1.); 			// set imaginary number 1
//const floatC i1f(0., 1.); 			// set imaginary number 1

const int wordy = 2;   			// flag to enable excessive output (if > 0) for troubleshooting

const int nCoef = 30;   // maximum number of entries to allow in nonlinear coefficient lookup tables

typedef struct 
{
	double g;
	double WtrDpth;
	double rho_w;
	
	double kb;       // bottom stiffness (Pa/m)
	double cb;       // bottom damping   (Pa/m/s)
	int WaveKin;      // wave kinematics flag (0=off, >0=on...)<<<
	int Current;      // current flag (0=off, >0=on...)<<<
	double dtWave;   // time step used to downsample wave elevation data with
	int WriteUnits;	// a global switch for whether to show the units line in the output files (1, default), or skip it (0)
	int writeLog;   // whether to write a log file. (0=no, 1=basic, 2=full description, 3=ongoing output
	ofstream *outfileLogPtr;
	double FrictionCoefficient; // general bottom friction coefficient, as a start
	double FricDamp; // a damping coefficient used to model the friction at speeds near zero
	double StatDynFricScale; // a ratio of static to dynamic friction ( = mu_static/mu_dynamic)
	
} EnvCond;



typedef struct  // (matching Line Dictionary inputs)
{
	string type; 
	double d;
	double w;		// linear weight in air
	double EA;
	double EI;
	double c;    	// internal damping
	double cI;
	double Can;
	double Cat;
	double Cdn;
	double Cdt;	
	int nEApoints;        // number of values in stress-strain lookup table (0 means using constant E)
	double stiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double stiffYs[nCoef]; // y array for stress-strain lookup table
	int nCpoints;        // number of values in stress-strainrate lookup table (0 means using constant c)
	double dampXs[nCoef]; // x array for stress-strainrate lookup table (up to nCoef)
	double dampYs[nCoef]; // y array for stress-strainrate lookup table	
	int nEIpoints = 0; // number of values in bending stress-strain lookup table (0 means using constant E)
	double bstiffXs[nCoef]; // x array for stress-strain lookup table (up to nCoef)
	double bstiffYs[nCoef]; // y array for stress-strain lookup table
} LineProps;

typedef struct  // (matching Rod Dictionary inputs)
{
	string type; 
	double d;
	double w;		// linear weight in air
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
} RodProps;

typedef struct // matching node input stuff
{
	int number;
	string type;
	double X;
	double Y;
	double Z;
	double M;
	double V;
	double FX;
	double FY;
	double FZ;
	double CdA;  // added 2015/1/15 - product of drag coefficient and frontal area
	double Ca;  // added 2015/1/15  - added mass coefficient
} ConnectProps;

typedef struct // matching body input stuff
{
	int number;
	string type;
	double X0; // constants set at startup from input file
	double Y0;
	double Z0;
	double Xcg;
	double Ycg;
	double Zcg;
	double M;
	double V;
	double IX;
	double IY;
	double IZ;
	double CdA;
	double Ca;	
} BodyProps;

typedef struct     // for failure conditions
{
	int attachID;         // ID of connection or Rod the lines are attached to (index is -1 this value)
	int isRod;             // 1 Rod end A, 2 Rod end B, 0 if connection
	int lineIDs[30];        // array of one or more lines to detach (starting from 1...)
	int lineTops[30];       // an array that will be FILLED IN to return which end of each line was disconnected ... 1 = top/fairlead(end B), 0 = bottom/anchor(end A)
	int nLinesToDetach;  // how many lines to dettach
	double failTime;
	double failTen;        // N
	int failStatus;    // 0 not failed yet, 1 failed
} FailProps;

typedef struct 
{  // this is C version of MDOutParmType - a less literal alternative of the NWTC OutParmType for MoorDyn (to avoid huge lists of possible output channel permutations)                                                                         
	char Name[10]; 		// "name of output channel"   
	char Units[10];			// "units string"     - should match UnitsSize in MoorDyn.cpp (should turn into def)                                                                            
	int QType;     		// "type of quantity - 0=tension, 1=x, 2=y, 3=z..."                                         
	int OType;     		// "type of object - 1=line, 2=connect"                                                                              
	int NodeID;    		// "node number if OType=1.  0=anchor, -1=N=Fairlead"                                                              
	int ObjID;     		// "number of Connect or Line object", subtract 1 to get the index in the LineList or ConnectList
} OutChanProps;


  // --------------------------- Output definitions -----------------------------------------

  // The following are some definitions for use with the output options in MoorDyn.
  // These are for the global output quantities specified by OutList, not line-specific outputs.
  // Output definitions follow the structure described by the MD_OutParmType .
  // Each output channel is described by the following fields:
  //  Name   - (string) what appears at the top of the output column
  //  Units  - (string) selected from UnitList (see below) based on index QType
  //  OType  - (int) the type of object the output is from. 1=line, 2=connect (0=invalid)
  //  ObjID  - (int) the ID number of the line or connect
  //  QType  - (int) the type of quantity to output.  0=tension, 1=x pos, etc.  see the parameters below
  //  NodeID - (int) the ID number of the node of the output quantity
  // 
  // These are the "OTypes": 0=Connect object, 1=Line Object
  // (will just use 0 and 1 rather than parameter names)
  // 
  // Indices for computing output channels:  - customized for the MD_OutParmType approach
  // these are the "QTypes"
 
	  const int Time    =    0 ;
	  const int PosX    =    1 ;
	  const int PosY    =    2 ;
	  const int PosZ    =    3 ;
	  const int VelX    =    4 ;
	  const int VelY    =    5 ;
	  const int VelZ    =    6 ;
	  const int AccX    =    7 ;
	  const int AccY    =    8 ;
	  const int AccZ    =    9 ;
	  const int Ten     =    10;
	  const int FX      =    11;
	  const int FY      =    12;
	  const int FZ      =    13;

	  // UnitList is in MoorDyn.cpp
	  
	  
	  
	  //vector<string> strvector(strarray, strarray + 3);
	  
// // List of units corresponding to the quantities parameters for QTypes
//  struct Units 
// {
//	  char Time[10]    = "(s)      ";
//	  char PosX[10]    = "(m)      ";
//	  char PosY[10]    = "(m)      ";
//	  char PosZ[10]    = "(m)      ";
//	  char VelX[10]    = "(m/s)    ";
//	  char VelY[10]    = "(m/s)    ";
//	  char VelZ[10]    = "(m/s)    ";
//	  char AccX[10]    = "(m/s2)   ";
//	  char AccY[10]    = "(m/s2)   ";
//	  char AccZ[10]    = "(m/s2)   ";
//	  char Ten [10]    = "(N)      ";
//	  char FX  [10]    = "(N)      ";
//	  char FY  [10]    = "(N)      ";
//	  char FZ  [10]    = "(N)      ";
// };




// below are function prototypes for misc functions

// TODO: replace duplicates for different data types with templated functions <<< (or remove vector types)

void interpArray(int ndata, int n, double *xdata, double *ydata, double *xin, double *yout);
void interpArray(int ndata, int n, vector<double> xdata, vector<double> ydata, vector<double> xin, vector<double> &yout);

double calculate4Dinterpolation(double**** f, int ix0, int iy0, int iz0, int it0, double fx, double fy, double fz, double ft);
double calculate3Dinterpolation(double*** f, int ix0, int iy0, int iz0, double fx, double fy, double fz);
double calculate2Dinterpolation(double** f, int ix0, int iy0, double fx, double fy);
int getInterpNums(double *xlist, int nx, double xin, double *fout);
void getInterpNums(double *xlist, int nx, double xin, double fout[2], int iout[2]);

double GetCurvature(double length, double q1[3], double q2[3]);

void GetOrientationAngles(double p1[3], double p2[3], double* phi, double* sinPhi, double* cosPhi, 
                          double* tanPhi, double* beta, double* sinBeta, double* cosBeta);

int decomposeString(char outWord[10], char let1[10], 
     char num1[10], char let2[10], char num2[10], char let3[10]);

double eye(int I, int J);

void getH(double r[3], double H[3][3]);
void getH(double r[3], double H[9]);

double unitvector( vector< double > & u, vector< double > & r1, vector< double > & r2);
double unitvector( double u[3], vector< double > & r1, vector< double > & r2);
double unitvector( double u[3], double r1[3], double r2[3]);

void transposeM3(double A[3][3], double Atrans[3][3]);
void transposeM3(double A[9], double Atrans[9]);

void addM6(double Min1[6][6], double Min2[6][6], double Mout[6][6]);

void multiplyM3(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3(double A[9], double B[9], double C[9]);

void multiplyM3AtransB(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3AtransB(double A[9], double B[9], double C[9]);

void multiplyM3ABtrans(double A[3][3], double B[3][3], double C[3][3]);
void multiplyM3ABtrans(double A[9], double B[9], double C[9]);

double distance3d( double* r1, double* r2);

double dotProd( vector<double>& A, vector<double>& B);
double dotProd( double A[], vector<double>& B);
double dotProd( double A[], double B[]);

void scalevector( vector< double > & u, double newlength, vector< double > & y);
void scalevector( double u[3], double newlength, double y[3]);

void crossProd(double u[3], double v[3], double out[3]);
void crossProd(vector<double>& u, vector<double>& v, double out[3]);
void crossProd(vector<double>& u, double v[3], double out[3]);

void inverse3by3( vector< vector< double > > & minv, vector< vector< double > > & m);

void Crout(int d,double*S,double*D);
void solveCrout(int d,double*LU,double*b,double*x);

template <typename TwoD1, typename TwoD2>
void LUsolve(int n, TwoD1& A, TwoD2& LU, double*b, double *y, double*x)
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

// void LUsolve(int n, double **A,double **LU, double*b, double *y, double*x);
void LUsolve3(double A[3][3], double x[3], double b[3]);
void LUsolve6(double A[6][6], double x[6], double b[6]);

void RotMat( double x1, double x2, double x3, double TransMat[]);

void QuaternionToDCM(double q[4], double outMat[3][3]);

void rotateM3(double Min[3][3], double rotMat[3][3], double outMat[3][3]);
void rotateM3(double Min[9], double rotMat[9], double outMat[9]);

void rotateM6(double Min[6][6], double rotMat[3][3], double outMat[6][6]);
void rotateM6(double Min[36], double rotMat[9], double outMat[36]);

void rotateVector3(double inVec[3], double rotMat[9], double outVec[3]);
void rotateVector6(double inVec[6], double rotMat[9], double outVec[6]);

void transformKinematics(double rRelBody[3], double r_in[3], double TransMat[9], double rd_in[6], double rOut[3], double rdOut[3]);
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], vector< double > &rOut, vector< double > &rdOut);
void transformKinematicsAtoB(double rA[3], double u[3], double L, double rd_in[6], double rOut[3], double rdOut[3]);

void translateForce6DOF(double dx[3], double F[6], double Fout[6]);

void translateForce3to6DOF(double dx[3], double F[3], double Fout[6]);

void transformMass3to6DOF(double r[3], double TransMat[9], double Min[3][3], double Iin[3][3], double Mout[6][6]);

void transformMass3to6DOF(double r[3], double TransMat[9], double Min[3][3], double Iin[3][3], double Mout[6][6]);

void translateMassInertia3to6DOF(double r[3], double Min[3][3], double Iin[3][3], double Mout[6][6]);

//void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6]);
void translateMass3to6DOF(double r[3], double Min[3][3], double Mout[6][6]);

void translateMass6to6DOF(double r[3], double Min[6][6], double Mout[6][6]);
void translateMass6to6DOF(double r[3], double Min[36], double Mout[36]);

vector<string> split(const string &s);
vector<string> splitBar(const string &s);
vector<string> splitComma(const string &s);

void reverse(double* data, int datasize);
void doIIR(double* in, double* out, int dataSize, double* a, double* b, int kernelSize);
void doSSfilter(double* in, double* out, int dataSize, double* a, double* beta, double b0, int kernelSize);

double*    make1Darray(int n1);
double**   make2Darray(int n1, int n2);
double***  make3Darray(int n1, int n2, int n3);
double**** make4Darray(int n1, int n2, int n3, int n4);
void free2Darray(double**   theArray, int n1);
void free3Darray(double***  theArray, int n1, int n2);
void free4Darray(double**** theArray, int n1, int n2, int n3);


#endif
