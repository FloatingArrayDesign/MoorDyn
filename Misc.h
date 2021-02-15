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

#include "kiss_fft.h"  // used for any wave kinematics functions

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

const double pi=3.14159265;

const doubleC i1(0., 1.); 			// set imaginary number 1
const floatC i1f(0., 1.); 			// set imaginary number 1

const int wordy = 0;   			// flag to enable excessive output (if > 0) for troubleshooting


typedef struct 
{
	double g;
	double WtrDpth;
	double rho_w;
	
	double kb;       // bottom stiffness (Pa/m)
	double cb;       // bottom damping   (Pa/m/s)
	int WaveKin;	 // wave kinematics flag (0=off, >0=on)
	int WriteUnits;	// a global switch for whether to show the units line in the output files (1, default), or skip it (0)
	double FrictionCoefficient; // general bottom friction coefficient, as a start
	double FricDamp; // a damping coefficient used to model the friction at speeds near zero
	double StatDynFricScale; // a ratio of static to dynamic friction ( = mu_static/mu_dynamic)
} EnvCond;


typedef struct  // (matching line dictionary inputs)
{
	string type; 
	double d;
	double w;		// linear weight in air
	double EA;
	double c;    	// internal damping
	double Can;
	double Cat;
	double Cdn;
	double Cdt;
	double ReFac;
} LineProps;

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

typedef struct 
{  // this is C version of MDOutParmType - a less literal alternative of the NWTC OutParmType for MoorDyn (to avoid huge lists of possible output channel permutations)                                                                         
	char Name[10]; 		// "name of output channel"   
	char Units[10];			// "units string"     - should match UnitsSize in MoorDyn.cpp (should turn into def)                                                                            
	int QType;     		// "type of quantity - 0=tension, 1=x, 2=y, 3=z..."                                         
	int OType;     		// "type of object - 1=line, 2=connect"                                                                              
	int NodeID;    		// "node number if OType=1.  0=anchor, -1=N=Fairlead"                                                              
	int ObjID;     		// "number of Connect or Line object"
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

double eye(int I, int J);

void unitvector( vector< double > & u, vector< double > & r1, vector< double > & r2);

void inverse3by3( vector< vector< double > > & minv, vector< vector< double > > & m);

void RotMat( double x1, double x2, double x3, double TransMat[]);

double dotprod( vector<double>& A, vector<double>& B);

double dotprod( double A[], vector<double>& B);

//std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

vector<string> split(const string &s, char delim);

// calculate wave number from frequency, g, and depth (credit: FAST source)
float WaveNumber( float Omega, float g, float h );

float JONSWAP(float Omega, float Hs, float Tp, float Gamma );

float SINHNumOvrSIHNDen(float k, float h, float z );

float COSHNumOvrSIHNDen(float k, float h, float z );

void reverse(double* data, int datasize);
void doIIR(double* in, double* out, int dataSize, double* a, double* b, int kernelSize);
void doSSfilter(double* in, double* out, int dataSize, double* a, double* beta, double b0, int kernelSize);
double** make2Darray(int arraySizeX, int arraySizeY);
void free2Darray(double** theArray, int arraySizeX);


#endif
