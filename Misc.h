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

// note: this file contains the struct definitions for environmental and line/connect properties


// from Iñaki Zabala
#ifdef _MSC_VER
template<typename T> static inline T round(T val) {return floor(val + 0.5);}
#endif

using namespace std;

typedef complex<double> doubleC; // make shorthand for complex double type

const double pi=3.14159265;

const doubleC i1(0., 1.); // set imaginary number 1



struct EnvCond
{
	double g;
	double WtrDpth;
	double rho_w;
	
	double kb;       // bottom stiffness
	double cb;       // bottom damping
	int WaveKin;	 // wave kinematics flag (on if 1, off if 0)
};


struct LineProps // (matching line dictionary inputs)
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
};

struct ConnectProps // matching node input stuff
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
	double Cd;  // added 2015/1/15
	double Ca;  // added 2015/1/15
};


double eye(int I, int J);

void unitvector( vector< double > & u, vector< double > & r1, vector< double > & r2);

void inverse3by3( vector< vector< double > > & minv, vector< vector< double > > & m);

void RotMat( double x1, double x2, double x3, double TransMat[]);

double dotprod( vector<double>& A, vector<double>& B);

double dotprod( double A[], vector<double>& B);

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim);

#endif