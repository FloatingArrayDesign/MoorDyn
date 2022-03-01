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

#ifndef __MOORDYN_H__
#define __MOORDYN_H__

#ifdef MoorDyn_EXPORTS     // this is set as a preprocessor definition!!!
	#ifndef LINUX
        #ifndef OSX
            #define DECLDIR __declspec(dllexport)
        #else
            #define DECLDIR
        #endif
	#else
		#define DECLDIR 
	#endif
#else
	#define DECLDIR //__declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/*#ifndef LINUX
#ifndef OSX
 #include <Windows.h>
#endif
#endif*/

int DECLDIR LinesInit(double X[], double XD[]);

int DECLDIR LinesCalc(double X[], double XD[], double Flines[], double* , double* );

int DECLDIR FairleadsCalc2(double rFairIn[], double rdFairIn[], double fFairIn[], double* t_in, double *dt_in); // easier to call version
int DECLDIR FairleadsCalc(double **rFairIn, double **rdFairIn, double ** fFairIn, double* t_in, double *dt_in);


int DECLDIR LinesClose(void);

double DECLDIR GetFairTen(int);

int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] );

int DECLDIR GetConnectPos(int l, double pos[3]);
int DECLDIR GetConnectForce(int l, double force[3]);
int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3]);

int DECLDIR DrawWithGL(void);

void AllOutput(double, double);

#ifdef __cplusplus
}
#endif

#endif
