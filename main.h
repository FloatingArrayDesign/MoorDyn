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

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef MoorDyn_EXPORTS     // this is set as a preprocessor definition!!!
#define DECLDIR __declspec(dllexport)
#else
#define DECLDIR //__declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#include <Windows.h>

int DECLDIR LinesInit(float X[], float XD[], float TransMat[], float* dTime);

int DECLDIR LinesCalc(float X[], float XD[], float TransMat[], float Flines[], float* ZTime, float* dTime, 
				int* NumLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[]);

int DECLDIR LinesClose(void);


void AllOutput(double);

#ifdef __cplusplus
}
#endif

#endif