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

 // This is version 2.a5, 2021-03-16
 
#include "MoorDyn.h"
#include "MoorDyn2.h"
#include <stdlib.h>
#include <iostream>

// =============================================================================
//
//                     ||                     ||
//                     ||      Old C API      ||
//                    \  /                   \  /
//                     \/                     \/
//
// =============================================================================

/** @defgroup cmd Output console handling
 *  @{
 */

#ifdef WIN32

/// Console handle
int hConHandle;
/// Std output handle
intptr_t lStdHandle;

/// pointer to be made to environment variable PROMPT
char const* PromptPtr;
/// 0 if the system console is used, 1 if the console has been created by us
int OwnConsoleWindow = 0;

#endif

/**
 * @}
 */

/** \addtogroup old_api
 *  @{
 */

/// The singleton of the very only MoorDyn instance that this process might hold
/// This only applies if the old API is cosidered. See @ref old_api
MoorDyn md_singleton = NULL;

/**
 * @}
 */

int DECLDIR MoorDynInit(const double x[], const double xd[], const char *infilename)
{
#ifdef WIN32
	// ------------ create console window for messages if none already available -----------------
	// adapted from Andrew S. Tucker, "Adding Console I/O to a Win32 GUI App" in Windows Developer Journal, December 1997. source code at http://dslweb.nwnexus.com/~ast/dload/guicon.htm

	FILE *fp;
	// get pointer to environment variable "PROMPT" (NULL if not in console)
	PromptPtr = getenv("PROMPT");
	
	//TODO: simplify this to just keep the output parts I need

	HWND consoleWnd = GetConsoleWindow();
	if (!consoleWnd)
	{
		// if not in console, create our own
		OwnConsoleWindow = 1;

		// allocate a console for this app
		AllocConsole();

		// set the screen buffer to be big enough to let us scroll text
	    static const WORD MAX_CONSOLE_LINES = 500;
	    CONSOLE_SCREEN_BUFFER_INFO coninfo;
		GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
		coninfo.dwSize.Y = MAX_CONSOLE_LINES;
		SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

		// redirect unbuffered STDOUT to the console
		//lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
		lStdHandle = (intptr_t)GetStdHandle(STD_OUTPUT_HANDLE);
		hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
		fp = _fdopen( hConHandle, "w" );
		*stdout = *fp;
		setvbuf(stdout, NULL, _IONBF, 0);

		// redirect unbuffered STDERR to the console
		lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
		hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
		fp = _fdopen( hConHandle, "w" );
		*stderr = *fp;
		setvbuf( stderr, NULL, _IONBF, 0 );

		// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
		// point to console as well
		ios::sync_with_stdio();
		
		cout << "(MoorDyn-initiated console window)" << endl;
	}
#endif

	MoorDyn instance = MoorDyn_Create(infilename);
	if (!instance)
		return MOORDYN_UNHANDLED_ERROR;

	int err = MoorDyn_Init(instance, x, xd);
	if (err)
		return err;

	if (md_singleton)
		MoorDyn_Close(md_singleton);  // We do not care if this fails
	md_singleton = instance;

	return MOORDYN_SUCCESS;
}

// This is the original time stepping function, for platform-centric coupling, but now it has capabilities for multiple 6DOF coupled bodies
int DECLDIR MoorDynStep(const double x[], const double xd[], double f[], double* t_in, double* dt_in) 
{
	if (!md_singleton)
		return MOORDYN_INVALID_VALUE;

	return MoorDyn_Step(md_singleton, x, xd, f, t_in, dt_in);
}

int DECLDIR MoorDynClose(void)
{
	if (!md_singleton)
		return MOORDYN_INVALID_VALUE;

	int err = MoorDyn_Close(md_singleton);
	if (err)
		return err;

	md_singleton = NULL;
	std::cout << "   MoorDyn closed." << std::endl;

#ifdef WIN32
	if (OwnConsoleWindow == 1)  {
		std::cout << "press enter to close: " << std::endl;
		std::cin.get();
		FreeConsole();
	}
#endif

	return 0;
}

int DECLDIR externalWaveKinInit()
{
	if (!md_singleton)
		return 0;

	unsigned int n;
	int err = MoorDyn_InitExtWaves(md_singleton, &n);
	if (err)
		return 0;

	return (int)n;
}

// returns array providing coordinates of all points that will be receiving wave kinematics
void DECLDIR getWaveKinCoordinates(double r_out[])
{
	if (!md_singleton)
		return;
	MoorDyn_GetWavesCoords(md_singleton, r_out);
}

// receives arrays containing U and Ud for each point at which wave kinematics will be applied (and the time they are calculated at)
void DECLDIR setWaveKin(const double U_in[], const double Ud_in[], double t_in)
{
	if (!md_singleton)
		return;
	MoorDyn_SetWaves(md_singleton, U_in, Ud_in, t_in);
}


double DECLDIR GetFairTen(int l)
{
	if (!md_singleton)
		return -1;
	return MoorDyn_GetFairTen(md_singleton, l);
}



int DECLDIR GetFASTtens(int* numLines, float FairHTen[], float FairVTen[], float AnchHTen[], float AnchVTen[] )
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetFASTtens(
		md_singleton, numLines, FairHTen, FairVTen, AnchHTen, AnchVTen);
}

int DECLDIR GetConnectPos(int l, double pos[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetConnectPos(md_singleton, (unsigned int)l, pos);
}

int DECLDIR GetConnectForce(int l, double force[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetConnectForce(md_singleton, (unsigned int)l, force);
}

int DECLDIR GetNodePos(int LineNum, int NodeNum, double pos[3])
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_GetNodePos(
		md_singleton, (unsigned int)LineNum, (unsigned int)NodeNum, pos);
}


int DECLDIR DrawWithGL()
{
	if (!md_singleton)
		return MOORDYN_MEM_ERROR;
	return MoorDyn_DrawWithGL(md_singleton);
}
