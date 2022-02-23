/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
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

/** @file minimal.cpp
 * Minimal tests that only checks the library is correctly initialized,
 * running and closing
 */

#include "MoorDyn.h" 
#include "MoorDyn2.h" 
#include <stdexcept>
#include <iostream>
#include <algorithm>


using namespace std;

namespace old_api
{

/** @brief Check that bad input files are correctly handled
 * @return true if the test worked, false otherwise
 */
bool bad_input_file()
{
    cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

    int err;
    double x[6], dx[6];
    std::fill(x, x + 6, 0.0);
    std::fill(dx, dx + 6, 0.0);
    err = MoorDynInit(x, dx, "badfile.txt");
    if (err != MOORDYN_UNHANDLED_ERROR) {
        cerr << "The error code " << MOORDYN_UNHANDLED_ERROR
             << " was expected, but " << err << " was received" << endl;
        return false;
    }

    err = MoorDynClose();
    if (err != MOORDYN_INVALID_VALUE) {
        cerr << "Failure closing Moordyn: " << err << endl;
        return false;
    }

    return true;
}

/** @brief Check that a mooring system can be initialized, that a step can be
 * ran and that everyting can be closed
 * @return true if the test worked, false otherwise
 */
bool minimal()
{
    cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

    int err;
    double x[9], dx[9];
    // Set the fairlead connections, as they are in the config file
    x[0] = 5.2;  x[1] = 0.0;  x[2] = -70.0;
    x[3] = -2.6; x[4] = 4.5;  x[5] = -70.0;
    x[6] = -2.6; x[7] = -4.5; x[8] = -70.0;
    std::fill(dx, dx + 9, 0.0);
    err = MoorDynInit(x, dx, "../../test/Mooring/lines.txt");
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure during the mooring initialization: " << err << endl;
        return false;
    }

    double f[6 * 3 + 3 * 3];  // 6x3 for the fairleads, 3x3 for the anchors
    double t = 0.0, dt = 0.5;
    err = MoorDynStep(x, dx, f, &t, &dt);
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure during the mooring step: " << err << endl;
        return false;
    }

    err = MoorDynClose();
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure closing Moordyn: " << err << endl;
        return false;
    }

    return true;
}

}  // ::old_api

/** @brief Check that bad input files are correctly handled
 * @return true if the test worked, false otherwise
 */
bool bad_input_file()
{
    cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

    MoorDyn system = MoorDyn_Create("badfile.txt");
    if (system) {
        cerr << "The system is not Null when a bad file is provided" << endl;
        MoorDyn_Close(system);
        return false;
    }

    return true;
}

/** @brief Check that a mooring system can be initialized, that a step can be
 * ran and that everyting can be closed
 * @return true if the test worked, false otherwise
 */
bool minimal()
{
    cout << endl << " => " << __PRETTY_FUNC_NAME__ << "..." << endl;

    MoorDyn system = MoorDyn_Create("../../test/Mooring/lines.txt");
    if (!system)
    {
        cerr << "Failure Creating the Mooring system" << endl;
        return false;        
    }

    const unsigned int n_dof = MoorDyn_NCoupledDOF(system);
    if (n_dof != 9) {
        cerr << "3x3 = 9 DOFs were expected, but " << n_dof
             << "were reported" << endl;
        MoorDyn_Close(system);
        return false;
    }

    int err;
    double x[9], dx[9];
    // Get the initial positions from the config file
    for (unsigned int i = 0; i < 3; i++)
    {
        err = MoorDyn_GetConnectPos(system,
                                    i + 4,  // 4 = first fairlead id
                                    x + 3 * i);
        if (err != MOORDYN_SUCCESS) {
            cerr << "Failure retrieving the fairlead " << i + 4
                 << " position: " << err << endl;
            MoorDyn_Close(system);
            return false;
        }
    }
    std::fill(dx, dx + 9, 0.0);
    err = MoorDyn_Init(system, x, dx);
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure during the mooring initialization: " << err << endl;
        return false;
    }

    double f[9];
    double t = 0.0, dt = 0.5;
    err = MoorDyn_Step(system, x, dx, f, &t, &dt);
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure during the mooring step: " << err << endl;
        return false;
    }

    err = MoorDyn_Close(system);
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure closing Moordyn: " << err << endl;
        return false;
    }

    return true;
}

/** @brief Runs all the test
 * @return 0 if the tests have ran just fine. The index of the failing test
 * otherwise
 */
int main(int, char**)
{
    if (!old_api::bad_input_file())
        return 1;
    if (!old_api::minimal())
        return 2;
    if (!bad_input_file())
        return 3;
    if (!minimal())
        return 4;
    return 0;
}
