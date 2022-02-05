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
    cout << "bad_input_file()..." << endl;

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
    cout << "minimal()..." << endl;

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

/** @brief Runs all the test
 * @param argc Unused
 * @param argv Unused
 * @return 0 if the tests have ran just fine. The index of the failing test
 * otherwise
 */
int main(int argc, char** argv)
{
    if (!old_api::bad_input_file())
        return 1;
    if (!old_api::minimal())
        return 2;
    return 0;
}
