/*
 * Copyright (c) 2022 Matt Hall <mtjhall@alumni.uvic.ca> and Jose Luis
 * Cercos-Pita <jlc@core-marine.com>
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

/** @file bodies_and_rods.cpp
 * A simple driver program that will run MoorDyn VERSION 2 without any platform
 * motion. It includes bodies and rods
 */
#include "MoorDyn2.h"
#include <iostream>
#include <algorithm>


using namespace std;

/** @brief Runs the test
 * @return 0 if the tests have ran just fine. The index of the failing test
 * otherwise
 */
int main(int, char**)
{
    MoorDyn system = MoorDyn_Create("../../test/Mooring/BodiesAndRods.dat");
    if (!system)
    {
        cerr << "Failure Creating the Mooring system" << endl;
        return 1;
    }

    const unsigned int n_dof = MoorDyn_NCoupledDOF(system);
    if (n_dof)
    {
        cerr << "No coupled Degrees Of Freedom were expected, but " << n_dof
             << "were reported" << endl;
        MoorDyn_Close(system);
        return 2;
    }

    int err;
    err = MoorDyn_Init(system, NULL, NULL);
    if (err != MOORDYN_SUCCESS)
    {
        cerr << "Failure during the mooring initialization: " << err << endl;
        MoorDyn_Close(system);
        return 3;
    }

    double dt = 0.1;
    const unsigned int nts = 10;
    for (unsigned int i = 0; i < nts; i++)
    {
        double t = i * dt;

        err = MoorDyn_Step(system, NULL, NULL, NULL, &t, &dt);
        if (err != MOORDYN_SUCCESS) {
            cerr << "Failure during the mooring step " << i << ": "
                << err << endl;
            MoorDyn_Close(system);
            return 4;
        }
    }

    err = MoorDyn_Close(system);
    if (err != MOORDYN_SUCCESS) {
        cerr << "Failure closing Moordyn: " << err << endl;
        return false;
    }

    return 0;
}
