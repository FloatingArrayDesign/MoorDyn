.. _coupling:

Coupling with your own code
===========================

Established couplings
---------------------

FAST/OpenFAST
^^^^^^^^^^^^^

The Moordyn fortran incarnation, MoorDyn-F, is a core module within OpenFAST and
is available in
`OpenFAST releases <https://github.com/openfast/openfast/releases>`_.
Originally, it was coupled to a modified form of FAST v7. 

WEC-Sim
^^^^^^^

WEC-Sim is currently coupled with MoorDyn v1. Support for the current version,
MoorDyn v2, will be added in the future when the opportunity arises.

DualSPHysics
^^^^^^^^^^^^

After developing a coupling with MoorDyn, the DualSPHysics team has forked it in
a seperate version called MoorDyn+, specifically dedicated to the coupling with
DualSPHysics.
MoorDyn v2 should on the other hand provide all the functionality required by
DUalSphysics, so DualSPHysics team will be queried to merge back on the
upstream.

Using from different languages
------------------------------

MoorDyn v2 can be used at a high level of abstraction in several languages. It
is also a possibility to micromanage MoorDyn in a much lower level in C++.

Below you can find documumentation on how handle MoorDyn in each language.

Note: Check out the :ref:`"Getting Started" <starting>` documentation to
learn more on how to install MoorDyn with support for each language.

C
^^^^^^

This is the primary language to use MoorDyn and it is always provided "out
of the box" when you install it. It is strongly recommended to use CMake to link
MoorDyn into your project (see :ref:`"Getting Started" <starting>`), although it
is not strictly required. For instance, if you installed it in the default
folder in Linux, you just need to add the flag "-lmoording" while linking
(either with GCC or CLang).

We can start considering the following basic example, consisting in 3 lines
anchored at the sea bed, and connected to 3 coupled fairleads that we are
controlling:

.. code-block:: c

    #include <stdio.h>
    #include <stdlib.h>
    #include <moordyn/MoorDyn2.h>

    int main(int, char**)
    {
        int err;
        MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
        if (!system)
            return 1;

        // 3 coupled connections x 3 components per connection = 9 DoF
        double x[9], dx[9];
        memset(dx, 0.0, sizeof(double));
        // Get the initial positions from the system itself
        for (unsigned int i = 0; i < 3; i++) {
            // 4 = first fairlead id
            MoorDynConnection conn = MoorDyn_GetConnection(system, i + 4);
            err = MoorDyn_GetConnectPos(conn, x + 3 * i);
            if (err != MOORDYN_SUCCESS) {
                MoorDyn_Close(system);
                return 1;
            }
        }

        // Setup the initial condition
        err = MoorDyn_Init(system, x, dx);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }

        // Make the connections move at 0.5 m/s to the positive x direction
        for (unsigned int i = 0; i < 3; i++)
            dx[3 * i] = 0.5;
        double t = 0.0, dt = 0.5;
        double f[9];
        err = MoorDyn_Step(system, x, dx, f, &t, &dt);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }

        // Print the position and tension of the line nodes
        unsigned int n_lines;
        err = MoorDyn_GetNumberLines(system, &n_lines);
        if (err != MOORDYN_SUCCESS) {
            MoorDyn_Close(system);
            return 1;
        }
        for (unsigned int i = 0; i < n_lines; i++) {
            const unsigned int line_id = i + 1;
            printf("Line %u\n", line_id);
            printf("=======\n");
            MoorDynLine line = MoorDyn_GetLine(system, line_id);
            if (!line) {
                MoorDyn_Close(system);
                return 1;
            }
            unsigned int n_nodes;
            err = MoorDyn_GetLineNumberNodes(line, &n_nodes);
            if (err != MOORDYN_SUCCESS) {
                MoorDyn_Close(system);
                return 1;
            }
            for (unsigned int j = 0; j < n_nodes; j++) {
                printf("  node %u:\n", j);
                double pos[3], ten[3];
                err = MoorDyn_GetLineNodePos(line, j, pos);
                if (err != MOORDYN_SUCCESS) {
                    MoorDyn_Close(system);
                    return 1;
                }
                printf("  pos = [%g, %g, %g]\n", pos[0], pos[1], pos[2]);
                err = MoorDyn_GetLineNodeTen(line, j, ten);
                if (err != MOORDYN_SUCCESS) {
                    MoorDyn_Close(system);
                    return 1;
                }
                printf("  ten = [%g, %g, %g]\n", ten[0], ten[1], ten[2]);
            }
        }

        // Alright, time to finish!
        err = MoorDyn_Close(system);
        if (err != MOORDYN_SUCCESS)
            return 1;

        return 0;
    }

In the example above everything starts calling

.. doxygenfunction:: MoorDyn_Create

and checking that it returned a non-NULL system. A NULL system would means that
there were some error building up the system. You can always know more about the
error in the information printed on the terminal.

In C you always need to very explicit, while in C++ you can be a little bit more
abstract, not needing indeed to worry about the type names, i.e. you can do
something like this:

.. code-block:: c

    auto system = MoorDyn_Create("Mooring/lines.txt");
    auto line = MoorDyn_GetLine(system, 1);

Anyway, the next step is initializing the system, that is computing the
static solution. But to this end, we need first to know the positions of the
coupled fairleads, so we use the functions

.. doxygenfunction:: MoorDyn_GetConnection
.. doxygenfunction:: MoorDyn_GetConnectPos

As you can appreciate, the :ref:`C API <api_c>` is always returning either an
object or an error code:

.. doxygengroup:: moordyn_errors_c

Thus, you can always programatically check that everything properly worked.

With the information of the initial positions of the fairlead, you can compute
the initial condition with the function

.. doxygenfunction:: MoorDyn_Init

Afterwards, you can start running MoorDyn by calling

.. doxygenfunction:: MoorDyn_Step

In this example, we are just calling it once. In a more complex application that
function will be called in a loop over time. Probably you need to feed back your
application with some information. In this example we are just collecting
information about the positions and forces at the line nodes, but you can
collect much more useful information. See the :ref:`C API <api_c>`.

Finally, it is very important that you always properly close the MoorDyn system,
so the allocated resources are released:

.. doxygenfunction:: MoorDyn_Close

Python
^^^^^^

You love Python! Who doesn't! Well, today it is your lucky day because MoorDyn
v2 is deploying its own Python wrapper. You can read :ref:`here <starting>` how
to install MoorDyn to have Python support.

Following you can find the equivalent example discussed above for C language,
this time developed in Python:

.. code-block:: c

    import moordyn

    system = moordyn.Create("Mooring/lines.txt")

    # 3 coupled connections x 3 components per connection = 9 DoF
    dx = [0] * 9
    # Get the initial positions from the system itself
    x = []
    for i in range(3):
        // 4 = first fairlead id
        conn = moordyn.GetConnection(system, i + 4)
        x = x + moordyn.GetConnectPos(conn)

    # Setup the initial condition
    moordyn.Init(system, x, dx)

    # Make the connections move at 0.5 m/s to the positive x direction
    for i in range(3):
        dx[3 * i] = 0.5
    t, dt = 0.0, 0.5
    f = moordyn.Step(system, x, dx, t, dt)

    # Print the position and tension of the line nodes
    n_lines = moordyn.GetNumberLines(system)
    for line_id in range(1, n_lines + 1):
        print("Line {}".format(line_id))
        print("=======")
        line = moordyn.GetLine(system, line_id)
        n_nodes = moordyn.GetLineNumberNodes(line)
        for node_id in range(n_nodes):
            print("  node {}:".format(node_id))
            pos = moordyn.GetLineNodePos(line, node_id)
            printf("  pos = {}".format(pos))
            ten = moordyn.GetLineNodeTen(line, node_id)
            printf("  ten = {}".format(ten))
        }
    }

    # Alright, time to finish!
    moordyn.Close(system)

That's all! You probably noticed that there are some differences with the C
code shown above, which makes it a bit simpler.
First, you obviously do not need to worry much about the variables typing.
Second, in Python the functions are not returning error codes. Instead, they are
triggering exceptions if errors are detected. Thus you can let Python to
stop the execution when an error is detected, but it is even better if you
enclose your code in a function within a try:

.. code-block:: c

    import moordyn

    system = moordyn.Create("Mooring/lines.txt")
    try:
        your_coupling_code(system)
    except Exception:
        raise
    finally:
        moordyn.Close(system)

So you can assert that the resources are always correctly released, no matter
if the code worked properly or exceptions were triggered.

Matlab
^^^^^^

MoorDyn can be called from Matlab scripts.

Simulink
^^^^^^^^

MoorDyn can be used with Simulink (and SimMechanics) models. The challenge is in
supporting MoorDyn's loose-coupling approach where it expects to be called for
sequential time steps and never for correction steps that might repeat a time
step.
A pulse/time-triggering block can be used in Simulink to ensure MoorDyn is
called correctly. An example of this can be seen in WEC-Sim.
