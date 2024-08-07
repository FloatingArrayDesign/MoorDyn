/*
 * Copyright (c) 2022 Jose Luis Cercos-Pita <jlc@core-marine.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <sstream>
#include <Python.h>
#include "MoorDyn2.h"

const char moordyn_capsule_name[] = "MoorDyn";
const char waves_capsule_name[] = "MoorDynWaves";
const char seafloor_capsule_name[] = "MoorDynSeafloor";
const char body_capsule_name[] = "MoorDynBody";
const char rod_capsule_name[] = "MoorDynRod";
const char point_capsule_name[] = "MoorDynPoint";
const char line_capsule_name[] = "MoorDynLine";

/** @brief Allocates and fill a C array with doubles
 * @param lst The iterable object
 * @return The C array with the values
 */
double*
py_iterable_to_double(PyObject* lst)
{
	const int l = PySequence_Fast_GET_SIZE(lst);
	double* arr = (double*)malloc(l * sizeof(double));
	if (!arr) {
		PyErr_SetString(PyExc_MemoryError, "Failure allocating memory");
		return NULL;
	}
	for (int i = 0; i < l; i++) {
		PyObject* fitem;
		PyObject* item = PySequence_Fast_GET_ITEM(lst, i);
		if (!item) {
			free(arr);
			return NULL;
		}
		fitem = PyNumber_Float(item);
		if (!fitem) {
			free(arr);
			PyErr_SetString(PyExc_TypeError, "Non-float number detected");
			return 0;
		}
		arr[i] = PyFloat_AS_DOUBLE(fitem);
		Py_DECREF(fitem);
	}

	return arr;
}

//                                 MoorDyn2.h
// =============================================================================

/** @brief Wrapper to MoorDyn_Create() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
create(PyObject*, PyObject* args)
{
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "|s", &filepath))
		return NULL;

	MoorDyn system = MoorDyn_Create(filepath);
	if (!system) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_Create() failed");
		return NULL;
	}

	return PyCapsule_New((void*)system, moordyn_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_NCoupledDOF() function
 * @param args Python passed arguments
 * @return The number of coupled DOFs
 */
static PyObject*
n_coupled_dof(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_NCoupledDOF(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_SetVerbosity() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
set_verbosity(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int verbosity;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &verbosity))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	return PyLong_FromLong(MoorDyn_SetVerbosity(system, verbosity));
}

/** @brief Wrapper to MoorDyn_SetLogFile() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
set_logfile(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	return PyLong_FromLong(MoorDyn_SetLogFile(system, filepath));
}

/** @brief Wrapper to MoorDyn_SetLogLevel() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
set_loglevel(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int verbosity;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &verbosity))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	return PyLong_FromLong(MoorDyn_SetLogLevel(system, verbosity));
}

/** @brief Wrapper to MoorDyn_Log() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
log(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int level;
	char* msg = NULL;

	if (!PyArg_ParseTuple(args, "Ois", &capsule, &level, &msg))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	return PyLong_FromLong(MoorDyn_Log(system, level, msg));
}

/** @brief Wrapper to MoorDyn_Init() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
init(PyObject*, PyObject* args)
{
	PyObject *capsule, *x_lst, *v_lst;
	int skip_ic = 0;

	if (!PyArg_ParseTuple(args, "OOO|i", &capsule, &x_lst, &v_lst, &skip_ic))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	// Get the expected number of DOFs in the provided arrays
	unsigned int n_dof;
	MoorDyn_NCoupledDOF(system, &n_dof);

	x_lst = PySequence_Fast(x_lst, "1st argument must be iterable");
	if (!x_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(x_lst) != n_dof) {
		std::stringstream err;
		err << "1st argument must have " << n_dof << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	v_lst = PySequence_Fast(v_lst, "2nd argument must be iterable");
	if (!v_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(v_lst) != n_dof) {
		std::stringstream err;
		err << "2nd argument must have " << n_dof << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	// Convert them to C arrays that MoorDyn might handle
	double* x_arr = py_iterable_to_double(x_lst);
	Py_DECREF(x_lst);
	double* v_arr = py_iterable_to_double(v_lst);
	Py_DECREF(v_lst);
	if ((!x_arr) || (!v_arr)) {
		return NULL;
	}

	// Now we can call MoorDyn
	int err;
	if (skip_ic)
		err = MoorDyn_Init_NoIC(system, x_arr, v_arr);
	else
		err = MoorDyn_Init(system, x_arr, v_arr);
	free(x_arr);
	free(v_arr);

	return PyLong_FromLong(err);
}

/** @brief Wrapper to MoorDyn_Step() function
 * @param args Python passed arguments
 * @return The forces on the coupled objects
 */
static PyObject*
step(PyObject*, PyObject* args)
{
	PyObject *capsule, *x_lst, *v_lst;
	double t, dt;

	if (!PyArg_ParseTuple(args, "OOOdd", &capsule, &x_lst, &v_lst, &t, &dt))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	// Get the expected number of DOFs in the provided arrays
	unsigned int n_dof;
	MoorDyn_NCoupledDOF(system, &n_dof);

	x_lst = PySequence_Fast(x_lst, "1st argument must be iterable");
	if (!x_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(x_lst) != n_dof) {
		std::stringstream err;
		err << "1st argument must have " << n_dof << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	v_lst = PySequence_Fast(v_lst, "2nd argument must be iterable");
	if (!v_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(v_lst) != n_dof) {
		std::stringstream err;
		err << "2nd argument must have " << n_dof << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	// Convert them to C arrays that MoorDyn might handle
	double* x_arr = py_iterable_to_double(x_lst);
	Py_DECREF(x_lst);
	double* v_arr = py_iterable_to_double(v_lst);
	Py_DECREF(v_lst);
	if ((!x_arr) || (!v_arr)) {
		return NULL;
	}
	double* forces;
	forces = (double*)malloc(n_dof * sizeof(double));
	if (!forces) {
		PyErr_SetString(PyExc_MemoryError, "Failure allocating the forces");
		return NULL;
	}

	// Now we can call MoorDyn
	const int err = MoorDyn_Step(system, x_arr, v_arr, forces, &t, &dt);

	free(x_arr);
	free(v_arr);

	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError,
		                "MoorDyn reported an error integrating");
		return NULL;
	}

	PyObject* f_lst = PyTuple_New(n_dof);
	for (unsigned int i = 0; i < n_dof; i++) {
		PyTuple_SET_ITEM(f_lst, i, PyFloat_FromDouble(forces[i]));
	}
	free(forces);
	return f_lst;
}

/** @brief Wrapper to MoorDyn_Close() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
close(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	const int err = MoorDyn_Close(system);
	return PyLong_FromLong(err);
}

/** @brief Wrapper to MoorDyn_GetWaves() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_waves(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynWaves waves = MoorDyn_GetWaves(system);
	if (!waves) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetWaves() failed");
		return NULL;
	}

	return PyCapsule_New((void*)waves, waves_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_GetSeafloor() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_seafloor(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynSeafloor seafloor = MoorDyn_GetSeafloor(system);
	if (!seafloor) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetSeafloor() failed");
		return NULL;
	}

	return PyCapsule_New((void*)seafloor, seafloor_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_ExternalWaveKinInit() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
ext_wave_init(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_ExternalWaveKinInit(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(err);
}

/** @brief Wrapper to MoorDyn_ExternalWaveKinGetN() function
 * @param args Python passed arguments
 * @return The number of points
 */
static PyObject*
ext_wave_n(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_ExternalWaveKinGetN(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetWaveKinCoordinates() function
 * @param args Python passed arguments
 * @return The list of coordinates where the wave kinematics shall be provided
 * (3 by the number of points)
 */
static PyObject*
ext_wave_coords(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	// We need to know the number of coordinates to allocate
	int err;
	unsigned int n;
	err = MoorDyn_ExternalWaveKinGetN(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	double* coords = (double*)malloc(n * 3 * sizeof(double));
	if (!coords) {
		PyErr_SetString(PyExc_RuntimeError, "Failure allocating memory");
		return NULL;
	}
	err = MoorDyn_ExternalWaveKinGetCoordinates(system, coords);
	if (err != 0) {
		free(coords);
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(n * 3);
	for (unsigned int i = 0; i < n * 3; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(coords[i]));
	}
	free(coords);
	return lst;
}

/** @brief Wrapper to MoorDyn_SetWaveKin() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
ext_wave_set(PyObject*, PyObject* args)
{
	PyObject *capsule, *v_lst, *a_lst;
	double t;

	if (!PyArg_ParseTuple(args, "OOOd", &capsule, &v_lst, &a_lst, &t))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	// We need to know the number of coordinates to avoid errors
	int err;
	unsigned int n;
	err = MoorDyn_ExternalWaveKinGetN(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	n *= 3;

	v_lst = PySequence_Fast(v_lst, "1st argument must be iterable");
	if (!v_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(v_lst) != n) {
		std::stringstream err;
		err << "1st argument must have " << n << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	a_lst = PySequence_Fast(a_lst, "2nd argument must be iterable");
	if (!a_lst)
		return NULL;
	if (PySequence_Fast_GET_SIZE(a_lst) != n) {
		std::stringstream err;
		err << "2nd argument must have " << n << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	// Convert them to C arrays that MoorDyn might handle
	double* v_arr = py_iterable_to_double(v_lst);
	Py_DECREF(v_lst);
	double* a_arr = py_iterable_to_double(a_lst);
	Py_DECREF(a_lst);
	if ((!v_arr) || (!a_arr)) {
		return NULL;
	}

	// Now we can call MoorDyn
	err = MoorDyn_ExternalWaveKinSet(system, v_arr, a_arr, t);
	free(v_arr);
	free(a_arr);
	return PyLong_FromLong(err);
}

/** @brief Wrapper to MoorDyn_GetNumberBodies() function
 * @param args Python passed arguments
 * @return The number of rigid bodies
 */
static PyObject*
get_number_bodies(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetNumberBodies(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetBody() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_body(PyObject*, PyObject* args)
{
	PyObject* capsule;
	unsigned int i;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &i))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynBody body = MoorDyn_GetBody(system, i);
	if (!body) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetBody() failed");
		return NULL;
	}

	return PyCapsule_New((void*)body, body_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_GetNumberRods() function
 * @param args Python passed arguments
 * @return The number of rods
 */
static PyObject*
get_number_rods(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetNumberRods(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetRod() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_rod(PyObject*, PyObject* args)
{
	PyObject* capsule;
	unsigned int i;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &i))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynRod rod = MoorDyn_GetRod(system, i);
	if (!rod) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetRod() failed");
		return NULL;
	}

	return PyCapsule_New((void*)rod, rod_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_GetNumberPoints() function
 * @param args Python passed arguments
 * @return The number of points
 */
static PyObject*
get_number_points(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetNumberPoints(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetPoint() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_point(PyObject*, PyObject* args)
{
	PyObject* capsule;
	unsigned int i;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &i))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynPoint point = MoorDyn_GetPoint(system, i);
	if (!point) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetPoint() failed");
		return NULL;
	}

	return PyCapsule_New((void*)point, point_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_GetNumberLines() function
 * @param args Python passed arguments
 * @return The number of lines
 */
static PyObject*
get_number_lines(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetNumberLines(system, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetLine() function
 * @param args Python passed arguments
 * @return A Python capsule
 */
static PyObject*
get_line(PyObject*, PyObject* args)
{
	PyObject* capsule;
	unsigned int i;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &i))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	MoorDynLine line = MoorDyn_GetLine(system, i);
	if (!line) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn_GetLine() failed");
		return NULL;
	}

	return PyCapsule_New((void*)line, line_capsule_name, NULL);
}

/** @brief Wrapper to MoorDyn_GetFASTtens() function
 * @param args Python passed arguments
 * @return The horizontal and vertical forces on the fairleads and
 anchors
 */
static PyObject*
get_fast_tens(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int num_lines;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &num_lines))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	float* fair_h_ten = (float*)malloc(num_lines * sizeof(float));
	float* fair_v_ten = (float*)malloc(num_lines * sizeof(float));
	float* anch_h_ten = (float*)malloc(num_lines * sizeof(float));
	float* anch_v_ten = (float*)malloc(num_lines * sizeof(float));
	if (!fair_h_ten || !fair_v_ten || !anch_h_ten || !anch_v_ten) {
		PyErr_SetString(PyExc_MemoryError, "Failure allocating memory");
		return NULL;
	}

	const int err = MoorDyn_GetFASTtens(
	    system, &num_lines, fair_h_ten, fair_v_ten, anch_h_ten, anch_v_ten);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* fair_h_ten_lst = PyTuple_New(num_lines);
	PyObject* fair_v_ten_lst = PyTuple_New(num_lines);
	PyObject* anch_h_ten_lst = PyTuple_New(num_lines);
	PyObject* anch_v_ten_lst = PyTuple_New(num_lines);
	PyObject* lst = PyTuple_New(4);
	if (!fair_h_ten_lst || !fair_v_ten_lst || !anch_h_ten_lst ||
	    !anch_v_ten_lst || !lst) {
		PyErr_SetString(PyExc_MemoryError, "Failure allocating memory");
		return NULL;
	}
	for (int i = 0; i < num_lines; i++) {
		PyTuple_SET_ITEM(fair_h_ten_lst, i, PyFloat_FromDouble(fair_h_ten[i]));
		PyTuple_SET_ITEM(fair_v_ten_lst, i, PyFloat_FromDouble(fair_v_ten[i]));
		PyTuple_SET_ITEM(anch_h_ten_lst, i, PyFloat_FromDouble(anch_h_ten[i]));
		PyTuple_SET_ITEM(anch_v_ten_lst, i, PyFloat_FromDouble(anch_v_ten[i]));
	}
	free(fair_h_ten);
	free(fair_v_ten);
	free(anch_h_ten);
	free(anch_v_ten);

	PyTuple_SET_ITEM(lst, 0, fair_h_ten_lst);
	PyTuple_SET_ITEM(lst, 1, fair_v_ten_lst);
	PyTuple_SET_ITEM(lst, 2, anch_h_ten_lst);
	PyTuple_SET_ITEM(lst, 3, anch_v_ten_lst);

	return lst;
}

/** @brief Wrapper to MoorDyn_Serialize() function
 * @param args Python passed arguments
 * @return The bytes array
 */
static PyObject*
serialize(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	int err;
	size_t array_size;
	err = MoorDyn_Serialize(system, &array_size, NULL);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	char* array = (char*)malloc(array_size);
	if (!array) {
		std::stringstream err;
		err << "Failure allocating " << array_size << " bytes";
		PyErr_SetString(PyExc_MemoryError, err.str().c_str());
		return NULL;
	}
	err = MoorDyn_Serialize(system, NULL, (uint64_t*)array);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* bytes = PyBytes_FromStringAndSize(array, array_size);
	free(array);

	return bytes;
}

/** @brief Wrapper to MoorDyn_Deserialize() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
deserialize(PyObject*, PyObject* args)
{
	PyObject *capsule, *bytes;

	if (!PyArg_ParseTuple(args, "OO", &capsule, &bytes))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system || !PyBytes_Check(bytes))
		return NULL;

	char* array = PyBytes_AsString(bytes);
	const int err = MoorDyn_Deserialize(system, (uint64_t*)array);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_Save() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
save(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	const int err = MoorDyn_Save(system, filepath);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return Py_None;
}

/** @brief Wrapper to MoorDyn_Load() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
load(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	const int err = MoorDyn_Load(system, filepath);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	return Py_None;
}

/** @brief Wrapper to MoorDyn_SaveRodVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
save_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	return PyLong_FromLong(MoorDyn_SaveVTK(system, filepath));
}

//                                 Waves.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetWavesKin() function
 * @param args Python passed arguments
 * @return The velocity, the acceleration, the wave height and the dynamic
 * pressure
 */
static PyObject*
waves_getkin(PyObject*, PyObject* args)
{
	PyObject* capsule;
	double x, y, z;
	PyObject* seafloor;

	if (!PyArg_ParseTuple(args, "Oddd|O", &capsule, &x, &y, &z, &seafloor))
		return NULL;

	MoorDynWaves instance =
	    (MoorDynWaves)PyCapsule_GetPointer(capsule, waves_capsule_name);
	if (!instance)
		return NULL;

	MoorDynSeafloor seabed = NULL;
	if (seafloor != Py_None) {
		seabed = (MoorDynSeafloor)PyCapsule_GetPointer(
			seafloor, seafloor_capsule_name);
		if (!seabed)
			return NULL;
	}

	double u[3], ud[3], zeta, pdyn;
	const int err = MoorDyn_GetWavesKin(
		instance, x, y, z, u, ud, &zeta, &pdyn, seabed);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(4);
	PyObject* pyu = PyTuple_New(3);
	PyObject* pyud = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyTuple_SET_ITEM(pyu, i, PyFloat_FromDouble(u[i]));
		PyTuple_SET_ITEM(pyud, i, PyFloat_FromDouble(ud[i]));
	}
	PyTuple_SET_ITEM(lst, 0, pyu);
	PyTuple_SET_ITEM(lst, 1, pyud);
	PyTuple_SET_ITEM(lst, 2, PyFloat_FromDouble(zeta));
	PyTuple_SET_ITEM(lst, 3, PyFloat_FromDouble(pdyn));

	return lst;
}

//                                 Seafloor.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetDepthAt() function
 * @param args Python passed arguments
 * @return The depth
 */
static PyObject*
seafloor_getdepth(PyObject*, PyObject* args)
{
	PyObject* capsule;
	double x, y;

	if (!PyArg_ParseTuple(args, "Odd", &capsule, &x, &y))
		return NULL;

	MoorDynSeafloor instance =
	    (MoorDynSeafloor)PyCapsule_GetPointer(capsule, seafloor_capsule_name);
	if (!instance)
		return NULL;

	double depth;
	const int err = MoorDyn_GetDepthAt(instance, x, y, &depth);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(depth);
}

/** @brief Wrapper to MoorDyn_GetAverageDepth() function
 * @param args Python passed arguments
 * @return The depth
 */
static PyObject*
seafloor_getavgdepth(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynSeafloor instance =
	    (MoorDynSeafloor)PyCapsule_GetPointer(capsule, seafloor_capsule_name);
	if (!instance)
		return NULL;

	double depth;
	const int err = MoorDyn_GetAverageDepth(instance, &depth);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(depth);
}

/** @brief Wrapper to MoorDyn_GetMinDepth() function
 * @param args Python passed arguments
 * @return The depth
 */
static PyObject*
seafloor_getmindepth(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynSeafloor instance =
	    (MoorDynSeafloor)PyCapsule_GetPointer(capsule, seafloor_capsule_name);
	if (!instance)
		return NULL;

	double depth;
	const int err = MoorDyn_GetMinDepth(instance, &depth);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(depth);
}

//                                 Body.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetBodyID() function
 * @param args Python passed arguments
 * @return The body id
 */
static PyObject*
body_get_id(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetBodyID(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetBodyType() function
 * @param args Python passed arguments
 * @return The body type
 */
static PyObject*
body_get_type(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetBodyType(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetBodyState() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_state(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double r[6], rd[6];
	const int err = MoorDyn_GetBodyState(instance, r, rd);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(2);
	PyObject* pyr = PyTuple_New(6);
	PyObject* pyrd = PyTuple_New(6);
	for (unsigned int i = 0; i < 6; i++) {
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));
		PyTuple_SET_ITEM(pyrd, i, PyFloat_FromDouble(rd[i]));
	}
	PyTuple_SET_ITEM(lst, 0, pyr);
	PyTuple_SET_ITEM(lst, 1, pyrd);

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyPos() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_pos(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetBodyPos(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(r[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyAngle() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_angle(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetBodyAngle(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(r[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyVel() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_vel(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetBodyVel(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(r[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyAngVel() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_angvel(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetBodyAngVel(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(r[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyForce() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_force(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double f[6];
	const int err = MoorDyn_GetBodyForce(instance, f);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(6);
	for (unsigned int i = 0; i < 6; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(f[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetBodyM() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
body_get_m(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	double m[6][6];
	const int err = MoorDyn_GetBodyM(instance, m);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(6);
	for (unsigned int i = 0; i < 6; i++) {
		PyObject* sub = PyTuple_New(6);
		for (unsigned int j = 0; j < 6; j++) {
			PyTuple_SET_ITEM(sub, j, PyFloat_FromDouble(m[i][j]));
		}
		PyTuple_SET_ITEM(lst, i, sub);
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_SaveBodyVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
body_save_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	return PyLong_FromLong(MoorDyn_SaveBodyVTK(instance, filepath));
}

/** @brief Wrapper to MoorDyn_UseBodyVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
body_use_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDynBody instance =
	    (MoorDynBody)PyCapsule_GetPointer(capsule, body_capsule_name);
	if (!instance)
		return NULL;

	return PyLong_FromLong(MoorDyn_UseBodyVTK(instance, filepath));
}

//                                 Rod.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetRodID() function
 * @param args Python passed arguments
 * @return The rod id
 */
static PyObject*
rod_get_id(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetRodID(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetRodType() function
 * @param args Python passed arguments
 * @return The rod type
 */
static PyObject*
rod_get_type(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetRodType(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetRodForce() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
rod_get_force(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	double f[6];
	const int err = MoorDyn_GetRodForce(instance, f);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(6);
	for (unsigned int i = 0; i < 6; i++) {
		PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(f[i]));
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetRodM() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
rod_get_m(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	double m[6][6];
	const int err = MoorDyn_GetRodM(instance, m);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(6);
	for (unsigned int i = 0; i < 6; i++) {
		PyObject* sub = PyTuple_New(6);
		for (unsigned int j = 0; j < 6; j++) {
			PyTuple_SET_ITEM(sub, j, PyFloat_FromDouble(m[i][j]));
		}
		PyTuple_SET_ITEM(lst, i, sub);
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetRodN() function
 * @param args Python passed arguments
 * @return The number of segments
 */
static PyObject*
rod_get_n(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetRodN(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetRodNodePos() function
 * @param args Python passed arguments
 * @return The position
 */
static PyObject*
rod_get_node_pos(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetRodNodePos(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetRodNodeVel() function
 * @param args Python passed arguments
 * @return The position
 */
static PyObject*
rod_get_node_vel(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetRodNodeVel(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_SaveRodVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
rod_save_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDynRod instance =
	    (MoorDynRod)PyCapsule_GetPointer(capsule, rod_capsule_name);
	if (!instance)
		return NULL;

	return PyLong_FromLong(MoorDyn_SaveRodVTK(instance, filepath));
}

//                              Point.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetPointID() function
 * @param args Python passed arguments
 * @return The point id
 */
static PyObject*
point_get_id(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetPointID(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetPointType() function
 * @param args Python passed arguments
 * @return The point type
 */
static PyObject*
point_get_type(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	int t;
	const int err = MoorDyn_GetPointType(instance, &t);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(t);
}

/** @brief Wrapper to MoorDyn_GetPointPos() function
 * @param args Python passed arguments
 * @return The position
 */
static PyObject*
point_get_pos(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetPointPos(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetPointVel() function
 * @param args Python passed arguments
 * @return The velocity
 */
static PyObject*
point_get_vel(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetPointVel(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetPointForce() function
 * @param args Python passed arguments
 * @return The velocity
 */
static PyObject*
point_get_force(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetPointForce(instance, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetPointM() function
 * @param args Python passed arguments
 * @return The velocity and the acceleration
 */
static PyObject*
point_get_m(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	double m[3][3];
	const int err = MoorDyn_GetPointM(instance, m);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyObject* sub = PyTuple_New(3);
		for (unsigned int j = 0; j < 3; j++) {
			PyTuple_SET_ITEM(sub, j, PyFloat_FromDouble(m[i][j]));
		}
		PyTuple_SET_ITEM(lst, i, sub);
	}

	return lst;
}

/** @brief Wrapper to MoorDyn_GetPointNAttached() function
 * @param args Python passed arguments
 * @return The number of attached lines
 */
static PyObject*
point_get_nattached(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetPointNAttached(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetPointNAttached() function
 * @param args Python passed arguments
 * @return The number of attached lines
 */
static PyObject*
point_get_attached(PyObject*, PyObject* args)
{
	PyObject* capsule;
	unsigned int i;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &i))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	MoorDynLine l;
	int e;
	const int err = MoorDyn_GetPointAttached(instance, i, &l, &e);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyv = PyTuple_New(2);
	PyTuple_SET_ITEM(pyv, 0, PyCapsule_New((void*)l, line_capsule_name, NULL));
	PyTuple_SET_ITEM(pyv, 1, PyLong_FromLong(e));
	return pyv;
}

/** @brief Wrapper to MoorDyn_SavePointVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
point_save_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDynPoint instance =
	    (MoorDynPoint)PyCapsule_GetPointer(capsule, point_capsule_name);
	if (!instance)
		return NULL;

	return PyLong_FromLong(MoorDyn_SavePointVTK(instance, filepath));
}

//                                 Line.h
// =============================================================================

/** @brief Wrapper to MoorDyn_GetLineID() function
 * @param args Python passed arguments
 * @return The line id
 */
static PyObject*
line_get_id(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	int n;
	const int err = MoorDyn_GetLineID(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetLineN() function
 * @param args Python passed arguments
 * @return The number of segments
 */
static PyObject*
line_get_n(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	unsigned int n;
	const int err = MoorDyn_GetLineN(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(n);
}

/** @brief Wrapper to MoorDyn_GetLineUnstretchedLength() function
 * @param args Python passed arguments
 * @return The unstretched length
 */
static PyObject*
line_get_ulen(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double l;
	const int err = MoorDyn_GetLineUnstretchedLength(instance, &l);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(l);
}

/** @brief Wrapper to MoorDyn_SetLineUnstretchedLength() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
line_set_ulen(PyObject*, PyObject* args)
{
	PyObject* capsule;
	double l;

	if (!PyArg_ParseTuple(args, "Od", &capsule, &l))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	const int err = MoorDyn_SetLineUnstretchedLength(instance, l);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_SetLineUnstretchedLength() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
line_set_ulenv(PyObject*, PyObject* args)
{
	PyObject* capsule;
	double v;

	if (!PyArg_ParseTuple(args, "Od", &capsule, &v))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	const int err = MoorDyn_SetLineUnstretchedLengthVel(instance, v);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_IsLineConstantEA() function
 * @param args Python passed arguments
 * @return The unstretched length
 */
static PyObject*
line_is_const_ea(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	int b;
	const int err = MoorDyn_IsLineConstantEA(instance, &b);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyLong_FromLong(b);
}

/** @brief Wrapper to MoorDyn_GetLineConstantEA() function
 * @param args Python passed arguments
 * @return The unstretched length
 */
static PyObject*
line_get_const_ea(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double ea;
	const int err = MoorDyn_GetLineConstantEA(instance, &ea);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(ea);
}

/** @brief Wrapper to MoorDyn_SetLineConstantEA() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
line_set_const_ea(PyObject*, PyObject* args)
{
	PyObject* capsule;
	double ea;

	if (!PyArg_ParseTuple(args, "Od", &capsule, &ea))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	const int err = MoorDyn_SetLineConstantEA(instance, ea);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_IsLinePressBend() function
 * @param args Python passed arguments
 * @return The unstretched length
 */
static PyObject*
line_is_pbend(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	int b;
	const int err = MoorDyn_IsLinePressBend(instance, &b);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyBool_FromLong(b);
}

/** @brief Wrapper to MoorDyn_SetLinePressBend() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
line_set_pbend(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int b;

	if (!PyArg_ParseTuple(args, "Ob", &capsule, &b))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	const int err = MoorDyn_SetLinePressBend(instance, b);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_SetLinePressInt() function
 * @param args Python passed arguments
 * @return None
 */
static PyObject*
line_set_pint(PyObject*, PyObject* args)
{
	PyObject* capsule;
	PyObject* pvals;

	if (!PyArg_ParseTuple(args, "OO", &capsule, &pvals))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	pvals = PySequence_Fast(pvals, "2 argument must be iterable");
	if (!pvals)
		return NULL;
	unsigned int n;
	int err;
	err = MoorDyn_GetLineN(instance, &n);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}
	if (PySequence_Fast_GET_SIZE(pvals) != n + 1) {
		std::stringstream err;
		err << "2nd argument must have " << n + 1 << " components";
		PyErr_SetString(PyExc_ValueError, err.str().c_str());
		return NULL;
	}

	// Convert them to C arrays that MoorDyn might handle
	double* pin = py_iterable_to_double(pvals);
	Py_DECREF(pvals);
	if (!pin) {
		return NULL;
	}

	err = MoorDyn_SetLinePressInt(instance, pin);
	free(pin);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return Py_None;
}

/** @brief Wrapper to MoorDyn_GetLineNodePos() function
 * @param args Python passed arguments
 * @return The position
 */
static PyObject*
line_get_node_pos(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodePos(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeVel() function
 * @param args Python passed arguments
 * @return The position
 */
static PyObject*
line_get_node_vel(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeVel(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeForce() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_force(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeForce(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeTen() function
 * @param args Python passed arguments
 * @return The tension
 */
static PyObject*
line_get_node_ten(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeTen(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeBendStiff() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_bend(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeBendStiff(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeWeight() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_w(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeWeight(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeDrag() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_drag(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeDrag(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeFroudeKrilov() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_froudekrylov(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeFroudeKrilov(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeSeaBedForce() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_seabed(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double r[3];
	const int err = MoorDyn_GetLineNodeSeabedForce(instance, node, r);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* pyr = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++)
		PyTuple_SET_ITEM(pyr, i, PyFloat_FromDouble(r[i]));

	return pyr;
}

/** @brief Wrapper to MoorDyn_GetLineNodeCurv() function
 * @param args Python passed arguments
 * @return The curvature
 */
static PyObject*
line_get_node_curv(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double t;
	const int err = MoorDyn_GetLineNodeCurv(instance, node, &t);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(t);
}

/** @brief Wrapper to MoorDyn_GetLineNodeM() function
 * @param args Python passed arguments
 * @return The force
 */
static PyObject*
line_get_node_m(PyObject*, PyObject* args)
{
	PyObject* capsule;
	int node;

	if (!PyArg_ParseTuple(args, "Oi", &capsule, &node))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double m[3][3];
	const int err = MoorDyn_GetLineNodeM(instance, node, m);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	PyObject* lst = PyTuple_New(3);
	for (unsigned int i = 0; i < 3; i++) {
		PyObject* sub = PyTuple_New(3);
		for (unsigned int j = 0; j < 3; j++) {
			PyTuple_SET_ITEM(sub, j, PyFloat_FromDouble(m[i][j]));
		}
		PyTuple_SET_ITEM(lst, i, sub);
	}

	return lst;
}


/** @brief Wrapper to MoorDyn_GetLineFairTen() function
 * @param args Python passed arguments
 * @return The tension magnitude
 */
static PyObject*
line_get_fairlead_tension(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double t;
	const int err = MoorDyn_GetLineFairTen(instance, &t);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(t);
}

/** @brief Wrapper to MoorDyn_GetLineMaxTen() function
 * @param args Python passed arguments
 * @return The tension magnitude
 */
static PyObject*
line_get_max_tension(PyObject*, PyObject* args)
{
	PyObject* capsule;

	if (!PyArg_ParseTuple(args, "O", &capsule))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	double t;
	const int err = MoorDyn_GetLineMaxTen(instance, &t);
	if (err != 0) {
		PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
		return NULL;
	}

	return PyFloat_FromDouble(t);
}

/** @brief Wrapper to MoorDyn_SaveLineVTK() function
 * @param args Python passed arguments
 * @return 0 in case of success, an error code otherwise
 */
static PyObject*
line_save_vtk(PyObject*, PyObject* args)
{
	PyObject* capsule;
	char* filepath = NULL;

	if (!PyArg_ParseTuple(args, "Os", &capsule, &filepath))
		return NULL;

	MoorDynLine instance =
	    (MoorDynLine)PyCapsule_GetPointer(capsule, line_capsule_name);
	if (!instance)
		return NULL;

	return PyLong_FromLong(MoorDyn_SaveLineVTK(instance, filepath));
}

static PyMethodDef moordyn_methods[] = {
	{ "create", create, METH_VARARGS, "Creates the MoorDyn system" },
	{ "n_coupled_dof",
	  n_coupled_dof,
	  METH_VARARGS,
	  "Get the number of coupled Degrees Of Freedom" },
	{ "set_verbosity",
	  set_verbosity,
	  METH_VARARGS,
	  "Set the instance verbosity level" },
	{ "set_logfile",
	  set_logfile,
	  METH_VARARGS,
	  "Set the filepath of the output log file" },
	{ "set_loglevel",
	  set_loglevel,
	  METH_VARARGS,
	  "Set the instance log file verbosity" },
	{ "log",
	  log,
	  METH_VARARGS,
	  "Log a message to both the terminal screen and the log file" },
	{ "init", init, METH_VARARGS, "Initializes MoorDyn" },
	{ "step",
	  step,
	  METH_VARARGS,
	  "simulates the mooring system starting at time t and ending at time "
	  "t+d" },
	{ "close",
	  close,
	  METH_VARARGS,
	  "deallocates the variables used by MoorDyn" },
	{ "get_waves", get_waves, METH_VARARGS, "Get the waves manager instance" },
	{ "get_seafloor",
	   get_seafloor,
	   METH_VARARGS,
	   "Get the seafloor instance" },
	{ "ext_wave_init",
	  ext_wave_init,
	  METH_VARARGS,
	  "Initialize the externally handled waves" },
	{ "ext_wave_n",
	  ext_wave_n,
	  METH_VARARGS,
	  "Get the number of points where the wave kinematics shall be "
	  "provided" },
	{ "ext_wave_coords",
	  ext_wave_coords,
	  METH_VARARGS,
	  "Get the coordinates where the wave kinematics shall be provided" },
	{ "ext_wave_set", ext_wave_set, METH_VARARGS, "Set the wave kinematics" },
	{ "get_number_bodies",
	  get_number_bodies,
	  METH_VARARGS,
	  "Get the number of rigid bodies" },
	{ "get_body", get_body, METH_VARARGS, "Get a rigid body" },
	{ "get_number_rods",
	  get_number_rods,
	  METH_VARARGS,
	  "Get the number of rods" },
	{ "get_rod", get_rod, METH_VARARGS, "Get a rod" },
	{ "get_number_points",
	  get_number_points,
	  METH_VARARGS,
	  "Get the number of points" },
	{ "get_point", get_point, METH_VARARGS, "Get a point" },
	{ "get_number_lines",
	  get_number_lines,
	  METH_VARARGS,
	  "Get the number of mooring lines" },
	{ "get_line", get_line, METH_VARARGS, "Get a mooring line" },
	{ "get_fast_tens",
	  get_fast_tens,
	  METH_VARARGS,
	  "Get vertical and horizontal forces in the mooring lines" },
	{ "serialize",
	  serialize,
	  METH_VARARGS,
	  "Serialize the system to a bytes array" },
	{ "deserialize",
	  deserialize,
	  METH_VARARGS,
	  "Deserialize the system from a bytes array" },
	{ "save", save, METH_VARARGS, "Save the system to a file" },
	{ "load", load, METH_VARARGS, "Load the system from a file" },
	{ "save_vtk",
	  save_vtk,
	  METH_VARARGS,
	  "Save a .vtm file of the whole system" },
	{ "waves_getkin", waves_getkin, METH_VARARGS, "Get waves kinematics" },
	{ "seafloor_getdepth",
	  seafloor_getdepth,
	  METH_VARARGS,
	  "Get depth at a point" },
	{ "seafloor_getavgdepth",
	  seafloor_getavgdepth,
	  METH_VARARGS,
	  "Get the average depth" },
	{ "seafloor_getmindepth",
	  seafloor_getmindepth,
	  METH_VARARGS,
	  "Get shallowest depth" },
	{ "body_get_id", body_get_id, METH_VARARGS, "Get the body id" },
	{ "body_get_type", body_get_type, METH_VARARGS, "Get the body type" },
	{ "body_get_state", body_get_state, METH_VARARGS, "Get the body state" },
	{ "body_get_pos", body_get_pos, METH_VARARGS, "Get the body pos" },
	{ "body_get_angle", body_get_angle, METH_VARARGS, "Get the body angle" },
	{ "body_get_vel", body_get_vel, METH_VARARGS, "Get the body velocity" },
	{ "body_get_angvel", body_get_angvel, METH_VARARGS, "Get the body ang vel" },
	{ "body_get_force", body_get_force, METH_VARARGS, "Get the body force" },
	{ "body_get_m", body_get_m, METH_VARARGS, "Get the body mass" },
	{ "body_save_vtk",
	  body_save_vtk,
	  METH_VARARGS,
	  "Save a .vtp file of the body" },
	{ "body_use_vtk",
	  body_use_vtk,
	  METH_VARARGS,
	  "Load a representation model for the body" },
	{ "rod_get_id", rod_get_id, METH_VARARGS, "Get the rod id" },
	{ "rod_get_type", rod_get_type, METH_VARARGS, "Get the rod type" },
	{ "rod_get_force", rod_get_force, METH_VARARGS, "Get the rod force" },
	{ "rod_get_m", rod_get_m, METH_VARARGS, "Get the rod mass" },
	{ "rod_get_n", rod_get_n, METH_VARARGS, "Get the rod number of segments" },
	{ "rod_get_node_pos",
	  rod_get_node_pos,
	  METH_VARARGS,
	  "Get a rod node position" },
	{ "rod_get_node_vel",
	  rod_get_node_vel,
	  METH_VARARGS,
	  "Get a rod node velocity" },
	{ "rod_save_vtk",
	  rod_save_vtk,
	  METH_VARARGS,
	  "Save a .vtp file of the rod" },
	{ "point_get_id", point_get_id, METH_VARARGS, "Get the point id" },
	{ "point_get_type", point_get_type, METH_VARARGS, "Get the point type" },
	{ "point_get_pos",
	  point_get_pos,
	  METH_VARARGS,
	  "Get the point position" },
	{ "point_get_vel",
	  point_get_vel,
	  METH_VARARGS,
	  "Get the point velocity" },
	{ "point_get_force",
	  point_get_force,
	  METH_VARARGS,
	  "Get the point force" },
	{ "point_get_m", point_get_m, METH_VARARGS, "Get the point mass matrix" },
	{ "point_get_nattached",
	  point_get_nattached,
	  METH_VARARGS,
	  "Get the number of attached lines" },
	{ "point_get_attached",
	  point_get_attached,
	  METH_VARARGS,
	  "Get an attached line" },
	{ "point_save_vtk",
	  point_save_vtk,
	  METH_VARARGS,
	  "Save a .vtp file of the point" },
	{ "line_get_id", line_get_id, METH_VARARGS, "Get the line id" },
	{ "line_get_n",
	  line_get_n,
	  METH_VARARGS,
	  "Get the line number of segments" },
	{ "line_get_ulen",
	  line_get_ulen,
	  METH_VARARGS,
	  "Get the line unstretched length" },
	{ "line_set_ulen",
	  line_set_ulen,
	  METH_VARARGS,
	  "Set the line unstretched length" },
	{ "line_set_ulenv",
	  line_set_ulenv,
	  METH_VARARGS,
	  "Set the line rate of change of unstretched length" },
	{ "line_is_const_ea",
	  line_is_const_ea,
	  METH_VARARGS,
	  "Check if the stiffness is constant" },
	{ "line_get_const_ea",
	  line_get_const_ea,
	  METH_VARARGS,
	  "Get the constant stiffness" },
	{ "line_set_const_ea",
	  line_set_const_ea,
	  METH_VARARGS,
	  "Set the constant stiffness" },
	{ "line_is_pbend",
	  line_is_pbend,
	  METH_VARARGS,
	  "Get whether pressure bending forces are considered or not" },
	{ "line_set_pbend",
	  line_set_pbend,
	  METH_VARARGS,
	  "Set whether pressure bending forces are considered or not" },
	{ "line_set_pint",
	  line_set_pint,
	  METH_VARARGS,
	  "Set the internal flow pressure" },
	{ "line_get_node_pos",
	  line_get_node_pos,
	  METH_VARARGS,
	  "Get a line node position" },
	{ "line_get_node_vel",
	  line_get_node_vel,
	  METH_VARARGS,
	  "Get a line node velocity" },
	{ "line_get_node_force",
	  line_get_node_force,
	  METH_VARARGS,
	  "Get a line node net force" },
	{ "line_get_node_ten",
	  line_get_node_ten,
	  METH_VARARGS,
	  "Get a line node tension" },
	{ "line_get_node_bend",
	  line_get_node_bend,
	  METH_VARARGS,
	  "Get a line node bending stiffness force" },
	{ "line_get_node_w",
	  line_get_node_w,
	  METH_VARARGS,
	  "Get a line node weight" },
	{ "line_get_node_drag",
	  line_get_node_drag,
	  METH_VARARGS,
	  "Get a line node drag force" },
	{ "line_get_node_froudekrylov",
	  line_get_node_froudekrylov,
	  METH_VARARGS,
	  "Get a line node Froude-Krylov force" },
	{ "line_get_node_seabed",
	  line_get_node_seabed,
	  METH_VARARGS,
	  "Get a line node seabed reaction" },
	{ "line_get_node_curv",
	  line_get_node_curv,
	  METH_VARARGS,
	  "Get a line node curvature" },
	{ "line_get_node_m",
	  line_get_node_m,
	  METH_VARARGS,
	  "Get a line node mass matrix" },
	{ "line_get_fairlead_tension",
	  line_get_fairlead_tension,
	  METH_VARARGS,
	  "Get the line fairlead tension magnitude" },
	{ "line_get_max_tension",
	  line_get_max_tension,
	  METH_VARARGS,
	  "Get the line maximum tension magnitude" },
	{ "line_save_vtk",
	  line_save_vtk,
	  METH_VARARGS,
	  "Save a .vtp file of the line" },
	{ NULL, NULL, 0, NULL }
};

static struct PyModuleDef moordyn_module = { PyModuleDef_HEAD_INIT,
	                                         "cmoordyn",
	                                         "MoorDyn python wrapper",
	                                         -1,
	                                         moordyn_methods };

PyMODINIT_FUNC
PyInit_cmoordyn(void)
{
	PyObject* m = PyModule_Create(&moordyn_module);
	if (m == NULL) {
		return NULL;
	}
	return m;
}
