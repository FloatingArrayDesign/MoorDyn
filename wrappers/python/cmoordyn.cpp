#include <string>
#include <sstream>
#include <Python.h>
#include "MoorDyn2.h"

const char moordyn_capsule_name[] = "MoorDyn";
const char waves_capsule_name[] = "MoorDynWaves";
const char body_capsule_name[] = "MoorDynBody";

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

	return PyLong_FromLong(MoorDyn_NCoupledDOF(system));
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

	if (!PyArg_ParseTuple(args, "OOO", &capsule, &x_lst, &v_lst))
		return NULL;

	MoorDyn system =
	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
	if (!system)
		return NULL;

	// Get the expected number of DOFs in the provided arrays
	const unsigned int n_dof = MoorDyn_NCoupledDOF(system);

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
	const int err = MoorDyn_Init(system, x_arr, v_arr);
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
	const unsigned int n_dof = MoorDyn_NCoupledDOF(system);

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
	double forces[n_dof];
	const int err = MoorDyn_Step(system, x_arr, v_arr, forces, &t, &dt);
	;
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
	return PyLong_FromLong(err);
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
	const unsigned int nlines = MoorDyn_GetNumberLines(system);
	unsigned int n = 0;
	for (unsigned int i = 0; i < nlines; i++) {
		unsigned int nnodes;
		MoorDynLine l = MoorDyn_GetLine(system, i + 1);
		MoorDyn_GetLineNumberNodes(l, &nnodes);
		n += nnodes;
	}

	double* coords = (double*)malloc(n * 3 * sizeof(double));
	if (!coords) {
		PyErr_SetString(PyExc_RuntimeError, "Failure allocating memory");
		return NULL;
	}
	const int err = MoorDyn_GetWaveKinCoordinates(system, coords);
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
	const unsigned int nlines = MoorDyn_GetNumberLines(system);
	unsigned int n = 0;
	for (unsigned int i = 0; i < nlines; i++) {
		unsigned int nnodes;
		MoorDynLine l = MoorDyn_GetLine(system, i + 1);
		MoorDyn_GetLineNumberNodes(l, &nnodes);
		n += nnodes;
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
		const int err = MoorDyn_SetWaveKin(system, v_arr, a_arr, t);
		free(v_arr);
		free(a_arr);
		return PyLong_FromLong(err);
	}
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

	return PyLong_FromLong(MoorDyn_GetNumberBodies(system));
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

// /** @brief Wrapper to MoorDyn_GetNumberLines() function
//  * @param args Python passed arguments
//  * @return The number of mooring lines
//  */
// static PyObject*
// get_number_lines(PyObject*, PyObject* args)
// {
// 	PyObject* capsule;
//
// 	if (!PyArg_ParseTuple(args, "O", &capsule))
// 		return NULL;
//
// 	MoorDyn system =
// 	    (MoorDyn)PyCapsule_GetPointer(capsule, moordyn_capsule_name);
// 	if (!system)
// 		return NULL;
//
// 	return PyLong_FromLong(MoorDyn_GetNumberLines(system));
// }
//
// /** @brief Wrapper to MoorDyn_GetNumberLineNodes() function
//  * @param args Python passed arguments
//  * @return The number of line nodes
//  */
// static PyObject* get_number_line_nodes(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int line;
//
//     if (!PyArg_ParseTuple(args, "Oi", &capsule, &line))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                     moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     return PyLong_FromLong(MoorDyn_GetNumberLineNodes(system, line));
// }
//
// /** @brief Wrapper to MoorDyn_GetFairTen() function
//  * @param args Python passed arguments
//  * @return The tension on the fairlead
//  */
// static PyObject* get_fair_ten(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int line;
//
//     if (!PyArg_ParseTuple(args, "Oi", &capsule, &line))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                      moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     const double t = MoorDyn_GetFairTen(system, line);
//     return PyFloat_FromDouble(t);
// }
//
// /** @brief Wrapper to MoorDyn_GetConnectPos() function
//  * @param args Python passed arguments
//  * @return The position of the connection
//  */
// static PyObject* get_connect_pos(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int conn;
//
//     if (!PyArg_ParseTuple(args, "Oi", &capsule, &conn))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                      moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     double pos[3];
//     const int err = MoorDyn_GetConnectPos(system, conn, pos);
//     if (err != 0) {
//         PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
//         return NULL;
//     }
//
//     PyObject* lst = PyTuple_New(3);
//     for (unsigned int i = 0; i < 3; i++) {
//         PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(pos[i]));
//     }
//     return lst;
// }
//
// /** @brief Wrapper to MoorDyn_GetConnectForce() function
//  * @param args Python passed arguments
//  * @return The force on the connection
//  */
// static PyObject* get_connect_force(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int conn;
//
//     if (!PyArg_ParseTuple(args, "Oi", &capsule, &conn))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                      moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     double force[3];
//     const int err = MoorDyn_GetConnectForce(system, conn, force);
//     if (err != 0) {
//         PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
//         return NULL;
//     }
//
//     PyObject* lst = PyTuple_New(3);
//     for (unsigned int i = 0; i < 3; i++) {
//         PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(force[i]));
//     }
//     return lst;
// }
//
// /** @brief Wrapper to MoorDyn_GetNodePos() function
//  * @param args Python passed arguments
//  * @return The position of the node
//  */
// static PyObject* get_node_pos(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int line, node;
//
//     if (!PyArg_ParseTuple(args, "Oii", &capsule, &line, &node))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                      moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     double pos[3];
//     const int err = MoorDyn_GetNodePos(system, line, node, pos);
//     if (err != 0) {
//         PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
//         return NULL;
//     }
//
//     PyObject* lst = PyTuple_New(3);
//     for (int i = 0; i < 3; i++) {
//         PyTuple_SET_ITEM(lst, i, PyFloat_FromDouble(pos[i]));
//     }
//     return lst;
// }
//
// /** @brief Wrapper to MoorDyn_GetFASTtens() function
//  * @param args Python passed arguments
//  * @return The horizontal and vertical forces on the fairleads and
//  anchors
//  */
// static PyObject* get_fast_tens(PyObject*, PyObject* args)
// {
//     PyObject *capsule;
//     int num_lines;
//
//     if (!PyArg_ParseTuple(args, "Oi", &capsule, &num_lines))
//         return NULL;
//
//     MoorDyn system = (MoorDyn)PyCapsule_GetPointer(capsule,
//                                                      moordyn_capsule_name);
//     if (!system)
//         return NULL;
//
//     float *fair_h_ten = (float*)malloc(num_lines * sizeof(float));
//     float *fair_v_ten = (float*)malloc(num_lines * sizeof(float));
//     float *anch_h_ten = (float*)malloc(num_lines * sizeof(float));
//     float *anch_v_ten = (float*)malloc(num_lines * sizeof(float));
//     if (!fair_h_ten || !fair_v_ten || !anch_h_ten || !anch_v_ten)
//     {
//         PyErr_SetString(PyExc_MemoryError, "Failure allocating memory");
//         return NULL;
//     }
//
//     const int err = MoorDyn_GetFASTtens(system,
//                                         &num_lines,
//                                         fair_h_ten,
//                                         fair_v_ten,
//                                         anch_h_ten,
//                                         anch_v_ten);
//     if (err != 0) {
//         PyErr_SetString(PyExc_RuntimeError, "MoorDyn reported an error");
//         return NULL;
//     }
//
//     PyObject* fair_h_ten_lst = PyTuple_New(num_lines);
//     PyObject* fair_v_ten_lst = PyTuple_New(num_lines);
//     PyObject* anch_h_ten_lst = PyTuple_New(num_lines);
//     PyObject* anch_v_ten_lst = PyTuple_New(num_lines);
//     PyObject* lst = PyTuple_New(4);
//     if (!fair_h_ten_lst || !fair_v_ten_lst || !anch_h_ten_lst ||
//         !anch_v_ten_lst || !lst)
//     {
//         PyErr_SetString(PyExc_MemoryError, "Failure allocating memory");
//         return NULL;
//     }
//     for (int i = 0; i < num_lines; i++) {
//         PyTuple_SET_ITEM(fair_h_ten_lst, i,
//         PyFloat_FromDouble(fair_h_ten[i]));
//         PyTuple_SET_ITEM(fair_v_ten_lst, i,
//         PyFloat_FromDouble(fair_v_ten[i]));
//         PyTuple_SET_ITEM(anch_h_ten_lst, i,
//         PyFloat_FromDouble(anch_h_ten[i]));
//         PyTuple_SET_ITEM(anch_v_ten_lst, i,
//         PyFloat_FromDouble(anch_v_ten[i]));
//     }
//     free(fair_h_ten);
//     free(fair_v_ten);
//     free(anch_h_ten);
//     free(anch_v_ten);
//
//     PyTuple_SET_ITEM(lst, 0, fair_h_ten_lst);
//     PyTuple_SET_ITEM(lst, 1, fair_v_ten_lst);
//     PyTuple_SET_ITEM(lst, 2, anch_h_ten_lst);
//     PyTuple_SET_ITEM(lst, 3, anch_v_ten_lst);
//
//     return lst;
// }

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
	{ "ext_wave_init",
	  ext_wave_init,
	  METH_VARARGS,
	  "Initialize the externally handled waves" },
	{ "ext_wave_coords",
	  ext_wave_coords,
	  METH_VARARGS,
	  "Get the coordinates where the wave kinematics shall be provided" },
	{ "ext_wave_set", ext_wave_set, METH_VARARGS, "Set the wave kinematics" },
	{ "get_number_bodies",
	  get_number_bodies,
	  METH_VARARGS,
	  "Get the number of rigid bodies" },
	{ "get_body",
	  get_body,
	  METH_VARARGS,
	  "Get a rigid body" },

	/*
	{"get_number_lines", get_number_lines, METH_VARARGS,
	 "Get the number of mooring lines"},
	{"get_number_line_nodes", get_number_line_nodes, METH_VARARGS,
	 "Get the number of mooring line nodes"},
	{"get_fair_ten", get_fair_ten, METH_VARARGS,
	 "Get the tension of a line at the fairlead connection"},
	{"get_connect_pos", get_connect_pos, METH_VARARGS,
	 "Get a connection position"},
	{"get_connect_force", get_connect_force, METH_VARARGS,
	 "Get the force on a connection"},
	{"get_node_pos", get_node_pos, METH_VARARGS,
	 "Get the position of a line node"},
	{"get_fast_tens", get_fast_tens, METH_VARARGS,
	 "Get the horizontal and vertical tensions at the fairlead and anchor"},
	*/
	{ NULL, NULL, 0, NULL }
};

static struct PyModuleDef moordyn_module = { PyModuleDef_HEAD_INIT,
	                                         "moordyn",
	                                         "MoorDyn python wrapper",
	                                         -1,
	                                         moordyn_methods };

PyMODINIT_FUNC
PyInit_moordyn(void)
{
	PyObject* m = PyModule_Create(&moordyn_module);
	if (m == NULL) {
		return NULL;
	}
	return m;
}