/***
 * fermat
 * -------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of fermat.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef PY_COMMON_H_
#define PY_COMMON_H_

#include <Python.h>
#include <fermat/core.h>

_fer_inline fer_real_t numberAsReal(PyObject *n)
{
    PyObject *onum;
    fer_real_t r;

    if (PyFloat_Check(n))
        return PyFloat_AsDouble(n);

    onum = PyNumber_Float(n);
    r = PyFloat_AsDouble(onum);
    Py_DECREF(onum);

    return r;
}

_fer_inline long numberAsLong(PyObject *n)
{
    PyObject *onum;
    long v;

    if (PyLong_Check(n))
        return PyLong_AsLong(n);

    onum = PyNumber_Long(n);
    v = PyFloat_AsDouble(onum);
    Py_DECREF(onum);

    return v;
}

#define CHECK_VEC2(o) \
    if (!PyObject_TypeCheck((o), &py_vec2_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Vec2"); \
        return NULL; \
    }

#define CHECK_VEC3(o) \
    if (!PyObject_TypeCheck((o), &py_vec3_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Vec3"); \
        return NULL; \
    }

#define CHECK_VEC4(o) \
    if (!PyObject_TypeCheck((o), &py_vec4_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Vec4"); \
        return NULL; \
    }

#define CHECK_QUAT(o) \
    if (!PyObject_TypeCheck((o), &py_quat_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Quat"); \
        return NULL; \
    }

#define CHECK_FLOAT2(o, ret) \
    if (!PyNumber_Check(o)){ \
        PyErr_SetString(PyExc_TypeError, "Expected float"); \
        return (ret); \
    }
#define CHECK_FLOAT(o) CHECK_FLOAT2(o, NULL)

#define CHECK_MAT3(o) \
    if (!PyObject_TypeCheck((o), &py_mat3_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Mat3"); \
        return NULL; \
    }

#define CHECK_MAT4(o) \
    if (!PyObject_TypeCheck((o), &py_mat4_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Mat4"); \
        return NULL; \
    }

#define CHECK_VEC(o) \
    if (!PyObject_TypeCheck((o), &py_vec_type)){ \
        PyErr_SetString(PyExc_TypeError, "Expected Vec"); \
        return NULL; \
    }

#endif /* PY_COMMON_H_ */
