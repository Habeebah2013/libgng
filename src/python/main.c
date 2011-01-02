#include <Python.h>
#include "vec3.h"
#include "vec4.h"
#include "quat.h"
#include "point_cloud.h"

static PyMethodDef mg_methods[] = {
    {NULL}  /* Sentinel */
};

static struct PyModuleDef mgmodule = {
   PyModuleDef_HEAD_INIT,
   "mg",   /* name of module */
   "TODO", /* module documentation, may be NULL */
   -1,       /* size of per-interpreter state of the module,
                or -1 if the module keeps state in global variables. */
   mg_methods
};

PyMODINIT_FUNC
PyInit_mg(void) 
{
    PyObject* m;

    m = PyModule_Create(&mgmodule);

    vec3Init(m);
    vec4Init(m);
    quatInit(m);
    pcInit(m);

    return m;
}
