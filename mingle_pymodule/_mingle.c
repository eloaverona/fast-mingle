#include <Python.h>
#include <stdio.h>
#include "Graph.h"
#include "Vertex.h"

/* Docstrings */
static char module_docstring[] =
    "This module provides an interface for querying a bundled graph.";
static char load_vertices_docstring[] =
    "Load vertices from a vertices file.";

/* Available functions */
static PyObject *mingle_load_vertices(PyObject *self, PyObject *args);

/* Module specification */
static PyMethodDef module_methods[] = {
    {"load_vertices", mingle_load_vertices, METH_VARARGS, load_vertices_docstring},
    {NULL, NULL, 0, NULL}
};

/* Initialize the module */
PyMODINIT_FUNC init_mingle_graph(void)
{
    PyObject *m = Py_InitModule3("_mingle", module_methods, module_docstring);
    if (m == NULL)
        return;
}

static PyObject *mingle_load_vertices(PyObject *self, PyObject *args)
{
  PyStringObject *verticesFilePathObject;
  if (!PyArg_ParseTuple(args, "s", verticesFilePathObject)){
    return NULL;
  }
  char *verticesFilePath = PyString_AsString(verticesFilePathObject);

  std::ifstream verticesStream(verticesFilePath);
  if (!verticesStream.is_open()) {
    return PyErr_SetString(PyExc_RuntimeError, "Could not open vertices file. Check file path.");
  }
  int numPoints;
  verticesStream >> numPoints;
  for (int i = 0; i < numPoints; i++) {
    readNextPointInFile(verticesStream);
  }
  verticesStream.close();
  Py_RETURN_TRUE;
}

static readNextPointInFile(std::ifstream &verticesStream) {
  float xCoord;
  float yCoord;
  std::string pointIdString;
  try {
    verticesStream >> pointIdString >> xCoord >> yCoord;
  } catch (...) {
    return PyErr_SetString(PyExc_RuntimeError, "Vertices file has unexpected format.");
  }
  Vertex vertex = {xCoord, yCoord};
  add_vertex(pointIdString, vertex);
}