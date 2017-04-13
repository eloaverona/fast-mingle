#include <Python.h>
#include <stdio.h>

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
  int numPoints;
  verticesStream >> numPoints;
  for (int i = 0; i < numPoints; i++) {
    readNextPointInFile(verticesStream);
  }
  verticesStream.close();
}



static PyObject *load_graph(PyObject *self, PyObject *args)
{
	PyStringObject *nodesFilePathObject;
	PyStringObject *edgesFilePathObject;
	if (!PyArg_ParseTuple(args, "ss", nodesFilePath, edgesFilePath)){
      return NULL;
  }
  char *nodesFilePath = PyString_AsString(nodesFilePathObject);
  char *edgesFilePath = PyString_AsString(edgesFilePathObject);

  FILE *nodesFilePointer = fopen(nodesFilePath);
  assert(nodesFilePointer != nullptr);
  int nodeId;
  double xCoord;
  double yCoord;
  while(fscanf(nodesFilePointer, "%d", &nodeId) != EOF &&
  	  fscanf(nodesFilePointer, "%f", &xCoord) != EOF &&
  	  fscanf(nodesFilePointer, "%f", &yCoord) != EOF) {

  }
  fclose(nodesFilePointer);

	FILE *edgesFilePath = fopen(edgesFilePath);
	assert(edgesFilePointer != nullptr);
	int nodeId1;
	int nodeId2;
	int weight;
	while(fscanf(edgesFilePointer, "%d", &nodeId1) != EOF &&
		  fscanf(edgesFilePointer, "%d", &nodeId2) != EOF &&
		  fscanf(edgesFilePointer, "%d", &weight)) {
	}
    fclose(edgesFilePointer);

    fscanf(nodesFilePointer, "%d");
}

static PyObject *get_points_in_square(PyObject *self, PyObject *args)
{
  double x_lower, x_upper, y_lower, y_upper;
  /* Parse the input arguments */
  if (!PyArg_ParseTuple(args, "dddd", &x_lower, &x_upper, &y_lower, &y_upper)){
    return NULL;
  }
  points[] = [];
  int points_list_length = 1;
  PyObject *points_list = PyList_New(points_list_length)
  if(!points_list){
  	return NULL;
  }
  for(i = 0; i < points_list_length; i++) {
  	PyObject *point = Py_BuildValue("dd", points_list_length[i][0], points_list_length[i][1]);
  	if(!point) {
  		Py_DECREF(points_list);
  		return NULL;
  	}
  	PyList_SET_ITEM(points_list, i, point);
  }
}