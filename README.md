# MINGLE

### Overview
This is an implementation of the MINGLE edge bundling algorithm based on
[this paper](http://yifanhu.net/PUB/edge_bundling.pdf). It is written in C/C++.

## Building MINGLE

### Installation of dependencies

To build mingle, you will need the following dependencies:

- CMake
- Boost (math and program options libraries)

In Ubuntu, you can install the needed dependencies with the following command:

```
sudo apt-get install cmake libboost-math-dev libboost-program-options-dev
```

### Creating the makefiles with CMAKE

CMake is a tool that automates the generation of makefiles. It also checks that
dependencies have been correctly installed to build the program. You can
generate makefiles with the debug option enabled or disabled.

To generate makefiles with the debug option enabled run the command:

```
cmake -DCMAKE_BUILD_TYPE=Debug CMakeLists.txt
```

To generate makefiles with the debug option disabled run the command:

```
cmake CMakeLists.txt
```

Enabling the debug option will allow you to use GDB (or the Eclipse debugger) to
debug the program. The program, however, will run up to 10 times slower. If you
will not be working with the code, you should generate makefiles with the debug
option disabled.

### Building the binary

To build the MINGLE binary run the following command:

```
make
```

Now you're done! you should have the mingle binary in your working directory.


## MINGLE input file formats

MINGLE expects two files describing the graph where the algorithm will run.

### Vertices file
The first file is a vertices file, it is a file that details the id, the x
coordinate and the y coordinate of each vertex. The first line of this file
should be an integer number, which is the total number of vertices. The rest of
the lines should have the vertex id, vertex x coordinate and vertex y coordinate
separated by a space, one vertex per line. For example, the header of
`test_data/philippines_list/philippines_list_vertices.txt` is the following:

```
220
pointId129 682.8690833333333 -393.8329166666667
pointId78 690.1295706997546 -520.118081965125
pointId86 1335.2876485368986 -183.8627140546455
```

The `philippines_list` graph has a total of 220 vertices. The first three are:

| Vertex id  | x coordinate       | y coordinate        |
| ---------- | ------------------ | ------------------- |
| pointId129 | 682.8690833333333  |  -393.8329166666667 |
| pointId78  | 690.1295706997546  |  -520.118081965125  |
| pointId86  | 1335.2876485368986 |  -183.8627140546455 |

The file contains 220 lines after the first line, each one detailing the
information about each vertex.

### Edges file
The second file an edges file, it is a file that details the pairs of vertices
that have an edge between them. The first line of this file is an integer
number, which is the total number of edges. The rest of the lines should be
pairs of vertex ids that indicate an edge between them.

For example, the header of
`test_data/philippines_list/philippines_list_vertices.txt` is the following:

```
219
pointId1 pointId2
pointId3 pointId2
pointId4 pointId2
```

This means that there are 219 edges in the file. The first three are the edges
between the vertices (pointId1, pointId2) then (pointId3, pointId2) and
(pointId4,pointId2)

## Running MINGLE

The mingle binary needs four arguments two run. It needs the path to the input
vertices file, the path to the input edges file, the path to output the
vertices created on the mingling process, and the path to output the final
edges in the graph. Running `./mingle --help` shows the following options:

```
Options:
  --help                Print help message
  --input_vertices arg  A file containing the list of input vertices in the
                        original graph. See the test_data directory for example
                        of the format.
  --input_edges arg     A file containing the list of input edges in the
                        original graph. These should be pairs of vertex ids.
                        The ids need to exist in the vertices file. See the
                        test_data directory for example of the format.
  --output_vertices arg The output vertices that were created in the mingling
                        process.
  --output_edges arg    The output edges of the final, mingled graph.
```

To run mingle on the philippines_list, we can run the following command:

```
./mingle --input_vertices test_data/philippines_list/philippines_list_vertices.txt --input_edges test_data/philippines_list/philippines_list_edges.txt --output_vertices output_vertices.txt --output_edges output_edges.txt
```

This will generate the files `output_vertices.txt` and `output_edges.txt`. The
`output_vertices.txt` file contains the details of each vertex, one vertex per
line. Each line has the vertex id, the vertex x coordinate and the vertex y
coordinate, each one separated by a single space. The `output_edges.txt` file
contains the information of all of the edges, one edge per line. Each line has
the two vertices that make the edge, and the edge's weight, comma separated.

## Visualizing the graph results of mingle

To see the results of mingle graphically, we can use the `plot_igraph.py`
script. To run the script, we need the igraph python library. In Ubuntu, we
can install igraph with the following command:

```
sudo apt-get install python-igraph
```
Now we are ready to visualize our graphs.

The `plot_igraph.py` script takes two required arguments. The first is a
`vertices` argument, which is the file path to a vertices file in the format
of mingle's vertices file. The second is an `edges` argument, which is the file
path to an edges file in the format of mingle's edges file. The script
`plot_igraph.py` can also take a `more_vertices` argument for an additional
vertices file with more vertices.

Let's walk through an example to clarify the usage of the script.

First, let's look at how the `philippines_list` graph looked before the mingle
process. We can do this with the following command:

```
python plot_igraph.py --vertices test_data/philippines_list/philippines_list_vertices.txt --edges test_data/philippines_list/philippines_list_edges.txt
```

The two input arguments on the command above are the original vertices and edges
files of the `philippines_list` graph. To visualize how the `philippines_list`
graph looked after the mingle process, we can use the following command:

```
python plot_igraph.py --vertices output_vertices.txt --edges output_edges.txt --more_vertices test_data/philippines_list/philippines_list_vertices.txt
```

The `vertices` and `edges` input arguments have changed to point to the output
vertices and output edges file paths. However, recall that the
`output_vertices.txt` file only has the newly created vertices in the mingling
process. Yet, the `output_edges.txt` has the edges of the whole graph.
Therefore, we also need to specify the `more_vertices` flag to include the
original vertices in the graph.

## FAQ

Create an issue if you have any questions about the ussage of mingle! Don't
be shy! I will do my best to be quick to reply.
