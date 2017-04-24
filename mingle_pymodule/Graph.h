#include "Vertex.h"
#include <stdio.h>
#include <unordered_map>
#include <vector>

class Graph {
public:
	void add_vertex(std::string id, Vertex vertex);

	Vertex get_vertex(std::string id);

	bool has_vertex(std::string id);

	void add_edge(std::string vertex_source, std::string vertex_target);

	std::vector<std::string> get_target_vertices(std::string source_vertex_id);

	std::vector<std::string> get_source_vertices(std::string target_vertex_id);
private:
	std::unordered_map<std::string, Vertex> vertex_to_vertex_id;
	std::unordered_multimap<std::string, std::string> source_to_target_map;
	std::unordered_multimap<std::string, std::string> target_to_source_map;
};
