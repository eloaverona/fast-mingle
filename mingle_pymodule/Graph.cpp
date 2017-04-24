#include "Graph.h"

Graph::Graph() {}

void Graph::add_vertex(std::string id, Vertex vertex){
	vertex_to_vertex_id[id] = vertex;
}

bool Graph::has_vertex(std::string id) {
	std::unordered_map<std::string, Vertex>::const_iterator find_result = vertex_to_vertex_id.find(id);
	if (find_result == vertex_to_vertex_id.end()){
		return false;
	} else {
		return true;
	}
}

Vertex Graph::get_vertex(std::string id){
	std::unordered_map<std::string, Vertex>::const_iterator result = vertex_to_vertex_id.find(id);
	if (find_result == vertex_to_vertex_id.end()){
		return NULL;
	} else {
		return result->second;
	}
}

void Graph::add_edge(std::string vertex_source, std::string vertex_target){
	source_to_target_map.insert(std::make_pair<std::string,std::string>(vertex_source, vertex_target));
	target_to_source_map.insert(std::make_pair<std::string,std::string>(vertex_target, vertex_source));
}

std::vector<std::string> Graph::get_target_vertices(std::string source_vertex_id) {
	std::vector<std::string> target_vertices;
	auto range = source_to_target_map.equal_range(source_vertex_id);

    for (auto vertex = range.first; vertex != range.second; ++vertex)
    {
        target_vertices.push_back(vertex);
    }
    return target_vertices;
}

std::vector<std::string> Graph::get_source_vertices(std::string target_vertex_id) {
	std::vector<std::string> source_vertices;
	auto range = target_to_source_map.equal_range(target_vertex_id);

    for (auto vertex = range.first; vertex != range.second; ++vertex)
    {
        source_vertices.push_back(vertex);
    }
    return source_vertices;
}
