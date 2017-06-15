#include "main.h"
#include "Graph.h"
#include "boost/program_options.hpp"
#include <iostream>
#include <string>

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

int main(int argc, char *argv[]) {
  try {
    boost::program_options::options_description desc("Options");
    desc.add_options()("help", "Print help message")(
        "input_vertices",
        boost::program_options::value<std::string>()->required(),
        "A file containing the list of input vertices in the original graph. "
        "See the test_data directory for example of the format.")(
        "input_edges", boost::program_options::value<std::string>()->required(),
        "A file containing the list of input edges in the original graph. "
        "These should be pairs of vertex ids. The ids need to exist in the "
        "vertices file. See the test_data directory for example of the "
        "format.")(
        "output_vertices",
        boost::program_options::value<std::string>()->required(),
        "The output vertices that were created in the mingling process.")(
        "output_edges",
        boost::program_options::value<std::string>()->required(),
        "The output edges of the final, mingled graph.")(
        "output_semanticTreeOfEdges",
        boost::program_options::value<std::string>()->required(),
        "Path for the semantic tree of the bundled edges"
    );

    boost::program_options::variables_map variables_map;
    try {
      boost::program_options::store(
          boost::program_options::parse_command_line(argc, argv, desc),
          variables_map);

      if (variables_map.count("help")) {
        std::cout << "MINGLE implements the mingle edge bundling algorithm "
                     "that bundles edges into graphs recursively. The "
                     "algorithm is detailed on this paper: "
                     "http://yifanhu.net/PUB/edge_bundling.pdf."
                  << std::endl
                  << desc << std::endl;
        return SUCCESS;
      }

      boost::program_options::notify(variables_map);
    } catch (boost::program_options::error &e) {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return ERROR_IN_COMMAND_LINE;
    }
    Graph *graph = new Graph();
    printf("Created graph.\n");
    graph->readVerticesAndEdges(
        variables_map["input_vertices"].as<std::string>().c_str(),
        variables_map["input_edges"].as<std::string>().c_str());
    graph->doRecursiveMingle();
    graph->writePointsAndEdges(
        variables_map["output_vertices"].as<std::string>().c_str(),
        variables_map["output_edges"].as<std::string>().c_str(),
        variables_map["output_semanticTreeOfEdges"].as<std::string>().c_str());


    return SUCCESS;
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception reached the top of main: " << e.what()
              << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  } catch (char const *msg) {
    std::cerr << "Unhandled Exception reached the top of main: " << msg
              << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }
  return SUCCESS;
}
