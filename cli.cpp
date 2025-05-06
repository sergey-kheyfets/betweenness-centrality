#include "components.h"
#include "io.h"
#include "traits.h"

#include "cli.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>

std::vector<Graph> ReadGraphIntoComponents(std::filesystem::path &path) {
  std::cout << "Reading graph..." << std::endl;

  auto components = SplitIntoComponents(ReadGraph(path));
  std::cout << "Number of components: " << components.size() << "\n";
  for (auto &graph : components) {
    PrintBasicInfo(graph);
  }
  return components;
}