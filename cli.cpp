#include "traits.h"
#include "io.h"
#include "components.h"
#include "diameter.h"
#include "centroids.h"
#include "utils.h"

#include "cli.h"

#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

std::vector<Graph> readGraphIntoComponents(std::filesystem::path& path) {
  std::cout << "Reading graph..." << std::endl;

  auto components = splitIntoComponents(readGraph(path));
  std::cout << "Number of components: " << components.size() << "\n";  
  for (auto& graph : components) {
    printBasicInfo(graph);
  //   std::vector<size_t> distances(boost::num_vertices(graph));
  //   auto diameter = getEstimatedDiameter(graph);
  //   std::cout << "Estimated diameter: " << diameter << std::endl;
  //   // std::cout << "Exact diameter: " << getExactDiameter(graph) << std::endl;
  
  //   auto centroids_number = getCentroidsNumber(graph, 0.33, 0.1);
  //   std::cout << "Centroids number: " << centroids_number << std::endl;
  //   auto centroids = getBestCentroids(graph, centroids_number);
  //   std::cout << "Centroids: ";
  //   for (const auto &centroid : centroids) {
  //     std::cout << centroid << ' ';
  //   }
  //   std::cout << std::endl;
  
  //   fillNearestCentroids(&graph, centroids);
  }
  return components;
}