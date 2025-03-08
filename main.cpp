#include <iostream>

#include "centroids.h"
#include "diameter.h"
#include "io.h"

void printInfo(const Graph &graph) {
  std::cout << "Graph has " << boost::num_vertices(graph) << " vertices and "
            << boost::num_edges(graph) << " edges." << std::endl;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <dataset_filename>" << std::endl;
    return 1;
  }

  std::cout << "Reading graph..." << std::endl;
  Graph graph;
  try {
    graph = readGraph({argv[1]});
  } catch (...) {
    return 1;
  }

  size_t i = 0;
  printInfo(graph);

  std::vector<size_t> distances(boost::num_vertices(graph));
  auto diameter = getEstimatedDiameter(graph);
  std::cout << "Estimated diameter: " << diameter << std::endl;
  std::cout << "Exact diameter: " << getExactDiameter(graph) << std::endl;

  auto centroids_number = getCentroidsNumber(graph, 1.0 / 3, 0.1);
  std::cout << "Centroids number: " << centroids_number << std::endl;
  auto centroids = getBestCentroids(graph, centroids_number);
  std::cout << "Centroids: ";
  for (const auto &centroid : centroids) {
    std::cout << centroid << ' ';
  }
  std::cout << std::endl;

  fillNearestCentroids(&graph, centroids);
  // for (const auto& vertex :
  // boost::make_iterator_range(boost::vertices(graph))) {
  //     std::cout << graph[vertex].nearest_centroid << ' ';
  // }
  std::cout << std::endl;

  std::filesystem::path output_path = argv[1];
  output_path = output_path.parent_path() / "result.dot";
  writeGraph(&graph, {output_path});
}
