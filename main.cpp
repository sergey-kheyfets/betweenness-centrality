#include <cstdint>
#include <iostream>
#include <random>

#include "centroids.h"
#include "diameter.h"
#include "io.h"
#include "paths.h"

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
  std::filesystem::path output_path = argv[1];
  output_path = output_path.parent_path() / "result.dot";
  writeGraph(&graph, {output_path});

  auto index_map = (boost::get(boost::vertex_index, graph));
  CentroidsPathEstimator estimator{graph, centroids};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, boost::num_vertices(graph) - 1);

  auto begin = *(boost::vertices(graph).first);

  std::vector<int64_t> exact, estimated;
  size_t number_of_trials = 100;
  for (size_t i = 0; i != number_of_trials; ++i) {
    auto first = begin + dist(gen);
    auto second = begin + dist(gen);
    exact.push_back(getDistances(graph, first)[index_map[second]]);
    estimated.push_back(estimator.Estimate(first, second));
  }

  long double mae = 0;
  long double mse = 0;

  long double mean_exact =
      std::accumulate(exact.begin(), exact.end(), 0.0) / number_of_trials;
  long double mean_estimated =
      std::accumulate(estimated.begin(), estimated.end(), 0.0) /
      number_of_trials;

  for (size_t i = 0; i != number_of_trials; ++i) {
    mae += std::abs(exact[i] - estimated[i]);
    mse += (exact[i] - estimated[i]) * (exact[i] - estimated[i]);
  }
  mae /= number_of_trials;
  mse /= number_of_trials;

  std::cout << "Mean exact distance: " << mean_exact << std::endl;
  std::cout << "Mean estimated distance: " << mean_estimated << std::endl;
  std::cout << "Mean absolute error: " << mae << std::endl;
  std::cout << "Mean squared error: " << mse << std::endl;
}
