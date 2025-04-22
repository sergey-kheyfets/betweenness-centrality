#pragma once

#include "brandes.h"
#include "components.h"
#include "io.h"
#include "paths.h"
#include "traits.h"
#include "utils.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/range/iterator_range_core.hpp>

std::vector<Graph> readGraphIntoComponents(std::filesystem::path &path);

template <class Graph> void printBasicInfo(const Graph &graph) {
  std::cout << "Graph has " << boost::num_vertices(graph) << " vertices and "
            << boost::num_edges(graph) << " edges." << std::endl;
}

template <class Graph> void printExactCentrality(Graph &graph) {
  std::map<int64_t, long double> result;

  auto bc = getExactBetwennessCentrality(graph);
  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    result[graph[vertex].id] = bc[vertex];
  }
  std::cout << std::endl << "Exact BC: " << result << "\n\n";
}

template <class Graph>
void testPathEstimation(Graph &graph, size_t num_samples) {
  std::vector<size_t> distances(boost::num_vertices(graph));

  auto centroids = getAndApplyCentroids(graph);

  auto index_map = (boost::get(boost::vertex_index, graph));
  CentroidsPathEstimator estimator{graph, centroids};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, boost::num_vertices(graph) - 1);

  auto begin = *(boost::vertices(graph).first);

  std::vector<int64_t> exact, estimated;

  std::cout << "Calculating paths...\n";
  for (size_t i = 0; i != num_samples; ++i) {
    auto first = begin + dist(gen);
    auto second = begin + dist(gen);
    exact.push_back(getDistances(graph, first)[index_map[second]]);
    estimated.push_back(estimator.Estimate(first, second));
  }

  long double mae = 0;
  long double mse = 0;

  long double mean_exact =
      std::accumulate(exact.begin(), exact.end(), 0.0) / num_samples;
  long double mean_estimated =
      std::accumulate(estimated.begin(), estimated.end(), 0.0) / num_samples;

  for (size_t i = 0; i != num_samples; ++i) {
    mae += std::abs(exact[i] - estimated[i]);
    mse += (exact[i] - estimated[i]) * (exact[i] - estimated[i]);
  }
  mae /= num_samples;
  mse /= num_samples;

  std::cout << "Mean exact distance: " << mean_exact << std::endl;
  std::cout << "Mean estimated distance: " << mean_estimated << std::endl;
  std::cout << "Mean absolute error: " << mae << std::endl;
  std::cout << "Mean squared error: " << mse << std::endl;
}