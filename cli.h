#pragma once

#include "abra.h"
#include "brandes.h"
#include "components.h"
#include "io.h"
#include "paths.h"
#include "schedule.h"
#include "traits.h"
#include "utils.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/range/iterator_range_core.hpp>

std::vector<Graph> ReadGraphIntoComponents(std::filesystem::path &path);

template <class Graph> void PrintBasicInfo(const Graph &graph) {
  std::cout << "Graph has " << boost::num_vertices(graph) << " vertices and "
            << boost::num_edges(graph) << " edges." << std::endl;
}

template <class Graph> void PrintExactCentrality(Graph &graph) {
  std::map<int64_t, long double> result;

  auto bc = GetExactBetwennessCentrality(graph);
  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    result[graph[vertex].id] = bc[vertex];
  }
  std::cout << std::endl << "Exact BC: " << result << "\n";
}

template <class Graph>
void PrintCentralityAbra(Graph &graph, long double desired_accuracy,
                         long double desired_confidence,
                         long double rademacher_sample_size,
                         size_t max_iterations) {
  std::map<int64_t, std::pair<long double, long double>> result;

  std::unique_ptr<SamplingSchedule> schedule =
      std::make_unique<GeometricSamplingSchedule>(8, 2);
  auto abra =
      AbraEstimator(graph, std::move(schedule), desired_accuracy,
                    desired_confidence, rademacher_sample_size, max_iterations);
  auto abra_bc = abra.Estimate();

  auto exact_bc = GetExactBetwennessCentrality(graph);

  std::vector<size_t> exact_order, estimated_order;

  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    result[graph[vertex].id] = {exact_bc[vertex], abra_bc[vertex]};
    exact_order.push_back(graph[vertex].id);
    estimated_order.push_back(graph[vertex].id);
  }

  std::sort(exact_order.begin(), exact_order.end(), [&result](auto a, auto b) {
    return result[a].first > result[b].first;
  });
  std::sort(estimated_order.begin(), estimated_order.end(),
            [&result](auto a, auto b) {
              return result[a].second > result[b].second;
            });

  long double max_abs_diff = 0.0;
  long double max_rel_diff = 0.0;
  long double max_rel_diff_exact = -1;
  long double max_rel_diff_estimated = -1;
  for (auto [_, pair] : result) {
    max_abs_diff = std::max(max_abs_diff, std::abs(pair.first - pair.second));
    if (pair.first > 1e-6) {
      max_rel_diff = std::max(max_rel_diff, max_abs_diff / pair.first);
      max_rel_diff_exact = pair.first;
      max_rel_diff_estimated = pair.second;
    }
    std::cout << "exact " << pair.first << "; estimated " << pair.second
              << "\n";
  }

  std::cout << "Max abs. difference: " << max_abs_diff << "\n";
  std::cout << "Max rel. difference: " << 100 * max_rel_diff << "% ("
            << max_rel_diff_exact << " " << max_rel_diff_estimated << ")\n";

  std::cout << "Exact order: " << exact_order << "\n";
  std::cout << "Estimated order: " << estimated_order << "\n";
}

template <class Graph>
void TestPathEstimation(Graph &graph, size_t num_samples) {
  std::vector<size_t> distances(boost::num_vertices(graph));

  auto centroids = GetAndApplyCentroids(graph);

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
    exact.push_back(GetDistances(graph, first)[index_map[second]]);
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