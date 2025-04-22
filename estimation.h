#include <map>
#include <vector>

#include "brandes.h"
#include "centroids.h"
#include "paths.h"
#include "traits.h"

template <class Graph>
std::vector<long double> estimateBetwennessCentralityWithCentroidsOnly(
    const Graph &graph, const std::vector<Vertex<Graph>> &centroids) {
  auto num_vertices = boost::num_vertices(graph);

  std::vector<long double> centrality(num_vertices, 0);

  std::vector<std::map<Vertex<Graph>, size_t>> distances_from_centroids;
  std::vector<std::map<Vertex<Graph>, size_t>> counts_from_centroids;

  for (const auto &centroid : centroids) {
    distances_from_centroids.push_back({});
    counts_from_centroids.push_back({});

    auto &distances = distances_from_centroids.back();
    auto &counts = counts_from_centroids.back();

    distances[centroid] = 0;
    counts[centroid] = 1;

    std::map<size_t, std::vector<size_t>> previous;
    std::stack<size_t> stack;

    auto subgraph = getCentroidSubgraphView(graph, centroid);
    boost::breadth_first_search(
        subgraph, centroid,
        boost::visitor(BrandesVisitor<decltype(subgraph)>(distances, counts,
                                                          previous, stack)));

    distances.clear();
    counts.clear();

    distances[centroid] = 0;
    counts[centroid] = 1;

    boost::breadth_first_search(
        graph, centroid,
        boost::visitor(ShortestPathCountsVisitor<Graph>(distances, counts)));

    std::map<Vertex<Graph>, long double> deltas;

    while (!stack.empty()) {
      auto vertex = stack.top();
      stack.pop();
      for (auto neighbour : previous[vertex]) {
        deltas[neighbour] +=
            (static_cast<long double>(counts[neighbour]) / counts[vertex]) *
            (1.0 + deltas[vertex]);
      }
      if (vertex != centroid) {
        centrality[vertex] += 2 * (num_vertices - 1) * deltas[vertex];
      }
    }
  }

  for (size_t i = 0; i != centroids.size(); ++i) {
    for (size_t j = 0; j != centroids.size(); ++j) {
      for (const auto &vertex :
           boost::make_iterator_range(boost::vertices(graph))) {
        if (distances_from_centroids[i][vertex] +
                distances_from_centroids[j][vertex] ==
            distances_from_centroids[i][j]) {
          centrality[vertex] +=
              distances_from_centroids[i].size() *
              distances_from_centroids[j].size() *
              static_cast<long double>(counts_from_centroids[i][vertex]) *
              counts_from_centroids[j][vertex] / counts_from_centroids[i][j];
        }
      }
    }
  }

  return centrality;
}