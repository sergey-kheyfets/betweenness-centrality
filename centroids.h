#pragma once

#include <algorithm>
#include <boost/graph/filtered_graph.hpp>
#include <queue>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include "traits.h"

template <class Graph>
struct AdjacencyDegreeVisitor : public boost::default_bfs_visitor {
  std::vector<size_t> &adjacency_degrees;

  AdjacencyDegreeVisitor(std::vector<size_t> &adjacency_degrees)
      : adjacency_degrees(adjacency_degrees) {}

  void examine_edge(const Edge<Graph> &edge, const Graph &graph) const {
    adjacency_degrees[boost::source(edge, graph)] +=
        boost::degree(boost::target(edge, graph), graph);
  }
};

template <class Graph>
std::vector<size_t> getAdjacencyDegrees(const Graph &graph) {
  std::vector<size_t> adjacency_degrees(boost::num_vertices(graph));
  boost::breadth_first_search(
      graph, 0,
      boost::visitor(AdjacencyDegreeVisitor<Graph>(adjacency_degrees)));

  return adjacency_degrees;
}

template <class Graph>
std::vector<long double>
getAdjacencyEntropy(const Graph &graph,
                    const std::vector<size_t> &adjacency_degrees) {
  std::vector<long double> entropy(boost::num_vertices(graph), 0);
  typename boost::property_map<Graph, boost::vertex_index_t>::type index_map =
      get(boost::vertex_index, graph);
  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    auto index = index_map[vertex];
    long double degree = boost::degree(vertex, graph);
    for (const auto &neighbour :
         boost::make_iterator_range(boost::adjacent_vertices(vertex, graph))) {
      long double selection_probability =
          degree / adjacency_degrees[index_map[neighbour]];
      entropy[index] +=
          selection_probability * std::log2(selection_probability);
    }
    entropy[index] *= -1;
  }

  return entropy;
}

template <class Graph>
size_t getCentroidsNumber(const Graph &graph, long double c,
                          long double delta) {
  auto diameter = getEstimatedDiameter(graph);
  if (diameter <= 2) {
    return 1;
  }
  return std::min(boost::num_vertices(graph),
                  static_cast<size_t>(c * (std::floor(std::log2(diameter - 2)) +
                                           1 + std::log(1 / delta))));
}

template <class Graph>
std::vector<Vertex<Graph>> getBestCentroids(const Graph &graph, size_t number) {
  auto adjacency_degrees = getAdjacencyDegrees(graph);
  auto entropy = getAdjacencyEntropy(graph, adjacency_degrees);

  auto [begin, end] = boost::vertices(graph);

  typename boost::property_map<Graph, boost::vertex_index_t>::type index_map =
      get(boost::vertex_index, graph);

  std::vector<Vertex<Graph>> heap(number);
  for (size_t i = 0; i != number; ++i) {
    heap[i] = *(begin + i);
  }

  auto compare = [&index_map, &entropy](const Vertex<Graph> &first,
                                        const Vertex<Graph> &second) {
    return entropy[index_map[first]] > entropy[index_map[second]];
  };

  std::make_heap(heap.begin(), heap.end(), compare);

  for (size_t i = number; i < boost::num_vertices(graph); ++i) {
    heap.push_back(*(begin + i));
    std::push_heap(heap.begin(), heap.end(), compare);

    std::pop_heap(heap.begin(), heap.end(), compare);
    heap.pop_back();
  }

  return heap;
}

template <class Graph>
void fillNearestCentroids(Graph *graph,
                          const std::vector<Vertex<Graph>> &centroids) {
  typename boost::property_map<Graph, boost::vertex_index_t>::type index_map =
      get(boost::vertex_index, *graph);

  std::vector<size_t> distances(boost::num_vertices(*graph), -1);
  std::queue<Vertex<Graph>> queue;
  for (const auto &centroid : centroids) {
    distances[index_map[centroid]] = 0;
    queue.push(centroid);
    (*graph)[centroid].nearest_centroid = index_map[centroid];
  }

  while (!queue.empty()) {
    auto vertex = queue.front();
    queue.pop();

    auto index = index_map[vertex];
    for (const auto &neighbour :
         boost::make_iterator_range(boost::adjacent_vertices(vertex, *graph))) {
      auto neighbour_index = index_map[neighbour];
      if (distances[neighbour_index] == -1) {
        distances[neighbour_index] = distances[vertex] + 1;
        queue.push(neighbour);
        (*graph)[neighbour].nearest_centroid =
            (*graph)[vertex].nearest_centroid;
      }
    }
  }
}

template <class Graph>
auto getCentroidSubgraphView(const Graph &graph,
                             const Vertex<Graph> &centroid) {
  std::function filter{[&](Vertex<Graph> vertex) {
    return graph[vertex].nearest_centroid == graph[centroid].nearest_centroid;
  }};
  return boost::filtered_graph(graph, boost::keep_all{}, filter);
}

template <class Graph>
std::vector<Vertex<Graph>> getAndApplyCentroids(Graph &graph,
                                                long double c = 0.33,
                                                long double delta = 0.1) {
  auto centroids_number = getCentroidsNumber(graph, c, delta);
  auto centroids = getBestCentroids(graph, centroids_number);
  fillNearestCentroids(&graph, centroids);
  return centroids;
}