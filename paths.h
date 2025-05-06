#pragma once

#include <boost/graph/detail/adjacency_list.hpp>

#include <deque>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/visitors.hpp>

#include <iostream>

#include "centroids.h"
#include "traits.h"

template <class Graph>
std::vector<size_t> GetDistances(const Graph &graph,
                                 const Vertex<Graph> &start) {
  std::vector<size_t> distances(boost::num_vertices(graph));
  auto recorder =
      boost::record_distances(distances.data(), boost::on_tree_edge{});

  boost::breadth_first_search(
      graph, start, boost::visitor(boost::make_bfs_visitor(recorder)));

  return distances;
}

template <class Graph> class CentroidsPathEstimator {
public:
  CentroidsPathEstimator(const Graph &graph,
                         const std::vector<Vertex<Graph>> &centroids)
      : graph_(graph), index_map_(boost::get(boost::vertex_index, graph)) {
    for (const auto &centroid : centroids) {
      centroid_distances_[index_map_[centroid]] =
          std::move(GetDistances(graph, centroid));
    }
  }

  size_t Estimate(const Vertex<Graph> &first, const Vertex<Graph> &second) {
    size_t first_distance =
        centroid_distances_[graph_[first].nearest_centroid][index_map_[first]];
    size_t centroids_distance =
        centroid_distances_[graph_[first].nearest_centroid]
                           [graph_[second].nearest_centroid];
    size_t second_distance =
        centroid_distances_[graph_[second].nearest_centroid]
                           [index_map_[second]];
    return first_distance + centroids_distance + second_distance;
  }

private:
  const Graph &graph_; // should be replaced by a shared_pointer (or maybe I'll
                       // change the interface instead)
  typename boost::property_map<Graph, boost::vertex_index_t>::type index_map_;
  std::unordered_map<size_t, std::vector<size_t>> centroid_distances_;
};

template <class Graph>
struct ShortestPathCountsVisitor : boost::default_bfs_visitor {
  std::map<Vertex<Graph>, size_t> &distances;
  std::map<Vertex<Graph>, size_t> &counts;

  ShortestPathCountsVisitor(std::map<Vertex<Graph>, size_t> &distances,
                            std::map<Vertex<Graph>, size_t> &counts)
      : distances(distances), counts(counts) {}

  void tree_edge(const Edge<Graph> &edge, const Graph &graph) {
    auto target = boost::target(edge, graph);
    auto source = boost::source(edge, graph);
    distances[target] = distances[source] + 1;
    counts[target] += counts[source];
  }

  void non_tree_edge(const Edge<Graph> &edge, const Graph &graph) {
    auto target = boost::target(edge, graph);
    auto source = boost::source(edge, graph);
    if (distances[target] == distances[source] + 1) {
      counts[target] += counts[source];
    }
  }
};

template <class Graph> struct CentroidInfo {
  Vertex<Graph> centroid;
  std::map<Vertex<Graph>, size_t> distances, counts;
  std::map<Vertex<Graph>, long double> deltas;
};

template <class Graph>
std::pair<std::vector<size_t>, std::map<size_t, size_t>>
GetShortestPathCountsAndDistancesWithFinish(const Graph &graph,
                                            Vertex<Graph> start,
                                            Vertex<Graph> finish) {
  std::vector<size_t> counts(boost::num_vertices(graph), 0);
  counts[start] = 1;

  std::map<size_t, size_t> distances;
  distances[start] = 0;

  std::queue<size_t> queue;
  queue.push(start);

  bool finish_reached = false;
  while (!queue.empty()) {
    auto vertex = queue.front();
    queue.pop();
    auto distance = distances[vertex];
    if (finish_reached && distance == distances[finish]) {
      break;
    }
    auto count = counts[vertex];
    for (auto neighbour :
         boost::make_iterator_range(boost::adjacent_vertices(vertex, graph))) {
      if (distances.find(neighbour) == distances.end()) {
        distances[neighbour] = distance + 1;
        queue.push(neighbour);
      }
      if (distances[neighbour] == distance + 1) {
        counts[neighbour] += count;
      }
      if (neighbour == finish) {
        finish_reached = true;
      }
    }
  }

  return {counts, distances};
}