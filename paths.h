#pragma once

#include <unordered_map>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "traits.h"

template <class Graph> class CentroidsPathEstimator {
public:
  CentroidsPathEstimator(const Graph &graph,
                         const std::vector<Vertex<Graph>> &centroids)
      : graph_(graph), index_map_(boost::get(boost::vertex_index, graph)) {
    for (const auto &centroid : centroids) {
      centroid_distances_[index_map_[centroid]] =
          std::move(getDistances(graph, centroid));
    }
  }

  size_t Estimate(const Vertex<Graph> &first, const Vertex<Graph> &second) {
    size_t first_distance =
        centroid_distances_[graph_[first].nearest_centroid][index_map_[first]];
    size_t second_distance =
        centroid_distances_[graph_[second].nearest_centroid]
                           [index_map_[second]];
    return first_distance + second_distance;
  }

private:
  const Graph &graph_; // should be replaced by a shared_pointer (or maybe I'll
                       // change the interface instead)
  typename boost::property_map<Graph, boost::vertex_index_t>::type index_map_;
  std::unordered_map<size_t, std::vector<size_t>> centroid_distances_;
};

template <class Graph>
std::vector<size_t> getDistances(const Graph &graph,
                                 const Vertex<Graph> &start) {
  std::vector<size_t> distances(boost::num_vertices(graph));
  auto recorder =
      boost::record_distances(distances.data(), boost::on_tree_edge{});

  boost::breadth_first_search(
      graph, start, boost::visitor(boost::make_bfs_visitor(recorder)));

  return distances;
}