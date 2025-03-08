#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

template <class Graph> size_t getEstimatedDiameter(const Graph &graph) {
  std::vector<size_t> distances(boost::num_vertices(graph));
  auto recorder =
      boost::record_distances(distances.data(), boost::on_tree_edge{});

  boost::breadth_first_search(
      graph, 0, boost::visitor(boost::make_bfs_visitor(recorder)));

  size_t first_max = 0;
  size_t second_max = 0;
  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    auto distance = distances[vertex];
    if (distance >= first_max) {
      second_max = first_max;
      first_max = distance;
    } else if (distance > second_max) {
      second_max = distance;
    }
  }

  return first_max + second_max;
}

template <class Graph> size_t getExactDiameter(const Graph &graph) {
  size_t max = 0;
  for (const auto &start : boost::make_iterator_range(boost::vertices(graph))) {
    std::vector<size_t> distances(boost::num_vertices(graph));
    auto recorder =
        boost::record_distances(distances.data(), boost::on_tree_edge{});

    boost::breadth_first_search(
        graph, start, boost::visitor(boost::make_bfs_visitor(recorder)));

    for (const auto &vertex :
         boost::make_iterator_range(boost::vertices(graph))) {
      max = std::max(max, distances[vertex]);
    }
  }

  return max;
}