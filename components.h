#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <concepts>
#include <vector>

#include "traits.h"

template <class Graph>
  requires(!std::is_lvalue_reference<Graph>::value)
auto splitIntoComponents(Graph &&graph) {
  std::vector<size_t> component_map(boost::num_vertices(graph));
  auto num_components = boost::connected_components(graph, &component_map[0]);
  std::vector<Graph> result(num_components);

  std::map<Vertex<Graph>, Vertex<Graph>> global_to_local_map;

  for (const auto &vertex :
       boost::make_iterator_range(boost::vertices(graph))) {
    auto &component = result[component_map[vertex]];
    global_to_local_map[vertex] = boost::add_vertex(component);
    component[global_to_local_map[vertex]].id = graph[vertex].id;
  }

  for (const auto &edge : boost::make_iterator_range(boost::edges(graph))) {
    auto &component = result[component_map[boost::target(edge, graph)]];
    boost::add_edge(global_to_local_map[boost::source(edge, graph)],
                    global_to_local_map[boost::target(edge, graph)], component);
  }

  return result;
}