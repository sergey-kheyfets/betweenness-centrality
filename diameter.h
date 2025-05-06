#pragma once

#include <optional>

#include "paths.h"

#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

template <class Graph> size_t GetEstimatedDiameter(const Graph &graph) {
  auto start = *(boost::vertices(graph).first);
  auto distances = GetDistances(graph, start);

  size_t first_max = 0;
  size_t second_max = 0;
  for (const auto &distance : distances) {
    if (distance >= first_max) {
      second_max = first_max;
      first_max = distance;
    } else if (distance > second_max) {
      second_max = distance;
    }
  }

  return first_max + second_max;
}

template <class Graph> size_t GetExactDiameter(const Graph &graph) {
  size_t max = 0;
  for (const auto &start : boost::make_iterator_range(boost::vertices(graph))) {
    max = std::max(max, std::ranges::max(GetDistances(graph, start)));
  }

  return max;
}