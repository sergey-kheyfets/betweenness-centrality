#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "traits.h"

struct VertexInfo {
  size_t nearest_centroid;
  size_t id;
};

// Define the graph using Boost adjacency_list
// We use vecS for both VertexList and EdgeList for efficiency
// undirectedS specifies that the graph is undirected
using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                    boost::undirectedS, VertexInfo>;

Graph readGraph(const std::filesystem::path &path);
void writeGraph(Graph *graph, const std::filesystem::path &path);