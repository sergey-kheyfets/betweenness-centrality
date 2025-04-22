#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <iostream>
#include <map>
#include <stack>

#include "centroids.h"
#include "traits.h"

template <class Graph>
class BrandesVisitor : public boost::default_bfs_visitor {
public:
  std::map<Vertex<Graph>, size_t>& distances;
  std::map<Vertex<Graph>, size_t>& counts;
  std::map<size_t, std::vector<size_t>>& previous;
  std::stack<size_t>& stack;
  
  BrandesVisitor(std::map<Vertex<Graph>, size_t>& distances,
    std::map<Vertex<Graph>, size_t>& counts,
    std::map<size_t, std::vector<size_t>>& previous, std::stack<size_t>& stack)
      : distances(distances), counts(counts), previous(previous), stack(stack) {}

  void discover_vertex(const Vertex<Graph>& vertex, const Graph& graph) {
    stack.push(vertex);
  }

  void tree_edge(const Edge<Graph>& edge, const Graph& graph) {
    auto target = boost::target(edge, graph);
    auto source = boost::source(edge, graph);
    distances[target] = distances[source] + 1;
    Update_(source, target);
  }

  void non_tree_edge(const Edge<Graph>& edge, const Graph& graph) {
    auto target = boost::target(edge, graph);
    auto source = boost::source(edge, graph);
    if (distances[target] == distances[source] + 1) {
      Update_(source, target);
    }
  }

private:
  void Update_(const Vertex<Graph>& source, const Vertex<Graph>& target) {
    counts[target] += counts[source];
    previous[target].push_back(source);
  }
};

template <class Graph>
std::vector<long double> getExactBetwennessCentrality(const Graph& graph) {
    auto num_vertices = boost::num_vertices(graph);

    std::vector<long double> centrality(num_vertices, 0);

    for (const auto &start : boost::make_iterator_range(boost::vertices(graph))) {
        std::map<Vertex<Graph>, size_t> distances, counts;
        distances[start] = 0;
        counts[start] = 1;

        std::map<size_t, std::vector<size_t>> previous;
        std::stack<size_t> stack;
    
        boost::breadth_first_search(graph, start, boost::visitor(BrandesVisitor<Graph>(distances, counts, previous, stack))); 

        std::map<Vertex<Graph>, long double> deltas;
        
        while (!stack.empty()) {
            auto vertex = stack.top();
            stack.pop();
            for (auto neighbour : previous[vertex]) {
                deltas[neighbour] += (static_cast<long double>(counts[neighbour]) 
                    / counts[vertex]) * (1.0 + deltas[vertex]);
            } 
            if (vertex != start) {
                centrality[vertex] += deltas[vertex];
            }
        }
    }

    return centrality;
}