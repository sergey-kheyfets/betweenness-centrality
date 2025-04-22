#include <boost/graph/graph_traits.hpp>

template <class Graph>
using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

template <class Graph>
using Edge = typename boost::graph_traits<Graph>::edge_descriptor;
