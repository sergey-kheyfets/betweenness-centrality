#include "io.h"

// Function to add an edge between two nodes, adding vertices if necessary
void addEdgeToGraph(Graph *graph, size_t first, size_t second) {
  // Ensure the graph has enough vertices
  size_t graph_size = boost::num_vertices(*graph);
  for (size_t i = graph_size; i < std::max(first, second) + 1; ++i) {
    boost::add_vertex(*graph);
  }
  boost::add_edge(first, second, *graph);
}

Graph readGraph(const std::filesystem::path &path) {
  std::ifstream file(path);
  if (!file) {
    std::cerr << "Can't open file " << path << '.' << std::endl;
    throw std::runtime_error("");
  }

  Graph graph;

  std::string line;
  while (std::getline(file, line)) {
    // Example line: "0 4 {'weight': 0.002105263157894737}"
    std::istringstream iss(line);
    int first, second;
    if (!(iss >> first >> second)) {
      std::cerr << "Invalid line format: " << line << std::endl;
      continue; // Skip invalid lines
    }

    // Add edge to the graph (vertices are added automatically if they don't
    // exist)
    addEdgeToGraph(&graph, first, second);
  }

  file.close();

  return graph;
}

void writeGraph(Graph *graph, const std::filesystem::path &path) {
  std::ofstream file(path);
  if (!file) {
    std::cerr << "Can't open file " << path << '.' << std::endl;
    throw std::runtime_error("");
  }

  boost::dynamic_properties dp;
  dp.property("nearest_centroid",
              boost::get(&VertexInfo::nearest_centroid, *graph));
  dp.property("node_id", boost::get(boost::vertex_index, *graph));
  boost::write_graphviz_dp(file, *graph, dp);
}