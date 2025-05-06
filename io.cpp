#include "io.h"

size_t AddVertexIfNotExist(Graph &graph,
                           std::unordered_map<size_t, size_t> &compress,
                           size_t vertex) {
  if (compress.find(vertex) == compress.end()) {
    compress[vertex] = compress.size();
    auto vertex_iterator = boost::add_vertex(graph);
    graph[vertex_iterator].id = vertex;
    return compress.size() - 1;
  }
  return compress[vertex];
}

// Function to add an edge between two nodes, adding vertices if necessary
void AddEdgeToGraph(Graph &graph, std::unordered_map<size_t, size_t> &compress,
                    size_t first, size_t second) {
  // Add vertices if needed
  first = AddVertexIfNotExist(graph, compress, first);
  second = AddVertexIfNotExist(graph, compress, second);
  boost::add_edge(first, second, graph);
}

Graph ReadGraph(const std::filesystem::path &path) {
  std::ifstream file(path);
  if (!file) {
    std::cerr << "Can't open file " << path << '.' << std::endl;
    throw std::runtime_error("");
  }

  Graph graph;

  std::unordered_map<size_t, size_t> compress;

  std::string line;
  while (std::getline(file, line)) {
    // Comments: "# ..."
    if (line[0] == '#') {
      std::cout << line << std::endl;
      continue;
    }
    // Example line: "0 4 {'weight': 0.002105263157894737}"
    std::istringstream iss(line);
    size_t first, second;
    if (!(iss >> first >> second)) {
      std::cerr << "Invalid line format: " << line << std::endl;
      continue; // Skip invalid lines
    }

    // Add edge to the graph  (vertices are added automatically if they don't
    // exist)
    AddEdgeToGraph(graph, compress, first, second);
  }

  file.close();

  return graph;
}

void WriteGraph(Graph *graph, const std::filesystem::path &path) {
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