#include <cstdint>
#include <iostream>
#include <random>

#include "centroids.h"
#include "components.h"
#include "diameter.h"
#include "io.h"
#include "paths.h"
#include "utils.h"

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <dataset_filename> <num_samples>"
              << std::endl;
    return 1;
  }

  size_t num_samples = 10;
  if (argc == 3) {
    num_samples = std::stoi(argv[2]);
  }

  std::cout << "Reading graph..." << std::endl;
  std::vector<Graph> components;
  try {
    components = splitIntoComponents(readGraph({argv[1]}));
  } catch (...) {
    return 1;
  }

  // auto components = getComponentsSubgraphs(graph);
  std::cout << "Number of components: " << components.size() << "\n\n";

  for (auto &component : components) {
    printBasicInfo(component);
    testPathEstimation(component, num_samples);
    std::cout << "\n\n";
  }

  // std::filesystem::path output_path = argv[1];
  // output_path = output_path.parent_path() / "result.dot";
  // writeGraph(&graph, {output_path});
}
