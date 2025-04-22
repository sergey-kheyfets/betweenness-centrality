#include <cassert>
#include <iostream>

#include "centroids.h"
#include "cli.h"

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <dataset_filename> <command> <...parameters>" << std::endl;
    std::cerr << "Commands: exact-bc, shortest-paths, estimate-bc" << std::endl;
    return 1;
  }

  std::filesystem::path path{argv[1]};
  auto components = readGraphIntoComponents(path);

  auto command = std::string{argv[2]};

  if (command == "exact-bc") {
    if (components.size() != 1) {
      std::cerr << "Should have one component";
      return 1;
    }
    printExactCentrality(components[0]);
    return 0;
  }

  if (command == "shortest-paths") {
    size_t num_samples = 10;
    if (argc == 4) {
      num_samples = std::stoi(argv[3]);
    }

    for (auto &component : components) {
      testPathEstimation(component, num_samples);
    }
    return 0;
  }

  if (command == "estimate-bc") {
    if (components.size() != 1) {
      std::cerr << "Should have one component";
      return 1;
    }

    std::vector<Vertex<Graph>> centroids;
    if (argc == 4) {
      centroids = getAndApplyCentroids(components[0],
                                       static_cast<size_t>(std::stoi(argv[3])));
    } else {
      centroids = getAndApplyCentroids(components[0]);
    }

    printEstimatedCentrality(components[0], centroids);
    return 0;
  }
}
