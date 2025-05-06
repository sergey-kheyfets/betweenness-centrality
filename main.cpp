#include <cassert>
#include <iostream>
#include <string>

#include "components.h"
#include "diameter.h"

#include "io.h"

#include "paths.h"
#include "utils.h"

#include "cli.h"

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <dataset_filename> <command> <...parameters>" << std::endl;
    std::cerr << "Commands: exact-bc, shortest-paths, abra-bc" << std::endl;
    return 1;
  }

  std::filesystem::path path{argv[1]};
  auto components = ReadGraphIntoComponents(path);

  auto command = std::string{argv[2]};

  if (command == "exact-bc") {
    if (components.size() != 1) {
      std::cerr << "Should have one component";
      return 1;
    }
    PrintExactCentrality(components[0]);
    return 0;
  }

  if (command == "abra-bc") {
    if (components.size() != 1) {
      std::cerr << "Should have one component";
      return 1;
    }

    long double desired_error = std::stold(argv[3]);
    long double desired_condifence = std::stold(argv[4]);
    size_t rademacher_sample_size = std::stoi(argv[5]);
    size_t max_iterations = std::stoi(argv[6]);

    PrintCentralityAbra(components[0], desired_error, desired_condifence,
                        rademacher_sample_size, max_iterations);
    return 0;
  }

  if (command == "shortest-paths") {
    size_t num_samples = 10;
    if (argc == 4) {
      num_samples = std::stoi(argv[3]);
    }

    for (auto &component : components) {
      TestPathEstimation(component, num_samples);
    }
  }
}
