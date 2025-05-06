cmake_minimum_required(VERSION 3.30)

project(BetweennessCentrality VERSION 0.1 LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 23)

# Find the Boost library, specifically the Graph component
find_package(Boost 1.86 REQUIRED COMPONENTS graph)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost library not found. Please install Boost and try again.")
endif()

# Include Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Add the executable and specify its source files
add_executable(betweenness-centrality main.cpp io.cpp cli.cpp)

# Link the Boost Graph library to your executable
target_link_libraries(betweenness-centrality PRIVATE Boost::graph)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(
    CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -Werror -fsanitize=undefined -fsanitize=address"
  )
  target_link_options(betweenness-centrality
    BEFORE PUBLIC -fsanitize=undefined PUBLIC -fsanitize=address
  )
endif()
