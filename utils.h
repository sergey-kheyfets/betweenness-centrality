#pragma once

#include "paths.h"
#include "traits.h"
#include "centroids.h"

#include <vector>
#include <iostream>
#include <numeric>
#include <random>

template <class T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& data) {
  for (const auto& value : data) {
    stream << value << ' ';
  }
  return stream;
}

template <class K, class V>
std::ostream& operator<<(std::ostream& stream, const std::map<K, V>& data) {
  for (const auto& pair : data) {
    stream << '{' << pair.first << ": " << pair.second << "} ";
  }
  return stream;
}

template <class K, class V>
std::ostream& operator<<(std::ostream& stream, const std::unordered_map<K, V>& data) {
  for (const auto& pair : data) {
    stream << '{' << pair.first << ": " << pair.second << "} ";
  }
  return stream;
}