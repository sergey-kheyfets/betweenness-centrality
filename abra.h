#pragma once

#include <boost/graph/detail/adjacency_list.hpp>

#include <boost/graph/filtered_graph.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <unordered_map>

#include "paths.h"
#include "schedule.h"
#include "traits.h"

template <class Graph> class AbraEstimator {
public:
  AbraEstimator(const Graph &graph,
                std::unique_ptr<SamplingSchedule> sample_schedule,
                long double desired_error, long double desired_confidence,
                size_t rademacher_sample_size, size_t max_iterations)
      : graph_(graph), result_(boost::num_vertices(graph_), 0.0),
        generator_{static_cast<std::mt19937>(std::random_device{}())},
        vertex_distribution_(0, boost::num_vertices(graph) - 1),
        rademacher_distribution_(-1, 1),
        sums_(std::vector<std::vector<long double>>(
            boost::num_vertices(graph),
            std::vector<long double>(rademacher_sample_size + 2, 0))),
        sample_schedule_(std::move(sample_schedule)) {
    desired_error_ = desired_error;
    desired_confidence_ = desired_confidence;
    rademacher_sample_size_ = rademacher_sample_size;
    max_iterations_ = max_iterations;
  }

  std::vector<long double> Estimate() {
    if (is_completed_) {
      return result_;
    }

    long double actual_error = std::numeric_limits<long double>::infinity();
    for (size_t i = 0; i != max_iterations_ && actual_error > desired_error_;
         ++i) {
      auto old_size = sample_size_;
      sample_schedule_->Next();
      sample_size_ = sample_schedule_->Get();

      RunIteration(sample_size_ - old_size);
      actual_error = GetError();
      std::cout << i << ") current error: " << actual_error << "\n";
    }

    FillResult();
    is_completed_ = true;

    return result_;
  }

private:
  const Graph &graph_;
  bool is_completed_ = false;

  std::mt19937 generator_;
  std::uniform_int_distribution<size_t> vertex_distribution_;
  std::uniform_int_distribution<short> rademacher_distribution_;

  std::unique_ptr<SamplingSchedule> sample_schedule_;
  long double desired_error_;
  long double desired_confidence_;
  size_t rademacher_sample_size_;
  size_t max_iterations_;

  size_t sample_size_ = 0;
  std::vector<std::vector<long double>> sums_;
  std::vector<long double> result_;

  void FillResult() {
    for (size_t i = 0; i != boost::num_vertices(graph_); ++i) {
      result_[i] = sums_[i][rademacher_sample_size_ + 1] / sample_size_;
    }
  }

  std::vector<long double> GetFunctionValues(size_t first, size_t second) {
    auto [counts_from_first, distances_from_first] =
        GetShortestPathCountsAndDistancesWithFinish(graph_, first, second);
    auto [counts_from_second, distances_from_second] =
        GetShortestPathCountsAndDistancesWithFinish(graph_, second, first);

    auto distance = distances_from_first[second];
    auto count = counts_from_first[second];

    std::vector<long double> result(boost::num_vertices(graph_), 0);
    for (size_t vertex = 0; vertex != result.size(); ++vertex) {
      if (vertex == first || vertex == second) {
        continue;
      }
      if (distances_from_first.find(vertex) != distances_from_first.end() &&
          distances_from_second.find(vertex) != distances_from_second.end() &&
          distances_from_first[vertex] + distances_from_second[vertex] ==
              distance) {
        result[vertex] = static_cast<long double>(counts_from_first[vertex] *
                                                  counts_from_second[vertex]) /
                         count;
      }
    }

    return result;
  }

  std::pair<size_t, size_t> GetSample() {
    size_t first = vertex_distribution_(generator_);
    size_t second;
    do {
      second = vertex_distribution_(generator_);
    } while (second == first);

    return {first, second};
  }

  std::vector<long double> GetRademacherSample() {
    std::vector<long double> result(rademacher_sample_size_);
    for (size_t i = 0; i != rademacher_sample_size_; ++i) {
      result[i] = rademacher_distribution_(generator_);
    }

    return result;
  }

  long double GetKmcera() {
    long double k_mcera = 0.0;
    for (size_t i = 0; i != rademacher_sample_size_; ++i) {
      long double supremum = -std::numeric_limits<long double>::infinity();
      for (size_t j = 0; j != sums_.size(); ++j) {
        supremum = std::max(supremum, sums_[j][i]);
      }
      supremum /= sample_size_;
      k_mcera += supremum;
    }
    k_mcera /= rademacher_sample_size_;
    return k_mcera;
  }

  long double GetBeta() {
    long double supremum = -std::numeric_limits<long double>::infinity();
    for (size_t j = 0; j != sums_.size(); ++j) {
      supremum = std::max(supremum, sums_[j][rademacher_sample_size_]);
    }
    supremum /= sample_size_;
    return supremum;
  }

  long double GetError() {
    auto beta = GetBeta();

    auto log_div_size =
        std::logl(5.0 / (1 - desired_confidence_)) / sample_size_;
    auto gamma = beta + log_div_size * (2.0 / 3.0);
    gamma +=
        std::sqrtl(log_div_size * log_div_size / 3.0 + 2 * beta * log_div_size);

    auto k_mcera = GetKmcera();
    auto rho = k_mcera + 2.0 * log_div_size / (3.0 * rademacher_sample_size_);
    rho += 2 * std::sqrtl(beta * log_div_size / rademacher_sample_size_);

    auto r =
        rho + log_div_size / 3.0 +
        std::sqrtl(log_div_size * log_div_size / 12.0 + rho * log_div_size);

    auto epsilon = 2 * r + log_div_size / 3.0 +
                   std::sqrtl(2.0 * (gamma + 4.0 * r) * log_div_size);
    return epsilon;
  }

  void RunIteration(size_t count) {
    for (size_t i = 0; i != count; ++i) {
      auto [from, to] = GetSample();
      auto values = GetFunctionValues(from, to);
      auto rademacher = GetRademacherSample();
      for (size_t vertex = 0; vertex != boost::num_vertices(graph_); ++vertex) {
        auto value = values[vertex];
        for (size_t j = 0; j != rademacher_sample_size_; ++j) {
          sums_[vertex][j] += value * rademacher[j];
        }
        sums_[vertex][rademacher_sample_size_] += value * value;
        sums_[vertex][rademacher_sample_size_ + 1] += value;
      }
    }
  }
};