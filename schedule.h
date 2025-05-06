#pragma once

#include <cstddef>

class SamplingSchedule {
public:
  virtual size_t Get() const = 0;
  virtual void Next() = 0;
  virtual ~SamplingSchedule() = default;
};

class GeometricSamplingSchedule : public SamplingSchedule {
public:
  GeometricSamplingSchedule(size_t initial, long double coefficient)
      : current_(static_cast<long double>(initial)), coefficient_(coefficient) {
  }

  size_t Get() const override { return static_cast<size_t>(current_); }

  void Next() override { current_ *= coefficient_; }

  ~GeometricSamplingSchedule() = default;

private:
  long double current_;
  long double coefficient_;
};