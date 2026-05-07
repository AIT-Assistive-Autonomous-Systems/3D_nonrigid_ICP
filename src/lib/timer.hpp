#pragma once

#include <chrono>
#include <format>
#include <ostream>
#include <string>

class Timer {
 public:
  Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_time_ = std::chrono::high_resolution_clock::now(); }

  double elapsed() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);
    return duration.count();
  }

  friend std::ostream& operator<<(std::ostream& os, const Timer& timer) {
    os << std::format("{:.3f}s", timer.elapsed());
    return os;
  }

  std::string str() const { return std::format("{:.3f}s", elapsed()); }

 private:
  std::chrono::high_resolution_clock::time_point start_time_;
};

template <>
struct std::formatter<Timer> : std::formatter<std::string> {
  auto format(const Timer& timer, std::format_context& ctx) const {
    return std::formatter<std::string>::format(timer.str(), ctx);
  }
};
