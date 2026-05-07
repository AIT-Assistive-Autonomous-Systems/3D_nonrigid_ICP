#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "src/lib/logger.hpp"

class Profiler {
 public:
  static Profiler& Instance() {
    static Profiler instance{};
    return instance;
  }

  void Start(const std::string& section, uint32_t group_idx = 0) {
    std::lock_guard<std::mutex> lock{mutex_};
    start_times_[section] = {Clock::now(), group_idx};
  }

  void Stop(const std::string& section) {
    auto end_time{Clock::now()};

    std::lock_guard<std::mutex> lock{mutex_};
    auto it{start_times_.find(section)};
    if (it != start_times_.end()) {
      double duration{
          std::chrono::duration<double, std::milli>(end_time - it->second.time_point).count()};
      timing_data_[section].push_back({duration, it->second.group_idx});
      start_times_.erase(it);
    }
  }

  void PrintSummary() const {
    std::lock_guard<std::mutex> lock{mutex_};

    std::vector<std::string> sections;
    sections.reserve(timing_data_.size());
    for (const auto& [section, _] : timing_data_) {
      sections.push_back(section);
    }
    std::sort(sections.begin(), sections.end());

    const std::string separator(125, '-');
    logger::Log("{}", separator);
    logger::Log("{:<60} | {:>10} | {:>10} | {:>10} | {:>10} | {:>10}", "Timings per section",
                "Count", "Min", "Max", "Mean", "StdDev");
    logger::Log("{:<60} | {:>10} | {:>10} | {:>10} | {:>10} | {:>10}", "", "[#]", "[ms]", "[ms]",
                "[ms]", "[ms]");
    logger::Log("{}", separator);

    for (const auto& section : sections) {
      const auto& times = timing_data_.at(section);
      if (times.empty()) continue;

      std::vector<double> durations;
      durations.reserve(times.size());
      for (const auto& entry : times) {
        durations.push_back(entry.duration_ms);
      }

      double sum{std::accumulate(durations.begin(), durations.end(), 0.0)};
      double mean{sum / durations.size()};
      double min_time{*std::min_element(durations.begin(), durations.end())};
      double max_time{*std::max_element(durations.begin(), durations.end())};

      double variance{
          std::accumulate(durations.begin(), durations.end(), 0.0,
                          [mean](double acc, double t) { return acc + (t - mean) * (t - mean); }) /
          durations.size()};
      double stddev{std::sqrt(variance)};

      logger::Log("{:<60} | {:>10} | {:>10.1f} | {:>10.1f} | {:>10.1f} | {:>10.1f}", section,
                  durations.size(), min_time, max_time, mean, stddev);
    }
    logger::Log("{}", separator);
  }

  void WriteCSV(const std::filesystem::path& filepath) const {
    std::lock_guard<std::mutex> lock{mutex_};
    std::ofstream ofs{filepath};
    if (!ofs) {
      throw std::runtime_error{"Failed to open file for writing: " + filepath.string()};
    }
    ofs << "section,index,group_index,duration_ms\n";
    std::vector<std::string> sections;
    sections.reserve(timing_data_.size());
    for (const auto& [section, _] : timing_data_) {
      sections.push_back(section);
    }
    std::sort(sections.begin(), sections.end());

    for (const auto& section : sections) {
      const auto& times = timing_data_.at(section);
      for (size_t i{0}; i < times.size(); ++i) {
        ofs << section << "," << i << "," << times[i].group_idx << "," << times[i].duration_ms
            << "\n";
      }
    }
  }

  void Reset() {
    std::lock_guard<std::mutex> lock{mutex_};
    timing_data_.clear();
    start_times_.clear();
  }

 private:
  Profiler() = default;
  using Clock = std::chrono::high_resolution_clock;

  struct TimingEntry {
    double duration_ms;
    uint32_t group_idx;
  };

  struct StartEntry {
    Clock::time_point time_point;
    uint32_t group_idx;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::vector<TimingEntry>> timing_data_;
  std::unordered_map<std::string, StartEntry> start_times_;
};
