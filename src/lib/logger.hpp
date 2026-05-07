#pragma once

#include <chrono>
#include <ctime>
#include <format>
#include <iostream>
#include <string>
#include <utility>

namespace logger {

inline bool& EnabledFlag() {
  static bool enabled = true;
  return enabled;
}

inline void SetEnabled(bool enabled) { EnabledFlag() = enabled; }

// Captured on first call (effectively program start for a CLI tool). steady_clock
// is monotonic, so the offset is unaffected by wall-clock adjustments.
inline std::chrono::steady_clock::time_point StartTime() {
  static const auto t = std::chrono::steady_clock::now();
  return t;
}

inline std::string Timestamp() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  const auto t = system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  return std::format("{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:03}", tm.tm_year + 1900, tm.tm_mon + 1,
                     tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms.count()));
}

inline std::string ElapsedTimestamp() {
  using namespace std::chrono;
  const auto elapsed = duration<double>(steady_clock::now() - StartTime()).count();
  return std::format("+{:.3f}s", elapsed);
}

template <typename... Args>
void Log(std::format_string<Args...> fmt, Args&&... args) {
  if (!EnabledFlag()) return;
  std::cout << '[' << Timestamp() << "] [" << ElapsedTimestamp() << "] "
            << std::format(fmt, std::forward<Args>(args)...) << std::endl;
}

}  // namespace logger
