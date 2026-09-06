/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file logging.cpp Sink-based logging implementation (no spdlog). */

#include <libfreenect2/logging.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

#ifdef LIBFREENECT2_WITH_PROFILING
#include <vector>
#include <numeric>
#include <cmath>
#endif

#include <chrono>

#include <fmt/format.h>

namespace libfreenect2
{

namespace
{
const char *levelName(LogLevel level)
{
  switch (level)
  {
    case LogLevel::Error:   return "error";
    case LogLevel::Warning: return "warning";
    case LogLevel::Info:    return "info";
    case LogLevel::Debug:   return "debug";
    default:                return "none";
  }
}

void defaultSink(LogLevel level, std::string_view message)
{
  // Matches the historical spdlog pattern "[%l] %v", line-buffered on stderr.
  std::fprintf(stderr, "[%s] %.*s\n", levelName(level),
               static_cast<int>(message.size()), message.data());
  std::fflush(stderr);
}

std::atomic<LogLevel> g_level{LogLevel::Info};

std::mutex g_sink_mutex;
LogSink g_sink;   // empty => use defaultSink
} // namespace

LogLevel getDefaultLevel()
{
  LogLevel level = LogLevel::Info;

  const char *env = std::getenv("LIBFREENECT2_LOGGER_LEVEL");
  if (env != nullptr)
  {
    std::string s(env);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(::tolower(c)); });

    if (s == "debug")
      level = LogLevel::Debug;
    else if (s == "info")
      level = LogLevel::Info;
    else if (s == "warning" || s == "warn")
      level = LogLevel::Warning;
    else if (s == "error")
      level = LogLevel::Error;
    else if (s == "none" || s == "off")
      level = LogLevel::None;
  }

  return level;
}

LogLevel getLogLevel()
{
  return g_level.load(std::memory_order_relaxed);
}

void setLogLevel(LogLevel level)
{
  g_level.store(level, std::memory_order_relaxed);
}

void setLogSink(LogSink sink)
{
  std::scoped_lock lock(g_sink_mutex);
  g_sink = std::move(sink);
}

namespace detail
{
bool shouldLog(LogLevel level)
{
  const LogLevel current = g_level.load(std::memory_order_relaxed);
  return current != LogLevel::None && level != LogLevel::None &&
         static_cast<int>(level) <= static_cast<int>(current);
}

void emitLog(LogLevel level, std::string_view message)
{
  LogSink sink;
  {
    std::scoped_lock lock(g_sink_mutex);
    sink = g_sink;
  }
  if (sink)
    sink(level, message);
  else
    defaultSink(level, message);
}
} // namespace detail

// Initialise the threshold from the environment at load time.
namespace
{
const bool g_level_initialised = [] {
  g_level.store(getDefaultLevel(), std::memory_order_relaxed);
  return true;
}();
} // namespace

/** Timer for measuring performance. */
class Timer
{
 public:
  double duration;
  size_t count;

  Timer()
  {
    reset();
  }

  void reset()
  {
    duration = 0;
    count = 0;
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> time_start;

  void start()
  {
    time_start = std::chrono::high_resolution_clock::now();
  }

  double stop()
  {
    auto time_diff = std::chrono::high_resolution_clock::now() - time_start;
    double this_duration = std::chrono::duration_cast<std::chrono::duration<double>>(time_diff).count();
    duration += this_duration;
    count++;
    return this_duration;
  }
};

class WithPerfLoggingImpl: public Timer
{
public:
#ifdef LIBFREENECT2_WITH_PROFILING
  std::vector<double> stats;
  std::string name;

  WithPerfLoggingImpl()
  {
    stats.reserve(30*100);
  }

  ~WithPerfLoggingImpl()
  {
    if (stats.size() < 2)
      return;
    std::vector<double> &v = stats;
    std::sort(v.begin(), v.end());
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    size_t n = v.size();
    double mean = sum / n;
    std::vector<double> diff(n);
    for (size_t i = 0; i < n; ++i)
    {
      diff[i] = v[i] - mean;
    }
    double sqsum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stddev = std::sqrt(sqsum / (n-1));

    if (detail::shouldLog(LogLevel::Info))
      detail::emitLog(LogLevel::Info,
        fmt::format("[{}] {:.6g} {:.6g} {:.6g} {:.6g} {:.6g} mean={:.6g} std={:.6g} n={}",
                    name, v[0], v[n/20], v[n/2], v[n - (n+19)/20], v[n-1], mean, stddev, n));
  }
#else
  WithPerfLoggingImpl() = default;
#endif

  void start()
  {
    Timer::start();
  }

  std::string stop(const char *source)
  {
#ifdef LIBFREENECT2_WITH_PROFILING
    double this_duration = Timer::stop();
    if(name.empty())
      name = getShortName(source);
    stats.push_back(this_duration*1e3);
#else
    Timer::stop();
#endif
    if(count < 100)
      return std::string();
    double avg = duration / count;
    reset();
    return fmt::format("avg. time: {:.6g}ms -> ~{:.6g}Hz", avg * 1000, 1.0/avg);
  }
};

WithPerfLogging::WithPerfLogging() : impl_(std::make_unique<WithPerfLoggingImpl>())
{
}

WithPerfLogging::~WithPerfLogging() = default;

void WithPerfLogging::startTiming()
{
  impl_->start();
}

std::string WithPerfLogging::stopTiming(const char *source)
{
  return impl_->stop(source);
}

} /* namespace libfreenect2 */
