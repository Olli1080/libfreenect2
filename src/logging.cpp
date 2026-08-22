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

/** @file logging.cpp spdlog-backed logging implementation. */

#include <libfreenect2/logging.h>
#include <cstdlib>
#include <cctype>
#include <string>
#include <algorithm>
#include <memory>

#ifdef LIBFREENECT2_WITH_PROFILING
#include <vector>
#include <numeric>
#include <cmath>
#endif

#include <chrono>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>

namespace libfreenect2
{

spdlog::level::level_enum getDefaultLevel()
{
  spdlog::level::level_enum level = spdlog::level::info;

  char *env_logger_level_c_str = getenv("LIBFREENECT2_LOGGER_LEVEL");

  if(env_logger_level_c_str != nullptr)
  {
    std::string env_logger_level_str(env_logger_level_c_str);
    std::transform(env_logger_level_str.begin(), env_logger_level_str.end(), env_logger_level_str.begin(),
                   [](unsigned char c) { return static_cast<char>(::tolower(c)); });

    if(env_logger_level_str == "debug")
      level = spdlog::level::debug;
    else if(env_logger_level_str == "info")
      level = spdlog::level::info;
    else if(env_logger_level_str == "warning" || env_logger_level_str == "warn")
      level = spdlog::level::warn;
    else if(env_logger_level_str == "error")
      level = spdlog::level::err;
    else if(env_logger_level_str == "none" || env_logger_level_str == "off")
      level = spdlog::level::off;
  }

  return level;
}

std::shared_ptr<spdlog::logger> createConsoleLogger(spdlog::level::level_enum level)
{
  // Construct directly rather than via spdlog::stderr_logger_mt(), which
  // registers the logger by name in spdlog's global registry and throws if
  // called twice with the same name (it is: once for the default instance
  // at library load, again whenever a caller does setGlobalLogger(createConsoleLogger(...))).
  auto sink = std::make_shared<spdlog::sinks::stderr_sink_mt>();
  auto logger = std::make_shared<spdlog::logger>("freenect2", sink);
  logger->set_level(level);
  logger->set_pattern("[%l] %v");
  // The legacy ConsoleLogger flushed after every line; keep that behavior.
  logger->flush_on(spdlog::level::debug);
  return logger;
}

std::shared_ptr<spdlog::logger> createDefaultLogger()
{
  return createConsoleLogger(getDefaultLevel());
}

static std::shared_ptr<spdlog::logger> global_logger_ = createDefaultLogger();

std::shared_ptr<spdlog::logger> getGlobalLogger()
{
  return std::atomic_load(&global_logger_);
}

void setGlobalLogger(std::shared_ptr<spdlog::logger> logger)
{
  if(logger == nullptr)
    logger = createDefaultLogger();
  std::atomic_store(&global_logger_, std::move(logger));
}

spdlog::level::level_enum getLogLevel()
{
  return getGlobalLogger()->level();
}

void setLogLevel(spdlog::level::level_enum level)
{
  getGlobalLogger()->set_level(level);
}

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

    auto logger = getGlobalLogger();
    logger->info("[{}] {:.6g} {:.6g} {:.6g} {:.6g} {:.6g} mean={:.6g} std={:.6g} n={}", name, v[0], v[n/20], v[n/2], v[n - (n+19)/20], v[n-1], mean, stddev, n);
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
