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

/** @file logging_spdlog.cpp Optional spdlog bridge -- see logger_spdlog.h.
 *
 * Compiled into freenect2_logging so the symbols always exist, but nothing
 * in the library uses them: only a consumer that includes logger_spdlog.h
 * pulls spdlog in.
 */

#include <libfreenect2/logger_spdlog.h>

#include <mutex>

#include <spdlog/sinks/stdout_sinks.h>

namespace libfreenect2
{

namespace
{
std::mutex g_spdlog_mutex;
std::shared_ptr<spdlog::logger> g_spdlog_logger;
} // namespace

spdlog::level::level_enum toSpdlogLevel(LogLevel level)
{
  switch (level)
  {
    case LogLevel::None:    return spdlog::level::off;
    case LogLevel::Error:   return spdlog::level::err;
    case LogLevel::Warning: return spdlog::level::warn;
    case LogLevel::Info:    return spdlog::level::info;
    case LogLevel::Debug:   return spdlog::level::debug;
  }
  return spdlog::level::info;
}

LogLevel fromSpdlogLevel(spdlog::level::level_enum level)
{
  switch (level)
  {
    case spdlog::level::off:      return LogLevel::None;
    case spdlog::level::critical:
    case spdlog::level::err:      return LogLevel::Error;
    case spdlog::level::warn:     return LogLevel::Warning;
    case spdlog::level::info:     return LogLevel::Info;
    case spdlog::level::debug:
    case spdlog::level::trace:    return LogLevel::Debug;
    default:                      return LogLevel::Info;
  }
}

std::shared_ptr<spdlog::logger> createConsoleLogger(spdlog::level::level_enum level)
{
  // Construct directly rather than via spdlog::stderr_logger_mt(), which
  // registers by name in spdlog's global registry and throws on a second
  // call with the same name.
  auto sink = std::make_shared<spdlog::sinks::stderr_sink_mt>();
  auto logger = std::make_shared<spdlog::logger>("freenect2", sink);
  logger->set_level(level);
  logger->set_pattern("[%l] %v");
  logger->flush_on(spdlog::level::debug);
  return logger;
}

void setSpdlogLogger(std::shared_ptr<spdlog::logger> logger)
{
  {
    std::scoped_lock lock(g_spdlog_mutex);
    g_spdlog_logger = logger;
  }

  if (logger == nullptr)
  {
    setLogSink(nullptr);
    return;
  }

  setLogSink([logger](LogLevel level, std::string_view message) {
    logger->log(toSpdlogLevel(level), spdlog::string_view_t(message.data(), message.size()));
  });
}

std::shared_ptr<spdlog::logger> getSpdlogLogger()
{
  std::scoped_lock lock(g_spdlog_mutex);
  return g_spdlog_logger;
}

} /* namespace libfreenect2 */
