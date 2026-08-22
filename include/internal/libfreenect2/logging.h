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

/** @file logging.h Internal spdlog-native logging macros. */

#ifndef LOGGING_H_
#define LOGGING_H_

#include <iterator>
#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <libfreenect2/config.h>
#include <libfreenect2/logger.h>
#include <internal/libfreenect2/perf_logging.h>

namespace libfreenect2
{

inline std::string getShortName(const char *func)
{
  std::string src(func);
  size_t end = src.rfind('(');
  if (end == std::string::npos)
    end = src.size();
  size_t begin = 1 + src.rfind(' ', end);
  size_t first_ns = src.find("::", begin);
  if (first_ns != std::string::npos)
    begin = first_ns + 2;
  size_t last_ns = src.rfind("::", end);
  if (last_ns != std::string::npos)
    end = last_ns;
  return src.substr(begin, end - begin);
}

namespace detail
{
// Level-gated single fmt pass into one buffer. The finished message reaches
// spdlog as a raw string view, so user arguments are never re-parsed.
template <typename... Args>
void log(spdlog::level::level_enum level, const char *source, fmt::format_string<Args...> format, Args &&...args)
{
  auto logger = getGlobalLogger();
  if (logger != nullptr && logger->should_log(level))
  {
    spdlog::memory_buf_t buf;
    fmt::format_to(std::back_inserter(buf), "[{}] ", getShortName(source));
    fmt::format_to(std::back_inserter(buf), format, std::forward<Args>(args)...);
    logger->log(level, spdlog::string_view_t(buf.data(), buf.size()));
  }
}
} // namespace detail

} /* namespace libfreenect2 */

#if defined(__GNUC__) || defined(__clang__)
#define LOG_SOURCE __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#define LOG_SOURCE __FUNCSIG__
#else
#define LOG_SOURCE ""
#endif

#define LOG_DEBUG(...)   ::libfreenect2::detail::log(spdlog::level::debug, LOG_SOURCE, __VA_ARGS__)
#define LOG_INFO(...)    ::libfreenect2::detail::log(spdlog::level::info, LOG_SOURCE, __VA_ARGS__)
#define LOG_WARNING(...) ::libfreenect2::detail::log(spdlog::level::warn, LOG_SOURCE, __VA_ARGS__)
#define LOG_ERROR(...)   ::libfreenect2::detail::log(spdlog::level::err, LOG_SOURCE, __VA_ARGS__)

#endif /* LOGGING_H_ */
