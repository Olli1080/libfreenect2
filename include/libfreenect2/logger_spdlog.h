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

/** @file logger_spdlog.h Optional spdlog bridge for %libfreenect2 logging.
 *
 * Include this (instead of, or in addition to, <libfreenect2/logger.h>) to
 * route the library's output through an spdlog logger you control. This is
 * the only libfreenect2 header that depends on spdlog; a project that does
 * not include it does not need spdlog at all.
 *
 * The functions install a libfreenect2::LogSink that forwards to the given
 * spdlog logger, so setLogLevel() / setLogSink() from <logger.h> keep
 * working and interoperate with these.
 */

#ifndef LIBFREENECT2_LOGGER_SPDLOG_H_
#define LIBFREENECT2_LOGGER_SPDLOG_H_

#include <memory>

#include <spdlog/spdlog.h>

#include <freenect2_logging/export.h>
#include <libfreenect2/logger.h>

namespace libfreenect2
{

/** Route all subsequent library output into `logger` (its own sinks, level
 * and formatting apply). Passing nullptr restores the built-in stderr sink.
 * Equivalent to setLogSink() with a forwarding lambda. */
FREENECT2_LOGGING_EXPORT void setSpdlogLogger(std::shared_ptr<spdlog::logger> logger);

/** The spdlog logger last installed with setSpdlogLogger(), or nullptr if
 * the active sink is not spdlog-backed. */
FREENECT2_LOGGING_EXPORT std::shared_ptr<spdlog::logger> getSpdlogLogger();

/** A ready-made stderr spdlog logger at `level`, pattern `[%l] %v`,
 * flushing every line -- the same logger the library uses by default.
 * Install it with setSpdlogLogger(). */
FREENECT2_LOGGING_EXPORT std::shared_ptr<spdlog::logger>
    createConsoleLogger(spdlog::level::level_enum level);

/** Map a libfreenect2::LogLevel to the nearest spdlog level. */
FREENECT2_LOGGING_EXPORT spdlog::level::level_enum toSpdlogLevel(LogLevel level);

/** Map an spdlog level to the nearest libfreenect2::LogLevel. */
FREENECT2_LOGGING_EXPORT LogLevel fromSpdlogLevel(spdlog::level::level_enum level);

} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_LOGGER_SPDLOG_H_ */
