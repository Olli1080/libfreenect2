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

/** @file logger.h Logging utilities built on spdlog. */

#ifndef LIBFREENECT2_LOGGER_H_
#define LIBFREENECT2_LOGGER_H_

#include <memory>

#include <spdlog/spdlog.h>

#include <freenect2_logging/export.h>

namespace libfreenect2
{

/** @defgroup logging Logging utilities
 * %libfreenect2 logs through its own dedicated spdlog logger instance
 * (never the spdlog default logger). Install your own logger (console, file,
 * rotating, network sink, ...) with setGlobalLogger(), or adjust the level of
 * the library's own instance with getLogLevel()/setLogLevel(). */
///@{

/** Get the default log level.
 * Default is `spdlog::level::info`, or overridden by environment variable
 * `LIBFREENECT2_LOGGER_LEVEL`. `LIBFREENECT2_LOGGER_LEVEL` can contain a
 * case-insensitive level name: `none`, `error`, `warning`, `info`, `debug`.
 */
FREENECT2_LOGGING_EXPORT spdlog::level::level_enum getDefaultLevel();

/** Create a console (stderr) spdlog logger that logs at the given level. */
FREENECT2_LOGGING_EXPORT std::shared_ptr<spdlog::logger> createConsoleLogger(spdlog::level::level_enum level);

/** Create a console logger using the default level (see getDefaultLevel()). */
FREENECT2_LOGGING_EXPORT std::shared_ptr<spdlog::logger> createDefaultLogger();

/** Get the pointer to the current logger.
 * @return The current global logger. Never null; a default console logger
 * is always available.
 */
FREENECT2_LOGGING_EXPORT std::shared_ptr<spdlog::logger> getGlobalLogger();

/** Set the logger for all log output in this library.
 * @param logger Your logger, or null to restore the default console logger.
 */
FREENECT2_LOGGING_EXPORT void setGlobalLogger(std::shared_ptr<spdlog::logger> logger);

/** Get the log level of the library's own logger instance.
 * This never affects spdlog's default logger or any other application loggers.
 */
FREENECT2_LOGGING_EXPORT spdlog::level::level_enum getLogLevel();

/** Set the log level of the library's own logger instance.
 * Use `spdlog::level::off` to disable all logging from this library.
 * This never affects spdlog's default logger or any other application loggers.
 */
FREENECT2_LOGGING_EXPORT void setLogLevel(spdlog::level::level_enum level);

///@}
} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_LOGGER_H_ */
