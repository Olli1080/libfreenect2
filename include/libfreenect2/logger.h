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

/** @file logger.h Logging control for %libfreenect2.
 *
 * This header has no third-party dependency. The library formats each line
 * internally (with fmt) and hands the finished text to a sink; by default the
 * sink writes to stderr. Point it somewhere else with setLogSink(), or turn
 * logging off entirely with setLogLevel(LogLevel::None).
 *
 * If you would rather route output through an spdlog logger you own (rotating
 * files, syslog, a network sink, ...), include <libfreenect2/logger_spdlog.h>
 * instead -- that is the only libfreenect2 header that depends on spdlog, and
 * only code that includes it needs spdlog on its include path.
 */

#ifndef LIBFREENECT2_LOGGER_H_
#define LIBFREENECT2_LOGGER_H_

#include <functional>
#include <string_view>

#include <freenect2_logging/export.h>

namespace libfreenect2
{

/** @defgroup logging Logging utilities
 * Control where %libfreenect2's diagnostic output goes and how much of it.
 */
///@{

/** Severity levels, most to least severe. `None` disables all output. */
enum class LogLevel
{
  None = 0,
  Error,
  Warning,
  Info,
  Debug
};

/** The level the library starts at: `Info`, unless the environment variable
 * `LIBFREENECT2_LOGGER_LEVEL` overrides it (case-insensitive `none`/`off`,
 * `error`, `warning`/`warn`, `info`, `debug`). */
FREENECT2_LOGGING_EXPORT LogLevel getDefaultLevel();

/** The current threshold; lines below it are dropped. */
FREENECT2_LOGGING_EXPORT LogLevel getLogLevel();

/** Set the threshold. `LogLevel::None` silences the library completely
 * without touching your application's own logging. */
FREENECT2_LOGGING_EXPORT void setLogLevel(LogLevel level);

/** Receives one fully formatted log line (no trailing newline). Called from
 * whichever thread produced the message; an implementation must be
 * thread-safe if the library is used from more than one thread. */
using LogSink = std::function<void(LogLevel level, std::string_view message)>;

/** Install a sink for all subsequent log output. Passing `nullptr` restores
 * the built-in stderr sink. */
FREENECT2_LOGGING_EXPORT void setLogSink(LogSink sink);

///@}
} /* namespace libfreenect2 */
#endif /* LIBFREENECT2_LOGGER_H_ */
