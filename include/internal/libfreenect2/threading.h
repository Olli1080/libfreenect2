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

/** @file threading.h Threading abstraction definitions. */

#ifndef THREADING_H_
#define THREADING_H_

#include <thread>
#include <mutex>
#include <condition_variable>

#if defined(__linux__)
  #include <sys/prctl.h>
#elif defined(__APPLE__)
  #include <pthread.h>
#elif defined(_WIN32)
  #include <windows.h>
  #include <processthreadsapi.h>
#endif

namespace std
{
namespace this_thread
{
  inline void set_name(const char *name)
  {
#if defined(__linux__)
    // Linux prctl: max 15 printable chars + null terminator
    prctl(PR_SET_NAME, (unsigned long)name, 0, 0, 0);
#elif defined(__APPLE__)
    // macOS pthread_setname_np: max 63 chars, targets calling thread
    pthread_setname_np(name);
#elif defined(_WIN32)
    // Windows 10 (1607+): expects wchar_t string
    std::size_t size = std::strlen(name) + 1;
    wchar_t* wname = new wchar_t[size];
    std::mbstowcs(wname, name, size);
    SetThreadDescription(GetCurrentThread(), wname);
    delete[] wname;
#endif
  }
}
}

#endif /* THREADING_H_ */
