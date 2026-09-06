/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
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

/** @file Protonect.cpp Main application file. */

#include <iostream>
#include <cstdlib>
#include <signal.h>

#include <argparse/argparse.hpp>

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/pipeline_factory.h>
#include <libfreenect2/logger.h>
/// [headers]
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#endif


bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

//Doing non-trivial things in signal handler is bad. If you want to pause,
//do it in another thread.
//Though libusb operations are generally thread safe, I cannot guarantee
//everything above is thread safe when calling start()/stop() while
//waitForNewFrame().
void sigusr1_handler(int s)
{
  if (devtopause == nullptr)
    return;
/// [pause]
  if (protonect_paused)
    (void)devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
/// [pause]
}

//The following demonstrates how to send log lines somewhere of your choosing
//via a LogSink -- here, appended to a file. No third-party logging library is
//involved; see logger_spdlog.h if you would rather hand off to spdlog.
/// [logger]
#include <cstdio>
#include <mutex>
libfreenect2::LogSink createFileSink(const char *filename)
{
  if (filename == nullptr)
    return nullptr;

  auto file = std::shared_ptr<std::FILE>(std::fopen(filename, "w"),
                                         [](std::FILE *f) { if (f) std::fclose(f); });
  if (!file)
    return nullptr;

  auto mtx = std::make_shared<std::mutex>();
  return [file, mtx](libfreenect2::LogLevel level, std::string_view message)
  {
    static const char *names[] = {"none", "error", "warning", "info", "debug"};
    std::scoped_lock lock(*mtx);
    std::fprintf(file.get(), "[%s] %.*s\n", names[static_cast<int>(level)],
                 static_cast<int>(message.size()), message.data());
    std::fflush(file.get());
  };
}
/// [logger]

/// [main]
/**
 * Main application entry point. Run with --help for the argument list
 * (parsed with p-ranav/argparse; the pipeline token and the device serial
 * are bare positionals, kept together, e.g. `Protonect gl <serial> -noviewer`).
 */
int main(int argc, char *argv[])
/// [main]
{
  std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;

  argparse::ArgumentParser program("Protonect", LIBFREENECT2_VERSION);
  program.add_description("Kinect v2 capture demo (color, IR/depth, registration).");
  program.add_epilog("Environment: LIBFREENECT2_LOGGER_LEVEL, LOGFILE.\n"
                     "Pause/unpause on POSIX: pkill -USR1 Protonect");
  program.add_argument("-gpu").metavar("<id>").scan<'i', int>().default_value(-1)
      .help("GPU device id for the cl/cuda/vk pipelines");
  program.add_argument("-frames").metavar("<N>").scan<'u', std::size_t>()
      .help("stop after N frames");
  program.add_argument("-noviewer").flag().help("disable the viewer window");
  program.add_argument("-norgb").flag().help("disable the color stream");
  program.add_argument("-nodepth").flag().help("disable the depth stream");
  program.add_argument("tokens").nargs(argparse::nargs_pattern::any).metavar("[pipeline] [serial]")
      .help("a pipeline token (cpu/gl/cl/clkde/cuda/cudakde/vk) and/or a device serial");

  try
  {
    program.parse_args(argc, argv);
  }
  catch (const std::exception &err)
  {
    std::cerr << err.what() << "\n\n" << program;
    return 1;
  }

/// [logging]
  // The library reads LIBFREENECT2_LOGGER_LEVEL on load; only override it here
  // if the user did not set it. On the (slow) Windows console the Info default
  // is already sensible; elsewhere raise to Debug for development.
#if !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32__)
  if (getenv("LIBFREENECT2_LOGGER_LEVEL") == nullptr)
    libfreenect2::setLogLevel(libfreenect2::LogLevel::Debug);
#endif
/// [logging]
/// [file logging]
  if (auto sink = createFileSink(getenv("LOGFILE")))
    libfreenect2::setLogSink(std::move(sink));
/// [file logging]

/// [context]
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = nullptr;
  libfreenect2::PacketPipeline *pipeline = nullptr;
/// [context]

  const int deviceId = program.get<int>("-gpu");
  const bool viewer_enabled = !program.get<bool>("-noviewer");
  const bool enable_rgb = !program.get<bool>("-norgb");
  const bool enable_depth = !program.get<bool>("-nodepth");

  size_t framemax = static_cast<size_t>(-1);
  if (auto f = program.present<std::size_t>("-frames"))
  {
    if (*f == 0)
    {
      std::cerr << "invalid frame count '0'" << std::endl;
      return -1;
    }
    framemax = *f;
  }

  // The pipeline token and device serial are both bare positionals; tell them
  // apart the way protonect.py does (all-digits => serial).
  std::string serial;
  std::string pipeline_name;
  if (auto tokens = program.present<std::vector<std::string>>("tokens"))
  {
    for (const auto &tok : *tokens)
    {
      if (tok.find_first_not_of("0123456789") == std::string::npos)
        serial = tok;
      else if (libfreenect2::hasPacketPipeline(tok))
        pipeline_name = tok;
      else
      {
        std::cerr << "unrecognized argument: " << tok << "\n\n" << program;
        return 1;
      }
    }
  }

  if (!enable_rgb && !enable_depth)
  {
    std::cerr << "Disabling both streams is not allowed!" << std::endl;
    return -1;
  }

/// [discovery]
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  if (serial == "")
  {
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
/// [discovery]

  if (!pipeline_name.empty())
  {
/// [pipeline]
    // Build a pipeline by name -- same tokens as LIBFREENECT2_PIPELINE.
    // -gpu=<id> is forwarded to the backends that use it (cl, cuda, vk).
    pipeline = libfreenect2::createPacketPipeline(pipeline_name, deviceId).release();
/// [pipeline]
  }

  if(pipeline)
  {
/// [open]
    dev = freenect2.openDevice(serial, pipeline);
/// [open]
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == nullptr)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  devtopause = dev;

  signal(SIGINT,sigint_handler);
#ifdef SIGUSR1
  signal(SIGUSR1, sigusr1_handler);
#endif
  protonect_shutdown = false;

/// [listeners]
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
/// [listeners]

/// [start]
  if (enable_rgb && enable_depth)
  {
    if (!dev->start())
      return -1;
  }
  else
  {
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
/// [start]

/// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
/// [registration setup]

  size_t framecount = 0;
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
  Viewer viewer;
  if (viewer_enabled)
    viewer.initialize();
#else
  viewer_enabled = false;
#endif

/// [loop start]
  while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
  {
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame *rgb = libfreenect2::getFrame(frames, libfreenect2::Frame::Color);
    libfreenect2::Frame *ir = libfreenect2::getFrame(frames, libfreenect2::Frame::Ir);
    libfreenect2::Frame *depth = libfreenect2::getFrame(frames, libfreenect2::Frame::Depth);
/// [loop start]

    if (enable_rgb && enable_depth)
    {
/// [registration]
      registration->apply(rgb, depth, &undistorted, &registered);
/// [registration]
    }

    framecount++;
    if (!viewer_enabled)
    {
      if (framecount % 100 == 0)
        std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
      listener.release(frames);
      continue;
    }

#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
    if (enable_rgb)
    {
      viewer.addFrame("RGB", rgb);
    }
    if (enable_depth)
    {
      viewer.addFrame("ir", ir);
      viewer.addFrame("depth", depth);
    }
    if (enable_rgb && enable_depth)
    {
      viewer.addFrame("registered", &registered);
    }

    protonect_shutdown = protonect_shutdown || viewer.render();
#endif

/// [loop end]
    listener.release(frames);
    /** libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100)); */
  }
/// [loop end]

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
/// [stop]
  dev->stop();
  dev->close();
/// [stop]

  delete registration;

  return 0;
}
