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

/** @file MultiKinect.cpp Open every connected Kinect v2 at once, report
 * per-device throughput, and optionally show one window with an
 * RGB + colourised-depth panel per device.
 *
 * libfreenect2's core already supports any number of concurrently open
 * devices: Freenect2 keeps an internal registry, and each open
 * Freenect2Device gets its own USB transfer pool, its own packet-processing
 * threads and its own PacketPipeline. This example is the missing
 * demonstration of that (doc/mainpage.dox lists "Example of multiple
 * Kinects" under future work): it enumerates the bus, opens each device with
 * its own fresh pipeline, runs one capture thread per device, prints a live
 * per-serial FPS table, and with -viewer draws all devices side by side.
 *
 * Things that matter when driving more than one Kinect from one process:
 *
 *  - Each openDevice() needs its OWN freshly constructed PacketPipeline.
 *    libfreenect2 takes ownership and frees it (even on failure); pipelines
 *    are never shared between devices. This example calls makePipeline()
 *    once per device inside the open loop.
 *
 *  - One SyncMultiFrameListener PER device. A listener's FrameMap is keyed
 *    by Frame::Type, so a shared listener would let two devices' Color
 *    frames collide. At the default capacity 1 a consumer slower than the
 *    sensor drops intermediate frames; -capacity <N> (N >= 2) buffers N sets
 *    of copies so a bursty consumer keeps up. This example still releases
 *    each FrameMap promptly and copies what -viewer needs on the capture
 *    thread.
 *
 *  - GPU pipelines (cl/cuda/vk) take a device id -- pass -gpu=<id> to spread
 *    several Kinects across several GPUs. This example applies one id to all
 *    devices; edit the makePipeline() call site for per-device placement.
 *
 *  - The Kinect's one-time device-wide USB reset on first open is
 *    coordinated once per process (see
 *    include/internal/libfreenect2/device_reset_coordination.h) so core and
 *    freenect2_audio don't reset each other's session. It is NOT coordinated
 *    across processes.
 *
 *  - On Windows with the UsbDk backend only ONE open session per physical
 *    device is allowed at all (libfreenect2.hpp, getUsbDeviceHandle() note).
 *    Multiple *different* devices in one process are fine; a second opener of
 *    the *same* device is not.
 *
 *  - USB 3 bandwidth is the real ceiling. Three-plus Kinects streaming at
 *    once need multiple separate PCIe USB3 controllers (see README,
 *    "Requirements for multiple Kinects"); on a single shared host
 *    controller the per-device frame rate in the table will sag once the bus
 *    saturates.
 *
 * Run with --help for the argument list (parsed with p-ranav/argparse). The
 * pipeline token and the device serials are bare positionals, kept together;
 * no pipeline arg uses libfreenect2's compile-time default, no serials opens
 * every device found. -viewer needs an OpenCV build; prefer a non-gl pipeline
 * with it (the gl depth pipeline opens its own GL window per device on top of
 * the viewer). -capacity <N> is the per-device listener buffer depth.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <argparse/argparse.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/pipeline_factory.h>
#include <libfreenect2/logger.h>

#ifdef MULTIKINECT_WITH_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace
{

constexpr int kPanelW = 512;   ///< per-device column width, px
constexpr int kRgbH = 288;     ///< 16:9 RGB strip height (depth strip is the native 424)

std::atomic<bool> g_shutdown{false};

void sigint_handler(int)
{
  g_shutdown = true;
}

/// Build a fresh pipeline for one device. `name` empty -> nullptr (let
/// libfreenect2 pick its compile-time default). Names are the shared
/// libfreenect2 pipeline tokens (cpu, gl, cl, clkde, cuda, cudakde, vk).
libfreenect2::PacketPipeline *makePipeline(const std::string &name, int gpuId)
{
  if (name.empty())
    return nullptr;
  auto pipeline = libfreenect2::createPacketPipeline(name, gpuId);
  if (!pipeline)
    std::cerr << "pipeline '" << name << "' not available in this build; using default" << std::endl;
  return pipeline.release();
}

/// One open Kinect plus its capture thread.
struct DeviceRunner
{
  libfreenect2::Freenect2Device *dev = nullptr;   ///< owned by the Freenect2 context
  std::string serial;
  std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener;
  std::thread thread;

  std::atomic<uint64_t> frames{0};
  std::atomic<bool> stop{false};

  uint64_t last_report_frames = 0;   ///< main-thread only, for FPS deltas
  double fps_display = 0.0;          ///< main-thread only, refreshed once a second

#ifdef MULTIKINECT_WITH_OPENCV
  // Viewer hand-off: the capture thread converts the frames it needs to
  // self-owned cv::Mats and releases the listener frame immediately (its pool
  // is small); the UI thread promotes `pending_*` to `shown_*`.
  std::mutex mtx;
  cv::Mat pending_rgb, pending_depth;
  bool has_pending = false;
  cv::Mat shown_rgb, shown_depth;   ///< UI thread only
#endif
};

#ifdef MULTIKINECT_WITH_OPENCV
cv::Mat colouriseDepth(const libfreenect2::Frame *d)
{
  const cv::Mat mm(static_cast<int>(d->height), static_cast<int>(d->width), CV_32FC1, d->data);
  cv::Mat u8, colour;
  mm.convertTo(u8, CV_8UC1, 255.0 / 4500.0);   // 0..4.5 m -> 0..255; invalid (<=0) stays 0
  cv::applyColorMap(u8, colour, cv::COLORMAP_JET);
  return colour;   // fresh buffer, no alias to d->data
}

cv::Mat rgbPanel(const libfreenect2::Frame *c)
{
  const cv::Mat bgra(static_cast<int>(c->height), static_cast<int>(c->width), CV_8UC4, c->data);
  cv::Mat bgr, small;
  cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
  cv::resize(bgr, small, cv::Size(kPanelW, kRgbH));
  return small;
}

/// Stack a device's RGB strip over its depth strip with a labelled header.
cv::Mat buildPanel(const DeviceRunner &r)
{
  std::vector<cv::Mat> parts;
  if (!r.shown_rgb.empty())
    parts.push_back(r.shown_rgb);
  if (!r.shown_depth.empty())
  {
    cv::Mat d = r.shown_depth;
    if (d.cols != kPanelW)
      cv::resize(d, d, cv::Size(kPanelW, d.rows));
    parts.push_back(d);
  }
  cv::Mat body;
  cv::vconcat(parts, body);

  cv::Mat header(26, body.cols, CV_8UC3, cv::Scalar::all(0));
  char label[96];
  std::snprintf(label, sizeof(label), "%s   %4.1f fps", r.serial.c_str(), r.fps_display);
  cv::putText(header, label, {8, 19}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
              cv::Scalar::all(240), 1, cv::LINE_AA);

  cv::Mat panel;
  cv::vconcat(std::vector<cv::Mat>{header, body}, panel);
  return panel;
}
#endif // MULTIKINECT_WITH_OPENCV

void captureLoop(DeviceRunner *r, bool viewer_enabled)
{
  (void)viewer_enabled;
  while (!r->stop.load(std::memory_order_relaxed))
  {
    libfreenect2::FrameMap fm;
    if (!r->listener->waitForNewFrame(fm, 1000))
      continue;   // timeout -- re-check stop flag and wait again

    r->frames.fetch_add(1, std::memory_order_relaxed);

#ifdef MULTIKINECT_WITH_OPENCV
    if (viewer_enabled)
    {
      cv::Mat rgb, depth;
      if (const libfreenect2::Frame *c = libfreenect2::getFrame(fm, libfreenect2::Frame::Color))
        rgb = rgbPanel(c);
      if (const libfreenect2::Frame *d = libfreenect2::getFrame(fm, libfreenect2::Frame::Depth))
        depth = colouriseDepth(d);
      std::lock_guard<std::mutex> lk(r->mtx);
      r->pending_rgb = rgb;
      r->pending_depth = depth;
      r->has_pending = true;
    }
#endif

    r->listener->release(fm);   // prompt: the listener's frame pool is small
  }
}

} // namespace

int main(int argc, char *argv[])
{
  std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;

  argparse::ArgumentParser program("MultiKinect", LIBFREENECT2_VERSION);
  program.add_description("Open every connected Kinect v2 at once and report per-device throughput.");
  program.add_epilog("The pipeline token and the device serials are bare positionals, kept together.");
  program.add_argument("-gpu").metavar("<id>").scan<'i', int>().default_value(-1)
      .help("GPU device id for the cl/cuda/vk pipelines");
  program.add_argument("-frames").metavar("<N>").scan<'u', std::size_t>()
      .help("stop once every device has delivered N frames");
  program.add_argument("-capacity").metavar("<N>").scan<'u', std::size_t>().default_value<std::size_t>(1)
      .help("per-device listener buffer depth (default 1)");
  program.add_argument("-norgb").flag().help("disable the color stream");
  program.add_argument("-nodepth").flag().help("disable the depth stream");
  program.add_argument("-viewer").flag().help("one OpenCV window, RGB + depth per device");
  program.add_argument("tokens").nargs(argparse::nargs_pattern::any).metavar("[pipeline] [serial ...]")
      .help("a pipeline token and/or the device serials to open (default: all)");

  try
  {
    program.parse_args(argc, argv);
  }
  catch (const std::exception &err)
  {
    std::cerr << err.what() << "\n\n" << program;
    return 1;
  }

  // The library honours LIBFREENECT2_LOGGER_LEVEL on load; raise to Debug for
  // development only when the user has not set it, and not on the slow
  // Windows console.
#if !defined(_WIN32)
  if (std::getenv("LIBFREENECT2_LOGGER_LEVEL") == nullptr)
    libfreenect2::setLogLevel(libfreenect2::LogLevel::Debug);
#endif

  const int gpuId = program.get<int>("-gpu");
  const std::size_t capacity = std::max<std::size_t>(1, program.get<std::size_t>("-capacity"));
  const bool enable_rgb = !program.get<bool>("-norgb");
  const bool enable_depth = !program.get<bool>("-nodepth");
  bool viewer_enabled = program.get<bool>("-viewer");

  size_t framemax = static_cast<size_t>(-1);
  if (auto f = program.present<std::size_t>("-frames"))
  {
    if (*f == 0) { std::cerr << "invalid frame count '0'" << std::endl; return -1; }
    framemax = *f;
  }

  std::string pipeline_name;
  std::vector<std::string> wanted_serials;
  if (auto tokens = program.present<std::vector<std::string>>("tokens"))
  {
    for (const auto &tok : *tokens)
    {
      if (tok.find_first_not_of("0123456789") == std::string::npos)
        wanted_serials.push_back(tok);
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

#ifndef MULTIKINECT_WITH_OPENCV
  if (viewer_enabled)
  {
    std::cerr << "this build has no OpenCV -- -viewer unavailable, running headless" << std::endl;
    viewer_enabled = false;
  }
#endif

  libfreenect2::Freenect2 freenect2;

  const int num_devices = freenect2.enumerateDevices();
  if (num_devices == 0)
  {
    std::cout << "no devices connected!" << std::endl;
    return -1;
  }

  std::vector<std::string> serials;
  if (wanted_serials.empty())
    for (int i = 0; i < num_devices; ++i)
      serials.push_back(freenect2.getDeviceSerialNumber(i));
  else
    serials = wanted_serials;

  const int frame_types =
      (enable_rgb ? libfreenect2::Frame::Color : 0) |
      (enable_depth ? (libfreenect2::Frame::Ir | libfreenect2::Frame::Depth) : 0);

  std::vector<std::unique_ptr<DeviceRunner>> runners;

  for (const auto &serial : serials)
  {
    libfreenect2::PacketPipeline *pipeline = makePipeline(pipeline_name, gpuId);
    libfreenect2::Freenect2Device *dev =
        pipeline ? freenect2.openDevice(serial, pipeline) : freenect2.openDevice(serial);
    if (dev == nullptr)
    {
      std::cerr << "  [" << serial << "] failed to open -- skipping" << std::endl;
      continue;
    }

    auto r = std::make_unique<DeviceRunner>();
    r->dev = dev;
    r->serial = serial;
    r->listener = std::make_unique<libfreenect2::SyncMultiFrameListener>(
        frame_types, capacity,
        capacity > 1 ? libfreenect2::FrameQueueMode::DropOldest
                     : libfreenect2::FrameQueueMode::DropNew);
    dev->setColorFrameListener(r->listener.get());
    dev->setIrAndDepthFrameListener(r->listener.get());
    runners.push_back(std::move(r));
  }

  if (runners.empty())
  {
    std::cout << "no devices opened!" << std::endl;
    return -1;
  }

  for (auto &r : runners)
  {
    const bool ok = (enable_rgb && enable_depth) ? r->dev->start()
                                                 : r->dev->startStreams(enable_rgb, enable_depth);
    if (!ok)
    {
      std::cerr << "  [" << r->serial << "] failed to start streams" << std::endl;
      return -1;
    }
    std::cout << "opened  serial=" << r->dev->getSerialNumber()
              << "  firmware=" << r->dev->getFirmwareVersion() << std::endl;
  }

  std::signal(SIGINT, sigint_handler);

  for (auto &r : runners)
    r->thread = std::thread(captureLoop, r.get(), viewer_enabled);

  std::cout << "\nstreaming from " << runners.size() << " device(s) -- "
            << (viewer_enabled ? "q/Esc in the window or Ctrl-C" : "Ctrl-C") << " to stop\n"
            << std::endl;

  auto last_report = std::chrono::steady_clock::now();

  while (!g_shutdown.load())
  {
    if (framemax != static_cast<size_t>(-1))
    {
      bool all_done = true;
      for (auto &r : runners)
        all_done &= (r->frames.load() >= framemax);
      if (all_done)
        break;
    }

#ifdef MULTIKINECT_WITH_OPENCV
    if (viewer_enabled)
    {
      std::vector<cv::Mat> panels;
      for (auto &r : runners)
      {
        {
          std::lock_guard<std::mutex> lk(r->mtx);
          if (r->has_pending)
          {
            r->shown_rgb = r->pending_rgb;
            r->shown_depth = r->pending_depth;
            r->has_pending = false;
          }
        }
        if (!r->shown_rgb.empty() || !r->shown_depth.empty())
          panels.push_back(buildPanel(*r));
      }
      if (!panels.empty())
      {
        const int h = std::max_element(panels.begin(), panels.end(),
                        [](const cv::Mat &a, const cv::Mat &b) { return a.rows < b.rows; })->rows;
        for (auto &p : panels)
          if (p.rows != h)
            cv::copyMakeBorder(p, p, 0, h - p.rows, 0, 0, cv::BORDER_CONSTANT);
        cv::Mat grid;
        cv::hconcat(panels, grid);
        cv::imshow("MultiKinect", grid);
      }
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27)
        g_shutdown = true;
    }
    else
#endif
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_report).count();
    if (dt >= 1.0)
    {
      std::printf("fps");
      for (auto &r : runners)
      {
        const uint64_t total = r->frames.load();
        r->fps_display = (total - r->last_report_frames) / dt;
        r->last_report_frames = total;
        std::printf("  |  %s %5.1f (%llu)", r->serial.c_str(), r->fps_display,
                    static_cast<unsigned long long>(total));
      }
      std::printf("\n");
      std::fflush(stdout);
      last_report = now;
    }
  }

  // Shut down: stop capture threads, then stop+close devices BEFORE the
  // listeners (unique_ptr members of DeviceRunner) are destroyed -- releasing
  // a listener while its device still streams is the hazard Protonect warns
  // about.
  for (auto &r : runners)
    r->stop = true;
  for (auto &r : runners)
    if (r->thread.joinable())
      r->thread.join();
  for (auto &r : runners)
  {
    if (!r->dev->stop())
      std::cerr << "  [" << r->serial << "] stop() reported failure" << std::endl;
    if (!r->dev->close())
      std::cerr << "  [" << r->serial << "] close() reported failure" << std::endl;
  }

#ifdef MULTIKINECT_WITH_OPENCV
  if (viewer_enabled)
    cv::destroyAllWindows();
#endif

  std::cout << "\nper-device totals:" << std::endl;
  for (auto &r : runners)
    std::cout << "  " << r->serial << "  " << r->frames.load() << " frames" << std::endl;

  runners.clear();   // destroy listeners now, after stop()/close()
  return 0;
}
