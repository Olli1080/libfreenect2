# libfreenect2

## Table of Contents

* [Description](README.md#description)
* [Requirements](README.md#requirements)
* [Troubleshooting](README.md#troubleshooting-and-reporting-bugs)
* [Maintainers](README.md#maintainers)
* [Installation](README.md#installation)
  * [Windows / Visual Studio](README.md#windows--visual-studio)
  * [Windows / vcpkg](README.md#windows--vcpkg)
  * [Python bindings](README.md#python-bindings)
  * [MacOS](README.md#macos)
  * [Linux](README.md#linux)
* [API Documentation (external)](https://openkinect.github.io/libfreenect2/)

## Description

Driver for Kinect for Windows v2 (K4W2) devices (release and developer preview).

Note: libfreenect2 does not do anything for either Kinect for Windows v1 or Kinect for Xbox 360 sensors. Use libfreenect1 for those sensors.

If you are using libfreenect2 in an academic context, please cite our work using the following DOI: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.50641.svg)](https://doi.org/10.5281/zenodo.50641)



If you use the KDE depth unwrapping algorithm implemented in the library, please also cite this ECCV 2016 [paper](http://users.isy.liu.se/cvl/perfo/abstracts/jaremo16.html).

This driver supports:
* RGB image transfer
* IR and depth image transfer
* registration of RGB and depth images

This fork ([Olli1080/libfreenect2](https://github.com/Olli1080/libfreenect2)) additionally adds:
* a Vulkan depth-processing backend (`ENABLE_VULKAN`, alongside OpenGL/OpenCL/CUDA)
* microphone-array audio capture with direction-of-arrival/beamforming DSP (`ENABLE_AUDIO`)
* a VA-API hardware JPEG decode backend for Intel GPUs on Linux (`ENABLE_VAAPI`, off by default)
* `pyfreenect2`, pybind11-based Python bindings (`BUILD_PYTHON_BINDINGS`, off by default) — see [Python bindings](README.md#python-bindings)
* example apps: a multi-Kinect capture/throughput demo with an optional side-by-side viewer ([examples/MultiKinect.cpp](examples/MultiKinect.cpp), plus [examples/python/multikinect.py](examples/python/multikinect.py)), a PCL point-cloud viewer (`BUILD_PCL_EXAMPLE`), ONNX Runtime object-detection/pose/hand-tracking demos (`BUILD_ONNX_EXAMPLES`), and a network streaming/recording tool (`BUILD_STREAMER_RECORDER`)

See [cmake/Options.cmake](cmake/Options.cmake) for the full list of build options.

Missing features:
* firmware updates (see [issue #460](https://github.com/OpenKinect/libfreenect2/issues/460) for WiP)

Watch the OpenKinect wiki at www.openkinect.org and the mailing list at https://groups.google.com/forum/#!forum/openkinect for the latest developments and more information about the K4W2 USB protocol.

The API reference documentation is provided here https://openkinect.github.io/libfreenect2/.

## Requirements

### Hardware requirements

* USB 3.0 controller. USB 2 is not supported.

Intel and NEC USB 3.0 host controllers are known to work. ASMedia controllers are known to not work.

Virtual machines likely do not work, because USB 3.0 isochronous transfer is quite delicate.

##### Requirements for multiple Kinects

It has been reported to work for up to 5 devices on a high-end PC using multiple separate PCI Express USB3 expansion cards (with NEC controller chip). If you're using Linux, you may have to [increase USBFS memory buffers](https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting#multiple-kinects-try-increasing-usbfs-buffer-size). Depending on the number of Kinects, you may need to use an even larger buffer size. If you're using an expansion card, make sure it's not plugged into an PCI-E x1 slot. A single lane doesn't have enough bandwidth. x8 or x16 slots usually work.

In practice a single modern host controller can also drive a few Kinects at once — three K4W2 streaming color + depth at 30 fps have been run off one Intel USB 3.2 xHCI controller, with occasional brief frame-rate dips under bus contention. Separate controllers remain the recommendation as the device count grows. `MultiKinect` (see [examples/MultiKinect.cpp](examples/MultiKinect.cpp)) opens every connected device and prints a per-device FPS table, which is the quickest way to see whether your bus keeps up.

### Operating system requirements

* Windows 10/11
* Debian 12+, Ubuntu 22.04+, or newer Linux distros. Kernel 5.15+ recommended.
* macOS 12+

### Requirements for optional features

* OpenGL depth processing: OpenGL 3.1+ (Windows, Linux, Mac OS X). OpenGL ES is not supported at the moment.
* OpenCL depth processing: OpenCL 1.1
* CUDA depth processing: CUDA 11+ (CUDA Toolkit 11.x or newer)
* Vulkan depth processing: Vulkan 1.2+
* VAAPI JPEG decoding: Intel (minimum Ivy Bridge or newer) and Linux only, optional hardware-accelerated alternative to TurboJPEG, off by default
* VideoToolbox JPEG decoding: macOS only
* OpenNI2 integration: OpenNI2 2.2.0.33
* Microphone array audio capture: FFTW3 (for direction-of-arrival/beamforming DSP)
* Python bindings: pybind11, Python 3 (with a dynamic/extension-capable build — see [Python bindings](README.md#python-bindings))


## Troubleshooting and reporting bugs

First, check https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting for known issues.

When you report USB issues, please attach relevant debug log from running the program with environment variable `LIBUSB_DEBUG=3`, and relevant log from `dmesg`. Also include relevant hardware information `lspci` and `lsusb -t`.

## Maintainers

* Joshua Blake <joshblake@gmail.com>
* Florian Echtler
* Christian Kerl
* Lingzhu Xiang (development/master branch)

## Installation

### Windows / Visual Studio

* Install UsbDk driver (recommended)

    1. Download the latest x64 installer from https://github.com/daynix/UsbDk/releases, install it.

    This doesn't interfere with the Microsoft SDK. Do not install both UsbDK and libusbK drivers.

* (Legacy) Install libusbK driver

    Use this only if UsbDk does not work. You don't need the Kinect for Windows v2 SDK to build and install libfreenect2, though it doesn't hurt to have it too. You don't need to uninstall the SDK or the driver before doing this procedure.

    Install the libusbK backend driver for libusb. Please follow the steps exactly:

    1. Download Zadig from http://zadig.akeo.ie/.
    2. Run Zadig and in options, check "List All Devices" and uncheck "Ignore Hubs or Composite Parents"
    3. Select the "Xbox NUI Sensor (composite parent)" from the drop-down box. (Important: Ignore the "NuiSensor Adaptor" varieties, which are the adapter, NOT the Kinect) The current driver will list usbccgp. USB ID is VID 045E, PID 02C4 or 02D8.
    4. Select libusbK (v3.0.7.0 or newer) from the replacement driver list.
    5. Click the "Replace Driver" button. Click yes on the warning about replacing a system driver. (This is because it is a composite parent.)

    To uninstall the libusbK driver (and get back the official SDK driver, if installed):

    1. Open "Device Manager"
    2. Under "libusbK USB Devices" tree, right click the "Xbox NUI Sensor (Composite Parent)" device and select uninstall.
    3. Important: Check the "Delete the driver software for this device." checkbox, then click OK.

    If you already had the official SDK driver installed and you want to use it:

    4. In Device Manager, in the Action menu, click "Scan for hardware changes."

    This will enumerate the Kinect sensor again and it will pick up the K4W2 SDK driver, and you should be ready to run KinectService.exe again immediately.

    You can go back and forth between the SDK driver and the libusbK driver very quickly and easily with these steps.

* Build

    This repo vendors [vcpkg](https://github.com/microsoft/vcpkg) as a submodule (`externals/vcpkg`) in manifest mode, so all C++ dependencies (libusb, TurboJPEG, GLFW, OpenCL, Vulkan/shaderc, FFTW3, OpenNI2, etc.) are fetched and built automatically from the top-level `vcpkg.json` based on which `cmake/Options.cmake` features you enable -- no manual per-dependency downloads needed.

    The default installation path is `install`, you may change it by editing `CMAKE_INSTALL_PREFIX`. This repo ships a [CMakePresets.json](CMakePresets.json); the easiest path is the bundled `vs2026-vcpkg-all` preset:
    ```
    cmake --preset vs2026-vcpkg-all
    cmake --build --preset vs2026-vcpkg-all --config RelWithDebInfo --target install
    ```
    Or configure manually against a different Visual Studio version:
    ```
    mkdir build && cd build
    cmake .. -G "Visual Studio 18 2026" -DCMAKE_TOOLCHAIN_FILE=../externals/vcpkg/scripts/buildsystems/vcpkg.cmake
    cmake --build . --config RelWithDebInfo --target install
    ```
    Toggle optional backends/examples with `-D<OPTION>=ON/OFF` (see [cmake/Options.cmake](cmake/Options.cmake)), e.g. `-DENABLE_VULKAN=OFF` or `-DBUILD_PCL_EXAMPLE=ON`.
* Install CUDA (optional, Nvidia only)

    CUDA is not vcpkg-managed. Download and install the CUDA Toolkit (including the samples) before configuring with `-DENABLE_CUDA=ON`.
* Run the test program: `.\install\bin\Protonect.exe`, or start debugging in Visual Studio.
* Test OpenNI2 (optional)

    Copy freenect2-openni2.dll, and other dll files (libusb-1.0.dll, glfw3.dll, etc.) in `install\bin` to `C:\Program Files\OpenNI2\Tools\OpenNI2\Drivers`. Then run `C:\Program Files\OpenNI\Tools\NiViewer.exe`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, `vk`, etc to specify the pipeline.

#### Non-vcpkg / manual builds

If you'd rather not use vcpkg, `cmake_modules/` still ships `Find*.cmake` modules for GLFW3, TurboJPEG, OpenNI2, and OpenCL, driven by the same `<Dep>_ROOT` environment-variable convention as upstream libfreenect2 (e.g. extract libusb/GLFW/TurboJPEG anywhere and point `GLFW_ROOT`/`TurboJPEG_ROOT` at them, or drop them under `depends/<name>`). This path is intentionally kept working for anyone who can't or doesn't want to take the vcpkg dependency.

### Windows / vcpkg

The libfreenect2 port in the official [vcpkg registry](https://github.com/microsoft/vcpkg) tracks upstream [OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2) release tags and does **not** include this fork's Vulkan/audio/VAAPI backends.

To consume this fork from another CMake project via vcpkg, use it as a local overlay port:
```
git clone https://github.com/Olli1080/libfreenect2.git
```
Then in your own project's `vcpkg-configuration.json`, point `overlay-ports` at the clone's `overlay-ports` directory:
```json
{
  "overlay-ports": ["<path-to-libfreenect2-clone>/overlay-ports"]
}
```
(or pass `--overlay-ports=<path-to-libfreenect2-clone>/overlay-ports` on the `vcpkg install` command line). `vcpkg install libfreenect2` then resolves against this fork's port (`overlay-ports/libfreenect2/`) instead of the upstream registry one. Available features: `opengl`, `opencl`, `openni2`, `audio`, `vulkan` (all on by default), plus the opt-in `python` feature below; see [overlay-ports/libfreenect2/vcpkg.json](overlay-ports/libfreenect2/vcpkg.json).

### Python bindings

`pyfreenect2` is a [pybind11](https://github.com/pybind/pybind11)-based Python extension module covering device enumeration/control, video+depth capture, and registration -- plus microphone-array audio/DOA when built with `ENABLE_AUDIO` (the default). It's off by default; enable it with:
```
cmake --preset vs2026-vcpkg-all -DBUILD_PYTHON_BINDINGS=ON
cmake --build --preset vs2026-vcpkg-all --config RelWithDebInfo --target pyfreenect2
```
This pulls in `pybind11` and a dynamic/extension-capable `python3` via the vcpkg manifest's `python` feature (see [vcpkg.json](vcpkg.json)). The built module (`pyfreenect2.pyd`) lands next to `Protonect.exe` in the build's `bin` output directory; point `PYTHONPATH` at that directory to import it:
```python
import sys
sys.path.insert(0, r"build\vs2026-vcpkg-all\bin")  # or wherever CMAKE_LIBRARY_OUTPUT_DIRECTORY points
import pyfreenect2

fn = pyfreenect2.Freenect2()
if fn.enumerate_devices() == 0:
    raise SystemExit("no Kinect v2 found")

dev = fn.open_default_device()
listener = pyfreenect2.SyncMultiFrameListener(pyfreenect2.FrameType.Color | pyfreenect2.FrameType.Depth)
dev.set_color_frame_listener(listener)
dev.set_ir_and_depth_frame_listener(listener)

if not dev.start():
    raise SystemExit("failed to start device")
try:
    frames = listener.wait_for_new_frame(10_000)
    depth = frames[pyfreenect2.FrameType.Depth]
    print(depth.width, depth.height)   # e.g. numpy.asarray(depth) for a zero-copy view
    listener.release(frames)
finally:
    dev.stop()
    dev.close()
```
This mirrors [examples/Protonect.cpp](examples/Protonect.cpp)'s enumerate -> open -> listen -> start -> loop -> stop/close flow. Frames support the buffer protocol, so `numpy.asarray(frame)` gives a zero-copy view (valid only until the frame is released).

**v1 limitations**, tracked as known follow-up work rather than silently missing:
* Only the polling model (`SyncMultiFrameListener` for video, `SyncAudioListener` for audio) is exposed. Subclassing `FrameListener`/`AudioFrameListener` from Python to receive push callbacks is not yet supported.
* No wheel/sdist packaging or `.pyi` type stubs yet -- import via `PYTHONPATH` as shown above.

### MacOS

Use your favorite package managers (brew, ports, etc.) to install most if not all dependencies:

* Make sure these build tools are available: wget, git, cmake, pkg-config. Xcode may provide some of them. Install the rest via package managers.
* Download libfreenect2 source
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    ```
* Install dependencies: libusb, GLFW
    ```
    brew update
    brew install libusb
    brew install glfw3
    ```
* Install TurboJPEG (optional)
    ```
    brew install jpeg-turbo
    ```
* Install CUDA (optional): TODO
* Install OpenNI2 (optional)
    ```
    brew tap brewsci/science
    brew install openni2
    export OPENNI2_REDIST=/usr/local/lib/ni2
    export OPENNI2_INCLUDE=/usr/local/include/ni2
    ```
* Build
    ```
    mkdir build && cd build
    cmake ..
    make
    make install
    ```
* Run the test program: `./bin/Protonect`
* Test OpenNI2. `make install-openni2` (may need sudo), then run `NiViewer`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, etc to specify the pipeline.

### Linux

* Download libfreenect2 source
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    ```
* Install build tools
    ```
    sudo apt-get install build-essential cmake pkg-config
    ```
* Install libusb (version >= 1.0.20)
    ```
    sudo apt-get install libusb-1.0-0-dev
    ```
* Install TurboJPEG
    ```
    sudo apt-get install libturbojpeg0-dev
    ```
* Install OpenGL
    ```
    sudo apt-get install libglfw3-dev
    ```
* Install OpenCL (optional)
    - Intel/AMD GPU: install your GPU vendor's OpenCL drivers and `opencl-headers`.
    - Verify: install `clinfo` to verify your OpenCL stack.
* Install CUDA (optional, Nvidia only): Follow https://developer.nvidia.com/cuda-downloads.
* Install VAAPI (optional, Intel GPU hardware JPEG decode): `sudo apt-get install libva-dev libjpeg-dev`
* Install OpenNI2 (optional): `sudo apt-get install libopenni2-dev`
* Build
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
    make
    make install
    ```
    You need to specify `cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2` for CMake based third-party application to find libfreenect2.
* Set up udev rules for device access: `sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, then replug the Kinect.
* Run the test program: `./bin/Protonect`
* Run OpenNI2 test (optional): `sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, etc to specify the pipeline.
