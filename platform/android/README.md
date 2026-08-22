# Build libfreenect2 for Android

## 1. Requirements

* A latest release of Android NDK.
* A rooted Android device with USB3.0 host.

## 2. Build libusb

* See the [official documentation](https://github.com/libusb/libusb/blob/master/android/README) for instructions to build libusb for Android.
* Due to [certain modifications](https://github.com/libusb/libusb/commit/2f3bc98b0d0f4766496df53c855685a5f0e5e7cf), libusb can no longer be used directly on Android. Grab a latest available [release](https://github.com/libusb/libusb/releases/tag/v1.0.22) instead.
* The arm64-v8a build of libusb segfaults, use an armeabi-v7a build instead if you are running Android on arm64.

## 3. Build libturbojpeg

* See the [official documentation](https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/BUILDING.md) for instructions to build libturbojpeg for Android.
* May encounter problems when using the master branch of libturbojpeg with libfreenect2. However, [2.0.1 release](https://github.com/libjpeg-turbo/libjpeg-turbo/releases/tag/2.0.1) and older versions work well.

## 4. Build spdlog and fmt

* libfreenect2's logging is built on [spdlog](https://github.com/gabime/spdlog), which uses [fmt](https://github.com/fmtlib/fmt) as an external dependency (`SPDLOG_FMT_EXTERNAL`). Build both as shared libraries for Android with the NDK toolchain (e.g. via their CMake builds with `-DBUILD_SHARED_LIBS=ON -DSPDLOG_FMT_EXTERNAL=ON` for spdlog, `-DBUILD_SHARED_LIBS=ON` for fmt).

## 5. Build libfreenect2

* Grab libfreenect2 and change into Android build directory.

```bash
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2/platform/android/jni
```

* Build libfreenect2 with Android NDK.

```bash
/path/to/ndk-build \
  LIBUSB_ROOT=/path/to/libusb/root \
  LIBUSB_SHARED_REL=relative/path/to/libusb1.0.so \
  LIBTURBOJPEG_ROOT=/path/to/libturbojpeg \
  LIBTURBOJPEG_SHARED_REL=relative/path/to/libturbojpeg.so \
  LIBSPDLOG_ROOT=/path/to/spdlog \
  LIBSPDLOG_SHARED_REL=relative/path/to/libspdlog.so \
  LIBFMT_ROOT=/path/to/fmt \
  LIBFMT_SHARED_REL=relative/path/to/libfmt.so
```

* `LIBSPDLOG_ROOT` and `LIBFMT_ROOT` must each contain an `include/` directory with that library's headers, alongside the shared library referenced by `*_SHARED_REL`.

* You will find the built binaries in platform/android/libs.

## 6. Notes

* Now we can only use CPU for depth packet processing. OpenGL ES on Android should work, but it's not yet supported.