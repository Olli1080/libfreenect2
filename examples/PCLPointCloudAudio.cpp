/**
 * @file PCLPointCloudAudio.cpp
 * @brief PCL point cloud visualization with live direction-of-arrival
 * estimation, highlighted against the point cloud.
 *
 * Combines PCLPointCloud.cpp's rendering with AudioVideoSync.cpp's
 * simultaneous audio+video capture pattern (AudioCapture(device) reusing
 * the video device's already-open USB session) and DoaEstimator
 * (audio_doa.hpp). Shows both DOA estimation methods (GCC-PHAT and
 * delay-and-sum beamforming) as separate arrows so they can be compared
 * directly, and highlights point-cloud points whose azimuth falls near
 * the beamforming estimate.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/audio_capture.hpp>
#include <libfreenect2/audio_doa.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

namespace
{
// Points within this many degrees of the beamforming DOA estimate get
// highlighted. Only applied when the estimate is valid and reasonably
// confident, so a quiet/diffuse-noise room doesn't highlight everything.
const float kHighlightToleranceDeg = 10.0f;
const float kHighlightConfidenceThreshold = 0.15f;

class DoaFeeder : public libfreenect2::AudioFrameListener
{
public:
  explicit DoaFeeder(libfreenect2::DoaEstimator &estimator) : estimator_(estimator) {}

  bool onNewAudioFrame(libfreenect2::AudioFrame *frame) override
  {
    estimator_.addSamples(*frame);
    return false; // AudioCapture deletes the original.
  }

private:
  libfreenect2::DoaEstimator &estimator_;
};

PointCloud::Ptr frame_to_cloud(const libfreenect2::Frame &undistorted,
                                const libfreenect2::Frame &registered,
                                libfreenect2::Registration &reg,
                                const libfreenect2::DoaEstimate &highlight)
{
  PointCloud::Ptr cloud(new PointCloud());
  cloud->width = undistorted.width;
  cloud->height = undistorted.height;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  bool do_highlight = highlight.valid && highlight.confidence >= kHighlightConfidenceThreshold;
  float highlight_rad = highlight.azimuth_degrees * static_cast<float>(M_PI) / 180.0f;

  for (int r = 0; r < static_cast<int>(undistorted.height); ++r)
  {
    for (int c = 0; c < static_cast<int>(undistorted.width); ++c)
    {
      float x, y, z, color;
      reg.getPointXYZRGB(&undistorted, &registered, r, c, x, y, z, color);

      pcl::PointXYZRGB &point = cloud->points[r * cloud->width + c];
      if (!std::isfinite(z))
      {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      point.x = x;
      point.y = y;
      point.z = z;

      uint8_t bgrx[4];
      std::memcpy(bgrx, &color, sizeof(bgrx));
      point.b = bgrx[0];
      point.g = bgrx[1];
      point.r = bgrx[2];

      if (do_highlight)
      {
        // Camera-space azimuth: 0 = straight ahead (+z), matching
        // DoaEstimate::azimuth_degrees's convention exactly (see
        // audio_doa.hpp) -- no conversion needed here.
        float azimuth = std::atan2(x, z);
        float diff = std::fabs(azimuth - highlight_rad);
        if (diff < kHighlightToleranceDeg * static_cast<float>(M_PI) / 180.0f)
        {
          // Tint toward yellow, scaled by how close to dead-center the
          // point is within the tolerance cone.
          float strength = 1.0f - diff / (kHighlightToleranceDeg * static_cast<float>(M_PI) / 180.0f);
          point.r = static_cast<uint8_t>(std::min(255.0f, point.r + strength * (255 - point.r)));
          point.g = static_cast<uint8_t>(std::min(255.0f, point.g + strength * (255 - point.g)));
          point.b = static_cast<uint8_t>(point.b * (1.0f - strength));
        }
      }
    }
  }

  return cloud;
}

// Adds/replaces a named arrow shape pointing from the origin toward
// azimuth_degrees at a fixed length, or removes it if the estimate isn't
// valid/confident enough to show.
void updateDoaArrow(pcl::visualization::PCLVisualizer::Ptr &viewer, const std::string &id,
                     const libfreenect2::DoaEstimate &estimate, double r, double g, double b)
{
  viewer->removeShape(id);
  if (!estimate.valid || estimate.confidence < kHighlightConfidenceThreshold)
    return;

  float rad = estimate.azimuth_degrees * static_cast<float>(M_PI) / 180.0f;
  const float length = 1.0f;
  pcl::PointXYZ start(0.0f, 0.0f, 0.0f);
  pcl::PointXYZ end(length * std::sin(rad), 0.0f, length * std::cos(rad));
  viewer->addArrow(end, start, r, g, b, false, id);
}
} // namespace

int main()
{
  libfreenect2::Freenect2 freenect2;

  if (freenect2.enumerateDevices() == 0)
  {
    std::cerr << "No Kinect v2 device found.\n";
    return 1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  libfreenect2::Freenect2Device *device = freenect2.openDevice(serial);
  if (!device)
  {
    std::cerr << "Failed to open device with serial " << serial << ".\n";
    return 1;
  }

  libfreenect2::SyncMultiFrameListener listener(
      libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
  device->setColorFrameListener(&listener);
  device->setIrAndDepthFrameListener(&listener);

  if (!device->start())
  {
    std::cerr << "Failed to start device.\n";
    return 1;
  }

  // Reuses the video device's already-open USB session -- see
  // audio_capture.hpp's AudioCapture(Freenect2Device*) constructor.
  // Video must already be started before this, same as AudioVideoSync.cpp.
  libfreenect2::DoaEstimator doa;
  DoaFeeder feeder(doa);
  libfreenect2::AudioCapture audio(device);
  bool audio_running = audio.isAvailable() && audio.start(&feeder);
  if (!audio_running)
    std::cerr << "No Kinect v2 microphone array found -- continuing video-only, no DOA.\n";

  libfreenect2::Registration registration(device->getIrCameraParams(), device->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4);
  libfreenect2::Frame registered(512, 424, 4);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Kinect v2 Point Cloud + DOA"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();

  bool first_frame = true;

  while (!viewer->wasStopped())
  {
    libfreenect2::FrameMap frames;
    listener.waitForNewFrame(frames);

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration.apply(rgb, depth, &undistorted, &registered);

    libfreenect2::DoaEstimate gcc = doa.estimateGccPhat();
    libfreenect2::DoaEstimate beam = doa.estimateBeamforming();

    PointCloud::Ptr cloud = frame_to_cloud(undistorted, registered, registration, beam);

    if (first_frame)
    {
      viewer->addPointCloud(cloud, "cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
      first_frame = false;
    }
    else
    {
      viewer->updatePointCloud(cloud, "cloud");
    }

    if (audio_running)
    {
      updateDoaArrow(viewer, "doa_gcc", gcc, 0.0, 1.0, 0.0);   // green = GCC-PHAT
      updateDoaArrow(viewer, "doa_beam", beam, 0.0, 0.4, 1.0); // blue = delay-and-sum

      if (gcc.valid && beam.valid)
      {
        std::printf("DOA gcc=%.1fdeg (conf=%.2f)  beam=%.1fdeg (conf=%.2f)\n",
                    gcc.azimuth_degrees, gcc.confidence, beam.azimuth_degrees, beam.confidence);
        std::fflush(stdout);
      }
    }

    listener.release(frames);
    viewer->spinOnce(10);
  }

  if (audio_running)
    audio.stop();

  (void)device->stop();
  (void)device->close();

  return 0;
}
