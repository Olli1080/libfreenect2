/**
 * @file PCLPointCloud.cpp
 * @brief PCL point cloud visualization example
 *
 * Opens the first available Kinect v2, registers depth to color, builds a
 * pcl::PointCloud<PointXYZRGB> per frame, and streams it into a
 * PCLVisualizer window until the window is closed.
 */

#include <cstring>
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/pipeline_factory.h>
#include <libfreenect2/registration.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

static PointCloud::Ptr frame_to_cloud(const libfreenect2::Frame& undistorted,
                                       const libfreenect2::Frame& registered,
                                       libfreenect2::Registration& reg)
{
	PointCloud::Ptr cloud(new PointCloud());
	cloud->width = undistorted.width;
	cloud->height = undistorted.height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (int r = 0; r < static_cast<int>(undistorted.height); ++r)
	{
		for (int c = 0; c < static_cast<int>(undistorted.width); ++c)
		{
			float x, y, z, color;
			reg.getPointXYZRGB(&undistorted, &registered, r, c, x, y, z, color);

			pcl::PointXYZRGB& point = cloud->points[r * cloud->width + c];
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
		}
	}

	return cloud;
}

int main(int argc, char** argv)
{
	libfreenect2::PacketPipeline* pipeline = nullptr;
	if (argc > 1)
	{
		std::string arg(argv[1]);
		pipeline = libfreenect2::createPacketPipeline(arg).release();
		if (!pipeline)
			std::cerr << "Unknown/unavailable pipeline '" << arg
			          << "' (expected cpu | gl | cl | clkde | cuda | cudakde | vk)\n";
	}

	libfreenect2::Freenect2 freenect2;

	if (freenect2.enumerateDevices() == 0)
	{
		std::cerr << "No Kinect v2 device found.\n";
		return 1;
	}

	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	libfreenect2::Freenect2Device* device = pipeline
		? freenect2.openDevice(serial, pipeline)
		: freenect2.openDevice(serial);
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

	std::cout << "Serial:   " << device->getSerialNumber() << "\n";
	std::cout << "Firmware: " << device->getFirmwareVersion() << "\n";

	libfreenect2::Registration registration(device->getIrCameraParams(), device->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4);
	libfreenect2::Frame registered(512, 424, 4);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Kinect v2 Point Cloud"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();

	bool first_frame = true;

	while (!viewer->wasStopped())
	{
		libfreenect2::FrameMap frames;
		listener.waitForNewFrame(frames);

		libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

		registration.apply(rgb, depth, &undistorted, &registered);
		PointCloud::Ptr cloud = frame_to_cloud(undistorted, registered, registration);

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

		listener.release(frames);
		viewer->spinOnce(10);
	}

	(void)device->stop();
	(void)device->close();

	return 0;
}
