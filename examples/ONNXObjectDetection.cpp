/**
 * @file ONNXObjectDetection.cpp
 * @brief YOLOv8 object-detection example: runs a YOLOv8-family ONNX model
 * (CPU execution provider, see onnx_common.h) on the Kinect's color stream
 * and draws detected boxes + COCO class labels in an OpenCV window.
 *
 * Usage: ONNXObjectDetection --model <path-to-yolov8n.onnx> [--conf <0..1>] [--iou <0..1>]
 */

#include <cstring>
#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "onnx_common.h"

namespace
{

void printUsage()
{
  std::cerr << "Usage: ONNXObjectDetection --model <path-to-yolov8n.onnx> [--conf <0..1>] [--iou <0..1>]\n";
}

} // namespace

int main(int argc, char **argv)
{
  std::string model_path;
  float conf_threshold = 0.25f;
  float iou_threshold = 0.45f;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    if (arg == "--model" && i + 1 < argc)
      model_path = argv[++i];
    else if (arg == "--conf" && i + 1 < argc)
      conf_threshold = std::stof(argv[++i]);
    else if (arg == "--iou" && i + 1 < argc)
      iou_threshold = std::stof(argv[++i]);
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage();
      return 1;
    }
  }

  if (model_path.empty())
  {
    printUsage();
    return 1;
  }

  Ort::Session session{nullptr};
  try
  {
    session = onnx_common::createSession(model_path);
  }
  catch (const Ort::Exception &e)
  {
    std::cerr << "Failed to load ONNX model '" << model_path << "': " << e.what() << "\n";
    return 1;
  }

  Ort::AllocatorWithDefaultOptions allocator;
  Ort::AllocatedStringPtr input_name = session.GetInputNameAllocated(0, allocator);
  Ort::AllocatedStringPtr output_name = session.GetOutputNameAllocated(0, allocator);
  const char *input_names[] = {input_name.get()};
  const char *output_names[] = {output_name.get()};

  std::vector<int64_t> input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  int model_size = 640;
  if (input_shape.size() == 4 && input_shape[2] > 0)
    model_size = static_cast<int>(input_shape[2]);
  std::cout << "Model input size: " << model_size << "x" << model_size << "\n";

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

  std::cout << "Serial:   " << device->getSerialNumber() << "\n";
  std::cout << "Firmware: " << device->getFirmwareVersion() << "\n";

  const std::string window_name = "ONNX Object Detection";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  while (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) > 0)
  {
    libfreenect2::FrameMap frames;
    listener.waitForNewFrame(frames);

    libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
    cv::Mat bgrx(color->height, color->width, CV_8UC4, color->data);

    cv::Mat letterboxed;
    onnx_common::LetterboxInfo lb = onnx_common::letterbox(bgrx, letterboxed, model_size);
    std::vector<float> tensor_data = onnx_common::toChwTensor(letterboxed);

    std::array<int64_t, 4> tensor_shape = {1, 3, model_size, model_size};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, tensor_data.data(), tensor_data.size(), tensor_shape.data(), tensor_shape.size());

    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    const float *output = output_tensors[0].GetTensorData<float>();
    std::vector<int64_t> out_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    // [1, 84, num_anchors]: 4 box coords + 80 class scores, anchor-free (no objectness).
    const int64_t num_channels = out_shape[1];
    const int64_t num_anchors = out_shape[2];
    const int num_classes = static_cast<int>(num_channels - 4);

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;

    for (int64_t a = 0; a < num_anchors; ++a)
    {
      float cx = output[0 * num_anchors + a];
      float cy = output[1 * num_anchors + a];
      float w = output[2 * num_anchors + a];
      float h = output[3 * num_anchors + a];

      int best_class = -1;
      float best_score = 0.0f;
      for (int c = 0; c < num_classes; ++c)
      {
        float score = output[(4 + c) * num_anchors + a];
        if (score > best_score)
        {
          best_score = score;
          best_class = c;
        }
      }

      if (best_score < conf_threshold)
        continue;

      boxes.push_back(onnx_common::mapBoxToOriginal(cx, cy, w, h, lb));
      scores.push_back(best_score);
      class_ids.push_back(best_class);
    }

    std::vector<int> keep = onnx_common::nms(boxes, scores, iou_threshold);

    cv::Mat display;
    cv::cvtColor(bgrx, display, cv::COLOR_BGRA2BGR);

    for (int idx : keep)
    {
      const cv::Rect &box = boxes[idx] & cv::Rect(0, 0, display.cols, display.rows);
      const char *label = (class_ids[idx] >= 0 && class_ids[idx] < static_cast<int>(onnx_common::kCocoClassNames.size()))
                               ? onnx_common::kCocoClassNames[class_ids[idx]]
                               : "?";
      cv::rectangle(display, box, cv::Scalar(0, 255, 0), 2);
      char text[128];
      std::snprintf(text, sizeof(text), "%s %.2f", label, scores[idx]);
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      cv::rectangle(display, cv::Point(box.x, box.y - text_size.height - 4),
                    cv::Point(box.x + text_size.width + 4, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
      cv::putText(display, text, cv::Point(box.x + 2, box.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 0, 0), 1);
    }

    cv::imshow(window_name, display);
    listener.release(frames);

    if (cv::waitKey(1) == 27) // Esc
      break;
  }

  static_cast<void>(device->stop());
  static_cast<void>(device->close());

  return 0;
}
