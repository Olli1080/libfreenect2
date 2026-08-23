#include "onnx_common.h"

#include <algorithm>
#include <cstdio>
#include <numeric>

#include <opencv2/imgproc.hpp>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <io.h>
#else
#include <unistd.h>
#endif

namespace onnx_common
{

namespace
{

// onnxruntime.dll internally double-registers its own bundled ONNX operator
// schemas at process-wide-registry init time (every "already registered
// from" points at the exact same file/line as the "registering" one --
// confirmed this is onnxruntime's own bug, not this project's, and it's
// harmless: the registry still ends up correct, this is just noise from the
// duplicate-registration attempts). It writes straight to the OS file
// descriptor for stderr (verified: unaffected by redirecting stdout,
// suppressed by redirecting stderr), not through std::cerr, so silencing it
// means redirecting fd 2 itself for the duration of the one-time Ort::Env
// init, not swapping std::cerr's stream buffer.
class ScopedStderrSuppress
{
public:
  ScopedStderrSuppress()
  {
    std::fflush(stderr);
#ifdef _WIN32
    saved_fd_ = _dup(_fileno(stderr));
    FILE *null_file = nullptr;
    freopen_s(&null_file, "NUL", "w", stderr);
#else
    saved_fd_ = dup(fileno(stderr));
    std::freopen("/dev/null", "w", stderr);
#endif
  }

  ~ScopedStderrSuppress()
  {
    std::fflush(stderr);
    if (saved_fd_ >= 0)
    {
#ifdef _WIN32
      _dup2(saved_fd_, _fileno(stderr));
      _close(saved_fd_);
#else
      dup2(saved_fd_, fileno(stderr));
      close(saved_fd_);
#endif
    }
  }

  ScopedStderrSuppress(const ScopedStderrSuppress &) = delete;
  ScopedStderrSuppress &operator=(const ScopedStderrSuppress &) = delete;

private:
  int saved_fd_ = -1;
};

#ifdef _WIN32
std::wstring toWide(const std::string &s)
{
  if (s.empty())
    return std::wstring();
  int size = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring result(size, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), result.data(), size);
  return result;
}
#endif
} // namespace

Ort::Env &getEnv()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "libfreenect2-onnx-examples");
  return env;
}

Ort::Session createSession(const std::string &model_path)
{
  // The schema-registration spam (see ScopedStderrSuppress's comment) turns
  // out to fire lazily on first real API use -- verified via a temporary
  // marker print that it happens somewhere inside this call, not at
  // Ort::Env construction specifically and not at DLL-load/static-init
  // time -- so the whole call needs to be inside the suppressed scope, not
  // just getEnv().
  ScopedStderrSuppress suppress;

  Ort::SessionOptions options;
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  // CPU execution provider only -- see the file header comment.
#ifdef _WIN32
  std::wstring wpath = toWide(model_path);
  return Ort::Session(getEnv(), wpath.c_str(), options);
#else
  return Ort::Session(getEnv(), model_path.c_str(), options);
#endif
}

LetterboxInfo letterbox(const cv::Mat &src, cv::Mat &dst, int model_size)
{
  LetterboxInfo info;
  info.scale = std::min(static_cast<float>(model_size) / src.cols, static_cast<float>(model_size) / src.rows);

  int new_w = static_cast<int>(std::round(src.cols * info.scale));
  int new_h = static_cast<int>(std::round(src.rows * info.scale));

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  info.pad_x = (model_size - new_w) / 2;
  info.pad_y = (model_size - new_h) / 2;
  int pad_right = model_size - new_w - info.pad_x;
  int pad_bottom = model_size - new_h - info.pad_y;

  cv::copyMakeBorder(resized, dst, info.pad_y, pad_bottom, info.pad_x, pad_right,
                      cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return info;
}

std::vector<float> toChwTensor(const cv::Mat &letterboxed_bgr)
{
  cv::Mat rgb;
  if (letterboxed_bgr.channels() == 4)
    cv::cvtColor(letterboxed_bgr, rgb, cv::COLOR_BGRA2RGB);
  else
    cv::cvtColor(letterboxed_bgr, rgb, cv::COLOR_BGR2RGB);

  const int h = rgb.rows;
  const int w = rgb.cols;
  std::vector<float> tensor(static_cast<size_t>(3) * h * w);

  for (int y = 0; y < h; ++y)
  {
    const cv::Vec3b *row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < w; ++x)
    {
      const cv::Vec3b &px = row[x];
      size_t idx = static_cast<size_t>(y) * w + x;
      tensor[0 * static_cast<size_t>(h) * w + idx] = px[0] / 255.0f;
      tensor[1 * static_cast<size_t>(h) * w + idx] = px[1] / 255.0f;
      tensor[2 * static_cast<size_t>(h) * w + idx] = px[2] / 255.0f;
    }
  }
  return tensor;
}

cv::Rect mapBoxToOriginal(float cx, float cy, float w, float h, const LetterboxInfo &info)
{
  float x1 = (cx - w * 0.5f - info.pad_x) / info.scale;
  float y1 = (cy - h * 0.5f - info.pad_y) / info.scale;
  float x2 = (cx + w * 0.5f - info.pad_x) / info.scale;
  float y2 = (cy + h * 0.5f - info.pad_y) / info.scale;
  return cv::Rect(cv::Point(static_cast<int>(std::round(x1)), static_cast<int>(std::round(y1))),
                   cv::Point(static_cast<int>(std::round(x2)), static_cast<int>(std::round(y2))));
}

cv::Point2f mapPointToOriginal(float x, float y, const LetterboxInfo &info)
{
  return cv::Point2f((x - info.pad_x) / info.scale, (y - info.pad_y) / info.scale);
}

std::vector<int> nms(const std::vector<cv::Rect> &boxes, const std::vector<float> &scores, float iou_threshold)
{
  std::vector<int> order(boxes.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) { return scores[a] > scores[b]; });

  std::vector<float> areas(boxes.size());
  for (size_t i = 0; i < boxes.size(); ++i)
    areas[i] = static_cast<float>(boxes[i].area());

  std::vector<bool> suppressed(boxes.size(), false);
  std::vector<int> keep;

  for (size_t oi = 0; oi < order.size(); ++oi)
  {
    int i = order[oi];
    if (suppressed[i])
      continue;
    keep.push_back(i);

    for (size_t oj = oi + 1; oj < order.size(); ++oj)
    {
      int j = order[oj];
      if (suppressed[j])
        continue;

      cv::Rect inter = boxes[i] & boxes[j];
      float inter_area = static_cast<float>(inter.area());
      float union_area = areas[i] + areas[j] - inter_area;
      float iou = union_area > 0.0f ? inter_area / union_area : 0.0f;
      if (iou > iou_threshold)
        suppressed[j] = true;
    }
  }
  return keep;
}

const std::array<const char *, 80> kCocoClassNames = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

const std::array<std::pair<int, int>, 19> kCocoSkeletonEdges = {{
    {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11}, {6, 12}, {5, 6}, {5, 7},
    {6, 8}, {7, 9}, {8, 10}, {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}}};

} // namespace onnx_common
