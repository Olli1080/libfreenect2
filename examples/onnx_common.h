/**
 * @file onnx_common.h
 * @brief Shared ONNX Runtime + OpenCV helpers for the YOLOv8-family example
 * apps (ONNXObjectDetection, ONNXPoseEstimation). CPU execution provider
 * only -- see the "Add ONNX Runtime object-detection and pose-estimation
 * examples" plan for why (DirectML hit an onnxruntime v1.23.2 vs. this
 * vcpkg tree's flatbuffers version wall; CUDA wasn't pursued once CPU was
 * chosen instead).
 */

#ifndef LIBFREENECT2_EXAMPLES_ONNX_COMMON_H_
#define LIBFREENECT2_EXAMPLES_ONNX_COMMON_H_

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

namespace onnx_common
{

/** Process-wide ONNX Runtime environment (one per process, ONNX Runtime's own requirement). */
Ort::Env &getEnv();

/** Create a CPU inference session for the model at `model_path`. Throws Ort::Exception on failure
 * (bad path, malformed model, unsupported ops) -- callers should catch and report, not crash. */
Ort::Session createSession(const std::string &model_path);

/** Aspect-ratio-preserving resize + pad to `model_size` x `model_size`, matching Ultralytics'
 * own YOLOv8 preprocessing convention so a stock export works unmodified. `scale`/`pad_x`/`pad_y`
 * describe the transform so results can be mapped back to `src`'s coordinate space afterward. */
struct LetterboxInfo
{
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
};

LetterboxInfo letterbox(const cv::Mat &src, cv::Mat &dst, int model_size);

/** Converts a letterboxed BGR (or BGRA/BGRX, alpha ignored) uint8 image into a normalized
 * [0,1] RGB float32 CHW tensor (NCHW with N=1), the layout YOLOv8 ONNX exports expect. */
std::vector<float> toChwTensor(const cv::Mat &letterboxed_bgr);

/** Maps a box in letterboxed model-input pixel space back to `letterbox()`'s original image space. */
cv::Rect mapBoxToOriginal(float cx, float cy, float w, float h, const LetterboxInfo &info);

/** Maps a single point (e.g. a keypoint) the same way. */
cv::Point2f mapPointToOriginal(float x, float y, const LetterboxInfo &info);

/** Greedy NMS. Returns indices into `boxes`/`scores` to keep, highest score first. */
std::vector<int> nms(const std::vector<cv::Rect> &boxes, const std::vector<float> &scores, float iou_threshold);

/** One detected object (plain detection) or person (pose) after NMS + coordinate mapping. */
struct Detection
{
  cv::Rect box;
  float confidence = 0.0f;
  int class_id = -1;                    ///< index into kCocoClassNames; -1 for pose output (single implicit "person" class)
  std::vector<cv::Point3f> keypoints;    ///< x, y, visibility -- empty for plain object detection
};

extern const std::array<const char *, 80> kCocoClassNames;

/** 0-indexed COCO 17-keypoint skeleton connections (nose, eyes, ears, shoulders, elbows,
 * wrists, hips, knees, ankles), Ultralytics' standard edge list. */
extern const std::array<std::pair<int, int>, 19> kCocoSkeletonEdges;

} // namespace onnx_common

#endif // LIBFREENECT2_EXAMPLES_ONNX_COMMON_H_
