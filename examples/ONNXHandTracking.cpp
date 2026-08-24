/**
 * @file ONNXHandTracking.cpp
 * @brief Two-stage MediaPipe hand tracking (palm detector + 21-point hand
 * landmark model, CPU execution provider, see onnx_common.h) on the
 * Kinect's color stream, drawing hand skeletons in an OpenCV window.
 *
 * Ported from opencv/opencv_zoo's palm_detection_mediapipe (demo.cpp,
 * MPPalmDet) and handpose_estimation_mediapipe (mp_handpose.py,
 * MPHandPose) reference implementations (Apache-2.0) -- same math, same
 * constants, translated from cv::dnn::Net + numpy to Ort::Session + cv::Mat.
 * Both models use NHWC input (unlike the YOLOv8 examples' NCHW), black
 * padding (not YOLOv8's 114-gray), and each has its own coordinate-mapping
 * convention -- deliberately kept separate from onnx_common.h's
 * letterbox()/toChwTensor()/mapBoxToOriginal(), which are YOLOv8-specific
 * and would silently give wrong answers if reused here.
 *
 * Usage: ONNXHandTracking --palm-model <path> --hand-model <path>
 *          [--palm-conf <0..1>] [--palm-iou <0..1>] [--hand-conf <0..1>]
 *          [--min-depth <meters>] [--max-depth <meters>] [--max-hands <n>]
 *
 * Palm detection never runs on the raw full color frame: doing so crushes a
 * distant hand's true pixel size below what the palm CNN can see (a ~9cm
 * palm is ~10px in the 192x192 input at 1m, ~5px at 2m). Instead, an
 * already-tracked hand gets a tight crop+upscale around its last known box
 * each frame (see Track/enlargeAndClamp below), and a new hand is
 * (re)acquired via the depth sensor (Registration/bigdepth) picking out
 * plausible-distance blobs to crop around instead of scanning the whole
 * frame.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "onnx_common.h"
#include "mediapipe_palm_anchors.inc.h"

namespace
{

const float kPi = 3.14159265358979323846f;

void printUsage()
{
  std::cerr << "Usage: ONNXHandTracking --palm-model <path> --hand-model <path> "
               "[--palm-conf <0..1>] [--palm-iou <0..1>] [--hand-conf <0..1>] "
               "[--min-depth <meters>] [--max-depth <meters>] [--max-hands <n>]\n";
}

// A 2-point axis-aligned box, [top-left, bottom-right] -- mirrors the
// reference implementation's [[x1,y1],[x2,y2]] convention exactly, since
// several steps (shift/enlarge/rotate) are easiest to keep faithful to the
// source when expressed the same way rather than as a cv::Rect.
using Box2 = std::array<cv::Point2f, 2>;

// --- Stage 1: palm detection (192x192 NHWC input) -----------------------

constexpr int kPalmInputSize = 192;
constexpr int kPalmNumAnchors = 2016;

struct PalmDetection
{
  cv::Rect2f box;                  // original image space
  std::array<cv::Point2f, 7> landmarks; // original image space (palm base, middle-finger base, + 5 more)
  float score = 0.0f;
};

// NHWC RGB float32 [0,1], black-padded aspect-preserving resize -- see
// MPPalmDet::preprocess() in opencv_zoo's demo.cpp.
struct PalmPreprocessResult
{
  std::vector<float> tensor; // 1x192x192x3
  cv::Point2f pad_bias;      // in ORIGINAL image pixels (already divided by ratio)
  float ratio = 1.0f;
};

PalmPreprocessResult preprocessForPalm(const cv::Mat &bgrx)
{
  PalmPreprocessResult result;
  float ratio = std::min(static_cast<float>(kPalmInputSize) / bgrx.cols,
                          static_cast<float>(kPalmInputSize) / bgrx.rows);
  result.ratio = ratio;

  cv::Size ratio_size(static_cast<int>(bgrx.cols * ratio), static_cast<int>(bgrx.rows * ratio));
  cv::Mat resized;
  cv::resize(bgrx, resized, ratio_size);

  int pad_w = kPalmInputSize - ratio_size.width;
  int pad_h = kPalmInputSize - ratio_size.height;
  int pad_x = pad_w / 2;
  int pad_y = pad_h / 2;

  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, pad_y, pad_h - pad_y, pad_x, pad_w - pad_x,
                      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  cv::Mat rgb;
  cv::cvtColor(padded, rgb, padded.channels() == 4 ? cv::COLOR_BGRA2RGB : cv::COLOR_BGR2RGB);

  result.tensor.resize(static_cast<size_t>(kPalmInputSize) * kPalmInputSize * 3);
  for (int y = 0; y < kPalmInputSize; ++y)
  {
    const cv::Vec3b *row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kPalmInputSize; ++x)
    {
      size_t idx = (static_cast<size_t>(y) * kPalmInputSize + x) * 3;
      result.tensor[idx + 0] = row[x][0] / 255.0f;
      result.tensor[idx + 1] = row[x][1] / 255.0f;
      result.tensor[idx + 2] = row[x][2] / 255.0f;
    }
  }

  result.pad_bias = cv::Point2f(pad_x / ratio, pad_y / ratio);
  return result;
}

std::vector<PalmDetection> detectPalms(Ort::Session &session, const cv::Mat &bgrx,
                                        float score_threshold, float iou_threshold)
{
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::AllocatedStringPtr input_name = session.GetInputNameAllocated(0, allocator);
  Ort::AllocatedStringPtr output_name0 = session.GetOutputNameAllocated(0, allocator); // boxes+landmarks [1,2016,18]
  Ort::AllocatedStringPtr output_name1 = session.GetOutputNameAllocated(1, allocator); // scores [1,2016,1]
  const char *input_names[] = {input_name.get()};
  const char *output_names[] = {output_name0.get(), output_name1.get()};

  PalmPreprocessResult pre = preprocessForPalm(bgrx);

  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::array<int64_t, 4> tensor_shape = {1, kPalmInputSize, kPalmInputSize, 3};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info, pre.tensor.data(), pre.tensor.size(), tensor_shape.data(), tensor_shape.size());

  auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 2);
  const float *boxes = output_tensors[0].GetTensorData<float>();  // [2016, 18]
  const float *scores = output_tensors[1].GetTensorData<float>(); // [2016, 1]

  float scale = static_cast<float>(std::max(bgrx.rows, bgrx.cols));

  std::vector<cv::Rect> nms_boxes;
  std::vector<float> nms_scores;
  std::vector<std::array<cv::Point2f, 7>> nms_landmarks;

  for (int i = 0; i < kPalmNumAnchors; ++i)
  {
    float raw_score = scores[i];
    float score = 1.0f / (1.0f + std::exp(-raw_score));
    if (score <= score_threshold)
      continue;

    const float *box_row = boxes + static_cast<size_t>(i) * 18;
    cv::Point2f anchor(kPalmAnchors[i][0], kPalmAnchors[i][1]);

    cv::Point2f cxy_delta(box_row[0] / kPalmInputSize, box_row[1] / kPalmInputSize);
    cv::Point2f wh_delta(box_row[2] / kPalmInputSize, box_row[3] / kPalmInputSize);

    cv::Point2f xy1((cxy_delta.x - wh_delta.x * 0.5f + anchor.x) * scale - pre.pad_bias.x,
                     (cxy_delta.y - wh_delta.y * 0.5f + anchor.y) * scale - pre.pad_bias.y);
    cv::Point2f xy2((cxy_delta.x + wh_delta.x * 0.5f + anchor.x) * scale - pre.pad_bias.x,
                     (cxy_delta.y + wh_delta.y * 0.5f + anchor.y) * scale - pre.pad_bias.y);

    std::array<cv::Point2f, 7> landmarks;
    for (int j = 0; j < 7; ++j)
    {
      float dx = box_row[4 + j * 2] / kPalmInputSize;
      float dy = box_row[4 + j * 2 + 1] / kPalmInputSize;
      dx += anchor.x;
      dy += anchor.y;
      landmarks[j] = cv::Point2f(dx * scale - pre.pad_bias.x, dy * scale - pre.pad_bias.y);
    }

    nms_boxes.emplace_back(cv::Point(static_cast<int>(xy1.x), static_cast<int>(xy1.y)),
                            cv::Point(static_cast<int>(xy2.x), static_cast<int>(xy2.y)));
    nms_scores.push_back(score);
    nms_landmarks.push_back(landmarks);
  }

  std::vector<int> keep = onnx_common::nms(nms_boxes, nms_scores, iou_threshold);

  std::vector<PalmDetection> result;
  result.reserve(keep.size());
  for (int idx : keep)
  {
    PalmDetection det;
    det.box = cv::Rect2f(nms_boxes[idx]);
    det.landmarks = nms_landmarks[idx];
    det.score = nms_scores[idx];
    result.push_back(det);
  }
  return result;
}

// --- Between stage 1 and 2: crop, rotate upright, pad, resize to 224 -----
// See MPHandPose._cropAndPadFromPalm() / _preprocess() in opencv_zoo's
// mp_handpose.py.

constexpr int kHandInputSize = 224;
constexpr int kPalmLandmarkIndexOfPalmBase = 0;
constexpr int kPalmLandmarkIndexOfMiddleFingerBase = 2;
const cv::Point2f kPalmBoxPreShiftVector(0.0f, 0.0f);
constexpr float kPalmBoxPreEnlargeFactor = 4.0f;
const cv::Point2f kPalmBoxShiftVector(0.0f, -0.4f);
constexpr float kPalmBoxEnlargeFactor = 3.0f;
const cv::Point2f kHandBoxShiftVector(0.0f, -0.1f);
constexpr float kHandBoxEnlargeFactor = 1.65f;

// --- Temporal tracking + depth-guided acquisition ------------------------
// See Track/findDepthGuidedCandidates below kHandConnections for how these
// are used: kTrackingCropMarginFactor governs the crop used to re-find an
// already-tracked palm each frame (tight, cheap, ~1 palm-CNN pass/hand).
// kHandFootprintMeters/kDepthCandidateMinCropSize/kDepthCandidateMaxCropSize
// size the crop used to (re)acquire a NEW hand around a near-depth peak --
// sized from the physical footprint at that peak's actual depth (via focal
// length) rather than from a connected component's bounding box, which for
// two hands joined to one torso -- especially likely at close range, where
// more of the body falls within the depth window -- balloons back toward a
// full-frame-sized crop (undoing the whole point) and only lets ONE hand
// win the palm CNN's internal NMS.
constexpr float kTrackingCropMarginFactor = 3.0f;
constexpr int kTrackingCropMinSize = 100;
constexpr int kMaxMissedFrames = 5;
constexpr float kTrackOverlapIouThreshold = 0.3f;
constexpr float kHandFootprintMeters = 0.35f; // hand + a bit of forearm
constexpr int kDepthCandidateMinCropSize = 100;
constexpr int kDepthCandidateMaxCropSize = 900; // hard cap regardless of depth
// Fixed-range depth tolerance (mm, relative to the peak/seed pixel, not
// neighbor-to-neighbor) for the flood-fill that suppresses a whole
// near-uniform-depth surface around a peak in one go -- see the comment at
// its call site in findDepthGuidedCandidates for why this matters.
constexpr double kFloodDepthToleranceMm = 60.0;
// Extra candidate peaks requested beyond the number of open hand slots --
// needed because plenty of near-depth peaks (torso, face, shoulder) are
// valid depth-range peaks but contain no hand at all; without headroom
// those can burn through the whole candidate budget before a genuinely
// separate second hand's peak is ever reached.
constexpr int kDepthCandidateSearchHeadroom = 6;

struct CropResult
{
  cv::Mat image;   // padded square crop
  Box2 box;        // shifted/enlarged/clipped box, in the coordinate space `image` was cropped from
  cv::Point2f bias; // top-left of the padded crop, same coordinate space
};

// `for_rotation` pads to the crop's diagonal (not just its longer side) so
// nothing gets clipped once the crop is subsequently rotated.
CropResult cropAndPadFromPalm(const cv::Mat &image, Box2 box, cv::Point2f shift_vector,
                               float enlarge_factor, bool for_rotation)
{
  cv::Point2f wh = box[1] - box[0];
  cv::Point2f shift(shift_vector.x * wh.x, shift_vector.y * wh.y);
  box[0] += shift;
  box[1] += shift;

  cv::Point2f center = (box[0] + box[1]) * 0.5f;
  wh = box[1] - box[0];
  cv::Point2f half = wh * (enlarge_factor * 0.5f);
  box[0] = center - half;
  box[1] = center + half;

  cv::Point tl(static_cast<int>(box[0].x), static_cast<int>(box[0].y));
  cv::Point br(static_cast<int>(box[1].x), static_cast<int>(box[1].y));
  tl.x = std::clamp(tl.x, 0, image.cols);
  tl.y = std::clamp(tl.y, 0, image.rows);
  br.x = std::clamp(br.x, tl.x + 1, image.cols);
  br.y = std::clamp(br.y, tl.y + 1, image.rows);

  cv::Mat crop = image(cv::Rect(tl, br)).clone();

  float side_len_f = for_rotation
                          ? std::sqrt(static_cast<float>(crop.rows) * crop.rows + static_cast<float>(crop.cols) * crop.cols)
                          : static_cast<float>(std::max(crop.rows, crop.cols));
  int side = static_cast<int>(side_len_f);
  int pad_h = std::max(0, side - crop.rows);
  int pad_w = std::max(0, side - crop.cols);
  int left = pad_w / 2, top = pad_h / 2;
  int right = pad_w - left, bottom = pad_h - top;

  CropResult result;
  cv::copyMakeBorder(crop, result.image, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  result.box = {cv::Point2f(tl), cv::Point2f(br)};
  result.bias = cv::Point2f(static_cast<float>(tl.x - left), static_cast<float>(tl.y - top));
  return result;
}

struct HandPreprocessResult
{
  cv::Mat blob;           // 224x224x3 RGB float32 [0,1], HWC
  Box2 rotated_palm_bbox; // in `rotated_image`'s coordinate space (post second crop)
  float angle_deg = 0.0f;
  cv::Mat rotation_matrix; // 2x3 CV_64F, from getRotationMatrix2D during the first (pre-rotation) crop
  cv::Point2f pad_bias;
};

HandPreprocessResult preprocessForHand(const cv::Mat &bgrx, const PalmDetection &palm)
{
  Box2 palm_box = {cv::Point2f(palm.box.x, palm.box.y),
                    cv::Point2f(palm.box.x + palm.box.width, palm.box.y + palm.box.height)};

  cv::Mat bgr;
  if (bgrx.channels() == 4)
    cv::cvtColor(bgrx, bgr, cv::COLOR_BGRA2BGR);
  else
    bgr = bgrx;

  CropResult first = cropAndPadFromPalm(bgr, palm_box, kPalmBoxPreShiftVector, kPalmBoxPreEnlargeFactor, true);
  cv::Mat image_rgb;
  cv::cvtColor(first.image, image_rgb, cv::COLOR_BGR2RGB);
  cv::Point2f pad_bias = first.bias;

  Box2 local_box = {first.box[0] - pad_bias, first.box[1] - pad_bias};
  std::array<cv::Point2f, 7> local_landmarks;
  for (int i = 0; i < 7; ++i)
    local_landmarks[i] = palm.landmarks[i] - pad_bias;

  cv::Point2f p1 = local_landmarks[kPalmLandmarkIndexOfPalmBase];
  cv::Point2f p2 = local_landmarks[kPalmLandmarkIndexOfMiddleFingerBase];
  float radians = kPi / 2.0f - std::atan2(-(p2.y - p1.y), p2.x - p1.x);
  radians = radians - 2.0f * kPi * std::floor((radians + kPi) / (2.0f * kPi));
  float angle_deg = radians * 180.0f / kPi;

  cv::Point2f center = (local_box[0] + local_box[1]) * 0.5f;
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle_deg, 1.0);
  double m00 = rotation_matrix.at<double>(0, 0), m01 = rotation_matrix.at<double>(0, 1), m02 = rotation_matrix.at<double>(0, 2);
  double m10 = rotation_matrix.at<double>(1, 0), m11 = rotation_matrix.at<double>(1, 1), m12 = rotation_matrix.at<double>(1, 2);

  cv::Mat rotated_image;
  cv::warpAffine(image_rgb, rotated_image, rotation_matrix, image_rgb.size());

  // Standard affine point transform: x' = m00*x + m01*y + m02 (matrix ROWS
  // dot [x,y,1]) -- matches np.dot(homogeneous_coord, rotation_matrix[row]).
  cv::Point2f rmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  cv::Point2f rmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int i = 0; i < 7; ++i)
  {
    float x = local_landmarks[i].x, y = local_landmarks[i].y;
    cv::Point2f rp(static_cast<float>(m00 * x + m01 * y + m02), static_cast<float>(m10 * x + m11 * y + m12));
    rmin.x = std::min(rmin.x, rp.x);
    rmin.y = std::min(rmin.y, rp.y);
    rmax.x = std::max(rmax.x, rp.x);
    rmax.y = std::max(rmax.y, rp.y);
  }
  Box2 rotated_box_pre = {rmin, rmax};

  CropResult second = cropAndPadFromPalm(rotated_image, rotated_box_pre, kPalmBoxShiftVector, kPalmBoxEnlargeFactor, false);

  cv::Mat resized;
  cv::resize(second.image, resized, cv::Size(kHandInputSize, kHandInputSize), 0, 0, cv::INTER_AREA);

  HandPreprocessResult result;
  resized.convertTo(result.blob, CV_32FC3, 1.0 / 255.0);
  result.rotated_palm_bbox = second.box;
  result.angle_deg = angle_deg;
  result.rotation_matrix = rotation_matrix;
  result.pad_bias = pad_bias;
  return result;
}

// --- Stage 2: hand landmarks (224x224 NHWC input, direct regression) ----

constexpr int kNumHandKeypoints = 21;

struct HandResult
{
  Box2 box; // original image space
  std::array<cv::Point3f, kNumHandKeypoints> landmarks; // original image space (x,y); z relative to wrist
  float handedness = 0.0f; // 0 = left, 1 = right
  float confidence = 0.0f;
};

// Un-rotates model-space landmarks back to `bgrx`'s coordinate space. See
// MPHandPose._postprocess(). Note this deliberately does NOT use the same
// row-dot-matrix convention as preprocessForHand()'s point rotation above:
// the reference code applies `landmarks @ coords_rotation_matrix[:, :2]`
// (row-vector times matrix columns), which is the TRANSPOSE of the
// standard affine convention -- i.e. it's applying the inverse rotation
// using the same-signed-angle matrix, not a bug to "fix" into consistency
// with the other one.
std::optional<HandResult> postprocessHand(const float *landmarks_raw, float conf, float handedness,
                                           const HandPreprocessResult &pre, float conf_threshold)
{
  if (conf < conf_threshold)
    return std::nullopt;

  std::array<cv::Point3f, kNumHandKeypoints> landmarks;
  for (int i = 0; i < kNumHandKeypoints; ++i)
    landmarks[i] = cv::Point3f(landmarks_raw[i * 3 + 0], landmarks_raw[i * 3 + 1], landmarks_raw[i * 3 + 2]);

  cv::Point2f wh_rotated = pre.rotated_palm_bbox[1] - pre.rotated_palm_bbox[0];
  float scale_x = wh_rotated.x / kHandInputSize;
  float scale_y = wh_rotated.y / kHandInputSize;
  float max_scale = std::max(scale_x, scale_y);

  for (int i = 0; i < kNumHandKeypoints; ++i)
  {
    landmarks[i].x = (landmarks[i].x - kHandInputSize / 2.0f) * max_scale;
    landmarks[i].y = (landmarks[i].y - kHandInputSize / 2.0f) * max_scale;
    landmarks[i].z = landmarks[i].z * max_scale;
  }

  cv::Mat coords_rot = cv::getRotationMatrix2D(cv::Point2f(0, 0), pre.angle_deg, 1.0);
  double c00 = coords_rot.at<double>(0, 0), c01 = coords_rot.at<double>(0, 1);
  double c10 = coords_rot.at<double>(1, 0), c11 = coords_rot.at<double>(1, 1);

  std::array<cv::Point3f, kNumHandKeypoints> rotated_landmarks;
  for (int i = 0; i < kNumHandKeypoints; ++i)
  {
    float x = landmarks[i].x, y = landmarks[i].y;
    // v @ R (row-vector times matrix): result = x*R[0][*] + y*R[1][*], i.e. dot with matrix COLUMNS.
    rotated_landmarks[i] = cv::Point3f(static_cast<float>(x * c00 + y * c10),
                                        static_cast<float>(x * c01 + y * c11),
                                        landmarks[i].z);
  }

  // Invert the crop's rotation_matrix (rotation part transposed, translation re-derived).
  double m00 = pre.rotation_matrix.at<double>(0, 0), m01 = pre.rotation_matrix.at<double>(0, 1), m02 = pre.rotation_matrix.at<double>(0, 2);
  double m10 = pre.rotation_matrix.at<double>(1, 0), m11 = pre.rotation_matrix.at<double>(1, 1), m12 = pre.rotation_matrix.at<double>(1, 2);
  double r00 = m00, r01 = m10;
  double r10 = m01, r11 = m11;
  double inv_tx = -(r00 * m02 + r01 * m12);
  double inv_ty = -(r10 * m02 + r11 * m12);

  cv::Point2f bbox_center = (pre.rotated_palm_bbox[0] + pre.rotated_palm_bbox[1]) * 0.5f;
  cv::Point2f original_center(static_cast<float>(r00 * bbox_center.x + r01 * bbox_center.y + inv_tx),
                               static_cast<float>(r10 * bbox_center.x + r11 * bbox_center.y + inv_ty));

  cv::Point2f rmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  cv::Point2f rmax(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (int i = 0; i < kNumHandKeypoints; ++i)
  {
    landmarks[i].x = rotated_landmarks[i].x + original_center.x + pre.pad_bias.x;
    landmarks[i].y = rotated_landmarks[i].y + original_center.y + pre.pad_bias.y;
    landmarks[i].z = rotated_landmarks[i].z;
    rmin.x = std::min(rmin.x, landmarks[i].x);
    rmin.y = std::min(rmin.y, landmarks[i].y);
    rmax.x = std::max(rmax.x, landmarks[i].x);
    rmax.y = std::max(rmax.y, landmarks[i].y);
  }

  Box2 box = {rmin, rmax};
  cv::Point2f wh = box[1] - box[0];
  cv::Point2f shift(kHandBoxShiftVector.x * wh.x, kHandBoxShiftVector.y * wh.y);
  box[0] += shift;
  box[1] += shift;
  cv::Point2f center2 = (box[0] + box[1]) * 0.5f;
  wh = box[1] - box[0];
  cv::Point2f half = wh * (kHandBoxEnlargeFactor * 0.5f);
  box = {center2 - half, center2 + half};

  HandResult result;
  result.box = box;
  result.landmarks = landmarks;
  result.handedness = handedness;
  result.confidence = conf;
  return result;
}

// Runs the full stage-2 landmark model on `palm` and applies
// hand_conf_threshold -- factored out so Step 2 in main() can require this
// (not just the palm detector's own, looser confidence) before promoting a
// depth-guided candidate to a Track. Without this gate, a palm-level false
// positive (face, elbow, a fold of clothing -- the palm detector alone is
// only run at 0.5 confidence by default) would permanently squat a hand
// slot: it never draws (fails this stricter check down in main()'s display
// loop), but Step 1 keeps re-confirming its own palm-level detection every
// frame, so it never gets evicted either, silently blocking a genuinely
// different hand from ever being acquired.
std::optional<HandResult> runHandLandmarkStage(Ort::Session &hand_session, const char *const *hand_input_names,
                                                const char *const *hand_output_names, Ort::MemoryInfo &mem_info,
                                                const cv::Mat &bgrx, const PalmDetection &palm,
                                                float hand_conf_threshold)
{
  HandPreprocessResult pre = preprocessForHand(bgrx, palm);

  CV_Assert(pre.blob.isContinuous());
  std::vector<float> tensor_data(pre.blob.ptr<float>(0),
                                  pre.blob.ptr<float>(0) + static_cast<size_t>(kHandInputSize) * kHandInputSize * 3);

  std::array<int64_t, 4> tensor_shape = {1, kHandInputSize, kHandInputSize, 3};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info, tensor_data.data(), tensor_data.size(), tensor_shape.data(), tensor_shape.size());

  auto outputs = hand_session.Run(Ort::RunOptions{nullptr}, hand_input_names, &input_tensor, 1, hand_output_names, 3);
  const float *landmarks_raw = outputs[0].GetTensorData<float>();
  float conf = outputs[1].GetTensorData<float>()[0];
  float handedness = outputs[2].GetTensorData<float>()[0];

  return postprocessHand(landmarks_raw, conf, handedness, pre, hand_conf_threshold);
}

// Standard MediaPipe 21-point hand skeleton connections.
const std::array<std::pair<int, int>, 21> kHandConnections = {{
    {0, 1}, {1, 2}, {2, 3}, {3, 4},
    {0, 5}, {5, 6}, {6, 7}, {7, 8},
    {5, 9}, {9, 10}, {10, 11}, {11, 12},
    {9, 13}, {13, 14}, {14, 15}, {15, 16},
    {13, 17}, {17, 18}, {18, 19}, {19, 20},
    {0, 17}}};

// A tracked hand's last known tight palm box (full-frame coords). Driven
// purely off detectPalms() re-run on a crop around this box each frame --
// no identity/re-id, just nearest-box continuity (see main()'s loop).
struct Track
{
  cv::Rect box;
  int missed_frames = 0;
};

// Shifts a PalmDetection's box + all 7 landmarks by `offset` -- used to map
// a detection found on a crop (bgrx(cropRect)) back to full-frame
// coordinates, since detectPalms() treats whatever Mat it's given as "the
// whole image" (see its own comment above).
void offsetPalmDetection(PalmDetection &palm, cv::Point2f offset)
{
  palm.box += offset;
  for (cv::Point2f &p : palm.landmarks)
    p += offset;
}

float iou(const cv::Rect &a, const cv::Rect &b)
{
  cv::Rect inter = a & b;
  if (inter.area() <= 0)
    return 0.0f;
  return static_cast<float>(inter.area()) / static_cast<float>(a.area() + b.area() - inter.area());
}

// Center-enlarges `box` by `margin_factor`, with a minimum-size floor (so a
// tiny/tight box still yields a useful crop), clamped to frame bounds.
cv::Rect enlargeAndClamp(const cv::Rect &box, float margin_factor, int min_size, cv::Size frame_size)
{
  cv::Point2f center(box.x + box.width * 0.5f, box.y + box.height * 0.5f);
  float w = std::max(box.width * margin_factor, static_cast<float>(min_size));
  float h = std::max(box.height * margin_factor, static_cast<float>(min_size));
  cv::Rect r(static_cast<int>(center.x - w * 0.5f), static_cast<int>(center.y - h * 0.5f),
             static_cast<int>(w), static_cast<int>(h));
  return r & cv::Rect(0, 0, frame_size.width, frame_size.height);
}

// Finds candidate crop regions for (re)acquiring a hand via the depth
// sensor instead of scanning the full color frame. Repeatedly takes the
// globally nearest in-range depth pixel as a candidate peak (a hand reaching
// toward the camera is normally the closest thing in the scene at that
// location), sizes a crop around it from its actual depth (via
// `focal_length_px` and `kHandFootprintMeters` -- closer peaks get smaller,
// tighter crops; farther ones get larger), then masks out that crop's
// footprint before looking for the next peak. That masking is what lets two
// hands joined to one torso in the depth mask still surface as two separate
// candidates instead of one oversized bounding box -- and since crop size
// only depends on a peak's own depth (not blob extent), it can't balloon
// toward full-frame size the way a connected-component bounding box does
// when more of the body falls in range at close distance.
//
// `existing_tracks`' whole neighborhoods (same generous crop Step 1 uses to
// re-track them, see main()) are excluded from the search entirely before
// peak-picking starts, not merely checked after the fact: a single real
// hand can easily have several of its own depth peaks (spread fingers,
// wrist vs. palm can differ by well more than the flood-fill tolerance
// below), so reactively filtering only the exact peak found would still let
// an already-tracked hand's OTHER peaks resurface as spurious "new"
// candidates and eat the whole search budget before a genuinely different
// second hand's peak is ever reached.
// `depth_mm`: CV_32F, millimeters, 0 = invalid, already in COLOR image space.
std::vector<cv::Rect> findDepthGuidedCandidates(const cv::Mat &depth_mm,
                                                 const std::vector<cv::Rect> &existing_tracks,
                                                 float min_depth_m, float max_depth_m,
                                                 float focal_length_px, int max_candidates)
{
  cv::Mat mask;
  cv::inRange(depth_mm, min_depth_m * 1000.0f, max_depth_m * 1000.0f, mask);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  // Out-of-mask pixels pushed to +infinity so repeated minMaxLoc calls walk
  // through successive near peaks as earlier ones get masked out below.
  const float kInvalid = std::numeric_limits<float>::max();
  cv::Mat search(depth_mm.size(), CV_32FC1, cv::Scalar(kInvalid));
  depth_mm.copyTo(search, mask);

  for (const cv::Rect &t : existing_tracks)
  {
    cv::Rect exclude = enlargeAndClamp(t, kTrackingCropMarginFactor, kTrackingCropMinSize,
                                        cv::Size(depth_mm.cols, depth_mm.rows));
    search(exclude).setTo(kInvalid);
  }

  std::vector<cv::Rect> candidates;
  int max_iterations = max_candidates + 2;
  for (int iter = 0; iter < max_iterations && static_cast<int>(candidates.size()) < max_candidates; ++iter)
  {
    double min_val;
    cv::Point min_loc;
    cv::minMaxLoc(search, &min_val, nullptr, &min_loc, nullptr);
    if (min_val >= kInvalid)
      break; // mask exhausted -- no more in-range depth left to consider

    float depth_m = static_cast<float>(min_val) / 1000.0f;
    float crop_size_f = focal_length_px * kHandFootprintMeters / std::max(depth_m, 0.1f);
    int crop_size = std::clamp(static_cast<int>(crop_size_f), kDepthCandidateMinCropSize, kDepthCandidateMaxCropSize);

    cv::Rect peak_box(min_loc.x - crop_size / 2, min_loc.y - crop_size / 2, crop_size, crop_size);
    peak_box &= cv::Rect(0, 0, depth_mm.cols, depth_mm.rows);

    // Suppress this peak regardless of whether it's kept below, so the next
    // iteration is forced to look elsewhere. A fixed box alone isn't enough:
    // a large near-uniform-depth surface (most commonly a torso) is very
    // often the second-nearest peak after a hand, and nibbling it one small
    // box at a time never finishes before the candidate budget runs out --
    // reliably starving a genuinely separate, farther-away second hand
    // regardless of how far apart the two hands actually are. Flood-filling
    // from the peak within a tight, SEED-relative depth tolerance (fixed
    // range, not neighbor-to-neighbor -- otherwise it can "walk" along a
    // gradual arm/shoulder depth gradient right off the torso and swallow a
    // real hand too) consumes that whole surface patch in one iteration.
    search(peak_box).setTo(kInvalid);
    cv::floodFill(search, min_loc, cv::Scalar(kInvalid), nullptr, cv::Scalar(kFloodDepthToleranceMm),
                  cv::Scalar(kFloodDepthToleranceMm), 4 | cv::FLOODFILL_FIXED_RANGE);

    candidates.push_back(peak_box);
  }
  return candidates;
}

} // namespace

int main(int argc, char **argv)
{
  std::string palm_model_path;
  std::string hand_model_path;
  float palm_conf_threshold = 0.5f;
  float palm_iou_threshold = 0.3f;
  float hand_conf_threshold = 0.8f;
  float min_depth_m = 0.4f;
  float max_depth_m = 3.5f;
  int max_hands = 2;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    if (arg == "--palm-model" && i + 1 < argc)
      palm_model_path = argv[++i];
    else if (arg == "--hand-model" && i + 1 < argc)
      hand_model_path = argv[++i];
    else if (arg == "--palm-conf" && i + 1 < argc)
      palm_conf_threshold = std::stof(argv[++i]);
    else if (arg == "--palm-iou" && i + 1 < argc)
      palm_iou_threshold = std::stof(argv[++i]);
    else if (arg == "--hand-conf" && i + 1 < argc)
      hand_conf_threshold = std::stof(argv[++i]);
    else if (arg == "--min-depth" && i + 1 < argc)
      min_depth_m = std::stof(argv[++i]);
    else if (arg == "--max-depth" && i + 1 < argc)
      max_depth_m = std::stof(argv[++i]);
    else if (arg == "--max-hands" && i + 1 < argc)
      max_hands = std::stoi(argv[++i]);
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage();
      return 1;
    }
  }

  if (palm_model_path.empty() || hand_model_path.empty())
  {
    printUsage();
    return 1;
  }

  Ort::Session palm_session{nullptr};
  Ort::Session hand_session{nullptr};
  try
  {
    palm_session = onnx_common::createSession(palm_model_path);
    hand_session = onnx_common::createSession(hand_model_path);
  }
  catch (const Ort::Exception &e)
  {
    std::cerr << "Failed to load ONNX model(s): " << e.what() << "\n";
    return 1;
  }

  Ort::AllocatorWithDefaultOptions allocator;
  Ort::AllocatedStringPtr hand_input_name = hand_session.GetInputNameAllocated(0, allocator);
  Ort::AllocatedStringPtr hand_output_name0 = hand_session.GetOutputNameAllocated(0, allocator); // landmarks [1,63]
  Ort::AllocatedStringPtr hand_output_name1 = hand_session.GetOutputNameAllocated(1, allocator); // confidence [1,1]
  Ort::AllocatedStringPtr hand_output_name2 = hand_session.GetOutputNameAllocated(2, allocator); // handedness [1,1]
  const char *hand_input_names[] = {hand_input_name.get()};
  const char *hand_output_names[] = {hand_output_name0.get(), hand_output_name1.get(), hand_output_name2.get()};

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

  const std::string window_name = "ONNX Hand Tracking";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Depth-in-color-space (bigdepth) drives findDepthGuidedCandidates() below
  // -- undistorted/registered_frame are unused beyond being required
  // apply() out-params (same pattern as Protonect.cpp).
  libfreenect2::Registration registration(device->getIrCameraParams(), device->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered_frame(512, 424, 4), bigdepth(1920, 1082, 4);
  const float color_focal_length_px = device->getColorCameraParams().fx;
  std::vector<Track> tracks;

  while (cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) > 0)
  {
    libfreenect2::FrameMap frames;
    listener.waitForNewFrame(frames);

    libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    cv::Mat bgrx(color->height, color->width, CV_8UC4, color->data);

    registration.apply(color, depth, &undistorted, &registered_frame, true, &bigdepth);
    cv::Mat bigdepth_mat(bigdepth.height, bigdepth.width, CV_32FC1, bigdepth.data);
    // bigdepth has one blank row top and bottom (1082 vs color's 1080 rows).
    cv::Mat depth_color_space = bigdepth_mat.rowRange(1, 1 + bgrx.rows);

    std::vector<PalmDetection> palms;

    // Step 1: advance existing tracks -- crop tight around each track's last
    // known box and re-run palm detection on just that crop, so a distant
    // hand keeps getting full-detail treatment once found (see file header
    // comment). Evict a track once it's gone unseen for kMaxMissedFrames.
    for (auto it = tracks.begin(); it != tracks.end();)
    {
      cv::Rect crop_rect = enlargeAndClamp(it->box, kTrackingCropMarginFactor, kTrackingCropMinSize, bgrx.size());
      std::vector<PalmDetection> found = detectPalms(palm_session, bgrx(crop_rect), palm_conf_threshold, palm_iou_threshold);

      const PalmDetection *best = nullptr;
      for (const PalmDetection &p : found)
        if (!best || p.score > best->score)
          best = &p;

      if (best)
      {
        PalmDetection palm = *best;
        offsetPalmDetection(palm, cv::Point2f(static_cast<float>(crop_rect.x), static_cast<float>(crop_rect.y)));
        it->box = cv::Rect(palm.box);
        it->missed_frames = 0;
        palms.push_back(palm);
        ++it;
      }
      else
      {
        ++it->missed_frames;
        if (it->missed_frames > kMaxMissedFrames)
          it = tracks.erase(it);
        else
          ++it;
      }
    }

    // Step 2: while under max_hands, use the depth sensor to find candidate
    // regions for a NEW hand (rather than re-scanning the full color frame)
    // and try to acquire a track from each.
    if (static_cast<int>(tracks.size()) < max_hands)
    {
      std::vector<cv::Rect> existing_boxes;
      for (const Track &t : tracks)
        existing_boxes.push_back(t.box);

      int slots_left = max_hands - static_cast<int>(tracks.size());
      std::vector<cv::Rect> candidates =
          findDepthGuidedCandidates(depth_color_space, existing_boxes, min_depth_m, max_depth_m,
                                     color_focal_length_px, slots_left + kDepthCandidateSearchHeadroom);

      for (const cv::Rect &crop_rect : candidates)
      {
        if (static_cast<int>(tracks.size()) >= max_hands)
          break;
        std::vector<PalmDetection> found = detectPalms(palm_session, bgrx(crop_rect), palm_conf_threshold, palm_iou_threshold);
        for (PalmDetection &palm : found)
        {
          offsetPalmDetection(palm, cv::Point2f(static_cast<float>(crop_rect.x), static_cast<float>(crop_rect.y)));
          cv::Rect palm_box(palm.box);
          bool dup = std::any_of(tracks.begin(), tracks.end(), [&](const Track &t)
                                  { return iou(t.box, palm_box) > kTrackOverlapIouThreshold; });
          if (dup)
            continue;
          // Require the landmark stage to confirm this too (see
          // runHandLandmarkStage's comment) before committing a track --
          // otherwise a palm-only false positive can permanently squat a
          // hand slot without ever being visibly wrong.
          if (!runHandLandmarkStage(hand_session, hand_input_names, hand_output_names, mem_info, bgrx, palm,
                                     hand_conf_threshold))
            continue;
          tracks.push_back(Track{palm_box, 0});
          palms.push_back(palm);
          if (static_cast<int>(tracks.size()) >= max_hands)
            break;
        }
      }
    }

    cv::Mat display;
    cv::cvtColor(bgrx, display, cv::COLOR_BGRA2BGR);

    for (const PalmDetection &palm : palms)
    {
      HandPreprocessResult pre = preprocessForHand(bgrx, palm);

      // cv::Mat::convertTo() on a freshly-created (non-ROI) Mat is
      // guaranteed contiguous, but copy into a plain buffer rather than
      // relying on that -- matches the pattern used for the NCHW tensors in
      // onnx_common.cpp, and is one less thing to get wrong here.
      CV_Assert(pre.blob.isContinuous());
      std::vector<float> tensor_data(pre.blob.ptr<float>(0),
                                      pre.blob.ptr<float>(0) + static_cast<size_t>(kHandInputSize) * kHandInputSize * 3);

      std::array<int64_t, 4> tensor_shape = {1, kHandInputSize, kHandInputSize, 3};
      Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
          mem_info, tensor_data.data(), tensor_data.size(), tensor_shape.data(), tensor_shape.size());

      auto outputs = hand_session.Run(Ort::RunOptions{nullptr}, hand_input_names, &input_tensor, 1,
                                       hand_output_names, 3);
      const float *landmarks_raw = outputs[0].GetTensorData<float>();
      float conf = outputs[1].GetTensorData<float>()[0];
      float handedness = outputs[2].GetTensorData<float>()[0];

      std::optional<HandResult> hand = postprocessHand(landmarks_raw, conf, handedness, pre, hand_conf_threshold);
      if (!hand)
        continue;

      cv::Rect box(cv::Point(static_cast<int>(hand->box[0].x), static_cast<int>(hand->box[0].y)),
                   cv::Point(static_cast<int>(hand->box[1].x), static_cast<int>(hand->box[1].y)));
      box &= cv::Rect(0, 0, display.cols, display.rows);
      cv::rectangle(display, box, cv::Scalar(0, 200, 255), 2);

      char text[64];
      std::snprintf(text, sizeof(text), "%s %.2f", hand->handedness < 0.5f ? "Left" : "Right", hand->confidence);
      cv::putText(display, text, cv::Point(box.x, std::max(0, box.y - 6)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 200, 255), 1);

      for (const auto &edge : kHandConnections)
      {
        cv::Point p1(static_cast<int>(hand->landmarks[edge.first].x), static_cast<int>(hand->landmarks[edge.first].y));
        cv::Point p2(static_cast<int>(hand->landmarks[edge.second].x), static_cast<int>(hand->landmarks[edge.second].y));
        cv::line(display, p1, p2, cv::Scalar(0, 255, 0), 2);
      }
      for (const cv::Point3f &p : hand->landmarks)
        cv::circle(display, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 3, cv::Scalar(0, 0, 255), cv::FILLED);
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
