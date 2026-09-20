/**
 * DGVI - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

/**
 * @file Frame.cpp
 * @brief Source file for the CameraBase class.
 * @author Stefan Leutenegger
 */

#include <dgvi/Frame.hpp>
#include <dgvi/internal/Network.hpp>

/// \brief dgvi Main namespace of this package.
namespace dgvi {

#ifdef DGVI_USE_NN
namespace {
constexpr float kCnnSkyThreshold = 3.5f;
constexpr float kCnnPersonThreshold = 53.5f;

torch::Tensor runSegmentation(
    const cv::Mat& image, const std::shared_ptr<Network>& network, int sizeU, int sizeV,
    int numThreads, int& previousNumThreads) {
  DGVI_ASSERT_TRUE(Frame::Exception, network, "network not set");
#ifndef DGVI_USE_GPU
  // make 100% sure multithreading on
  previousNumThreads = torch::get_num_threads();
  torch::set_num_threads(numThreads);
#else
  (void)numThreads;
  previousNumThreads = 0;
#endif
  // convert image
  cv::Mat img2;
  cv::resize(image, img2, cv::Size(sizeU, sizeV));
  cv::cvtColor(img2, img2, cv::COLOR_GRAY2RGB);
  cv::Mat img_float;
  img2.convertTo(img_float, CV_32F, 1.0 / 255);
  auto tensor_image = torch::from_blob(img_float.data,
                                       {img_float.rows, img_float.cols, img_float.channels()},
                                       torch::kFloat32);
  tensor_image = tensor_image.permute({2, 0, 1});
  std::vector<double> norm_mean = {0.485, 0.456, 0.406};
  std::vector<double> norm_std = {0.229, 0.224, 0.225};
  tensor_image = torch::data::transforms::Normalize<>(norm_mean, norm_std)(tensor_image);
  tensor_image.unsqueeze_(0);
#ifdef DGVI_USE_GPU
#ifdef DGVI_USE_MPS
  tensor_image = tensor_image.to(torch::kMPS);
#else
  tensor_image = tensor_image.to(torch::kCUDA);
#endif
#endif

  // forward pass
  auto outputs = network->forward({tensor_image}).toTuple();

  // convert output
#ifdef DGVI_USE_GPU
  torch::Tensor out1 = outputs->elements()[0].toTensor().cpu();
#else
  torch::Tensor out1 = outputs->elements()[0].toTensor();
#endif
  //torch::Tensor out2 = torch::softmax(out1, 1).squeeze(0);
  return out1.squeeze(0);
}

void restoreSegmentationThreads(int previousNumThreads) {
#ifndef DGVI_USE_GPU
  torch::set_num_threads(previousNumThreads);
#else
  (void)previousNumThreads;
#endif
}
}  // namespace

int Frame::computeClassifications(int sizeU, int sizeV, int numThreads) {
  int previousNumThreads = 0;
  torch::Tensor out2 = runSegmentation(image_, network_, sizeU, sizeV, numThreads,
                                       previousNumThreads);
  const double scaleU = double(sizeU)/double(image_.cols);
  const double scaleV = double(sizeV)/double(image_.rows);
  auto logits = out2.accessor<float, 3>();

  // cnn classification
  // used by S-CNN: [7, 8, 11, 12, 13, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33]
  // https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
  // name                     id    trainId   category            catId     hasInstances   eval ign.
  // 'unlabeled'            ,  0 ,      255 , 'void'            , 0       , False        , True
  // 'ego vehicle'          ,  1 ,      255 , 'void'            , 0       , False        , True
  // 'rectification border' ,  2 ,      255 , 'void'            , 0       , False        , True
  // 'out of roi'           ,  3 ,      255 , 'void'            , 0       , False        , True
  // 'static'               ,  4 ,      255 , 'void'            , 0       , False        , True
  // 'dynamic'              ,  5 ,      255 , 'void'            , 0       , False        , True
  // 'ground'               ,  6 ,      255 , 'void'            , 0       , False        , True
  // 'road'                 ,  7 ,        0 , 'flat'            , 1       , False        , False
  // 'sidewalk'             ,  8 ,        1 , 'flat'            , 1       , False        , False
  // 'parking'              ,  9 ,      255 , 'flat'            , 1       , False        , True
  // 'rail track'           , 10 ,      255 , 'flat'            , 1       , False        , True
  // 'building'             , 11 ,        2 , 'construction'    , 2       , False        , False
  // 'wall'                 , 12 ,        3 , 'construction'    , 2       , False        , False
  // 'fence'                , 13 ,        4 , 'construction'    , 2       , False        , False
  // 'guard rail'           , 14 ,      255 , 'construction'    , 2       , False        , True
  // 'bridge'               , 15 ,      255 , 'construction'    , 2       , False        , True
  // 'tunnel'               , 16 ,      255 , 'construction'    , 2       , False        , True
  // 'pole'                 , 17 ,        5 , 'object'          , 3       , False        , False
  // 'polegroup'            , 18 ,      255 , 'object'          , 3       , False        , True
  // 'traffic light'        , 19 ,        6 , 'object'          , 3       , False        , False
  // 'traffic sign'         , 20 ,        7 , 'object'          , 3       , False        , False
  // 'vegetation'           , 21 ,        8 , 'nature'          , 4       , False        , False
  // 'terrain'              , 22 ,        9 , 'nature'          , 4       , False        , False
  // 'sky'                  , 23 ,       10 , 'sky'             , 5       , False        , False
  // 'person'               , 24 ,       11 , 'human'           , 6       , True         , False
  // 'rider'                , 25 ,       12 , 'human'           , 6       , True         , False
  // 'car'                  , 26 ,       13 , 'vehicle'         , 7       , True         , False
  // 'truck'                , 27 ,       14 , 'vehicle'         , 7       , True         , False
  // 'bus'                  , 28 ,       15 , 'vehicle'         , 7       , True         , False
  // 'caravan'              , 29 ,      255 , 'vehicle'         , 7       , True         , True
  // 'trailer'              , 30 ,      255 , 'vehicle'         , 7       , True         , True
  // 'train'                , 31 ,       16 , 'vehicle'         , 7       , True         , False
  // 'motorcycle'           , 32 ,       17 , 'vehicle'         , 7       , True         , False
  // 'bicycle'              , 33 ,       18 , 'vehicle'         , 7       , True         , False
  // 'license plate'        , -1 ,       -1 , 'vehicle'         , 7       , False        , True
  classifications_ = cv::Mat(keypoints_.size(), 19, CV_32FC1);
  for(size_t k=0; k<keypoints_.size(); ++k) {
    int u = std::round(scaleU*(keypoints_[k].pt.x+0.5)-0.5);
    int v = std::round(scaleV*(keypoints_[k].pt.y+0.5)-0.5);
    if(u < 0) u = 0;
    else if(u >= sizeU) u = sizeU - 1;
    if(v < 0) v = 0;
    else if(v >= sizeV) v = sizeV - 1;
    for(int c=0; c<19; ++c) {
      classifications_.at<float>(k,c) = logits[c][v][u];
    }
  }
  isClassified_=true;

  restoreSegmentationThreads(previousNumThreads);
  return int(keypoints_.size());
}

int Frame::computeDetectionMask(cv::Mat & mask, int sizeU, int sizeV, int numThreads) {
  int previousNumThreads = 0;
  torch::Tensor out2 = runSegmentation(image_, network_, sizeU, sizeV, numThreads,
                                       previousNumThreads);
  auto logits = out2.accessor<float, 3>();

  cv::Mat semanticMask(sizeV, sizeU, CV_8UC1, cv::Scalar(255));
  for(int v = 0; v < sizeV; ++v) {
    for(int u = 0; u < sizeU; ++u) {
      if(logits[10][v][u] > kCnnSkyThreshold ||
         logits[11][v][u] > kCnnPersonThreshold) {
        semanticMask.at<uchar>(v, u) = 0;
      }
    }
  }
  cv::resize(semanticMask, mask, image_.size(), 0, 0, cv::INTER_NEAREST);

  restoreSegmentationThreads(previousNumThreads);
  return cv::countNonZero(mask);
}
#else
int Frame::computeClassifications(int /*sizeU*/, int /*sizeV*/, int /*numThreads*/) {
  DGVI_THROW(Exception, "trying to remove points with CNN, but CNN support not enabled.")
  return int(keypoints_.size());
}

int Frame::computeDetectionMask(cv::Mat & /*mask*/, int /*sizeU*/, int /*sizeV*/,
                                int /*numThreads*/) {
  DGVI_THROW(Exception, "trying to compute CNN mask, but CNN support not enabled.")
  return 0;
}
#endif
}  // namespace dgvi
