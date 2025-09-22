/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CAM_RADTAN_H
#define CAM_RADTAN_H

#include "cam_base.h"

/**
 * @brief Radial-tangential / Brown–Conrady model pinhole camera model class
 */
class CamRadtan : public CamBase {

public:
  /**
   * @brief Constructor
   */
  CamRadtan(int width, int height, const std::vector<double> &camera_k, const std::vector<double> &camera_d)
    : CamBase(width, height, camera_k, camera_d),
      fx_(camera_k[0]), fy_(camera_k[1]), cx_(camera_k[2]), cy_(camera_k[3]),
      k1_(camera_d[0]), k2_(camera_d[1]), p1_(camera_d[2]), p2_(camera_d[3]),
      k3_(camera_d.size() > 4 ? camera_d[4] : 0.0)
  {
#if defined(HAVE_OPENCV)
    camera_k_OPENCV(0, 0) = camera_k_[0]; camera_k_OPENCV(0, 1) = 0;   camera_k_OPENCV(0, 2) = camera_k_[2];
    camera_k_OPENCV(1, 0) = 0;   camera_k_OPENCV(1, 1) = camera_k_[1]; camera_k_OPENCV(1, 2) = camera_k_[3];
    camera_k_OPENCV(2, 0) = 0;   camera_k_OPENCV(2, 1) = 0;   camera_k_OPENCV(2, 2) = 1;
    camera_d_OPENCV = cv::Mat(1, camera_d.size(), CV_64F, const_cast<double*>(camera_d.data()));
    
    // Precompute undistortion maps for better performance
    cv::initUndistortRectifyMap(camera_k_OPENCV, camera_d_OPENCV, cv::Mat(), 
                                camera_k_OPENCV, cv::Size(width_, height_), 
                                CV_32FC1, map1_, map2_);
#endif
  }

  ~CamRadtan() override = default;

  /**
   * @brief Undistorts a raw pixel coordinate into normalized camera coordinates.
   */
  Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) override {
#if defined(USE_OPENCV_IMPL) && defined(HAVE_OPENCV)
    cv::Mat mat(1, 1, CV_64FC2, const_cast<double*>(uv_dist.data()));
    cv::Mat mat_undistorted(1, 1, CV_64FC2);
    cv::undistortPoints(mat, mat_undistorted, camera_k_OPENCV, camera_d_OPENCV);
    return Eigen::Vector2d(mat_undistorted.at<cv::Vec2d>(0,0)[0], mat_undistorted.at<cv::Vec2d>(0,0)[1]);
#else
    Eigen::Vector2d pt_normalized;
    pt_normalized.x() = (uv_dist.x() - cx_) / fx_;
    pt_normalized.y() = (uv_dist.y() - cy_) / fy_;

    Eigen::Vector2d pt_undistorted = pt_normalized;
    for (int i = 0; i < 5; i++) {
        double r2 = pt_undistorted.squaredNorm();
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double radial_distortion = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
        double dx = 2.0 * p1_ * pt_undistorted.x() * pt_undistorted.y() + p2_ * (r2 + 2.0 * pt_undistorted.x() * pt_undistorted.x());
        double dy = p1_ * (r2 + 2.0 * pt_undistorted.y() * pt_undistorted.y()) + 2.0 * p2_ * pt_undistorted.x() * pt_undistorted.y();
        pt_undistorted.x() = (pt_normalized.x() - dx) / radial_distortion;
        pt_undistorted.y() = (pt_normalized.y() - dy) / radial_distortion;
    }
    return pt_undistorted;
#endif
  }

  /**
   * @brief Distorts a normalized camera coordinate to a raw pixel coordinate.
   */
  Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) override {
    double r2 = uv_norm.squaredNorm();
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double radial_distortion = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
    double x_distorted = uv_norm.x() * radial_distortion + (2.0 * p1_ * uv_norm.x() * uv_norm.y() + p2_ * (r2 + 2.0 * uv_norm.x() * uv_norm.x()));
    double y_distorted = uv_norm.y() * radial_distortion + (p1_ * (r2 + 2.0 * uv_norm.y() * uv_norm.y()) + 2.0 * p2_ * uv_norm.x() * uv_norm.y());
    return Eigen::Vector2d(fx_ * x_distorted + cx_, fy_ * y_distorted + cy_);
  }

#if defined(HAVE_OPENCV)
  /**
   * @brief Undistorts an entire image using OpenCV
   * @param distorted_image Input distorted image
   * @return Undistorted image
   */
  cv::Mat undistort_image(const cv::Mat& distorted_image) override {
    // Use precomputed undistortion maps for better performance
    cv::Mat undistorted_image;
    cv::remap(distorted_image, undistorted_image, map1_, map2_, cv::INTER_LINEAR);
    return undistorted_image;
  }
#endif

private:
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_, p1_, p2_;

#if defined(HAVE_OPENCV)
  cv::Matx33d camera_k_OPENCV;
  cv::Mat camera_d_OPENCV;
  cv::Mat map1_, map2_;  // Precomputed undistortion maps
#endif
};

#endif /* CAM_RADTAN_H */