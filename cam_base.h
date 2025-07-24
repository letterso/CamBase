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

#ifndef CAM_BASE_H
#define CAM_BASE_H

#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

// Uncomment the following line to use the OpenCV-based undistortion implementations
// for all supported camera models (Radtan, Equi).
// #define USE_OPENCV_IMPL


/**
 * @brief Base pinhole camera model class
 *
 * This is the base class for all our camera models.
 * All these models are pinhole cameras, thus just have standard reprojection logic.
 * See each derived class for detailed examples of each model.
 *
 * We have three function types for distortion and undistortion:
 * - `_d`: Main implementation using `double` and `Eigen::Vector2d`. This is the one to override for highest precision.
 * - `_f`: Wrapper that casts to and from `float` / `Eigen::Vector2f`.
 * - `_cv`: Wrapper that casts to and from `cv::Point2f`.
 */
class CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   * @param camera_k Camera intrinsics vector [fx, fy, cx, cy]
   * @param camera_d Camera distortion vector
   */
  CamBase(int width, int height, const std::vector<double> &camera_k, const std::vector<double> &camera_d) :
    width_(width), height_(height), camera_k_(camera_k), camera_d_(camera_d){}

  virtual ~CamBase() = default;

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  virtual Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) = 0;

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) {
    return undistort_d(uv_dist.cast<double>()).cast<float>();
  }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  cv::Point2f undistort_cv(const cv::Point2f &uv_dist) {
    Eigen::Vector2d eigen_pt(uv_dist.x, uv_dist.y);
    Eigen::Vector2d eigen_undistorted = undistort_d(eigen_pt);
    return cv::Point2f(static_cast<float>(eigen_undistorted.x()), static_cast<float>(eigen_undistorted.y()));
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  virtual Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) = 0;

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) {
    return distort_d(uv_norm.cast<double>()).cast<float>();
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  cv::Point2f distort_cv(const cv::Point2f &uv_norm) {
    Eigen::Vector2d eigen_pt(uv_norm.x, uv_norm.y);
    Eigen::Vector2d eigen_distorted = distort_d(eigen_pt);
    return cv::Point2f(static_cast<float>(eigen_distorted.x()), static_cast<float>(eigen_distorted.y()));
  }

  /// Gets the camera matrix by const reference
  const std::vector<double>& get_K() const noexcept { return camera_k_; }

  /// Gets the camera distortion by const reference
  const std::vector<double>& get_D() const noexcept { return camera_d_; }

  /// Gets the width of the camera images
  int w() const noexcept { return width_; }

  /// Gets the height of the camera images
  int h() const noexcept { return height_; }

protected:
  // Cannot construct the base camera class, needs a distortion model
  CamBase() = default;

  /// Width of the camera (raw pixels)
  int width_;

  /// Height of the camera (raw pixels)
  int height_;

  /// Camera intrinsics in OpenCV format [fx, fy, cx, cy]
  std::vector<double> camera_k_;

  /// Camera distortion in OpenCV format
  std::vector<double> camera_d_;
};

#endif /* CAM_BASE_H */
