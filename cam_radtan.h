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
 *
 * This class implements the Brown-Conrady distortion model, commonly known as the "radtan" model.
 * It is equivalent to Kalibr's `pinhole-radtan` model and OpenCV's standard distortion model.
 *
 * The distortion function is:
 * \f{align*}{ 
 * x_d &= x_n (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 x_n y_n + p_2(r^2 + 2 x_n^2) \\
 * y_d &= y_n (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n
 * \f}
 * where \f$ r^2 = x_n^2 + y_n^2 \f$.
 *
 * The camera intrinsics are \f$ [f_x, f_y, c_x, c_y] \f$ and distortion coefficients are \f$ [k_1, k_2, p_1, p_2, k_3] \f$.
 */
class CamRadtan : public CamBase {

public:
  /**
   * @brief Constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   * @param camera_k Intrinsics of the camera [fx, fy, cx, cy]
   * @param camera_d Distortion coefficients [k1, k2, p1, p2, k3] (k3 is optional)
   */
  CamRadtan(int width, int height, const std::vector<double> &camera_k, const std::vector<double> &camera_d)
    : CamBase(width, height, camera_k, camera_d) {
    fx_ = camera_k[0];
    fy_ = camera_k[1];
    cx_ = camera_k[2];
    cy_ = camera_k[3];

    k1_ = camera_d[0];
    k2_ = camera_d[1];
    p1_ = camera_d[2];
    p2_ = camera_d[3];
    k3_ = camera_d.size() > 4 ? camera_d[4] : 0.0;

#ifdef USE_OPENCV_IMPL
    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = camera_k_[0];
    tempK(0, 1) = 0;
    tempK(0, 2) = camera_k_[2];
    tempK(1, 0) = 0;
    tempK(1, 1) = camera_k_[1];
    tempK(1, 2) = camera_k_[3];
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV = tempK;

    // Distortion parameters
    camera_d_OPENCV = cv::Mat(1, camera_d.size(), CV_64F);
    for(size_t i=0; i<camera_d.size(); ++i) camera_d_OPENCV.at<double>(i) = camera_d[i];
#endif
  }

  ~CamRadtan() override = default;

  /**
   * @brief Undistorts a raw pixel coordinate into normalized camera coordinates.
   * This function iteratively inverts the distortion model.
   * @param uv_dist Raw (distorted) uv coordinate
   * @return Normalized (undistorted) 2D coordinate
   */
  Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) override {
#ifdef USE_OPENCV_IMPL
    // Convert to opencv format
    cv::Mat mat(1, 1, CV_64FC2);
    mat.at<cv::Vec2d>(0,0) = cv::Vec2d(uv_dist(0), uv_dist(1));

    // Undistort it
    cv::Mat mat_undistorted(1, 1, CV_64FC2);
    cv::undistortPoints(mat, mat_undistorted, camera_k_OPENCV, camera_d_OPENCV);

    // Construct our return vector
    Eigen::Vector2d pt_out;
    pt_out(0) = mat_undistorted.at<cv::Vec2d>(0,0)[0];
    pt_out(1) = mat_undistorted.at<cv::Vec2d>(0,0)[1];
    return pt_out;
#else
    // Convert from pixel to camera normalized coordinates (initial guess)
    Eigen::Vector2d pt_normalized;
    pt_normalized.x() = (uv_dist.x() - cx_) / fx_;
    pt_normalized.y() = (uv_dist.y() - cy_) / fy_;

    // Iteratively refine the undistorted point
    Eigen::Vector2d pt_undistorted = pt_normalized; // Start with initial guess
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
   * @param uv_norm Normalized (undistorted) 2D coordinate
   * @return Raw (distorted) uv coordinate
   */
  Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) override {
    double xn = uv_norm.x();
    double yn = uv_norm.y();

    // Apply distortion
    double r2 = xn * xn + yn * yn;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double radial_distortion = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;

    double x_distorted = xn * radial_distortion + (2.0 * p1_ * xn * yn + p2_ * (r2 + 2.0 * xn * xn));
    double y_distorted = yn * radial_distortion + (p1_ * (r2 + 2.0 * yn * yn) + 2.0 * p2_ * xn * yn);

    // Project to pixel coordinates
    Eigen::Vector2d uv_dist;
    uv_dist.x() = fx_ * x_distorted + cx_;
    uv_dist.y() = fy_ * y_distorted + cy_;
    return uv_dist;
  }

private:
  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_;
  double p1_, p2_;

#ifdef USE_OPENCV_IMPL
  cv::Matx33d camera_k_OPENCV;
  cv::Mat camera_d_OPENCV;
#endif
};

#endif /* CAM_RADTAN_H */