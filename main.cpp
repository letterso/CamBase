#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <string_view>

#include "cam_radtan.h"
#include "cam_equi.h"

// Define a macro to check which implementation is active
#if defined(USE_OPENCV_IMPL) && defined(HAVE_OPENCV)
    #define ACTIVE_IMPL_NAME "(OpenCV)"
#else
    #define ACTIVE_IMPL_NAME "(Internal)"
#endif

// Helper to print results
void print_test_header(std::string_view title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(60, '=') << std::endl;
}

// Correctness Test: Distort a point and then undistort it, and check the error.
void run_correctness_test(CamBase& camera, std::string_view model_name) {
    Eigen::Vector2d original_pt_norm(0.3, -0.2);
    Eigen::Vector2d distorted_pt = camera.distort_d(original_pt_norm);
    Eigen::Vector2d final_pt_norm = camera.undistort_d(distorted_pt);
    double error = (original_pt_norm - final_pt_norm).norm();

    std::cout << model_name << " " << ACTIVE_IMPL_NAME << " Correctness Test:\n";
    std::cout << "  Original Point:      (" << original_pt_norm.x() << ", " << original_pt_norm.y() << ")\n";
    std::cout << "  Distorted Pixel:     (" << distorted_pt.x() << ", " << distorted_pt.y() << ")\n";
    std::cout << "  Final Undistorted:   (" << final_pt_norm.x() << ", " << final_pt_norm.y() << ")\n";
    std::cout << "  Round-trip Error:    " << std::scientific << error << std::fixed << "\n";
    
    if (error > 1e-6) {
        std::cout << "  [WARNING] High error detected!\n";
    } else {
        std::cout << "  [SUCCESS] Low error.\n";
    }
}

// Speed Test: Distort and undistort a large number of points and measure time.
void run_speed_test(CamBase& camera, std::string_view model_name) {
    const int NUM_POINTS = 100000;
    std::vector<Eigen::Vector2d> points;
    points.reserve(NUM_POINTS);

    std::mt19937 gen(42);
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    for (int i = 0; i < NUM_POINTS; ++i) {
        points.emplace_back(dis(gen), dis(gen));
    }

    auto start_distort = std::chrono::high_resolution_clock::now();
    for (const auto& p : points) {
        volatile Eigen::Vector2d res = camera.distort_d(p);
    }
    auto end_distort = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> distort_ms = end_distort - start_distort;

    std::uniform_real_distribution<> dis_u(0, camera.w());
    std::uniform_real_distribution<> dis_v(0, camera.h());
    for (int i = 0; i < NUM_POINTS; ++i) {
        points[i] = Eigen::Vector2d(dis_u(gen), dis_v(gen));
    }

    auto start_undistort = std::chrono::high_resolution_clock::now();
    for (const auto& p : points) {
        volatile Eigen::Vector2d res = camera.undistort_d(p);
    }
    auto end_undistort = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> undistort_ms = end_undistort - start_undistort;

    std::cout << "\n" << model_name << " " << ACTIVE_IMPL_NAME << " Speed Test (" << NUM_POINTS << " points):\n";
    std::cout << "  distort() time:   " << distort_ms.count() << " ms\n";
    std::cout << "  undistort() time: " << undistort_ms.count() << " ms\n";
}

#if defined(HAVE_OPENCV)
// Image Undistortion Test
void run_image_undistort_test(CamBase& camera, std::string_view model_name) {    
    // Create a test image with a grid pattern
    int width = camera.w();
    int height = camera.h();
    cv::Mat test_image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // Draw a grid pattern on the image
    for (int i = 0; i < height; i += 50) {
        cv::line(test_image, cv::Point(0, i), cv::Point(width, i), cv::Scalar(0, 255, 0), 1);
    }
    for (int i = 0; i < width; i += 50) {
        cv::line(test_image, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(0, 255, 0), 1);
    }
    
    // Draw some circles to better visualize distortion
    for (int i = 100; i < width; i += 200) {
        for (int j = 100; j < height; j += 200) {
            cv::circle(test_image, cv::Point(i, j), 30, cv::Scalar(255, 0, 0), 2);
        }
    }
    
    // Distort the image to simulate a real distorted image
    // For this test, we'll just use the undistort function directly on our grid image
    std::cout << "  Creating test image with grid pattern...\n";
    std::cout << "  Running image undistortion...\n";
    
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat undistorted_image = camera.undistort_image(test_image);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    
    std::cout << "  Image undistortion time: " << elapsed.count() << " ms\n";
    std::cout << "  Input image size: " << test_image.cols << "x" << test_image.rows << "\n";
    std::cout << "  Output image size: " << undistorted_image.cols << "x" << undistorted_image.rows << "\n";
    
    // Save images for visual inspection (optional)
    // cv::imwrite("test_image_distorted.png", test_image);
    // cv::imwrite("test_image_undistorted.png", undistorted_image);
    
    std::cout << "  [SUCCESS] Image undistortion completed.\n";
}
#endif

int main() {
    int width = 1280;
    int height = 720;

    {
        print_test_header("Radtan Model Test");
        std::vector<double> K_radtan = {458.6, 457.3, 639.5, 359.5};
        std::vector<double> D_radtan = {-0.283, 0.074, 0.0002, 0.00017};
        CamRadtan radtan_camera(width, height, K_radtan, D_radtan);
        run_correctness_test(radtan_camera, "Radtan");
        run_speed_test(radtan_camera, "Radtan");
#if defined(HAVE_OPENCV)
        run_image_undistort_test(radtan_camera, "Radtan");
#endif
    }

    {
        print_test_header("Equi (Fisheye) Model Test");
        std::vector<double> K_equi = {320.1, 319.9, 640.0, 360.0};
        std::vector<double> D_equi = {0.01, -0.005, 0.002, -0.001};
        CamEqui equi_camera(width, height, K_equi, D_equi);
        run_correctness_test(equi_camera, "Equi");
        run_speed_test(equi_camera, "Equi");
#if defined(HAVE_OPENCV)
        run_image_undistort_test(equi_camera, "Equi");
#endif
    }

    return 0;
}
