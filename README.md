# 相机模型通用库

参考[open_vins](https://github.com/rpng/open_vins/tree/master/ov_core/src/cam)实现的轻量级的纯头文件C++库，用于处理针孔和鱼眼相机镜头畸变。它提供了在**畸变像素坐标**和**归一化相机坐标**之间进行转换的功能，具有高精度和高性能的特点。

## 主要特性

- **高精度计算**: 所有内部计算均使用 `double` 类型，以确保最高的数值精度。
- **高性能**: 核心算法为纯C++实现，避免了不必要的开销，速度更快（当前迭代次数和opencv默认一致，所以精度一样，如需要提高精度，增加迭代次数即可，当处理耗时会增加）。
- **实现可切换**: 可以通过一个宏在内置实现和OpenCV实现之间轻松切换。
- **智能编译**: **OpenCV是可选的**。如果系统中没有安装OpenCV，项目依然可以成功编译，并自动使用内置实现。
- **支持模型**:
  - `cam_radtan.h`: **Radtan模型** (Brown-Conrady)，兼容OpenCV和Kalibr的 `pinhole-radtan`。
  - `cam_equi.h`: **鱼眼模型** (Equidistant)，兼容OpenCV和Kalibr的 `pinhole-equi`。

## 环境要求

- **C++17 编译器**: 例如 GCC 8+ 或 Clang 6+。
- **CMake**: 3.10 或更高版本。
- **Eigen3**: 必需。
- **OpenCV**: 可选，用于启用备用实现。

## 快速上手

### 包含头文件

只需要包含 `cam_radtan.h` 或 `cam_equi.h`。

```cpp
#include "cam_radtan.h" // Radtan模型
// 或
#include "cam_equi.h"   // 鱼眼模型
```

### 选择实现方式 (可选)

默认情况下，库使用内置的高性能迭代算法。如果希望改用OpenCV的实现，只需打开 `cam_base.h` 文件，并取消对以下宏的注释：

```cpp
// 在 cam_base.h 文件顶部
// #define USE_OPENCV_IMPL
// 取消注释后变为:
#define USE_OPENCV_IMPL
```
这个开关将同时对 `CamRadtan` 和 `CamEqui` 生效。如果CMake在编译时未找到OpenCV，即使此宏被定义，编译器也会安全地忽略它并使用内置实现。

### 定义相机参数

准备相机的内参和畸变系数。

- **内参 `K`** (`std::vector<double>`): `[fx, fy, cx, cy]`
- **畸变系数 `D`** (`std::vector<double>`):
  - `CamRadtan`: `[k1, k2, p1, p2, k3]` (k3可选)
  - `CamEqui`: `[k1, k2, k3, k4]`

### 创建相机实例

```cpp
// 图像尺寸
int width = 1280;
int height = 720;

// Radtan相机示例
std::vector<double> K = {458.654, 457.296, 639.5, 359.5};
std::vector<double> D = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
CamRadtan camera(width, height, K, D);
```

### 转换坐标

使用 `undistort` (去畸变) 和 `distort` (畸变) 函数进行坐标转换。库提供了三种函数变体以适应不同的数据类型：

- `undistort_d(...)` / `distort_d(...)`: **(核心)** 使用 `double` 和 `Eigen::Vector2d`，精度最高。
- `undistort_f(...)` / `distort_f(...)`: 使用 `float` 和 `Eigen::Vector2f` 的包装器。
- `undistort_cv(...)` / `distort_cv(...)`: 使用 `cv::Point2f` 的包装器。

**示例：去畸变 (像素坐标 -> 归一化坐标)**

```cpp
#include <iostream>
#include <Eigen/Eigen>

// 图像上的一个畸变点
Eigen::Vector2d distorted_point(800.0, 450.0);

// 将其去畸变为归一化坐标
Eigen::Vector2d normalized_point = camera.undistort_d(distorted_point);

std::cout << "畸变点: (" << distorted_point.x() << ", " << distorted_point.y() << ")" << std::endl;
std::cout << "归一化点: (" << normalized_point.x() << ", " << normalized_point.y() << ")" << std::endl;
```

## 编译与测试

本项目使用 `CMake` 进行构建。

- Ubuntu/Debian 安装依赖

  ```bash
  # 必需的
  sudo apt-get update
  sudo apt-get install build-essential cmake libeigen3-dev

  # 可选的 (用于启用OpenCV实现)
  sudo apt-get install libopencv-dev
  ```

- 编译

  ```bash
  mkdir build && cd build
  cmake ..
  make -j
  ```

### 运行测试

直接运行可执行文件即可看到正确性和速度测试的结果。

```bash
./camera_test
```

测试程序会分别评估 **Radtan** 和 **Equi** 两种模型。它会打印出往返误差（用于验证正确性）和处理10万个点所需的时间（用于评估性能）。

要对比内置实现和OpenCV实现的性能，您只需：
1.  修改 `cam_base.h` 中的 `#define USE_OPENCV_IMPL` 宏（注释或取消注释）。
2.  重新编译 (`make`) 并再次运行测试。

测试结果

- 内置迭代算法

  ```bash
  ============================================================
    Radtan Model Test
  ============================================================
  Radtan (Internal) Correctness Test:
    Original Point:      (0.3, -0.2)
    Distorted Pixel:     (772.204, 271.3)
    Final Undistorted:   (0.3, -0.2)
    Round-trip Error:    2.271766e-08
    [SUCCESS] Low error.

  Radtan (Internal) Speed Test (100000 points):
    distort() time:   0.295156 ms
    undistort() time: 2.670738 ms

  ============================================================
    Equi (Fisheye) Model Test
  ============================================================
  Equi (Internal) Correctness Test:
    Original Point:      (0.300000, -0.200000)
    Distorted Pixel:     (732.269954, 298.525131)
    Final Undistorted:   (0.300000, -0.200000)
    Round-trip Error:    1.494683e-16
    [SUCCESS] Low error.

  Equi (Internal) Speed Test (100000 points):
    distort() time:   1.358781 ms
    undistort() time: 7.232002 ms
  ```

- opencv

  ```bash
  ============================================================
    Radtan Model Test
  ============================================================
  Radtan (OpenCV) Correctness Test:
    Original Point:      (0.3, -0.2)
    Distorted Pixel:     (772.204, 271.3)
    Final Undistorted:   (0.3, -0.2)
    Round-trip Error:    2.271766e-08
    [SUCCESS] Low error.

  Radtan (OpenCV) Speed Test (100000 points):
    distort() time:   0.297319 ms
    undistort() time: 54.243528 ms

  ============================================================
    Equi (Fisheye) Model Test
  ============================================================
  Equi (OpenCV) Correctness Test:
    Original Point:      (0.300000, -0.200000)
    Distorted Pixel:     (732.269954, 298.525131)
    Final Undistorted:   (0.300000, -0.200000)
    Round-trip Error:    1.494683e-16
    [SUCCESS] Low error.

  Equi (OpenCV) Speed Test (100000 points):
    distort() time:   1.251700 ms
    undistort() time: 37.757270 ms
  ```