# 相机畸变模型库

这是一个轻量级的纯头文件C++库，用于处理常见的相机镜头畸变。它提供了在**畸变像素坐标**和**归一化相机坐标**之间进行转换的功能，具有高精度和高性能的特点。

## 主要特性

- **高精度计算**: 所有内部计算均使用 `double` 类型，以确保最高的数值精度。
- **高性能**: 核心算法为纯C++实现，避免了不必要的开销，速度很快。
- **实现可切换**: 您可以通过一个宏在内置实现和OpenCV实现之间轻松切换。
- **易于集成**: 纯头文件，无外部依赖（OpenCV仅为可选）。
- **支持模型**:
  - `cam_radtan.h`: **Radtan模型** (Brown-Conrady)，兼容OpenCV和Kalibr的 `pinhole-radtan`。
  - `cam_equi.h`: **鱼眼模型** (Equidistant)，兼容OpenCV和Kalibr的 `pinhole-equi`。

## 快速上手

#### 1. 包含头文件

您只需要包含 `cam_radtan.h` 或 `cam_equi.h`。它们会自动包含基类 `cam_base.h`。

```cpp
#include "cam_radtan.h" // Radtan模型
// 或
#include "cam_equi.h"   // 鱼眼模型
```

#### 2. 选择实现方式 (可选)

默认情况下，库使用内置的高性能迭代算法。如果您希望改用OpenCV的实现，只需打开 `cam_base.h` 文件，并取消对以下宏的注释：

```cpp
// 在 cam_base.h 文件顶部
// #define USE_OPENCV_IMPL
// 取消注释后变为:
#define USE_OPENCV_IMPL
```
这个开关将同时对 `CamRadtan` 和 `CamEqui` 生效。

#### 3. 定义相机参数

您需要准备相机的内参和畸变系数。

- **内参 `K`** (`std::vector<double>`): `[fx, fy, cx, cy]`
- **畸变系数 `D`** (`std::vector<double>`):
  - `CamRadtan`: `[k1, k2, p1, p2, k3]` (k3可选)
  - `CamEqui`: `[k1, k2, k3, k4]`

#### 4. 创建相机实例

```cpp
// 图像尺寸
int width = 1280;
int height = 720;

// Radtan相机示例
std::vector<double> K = {458.654, 457.296, 639.5, 359.5};
std::vector<double> D = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
CamRadtan camera(width, height, K, D);
```

#### 5. 转换坐标

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

---

## 编译与测试

本项目使用 `CMake` 进行构建。您需要先在您的Linux系统上安装 `g++`, `cmake`, `Eigen3` 和 `OpenCV`。

**Ubuntu/Debian 安装依赖:**
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libeigen3-dev libopencv-dev
```

#### 编译步骤

1.  创建一个构建目录并进入：
    ```bash
    mkdir build && cd build
    ```

2.  运行 CMake 配置并编译：
    ```bash
    cmake ..
    make
    ```

3.  编译成功后，您会在 `build` 目录下找到一个名为 `camera_test` 的可执行文件。

#### 运行测试

直接运行可执行文件即可看到正确性和速度测试的结果。

```bash
./camera_test
```

测试程序会分别评估 **Radtan** 和 **Equi** 两种模型。它会打印出往返误差（用于验证正确性）和处理10万个点所需的时间（用于评估性能）。

要对比内置实现和OpenCV实现的性能，您只需：
1.  修改 `cam_base.h` 中的 `#define USE_OPENCV_IMPL` 宏（注释或取消注释）。
2.  重新编译 (`make`) 并再次运行测试。
