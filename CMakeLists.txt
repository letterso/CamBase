cmake_minimum_required(VERSION 3.10)
project(CameraModelTest)

# --- C++ Standard ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
# Eigen (header-only)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenCV
find_package(OpenCV REQUIRED)

# --- Add Executable ---
add_executable(camera_test main.cpp)

# --- Link Dependencies ---
target_include_directories(camera_test PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(camera_test PRIVATE ${OpenCV_LIBS})

# --- Compiler Flags for Performance ---
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_CLANG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
    message(STATUS "Enabled -O3 and -DNDEBUG for release builds.")
endif()

# --- Installation (Optional) ---
install(TARGETS camera_test DESTINATION bin)
