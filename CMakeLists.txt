cmake_minimum_required(VERSION 3.10)
project(CameraModelTest)

# --- C++ Standard ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
# Eigen (Required, header-only)
find_package(Eigen3 REQUIRED NO_MODULE)

# OpenCV (Optional)
find_package(OpenCV QUIET)

# --- Add Executable ---
add_executable(camera_test main.cpp)

# --- Link Dependencies ---
target_include_directories(camera_test PRIVATE ${EIGEN3_INCLUDE_DIR})

# Conditionally add OpenCV support
if(OpenCV_FOUND)
    message(STATUS "OpenCV found, enabling OpenCV-based implementations.")
    # Add a compile definition to let the code know OpenCV is available
    target_compile_definitions(camera_test PRIVATE HAVE_OPENCV)
    target_link_libraries(camera_test PRIVATE ${OpenCV_LIBS})
else()
    message(STATUS "OpenCV not found, building with internal implementations only.")
endif()


# --- Compiler Flags for Performance ---
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_CLANG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
    message(STATUS "Enabled -O3 and -DNDEBUG for release builds.")
endif()

# --- Installation (Optional) ---
install(TARGETS camera_test DESTINATION bin)
