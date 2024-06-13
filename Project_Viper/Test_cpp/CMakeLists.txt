cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ComputeDHM)

# set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libtorch)


# Add the include directories for LibTorch
include_directories(
    /opt/libtorch/include
    /opt/libtorch/include/torch/csrc/api/include
)

# Specify the location of the LibTorch libraries
link_directories(/opt/libtorch/lib)


find_package(Torch REQUIRED)

add_executable(compute_dhm main.cpp)
target_link_libraries(compute_dhm "${TORCH_LIBRARIES}")
set_property(TARGET compute_dhm PROPERTY CXX_STANDARD 14)
