cmake_minimum_required(VERSION 3.20)
project(teaser_global)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(thirdparty/TEASER-plusplus)

find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)

add_library(teaser_global SHARED
  src/teaser_global.cpp
)
target_include_directories(teaser_global PRIVATE
  include
)
target_link_libraries(teaser_global
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  teaserpp::teaser_registration
  teaserpp::teaser_io
  glim::glim
  glim_ext
)
