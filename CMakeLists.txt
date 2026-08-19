cmake_minimum_required(VERSION 3.16)
project(cubesat_adcs LANGUAGES CXX)
enable_testing()
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(GTest 1.14 REQUIRED)
add_executable(adcs_sil
  cpp/tests/test_math_guidance.cpp
  cpp/tests/test_control_modes.cpp
  cpp/tests/test_estimation.cpp
  cpp/tests/test_cross_validation.cpp)
target_include_directories(adcs_sil PRIVATE cpp/include)
target_compile_features(adcs_sil PRIVATE cxx_std_17)
target_compile_options(adcs_sil PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(adcs_sil PRIVATE Eigen3::Eigen GTest::gtest_main)
target_compile_definitions(adcs_sil PRIVATE ADCS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_test(NAME adcs_sil COMMAND adcs_sil)
