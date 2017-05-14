project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)
add_definitions(-std=c++0x)

# set(EIGEN_LIBS ${PROJECT_SOURCE_DIR}/Eigen)

include_directories(include)
include_directories(Eigen)

file(GLOB src_files "src/*.cpp")
file(GLOB header_files "include/*.h" "Eigen/*.h")

add_executable(UnscentedKF ${src_files} ${header_files})
# target_include_directories(UnscentedKF PUBLIC ${EIGEN_LIBS})

