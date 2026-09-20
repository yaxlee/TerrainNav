cmake_minimum_required(VERSION 3.8...3.30)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED core imgcodecs imgproc highgui calib3d)
find_package(PCL REQUIRED)
find_package(GDAL REQUIRED)
find_package(GeographicLib REQUIRED)

# Generate the configured header file
configure_file(${dgvi_slam_SOURCE_DIR}/cmake/dgviConfig.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/include/dgvi/dgviConfig.hpp" @ONLY)

set(LIB_NAME dgvi_multisensor_processing)
add_library(${LIB_NAME}
  include/dgvi/DatasetReader.hpp
  include/dgvi/QueuedTrajectory.hpp
  include/dgvi/RpgDatasetReader.hpp
  include/dgvi/ThreadedSlam.hpp
  include/dgvi/TrajectoryOutput.hpp
  include/dgvi/ViSensorBase.hpp
  include/dgvi/ViVisualizer.hpp
  include/dgvi/implementation/QueuedTrajectory.hpp
  include/dgvi/threadsafe/ThreadsafeQueue.hpp
  src/DatasetReader.cpp
  src/RpgDatasetReader.cpp
  src/ThreadedSlam.cpp
  src/TrajectoryOutput.cpp
  src/ViVisualizer.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GLOG_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
  ${GDAL_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${OpenCV_LIBS}
  Boost::filesystem
  Eigen3::Eigen
  dgvi::CV
  dgvi::Ceres
  dgvi::Common
  dgvi::Frontend
  dgvi::Mapping
  dgvi::Kinematics
  dgvi::Time
  dgvi::Timing
  dgvi::Util
  ${PCL_LIBRARIES}
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${DGVI_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${DGVI_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${DGVI_PRIVATE_CXX_FLAGS}
)

if(USE_COLIDMAP)
  target_compile_definitions(${LIB_NAME} PUBLIC DGVI_COLIDMAP)
endif()

add_library(dgvi::MultisensorProcessing ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTS)
  set(TEST_NAME dgvi_multisensor_processing_test)
  add_executable(${TEST_NAME}
    test/test_main.cpp
    test/TestDemConfiguration.cpp
  )
  target_compile_definitions(${TEST_NAME} PRIVATE
    DGVI_TEST_CONFIG_DIR="${dgvi_slam_SOURCE_DIR}/config")
  target_link_libraries(${TEST_NAME}
    ${LIB_NAME}
    gmock
    gtest
  )
  target_compile_options(${TEST_NAME}
    PUBLIC
      ${DGVI_PUBLIC_CXX_FLAGS}
    PRIVATE
      ${DGVI_PRIVATE_CXX_FLAGS}
  )
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endif()
