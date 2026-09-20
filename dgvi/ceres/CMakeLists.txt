cmake_minimum_required(VERSION 3.8...3.30)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED core imgcodecs imgproc)
find_package(OpenMP QUIET)
if(USE_SYSTEM_CERES)
  find_package(Ceres REQUIRED)
endif()

# require GeographicLib
find_package(GeographicLib REQUIRED)

set(LIB_NAME dgvi_ceres)
add_library(${LIB_NAME}
  include/dgvi/Component.hpp
  include/dgvi/PseudoInverse.hpp
  include/dgvi/ViGraph.hpp
  include/dgvi/ViGraphEstimator.hpp
  include/dgvi/ViSlamBackend.hpp
  include/dgvi/ceres/CeresIterationCallback.hpp
  include/dgvi/ceres/ErrorInterface.hpp
  include/dgvi/ceres/HomogeneousPointError.hpp
  include/dgvi/ceres/HomogeneousPointLocalParameterization.hpp
  include/dgvi/ceres/HomogeneousPointParameterBlock.hpp
  include/dgvi/ceres/ImuError.hpp
  include/dgvi/ceres/ParameterBlock.hpp
  include/dgvi/ceres/ParameterBlockSized.hpp
  include/dgvi/ceres/PoseError.hpp
  include/dgvi/ceres/PoseLocalParameterization.hpp
  include/dgvi/ceres/PoseParameterBlock.hpp
  include/dgvi/ceres/RelativePoseError.hpp
  include/dgvi/ceres/ReprojectionError.hpp
  include/dgvi/ceres/ReprojectionErrorBase.hpp
  include/dgvi/ceres/SpeedAndBiasError.hpp
  include/dgvi/ceres/SpeedAndBiasParameterBlock.hpp
  include/dgvi/ceres/TwoPoseGraphError.hpp
  include/dgvi/ceres/TwoPoseExtrinsicsGraphError.hpp
  include/dgvi/ceres/implementation/ReprojectionError.hpp
  include/dgvi/ceres/ode/ode.hpp
  include/dgvi/ceres/DepthError.hpp
  src/CeresIterationCallback.cpp
  src/ErrorInterface.cpp
  include/dgvi/ceres/DemHeightError.hpp
  src/DemHeightError.cpp
  src/GpsErrorAsynchronous.cpp
  src/GpsErrorSynchronous.cpp
  src/HomogeneousPointError.cpp
  src/HomogeneousPointLocalParameterization.cpp
  src/HomogeneousPointParameterBlock.cpp
  src/ImuError.cpp
  src/PoseError.cpp
  src/PoseLocalParameterization.cpp
  src/PoseParameterBlock.cpp
  src/RelativePoseError.cpp
  src/SpeedAndBiasError.cpp
  src/SpeedAndBiasParameterBlock.cpp
  src/SubmapIcpError.cpp
  src/TwoPoseGraphError.cpp
  src/TwoPoseExtrinsicsGraphError.cpp
  src/ViGraph.cpp
  src/ViGraphEstimator.cpp
  src/ViSlamBackend.cpp
  src/Component.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GeographicLib_INCLUDE_DIRS}
  PRIVATE
    ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
  PUBLIC
    ${OpenCV_LIBS}
    Ceres::ceres
    Eigen3::Eigen
    dgvi::Common
    dgvi::CV
    dgvi::Mapping
    dgvi::Time
    dgvi::Timing
    dgvi::Util
    dgvi::Mapping
    ${GeographicLib_LIBRARIES}
    SRL::Supereight2
    SRL::Projection
  PRIVATE
    ${GLOG_LIBRARIES}
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${DGVI_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${DGVI_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${DGVI_PRIVATE_CXX_FLAGS}
)
if(OPENMP_FOUND)
  target_compile_definitions(${LIB_NAME} PRIVATE USE_OPENMP)
endif()
add_library(dgvi::Ceres ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTS)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  set(TEST_NAME dgvi_ceres_test)
  add_executable(${TEST_NAME}
    test/TestGpsError.cpp
    test/TestHomogeneousPointError.cpp
    test/TestImuError.cpp
    test/TestPoseError.cpp
    test/TestReprojectionError.cpp
    test/TestSubmapIcpError.cpp
    test/test_main.cpp
  )
  target_link_libraries(${TEST_NAME}
    ${LIB_NAME}
    Boost::filesystem
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
