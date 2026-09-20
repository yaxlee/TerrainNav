cmake_minimum_required(VERSION 3.8...3.30)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED calib3d)

set(LIB_NAME dgvi_common)
add_library(${LIB_NAME}
  include/dgvi/FrameTypedefs.hpp
  include/dgvi/Measurements.hpp
  include/dgvi/Parameters.hpp
  include/dgvi/ViFrontendInterface.hpp
  include/dgvi/ViInterface.hpp
  include/dgvi/ViParametersReader.hpp
  src/ViInterface.cpp
  src/ViParametersReader.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GLOG_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
  PUBLIC
    Eigen3::Eigen
    dgvi::CV
    dgvi::Kinematics
    dgvi::Time
    dgvi::Util
    ${OpenCV_LIBS}
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
add_library(dgvi::Common ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
