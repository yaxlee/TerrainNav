cmake_minimum_required(VERSION 3.8...3.30)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED core features2d imgproc)
if(USE_SYSTEM_BRISK)
  find_package(brisk REQUIRED)
endif()
if(USE_NN)
  if(NOT Torch_FOUND)
    find_package(Torch REQUIRED)
  endif()
else()
  message(WARNING "Network keypoint classification deactivated")
endif()

set(LIB_NAME dgvi_cv)
add_library(${LIB_NAME}
  include/dgvi/Frame.hpp
  include/dgvi/MultiFrame.hpp
  include/dgvi/implementation/Frame.hpp
  include/dgvi/implementation/MultiFrame.hpp
  include/dgvi/cameras/CameraBase.hpp
  include/dgvi/cameras/DistortionBase.hpp
  include/dgvi/cameras/EquidistantDistortion.hpp
  include/dgvi/cameras/NCameraSystem.hpp
  include/dgvi/cameras/NoDistortion.hpp
  include/dgvi/cameras/PinholeCamera.hpp
  include/dgvi/cameras/RadialTangentialDistortion.hpp
  include/dgvi/cameras/RadialTangentialDistortion8.hpp
  include/dgvi/cameras/implementation/CameraBase.hpp
  include/dgvi/cameras/implementation/EquidistantDistortion.hpp
  include/dgvi/cameras/implementation/NCameraSystem.hpp
  include/dgvi/cameras/implementation/PinholeCamera.hpp
  include/dgvi/cameras/implementation/RadialTangentialDistortion.hpp
  include/dgvi/cameras/implementation/RadialTangentialDistortion8.hpp
  include/dgvi/internal/Network.hpp
  src/CameraBase.cpp
  src/Frame.cpp
  src/NCameraSystem.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${TORCH_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
  PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    dgvi::Kinematics
    dgvi::Time
    dgvi::Util
  PRIVATE
    brisk::brisk
    ${TORCH_LIBRARIES}
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${DGVI_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${DGVI_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${DGVI_PRIVATE_CXX_FLAGS}
)
if(USE_NN)
  target_compile_options(${LIB_NAME} PUBLIC ${TORCH_CXX_FLAGS})
  target_compile_definitions(${LIB_NAME} PUBLIC DGVI_USE_NN C10_USE_GLOG)
  if(USE_GPU)
    target_compile_definitions(${LIB_NAME} PUBLIC DGVI_USE_GPU)
  endif()
endif()
add_library(dgvi::CV ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTS)
  set(TEST_NAME dgvi_cv_test)
  add_executable(${TEST_NAME}
    test/TestFrame.cpp
    test/TestMultiFrame.cpp
    test/TestNCameraSystem.cpp
    test/TestPinholeCamera.cpp
    test/runTests.cpp
  )
  target_include_directories(${TEST_NAME}
    PRIVATE
      ${TORCH_INCLUDE_DIRS}
  )
  target_link_libraries(${TEST_NAME}
    brisk::brisk
    ${LIB_NAME}
    ${TORCH_LIBRARIES}
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
