cmake_minimum_required(VERSION 3.8...3.30)

find_package(Eigen3 REQUIRED)

set(LIB_NAME dgvi_kinematics)
add_library(${LIB_NAME} INTERFACE
  #include/dgvi/kinematics/operators.hpp
  #include/dgvi/kinematics/Transformation.hpp
  #include/dgvi/kinematics/implementation/Transformation.hpp
)
target_include_directories(dgvi_util
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  INTERFACE
    Eigen3::Eigen
    dgvi::Util
)
target_compile_features(${LIB_NAME} INTERFACE cxx_std_${DGVI_CXX_STANDARD})
add_library(dgvi::Kinematics ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTS)
  set(TEST_NAME dgvi_kinematics_test)
  add_executable(${TEST_NAME}
    test/TestTransformation.cpp
    test/runTests.cpp
  )
  target_link_libraries(${TEST_NAME}
    ${LIB_NAME}
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
