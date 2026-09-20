cmake_minimum_required(VERSION 3.8...3.30)
project(dgvi_slam VERSION 2.0.0 LANGUAGES CXX C)

include(GNUInstallDirs)

# options for build configuration
option (USE_SYSTEM_BRISK
        "Use brisk via find_package rather than downloading it as part of dgvi" OFF)
option (USE_SYSTEM_DBOW2
        "Use DBoW2 via find_package rather than downloading it as part of dgvi" OFF)
option (USE_SYSTEM_CERES
        "Use ceres via find_package rather than downloading it as part of dgvi" OFF)
option (BUILD_APPS
        "Build the DGVI-SLAM dataset application" ON)
option (BUILD_TESTS
        "Builds all gtests" OFF)
option (USE_NN
        "Use keypoint classification as part of dgvi, requires torch" OFF)
option (USE_GPU
        "Use keypoint classification with GPU inference, requires torch with Cuda" OFF)
option (DO_TIMING
        "Enable timing" ON)
option (BUILD_SHARED_LIBS
        "Whether to build dgvi as a shared library" OFF)

option (USE_COLIDMAP "Whether to build dgvi supportin colour and object's id 'for language based DGVI' or regular one" ON)


set(DGVI_CXX_STANDARD 17)
set(DGVI_PUBLIC_CXX_FLAGS)
if(APPLE)
  set(DGVI_PRIVATE_CXX_FLAGS -Wall -Wpedantic -Wextra -Wno-dtor-name)
else()
  set(DGVI_PRIVATE_CXX_FLAGS -Wall -Wpedantic -Wextra)
endif()

set(BLA_VENDOR Generic)
find_package(LAPACK REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

# Make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
endif()

if(USE_NN AND USE_GPU)
  if(APPLE)
    # MPS enabled by default
    add_compile_definitions(DGVI_USE_MPS)
  else()
    # If CUDA isn't enabled explicitly here it's not detected correctly by Torch.
    enable_language(CUDA)
    add_compile_definitions(DGVI_USE_CUDA)
  endif()
endif()

if(USE_NN)
  find_package(Torch REQUIRED)
endif()

if(NOT DO_TIMING)
  add_compile_definitions(DEACTIVATE_TIMERS)
  message(STATUS "Deactivating timers")
endif()

if(BUILD_TESTS)
  # Must be called before adding any tests for them to be registered with CMake
  enable_testing()
endif()

add_subdirectory(external)
# Supereight types and interpolation remain dependencies of the estimator.
option(SE_TEST "Compile the supereight tests" OFF)
option(SE_APP "Compile the supereight application" OFF)
add_subdirectory(supereight2)


add_subdirectory(dgvi)

if(BUILD_APPS)
  find_package(Boost REQUIRED COMPONENTS filesystem)
  find_package(OpenCV REQUIRED core imgcodecs highgui calib3d)
  if(USE_SYSTEM_BRISK)
    find_package(brisk REQUIRED)
  endif()
  if(USE_SYSTEM_DBOW2)
    find_package(DBoW2 REQUIRED)
  endif()
  
  # require GeographicLib
  find_package(GeographicLib REQUIRED)

  find_package(GDAL REQUIRED)

  if(GDAL_FOUND)
      message(STATUS "Found GDAL: ${GDAL_VERSION}")
  endif()

  # DGVI-SLAM: visual-inertial estimation with GNSS and DEM
  add_executable(dgvi_slam_app dgvi/app/src/dgvi_slam_app.cpp)
  target_link_libraries(dgvi_slam_app
    ${OpenCV_LIBS}
    Boost::filesystem
    dgvi::MultisensorProcessing
  )
  target_compile_options(dgvi_slam_app
    PUBLIC
      ${DGVI_PUBLIC_CXX_FLAGS}
    PRIVATE
      ${DGVI_PRIVATE_CXX_FLAGS}
  )
  install(TARGETS dgvi_slam_app EXPORT dgviTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(FILES resources/small_voc.yml.gz DESTINATION ${CMAKE_INSTALL_BINDIR})

  if(USE_NN)
    install(FILES resources/fast-scnn.pt DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# The directory where the .cmake files will be installed
if(WIN32 AND NOT CYGWIN)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}/cmake")
elseif(APPLE)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}.framework/Resources/CMake")
else()
    set(INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PACKAGE_NAME}")
endif()
# Generate dgviConfig.cmake
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/dgviConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dgviConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
# Generate dgviConfigVersion.cmake
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/dgviConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(EXPORT dgviTargets
  FILE dgviTargets.cmake
  NAMESPACE dgvi::
  DESTINATION ${INSTALL_CMAKEDIR}/dgvi)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dgviConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dgviConfigVersion.cmake
  DESTINATION ${INSTALL_CMAKEDIR}/dgvi)
