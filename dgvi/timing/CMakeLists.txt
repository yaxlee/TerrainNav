cmake_minimum_required(VERSION 3.8...3.30)

# Don't use find_package(Boost REQUIRED COMPONENTS headers) or link using the
# newer Boost::headers target to retain compatibility with the FindBoost.cmake
# shipped in Ubuntu 18.04.
find_package(Boost REQUIRED)

set(LIB_NAME dgvi_timing)
add_library(${LIB_NAME}
  include/dgvi/timing/NsecTimeUtilities.hpp
  include/dgvi/timing/Timer.hpp
  src/NsecTimeUtilities.cpp
  src/Timer.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  Boost::boost
  dgvi::Time
  dgvi::Util
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${DGVI_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${DGVI_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${DGVI_PRIVATE_CXX_FLAGS}
)
add_library(dgvi::Timing ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
