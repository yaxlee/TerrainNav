cmake_minimum_required(VERSION 3.8...3.30)

set(LIB_NAME dgvi_time)
add_library(${LIB_NAME}
  include/dgvi/Duration.hpp
  include/dgvi/Time.hpp
  include/dgvi/implementation/Duration.hpp
  include/dgvi/implementation/Time.hpp
  src/Duration.cpp
  src/Time.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${DGVI_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${DGVI_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${DGVI_PRIVATE_CXX_FLAGS}
)
add_library(dgvi::Time ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
