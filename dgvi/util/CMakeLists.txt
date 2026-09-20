cmake_minimum_required(VERSION 3.8...3.30)

set(LIB_NAME dgvi_util)
add_library(${LIB_NAME} INTERFACE
  #include/dgvi/assert_macros.hpp
  #include/dgvi/MstGraph.hpp
  #include/dgvi/source_file_pos.hpp
)
target_include_directories(${LIB_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${LIB_NAME} INTERFACE cxx_std_${DGVI_CXX_STANDARD})
add_library(dgvi::Util ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT dgviTargets)
install(DIRECTORY include/dgvi DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
