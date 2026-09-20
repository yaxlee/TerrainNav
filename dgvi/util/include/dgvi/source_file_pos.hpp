/**
 * DGVI - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich 
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENESE file for details
 */

/**
 * @file source_file_pos.hpp
 * @brief This file contains some helper functions for the assert macros.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 */

#ifndef DGVI_SOURCE_FILE_POS_HPP
#define DGVI_SOURCE_FILE_POS_HPP

#include <string>
#include <iostream>
#include <sstream>
// A class and macro that gives you the current file position.

/// \brief dgvi Main namespace of this package.
namespace dgvi {

/// \brief Helper class to get source file positions.
class source_file_pos {
public:
  std::string function; ///< The function.
  std::string file; ///< The file.
  int line; ///< The line.

  /// \brief Constructor.
  /// \param function The function.
  /// \param file The file.
  /// \param line The line.
  source_file_pos(std::string function, std::string file, int line) :
    function(function), file(file), line(line) {}

  /// \brief Return as string.
  /// \return String message.
  operator std::string()
  {
    return toString();
  }

  /// \brief Return as string.
  /// \return String message.
  std::string toString() const
  {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    return s.str();
  }

};

}// namespace dgvi

/// \brief Print to stream.
/// \param out Out stream buffer.
/// \param sfp dgvi::source_file_pos object to print.
/// \return The stream object.
inline std::ostream & operator<<(std::ostream & out, const dgvi::source_file_pos & sfp)
{
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}


#define DGVI_SOURCE_FILE_POS dgvi::source_file_pos(__FUNCTION__,__FILE__,__LINE__)

#endif // DGVI_SOURCE_FILE_POS_HPP
