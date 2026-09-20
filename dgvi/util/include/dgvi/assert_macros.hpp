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
 * @file assert_macros.hpp
 * @brief This file contains some useful assert macros.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 */

#ifndef DGVI_ASSERT_MACROS_HPP
#define DGVI_ASSERT_MACROS_HPP

#include <sstream>
#include "dgvi/source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define DGVI_DEFINE_EXCEPTION(exceptionName, exceptionParent)				\
  class exceptionName : public exceptionParent {						\
  public:																\
  typedef __typeof__(exceptionParent) exceptionParent_t;  \
  using exceptionParent_t::exceptionParent_t; \
};

/// \brief dgvi Main namespace of this package.
namespace dgvi {

  namespace detail {

    template<typename DGVI_EXCEPTION_T>
    inline void DGVI_throw_exception(std::string const & exceptionType, dgvi::source_file_pos sfp, std::string const & message)
    {
      std::stringstream dgvi_assert_stringstream;
      // I have no idea what broke doesn't work with the << operator. sleutenegger: not just Windows, but in general...???
      dgvi_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
      throw(DGVI_EXCEPTION_T(dgvi_assert_stringstream.str()));
    }

    template<typename DGVI_EXCEPTION_T>
    inline void DGVI_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
								   int line, std::string const & message)
    {
      DGVI_throw_exception<DGVI_EXCEPTION_T>(exceptionType, dgvi::source_file_pos(function,file,line),message);
    }


  } // namespace dgvi::detail

  template<typename DGVI_EXCEPTION_T>
  inline void dgvi_assert_throw(bool assert_condition, std::string message, dgvi::source_file_pos sfp) {
    if(!assert_condition)
      {
		detail::DGVI_throw_exception<DGVI_EXCEPTION_T>("", sfp,message);
      }
  }



} // namespace dgvi

#define DGVI_CHECK_MAP(obj, id)                                                                                          	\
  if(!(obj.count(id)))															\
  {																	\
      std::stringstream dgvi_assert_stringstream;											\
      dgvi_assert_stringstream << #obj << ".at(" << #id <<") failed! "; 						\
      dgvi::detail::DGVI_throw_exception<std::runtime_error>("[DGVI_AT_CHECKED] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
  }

#define DGVI_THROW(exceptionType, message) {								\
    std::stringstream dgvi_assert_stringstream;							\
    dgvi_assert_stringstream << message;									\
    dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
  }


#define DGVI_THROW_SFP(exceptionType, SourceFilePos, message){			\
    std::stringstream dgvi_assert_stringstream;							\
    dgvi_assert_stringstream << message;									\
    dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, dgvi_assert_stringstream.str()); \
  }

#define DGVI_ASSERT_TRUE(exceptionType, condition, message)				\
  if(!(condition))														\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_FALSE(exceptionType, condition, message)				\
  if((condition))														\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))							\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_LT(exceptionType, value, upperBound, message)			\
  if((value) >= (upperBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_GE(exceptionType, value, lowerBound, message)			\
  if((value) < (lowerBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_LE(exceptionType, value, upperBound, message)			\
  if((value) > (upperBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_GT(exceptionType, value, lowerBound, message)			\
  if((value) <= (lowerBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_EQ(exceptionType, value, testValue, message)			\
  if((value) != (testValue))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_NE(exceptionType, value, testValue, message)			\
  if((value) == (testValue))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))						\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define DGVI_THROW_DBG(exceptionType, message){							\
    std::stringstream dgvi_assert_stringstream;							\
    dgvi_assert_stringstream << message;									\
    dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
  }



#define DGVI_ASSERT_TRUE_DBG(exceptionType, condition, message)			\
  if(!(condition))														\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_FALSE_DBG(exceptionType, condition, message)			\
  if((condition))														\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, dgvi_assert_stringstream.str()); \
    }


#define DGVI_ASSERT_DBG_RE( condition, message) DGVI_ASSERT_DBG(std::runtime_error, condition, message)

#define DGVI_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))							\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_LT_DBG(exceptionType, value, upperBound, message)		\
  if((value) >= (upperBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)		\
  if((value) < (lowerBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_LE_DBG(exceptionType, value, upperBound, message)		\
  if((value) > (upperBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }

#define DGVI_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)		\
  if((value) <= (lowerBound))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_EQ_DBG(exceptionType, value, testValue, message)		\
  if((value) != (testValue))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }


#define DGVI_ASSERT_NE_DBG(exceptionType, value, testValue, message)		\
  if((value) == (testValue))												\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }



#define DGVI_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))						\
    {																	\
      std::stringstream dgvi_assert_stringstream;							\
      dgvi_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      dgvi::detail::DGVI_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,dgvi_assert_stringstream.str()); \
    }


#define DGVI_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define DGVI_OUT(X)
#define DGVI_THROW_DBG(exceptionType, message)
#define DGVI_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define DGVI_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define DGVI_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define DGVI_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define DGVI_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define DGVI_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define DGVI_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define DGVI_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define DGVI_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define DGVI_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)
#endif



#endif // DGVI_ASSERT_MACROS_HPP

