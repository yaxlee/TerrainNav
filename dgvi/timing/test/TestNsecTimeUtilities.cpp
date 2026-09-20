#include <gtest/gtest.h>
#include <dgvi/timing/NsecTimeUtilities.hpp>

TEST( NsetTimeTestSuite, testChronoConversion ) {

  std::chrono::system_clock::time_point tp1 = std::chrono::system_clock::now();
  dgvi::timing::NsecTime ns1 = dgvi::timing::chronoToNsec( tp1 );
  std::chrono::system_clock::time_point tp2 = dgvi::timing::nsecToChrono( ns1 );
  ASSERT_TRUE(tp1 == tp2);
  
}


TEST( NsetTimeTestSuite, testSecConversion ) {

  dgvi::timing::NsecTime ns1 = dgvi::timing::nsecNow();
  double s2 = dgvi::timing::nsecToSec(ns1);
  dgvi::timing::NsecTime ns2 = dgvi::timing::secToNsec(s2);
  
  ASSERT_LT(abs(ns1-ns2), 1000000);
  
}
