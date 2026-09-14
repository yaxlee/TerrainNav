// Regression coverage for the DGVI-SLAM configuration and DEM enable switch.
#include <gtest/gtest.h>
#include <okvis/DatasetReader.hpp>
#include <okvis/ViParametersReader.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace {

TEST(DemConfiguration, PublishedProfilesParse) {
  for (const char* profile : {"field/dgvi_slam.yaml",
                              "fusionportable/dgvi_slam.yaml",
                              "fusionportable/gnss_only.yaml",
                              "urbanloco/dgvi_slam.yaml"}) {
    SCOPED_TRACE(profile);
    okvis::ViParametersReader reader(std::string(DGVI_TEST_CONFIG_DIR) + "/" + profile);
    okvis::ViParameters parameters;
    reader.getParameters(parameters);
    ASSERT_TRUE(parameters.gps.has_value());
    ASSERT_TRUE(parameters.dem.has_value());
    EXPECT_FALSE(parameters.gps->gpsEnableReInit);
    EXPECT_EQ(parameters.dem->use, std::string(profile) != "fusionportable/gnss_only.yaml");
  }
}

TEST(DemConfiguration, IntegerHeightAndBlendValuesAreRead) {
  std::ifstream input(std::string(DGVI_TEST_CONFIG_DIR) + "/fusionportable/dgvi_slam.yaml");
  ASSERT_TRUE(input.good());
  std::string yaml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  for (const auto& replacement : {std::make_pair("sigma_h: 10.0", "sigma_h: 3"),
                                  std::make_pair("d_above_ground: 1.8", "d_above_ground: 2"),
                                  std::make_pair("dem_fusion_alpha: 0.0", "dem_fusion_alpha: 1")}) {
    const auto pos = yaml.find(replacement.first);
    ASSERT_NE(pos, std::string::npos);
    yaml.replace(pos, std::string(replacement.first).size(), replacement.second);
  }
  struct TemporaryConfig {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("dgvi-config-" + std::to_string(std::chrono::high_resolution_clock::now()
                                           .time_since_epoch().count()) + ".yaml");
    ~TemporaryConfig() { std::error_code error; std::filesystem::remove(path, error); }
  } temporary;
  { std::ofstream output(temporary.path); output << yaml; }
  okvis::ViParametersReader reader(temporary.path.string());
  okvis::ViParameters parameters;
  reader.getParameters(parameters);
  ASSERT_TRUE(parameters.dem.has_value());
  EXPECT_DOUBLE_EQ(parameters.dem->sigma_h, 3.0);
  EXPECT_DOUBLE_EQ(parameters.dem->d_above_ground, 2.0);
  EXPECT_DOUBLE_EQ(parameters.dem->demFusionAlpha, 1.0);
}

TEST(DemConfiguration, DisabledDemDoesNotOpenSuppliedRasters) {
  okvis::GpsParameters gps;
  gps.type = "geodetic";
  okvis::DemParameters dem;
  dem.use = false;
  dem.useDemHeightForGps = true;
  const std::vector<std::string> missingRaster{"/nonexistent/dgvi-test-dem.tif"};
  EXPECT_NO_THROW({
    okvis::DatasetReader reader("unused", 1, {0}, okvis::Duration(0.0), gps, dem, missingRaster);
  });
  dem.use = true;
  EXPECT_ANY_THROW({
    okvis::DatasetReader reader("unused", 1, {0}, okvis::Duration(0.0), gps, dem, missingRaster);
  });
}

}  // namespace
