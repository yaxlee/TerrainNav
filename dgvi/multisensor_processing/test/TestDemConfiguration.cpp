// Regression coverage for the DGVI-SLAM configuration and DEM enable switch.
#include <gtest/gtest.h>
#include <dgvi/DatasetReader.hpp>
#include <dgvi/ViParametersReader.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace {

TEST(DemConfiguration, RetainedProfilesParse) {
  for (const char* profile : {"field.yaml", "fp.yaml", "urbanloco.yaml"}) {
    SCOPED_TRACE(profile);
    dgvi::ViParametersReader reader(std::string(DGVI_TEST_CONFIG_DIR) + "/" + profile);
    dgvi::ViParameters parameters;
    reader.getParameters(parameters);
    EXPECT_GT(parameters.nCameraSystem.numCameras(), 0u);
    EXPECT_FALSE(parameters.output.enable_submapping);
    ASSERT_TRUE(parameters.gps.has_value());
    EXPECT_EQ(parameters.gps->type, "geodetic");
  }
}

TEST(DemConfiguration, IntegerHeightAndBlendValuesAreRead) {
  std::ifstream input(std::string(DGVI_TEST_CONFIG_DIR) + "/fp.yaml");
  ASSERT_TRUE(input.good());
  std::string yaml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  yaml += "\ndem_parameters:\n"
          "    use: true\n"
          "    sigma_h: 3\n"
          "    d_above_ground: 2\n"
          "    dem_fusion_alpha: 1\n";
  struct TemporaryConfig {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("dgvi-config-" + std::to_string(std::chrono::high_resolution_clock::now()
                                           .time_since_epoch().count()) + ".yaml");
    ~TemporaryConfig() { std::error_code error; std::filesystem::remove(path, error); }
  } temporary;
  { std::ofstream output(temporary.path); output << yaml; }
  dgvi::ViParametersReader reader(temporary.path.string());
  dgvi::ViParameters parameters;
  reader.getParameters(parameters);
  ASSERT_TRUE(parameters.dem.has_value());
  EXPECT_DOUBLE_EQ(parameters.dem->sigma_h, 3.0);
  EXPECT_DOUBLE_EQ(parameters.dem->d_above_ground, 2.0);
  EXPECT_DOUBLE_EQ(parameters.dem->demFusionAlpha, 1.0);
}

TEST(DemConfiguration, DisabledDemDoesNotOpenSuppliedRasters) {
  dgvi::GpsParameters gps;
  gps.type = "geodetic";
  dgvi::DemParameters dem;
  dem.use = false;
  dem.useDemHeightForGps = true;
  const std::vector<std::string> missingRaster{"/nonexistent/dgvi-test-dem.tif"};
  EXPECT_NO_THROW({
    dgvi::DatasetReader reader("unused", 1, {0}, dgvi::Duration(0.0), gps, dem, missingRaster);
  });
  dem.use = true;
  EXPECT_ANY_THROW({
    dgvi::DatasetReader reader("unused", 1, {0}, dgvi::Duration(0.0), gps, dem, missingRaster);
  });
}

}  // namespace
