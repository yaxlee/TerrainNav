// Regression coverage for the DGVI-SLAM configuration and DEM enable switch.
#include <gtest/gtest.h>
#include <dgvi/DatasetReader.hpp>
#include <dgvi/ViParametersReader.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>

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

class DemAutoDiscovery : public ::testing::Test {
 protected:
  void SetUp() override {
    root = std::filesystem::temp_directory_path() /
        ("dgvi-dem-" + std::to_string(std::chrono::high_resolution_clock::now()
                                        .time_since_epoch().count()));
    mav = root / "mav0";
    demDirectory = mav / "dem0";
    ASSERT_TRUE(std::filesystem::create_directories(demDirectory));
    GDALAllRegister();
    gps.type = "geodetic";
    dem.use = true;
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root, error);
  }

  void writeRaster(const std::filesystem::path& path, double height) {
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    ASSERT_NE(driver, nullptr);
    std::unique_ptr<GDALDataset, decltype(&GDALClose)> raster(
        driver->Create(path.string().c_str(), 2, 2, 1, GDT_Float32, nullptr), GDALClose);
    ASSERT_NE(raster, nullptr);
    double transform[] = {100.0, 0.01, 0.0, 20.0, 0.0, -0.01};
    ASSERT_EQ(raster->SetGeoTransform(transform), CE_None);
    OGRSpatialReference crs;
    ASSERT_EQ(crs.importFromEPSG(4326), OGRERR_NONE);
    char* projection = nullptr;
    ASSERT_EQ(crs.exportToWkt(&projection), OGRERR_NONE);
    const CPLErr projectionStatus = raster->SetProjection(projection);
    CPLFree(projection);
    ASSERT_EQ(projectionStatus, CE_None);
    ASSERT_EQ(raster->GetRasterBand(1)->Fill(height), CE_None);
  }

  std::filesystem::path root, mav, demDirectory;
  dgvi::GpsParameters gps;
  dgvi::DemParameters dem;
};

TEST_F(DemAutoDiscovery, LoadsFromSequenceRootAndMavDirectory) {
  writeRaster(demDirectory / "terrain.tif", 37.5);
  for (const auto& input : {root, mav}) {
    SCOPED_TRACE(input.string());
    dgvi::DatasetReader reader(input.string(), 1, {0}, dgvi::Duration(0.0), gps, dem);
    EXPECT_EQ(reader.numDemDatasets(), 1u);
    EXPECT_DOUBLE_EQ(reader.getDemHeight(19.995, 100.005), 37.5);
  }
}

TEST_F(DemAutoDiscovery, LoadsFromFlatDatasetLayout) {
  const auto flat = root / "flat";
  ASSERT_TRUE(std::filesystem::create_directories(flat / "dem0"));
  writeRaster(flat / "dem0" / "terrain.tif", 42.0);
  dgvi::DatasetReader reader(flat.string(), 1, {0}, dgvi::Duration(0.0), gps, dem);
  EXPECT_EQ(reader.numDemDatasets(), 1u);
  EXPECT_DOUBLE_EQ(reader.getDemHeight(19.995, 100.005), 42.0);
}

TEST_F(DemAutoDiscovery, SortsSupportedRastersAndIgnoresOtherEntries) {
  writeRaster(demDirectory / "z.tiff", 50.0);
  writeRaster(demDirectory / "a.TIF", 12.5);
  std::ofstream(demDirectory / "notes.txt") << "Not a raster";
  ASSERT_TRUE(std::filesystem::create_directory(demDirectory / "directory.tif"));

  const auto source = root / "source.tif";
  writeRaster(source, 99.0);
  GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("VRT");
  ASSERT_NE(driver, nullptr);
  {
    std::unique_ptr<GDALDataset, decltype(&GDALClose)> raster(
        static_cast<GDALDataset*>(GDALOpen(source.string().c_str(), GA_ReadOnly)), GDALClose);
    ASSERT_NE(raster, nullptr);
    std::unique_ptr<GDALDataset, decltype(&GDALClose)> vrt(
        driver->CreateCopy((demDirectory / "b.VRT").string().c_str(), raster.get(),
                           FALSE, nullptr, nullptr, nullptr), GDALClose);
    ASSERT_NE(vrt, nullptr);
  }
  dgvi::DatasetReader reader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem);
  EXPECT_EQ(reader.numDemDatasets(), 3u);
  EXPECT_DOUBLE_EQ(reader.getDemHeight(19.995, 100.005), 12.5);
}

TEST_F(DemAutoDiscovery, DisabledOrAbsentConfigDoesNotLoadRasters) {
  std::ofstream(demDirectory / "broken.tif") << "Not a raster";
  dem.use = false;
  dgvi::DatasetReader disabled(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem);
  EXPECT_EQ(disabled.numDemDatasets(), 0u);
  dgvi::DatasetReader absent(mav.string(), 1, {0}, dgvi::Duration(0.0), gps);
  EXPECT_EQ(absent.numDemDatasets(), 0u);
}

TEST_F(DemAutoDiscovery, SkipsNoDataAndRejectsPositionsOutsideAllRasters) {
  const auto first = demDirectory / "a.tif";
  writeRaster(first, -9999.0);
  {
    std::unique_ptr<GDALDataset, decltype(&GDALClose)> raster(
        static_cast<GDALDataset*>(GDALOpen(first.string().c_str(), GA_Update)), GDALClose);
    ASSERT_NE(raster, nullptr);
    ASSERT_EQ(raster->GetRasterBand(1)->SetNoDataValue(-9999.0), CE_None);
  }
  writeRaster(demDirectory / "b.tif", 42.0);
  dgvi::DatasetReader reader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem);
  EXPECT_EQ(reader.numDemDatasets(), 2u);
  EXPECT_DOUBLE_EQ(reader.getDemHeight(19.995, 100.005), 42.0);
  EXPECT_LT(reader.getDemHeight(0.0, 0.0), -100.0);
}

TEST_F(DemAutoDiscovery, EnabledRequiresReadableDemData) {
  EXPECT_THROW(dgvi::DatasetReader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem),
               dgvi::DatasetReader::Exception);
  std::ofstream(demDirectory / "broken.tif") << "Not a raster";
  EXPECT_THROW(dgvi::DatasetReader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem),
               dgvi::DatasetReader::Exception);
  std::filesystem::remove_all(demDirectory);
  EXPECT_THROW(dgvi::DatasetReader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem),
               dgvi::DatasetReader::Exception);
}

TEST_F(DemAutoDiscovery, ExplicitPathsOverrideDiscoveryAndKeepInputOrder) {
  std::ofstream(demDirectory / "broken.tif") << "Not a raster";
  writeRaster(root / "z.tif", 50.0);
  writeRaster(root / "a.tif", 12.5);
  const std::vector<std::string> paths{(root / "z.tif").string(), (root / "a.tif").string()};
  dgvi::DatasetReader reader(mav.string(), 1, {0}, dgvi::Duration(0.0), gps, dem, paths);
  EXPECT_EQ(reader.numDemDatasets(), 2u);
  EXPECT_DOUBLE_EQ(reader.getDemHeight(19.995, 100.005), 50.0);
}

}  // namespace
