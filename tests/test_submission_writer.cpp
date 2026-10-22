#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "mot3d/io/submission_writer.h"

namespace mot3d {
namespace {

using Json = nlohmann::json;
using Results = std::unordered_map<std::string, std::vector<TrackedBox>>;

constexpr double kPi = 3.14159265358979323846;

// A path in the system's temporary folder, so tests never write into the repo.
std::string TempPath(const std::string& name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

// Reads a written file back, so tests check what actually ended up on disk.
Json ReadBack(const std::string& path) {
  std::ifstream file(path);
  Json j;
  file >> j;
  return j;
}

// A car with known values everywhere, and track id 7.
TrackedBox MakeCar(double yaw) {
  TrackedBox t;
  t.box.center = Eigen::Vector3d(10.0, 20.0, 1.0);
  t.box.size = Eigen::Vector3d(1.9, 4.6, 1.7);  // width, length, height
  t.box.yaw = yaw;
  t.box.velocity = Eigen::Vector2d(1.0, 2.0);
  t.box.velocity_valid = true;
  t.box.score = 0.9;
  t.box.cls = ClassId::kCar;
  t.track_id = 7;
  return t;
}

TEST(SubmissionWriter, ReportsUnwritablePath) {
  std::string error;
  EXPECT_FALSE(WriteSubmission("/no/such/folder/out.json", Results{}, &error));
  EXPECT_FALSE(error.empty());
}

TEST(SubmissionWriter, WritesMetaAndEmptyFrames) {
  const std::string path = TempPath("mot3d_test_empty_frame.json");
  Results results;
  results["frame_a"] = {};  // a frame with no boxes

  std::string error;
  ASSERT_TRUE(WriteSubmission(path, results, &error)) << error;

  const Json j = ReadBack(path);
  ASSERT_TRUE(j.contains("meta"));
  EXPECT_EQ(j["meta"]["use_lidar"], true);
  ASSERT_TRUE(j["results"].contains("frame_a"));
  EXPECT_TRUE(j["results"]["frame_a"].is_array());
  EXPECT_TRUE(j["results"]["frame_a"].empty());
}

TEST(SubmissionWriter, WritesEveryFieldOfABox) {
  const std::string path = TempPath("mot3d_test_one_box.json");
  Results results;
  results["frame_a"] = {MakeCar(kPi / 2)};

  std::string error;
  ASSERT_TRUE(WriteSubmission(path, results, &error)) << error;

  const Json j = ReadBack(path);
  ASSERT_EQ(j["results"]["frame_a"].size(), 1u);
  const Json& b = j["results"]["frame_a"][0];

  EXPECT_EQ(b["sample_token"], "frame_a");

  EXPECT_DOUBLE_EQ(b["translation"][0].get<double>(), 10.0);
  EXPECT_DOUBLE_EQ(b["translation"][1].get<double>(), 20.0);
  EXPECT_DOUBLE_EQ(b["translation"][2].get<double>(), 1.0);

  EXPECT_DOUBLE_EQ(b["size"][0].get<double>(), 1.9);  // width
  EXPECT_DOUBLE_EQ(b["size"][1].get<double>(), 4.6);  // length
  EXPECT_DOUBLE_EQ(b["size"][2].get<double>(), 1.7);  // height

  // 90 degrees about the vertical axis: [cos 45°, 0, 0, sin 45°].
  EXPECT_NEAR(b["rotation"][0].get<double>(), std::cos(kPi / 4), 1e-9);
  EXPECT_NEAR(b["rotation"][1].get<double>(), 0.0, 1e-9);
  EXPECT_NEAR(b["rotation"][2].get<double>(), 0.0, 1e-9);
  EXPECT_NEAR(b["rotation"][3].get<double>(), std::sin(kPi / 4), 1e-9);

  EXPECT_DOUBLE_EQ(b["velocity"][0].get<double>(), 1.0);
  EXPECT_DOUBLE_EQ(b["velocity"][1].get<double>(), 2.0);

  EXPECT_TRUE(b["tracking_id"].is_string());
  EXPECT_EQ(b["tracking_id"], "7");
  EXPECT_EQ(b["tracking_name"], "car");
  EXPECT_DOUBLE_EQ(b["tracking_score"].get<double>(), 0.9);
}

TEST(SubmissionWriter, InvalidVelocityBecomesZero) {
  const std::string path = TempPath("mot3d_test_bad_velocity.json");
  TrackedBox car = MakeCar(0.0);
  car.box.velocity_valid = false;
  Results results;
  results["frame_a"] = {car};

  std::string error;
  ASSERT_TRUE(WriteSubmission(path, results, &error)) << error;

  const Json j = ReadBack(path);
  const Json& v = j["results"]["frame_a"][0]["velocity"];
  EXPECT_DOUBLE_EQ(v[0].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(v[1].get<double>(), 0.0);
}

// The scorer rejects a tracking_score that is a whole number. A score of
// exactly 1 must still be written as 1.0.
TEST(SubmissionWriter, ScoreOfOneStaysADecimal) {
  const std::string path = TempPath("mot3d_test_score_one.json");
  TrackedBox car = MakeCar(0.0);
  car.box.score = 1.0;
  Results results;
  results["frame_a"] = {car};

  std::string error;
  ASSERT_TRUE(WriteSubmission(path, results, &error)) << error;

  EXPECT_TRUE(ReadBack(path)["results"]["frame_a"][0]["tracking_score"].is_number_float());
}

// Writing yaw as a quaternion and reading it back with the same formula the
// detection loader uses must give the original angle, in every direction.
TEST(SubmissionWriter, YawSurvivesRoundTrip) {
  const std::string path = TempPath("mot3d_test_yaw.json");
  const std::vector<double> yaws = {0.0, kPi / 2, -kPi / 2, 3.0, -3.0};

  Results results;
  for (double yaw : yaws) results["frame_a"].push_back(MakeCar(yaw));

  std::string error;
  ASSERT_TRUE(WriteSubmission(path, results, &error)) << error;

  const Json boxes = ReadBack(path)["results"]["frame_a"];
  ASSERT_EQ(boxes.size(), yaws.size());
  for (std::size_t i = 0; i < yaws.size(); ++i) {
    const double w = boxes[i]["rotation"][0].get<double>();
    const double x = boxes[i]["rotation"][1].get<double>();
    const double y = boxes[i]["rotation"][2].get<double>();
    const double z = boxes[i]["rotation"][3].get<double>();
    const double recovered = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    EXPECT_NEAR(recovered, yaws[i], 1e-9) << "yaw index " << i;
  }
}

}  // namespace
}  // namespace mot3d