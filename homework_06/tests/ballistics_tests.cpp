#include "ballistics.hpp"

#include <gtest/gtest.h>
#include <cstring>

static constexpr const char* ZERO_ZD_FILE = TEST_DATA_DIR "/test_zero_zd.txt";
static constexpr const char* NEGATIVE_ZD_FILE = TEST_DATA_DIR "/test_neg_zd.txt";

// Test for correct drop point calculations
TEST(Ballistics, ComputesKnownDropPoint) {

  const BallisticsInput input{

      .xd = 100.0,

      .yd = 100.0,

      .zd = 100.0,

      .target_x = 200.0,

      .target_y = 200.0,

      .attack_speed = 10.0,

      .acceleration_path = 10.0,

      .ammo_name = "VOG-17",

  };

  AmmoInput ammo = getAmmoInput(input.ammo_name.c_str());

  Coord drop_point = calculateBallistics(ammo.mass, ammo.drag, ammo.lift, input.zd, input.attack_speed, input.xd, input.yd, input.target_x, input.target_y, input.acceleration_path);

  EXPECT_NEAR(drop_point.x, 173.759, 0.01);
  EXPECT_NEAR(drop_point.y, 173.759, 0.01);

}

// Test for correct drop point calculations with mid point
TEST(Ballistics, ComputesMidPoint) {

  const BallisticsInput input{

      .xd = 100.0,

      .yd = 100.0,

      .zd = 100.0,

      .target_x = 99.0,

      .target_y = 99.0,

      .attack_speed = 10.0,

      .acceleration_path = 10.0,

      .ammo_name = "VOG-17",

  };

  AmmoInput ammo = getAmmoInput(input.ammo_name.c_str());

  Coord drop_point = calculateBallistics(ammo.mass, ammo.drag, ammo.lift, input.zd, input.attack_speed, input.xd, input.yd, input.target_x, input.target_y, input.acceleration_path);

  EXPECT_NEAR(drop_point.x, 125.241, 0.01);
  EXPECT_NEAR(drop_point.y, 125.241, 0.01);

}

// Test for unknown ammo name
TEST(GetAmmoTest, UnknownAmmoExit) {

  EXPECT_EXIT(
    getAmmoInput("unknown_ammo"),
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "Unknown ammo type!"
  );
}

// Test for zero altitude
TEST(ReadInputZdTest, ZeroAltitudeExit) {

  EXPECT_EXIT(
    readInput(ZERO_ZD_FILE),
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "Altitude must be above zero!"
  );
}

// Test for negative altitude
TEST(ReadInputZdTest, NegativeAltitudeExit) {

  EXPECT_EXIT(
    readInput(NEGATIVE_ZD_FILE),
    ::testing::ExitedWithCode(EXIT_FAILURE),
    "Altitude must be above zero!"
  );
}