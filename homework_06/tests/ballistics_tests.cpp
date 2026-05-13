#include "ballistics.hpp"

#include <gtest/gtest.h>

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
