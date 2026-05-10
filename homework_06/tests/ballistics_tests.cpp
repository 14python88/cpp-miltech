#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownDropPoint) {

  const BallisticsInput input{

      .drone_x = 100.0,

      .drone_y = 100.0,

      .drone_z = 100.0,

      .target_x = 200.0,

      .target_y = 200.0,

      .attack_speed = 10.0,

      .acceleration_path = 10.0,

      .ammo_name = "VOG-17",

  };

  const DropSolution solution = compute_drop_solution(input);

  EXPECT_NEAR(solution.fire_x, 173.759, 0.01);

  EXPECT_NEAR(solution.fire_y, 173.759, 0.01);

}
