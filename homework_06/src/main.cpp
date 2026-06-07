#include "ballistics.hpp"

#include <iostream>

auto main(int argc, char** argv) -> int
{
  if (argc != 2) {
    std::cerr << "usage: ballistics <input_path>" << '\n';
    return 1;
  };

  BallisticsInput input = readInput(argv[1]);

  AmmoInput ammo = getAmmoInput(input.ammo_name.c_str());

  Coord drop_point = calculateBallistics(ammo.mass,
                                         ammo.drag,
                                         ammo.lift,
                                         input.zd,
                                         input.attack_speed,
                                         input.xd,
                                         input.yd,
                                         input.target_x,
                                         input.target_y,
                                         input.acceleration_path);

  printResult(drop_point.x, drop_point.y);

  return 0;
}