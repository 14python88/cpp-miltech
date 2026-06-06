#pragma once

#include "../Structs.hpp"

class IBallisticSolver {
    public:
        virtual ResultConst solve(const DroneConfig config, const Ammo& ammo) = 0;
        virtual Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) = 0;
        virtual ~IBallisticSolver() {};
};