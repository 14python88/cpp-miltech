#pragma once

#include <interfaces/IBallisticSolver.h>
#include <Structs.h>

class AnalyticalSolver : public IBallisticSolver {
    public:
        ResultConst solve(const DroneConfig config, const Ammo& ammo) override;
        Coord calculateBallistics(const float& acceleration_path, const Coord& dronePos, const Coord& targetPos, const float& h) override;
};