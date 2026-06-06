#pragma once

#include <vector>
#include "Structs.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/IBallisticSolver.hpp"

class MissionProcessor {

    inline float calculateBearing(const Coord& dronePos, const Coord& targetPos);
    float normalizeAngle(float& angle) {};
    float getDeltaAngle(const float& target, const float& current);
    float getTotalTime(const DroneConfig& config, const float& current_speed, const float& angle, const float& distance, const float& acceleration, const float& t_acceleration);

public:
    MissionProcessor(ITargetProvider* t, IConfigLoader* l, IBallisticSolver* s);
    void init();
    ResultConst solve(const DroneConfig droneConfig, const Ammo& ammo);
    Coord calculateBallistics(const Coord& targetPos, const float& h);
    Params calculateParameters(const Coord& dropPos);
    void interpolateTargets();
    void extrapolateTargets();
    void updatePrefParams (
        const std::vector<Params>& params,
        const std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& dropPos
        );
    SimStep updateSteps ();
    void dronePosChange();
    void getDropParameters(const float& h);
    void checkSuccess(const int& sim_step, const ResultConst& resultConst);
    void changeSolver(IBallisticSolver* s);
};