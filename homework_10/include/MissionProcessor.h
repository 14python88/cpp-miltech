#pragma once

#include <vector>
#include <memory>
#include <Structs.h>
#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IDroneState.h>
#include <PhysicsStructs.h>

class MissionProcessor {

    std::unique_ptr<ITargetProvider>  provider;
    std::unique_ptr<IConfigLoader> loader;
    std::unique_ptr<IBallisticSolver> solver;

    inline float calculateBearing(const Coord& dronePos, const Coord& targetPos);
    float normalizeAngle(float& angle);
    float getDeltaAngle(const float& target, const float& current);
    float getTotalTime(const DroneConfig& config, const float& current_speed, const float& angle, const float& distance, const float& acceleration, const float& t_acceleration);
public:

    int target_count;
    int time_steps;
    DroneConfig config;
    Ammo ammo;
    PrefParameters prefParameters;
    float delta_target_bomb;
    float acceleration;
    float t_acceleration;
    Coord bombLand;
    float current_time;

    MissionProcessor(std::unique_ptr<ITargetProvider> t, std::unique_ptr<IConfigLoader> l, std::unique_ptr<IBallisticSolver> s);
    
    void init();
    void getTelemetry(DroneTelemetry telemetry);
    ResultConst solve(const DroneConfig& droneConfig, const Ammo& ammo);
    Coord calculateBallistics(const DroneTelemetry telemetry, const Coord targetPos, const float& h);
    Params calculateParameters(const DroneTelemetry telemetry, const Coord& dropPos);
    void updatePrefParams (
        const std::vector<Params>& params,
        const std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& dropPos,
        DroneTelemetry telemetry
        );
    SimStep updateSteps (const DroneTelemetry telemetry, std::unique_ptr<IDroneState>& state);
    void getDropParameters(const DroneTelemetry telemetry, const float& h);
    void checkSuccess(const DroneTelemetry telemetry, int& sim_step, const ResultConst& resultConst);
    void changeSolver(std::unique_ptr<IBallisticSolver> s);
};