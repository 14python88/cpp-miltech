#pragma once

#include <vector>
#include <memory>
#include <Structs.h>
#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IDroneState.h>

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
    std::vector<std::vector<Coord>> targets;
    PrefParameters prefParameters;
    std::unique_ptr<IDroneState> state;
    float delta_target_bomb;
    float current_direction;
    float current_speed;
    float current_time;
    float acceleration;
    float t_acceleration;
    Coord bombLand;
    Coord dronePosNow;
    DroneContext ctx;

    MissionProcessor(std::unique_ptr<ITargetProvider> t, std::unique_ptr<IConfigLoader> l, std::unique_ptr<IBallisticSolver> s);
    
    void init();
    ResultConst solve(const DroneConfig& droneConfig, const Ammo& ammo);
    Coord calculateBallistics(const Coord& targetPos, const float& h);
    Params calculateParameters(const Coord& dropPos);
    // void interpolateTargets(
    //     std::vector<Coord>& targetPosNow,
    //     const int& sim_step,
    //     const ResultConst& resultConst
    // );
    // void extrapolateTargets(
    //     std::vector<Coord>& targetPosPredicted,
    //     const std::vector<Coord>& targetPosNow,
    //     const std::vector<Params>& params
    // );
    void updatePrefParams (
        const std::vector<Params>& params,
        const std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& dropPos
        );
    SimStep updateSteps ();
    // void dronePosChange(DroneContext& ctx);
    void getDropParameters(const float& h);
    void checkSuccess(int& sim_step, const ResultConst& resultConst);
    void changeSolver(std::unique_ptr<IBallisticSolver> s);
};