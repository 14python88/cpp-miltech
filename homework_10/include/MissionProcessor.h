#pragma once

#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <Structs.h>
#include <PhysicsStructs.h>
#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IDroneState.h>

class DronePhysics;
class Logger;

class MissionProcessor {
public:
    MissionProcessor(
        std::unique_ptr<ITargetProvider>  t,
        std::unique_ptr<IConfigLoader> l,
        std::unique_ptr<IBallisticSolver> s,
        DronePhysics& physics,
        Logger* logger);
    ~MissionProcessor();

    void init();

    // Thread lifecycle
    bool isThreadReady() const;
    void start();
    void stop();

    void waitUntilDone();

    void changeSolver(std::unique_ptr<IBallisticSolver> s);

private:
    // Mission loop
    void run();

    std::unique_ptr<IDroneState> decideState(const DroneTelemetry& telemetry);

    // Utilities
    inline float calculateBearing(const Coord& dronePos, const Coord& targetPos);
    float normalizeAngle(float& angle);
    float getDeltaAngle(const float& target, const float& current);
    float getTotalTime(
        const DroneConfig& config,
        const float& current_speed,
        const float& angle,
        const float& distance,
        const float& acceleration,
        const float& t_acceleration);

    ResultConst solve(const DroneConfig& config, const Ammo& ammo);
    Coord calculateBallistics(const DroneTelemetry& telemetry, const Coord& targetPos, const float& h);
    Params calculateParameters(const DroneTelemetry& telemetry, const Coord& dropPos);
    void updatePrefParams(
        const std::vector<Params>& params,
        const std::vector<Coord>&  targetPosPredicted,
        const std::vector<Coord>&  dropPos,
        const DroneTelemetry& telemetry);
    SimStep updateSteps(const DroneTelemetry& telemetry);
    void getDropParameters(const DroneTelemetry& telemetry, const float& h);
    void checkSuccess(const DroneTelemetry& telemetry, int& sim_step, const ResultConst& resultConst);

    std::unique_ptr<ITargetProvider>  provider;
    std::unique_ptr<IConfigLoader>    loader;
    std::unique_ptr<IBallisticSolver> solver;
    DronePhysics&                     physics;
    Logger*                           logger;

    // Mission parameters
    DroneConfig config;
    Ammo ammo;
    PrefParameters prefParameters {};
    ResultConst resultConst {};
    float delta_target_bomb = 0.0f;
    float acceleration = 0.0f;
    float t_acceleration = 0.0f;
    Coord bombLand {};
    int target_count = 0;
    int sim_step = 0;

    // Start gate
    std::mutex start_mutex;
    std::condition_variable start_cv;
    bool start_signaled = false;

    // Thread state
    std::atomic<bool> stop_flag    {false};
    std::atomic<bool> thread_ready {false};
    std::thread worker_thread;
};