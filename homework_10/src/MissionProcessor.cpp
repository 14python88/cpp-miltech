#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IDroneState.h>
#include <states/DroneStates.h>
#include <MissionProcessor.h>
#include <DronePhysics.h>
#include <PhysicsStructs.h>
#include <Structs.h>
#include <Logger.h>
#include <json.hpp>

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <vector>
#include <cfloat>
#include <memory>
#include <chrono>

#define USE_MATH_DEFINES

#define ENABLE_LOG 1

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

using json = nlohmann::json;

// Constructor / Destructor

MissionProcessor::MissionProcessor(std::unique_ptr<ITargetProvider>  t,
                                   std::unique_ptr<IConfigLoader>     l,
                                   std::unique_ptr<IBallisticSolver>  s,
                                   DronePhysics&                      physics,
                                   Logger*                            logger)
    : provider(std::move(t))
    , loader(std::move(l))
    , solver(std::move(s))
    , physics(physics)
    , logger(logger)
{
    worker_thread = std::thread(&MissionProcessor::run, this);
}

MissionProcessor::~MissionProcessor() {
    stop();
}

// Init

void MissionProcessor::init() {
    this->config       = loader->getConfig();
    this->ammo         = loader->getAmmo(this->config);
    this->target_count = provider->getTargetCount();
    this->resultConst  = solve(this->config, this->ammo);

    this->acceleration   = this->config.attack_speed * this->config.attack_speed
                           / (2.0f * this->config.acceleration_path);
    this->t_acceleration = (2.0f * this->config.acceleration_path)
                           / this->config.attack_speed;
}

// Thread lifecycle

bool MissionProcessor::isThreadReady() const {
    return thread_ready.load(std::memory_order_acquire);
}

void MissionProcessor::start() {
    {
        std::lock_guard<std::mutex> lk(start_mutex);
        start_signaled = true;
    }
    start_cv.notify_one();
}

void MissionProcessor::stop() {
    stop_flag.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(start_mutex);
        start_signaled = true;
    }
    start_cv.notify_one();

    if (worker_thread.joinable())
        worker_thread.join();
}

void MissionProcessor::waitUntilDone() {
    if (worker_thread.joinable())
        worker_thread.join();
}

// Solver swap

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> s) {
    solver = std::move(s);
}

// Private: mission loop

void MissionProcessor::run() {
    thread_ready.store(true, std::memory_order_release);
    // Waiting for start()
    {
        std::unique_lock<std::mutex> lk(start_mutex);
        start_cv.wait(lk, [this] { return start_signaled; });
    }

    if (stop_flag.load(std::memory_order_acquire))
    {
        return;
    }

    // Main body
    using Clock   = std::chrono::steady_clock;
    using Seconds = std::chrono::duration<double>;

    auto next_tick = Clock::now() + Seconds(config.sim_time_step);

    while (!stop_flag.load(std::memory_order_acquire)) {

        std::this_thread::sleep_until(next_tick);
        next_tick += Seconds(config.sim_time_step);

        if (stop_flag.load(std::memory_order_acquire))
            break;

        // Reading drone telemetry from DronePhysics
        DroneTelemetry telemetry = physics.getDroneTelemetry();

        // Reading current target positions from provider
        std::vector<Target> targetsNow = provider->getTargetNow();

        // Calculating current parameters for each target
        std::vector<Coord>  dropPos(target_count);
        std::vector<Params> params(target_count);

        for (int i = 0; i < target_count; ++i) {
            dropPos[i] = calculateBallistics(
                telemetry,
                targetsNow[i].pos,
                resultConst.h);
            params[i]  = calculateParameters(telemetry, dropPos[i]);
        }

        provider->extrapolateTargets(params);
        std::vector<Coord> targetPosPredicted = provider->getTargetPredicted();

        // Selecting/updating preferred target
        updatePrefParams(params, targetPosPredicted, dropPos, telemetry);

        // Calculating drop point parameters
        getDropParameters(telemetry, resultConst.h);

        // Check if bomb can be released
        checkSuccess(telemetry, sim_step, resultConst);

        // Preparaing step data for logge output
        SimStep step = updateSteps(telemetry);

        // Output of step data
        if (logger) {
            logger->debugLog(telemetry, prefParameters, sim_step);
            logger->outputLog(
                sim_step,
                step,
                prefParameters.dropPosPref,
                bombLand,
                prefParameters.targetPredictedPos);
        }

        LOG("Step " << sim_step
            << " | state: " << physics.getStateName()
            << " | pos: (" << telemetry.current_pos.x
            << ", " << telemetry.current_pos.y << ")"
            << " | target: " << prefParameters.target_pref);

        // Deciding on next drone state and pushing it to DronePhysics
        std::unique_ptr<IDroneState> nextState = decideState(telemetry);
        physics.pushCommand(std::move(nextState), prefParameters);

        ++sim_step;
    }
}

// Private: state decision

std::unique_ptr<IDroneState> MissionProcessor::decideState(
    const DroneTelemetry& telemetry)
{
    const float delta         = prefParameters.delta_angle_pref;
    const float speed         = telemetry.current_speed;
    const std::string current = physics.getStateName();

    bool angle_small = fabs(delta) <= config.turn_threshold;
    bool angle_large = fabs(delta) >  config.turn_threshold;
    bool speed_zero  = speed == 0.0f;
    bool speed_max   = speed >= config.attack_speed;

    if (angle_large && speed_zero && current != "Turning")
        return std::make_unique<StateTurning>();

    if (angle_small && speed_zero && current != "Accelerating")
        return std::make_unique<StateAccelerating>();

    if (angle_small && speed_max && current != "Moving")
        return std::make_unique<StateMoving>();

    if (angle_large && !speed_zero)
        return std::make_unique<StateDecelerating>();

    return nullptr;
}

// Private: utility functions

inline float MissionProcessor::calculateBearing(
    const Coord& dronePos,
    const Coord& targetPos)
{
    return atan2(targetPos.y - dronePos.y, targetPos.x - dronePos.x);
}

float MissionProcessor::normalizeAngle(float& angle)
{
    while (angle >  M_PI) {
        angle -= 2.0f * M_PI;
    };
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    };
    return angle;
}

float MissionProcessor::getDeltaAngle(const float& target, const float& current)
{
    float diff = target - current;
    return normalizeAngle(diff);
}

float MissionProcessor::getTotalTime(
    const DroneConfig& config,
    const float& current_speed,
    const float& angle,
    const float& distance,
    const float& acceleration,
    const float& t_acceleration)
    {
    float total_time = 0.0f;
    if ((fabs(angle) <= config.turn_threshold) && (current_speed == 0.0f)) {
        total_time = t_acceleration + distance / config.attack_speed;
    } else if ((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed)) {
        total_time = distance / config.attack_speed;
    } else if ((fabs(angle) <= config.turn_threshold) && (current_speed > 0.0f) && (current_speed < config.attack_speed)) {
        total_time = (config.attack_speed - current_speed) / acceleration + distance / config.attack_speed;
    } else if ((fabs(angle) > config.turn_threshold) && (current_speed == 0.0f)) {
        total_time = fabs(angle) / config.angular_speed + t_acceleration + distance / config.attack_speed;
    } else if ((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed)) {
        total_time = t_acceleration + fabs(angle) / config.angular_speed + t_acceleration + distance / config.attack_speed;
    } else if ((fabs(angle) > config.turn_threshold) && (current_speed > 0.0f) && (current_speed < config.attack_speed)) {
        total_time = current_speed / acceleration + fabs(angle) / config.angular_speed + t_acceleration + distance / config.attack_speed;
    }
    return total_time;
}

ResultConst MissionProcessor::solve(const DroneConfig& config, const Ammo& ammo) {
    return solver->solve(config, ammo);
}

Coord MissionProcessor::calculateBallistics(
    const DroneTelemetry& telemetry,
    const Coord& targetPos,
    const float& h)
{
    return solver->calculateBallistics(
        config.acceleration_path,
        telemetry.current_pos,
        targetPos,
        h);
}

Params MissionProcessor::calculateParameters(const DroneTelemetry& telemetry, const Coord& dropPos)
{
    Params params;
    params.bearing = calculateBearing(telemetry.current_pos, dropPos);
    params.delta_angle = getDeltaAngle(params.bearing, config.initial_dir);
    params.drop_dist = sqrt(pow(dropPos.x - telemetry.current_pos.x, 2) + pow(dropPos.y - telemetry.current_pos.y, 2));
    params.total_time = getTotalTime(
        config, telemetry.current_speed,
        params.delta_angle,
        params.drop_dist,
        acceleration,
        t_acceleration);
    return params;
}

void MissionProcessor::updatePrefParams(
    const std::vector<Params>& params,
    const std::vector<Coord>&  targetPosPredicted,
    const std::vector<Coord>&  dropPos,
    const DroneTelemetry& telemetry)
{
    float min_time = FLT_MAX;
    for (int i = 0; i < target_count; ++i) {
        if (params[i].total_time < min_time) {
            min_time = params[i].total_time;
            prefParameters = {
                params[i].total_time,
                i,
                targetPosPredicted[i],
                dropPos[i],
                params[i].bearing,
                params[i].delta_angle,
                params[i].drop_dist,
                (float)sqrt(pow(targetPosPredicted[i].x - telemetry.current_pos.x, 2)
                          + pow(targetPosPredicted[i].y - telemetry.current_pos.y, 2))
            };
        }
    }
}

SimStep MissionProcessor::updateSteps(const DroneTelemetry& telemetry)
{
    return {
        telemetry.current_pos,
        telemetry.current_direction,
        physics.getStateName(),
        prefParameters.target_pref
    };
}

void MissionProcessor::getDropParameters(const DroneTelemetry& telemetry, const float& h)
{
    bombLand = {
        static_cast<float>(telemetry.current_pos.x + h * cos(telemetry.current_direction)),
        static_cast<float>(telemetry.current_pos.y + h * sin(telemetry.current_direction))
    };
    delta_target_bomb = sqrt(pow(bombLand.x - prefParameters.targetPredictedPos.x, 2) + pow(bombLand.y - prefParameters.targetPredictedPos.y, 2));
}

void MissionProcessor::checkSuccess(const DroneTelemetry& telemetry, int& sim_step, const ResultConst& resultConst)
{
    if ((fabs(telemetry.current_direction - prefParameters.bearing_pref) < 1e-3f) &&
        (telemetry.current_speed == config.attack_speed) &&
        (prefParameters.drop_dist_pref <= config.hit_radius) &&
        (prefParameters.dist_target_predicted <= resultConst.h + config.hit_radius) &&
        (delta_target_bomb <= config.hit_radius)) {

        LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
        LOG("Bomb flight distance is " << resultConst.h);
        LOG("Bomb flight time is " << resultConst.t);
        LOG("Expected target coordinates are "
            << prefParameters.targetPredictedPos.x << ", "
            << prefParameters.targetPredictedPos.y);
        LOG("Expected bomb land coordinates are "
            << bombLand.x << ", " << bombLand.y);
        LOG("Delta between predicted target position and bomb land position is "
            << delta_target_bomb);

        exit(EXIT_SUCCESS);
    }
}