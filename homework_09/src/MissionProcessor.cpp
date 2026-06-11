#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <states/DroneStates.h>
#include <interfaces/IDroneState.h>
#include <MissionProcessor.h>

#include <Structs.h>
#include <json.hpp>

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstring>
#include <cctype>
#include <cfloat>
#include <memory>

#define USE_MATH_DEFINES

#define ENABLE_LOG	1
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;
using namespace std;

    std::unique_ptr<ITargetProvider> provider;
    std::unique_ptr<IConfigLoader> loader;
    std::unique_ptr<IBallisticSolver> solver;

    inline float MissionProcessor::calculateBearing(const Coord& dronePos, const Coord& targetPos) {
        float bearing = atan2(targetPos.y - dronePos.y, targetPos.x - dronePos.x);
        return bearing;
    };

    float MissionProcessor::normalizeAngle(float& angle) {
        while (angle > M_PI){
            angle -= 2.0f * M_PI;
        }
        while (angle < -M_PI){
            angle += 2.0f * M_PI;
        }
        return angle;
    };

    float MissionProcessor::getDeltaAngle(const float& target, const float& current) {
        float diff = target - current;
        return normalizeAngle(diff);
    };

    float MissionProcessor::getTotalTime(const DroneConfig& config, const float& current_speed, const float& angle, const float& distance, const float& acceleration, const float& t_acceleration){
        float total_time = 0.0f;
        if((fabs(angle) <= config.turn_threshold) && (current_speed == 0.0)){
            total_time = total_time + t_acceleration + distance / config.attack_speed;
        } else if((fabs(angle) <= config.turn_threshold) && (current_speed == config.attack_speed)){
            total_time = total_time + distance / config.attack_speed;
        } else if((fabs(angle) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            total_time = total_time + ((config.attack_speed - current_speed) / acceleration) + (distance / config.attack_speed);
        } else if((fabs(angle) > config.turn_threshold) && (current_speed == 0.0)){
            total_time = total_time + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
        } else if((fabs(angle) > config.turn_threshold) && (current_speed == config.attack_speed)){
            total_time = total_time + t_acceleration + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
        } else if((fabs(angle) > config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            total_time = total_time + (current_speed / acceleration) + (fabs(angle) / config.angular_speed) + t_acceleration + (distance / config.attack_speed);
        }
        return total_time;
    };

    MissionProcessor::MissionProcessor(std::unique_ptr<ITargetProvider> t, std::unique_ptr<IConfigLoader> l, std::unique_ptr<IBallisticSolver> s) : provider(std::move(t)), loader(std::move(l)), solver(std::move(s)) {};

    void MissionProcessor::init() {
        this->config = loader->getConfig();
        this->ammo = loader->getAmmo(this->config);
        this->targets = provider->getTargetsCoord();
        this->target_count = provider->getTargetCount();
        this->time_steps = provider->getTimeSteps();
        state = std::make_unique<StateStopped>();

        this->acceleration = this->config.attack_speed * this->config.attack_speed / (2 * this->config.acceleration_path);
        this->t_acceleration = (2 * this->config.acceleration_path) / this->config.attack_speed;
        this->current_direction = this->config.initial_dir;
        this->current_time = 0.0f;
        this->current_speed = 0.0f;
        this->dronePosNow = this->config.startPos;

        this->ctx = {
            this->current_direction,
            this->current_speed,
            this->current_time,
            this->acceleration,
            this->dronePosNow,
            this->config
        };
    };

    ResultConst MissionProcessor::solve(const DroneConfig droneConfig, const Ammo& ammo) {
        return solver->solve(droneConfig, ammo);
    };
    
    Coord MissionProcessor::calculateBallistics(const Coord& targetPos, const float& h) {
        return solver->calculateBallistics(this->config.acceleration_path, this->dronePosNow, targetPos, h);
    };

    Params MissionProcessor::calculateParameters(const Coord& dropPos) {
        Params params;
        params.bearing     = this->calculateBearing(this->dronePosNow, dropPos);
        params.delta_angle = this->getDeltaAngle(params.bearing, config.initial_dir);
        params.drop_dist   = sqrt(pow((dropPos.x - this->dronePosNow.x),2) + pow((dropPos.y - this->dronePosNow.y),2));
        params.total_time  = this->getTotalTime(this->config, this->current_time, params.delta_angle, params.drop_dist, this->acceleration, this->t_acceleration);
        return params;
    };

        // Interpolates target positions at the current simulation time
    void MissionProcessor::interpolateTargets(
        std::vector<Coord>& targetPosNow,
        const int& sim_step,
        const ResultConst& resultConst)
    {
        int index = (int)floor(this->current_time / this->config.array_time_step);
        int idx = index % 120;
        int next = (idx + 1) % 120;
        float frac = (resultConst.t / this->config.array_time_step) - floor(resultConst.t / this->config.array_time_step);

        for (int j = 0; j < this->target_count; ++j) {
            if (sim_step == 0) {
                targetPosNow[j] = targets[j][0];
            } else {
                targetPosNow[j] = {
                    this->targets[j][idx].x + (this->targets[j][next].x - this->targets[j][idx].x) * frac,
                    this->targets[j][idx].y + (this->targets[j][next].y - this->targets[j][idx].y) * frac
                };
            }
        }
    }

    // Extrapolates predicted target positions based on current velocity
    void MissionProcessor::extrapolateTargets(
        std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& targetPosNow,
        const std::vector<Params>& params)
        {
        std::vector<Coord>    targetDcoord(5);
        int index = (int)floor(this->current_time / this->config.array_time_step);
        int idx = index % 120;
        int next = (idx + 1) % 120;

        for (int j = 0; j < this->target_count; ++j) {
            targetDcoord[j] = this->targets[j][next] - this->targets[j][idx];

            Velocity speed = {
                targetDcoord[j].x / this->config.array_time_step,
                targetDcoord[j].y / this->config.array_time_step
            };

            targetPosPredicted[j] = {
                targetPosNow[j].x + speed.vx * params[j].total_time,
                targetPosNow[j].y + speed.vy * params[j].total_time
            };
        };
    };
    
    void MissionProcessor::updatePrefParams (
        const std::vector<Params>& params,
        const std::vector<Coord>& targetPosPredicted,
        const std::vector<Coord>& dropPos
        ) {
        float min_time = FLT_MAX;
        for (int i = 0; i < this->target_count; ++i) {
            if (params[i].total_time < min_time) {
                min_time = params[i].total_time;
                this->prefParameters = {
                    params[i].total_time,
                    i,
                    targetPosPredicted[i],
                    dropPos[i],
                    params[i].bearing,
                    params[i].delta_angle,
                    params[i].drop_dist,
                    (float)sqrt(pow((targetPosPredicted[i].x - this->dronePosNow.x),2) + pow((targetPosPredicted[i].y - this->dronePosNow.y),2))
                };
            }
        }
    };

    SimStep MissionProcessor::updateSteps () {
        SimStep steps = {
            this->dronePosNow,
            this->current_direction,
            this->state->name(),
            this->prefParameters.target_pref
        };
        return steps;
    };

    void MissionProcessor::dronePosChange(DroneContext& ctx){
        auto next = state->execute(ctx, this->prefParameters);
        if (next){
            state = std::move(next);
        };
    };

    void MissionProcessor::getDropParameters(const float& h) {
        this->bombLand = {
            this->dronePosNow.x + h * cos(this->current_direction),
            this->dronePosNow.y + h * sin(this->current_direction)
        };
        this->delta_target_bomb = sqrt(pow((this->bombLand.x - this->prefParameters.targetPredictedPos.x),2) + pow((this->bombLand.y - this->prefParameters.targetPredictedPos.y),2));
    };

    void MissionProcessor::checkSuccess(int& sim_step, const ResultConst& resultConst) {
        if ((fabs(this->current_direction - this->prefParameters.bearing_pref) < 1e-3) && (this->current_speed == config.attack_speed) &&
            (this->prefParameters.drop_dist_pref <= this->config.hit_radius) && (this->prefParameters.dist_target_predicted <= resultConst.h + this->config.hit_radius) &&
            (this->delta_target_bomb <= this->config.hit_radius)) {

            LOG("BOMB AWAY! Simulation complete. Steps: " << sim_step + 1);
            LOG("Bomb flight distance is " << resultConst.h);
            LOG("Bomb flight time is " << resultConst.t);
            LOG("Expected target coordinates are " << this->prefParameters.targetPredictedPos.x << ", " << this->prefParameters.targetPredictedPos.y);
            LOG("Expected bomb land coordinates are " << this->bombLand.x << ", " << this->bombLand.y);
            LOG("Delta between predicted target position and bomb land position is " << this->delta_target_bomb);

            exit(EXIT_SUCCESS);
        }
        this->current_time += this->config.sim_time_step;
        sim_step += 1;
    };

    void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> s) { solver = std::move(s); }