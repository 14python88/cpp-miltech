#include <interfaces/ITargetProvider.hpp>
#include <interfaces/IConfigLoader.hpp>
#include <interfaces/IBallisticSolver.hpp>

#include <Structs.hpp>
#include <json.hpp>

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstring>
#include <cctype>
#include <cfloat>

#define USE_MATH_DEFINES

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;
using namespace std;

class MissionProcessor {
    ITargetProvider*  provider;
    IConfigLoader* loader;
    IBallisticSolver* solver;

    inline float calculateBearing(const Coord& dronePos, const Coord& targetPos) {
        float bearing = atan2(targetPos.y - dronePos.y, targetPos.x - dronePos.x);
        return bearing;
    };

    float normalizeAngle(float& angle) {
        while (angle > M_PI){
            angle -= 2.0f * M_PI;
        }
        while (angle < -M_PI){
            angle += 2.0f * M_PI;
        }
        return angle;
    };

    float getDeltaAngle(const float& target, const float& current) {
        float diff = target - current;
        return normalizeAngle(diff);
    };

    float getTotalTime(const DroneConfig& config, const float& current_speed, const float& angle, const float& distance, const float& acceleration, const float& t_acceleration){
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
    }

public:

    int target_count;
    int time_steps;
    DroneConfig config;
    Ammo ammo;
    std::vector<vector<Coord>> targets;
    DroneState state;
    PrefParameters prefParameters;
    float delta_target_bomb;
    float current_direction;
    float current_speed;
    float current_time;
    float acceleration;
    float t_acceleration;
    Coord bombLand;
    Coord dronePosNow;

    MissionProcessor(ITargetProvider* t, IConfigLoader* l, IBallisticSolver* s) : provider(t), loader(l), solver(s) {}

    void init() {
        this->config = loader->getConfig();
        this->ammo = loader->getAmmo(this->config);
        this->targets = provider->getTargetsCoord();
        this->target_count = provider->getTargetCount();
        this->time_steps = provider->getTimeSteps();
    };

    ResultConst solve(const DroneConfig droneConfig, const Ammo& ammo) {
        return solver->solve(droneConfig, ammo);
    };
    
    Coord calculateBallistics(const Coord& targetPos, const float& h) {
        return solver->calculateBallistics(this->config.acceleration_path, this->dronePosNow, targetPos, h);
    };

    Params calculateParameters(const Coord& dropPos) {
        Params params;
        params.bearing     = this->calculateBearing(this->dronePosNow, dropPos);
        params.delta_angle = this->getDeltaAngle(params.bearing, config.initial_dir);
        params.drop_dist   = sqrt(pow((dropPos.x - this->dronePosNow.x),2) + pow((dropPos.y - this->dronePosNow.y),2));
        params.total_time  = this->getTotalTime(this->config, this->current_time, params.delta_angle, params.drop_dist, this->acceleration, this->t_acceleration);
        return params;
    };

        // Interpolates target positions at the current simulation time
    void interpolateTargets(
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
    void extrapolateTargets(
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
    
    void updatePrefParams (
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

    SimStep updateSteps () {
        SimStep steps = {
            this->dronePosNow,
            this->current_direction,
            this->state,
            this->prefParameters.target_pref
        };
        return steps;
    };

    void dronePosChange(){
        if((fabs(this->prefParameters.delta_angle_pref) <= this->config.turn_threshold) && this->current_speed == 0.0){
            this->state = ACCELERATING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed += this->acceleration * this->config.sim_time_step;
            this->current_speed = min(this->current_speed, this->config.attack_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) <= this->config.turn_threshold) && (this->current_speed == this->config.attack_speed) && (this->current_speed != 0.0)){
            this->state = MOVING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed = this->config.attack_speed;
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) <= config.turn_threshold) && (0 < current_speed) && (current_speed < config.attack_speed)){
            this->state = ACCELERATING;
            this->current_direction = this->prefParameters.bearing_pref;
            this->current_speed = this->current_speed + this->acceleration * this->config.sim_time_step;
            this->current_speed = min(this->current_speed, config.attack_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && this->current_speed == 0.0){
            this->state = TURNING;
            if(this->prefParameters.delta_angle_pref < 0){
                this->current_direction -= this->config.angular_speed * this->config.sim_time_step;
            } else if(this->prefParameters.delta_angle_pref > 0){
                this->current_direction += this->config.angular_speed * this->config.sim_time_step;
            }
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && (this->current_speed == this->config.attack_speed) && (this->current_speed != 0.0)){
            this->state = DECELERATING;
            this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
            this->current_speed = max(0.0f, this->current_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        } else if((fabs(this->prefParameters.delta_angle_pref) > this->config.turn_threshold) && (0 < this->current_speed) && (this->current_speed < this->config.attack_speed)){
            this->state = DECELERATING;
            this->current_speed = this->current_speed - this->acceleration * this->config.sim_time_step;
            this->current_speed = max(0.0f, this->current_speed);
            this->dronePosNow.x += this->current_speed * this->config.sim_time_step * cos(this->current_direction);
            this->dronePosNow.y += this->current_speed * this->config.sim_time_step * sin(this->current_direction);
        }
    };

    void getDropParameters(const float& h) {
        this->bombLand = {
            this->dronePosNow.x + h * cos(this->current_direction),
            this->dronePosNow.y + h * sin(this->current_direction)
        };
        this->delta_target_bomb = sqrt(pow((this->bombLand.x - this->prefParameters.targetPredictedPos.x),2) + pow((this->bombLand.y - this->prefParameters.targetPredictedPos.y),2));
    };

    void checkSuccess(const int& sim_step, const ResultConst& resultConst) {
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
    };

    void changeSolver(IBallisticSolver* s) { solver = s; }
};