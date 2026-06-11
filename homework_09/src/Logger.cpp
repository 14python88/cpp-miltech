#include <Logger.h>
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

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;
using namespace std;

    #define ENABLE_LOG	1
    #define ENABLE_DEBUG  1
    
    #if ENABLE_LOG
    #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
    #else
    #define LOG(msg)
    #endif
    
    #if ENABLE_DEBUG
    #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
    #else
    #define DEBUG(msg)
    #endif

    DroneConfig config;
    Ammo ammo;
    ResultConst resultConst;
    nlohmann::json out;
    std::ofstream fout;

    Logger::Logger(const std::string& output_path, const DroneConfig& c, const Ammo& a, const ResultConst& TH){
        this->config = c;
        this->ammo = a;
        this->resultConst = TH;
        this->fout.open(output_path);

        out["totalSteps"] = 0;
        out["steps"] = json::array();
    };

    void Logger::configLog() {
        LOG("Config loaded: ");
        LOG("altitude = " << this->config.altitude);
        LOG("initial direction = " << this->config.initial_dir);
        LOG("attack speed = " << this->config.attack_speed);
        LOG("acceleration path = " << this->config.acceleration_path);
        LOG("angular speed = " << this->config.angular_speed);
        LOG("turn threshold = " << this->config.turn_threshold);
        LOG("Ammo found: " << this->config.ammo_name);
        LOG("ammo mass = " << this->ammo.mass);
        LOG("ammo drag = " << this->ammo.drag);
        LOG("ammo lift = " << this->ammo.lift);
        LOG("ammo flight time = " << this->resultConst.t);
        LOG("ammo flight distance = " << this->resultConst.h);
    };

    void Logger::debugLog (const MissionProcessor& mission, const int& sim_step) {
        DEBUG("Current time is " << mission.ctx.current_time);
        DEBUG("Number of simulation steps = " << sim_step + 1 << " Current drone coordinates are " << mission.ctx.dronePosNow.x << ", " << mission.ctx.dronePosNow.y);
        DEBUG("Current drone direction is " << mission.ctx.current_direction);
        DEBUG("Preferred target bearing is: " << mission.prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << mission.ctx.current_speed);
        DEBUG("Prefered drop point coordinates are: " << mission.prefParameters.dropPosPref.x << ", " << mission.prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << mission.prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << mission.prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << mission.prefParameters.dist_target_predicted);
    };

    void Logger::outputLog(const float& sim_step, const std::vector<SimStep>& steps, const Coord& dropPosPref, const Coord& bombLand, const Coord& targetPredictedPos
    ) {
        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"]        = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"]       = steps[sim_step].direction;
        step["state"]           = steps[sim_step].state;
        step["targetIndex"]     = steps[sim_step].target_idx;
        step["dropPoint"]       = {{"x", dropPosPref.x}, {"y", dropPosPref.y}};
        step["aimPoint"]        = {{"x", bombLand.x}, {"y", bombLand.y}};
        step["predictedTarget"] = {{"x", targetPredictedPos.x}, {"y", targetPredictedPos.y}};
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();
    };