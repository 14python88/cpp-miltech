#include <json.hpp>
#include <MissionProcessor.h>
#include <Structs.h>
#include <config/ComponentFactory.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <cctype>
#include <vector>

#include <cfloat>

#define USE_MATH_DEFINES

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
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

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;
using namespace std;

int main(){

    auto* provider = createProvider(SourceType::JSON, "homework_08/data/targets.json");
    auto* loader = createLoader(LoaderType::JSON, "homework_08/data/config.json", "homework_08/data/ammo.json");
    auto* solver = createSolver(SolverType::ANALYTICAL);

    MissionProcessor mission(provider, loader, solver);
    mission.MissionProcessor::init();

    std::vector<SimStep>  steps(10000);
    std::vector<Coord>    dropPos(mission.target_count);
    std::vector<Coord>    targetPosPredicted(mission.target_count);
    std::vector<Coord>    targetPosNow(mission.target_count);
    std::vector<Params>   params(mission.target_count);

    // Calculating additional parameters
    mission.acceleration = mission.config.attack_speed * mission.config.attack_speed / (2 * mission.config.acceleration_path);
    mission.t_acceleration = (2 * mission.config.acceleration_path) / mission.config.attack_speed;
    mission.current_direction = mission.config.initial_dir;
    mission.current_time = 0.0f;
    mission.current_speed = 0.0f;
    mission.dronePosNow = mission.config.startPos;
    int sim_step = 0;
    mission.state = STOPPED;

    ResultConst resultConst = mission.MissionProcessor::solve(mission.config, mission.ammo);

    // Preparing json output
    ofstream fout("homework_08/data/simulation.json");
    json out;
    out["totalSteps"] = 0;
    out["steps"] = json::array();

    LOG("Config loaded: ");
    LOG("altitude = " << mission.config.altitude);
    LOG("initial direction = " << mission.config.initial_dir);
    LOG("attack speed = " << mission.config.attack_speed);
    LOG("acceleration path = " << mission.config.acceleration_path);
    LOG("angular speed = " << mission.config.angular_speed);
    LOG("turn threshold = " << mission.config.turn_threshold);
    LOG("Ammo found: " << mission.config.ammo_name);
    LOG("ammo mass = " << mission.ammo.mass);
    LOG("ammo drag = " << mission.ammo.drag);
    LOG("ammo lift = " << mission.ammo.lift);
    LOG("ammo flight time = " << resultConst.t);
    LOG("ammo flight distance = " << resultConst.h);

    // Main loop
    while (sim_step < 10000) {

        mission.MissionProcessor::interpolateTargets(targetPosNow, sim_step, resultConst);
        for (int j = 0; j < mission.target_count; ++j) {
            dropPos[j] = mission.MissionProcessor::calculateBallistics(targetPosNow[j], resultConst.h);
            params[j] = mission.MissionProcessor::calculateParameters(dropPos[j]);
        };
            if (sim_step > 0) {
                mission.MissionProcessor::extrapolateTargets(targetPosPredicted, targetPosNow, params);
                for (int j = 0; j < mission.target_count; ++j) {
                  dropPos[j] = mission.MissionProcessor::calculateBallistics(targetPosNow[j], resultConst.h);
                  params[j] = mission.MissionProcessor::calculateParameters(dropPos[j]);
            };
        };

        mission.MissionProcessor::updatePrefParams(params, targetPosPredicted, dropPos);

        steps[sim_step] = mission.MissionProcessor::updateSteps();

        // Updating drone position
        mission.MissionProcessor::dronePosChange();

        DEBUG("Current time is " << mission.current_time);
        DEBUG("Number of simulation steps = " << sim_step + 1 << " Current drone coordinates are " << mission.dronePosNow.x << ", " << mission.dronePosNow.y);
        DEBUG("Current drone direction is " << mission.current_direction);
        DEBUG("Preferred target bearing is: " << mission.prefParameters.bearing_pref);
        DEBUG("Current drone speed is " << mission.current_speed);
        DEBUG("Prefered drop point coordinates are: " << mission.prefParameters.dropPosPref.x << ", " << mission.prefParameters.dropPosPref.y);
        DEBUG("Preferred target index is " << mission.prefParameters.target_pref);
        DEBUG("Current distance to preferred drop point is " << mission.prefParameters.drop_dist_pref);
        DEBUG("Current distance to predicted target point is " << mission.prefParameters.dist_target_predicted);

        mission.MissionProcessor::getDropParameters(resultConst.h);

        // Outputting simulation.json
        json step;
        out["totalSteps"] = sim_step + 1;
        step["position"]        = {{"x", steps[sim_step].pos.x}, {"y", steps[sim_step].pos.y}};
        step["direction"]       = steps[sim_step].direction;
        step["state"]           = steps[sim_step].state;
        step["targetIndex"]     = steps[sim_step].target_idx;
        step["dropPoint"]       = {{"x", mission.prefParameters.dropPosPref.x}, {"y", mission.prefParameters.dropPosPref.y}};
        step["aimPoint"]        = {{"x", mission.bombLand.x}, {"y", mission.bombLand.y}};
        step["predictedTarget"] = {{"x", mission.prefParameters.targetPredictedPos.x}, {"y", mission.prefParameters.targetPredictedPos.y}};
        out["steps"].push_back(step);
        fout.seekp(0);
        fout << out.dump(2);
        fout.flush();

        mission.MissionProcessor::checkSuccess(sim_step, resultConst);
        mission.current_time += mission.config.sim_time_step;
        sim_step += 1;
        };
    delete provider;
    provider = nullptr;
    delete loader;
    loader = nullptr;
    delete solver;
    solver = nullptr;
    return 0;
}