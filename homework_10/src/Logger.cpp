#include <Logger.h>
#include <Structs.h>
#include <PhysicsStructs.h>
#include <json.hpp>

#include <iostream>
#include <cmath>

using json = nlohmann::json;

#define ENABLE_LOG   1
#define ENABLE_DEBUG 0

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

Logger::Logger(const std::string& output_path, const DroneConfig& c, const Ammo& a, const ResultConst& TH) : config(c), ammo(a), resultConst(TH)
{
    fout.open(output_path);
    out["totalSteps"] = 0;
    out["steps"] = json::array();
}

void Logger::configLog()
{
    LOG("Config loaded:");
    LOG("altitude = " << config.altitude);
    LOG("initial direction = " << config.initial_dir);
    LOG("attack speed = " << config.attack_speed);
    LOG("acceleration path = " << config.acceleration_path);
    LOG("angular speed = " << config.angular_speed);
    LOG("turn threshold = " << config.turn_threshold);
    LOG("Ammo found: " << config.ammo_name);
    LOG("ammo mass = " << ammo.mass);
    LOG("ammo drag = " << ammo.drag);
    LOG("ammo lift = " << ammo.lift);
    LOG("ammo flight time = " << resultConst.t);
    LOG("ammo flight distance = " << resultConst.h);
}

void Logger::debugLog(
  const DroneTelemetry& telemetry,
  const PrefParameters& prefParameters,
  const int& sim_step)
{
    DEBUG("Step = " << sim_step + 1 << " | pos: " << telemetry.current_pos.x << ", " << telemetry.current_pos.y);
    DEBUG("direction = " << telemetry.current_direction);
    DEBUG("speed = " << telemetry.current_speed);
    DEBUG("time = " << telemetry.timeSecSinceStart);
    DEBUG("bearing_pref = " << prefParameters.bearing_pref);
    DEBUG("dropPosPref: " << prefParameters.dropPosPref.x << ", " << prefParameters.dropPosPref.y);
    DEBUG("target_pref = " << prefParameters.target_pref);
    DEBUG("drop_dist_pref = " << prefParameters.drop_dist_pref);
    DEBUG("dist_target_predicted = " << prefParameters.dist_target_predicted);
}

void Logger::outputLog(
  const int& sim_step,
  const SimStep& step,
  const Coord& dropPosPref,
  const Coord& bombLand,
  const Coord& targetPredictedPos)
  {
    json s;
    out["totalSteps"] = sim_step + 1;

    s["position"] = { {"x", step.pos.x}, {"y", step.pos.y} };
    s["direction"] = step.direction;
    s["state"] = step.state;
    s["targetIndex"] = step.target_idx;
    s["dropPoint"] = { {"x", dropPosPref.x}, {"y", dropPosPref.y} };
    s["aimPoint"] = { {"x", bombLand.x}, {"y", bombLand.y} };
    s["predictedTarget"] = { {"x", targetPredictedPos.x}, {"y", targetPredictedPos.y} };

    out["steps"].push_back(s);
    fout.seekp(0);
    fout << out.dump(2);
    fout.flush();
}