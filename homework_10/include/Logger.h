#pragma once

#include <Structs.h>
#include <PhysicsStructs.h>
#include <json.hpp>
#include <fstream>
#include <string>

using json = nlohmann::json;

class Logger {
public:
    DroneConfig config;
    Ammo ammo;
    ResultConst resultConst;
    nlohmann::json out;
    std::ofstream fout;

    Logger(const std::string& output_path, const DroneConfig& c, const Ammo& a, const ResultConst& TH);

    void configLog();

    void debugLog(const DroneTelemetry& telemetry, const PrefParameters& prefParameters, const int& sim_step);

    void outputLog(
       const int& sim_step,
       const SimStep& step,
       const Coord& dropPosPref,
       const Coord& bombLand,
       const Coord& targetPredictedPos);
};