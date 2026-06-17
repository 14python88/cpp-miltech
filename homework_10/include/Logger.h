#pragma once

#include <Structs.h>
#include <json.hpp>
#include <MissionProcessor.h>

#include <fstream>

using json = nlohmann::json;
using namespace std;

class Logger {

public:
    DroneConfig config;
    Ammo ammo;
    ResultConst resultConst;
    nlohmann::json out;
    std::ofstream fout;

    Logger(const std::string& output_path, const DroneConfig& c, const Ammo& a, const ResultConst& TH);
    void configLog();
    void debugLog (const MissionProcessor& mission, const int& sim_step);
    void outputLog(const float& sim_step, const std::vector<SimStep>& steps, const Coord& dropPosPref, const Coord& bombLand, const Coord& targetPredictedPos);
};