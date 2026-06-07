#include <interfaces/IConfigLoader.h>
#include <JSONConfigLoader.h>

#include <fstream>
#include <iostream>
#include <json.hpp>

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

        JSONConfigLoader::JSONConfigLoader(std::string config_path, std::string ammo_path) {
            this->ammo_path = ammo_path;
            this->config_path = config_path;
        };

        DroneConfig JSONConfigLoader::getConfig() {
            std::ifstream config_json(this->config_path);
            json data = json::parse(config_json);

            DroneConfig droneConfig = {
                .startPos = {data["drone"]["position"]["x"], data["drone"]["position"]["y"]},
                .ammo_name = data["ammo"],
                .altitude = data["drone"]["altitude"],
                .initial_dir = data["drone"]["initialDirection"],
                .attack_speed = data["drone"]["attackSpeed"],
                .acceleration_path = data["drone"]["accelerationPath"],
                .array_time_step = data["targetArrayTimeStep"],
                .sim_time_step = data["simulation"]["timeStep"],
                .hit_radius = data["simulation"]["hitRadius"],
                .angular_speed = data["drone"]["angularSpeed"],
                .turn_threshold = data["drone"]["turnThreshold"]
            };
            return droneConfig;
        };

        Ammo JSONConfigLoader::getAmmo(const DroneConfig& config) {
            std::ifstream ammo_json(this->ammo_path);
            json j_ammo = json::parse(ammo_json);

            std::vector<Ammo> arsenal(5);
            for (int i = 0; i < 5; ++i) {
                arsenal[i].name  = j_ammo[i]["name"];
                arsenal[i].mass  = j_ammo[i]["mass"];
                arsenal[i].drag  = j_ammo[i]["drag"];
                arsenal[i].lift  = j_ammo[i]["lift"];
            }

            for (const Ammo& a : arsenal) {
                if (config.ammo_name == a.name) {
                    return a;
                }
            }

            std::cout << "Unknown ammo type!" << '\n';
            exit(EXIT_FAILURE);
        };