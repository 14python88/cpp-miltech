#include <interfaces/ITargetProvider.h>
#include <JSONTargetProvider.h>

#include <fstream>
#include <json.hpp>

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

        JSONTargetProvider::JSONTargetProvider(std::string path) {
            this->path = path;
            std::ifstream targets_json(path);
            json j_targets = json::parse(targets_json);

            this->target_count = j_targets["targetCount"];
            this->time_steps = j_targets["timeSteps"];
            targets_json.close();
        };

        std::vector<std::vector<Coord>> JSONTargetProvider::getTargetsCoord() {
            std::vector<std::vector<Coord>> targets(target_count, std::vector<Coord>(time_steps));
            std::ifstream targets_json(this->path);
            json j_targets = json::parse(targets_json);
            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    targets[i][j].x = positions[j]["x"];
                    targets[i][j].y = positions[j]["y"];
                }
            }
            targets_json.close();
            return targets;
        };

        int JSONTargetProvider::getTargetCount() {
            return this->target_count;
        };

        int JSONTargetProvider::getTimeSteps() {
            return this->time_steps;
        };