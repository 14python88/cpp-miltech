#include <interfaces/ITargetProvider.h>
#include <ThreadSafeTargetProvider.h>

#include <fstream>
#include <json.hpp>
#include "Structs.h"

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

        ThreadSafeTargetProvider::ThreadSafeTargetProvider(std::string path) {
            this->path = path;
            std::ifstream targets_json(path);
            json j_targets = json::parse(targets_json);

            this->target_count = j_targets["targetCount"];
            this->time_steps = j_targets["timeSteps"];
            targets_json.close();

            // this->target.reserve(this->target_count);
            this->target = std::vector<std::vector<Target>>(target_count, std::vector<Target>(time_steps));
        };

        void ThreadSafeTargetProvider::getTarget(float current_time, ResultConst resultConst, DroneContext ctx) {
            std::vector<std::vector<Coord>> targets(target_count, std::vector<Coord>(time_steps));
            std::ifstream targets_json(this->path);
            json j_targets = json::parse(targets_json);
            std::vector<Coord> targetPosDelta(this->target_count);

            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    targets[i][j].x = positions[j]["x"];
                    targets[i][j].y = positions[j]["y"];
                };
            };

            int index = (int)floor(current_time / ctx.config.array_time_step);
            int idx = index % 120;
            int prev = (idx - 1) % 120;
            float frac = (resultConst.t / ctx.config.array_time_step) - floor(resultConst.t / ctx.config.array_time_step);

            for (int j = 0; j < this->target_count; ++j) {
                if (current_time == 0) {
                    this->target[j][current_time].pos = targets[j][0];
                    this->target[j][current_time].velocity = {0,0};
                }else{
                    this->target[j][current_time].pos = {
                        targets[j][idx].x + (targets[j][idx].x - targets[j][prev].x) * frac,
                        targets[j][idx].y + (targets[j][idx].y - targets[j][prev].y) * frac
                    };
                }
                targetPosDelta[j] = this->target[j][current_time].pos - this->target[j][prev].pos;

                this->target[j][current_time].velocity = {
                    targetPosDelta[j].x / ctx.config.array_time_step,
                    targetPosDelta[j].y / ctx.config.array_time_step
                };
            }
            targets_json.close();
        };

        void ThreadSafeTargetProvider::extrapolateTargets(
            std::vector<Coord>& targetPosPredicted,
            const std::vector<Params>& params,
            float current_time,
            std::vector<std::vector<Target>> target)
            {

            for (int j = 0; j < this->target_count; ++j) {
                targetPosPredicted[j] = {
                    target[j][current_time].pos.x + target[j][current_time].velocity.vx * params[j].total_time,
                    target[j][current_time].pos.y + target[j][current_time].velocity.vx * params[j].total_time
                };
            };
        };

        int ThreadSafeTargetProvider::getTargetCount() {
            return this->target_count;
        };

        int ThreadSafeTargetProvider::getTimeSteps() {
            return this->time_steps;
        };