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
            this->targets = std::vector<std::vector<Coord>>(this->target_count, std::vector<Coord>(this->time_steps));
        };

        void ThreadSafeTargetProvider::getTargetsVector() {
            std::ifstream targets_json(this->path);
            json j_targets = json::parse(targets_json);
            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    this->targets[i][j].x = positions[j]["x"];
                    this->targets[i][j].y = positions[j]["y"];
                }
            }
            targets_json.close();
        };

        std::vector<std::vector<Target>> ThreadSafeTargetProvider::getTargetsNow(float current_time, ResultConst resultConst, DroneContext ctx){
            std::vector<std::vector<Target>> target(this->target_count, std::vector<Target>(this->time_steps));
            std::vector<Coord> targetPosDelta(this->target_count);

            int index = (int)floor(current_time / ctx.config.array_time_step);
            int idx = index % 120;
            int prev = (idx - 1) % 120;
            float frac = (resultConst.t / ctx.config.array_time_step) - floor(resultConst.t / ctx.config.array_time_step);

            for (int j = 0; j < this->target_count; ++j) {
                if (current_time == 0) {
                    target[j][current_time].pos = targets[j][0];
                    target[j][current_time].velocity = {0,0};
                }else{
                    target[j][current_time].pos = {
                        targets[j][idx].x + (this->targets[j][idx].x - this->targets[j][prev].x) * frac,
                        targets[j][idx].y + (this->targets[j][idx].y - this->targets[j][prev].y) * frac
                    };
                }
                targetPosDelta[j] = target[j][current_time].pos - target[j][prev].pos;

                target[j][current_time].velocity = {
                    targetPosDelta[j].x / ctx.config.array_time_step,
                    targetPosDelta[j].y / ctx.config.array_time_step
                };
            }
            return target;
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