#include <interfaces/ITargetProvider.h>
#include <ThreadSafeTargetProvider.h>

#include <fstream>
#include <json.hpp>
#include "Structs.h"

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

        ThreadSafeTargetProvider::ThreadSafeTargetProvider(std::string path) {

            std::ifstream targets_json(path);
            json j_targets = json::parse(targets_json);
            this->target_count = j_targets["targetCount"];
            this->time_steps = j_targets["timeSteps"];
            this->targets = std::vector<std::vector<Coord>>(this->target_count, std::vector<Coord>(this->time_steps));
            for (int i = 0; i < target_count; ++i) {
                const auto& positions = j_targets["targets"][i]["positions"];
                for (int j = 0; j < time_steps; ++j) {
                    this->targets[i][j].x = positions[j]["x"];
                    this->targets[i][j].y = positions[j]["y"];
                }
            }
            targets_json.close();
        };

        std::vector<Target> ThreadSafeTargetProvider::interpolateTargets(float current_time, ResultConst resultConst, DroneContext ctx){
            std::vector<Target> targetNow(this->target_count);
            std::vector<Coord> targetPosDelta(this->target_count);

            int index = (int)floor(current_time / ctx.config.array_time_step);
            int idx = index % 120;
            int prev = (idx - 1) % 120;
            float frac = (resultConst.t / ctx.config.array_time_step) - floor(resultConst.t / ctx.config.array_time_step);

            for (int j = 0; j < this->target_count; ++j) {
                if (current_time == 0) {
                    targetNow[j].pos = targets[j][0];
                    targetNow[j].velocity = {0,0};
                }else{
                    targetNow[j].pos = {
                        targets[j][idx].x + (this->targets[j][idx].x - this->targets[j][prev].x) * frac,
                        targets[j][idx].y + (this->targets[j][idx].y - this->targets[j][prev].y) * frac
                    };
                }
                targetPosDelta[j] = targetNow[j].pos - targets[j][prev];

                targetNow[j].velocity = {
                    targetPosDelta[j].x / ctx.config.array_time_step,
                    targetPosDelta[j].y / ctx.config.array_time_step
                };
            }
            return targetNow;
        };

            std::vector<Coord> ThreadSafeTargetProvider::extrapolateTargets(
            const std::vector<Target>& targetNow,
            const std::vector<Params>& params)
            {
            std::vector<Coord> targetPosPredicted(this->target_count);

            for (int j = 0; j < this->target_count; ++j) {
                targetPosPredicted[j] = {
                    targetNow[j].pos.x + targetNow[j].velocity.vx * params[j].total_time,
                    targetNow[j].pos.y + targetNow[j].velocity.vx * params[j].total_time
                };
            }
            return targetPosPredicted;
        };

        int ThreadSafeTargetProvider::getTargetCount() {
            return this->target_count;
        };

        int ThreadSafeTargetProvider::getTimeSteps() {
            return this->time_steps;
        };