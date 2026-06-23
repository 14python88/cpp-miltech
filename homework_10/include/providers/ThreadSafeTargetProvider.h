#pragma once

#include <interfaces/ITargetProvider.h>
#include <Structs.h>

class ThreadSafeTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::vector<std::vector<Coord>> targets;

    public:
        ThreadSafeTargetProvider(std::string path);
        void getTargetsVector() override; 
        std::vector<Target> interpolateTargets(float current_time, ResultConst resultConst, DroneContext ctx) override;
        std::vector<Coord> extrapolateTargets(
            const std::vector<Target>& targetNow,
            const std::vector<Params>& params) override;
        int getTargetCount() override;
        int getTimeSteps() override;
};