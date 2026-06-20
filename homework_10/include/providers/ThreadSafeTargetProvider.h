#pragma once

#include <interfaces/ITargetProvider.h>
#include <Structs.h>

class ThreadSafeTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::string path;
    std::vector<std::vector<Coord>> targets;

    public:
        ThreadSafeTargetProvider(std::string path);
        void getTargetsVector() override; 
        std::vector<std::vector<Target>> getTargetsNow(float current_time, ResultConst resultConst, DroneContext ctx) override;
        void extrapolateTargets(
            std::vector<Coord>& targetPosPredicted,
            const std::vector<Params>& params,
            float current_time,
            std::vector<std::vector<Target>> target) override;
        int getTargetCount() override;
        int getTimeSteps() override;
};