#pragma once

#include <interfaces/ITargetProvider.h>
#include <Structs.h>

class ThreadSafeTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::string path;
    std::vector<std::vector<Target>> target;

    public:
        ThreadSafeTargetProvider(std::string path);
        void getTarget(float current_time, ResultConst resultConst, DroneContext ctx) override;
        void ThreadSafeTargetProvider::extrapolateTargets(
            std::vector<Coord>& targetPosPredicted,
            const std::vector<Params>& params,
            float current_time,
            std::vector<std::vector<Target>> target) override;
        int getTargetCount() override;
        int getTimeSteps() override;
};