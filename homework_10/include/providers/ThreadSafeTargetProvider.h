#pragma once

#include <interfaces/ITargetProvider.h>
#include <Structs.h>

class ThreadSafeTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::vector<std::vector<Coord>> targets;
    std::vector<Target> targetNow;
    std::vector<Coord> targetPosPredicted;

    public:
        ThreadSafeTargetProvider(std::string path);
        void getTargetsVector() override; 
        void interpolateTargets(
            float& current_time, 
            ResultConst& resultConst, 
            DroneConfig& config) override;
        void extrapolateTargets(
            const std::vector<Target> targetNow,
            const std::vector<Params> params) override;
        std::vector<Target> getTargetNow() override;
        std::vector<Coord> getTargetPredicted() override;
        int getTargetCount() override;
        int getTimeSteps() override;
};