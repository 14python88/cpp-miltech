#pragma once

#include <Structs.h>

class ITargetProvider {
    public:
        virtual void getTargetsVector() = 0; 
        virtual void interpolateTargets(
            float& current_time, 
            ResultConst& resultConst, 
            DroneConfig& config) = 0;
        virtual void extrapolateTargets(
            const std::vector<Target> targetNow,
            const std::vector<Params> params) = 0;
        virtual std::vector<Target> getTargetNow() = 0;
        virtual std::vector<Coord> getTargetPredicted() = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() = default;
};
