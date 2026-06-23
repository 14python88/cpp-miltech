#pragma once

#include <Structs.h>

class ITargetProvider {
    public:
        virtual void getTargetsVector() = 0; 
        virtual std::vector<Target> interpolateTargets(float current_time, ResultConst resultConst, DroneContext ctx) = 0;
        virtual std::vector<Coord> extrapolateTargets(
            const std::vector<Target>& targetNow,
            const std::vector<Params>& params) = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() = default;
};
