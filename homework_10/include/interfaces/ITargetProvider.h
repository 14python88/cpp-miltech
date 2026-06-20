#pragma once

#include <Structs.h>

class ITargetProvider {
    public:
        virtual void getTargetsVector() = 0; 
        virtual std::vector<std::vector<Target>> getTargetsNow(float current_time, ResultConst resultConst, DroneContext ctx) = 0;
        virtual void extrapolateTargets(
            std::vector<Coord>& targetPosPredicted,
            const std::vector<Params>& params,
            float current_time,
            std::vector<std::vector<Target>> target) = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() = default;
};
