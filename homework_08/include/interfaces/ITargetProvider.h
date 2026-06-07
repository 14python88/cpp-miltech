#pragma once

#include <Structs.h>

class ITargetProvider {
    public:
        virtual std::vector<std::vector<Coord>> getTargetsCoord() = 0;
        virtual int getTimeSteps() = 0;
        virtual int getTargetCount() = 0;
        virtual ~ITargetProvider() = default;
};
