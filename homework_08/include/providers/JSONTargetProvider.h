#pragma once

#include <interfaces/ITargetProvider.h>
#include <Structs.h>

class JSONTargetProvider : public ITargetProvider {
    int target_count;
    int time_steps;
    std::string path;
    public:
        JSONTargetProvider(std::string path);
        std::vector<std::vector<Coord>> getTargetsCoord() override;
        int getTargetCount() override;
        int getTimeSteps() override;
};