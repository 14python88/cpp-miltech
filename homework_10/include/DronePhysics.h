#pragma once

#include "Structs.h"
#include "interfaces/IDroneState.h"
#include <memory>


class DronePhysics {
    std::unique_ptr<IDroneState> state;
    DroneContext ctx;

    public:
        void changeState(PrefParameters& prefParameters);
};