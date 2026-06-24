#pragma once

#include "Structs.h"
#include "interfaces/IDroneState.h"
#include <memory>
#include "PhysicsStructs.h"


class DronePhysics {

    std::unique_ptr<IDroneState> state;
    
    DroneTelemetry telemetry {
        {0,0},
        0.0,
        0.0,
        0.0
    };

    DroneConfig config{};
    float acceleration;
    float t_acceleration;

    public:
        DronePhysics(DroneConfig& config, Coord pos, float speed, float direction, float time, float acceleration, float t_acceleration);
        void changeState(PrefParameters& prefParameters);
};