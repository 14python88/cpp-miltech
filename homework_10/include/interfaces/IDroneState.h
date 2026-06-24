#pragma once

#include <memory>
#include <Structs.h>
#include "PhysicsStructs.h"

class IDroneState {
    public:
        virtual std::unique_ptr<IDroneState> execute(DroneConfig& config, DroneTelemetry ctx, PrefParameters& prefParams, float& acceleration, float& t_acceleration) = 0;
    
        virtual std::string name() const = 0;

        virtual ~IDroneState() = default;
};
