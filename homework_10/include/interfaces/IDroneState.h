#pragma once
#include <memory>
#include <string>
#include <Structs.h>
#include "PhysicsStructs.h"

class IDroneState {
public:
    virtual ~IDroneState() = default;
    virtual std::string name() const = 0;

    struct ExecuteResult;
    virtual ExecuteResult execute(
        DroneConfig& config,
        DroneTelemetry ctx,
        PrefParameters& prefParams,
        float& acceleration,
        float& t_acceleration) = 0;
};

struct IDroneState::ExecuteResult {
    std::unique_ptr<IDroneState> nextState;  // nullptr = no state change
    DroneTelemetry telemetry;  // updated telemetry
};