#pragma once

#include <Structs.h>
#include <interfaces/IDroneState.h>

class StateStopped : public IDroneState {

public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) override;

    std::string name() const override;
};

class StateTurning : public IDroneState {

public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) override;

    std::string name() const override;
};

class StateAccelerating : public IDroneState {

public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) override;

    std::string name() const override;
};

class StateMoving : public IDroneState {

public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) override;

    std::string name() const override;
};

class StateDecelerating : public IDroneState {

public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) override;

    std::string name() const override;
};