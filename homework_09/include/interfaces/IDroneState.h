#pragma once

#include <memory>
#include <Structs.h>

class IDroneState {
    public:
        virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx, PrefParameters& prefParams) = 0;
    
        virtual std::string name() const = 0;

        virtual ~IDroneState() = default;
};
