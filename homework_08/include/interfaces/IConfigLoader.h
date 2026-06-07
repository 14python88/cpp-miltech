#pragma once

#include <Structs.h>

class IConfigLoader {
    std::string ammo_path;
    std::string config_path;

    public:
        virtual DroneConfig getConfig() = 0;
        virtual Ammo getAmmo(const DroneConfig& config) = 0;
        virtual ~IConfigLoader() = default;
};