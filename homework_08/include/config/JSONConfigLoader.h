#pragma once

#include <interfaces/IConfigLoader.h>

class JSONConfigLoader : public IConfigLoader {
    std::string ammo_path;
    std::string config_path;
    public:
        JSONConfigLoader(std::string config_path, std::string ammo_path);
        DroneConfig getConfig() override;
        Ammo getAmmo(const DroneConfig& config) override;
};