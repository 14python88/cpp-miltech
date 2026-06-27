#include <config/ComponentFactory.h>
#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <ThreadSafeTargetProvider.h>
#include <JSONConfigLoader.h>
#include <AnalyticalSolver.h>
#include <memory>

std::unique_ptr<ITargetProvider> createProvider(SourceType type, const std::string& path, float array_time_step)
{
    switch (type) {
        case SourceType::JSON:
            return std::make_unique<ThreadSafeTargetProvider>(path, array_time_step);
        default:
            return nullptr;
    }
}

std::unique_ptr<IBallisticSolver> createSolver(SolverType type) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        default:
            return nullptr;
    }
}

std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string& config_path, const std::string& ammo_path)
{
    switch (type) {
        case LoaderType::JSON:
            return std::make_unique<JSONConfigLoader>(config_path, ammo_path);
        default:
            return nullptr;
    }
}