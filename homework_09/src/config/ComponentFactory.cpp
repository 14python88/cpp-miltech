#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>

#include <JSONTargetProvider.h>
#include <JSONConfigLoader.h>
#include <AnalyticalSolver.h>

#include <ComponentFactory.h>
#include <memory>
 
std::unique_ptr<ITargetProvider> createProvider(
    SourceType type, std::string path) {
    switch (type) {
        case SourceType::JSON:
            return std::make_unique<JSONTargetProvider>(path);
        default: return nullptr;
    }
};

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        // case SolverType::TABLE:
        //     return std::make_unique<TableSolver>();
        default: return nullptr;
    }
};

std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type, std::string config_path, std::string ammo_path) {
    switch (type) {
        case LoaderType::JSON:
            return std::make_unique<JSONConfigLoader>(config_path, ammo_path);
        default: return nullptr;
    }
};