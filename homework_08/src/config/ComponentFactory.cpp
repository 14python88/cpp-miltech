#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>

#include <JSONTargetProvider.h>
#include <JSONConfigLoader.h>
#include <AnalyticalSolver.h>

#include <ComponentFactory.h>
 
ITargetProvider* createProvider(
    SourceType type, std::string path) {
    switch (type) {
    case SourceType::JSON:
        return new JSONTargetProvider(path);
    default: return nullptr;
    }
};

IBallisticSolver* createSolver(
    SolverType type) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return new AnalyticalSolver;
    default: return nullptr;
    }
};

IConfigLoader* createLoader(
    LoaderType type, std::string config_path, std::string ammo_path) {
    switch (type) {
        case LoaderType::JSON:
            return new JSONConfigLoader(config_path, ammo_path);
        default: return nullptr;
    }
};