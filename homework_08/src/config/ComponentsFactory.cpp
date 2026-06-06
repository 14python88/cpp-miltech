#include "../../include/interfaces/ITargetProvider.hpp"
#include "../../include/interfaces/IConfigLoader.hpp"
#include "../../include/interfaces/IBallisticSolver.hpp"

#include "../../include/providers/JSONTargetProvider.hpp"
#include "../../include/config/JSONConfigLoader.hpp"
#include "../../include/solvers/AnalyticalSolver.hpp"

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };
 
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