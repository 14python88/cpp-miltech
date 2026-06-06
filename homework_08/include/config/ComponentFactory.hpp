#pragma once

#include "../interfaces/ITargetProvider.hpp"
#include "../interfaces/IBallisticSolver.hpp"
#include "../interfaces/IConfigLoader.hpp"

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };
 
ITargetProvider* createProvider(
    SourceType type, std::string path);

IBallisticSolver* createSolver(
    SolverType type);

IConfigLoader* createLoader(
    LoaderType type, std::string config_path, std::string ammo_path);