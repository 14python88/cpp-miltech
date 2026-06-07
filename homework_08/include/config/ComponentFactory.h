#pragma once

#include <interfaces/ITargetProvider.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IConfigLoader.h>

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };
 
ITargetProvider* createProvider(
    SourceType type, std::string path);

IBallisticSolver* createSolver(
    SolverType type);

IConfigLoader* createLoader(
    LoaderType type, std::string config_path, std::string ammo_path);