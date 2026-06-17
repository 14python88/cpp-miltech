#pragma once

#include <interfaces/ITargetProvider.h>
#include <interfaces/IBallisticSolver.h>
#include <interfaces/IConfigLoader.h>
#include <memory>

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };
 
std::unique_ptr<ITargetProvider> createProvider(
    SourceType type, std::string path);

std::unique_ptr<IBallisticSolver> createSolver(
    SolverType type);

std::unique_ptr<IConfigLoader> createLoader(
    LoaderType type, std::string config_path, std::string ammo_path);