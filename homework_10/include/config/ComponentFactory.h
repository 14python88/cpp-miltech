#pragma once
#include <interfaces/ITargetProvider.h>
#include <interfaces/IConfigLoader.h>
#include <interfaces/IBallisticSolver.h>
#include <memory>
#include <string>

enum class SourceType { JSON };
enum class SolverType { ANALYTICAL };
enum class LoaderType { JSON };

std::unique_ptr<ITargetProvider> createProvider(SourceType type, const std::string& path, float array_time_step);
std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::unique_ptr<IConfigLoader>    createLoader(LoaderType type, const std::string& config_path, const std::string& ammo_path);