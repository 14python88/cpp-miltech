#include <json.hpp>
#include <MissionProcessor.h>
#include <ThreadSafeTargetProvider.h>
#include <DronePhysics.h>
#include <Structs.h>
#include <PhysicsStructs.h>
#include <config/ComponentFactory.h>
#include <Logger.h>

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

#define USE_MATH_DEFINES
#define USE_DEFAULT_PATHS 0

#if USE_DEFAULT_PATHS
    std::string targets_path = "targets.json";
    std::string config_path  = "config.json";
    std::string ammo_path    = "ammo.json";
    std::string output_path  = "simulation.json";
#else
    std::string targets_path;
    std::string config_path;
    std::string ammo_path;
    std::string output_path;
#endif

using namespace std;

int main() {
    // Input file paths
    if (!USE_DEFAULT_PATHS) {
        cout << "Enter path for targets file: \n";
        cin >> targets_path;
        if (targets_path.empty())
        {
            cout << "Targets path empty!\n";
            return 1;
        }

        cout << "Enter path for config file: \n";
        cin >> config_path;
        if (config_path.empty())
        {
            cout << "Config path empty!\n";
            return 1;
        }

        cout << "Enter path for ammo file: \n";
        cin >> ammo_path;
        if (ammo_path.empty())
        {
            cout << "Ammo path empty!\n";
            return 1;
        }

        cout << "Enter path for output: \n";
        cin >> output_path;
        if (output_path.empty())  { cout << "Output path empty!\n";  return 1; }
    }

    // Uploading config and ammo for DronePhysics and ThreadSafeTargetProvider
    auto loader = createLoader(LoaderType::JSON, config_path, ammo_path);
    DroneConfig config = loader->getConfig();
    Ammo ammo = loader->getAmmo(config);

    // Calculating resuiltConst (t,h) for Logger; calling solver directly outside of MissionProcessor
    auto solverTmp  = createSolver(SolverType::ANALYTICAL);
    ResultConst resultConst = solverTmp->solve(config, ammo);

    // Creating components through ComponentsFactory
    // array_time_step from config
    auto providerOwned = createProvider(SourceType::JSON,
                                        targets_path,
                                        config.array_time_step);

    // Raw pointer for use in MissionProcessor
    ThreadSafeTargetProvider* provider =
        static_cast<ThreadSafeTargetProvider*>(providerOwned.get());

    // Set up DronePhysics
    auto physics = std::make_unique<DronePhysics>(
        config,
        config.startPos,
        0.0f,               // початкова швидкість
        config.initial_dir, // початковий напрямок
        0.0f,               // початковий час
        config.sim_time_step
    );

    // Set up Logger
    Logger logger(output_path, config, ammo, resultConst);
    logger.configLog();

    // Set up MissionProcessor
    // Passing provider and loader as unique_ptr (owned by MissionProvider), physics as reference, logger as raw pointer
    auto mission = std::make_unique<MissionProcessor>(
        std::move(providerOwned),
        std::move(loader),
        createSolver(SolverType::ANALYTICAL),
        *physics,
        &logger
    );

    // Initializing mission to update internal fields
    mission->init();

    // Threads ready?
    while (!provider->isThreadReady()
           || !physics->isThreadReady()
           || !mission->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Synchronized start; provider and physics start first
    provider->start();
    physics->start();
    mission->start();

    // Waiting for mission to complete; main() is blocked via join() without stop_flag -- 
    // mission ends via exit(EXIT_SUCCESS) in checkSuccess() when the bomb is dropped
    mission->waitUntilDone();

    // All threads stop
    physics->stop();
    provider->stop();

    return 0;
}