#include <json.hpp>
#include <MissionProcessor.h>
#include <Structs.h>
#include <config/ComponentFactory.h>
#include <Logger.h>

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cctype>
#include <vector>
#include <cfloat>

#define USE_MATH_DEFINES

#define USE_DEFAULT_PATHS 0
#if USE_DEFAULT_PATHS
  std::string targets_path = "targets.json";
  std::string config_path = "config.json";
  std::string ammo_path = "ammo.json";
  std::string output_path = "simulation.json";
#else
  std::string targets_path;
  std::string config_path;
  std::string ammo_path;
  std::string output_path;
#endif

using namespace std;

int main(){

    if (!USE_DEFAULT_PATHS) {
  cout << "Enter path for targets file: " << '\n';
  cin >> targets_path;
    if(targets_path.empty()){
        cout << "Targets path empty!" << '\n';
        return 1;
    };

  cout << "Enter path for config file: " << '\n';
  cin >> config_path;
  if(config_path.empty()){
    cout << "Config path empty!" << '\n';
    return 1;
  };

  cout << "Enter path for ammo file: " << '\n';
  cin >> ammo_path;
  if(ammo_path.empty()){
    cout << "Ammo path empty!" << '\n';
    return 1;
  };

  cout << "Enter path for output: " << '\n';
  cin >> output_path;
  if(output_path.empty()){
    cout << "Output path empty!" << '\n';
    return 1;
  };
};

    MissionProcessor mission(
      createProvider(SourceType::JSON, targets_path),
      createLoader(LoaderType::JSON, config_path, ammo_path),
      createSolver(SolverType::ANALYTICAL)
    );
    mission.MissionProcessor::init();

    std::vector<SimStep>  steps(10000);
    std::vector<Coord>    dropPos(mission.target_count);
    std::vector<Coord>    targetPosPredicted(mission.target_count);
    std::vector<Coord>    targetPosNow(mission.target_count);
    std::vector<Params>   params(mission.target_count);

    int sim_step = 0;

    ResultConst resultConst = mission.MissionProcessor::solve(mission.ctx.config, mission.ammo);

    Logger logger(output_path, mission.ctx.config, mission.ammo, resultConst);
    logger.configLog();

    // Main loop
    while (sim_step < 10000) {

        mission.MissionProcessor::interpolateTargets(targetPosNow, sim_step, resultConst);
        for (int j = 0; j < mission.target_count; ++j) {
            dropPos[j] = mission.MissionProcessor::calculateBallistics(targetPosNow[j], resultConst.h);
            params[j] = mission.MissionProcessor::calculateParameters(dropPos[j]);
        };
            if (sim_step > 0) {
                mission.MissionProcessor::extrapolateTargets(targetPosPredicted, targetPosNow, params);
                for (int j = 0; j < mission.target_count; ++j) {
                  dropPos[j] = mission.MissionProcessor::calculateBallistics(targetPosNow[j], resultConst.h);
                  params[j] = mission.MissionProcessor::calculateParameters(dropPos[j]);
            };
        };

        mission.MissionProcessor::updatePrefParams(params, targetPosPredicted, dropPos);

        steps[sim_step] = mission.MissionProcessor::updateSteps();

        // mission.MissionProcessor::dronePosChange(ctx);

        auto next = mission.state->execute(mission.ctx, mission.prefParameters);
        if (next){
            mission.state = std::move(next);
        };

        logger.debugLog(mission, sim_step);

        mission.MissionProcessor::getDropParameters(resultConst.h);

        // Outputting simulation.json

        logger.outputLog(sim_step, steps, mission.prefParameters.dropPosPref, mission.bombLand, mission.prefParameters.targetPredictedPos);

        mission.MissionProcessor::checkSuccess(sim_step, resultConst);

        mission.ctx.current_time += mission.ctx.config.sim_time_step;
        sim_step += 1;
        };
    return 0;
}