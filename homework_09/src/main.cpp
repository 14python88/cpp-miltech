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

using namespace std;

int main(){

  cout << "Enter path for targets file: " << '\n';
  std::string targets_path;
  cin >> targets_path;
  cout << "Enter path for config file: " << '\n';
  std::string config_path;
  cin >> config_path;
  cout << "Enter path for ammo file: " << '\n';
  std::string ammo_path;
  cin >> ammo_path;
  cout << "Enter path for output: " << '\n';
  std::string output_path;
  cin >> output_path;

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

    ResultConst resultConst = mission.MissionProcessor::solve(mission.config, mission.ammo);

    Logger logger(output_path, mission.config, mission.ammo, resultConst);
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

        mission.MissionProcessor::dronePosChange();

        logger.debugLog();

        mission.MissionProcessor::getDropParameters(resultConst.h);

        // Outputting simulation.json

        logger.outputLog(sim_step, steps, mission.prefParameters.dropPosPref, mission.bombLand, mission.prefParameters.targetPredictedPos);

        mission.MissionProcessor::checkSuccess(sim_step, resultConst);

        mission.current_time += mission.config.sim_time_step;
        sim_step += 1;
        
        };
    return 0;
}