#ifdef __unix__
#include "LaneFree_linux.h"
#elif defined(WIN32)
#endif

#ifndef STATE_CONTROLLER
#define STATE_CONTROLLER
#include <LaneFree_win.h>
#include "libLaneFreePlugin_Export.h"
#include "Controller.h"
#include "carAction.hpp"
#include "utils.hpp"
#include "laneFreeState.hpp"
#include <stdio.h>
#include <stdlib.h>


carAction MCTSInstance::calculateAction(const laneFreeState& state) {
	auto mcts = createMCTS(state);
    mcts->setC(2);// set c parameter of UCT formula
    mcts->calculateAction();
    /*if (mcts.getRoot().empty()) {

    }*/
    delete mcts;
    return carAction(0.2, 0);
}

laneFreeMCTS* MCTSInstance::createMCTS(const laneFreeState& state) {
    auto backpropagation = new resultBackPropagation();
    auto terminationCheck = new finalState();
    auto scoring = new simulationResult();
    return new laneFreeMCTS(laneFreeState(state), backpropagation, terminationCheck,scoring);
}

#endif

