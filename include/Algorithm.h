//
// Created by sham on 2/16/24.
//

#include <algorithm>
#include <limits>
#include <queue>
#include <stack>

#ifndef TESTING_TASK_ALGORITHM_H
#define TESTING_TASK_ALGORITHM_H

#include "DataStructures.h"

namespace pf::algorithms {

    void depthFirstSearch(int nodeId, const IGraph& graph, std::vector<int>& visited, std::stack<int>& order);

    std::stack<int> topologicalSort(const IGraph &graph);


    using ShortPathWhithCost = std::pair<std::vector<int>, double>;

    ShortPathWhithCost weightedTopologicalSort(int idPointStart, int idPointTarget,
                                               const IGraph& graph,
                                               const std::function<float(int, int)>& calculateTransitionCosts);

    ShortPathWhithCost dijkstraAlgorithm2(int idPointStart, int idPointTarget,
                                          const IGraph &graph,
                                          const std::function<float(int, int)> &calculateTransitionCosts);
}

#endif //TESTING_TASK_ALGORITHM_H
