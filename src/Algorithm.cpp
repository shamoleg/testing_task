//
// Created by sham on 2/16/24.
//

#include "Algorithm.h"
//TODO: добавить проверку на валидность входа

namespace pf::algorithms{

    ShortPathWhithCost dijkstraAlgorithm2(const int idPointStart, const int idPointTarget,
                                          const IGraph& graph,
                                          const std::function<float(int, int)>& calculateTransitionCosts) {

        std::vector<int> previous(graph.getSizeUniqNode(), std::numeric_limits<int>::min());
        std::vector<bool> processed(graph.getSizeUniqNode(), false);
        std::vector<double> costToPoint(graph.getSizeUniqNode(), std::numeric_limits<double>::infinity());

        costToPoint[idPointStart] = 0.0;
        previous[idPointStart] = idPointStart;

        using queueT = std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>>;
        queueT pointsToCalc;
        pointsToCalc.emplace(costToPoint[idPointStart], idPointStart);

        while (!pointsToCalc.empty()) {
            const auto [cost, currId] = pointsToCalc.top();
            pointsToCalc.pop();

            if (currId == idPointTarget) {
                pointsToCalc = queueT();
                break;
            }

            for (const auto nextId : graph.getNeighborById(currId)) {
                const auto transitionCost = calculateTransitionCosts(currId, nextId);
                const auto newCost = costToPoint[currId] + transitionCost;
                if (newCost < costToPoint[nextId]) {
                    costToPoint[nextId] = newCost;
                    pointsToCalc.emplace(newCost, nextId);
                    previous[nextId] = currId;
                }
            }
        }

        if(costToPoint[idPointTarget] == std::numeric_limits<double>::infinity()){
            return {{}, costToPoint[idPointTarget]};
        }

        std::vector<int> shortestPath;
        for (int id = idPointTarget; id != idPointStart; id = previous[id]) {
            shortestPath.push_back(id);
        }

        // TODO: для того чтобы не вызывать reverse можно искать из конца в начало
        std::reverse(shortestPath.begin(), shortestPath.end());
        return {shortestPath, costToPoint[idPointTarget]};
}

}