//
// Created by sham on 2/16/24.
//

#include "Algorithm.h"
//TODO: добавить проверку на валидность входа

namespace pf::algorithms{

    //TODO: добавить проверку на валидность входа
    void depthFirstSearch(const int nodeId, const IGraph& graph, std::vector<int>& visited, std::stack<int>& order){
        for (const auto adjacentNode: graph.getNeighborById(nodeId)) {
            if (!visited[adjacentNode]) {
                visited[adjacentNode] = 1;
                depthFirstSearch(adjacentNode, graph, visited, order);
            }
        }
        order.push(nodeId);
    }

    //TODO: добавить проверку на валидность входа
    std::stack<int> topologicalSort(const IGraph& graph) {

        std::stack<int> order;
        std::vector<int> visited(graph.getSizeUniqNode(), 0);

        for (int nodeId = 0; nodeId < graph.getSizeUniqNode(); ++nodeId) {
            if (!visited[nodeId]) {
                depthFirstSearch(nodeId, graph, visited, order);
            }
        }
        return order;
    }

    //TODO: добавить проверку на валидность входа
    ShortPathWhithCost weightedTopologicalSort(const int idPointStart, const int idPointTarget,
                                               const IGraph& graph,
                                               const std::function<float(int, int)>& calculateTransitionCosts) {

        auto order = topologicalSort(graph);

        std::vector<double> costToPoint(graph.getSizeUniqNode(), std::numeric_limits<double>::max());
        std::vector<int> prev(graph.getSizeUniqNode(), std::numeric_limits<int>::max());
        costToPoint[idPointStart] = 0.0;

        while (!order.empty()) {
            auto currId = order.top();
            order.pop();
            if(currId < idPointStart) continue;

            for (const auto nextId : graph.getNeighborById(currId)) {
                const auto transitionCost = calculateTransitionCosts(currId, nextId) ;
                const auto newCost = costToPoint[currId] + transitionCost;

                if (newCost < costToPoint[nextId]) {
                    costToPoint[nextId] = newCost;
                    prev[nextId] = currId;
                }
            }
        }

        if(costToPoint[idPointTarget] == std::numeric_limits<double>::infinity()){
            return {{},costToPoint[idPointTarget]};
        }

        std::vector<int> shortestPath;
        shortestPath.push_back(idPointTarget);
        for (int prevIndex = prev.back(); prevIndex != std::numeric_limits<int>::max(); prevIndex = prev[prevIndex]) {
            shortestPath.push_back(prevIndex);
        }
        std::reverse(shortestPath.begin(),shortestPath.end());

        return {shortestPath, costToPoint[idPointTarget]};
    }

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