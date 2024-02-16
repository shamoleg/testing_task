//
// Created by sham on 2/12/24.
//
#include "PathFinder.h"

namespace pf{
    pf::DirectedAcyclicGraph::DirectedAcyclicGraph(size_t numOfNode) {
        data.reserve(numOfNode);
        for(size_t i = 0; i < numOfNode; i++ ){
            data.emplace_back();
        }
    }

    std::vector<int> DirectedAcyclicGraph::getNeighborById(int id) const  {
        std::vector<int> neighbors;

        if (id >= 0 && static_cast<size_t>(id) < data.size()) {
            for (const auto [_, neighbor] : data[id]) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    size_t DirectedAcyclicGraph::getSizeUniqNode() const {
        return data.size();
    }

    void DirectedAcyclicGraph::addDirectedEdge(int from, int to) {
        if (from >= 0 && from < data.size() && to >= 0 && to < data.size()) {
            data[from].emplace_back(0, to);
        }
    };

    int GridGraph::getIdByRowCol(int row, int col) const {
        return row * cols + col;
    }

    std::pair<int, int> GridGraph::getRowColById(int id) const {
        return {id/rows, id%cols};
    }

    void GridGraph::setObstacles(int row, int col) {
        int id = getIdByRowCol(row, col);
        if (id >= 0 && id < data.size()) {
            data[id] = false;
        }
    }

    std::vector<int> GridGraph::getNeighborById(int id) const {
        std::vector<int> neighbors;

        int row = id / cols;
        int col = id % cols;
        auto checkNeighbor = [&](int r, int c) {
            int neighborId = r * cols + c;
            if (r >= 0 && r < rows && c >= 0 && c < cols && data[neighborId]) {
                neighbors.push_back(neighborId);
            }
        };

        checkNeighbor(row - 1, col);
        checkNeighbor(row + 1, col);
        checkNeighbor(row, col - 1);
        checkNeighbor(row, col + 1);

        return neighbors;
    }

    size_t GridGraph::getSizeUniqNode() const {
        return rows * cols;
    }

    double PathCalculatorWithPenalty::calculateTransition(const WayPoint &w1, const WayPoint &w2) {
        return sqrt((pow(w1.pose2D.x - w2.pose2D.x, 2.0) + pow(w1.pose2D.y - w2.pose2D.y, 2.0)));
    }

    double PathCalculatorWithPenalty::calculateSkipCostSum(const int id1, const int id2) {
        if(id2 - id1 == 1){
            skipPenaltySum = 0;
        }
        double res = skipPenaltySum;
        skipPenaltySum += route[id2].skipPenalty;
        return res;
    }

    double PathCalculatorWithPenalty::operator()(const int id1, const int id2) {
        double cost = calculateTransition(route[id1], route[id2]) / param.speed;
        cost += calculateSkipCostSum(id1, id2);
        cost += param.loadTime;
        return cost;
    }

}