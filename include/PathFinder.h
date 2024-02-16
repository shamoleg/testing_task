//
// Created by sham on 2/11/24.
//
#ifndef TESTING_TASK_PATHFINDER_H
#define TESTING_TASK_PATHFINDER_H

#include <cmath>
#include <vector>
#include <limits>
#include <optional>
#include <iostream>
#include <algorithm>
#include <queue>

#include "DataStructures.h"

namespace pf {

    class DirectedAcyclicGraph : public IGraph{
    public:
        explicit DirectedAcyclicGraph(size_t numOfNode);
        std::vector<int> getNeighborById(int id) const final;
        size_t getSizeUniqNode () const final;

        void addDirectedEdge(int from, int to);

    private:
        std::vector<std::vector<std::pair<double, int>>> data;
    };

    class GridGraph : public IGraph{
    public:
        GridGraph(int rows, int cols, MapMeta mapMeta)
                : rows(rows), cols(cols), data(rows * cols, true), mapMeta(std::move(mapMeta)) {};

        void setObstacles(int row, int col);

        int getIdByRowCol(int row, int col) const;
        std::pair<int, int> getRowColById(int id) const;
        std::vector<int> getNeighborById(int id) const final;
        size_t getSizeUniqNode() const final;

    private:
        int rows;
        int cols;
        std::vector<bool> data;

        MapMeta mapMeta;
    };

    struct RobotParam{
        double speed = 2.0;
        double loadTime = 10.0;
    };

    class PathCalculatorWithPenalty {
    private:
        double skipPenaltySum = 0;
        RobotParam param;
        std::vector<WayPoint> route;

    public:
        explicit PathCalculatorWithPenalty( std::vector<WayPoint> route) : skipPenaltySum(0), route(std::move(route)){};

        PathCalculatorWithPenalty(RobotParam param, std::vector<WayPoint> route) :
                skipPenaltySum(0),param(param), route(std::move(route)) {};

        static double calculateTransition(const WayPoint& w1, const WayPoint& w2);
        double calculateSkipCostSum(int id1, int id2);
        double operator()(int id1, int id2);
    };
}

#endif //TESTING_TASK_PATHFINDER_H
