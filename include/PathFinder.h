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

namespace pf {

struct WayPoint{
    struct Position {
        float x;
        float y;
    };
    Position pos;
    float penaltySkipping;
};

float calcDistance(const WayPoint& p1, const WayPoint& p2);

constexpr WayPoint WayPointZeroPose{0, 0, 0};
constexpr WayPoint WayPointFinishPose{100, 100, 0};

struct PathFinderParams{
    float robotVelocity;
    float loadingTime;
};

using RouteWayPoint = std::vector<WayPoint>;

class PathFinder {
public:
    explicit PathFinder(PathFinderParams params);
    void setNodes(const RouteWayPoint &route);
    float getShortestTime() const;
    RouteWayPoint getPathWithShortestTime() const;

private:

    PathFinderParams params;
    float shortestTime;
    RouteWayPoint shortestPath;
};

std::optional<RouteWayPoint> createPathFinderWayPoint2D(std::istream &stream);

}

#endif //TESTING_TASK_PATHFINDER_H
