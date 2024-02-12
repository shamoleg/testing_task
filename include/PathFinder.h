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

class IPathFinder {
public:
    virtual ~IPathFinder() = default;

    virtual void calculatePath(const RouteWayPoint &route) = 0;
    virtual float getShortestTime() const = 0;
    virtual RouteWayPoint getPathWithShortestTime() const = 0;
};

class PathFinder : IPathFinder {
public:
    explicit PathFinder(PathFinderParams params);
    void calculatePath(const RouteWayPoint &route) final;
    float getShortestTime() const final;
    RouteWayPoint getPathWithShortestTime() const final;

private:

    PathFinderParams params;
    float shortestTime;
    RouteWayPoint shortestPath;
};

//Дополнительный пример реализации
class PathFinderDijkstra : IPathFinder {
public:
    explicit PathFinderDijkstra(PathFinderParams params);
    void calculatePath(const RouteWayPoint &route) final;
    float getShortestTime() const final;
    RouteWayPoint getPathWithShortestTime() const final;

private:

    PathFinderParams params;
    float shortestTime;
    RouteWayPoint shortestPath;
};


std::optional<RouteWayPoint> createRouteWayPoint(std::istream &stream);

}

#endif //TESTING_TASK_PATHFINDER_H
