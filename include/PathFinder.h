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

template<typename RouteType>
class IPathFinder {
public:
    virtual ~IPathFinder() = default;

    virtual void setNodes(const RouteType &route) = 0;
    virtual float getShortestTime() = 0;
    virtual RouteType getPathWithShortestTime() = 0;
};

struct PathFinderParams{
    float robotVelocity;
    float loadingTime;
};

template<typename RouteType>
class PathFinder : public IPathFinder<RouteType> {
public:
    explicit PathFinder(PathFinderParams params):
            params(params),
            shortestTime(std::numeric_limits<float>::max())
            {};

    void setNodes(const RouteType &route) final{
        std::vector<float> timeToPoint(route.size(), std::numeric_limits<float>::max());
        std::vector<int> previous(route.size(), -1);

        timeToPoint[0] = 0;
        for (int currPoint = 0; currPoint < route.size(); currPoint++) {
            int penaltySum = 0;
            for (int nextPoint = currPoint + 1; nextPoint < route.size(); nextPoint++) {
                float cost = calcDistance(route[currPoint], route[nextPoint])
                             / params.robotVelocity + params.loadingTime + static_cast<float>(penaltySum);
                if (timeToPoint[currPoint] + cost < timeToPoint[nextPoint]) {
                    timeToPoint[nextPoint] = timeToPoint[currPoint] + cost;
                    previous[nextPoint] = currPoint;
                }
                penaltySum += (route[nextPoint].penaltySkipping);
            }
        }
        shortestTime = timeToPoint.back();

        int prevIndex = previous.back();
        shortestPath.clear();
        shortestPath.push_back(route.back());
        while (prevIndex != -1){
            shortestPath.push_back(route[prevIndex]);
            prevIndex = previous[prevIndex];
        }
        std::reverse(shortestPath.begin(), shortestPath.end());
    };

    float getShortestTime() final {
        return shortestTime;
    };

    RouteType getPathWithShortestTime() {
            return shortestPath;
    };

private:

    PathFinderParams params;
    float shortestTime;
    RouteType shortestPath;
};

struct WayPoint2D{
    struct Position2D {
        int x;
        int y;
    };
    Position2D pos2D;
    int penaltySkipping;
};

float calcDistance(const WayPoint2D& p1, const WayPoint2D& p2){
    return std::sqrt(static_cast<float>(std::pow(p1.pos2D.x - p2.pos2D.x, 2) +
                                           std::pow(p1.pos2D.y - p2.pos2D.y, 2)));
}

constexpr WayPoint2D WayPointZeroPose{0, 0, 0};
constexpr WayPoint2D WayPointFinishPose{100, 100, 0};

using RouteWayPoint2D = std::vector<WayPoint2D>;
using PathFinderWayPoint2D = PathFinder<RouteWayPoint2D>;

std::optional<RouteWayPoint2D> createPathFinderWayPoint2D(std::istream &stream){
    int numberOfPointInIO = 0;
    stream >> numberOfPointInIO;
    if (numberOfPointInIO == 0){
        return std::nullopt;
    }

    std::vector<WayPoint2D> route(numberOfPointInIO + 2);
    route[0] = WayPointZeroPose;
    for(int i = 1; i <= numberOfPointInIO; i++){
        stream >> route[i].pos2D.x >> route[i].pos2D.y >> route[i].penaltySkipping;
    }
    route.back() = WayPointFinishPose;

    return route;
}

}

#endif //TESTING_TASK_PATHFINDER_H
