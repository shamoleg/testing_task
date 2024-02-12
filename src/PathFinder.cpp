//
// Created by sham on 2/12/24.
//
#include "PathFinder.h"

namespace pf{

void PathFinder::setNodes(const RouteWayPoint &route){
    std::vector<float> timeToPoint(route.size(), std::numeric_limits<float>::max());
    std::vector<int> previous(route.size(), -1);

    timeToPoint[0] = 0;
    for (int currPoint = 0; currPoint < route.size(); currPoint++) {
        float penaltySum = 0;
        for (int nextPoint = currPoint + 1; nextPoint < route.size(); nextPoint++) {
            float cost = calcDistance(route[currPoint], route[nextPoint])
                            / params.robotVelocity + params.loadingTime + penaltySum;
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
}

float PathFinder::getShortestTime() const  {
    return shortestTime;
}

RouteWayPoint PathFinder::getPathWithShortestTime() const{
    return shortestPath;
}

std::optional<RouteWayPoint> createPathFinderWayPoint2D(std::istream &stream){
    int numberOfPointInIO = 0;
    stream >> numberOfPointInIO;
    if (numberOfPointInIO == 0){
        return std::nullopt;
    }

    std::vector<WayPoint> route(numberOfPointInIO + 2);
    route[0] = WayPointZeroPose;
    for(int i = 1; i <= numberOfPointInIO; i++){
        stream >> route[i].pos.x >> route[i].pos.y >> route[i].penaltySkipping;
    }
    route.back() = WayPointFinishPose;

    return route;
}

PathFinder::PathFinder(PathFinderParams params):
        params(params),
        shortestTime(std::numeric_limits<float>::max())
{};

float calcDistance(const WayPoint& p1, const WayPoint& p2){
    return sqrt((std::pow(p1.pos.x - p2.pos.x, 2) + std::pow(p1.pos.y - p2.pos.y, 2)));
}

}