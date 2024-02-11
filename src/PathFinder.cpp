//
// Created by sham on 2/11/24.
//

#include "PathFinder.h"
namespace pf {

PathFinder::PathFinder(PathFinderParams params) :
        params(params),
        shortestTime(std::numeric_limits<float>::max())
        {}

void PathFinder::setNodes(const std::vector<WayPoint2D> &route) {
}


static float calcDistance(const WayPoint2D& p1, const WayPoint2D& p2){
    return std::sqrt(static_cast<float>(std::pow(p1.pos2D.x - p2.pos2D.x, 2) +
                                           std::pow(p1.pos2D.y - p2.pos2D.y, 2)));
}

}