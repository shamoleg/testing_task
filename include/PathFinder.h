//
// Created by sham on 2/11/24.
//
#include <vector>

#ifndef TESTING_TASK_PATHFINDER_H
#define TESTING_TASK_PATHFINDER_H

namespace pf {

template<typename PointType>
class IPathFinder {
public:
    virtual ~IPathFinder() = default;

    virtual void setNodes(const std::vector<PointType> &route) = 0;
    virtual float getShortestTime() = 0;
    virtual std::vector<PointType> getPathWithShortestTime() = 0;
};

struct WayPoint2D{
    struct Position2D {
        float x;
        float y;
    };
    Position2D pos2D;
    float penaltySkipping;
};

struct PathFinderParams{
    float robotVelocity;
    float loadingTime;
};

using GraphTable = std::vector<std::vector<std::pair<int, float>>>;

class PathFinder : public IPathFinder<WayPoint2D> {
public:
    PathFinder(PathFinderParams params);
    void setNodes(const std::vector<WayPoint2D> &route) final;
    float getShortestTime() final {return {};};
    std::vector<WayPoint2D> getPathWithShortestTime() final{return {};};

private:

    GraphTable transitionsTable;
};


}
#endif //TESTING_TASK_PATHFINDER_H
