//
// Created by sham on 2/16/24.
//
#include <vector>
#include <string>

#ifndef TESTING_TASK_DATASTRUCTURES_H
#define TESTING_TASK_DATASTRUCTURES_H

namespace pf {

    struct Point2D {
        double x;
        double y;

        Point2D() : x(0.0), y(0.0) {}

        Point2D(double x, double y) : x(x), y(y) {}
    };

    struct Point3D {
        double x;
        double y;
        double z;

        Point3D() : x(0.0), y(0.0), z(0.0) {}

        Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
    };

    struct Quaternion {
        double x;
        double y;
        double z;
        double w;

        Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}

        Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    };

    struct Pose3D {
        Point3D location;
        Quaternion orientation;

        Pose3D() : location(Point3D()), orientation() {}

        Pose3D(Point3D location, Quaternion orientation) : location(location), orientation(orientation) {}
    };

    struct WayPoint {
        Point2D pose2D;
        double skipPenalty;

        WayPoint() : pose2D(), skipPenalty(0.0) {}

        WayPoint(Point2D pose2D, double skipPenalty) : pose2D(pose2D), skipPenalty(skipPenalty) {}
    };

    struct MapMeta {
        double width;
        double height;
        std::string mapName;
        Pose3D poseInRealWorld;

        MapMeta() : width(0.0), height(0.0), poseInRealWorld() {};

        MapMeta(double width, double height, std::string mapName, Pose3D poseInRealWorld = Pose3D())
                : width(width), height(height), mapName(std::move(mapName)), poseInRealWorld(poseInRealWorld) {}
    };

    class IGraph {
    public:
        virtual std::vector<int> getNeighborById(int id) const = 0;

        virtual size_t getSizeUniqNode() const = 0;

        virtual ~IGraph() = default;
    };

}
#endif //TESTING_TASK_DATASTRUCTURES_H
