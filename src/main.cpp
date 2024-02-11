//
// Created by sham on 2/11/24.
//
#include <iostream>
#include <memory>
#include <PathFinder.h>

class TestRobot{
    // ...
    // ...
    std::unique_ptr<pf::IPathFinder<pf::Position2D>> pathFinder;
};

int main(){
    std::vector<pf::WayPoint2D> v{
            {0,0,0},
            {30, 30, 90},
            {60, 60, 80},
            {10, 90, 10},
            {100,100,0},
    };
    pf::WayPoint s{1,1,1};
}