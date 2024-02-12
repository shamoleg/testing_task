//
// Created by sham on 2/11/24.
//
#include <iostream>
#include <PathFinder.h>

int main(){
    while (true){
        auto creatorPathFinder = pf::createPathFinderWayPoint2D(std::cin);
        if(creatorPathFinder.has_value()){

            float robotVelocity = 2;
            float loadingTime = 10;
            pf::PathFinderParams params{robotVelocity,loadingTime};
            pf::PathFinder pathFinder(params);

            pathFinder.setNodes(creatorPathFinder.value());

            std::cout << std::roundf(pathFinder.getShortestTime() * 1000)/1000 << std::endl;
        } else{
            break;
        }
    }
}