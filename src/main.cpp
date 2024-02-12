//
// Created by sham on 2/11/24.
//

#include "PathFinder.h"

#include <sstream>
#include <iostream>
#include <cassert>

void testingTask(std::istream& in, std::ostream& out){
    while (true){
        auto creatorPathFinder = pf::createRouteWayPoint(in);
        if(creatorPathFinder.has_value()){

            float robotVelocity = 2;
            float loadingTime = 10;
            pf::PathFinderParams params{robotVelocity,loadingTime};
            pf::PathFinder pathFinder(params);

            pathFinder.calculatePath(creatorPathFinder.value());

            out << std::roundf(pathFinder.getShortestTime() * 1000)/1000 << std::endl;
        } else{
            break;
        }
    }
}

void testPathFinder(){
    std::stringstream out;
    std::stringstream in;
    in << "1\n"
           "50 50 20\n"
           "3\n"
           "30 30 90\n"
           "60 60 80\n"
           "10 90 100\n"
           "3\n"
           "30 30 90\n"
           "60 60 80\n"
           "10 90 10\n"
           "0";

    testingTask(in, out);
    std::string referentResult = "90.711\n156.858\n110.711\n";
    // Тут лучше использовать GoogleTest, но не хотел тянуть зависимость
    assert(referentResult == out.str());
}

int main(){
    testingTask(std::cin, std::cout);
}