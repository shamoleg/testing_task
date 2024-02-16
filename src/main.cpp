//
// Created by sham on 2/11/24.
//

#include "PathFinder.h"
#include "Algorithm.h"


#include <sstream>
#include <iostream>
#include <cassert>

void testingTask(std::istream& in, std::ostream& out);
void testPathFinder();
void testDijkstraAlgorithm2();

int main(){
    testingTask(std::cin, std::cout);
}

void testingTask(std::istream& in, std::ostream& out){
    while (true){
        int numNodes = 0;
        in >> numNodes;
        if(numNodes == 0){
            break;
        }

        // по сути при создании можно было бы считать вес каждого перехода,
        // но решил оставить эту логику только в алгоритме
        auto dag = pf::DirectedAcyclicGraph(numNodes+2);
        for(int i = 0; i < dag.getSizeUniqNode(); i++){
            for(int j = i+1; j < dag.getSizeUniqNode(); j++){
                dag.addDirectedEdge(i,j);
            }
        }

        std::vector<pf::WayPoint> route;
        route.push_back({{0,0},0});
        for(int i = 0; i < numNodes; i++){
            pf::WayPoint point;
            in >> point.pose2D.x >> point.pose2D.y >> point.skipPenalty;
            route.push_back(point);
        }
        route.push_back({{100,100},0});

        // в этом функторе реализованна логика подсчета стоймости перехода от точки к точке
        // При изменении условий местности, например появления препятствий, данный класс можно заменить
        // на другой
        pf::PathCalculatorWithPenalty pc({2, 10}, route);


        // не смог придумать алгоритм который бы был лучше O(V + E), где V - количество вершин, а E - количество рёбер
        // у дейкстры асимптотика хуже так как идет добавление элемента в приоритетную очередь
        auto [resRoute, cost] =
                pf::algorithms::weightedTopologicalSort(0, route.size()-1 , dag,  pc);
        cost = std::round(1000 * cost) / 1000;
        out << cost << std::endl;
    }
}


void testDijkstraAlgorithm2() {
    auto gridGraph = pf::GridGraph(5, 5, {});
    auto goal = gridGraph.getIdByRowCol(3, 4);
    gridGraph.setObstacles(2,0) ;

    auto awesomeFunc = [](float, float ) -> float  { return 1;};

    auto [route, cost] = pf::algorithms::dijkstraAlgorithm2(0, goal, gridGraph, awesomeFunc);
    std::cout << "this is cost " << cost << std::endl;
    for(const auto& id : route){
        auto [row, col] = gridGraph.getRowColById(id);
        std::cout << row << " " << col << std::endl;
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