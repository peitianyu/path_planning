#include <iostream>
#include <memory>
#include "a_star.h"
int main()
{
    std::shared_ptr<Astar> a_star;
    a_star = std::make_shared<Astar>(Grid::Option(), "../data/sample.grid");

    // a_star->Visual();
    std::vector<Point> path = a_star->FindShorestPath(Point(0, 0), Point(19, 19));

    if(path.empty()) std::cout<<"find none path!"<<std::endl;
    a_star->Visual();
    return 0;
}