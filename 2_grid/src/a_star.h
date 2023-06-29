#include"grid.h"

class Astar : public Grid
{
public: 
    Astar(const Option &p, const std::string &grid_file);

protected:
    virtual std::vector<Point> FindShorestPathFunc(const Point &start, const Point &dest) override;
private:
    float GetMovementCost(const Point &pos);

    float Heuristic(const Point &pos, const Point &dest);

    std::vector<Node> GetMotionModel();
};