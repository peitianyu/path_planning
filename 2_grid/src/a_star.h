#include"grid.h"

struct Node
{
    Point pos;
    float g;
    float h;
    Node *pre_node;

    Node(Point _pos)
        : pos(_pos), g(0), h(0), pre_node(nullptr)
    {}

    Node(Point _pos, float _g, float _h, Node *_pre_node)
        : pos(_pos), g(_g), h(_h), pre_node(_pre_node)
    {
        g += pre_node->g;
    }

    bool cmp(Point new_pos)
    {
        return (pos.x == new_pos.x) && (pos.y == new_pos.y);
    }

    float SumCost() const
    {
        return g + h;
    }

    std::vector<Point> Path() 
    {
        std::vector<Point> path;
        Node *curr_node = this;
        while (curr_node != NULL) {
            path.push_back(curr_node->pos);
            curr_node = curr_node->pre_node;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

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