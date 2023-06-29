#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <Eigen/Core>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define SHOW_PATH 1

struct Point
{
    Point(int _x, int _y) : x(_x), y(_y) {}
    int x;
    int y;

    bool operator<(const Point &other) const
    {
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
};

struct Node
{
    Point pos;
    float sum_cost;
    Node *pre_node;

    Node(Point _pos, float _sum_cost = 0, Node *_pre_node = NULL)
        : pos(_pos), sum_cost(_sum_cost), pre_node(_pre_node)
    {}

    bool cmp(Point new_pos)
    {
        return (pos.x == new_pos.x) && (pos.y == new_pos.y);
    }
};

class Grid
{
public:
    struct Option
    {
        int free = 0;
        int wall = 1;
        int unknown = 2;
        int path = 3;
    };

    Grid(const Option &p, const std::string &grid_file);

    bool VerifyNode(const Point &pos);

    bool IsValid(const Point &pos);

    bool IsWall(const Point &pos);

    bool IsFree(const Point &pos);

    bool IsUnknown(const Point &pos);

    void Visual();

    const Eigen::MatrixXi &data();

    std::vector<Point> FindShorestPath(const Point &start, const Point &dest);
protected:
    virtual std::vector<Point> FindShorestPathFunc(const Point &start, const Point &dest) = 0;

private:
    void ReadGridFile(const std::string &grid_file);

private:
    Option m_option;
    Eigen::MatrixXi m_grid;
};