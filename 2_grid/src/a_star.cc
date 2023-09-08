#include"a_star.h"

 
Astar::Astar(const Option &p, const std::string &grid_file)
    : Grid(p, grid_file)
{}

std::vector<Point> Astar::FindShorestPathFunc(const Point &start, const Point &dest) 
{
    if(!VerifyNode(start) || !VerifyNode(dest)) return std::vector<Point>();

    std::set<Point> closed_list;

    auto cmp = [](const Node *left, const Node *right) { return left->SumCost() > right->SumCost(); };
    std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> open_list(cmp);

    open_list.push(new Node(start)); // 给一个起点
    while (!open_list.empty())
    {
        Node *curr_node = open_list.top();
        open_list.pop();

        // 找到目标节点, 返回路径
        if (curr_node->cmp(dest)) 
        {
            std::cout << "closed_list.size(): " << closed_list.size() << std::endl;
            return curr_node->Path();
        }
           
        // 更新closed_list
        if(closed_list.find(curr_node->pos) != closed_list.end())   continue;  
        else                                                closed_list.insert(curr_node->pos);        

        // 更新open_list
        static std::vector<Node> movements = GetMotionModel(); // 四近邻/八近邻
        for (const Node &movement : movements)
        {
            Point new_pos = Point(curr_node->pos.x + movement.pos.x, curr_node->pos.y + movement.pos.y);
            
            // 若节点已经被探索过或者是障碍物则跳过
            if (!VerifyNode(new_pos)) continue;
               
            // 将更新后的节点加入openList
            // open_list.push(new Node(new_pos, GetMovementCost(new_pos) , Heuristic(new_pos, dest), curr_node));      // Astar
            open_list.push(new Node(new_pos, GetMovementCost(new_pos), 0, curr_node));                                 // Dijkstra
        }
    }

    return std::vector<Point>();
}

float Astar::GetMovementCost(const Point &pos)
{
    if (IsFree(pos))
        return 1.0;

    if (IsUnknown(pos))
        return 10.0;

    return 0.0;
}

float Astar::Heuristic(const Point &pos, const Point &dest)
{
    float dx = pos.x - dest.x;
    float dy = pos.y - dest.y;
    return std::sqrt(dx * dx + dy * dy);
}

// 四近邻/八近邻(FindNeighbors())
std::vector<Node> Astar::GetMotionModel()
{
    return {Node(Point(1, 0)),
            Node(Point(0, 1)),
            Node(Point(-1, 0)),
            Node(Point(0, -1))};

    // return {Node(Point(1, 0), 1),
    //         Node(Point(0, 1), 1),
    //         Node(Point(-1, 0), 1),
    //         Node(Point(0, -1), 1),
    //         Node(Point(-1, -1), std::sqrt(2)),
    //         Node(Point(-1, 1), std::sqrt(2)),
    //         Node(Point(1, -1), std::sqrt(2)),
    //         Node(Point(1, 1), std::sqrt(2))};
}
