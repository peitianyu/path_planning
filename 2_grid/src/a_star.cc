#include"a_star.h"

 
Astar::Astar(const Option &p, const std::string &grid_file)
    : Grid(p, grid_file)
{}

std::vector<Point> Astar::FindShorestPathFunc(const Point &start, const Point &dest) 
{
    std::set<Point> visited;

    auto cmp = [](const Node *left, const Node *right) { return left->sum_cost > right->sum_cost; };
    std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> pq(cmp);

    Node *start_node = new Node(start, 0, NULL);
    std::vector<Node> movements = GetMotionModel();
    pq.push(start_node);

    while (!pq.empty())
    {
        Node *curr_node = pq.top();
        pq.pop();

        if (curr_node->cmp(dest))
        {
            std::vector<Point> path;
            while (curr_node != NULL)
            {
                path.push_back(curr_node->pos);
                curr_node = curr_node->pre_node;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (visited.find(curr_node->pos) != visited.end())
            continue;
        visited.insert(curr_node->pos);

        for (const Node &movement : movements)
        {
            Point new_pos = Point(curr_node->pos.x + movement.pos.x, curr_node->pos.y + movement.pos.y);
            if (!VerifyNode(new_pos))
                continue;

            float cost = curr_node->sum_cost + GetMovementCost(new_pos);
            float heuristic = Heuristic(new_pos, dest);
            float sum_cost = cost + heuristic;

            Node *successor = new Node(new_pos, sum_cost, curr_node);
            pq.push(successor);
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

std::vector<Node> Astar::GetMotionModel()
{
    return {Node(Point(1, 0), 1),
            Node(Point(0, 1), 1),
            Node(Point(-1, 0), 1),
            Node(Point(0, -1), 1)};

    // return {Node(Point(1, 0), 1),
    //         Node(Point(0, 1), 1),
    //         Node(Point(-1, 0), 1),
    //         Node(Point(0, -1), 1),
    //         Node(Point(-1, -1), std::sqrt(2)),
    //         Node(Point(-1, 1), std::sqrt(2)),
    //         Node(Point(1, -1), std::sqrt(2)),
    //         Node(Point(1, 1), std::sqrt(2))};
}
