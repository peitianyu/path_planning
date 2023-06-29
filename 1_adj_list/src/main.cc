#include <iostream>
#include <vector>
#include "graph.h"
using namespace std;

int main()
{
    // Create a graph and add some vertices
    // std::string method = "Dstar";
    // std::string method = "Astar";
    std::string method = "Dijstra";
    Graph graph(method); // 选择使用Dstar算法

    graph.AddVertex(0, {0.0, 0.0});
    graph.AddVertex(1, {1.0, 1.0});
    graph.AddVertex(2, {2.0, 2.0});
    graph.AddVertex(3, {3.0, 3.0});
    graph.AddVertex(4, {4.0, 4.0});

    // Add some edges to the graph
    graph.AddEdge(0, 1, 2);
    graph.AddEdge(0, 3, 1);
    graph.AddEdge(1, 2, 3);
    graph.AddEdge(1, 3, 4);
    graph.AddEdge(2, 4, 5);
    graph.AddEdge(3, 4, 6);

    // Find the shortest path from vertex 0 to vertex 4 using the A* algorithm
    vector<Vertex> path = graph.FindShortestPath(0, 4);
    cout << "Shortest path from 0 to 4 using the "<<method<<" algorithm: ";
    for (const auto &vertex : path)
    {
        cout << vertex.id << " ";
    }
    cout << endl;

    return 0;
}
