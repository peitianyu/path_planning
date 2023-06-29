#ifndef __GRAPH_H__
#define __GRAPH_H__
#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <unordered_map>
#include <algorithm>
#include <cmath>

struct Point2D
{
    float x;
    float y;
};

struct Vertex
{
    float weight;
    int id;
    Point2D point;
};

struct Comparator
{
    bool operator()(const Vertex &v1, const Vertex &v2) const
    {
        return v1.weight > v2.weight;
    }
};

struct Edge
{
    int src, dest;
    float weight;
};

class Graph
{
public:
    Graph(const std::string &method) : m_method(method)
    {}

    Graph() : m_method("Dijstktra")
    {}

    void Reset()
    {
        m_vertices.clear();
        m_edges.clear();
        m_adj_list.clear();
    }

    void AddVertex(int id, Point2D p)
    {
        constexpr float INF = 1e9;
        m_vertices.emplace_back(Vertex{INF, id, p});
    }

    void AddEdge(int src, int dest, float weight)
    {
        m_edges.emplace_back(Edge{src, dest, weight});
        m_adj_list[src].emplace_back(dest, weight);
    }

    int GetNumVertices() const
    {
        return m_vertices.size();
    }

    std::vector<Vertex> FindShortestPath(int start, int dest)
    {
        if (m_method == "Astar")
            return Astar(start, dest);
        else if (m_method == "Dstar")
            return Dstar(start, dest);
        else
            return Dijkstra(start, dest);
    }

private:
    std::vector<Vertex> Dijkstra(int start, int dest)
    {
        std::cout<<"----Dijkstra-----"<<std::endl;
        // Initialize the distance and predecessor arrays
        std::vector<float> dist(GetNumVertices(), 1e9);
        std::vector<int> pred(GetNumVertices(), -1);
        dist[start] = 0;

        // Initialize the priority queue
        std::priority_queue<Vertex, std::vector<Vertex>, Comparator> q;
        q.emplace(Vertex{0, start});

        // Process the vertices in the queue
        while (!q.empty())
        {
            Vertex u = q.top();
            q.pop();

            // Check the distance of each neighbor
            for (const auto &p : m_adj_list[u.id])
            {
                int v = p.first;
                float w = p.second;
                if (dist[v] > dist[u.id] + w)
                {
                    dist[v] = dist[u.id] + w;
                    pred[v] = u.id;
                    q.emplace(m_vertices[v]);
                }
            }
        }

        // Construct the shortest path from the predecessor array
        std::vector<Vertex> path;
        int cur = dest;
        while (cur != -1)
        {
            path.emplace_back(m_vertices[cur]);
            cur = pred[cur];
        }
        reverse(path.begin(), path.end());

        return path;
    }

    std::vector<Vertex> Astar(int start, int dest)
    {
        std::cout << "----Astar-----" << std::endl;
        // Initialize the distance and predecessor arrays
        std::vector<float> dist(GetNumVertices(), 1e9);
        std::vector<int> pred(GetNumVertices(), -1);
        dist[start] = 0;

        // Initialize the priority queue
        std::priority_queue<Vertex, std::vector<Vertex>, Comparator> q;
        q.emplace(Vertex{0, start});

        // Process the vertices in the queue
        while (!q.empty())
        {
            Vertex u = q.top();
            q.pop();

            // Check the distance of each neighbor
            for (const auto &p : m_adj_list[u.id])
            {
                int v = p.first;
                float w = p.second;
                if (dist[v] > dist[u.id] + w)
                {
                    dist[v] = dist[u.id] + w;
                    pred[v] = u.id;

                    m_vertices[v].weight = dist[v] + Heuristic(v, dest);
                    q.emplace(m_vertices[v]);
                }
            }
        }

        // Construct the shortest path from the predecessor array
        std::vector<Vertex> path;
        int cur = dest;
        while (cur != -1)
        {
            path.emplace_back(m_vertices[cur]);
            cur = pred[cur];
        }
        reverse(path.begin(), path.end());

        return path;
    }

    std::vector<Vertex> Dstar(int start, int dest)
    {
        std::cout << "----Dstar-----" << std::endl;
        // Initialize the distance and predecessor arrays
        std::vector<float> dist(GetNumVertices(), 1e9);
        std::vector<int> pred(GetNumVertices(), -1);
        dist[start] = 0;

        // Initialize the priority queue
        std::priority_queue<Vertex, std::vector<Vertex>, Comparator> q;
        q.emplace(Vertex{0, start});

        // Process the vertices in the queue
        while (!q.empty())
        {
            Vertex u = q.top();
            q.pop();

            // Check the distance of each neighbor
            for (const auto &p : m_adj_list[u.id])
            {
                int v = p.first;
                float w = p.second;
                float cost = w + Heuristic(v, dest);
                if (dist[v] > dist[u.id] + cost)
                {
                    dist[v] = dist[u.id] + cost;
                    pred[v] = u.id;
                    q.emplace(m_vertices[v]);
                }
            }
        }

        // Construct the shortest path from the predecessor array
        std::vector<Vertex> path;
        int cur = dest;
        while (cur != -1)
        {
            path.emplace_back(m_vertices[cur]);
            cur = pred[cur];
        }
        reverse(path.begin(), path.end());

        return path;
    }

    float Heuristic(int u, int v)
    {
        // Example: Euclidean distance heuristic
        float dx = m_vertices[u].point.x - m_vertices[v].point.x;
        float dy = m_vertices[u].point.y - m_vertices[v].point.y;
        return std::sqrt(dx * dx + dy * dy);
    }

private :
    std::string m_method;
    std::vector<Vertex> m_vertices;
    std::vector<Edge> m_edges;
    std::unordered_map<int, std::vector<std::pair<int, float>>> m_adj_list;
};

#endif //__GRAPH_H__