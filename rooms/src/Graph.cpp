//
// Created by usuario on 13/12/23.
//

#include "Graph.h"
#include <ranges>

Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::add_node()
{
    nodes.push_back(nodes.size());
    return nodes.size();
}

int Graph::add_edge(int n1, int n2) {
    if (std::ranges::find(nodes, n1) != nodes.end() and
        std::ranges::find(nodes, n2) != nodes.end() and
        std::ranges::find(edges, std::make_pair(n1, n2)) == edges.end())
    {
        edges.emplace_back(n1, n2);
        return 1;
    }
    return -1;
}


void Graph::print()
{
    for (const auto &n : nodes)
    {
        qInfo() << n << " " ;
    }

    qInfo() << "\n";


    for (const auto &e : edges)
    {
        qInfo() << e.first << " "  << e.second;
    }

    qInfo() << "\n";

}