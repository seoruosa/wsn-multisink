#pragma once

#include <vector>

// create an adjacency matrix given a list of edges (pair of nodes)
std::vector<std::vector<int>> adj_matrix_from_edges(std::vector<std::vector<int>> edges, int number_nodes)
{
    std::vector<std::vector<int>> adj_matrix(number_nodes, std::vector<int>(number_nodes, 0));

    for (auto &edge : edges)
    {
        auto from = edge[0];
        auto to = edge[1];

        adj_matrix[from][to] = 1;
        adj_matrix[to][from] = 1;
    }
    
    return adj_matrix;
}

// return a square matrix
template <typename T>
std::vector<std::vector<T>> square_matrix(int size, T value)
{
    std::vector<std::vector<T>> matrix(size, std::vector<T>(size, value));

    return matrix;
}