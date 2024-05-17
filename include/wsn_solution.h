#pragma once

// #include "wsn_data.h"
#include <vector>
#include <string>
#include "util_solution.h"

class WSN_solution
{
private:
    const WSN_data &instance;

public:
    WSN_solution(const WSN_data &instance);
    ~WSN_solution();
    template <class T>
    bool is_valid(std::vector<std::vector<int>> &edges, T &masters,
                  T &bridges);
    bool is_valid(std::string filepath);
};

WSN_solution::WSN_solution(const WSN_data &instance) : instance(instance)
{
}

WSN_solution::~WSN_solution()
{
}

template <class T>
bool WSN_solution::is_valid(std::vector<std::vector<int>> &edges, T &masters,
                            T &bridges)
{
    std::set<int> masters_set(masters.begin(), masters.end());
    std::set<int> bridges_set(bridges.begin(), bridges.end());

    auto adj_forest = adj_list_forest(edges, instance);

    auto solution_valid = is_solution_valid(instance, adj_forest, masters_set, bridges_set);

    return solution_valid;
}
