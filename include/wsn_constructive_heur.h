#pragma once

#include "wsn_data.h"
#include <list>

#include <iostream>
#include <algorithm>
#include <chrono>
#include <random>
#include "wsn_solution.h"

class SolutionHeuristic
{
public:
    SolutionHeuristic(std::vector<std::vector<int>> &edges, std::vector<int> &masters,
                      std::vector<int> &bridges);

    std::vector<std::vector<int>> edges;
    std::vector<int> masters;
    std::vector<int> bridges;
};

// namespace util
// {
//     std::vector<int> ordered_vec(int size);

//     std::set<int> create_set(int size);

//     namespace print
//     {
//         template <class T>
//         void print_vec(T v, std::string message = "");

//         template <class T>
//         void print_matrix(std::vector<T> matrix);

//         void line(std::string str);

//         void line();

//     } // namespace print

// } // namespace util

class WSNConstructiveHeuristic
{
public:
    WSNConstructiveHeuristic(WSN_data &instance);
    template <class T>
    inline SolutionHeuristic build(T order);
    SolutionHeuristic solve(unsigned seed = std::chrono::system_clock::now().time_since_epoch().count());
    double weight_of_solution();

private:
    WSN_data instance;
    void clean_data(); // before build a solution, need to clean the data
    void create_master(int node, int idx_sink);
    void create_sink(int node);
    void insert_node(int node, int adj);
    bool insert_bridge(int bridge_node, int adj);
    bool insert_master(int master_node, int adj);
    bool node_can_be_master(int node);
    bool node_can_be_bridge_at_adj(int node, int adj);

    std::vector<std::vector<int>> edges;
    std::vector<int> is_master;
    std::vector<int> is_bridge;
    std::vector<int> nodes_sink; // contain the index of sink that node is associated

    std::vector<std::vector<int>> nodes_of_sink; // contains the nodes associated with the sink in order of insertion on tree
    std::vector<double> weight_of_sink;
    std::vector<int> sinks;
    std::set<int> masters;
    std::set<int> bridges;
    std::set<int> not_dominant;

    std::set<int> not_adj;
    std::vector<std::set<int>> build_graph;
    WSN_solution checker;

    void print_info();
};

SolutionHeuristic::SolutionHeuristic(std::vector<std::vector<int>> &edges, std::vector<int> &masters,
                                     std::vector<int> &bridges) : edges(edges), masters(masters),
                                                                  bridges(bridges)
{
}

namespace util
{
    std::vector<int> ordered_vec(int size)
    {
        std::vector<int> vec(size, 0);

        for (int i = 0; i < size; i++)
        {
            vec[i] = i;
        }

        return vec;
    }

    std::set<int> create_set(int size)
    {
        auto vec = ordered_vec(size);

        return std::set<int>(vec.begin(), vec.end());
    }

    namespace print
    {
        template <class T>
        void print_vec(T v, std::string message)
        {
            std::cout << message << "\t>> ";
            for (auto &el : v)
            {
                std::cout << el << " ";
            }
            std::cout << std::endl;
        }

        template <class T>
        void print_matrix(std::vector<T> matrix)
        {
            for (int i = 0; i < matrix.size(); i++)
            {
                std::cout << i << " >> ";
                for (auto &el : matrix[i])
                {
                    std::cout << el << " ";
                }

                std::cout << std::endl;
            }
        };

        void line(std::string str)
        {
            for (size_t i = 0; i < 30; i++)
                std::cout << str;
            std::cout << std::endl;
        }

        void line()
        {
            line("*");
        }

    } // namespace print

} // namespace util

WSNConstructiveHeuristic::WSNConstructiveHeuristic(WSN_data &instance) : instance(instance), checker(instance)
{
    clean_data();
}

inline void WSNConstructiveHeuristic::clean_data()
{
    is_master = std::vector<int>(instance.n, 0);
    is_bridge = std::vector<int>(instance.n, 0);
    nodes_sink = std::vector<int>(instance.n, -1);
    sinks = std::vector<int>({});

    masters = std::set<int>({});
    bridges = std::set<int>({});
    not_adj = std::set<int>(util::create_set(instance.n));
    not_dominant = std::set<int>(util::create_set(instance.n));

    nodes_of_sink = std::vector<std::vector<int>>({size_t(instance.number_trees), std::vector<int>({})});
    weight_of_sink = std::vector<double>(size_t(instance.number_trees), 0);
    edges = std::vector<std::vector<int>>({});
    build_graph = std::vector<std::set<int>>({size_t(instance.n), std::set<int>({})});
}

inline void WSNConstructiveHeuristic::create_master(int node, int idx_sink)
{
    is_master[node] = 1;
    nodes_sink[node] = idx_sink;

    masters.insert(node);

    not_adj.erase(node);
    not_dominant.erase(node);
    for (auto &el : instance.adj_list_from_v[node])
    {
        not_adj.erase(el);
    }
}

inline void WSNConstructiveHeuristic::create_sink(int node)
{
    sinks.push_back(node);
    int idx_of_sink = sinks.size() - 1;
    nodes_of_sink[idx_of_sink].push_back(node);

    create_master(node, idx_of_sink);
}

inline void WSNConstructiveHeuristic::insert_node(int node, int adj)
{
    int idx_sink = nodes_sink[adj];

    nodes_of_sink[idx_sink].push_back(node);
    edges.push_back({adj, node});
    weight_of_sink[idx_sink] += instance.weight[adj][node];

    build_graph[adj].insert(node);
}

inline bool WSNConstructiveHeuristic::insert_bridge(int bridge_node, int adj)
{
    int idx_sink = nodes_sink[adj];

    if (node_can_be_bridge_at_adj(bridge_node, adj))
    {
        is_bridge[bridge_node] = 1;
        bridges.insert(bridge_node);
        nodes_sink[bridge_node] = idx_sink;

        insert_node(bridge_node, adj);

        not_adj.erase(bridge_node);
        not_dominant.erase(bridge_node);

        return true;
    }

    return false;
}

inline bool WSNConstructiveHeuristic::insert_master(int cand_master_node, int adj)
{
    bool is_candidate_adj_to_adj_node = (instance.adj_list_from_v[adj].find(cand_master_node) != instance.adj_list_from_v[adj].end());

    if (!node_can_be_master(cand_master_node) || !is_candidate_adj_to_adj_node)
    {
        return false;
    }
    else
    {
        int idx_sink = nodes_sink[adj];

        create_master(cand_master_node, idx_sink);

        insert_node(cand_master_node, adj);

        return true;
    }
}

inline bool WSNConstructiveHeuristic::node_can_be_master(int node)
{
    for (auto &el : instance.adj_list_from_v[node])
    {
        if (is_master[el] == 1)
        {
            return false;
        }
    }

    return true;
}

inline bool WSNConstructiveHeuristic::node_can_be_bridge_at_adj(int node, int adj)
{
    int idx_sink = nodes_sink[adj];
    // check inside the same sink

    // node is adjacent to adj
    bool is_node_adj_to_adj = (instance.adj_list_from_v[adj].find(node) != instance.adj_list_from_v[adj].end());
    // adj is a bridge, than node cant be bridge
    bool adj_is_bridge = is_bridge[adj] == 1;

    if (!is_node_adj_to_adj || adj_is_bridge)
    {
        return false;
    }

    // node have some adjacent node that can be master
    for (auto &master_cand : instance.adj_list_from_v[node])
    {
        if ((master_cand != adj) & node_can_be_master(master_cand) & (is_master[master_cand] != 1))
        {
            return true;
        }
    }

    return false;
}

inline SolutionHeuristic WSNConstructiveHeuristic::solve(unsigned seed)
{
    auto order = util::ordered_vec(instance.n);
    auto rng = std::default_random_engine(seed);

    std::shuffle(order.begin(), order.end(), rng);

    auto best_order = order;
    double weight_best_sol = std::numeric_limits<double>::max();

    for (int i = 0; i < 1000; i++)
    {

        auto sol = build(order);
        auto weight_sol = weight_of_solution();

        // std::cout << "weight: " << weight_sol << std::endl;
        // util::print::print_vec(order, "order sol");

        // auto masters = 

        if (weight_sol < weight_best_sol & checker.is_valid(edges, masters, bridges))
        {
            best_order = order;
            weight_best_sol = weight_sol;
            // std::cout << ">>> Find a better solution" << std::endl;
        }

        std::shuffle(order.begin(), order.end(), rng);
    }

    util::print::print_vec(best_order, ">> BEST FOUND ORDER");

    return build(best_order);
}

template <class T>
inline SolutionHeuristic WSNConstructiveHeuristic::build(T order)
{
    clean_data();

    std::list<int> nodes_list(order.begin(), order.end());

    // choose sinks
    for (std::list<int>::iterator node_it = nodes_list.begin(); ((node_it != nodes_list.end()) & (sinks.size() < instance.number_trees));)
    {
        bool node_is_neighb_to_some_sink = false;

        for (auto &sink : sinks)
        {
            if (instance.adj_list_from_v[sink].find(*node_it) != instance.adj_list_from_v[sink].end())
            {
                node_is_neighb_to_some_sink = true;
            }
        }

        if (!node_is_neighb_to_some_sink)
        {
            create_sink(*node_it);

            node_it = nodes_list.erase(node_it);
        }
        else
        {
            ++node_it;
        }
    }

    std::list<int>::iterator node_it = nodes_list.begin();
    int actual_sink_id = 0;
    int num_iter = 0;
    while (!nodes_list.empty() & !not_adj.empty() & (num_iter < instance.number_trees + 1))
    {
        bool is_bridge_inserted = false;
        bool is_master_inserted = false;
        node_it = nodes_list.begin();

        // try to insert an bridge
        while ((node_it != nodes_list.end()) & !is_bridge_inserted)
        {
            // choose where to put the bridge
            std::vector<int>::iterator nodes_sink_it = nodes_of_sink[actual_sink_id].begin();
            while ((nodes_sink_it != nodes_of_sink[actual_sink_id].end()) & !is_bridge_inserted)
            {
                is_bridge_inserted = insert_bridge(*node_it, *nodes_sink_it);

                if (!is_bridge_inserted)
                {
                    ++nodes_sink_it;
                }
            }

            if (!is_bridge_inserted)
            {
                ++node_it;
            }
            else
            {
                node_it = nodes_list.erase(node_it);
                node_it = nodes_list.end();
            }
        }

        // if a bridge is inserted, a master should be added as her neighbor
        if (is_bridge_inserted)
        {
            node_it = nodes_list.begin();

            // choose a node that can be master
            while (node_it != nodes_list.end())
            {
                is_master_inserted = insert_master(*node_it, nodes_of_sink[actual_sink_id].back());

                if (is_master_inserted)
                {
                    node_it = nodes_list.erase(node_it);
                    node_it = nodes_list.end();
                }
                else
                {
                    ++node_it;
                }
            }
        }
        else
        {
            // try to add a master node somewhere in the tree
            node_it = nodes_list.begin();

            // choose a node that can be master
            while (node_it != nodes_list.end())
            {
                // choose where to insert the master
                std::vector<int>::iterator nodes_sink_it = nodes_of_sink[actual_sink_id].begin();
                while ((nodes_sink_it != nodes_of_sink[actual_sink_id].end()) & !is_master_inserted)
                {
                    is_master_inserted = insert_master(*node_it, *nodes_sink_it);

                    if (!is_master_inserted)
                    {
                        ++nodes_sink_it;
                    }
                }

                if (is_master_inserted)
                {
                    node_it = nodes_list.erase(node_it);
                    node_it = nodes_list.end();
                }
                else
                {
                    ++node_it;
                }
            }
        }

        actual_sink_id = (++actual_sink_id) % instance.number_trees;
        if (is_master_inserted)
        {
            num_iter = 0;
        }
        else
        {
            ++num_iter;
        }

        // print_info();
    }

    return SolutionHeuristic(edges, is_master, is_bridge);
}

double WSNConstructiveHeuristic::weight_of_solution()
{
    double max = 0;

    for (auto &el : weight_of_sink)
    {
        if (el > max)
        {
            max = el;
        }
    }

    return max;
}

void WSNConstructiveHeuristic::print_info()
{
    util::print::print_vec(sinks, "sinks");
    util::print::print_vec(masters, "masters");
    util::print::print_vec(bridges, "bridges");
    util::print::print_vec(not_adj, "not_adj");
    util::print::print_vec(is_master, "is_master");
    util::print::print_vec(is_bridge, "is_bridge");

    util::print::line("-");
    util::print::print_vec(not_adj, "not_adj");
    util::print::line("-");
    std::cout << "graph" << std::endl;
    util::print::print_matrix(build_graph);
    util::print::line("-");
    std::cout << "nodes_of_sink" << std::endl;
    util::print::print_matrix(nodes_of_sink);
    util::print::line("-");
    std::cout << "edges" << std::endl;
    util::print::print_matrix(edges);
    util::print::line("-");
}