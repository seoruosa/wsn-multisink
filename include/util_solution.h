#pragma once

#include "wsn_data.h"

/**
 * @brief Transform a vector of arcs in a list of adjacency
 *
 * @param arcs is a vector of arcs
 * @param instance is the instance of the problem
 * @return std::vector<std::vector<int>> list of adjacency
 */
std::vector<std::vector<int>> adj_list_forest(const std::vector<std::vector<int>> &arcs, const WSN_data &instance)
{
    std::vector<std::vector<int>> adj(instance.n, std::vector<int>({}));

    for (auto &edge : arcs)
    {
        if (!edge.empty())
        {
            auto from = edge[0];
            auto to = edge[1];

            if (from < instance.n)
            {
                adj[from].push_back(to);
            }
        }
    }

    return adj;
};

/**
 * @brief Find all roots of forest
 *
 * @param instance is the instance of the problem
 * @param adj is the list of adjacency of forest
 * @param masters is a set with all nodes designated as a master node
 * @param bridges is a set with all nodes designated as a bridge node
 * @return std::vector<int> a vector with all roots of the forest
 */
std::vector<int> find_roots_of_forest(const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                      const std::set<int> &masters, const std::set<int> &bridges)
{
    std::vector<int> incoming_edges(instance.n + instance.number_trees, 0);
    std::vector<int> node_in_dominating_tree(instance.n + instance.number_trees, 0);
    std::vector<int> roots;

    for (auto &type_list : {masters, bridges})
    {
        for (auto &node : type_list)
        {
            node_in_dominating_tree[node] = 1;
        }
    }

    for (int i = 0; i < adj.size(); i++)
    {
        for (auto &to : adj[i])
        {
            node_in_dominating_tree[i] = 1;
            incoming_edges[to] += 1;
            node_in_dominating_tree[to] = 1;
        }
    }

    for (int i = 0; i < node_in_dominating_tree.size(); i++)
    {
        if (node_in_dominating_tree[i] == 1)
        {
            if (incoming_edges[i] == 0)
            {
                roots.push_back(i);
            }
        }
    }

    return roots;
};

/**
 * @brief Valid if a solution of instance is feasible
 *
 * @param instance is the instance of problem
 * @param adj_forest contains the arcs of solution represented as a list of adjacency
 * @param masters is a set with all nodes designated as a master node
 * @param bridges is a set with all nodes designated as a bridge node
 * @return true if solution is feasible
 * @return false if solution is infeasible
 */
bool is_solution_valid(const WSN_data &instance, const std::vector<std::vector<int>> &adj_forest,
                       const std::set<int> &masters, const std::set<int> &bridges)
{
    // a node that belongs to the dominating forest, should be master or bridge, not both
    for (auto &bridge : bridges)
    {
        if (masters.find(bridge) != masters.end())
        {
            return false;
        }
    }

    std::vector<int> roots = find_roots_of_forest(instance, adj_forest, masters, bridges);

    // a solution should have k trees
    bool contains_k_trees = (roots.size() == instance.number_trees);

    if (!contains_k_trees)
    {
        return false;
    }

    // Traverse a graph and check if each node have at most one parent
    auto dfs_check_one_parent = [](const WSN_data &instance,
                                   const std::vector<std::vector<int>> &adj,
                                   std::vector<int> &is_visited, int start)
    {
        std::vector<int> stack = {};
        stack.push_back(start);

        while (!stack.empty())
        {
            int current = stack.back();
            stack.pop_back();

            if (is_visited[current] == 1)
            {
                return false;
            }
            else
            {
                is_visited[current] = 1;

                for (auto &neighbor : adj[current])
                {
                    if (is_visited[neighbor] == 1)
                    {
                        return false;
                    }
                    else
                    {
                        stack.push_back(neighbor);
                    }
                }
            }
        }

        return true;
    };

    auto check_is_forest = [dfs_check_one_parent](const WSN_data &instance,
                                                  const std::vector<std::vector<int>> &adj,
                                                  const std::vector<int> &roots,
                                                  const std::set<int> &master,
                                                  const std::set<int> &bridge)
    {
        std::vector<int> is_visited(instance.n + instance.number_trees, 0);

        if (roots.empty())
        {
            return false;
        }

        // Traverse each tree of forest and check for a loop
        for (auto &root : roots)
        {
            auto check = dfs_check_one_parent(instance, adj, is_visited, root);

            if (!check)
            {
                return false;
            }
        }

        // Check if all master or bridge nodes belongs to the forest
        for (auto &type_list : {master, bridge})
        {
            for (auto &node : type_list)
            {
                if (is_visited[node] == 0)
                {
                    return false;
                }
            }
        }

        return true;
    };

    bool is_forest = check_is_forest(instance, adj_forest, roots, masters, bridges);

    if (!is_forest)
    {
        return false;
    }

    std::vector<int> node_belongs_or_adj(instance.n, 0);

    for (auto &master : masters)
    {
        node_belongs_or_adj[master] = 1;

        for (auto &neighbor : instance.adj_list_from_v[master])
        {
            node_belongs_or_adj[neighbor] = 1;
        }
    }

    for (auto &bridge : bridges)
    {
        node_belongs_or_adj[bridge] = 1;
    }

    // All nodes should be a master node or be adjacent to one
    for (auto &node : node_belongs_or_adj)
    {
        if (node == 0)
        {
            return false;
        }
    }

    // Any master node should be adjacent to another master
    for (auto &master : masters)
    {
        for (auto &neighbor : instance.adj_list_from_v[master])
        {
            if (masters.find(neighbor) != masters.end())
            {
                return false;
            }
        }
    }

    // Traverse a tree from the root and returns a vector with all nodes that belong to the tree
    auto nodes_of_tree = [](const WSN_data &instance, 
                            const std::vector<std::vector<int>> &adj, 
                            int root)
    {
        std::vector<int> visited_nodes({});
        std::vector<int> is_visited(instance.n + instance.number_trees, 0);

        std::vector<int> stack = {};
        stack.push_back(root);

        while (!stack.empty())
        {
            int current = stack.back();
            stack.pop_back();
            visited_nodes.push_back(current);

            if (is_visited[current] == 1)
            {
                return visited_nodes;
            }
            else
            {
                is_visited[current] = 1;

                for (auto &neighbor : adj[current])
                {
                    if (is_visited[neighbor] == 1)
                    {
                        return visited_nodes;
                    }
                    else
                    {
                        stack.push_back(neighbor);
                    }
                }
            }
        }

        return visited_nodes;
    };

    // Check if a tree is wsn-valid: is trivial with just one master node or the number of masters is greater than 2 and bridges than 1
    auto tree_wsn_is_valid = [nodes_of_tree](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                                  const std::set<int> &masters, const std::set<int> &bridges, int root)
    {
        auto visited_nodes = nodes_of_tree(instance, adj, root);
        bool is_tree_trivial = (visited_nodes.size() == 1);
        bool is_valid_tree;

        int num_master_nodes = 0;
        int num_bridge_nodes = 0;

        for (auto &node : visited_nodes)
        {
            if (masters.find(node) != masters.end())
            {
                ++num_master_nodes;
            }
            else if (bridges.find(node) != bridges.end())
            {
                ++num_bridge_nodes;
            }
        }

        if (is_tree_trivial)
        {
            is_valid_tree = (num_master_nodes > 0);
        }
        else
        {
            is_valid_tree = ((num_master_nodes >= 2) & num_bridge_nodes >= 1);
        }

        return is_valid_tree;
    };

    auto forest_wsn_is_valid = [tree_wsn_is_valid](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                                   const std::set<int> &masters, const std::set<int> &bridges, std::vector<int> roots)
    {
        // All trees should be valid to the forest be valid
        for (auto &root : roots)
        {
            auto is_valid_tree = tree_wsn_is_valid(instance, adj, masters, bridges, root);

            if (!is_valid_tree)
            {
                return false;
            }
        }

        return true;
    };

    bool is_valid_forest = forest_wsn_is_valid(instance, adj_forest, masters, bridges, roots);

    return is_valid_forest;
};