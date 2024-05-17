#pragma once

#include "wsn_data.h"
// #include <iostream>

std::vector<std::vector<int>> adj_list_forest(const std::vector<std::vector<int>> &edges, const WSN_data &instance)
{
    // std::vector<std::vector<int>> adj(instance.n + instance.number_trees, std::vector<int>({}));
    std::vector<std::vector<int>> adj(instance.n, std::vector<int>({}));

    for (auto &edge : edges)
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

bool is_solution_valid(const WSN_data &instance, const std::vector<std::vector<int>> &adj_forest,
                       const std::set<int> &masters, const std::set<int> &bridges)
{
    auto print = [](auto vec, const auto texto)
    {
        // std::cout << texto << ">> ";
        for (auto &el : vec)
        {
            // std::cout << el << " ";
        }
        // std::cout << std::endl;
    };

    print(masters, "masters");
    print(bridges, "bridges");

    // a node that is in dominating forest, should be master or bridge, not both
    for (auto &bridge : bridges)
    {
        if (masters.find(bridge) != masters.end())
        {
            // std::cout << "Node " << bridge << " is a master and a bridge" << std::endl;
            return false;
        }
    }

    auto print_mat = [](auto mat, const auto texto = "")
    {
        // std::cout << texto << ">> " << std::endl;
        for (int i = 0; i < mat.size(); i++)
        {
            // std::cout << i << " >> ";
            for (auto &el : mat[i])
            {
                // std::cout << el << " ";
            }
            // std::cout << std::endl;
        }
    };

    print_mat(instance.is_connected, "mat_adj");
    print_mat(adj_forest, "adj_forest");
    print_mat(instance.adj_list_from_v, "adj_list_from_v");

    auto num_nodes_with_neighbor = [](const std::vector<std::vector<int>> &adj_forest)
    {
        int num = 0;

        for (int i = 0; i < adj_forest.size(); i++)
        {
            if (!adj_forest[i].empty())
            {
                ++num;
            }
        }

        return num;
    };

    // std::cout << "num_nodes_with_neighbor: " << num_nodes_with_neighbor(adj_forest) << std::endl;

    auto roots_of_forest = [](const WSN_data &instance, const std::vector<std::vector<int>> &adj)
    {
        std::vector<int> incoming_edges(instance.n + instance.number_trees, 0);
        std::vector<int> node_in_tree(instance.n + instance.number_trees, 0);
        std::vector<int> roots;

        for (int i = 0; i < adj.size(); i++)
        {
            for (auto &to : adj[i])
            {
                node_in_tree[i] = 1;
                incoming_edges[to] += 1;
                node_in_tree[to] = 1;
            }
        }

        for (int i = 0; i < node_in_tree.size(); i++)
        {
            if (node_in_tree[i] == 1)
            {
                if (incoming_edges[i] == 0)
                {
                    roots.push_back(i);
                }
            }
        }

        return roots;
    };

    auto roots_of_forest_v2 = [](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
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

    std::vector<int> roots = roots_of_forest_v2(instance, adj_forest, masters, bridges);
    std::set<int> roots_set(roots.begin(), roots.end());

    if (roots.empty())
    {
        // std::cout << "Dont have any root." << std::endl;
    }
    else
    {
        // std::cout << "Have " << roots.size() << " roots." << std::endl;
    }

    bool contains_k_trees = (roots.size() == instance.number_trees);

    if (!contains_k_trees)
    {
        // std::cout << "Dont contain number correct of roots." << std::endl;
        return false;
    }

    auto print_vec_name = [](auto vec, std::string name)
    {
        // std::cout << name << " >> ";
        // for (auto &el : vec)
        // {
        // std::cout << el << " ";
        // }
        // std::cout << std::endl;
    };

    auto _dfs_check_one_parent = [](const WSN_data &instance, const std::vector<std::vector<int>> &adj, std::vector<int> &is_visited, int start)
    {
        // is_visited = std::vector<int>(instance.n + instance.number_trees, 0);

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

    auto dfs_check_one_parent = [_dfs_check_one_parent](const WSN_data &instance, const std::vector<std::vector<int>> &adj, int start)
    {
        std::vector<int> is_visited(instance.n + instance.number_trees, 0);

        return _dfs_check_one_parent(instance, adj, is_visited, start);
    };

    auto check_is_forest = [_dfs_check_one_parent](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                                   const std::vector<int> &roots, const std::set<int> &master, const std::set<int> &bridge)
    {
        std::vector<int> is_visited(instance.n + instance.number_trees, 0);

        if (roots.empty())
        {
            return false;
        }

        for (auto &sink : roots)
        {
            auto check = _dfs_check_one_parent(instance, adj, is_visited, sink);

            if (!check)
            {
                return false;
            }

            // std::cout << "Tree of sink " << sink << (check ? " every node has just one parent" : " some node has more than one parent") << std::endl;
        }

        bool all_visited = true;
        for (auto &type_list : {master, bridge})
        {
            for (auto &node : type_list)
            {
                if (is_visited[node] == 0)
                {
                    // std::cout << "node " << node << " is not visited" << std::endl;
                    all_visited = false;
                }
            }
        }

        return all_visited;
    };

    print_vec_name(masters, "master (y)");
    print_vec_name(bridges, "bridge (z)");

    bool is_forest = check_is_forest(instance, adj_forest, roots, masters, bridges);

    if (is_forest)
    {
        // std::cout << "All master and bridge nodes are visited" << std::endl;
    }
    else
    {
        // std::cout << "All master and bridge nodes are not visited" << std::endl;
        return false;
    }

    auto print_adj = [](std::vector<std::vector<int>> adj)
    {
        for (int i = 0; i < adj.size(); i++)
        {
            // std::cout << i << " >>\t";
            for (auto &el : adj[i])
            {
                // std::cout << el << " - ";
            }
            // std::cout << std::endl;
        }
    };

    auto print_vec = [](std::vector<int> vec)
    {
        for (int i = 0; i < vec.size(); i++)
        {
            // std::cout << i << " >>\t" << vec[i] << std::endl;
        }
    };

    std::vector<int> node_belongs_or_adj(instance.n, 0);

    // for (auto &type_list : {masters, bridges}) // {master, bridge}
    // {
    //     for (auto &node : type_list)
    //     {
    //         node_belongs_or_adj[node] = 1;

    //         for (auto &neighbor : instance.adj_list_from_v[node])
    //         {
    //             node_belongs_or_adj[neighbor] = 1;
    //         }
    //     }
    // }

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

    bool all_nodes_belong_or_adj = true;

    for (auto &node : node_belongs_or_adj)
    {
        if (node == 0)
        {
            all_nodes_belong_or_adj = false;
        }
    }

    if (all_nodes_belong_or_adj)
    {
        // std::cout << "All nodes belong or are adjacent to the forest" << std::endl;
    }
    else
    {
        // std::cout << "Some node DON'T belong or are adjacent to the forest" << std::endl;
        return false;
    }

    bool is_any_master_adj_to_master = true;

    for (auto &master : masters)
    {
        for (auto &neighbor : instance.adj_list_from_v[master])
        {
            if (masters.find(neighbor) != masters.end())
            {
                is_any_master_adj_to_master = false;
            }
        }
    }

    if (is_any_master_adj_to_master)
    {
        // std::cout << "Any master is adjacent to another master node" << std::endl;
    }
    else
    {
        // std::cout << "Some master is adjacent to another master node" << std::endl;
        return false;
    }

    // adj should be a forest
    auto _dfs_nodes_in_tree = [](const WSN_data &instance, const std::vector<std::vector<int>> &adj, int root)
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

    auto tree_wsn_is_valid = [_dfs_nodes_in_tree, print_vec_name](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                                                  const std::set<int> &masters, const std::set<int> &bridges, int root)
    {
        auto visited_nodes = _dfs_nodes_in_tree(instance, adj, root);
        print_vec_name(visited_nodes, "visited_nodes");
        bool is_tree_trivial = (visited_nodes.size() == 1);
        bool is_valid_tree = false;

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
            // std::cout << ">> Is trivial and " << (is_valid_tree ? "is a tree valid" : "is not a tree valid") << std::endl;
        }
        else
        {
            is_valid_tree = ((num_master_nodes >= 2) & num_bridge_nodes >= 1);
            // std::cout << ">> Is not trivial and " << (is_valid_tree ? "is a tree valid" : "is not a tree valid") << std::endl;
            // std::cout << num_master_nodes << " master and " << num_bridge_nodes << " bridge nodes." << std::endl;
        }

        return is_valid_tree;
    };

    auto forest_wsn_is_valid = [tree_wsn_is_valid](const WSN_data &instance, const std::vector<std::vector<int>> &adj,
                                                   const std::set<int> &masters, const std::set<int> &bridges, std::vector<int> roots)
    {
        bool is_valid_forest = true;

        for (auto &root : roots)
        {

            auto is_valid_tree = tree_wsn_is_valid(instance, adj, masters, bridges, root);

            if (is_valid_tree)
            {
                // std::cout << "Tree (" << root << ") is valid" << std::endl;
            }
            else
            {
                // std::cout << "Tree (" << root << ") is not valid" << std::endl;
            }

            is_valid_forest &= is_valid_tree;
        }

        return is_valid_forest;
    };

    bool is_valid_forest = forest_wsn_is_valid(instance, adj_forest, masters, bridges, roots);

    if (is_valid_forest)
    {
        // std::cout << "Forest is valid" << std::endl;
    }
    else
    {
        // std::cout << "Forest is not valid" << std::endl;
    }

    return is_valid_forest;
};