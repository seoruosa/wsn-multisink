#pragma once

#include "WSN.h"

class WSN_flow_model_2_1_sbc : public WSN
{
    // Testing symmetry breaking constraints
public:
    WSN_flow_model_2_1_sbc(WSN_data &instance);

private:
    virtual void build_model();

    IloArray<IloNumVarArray> f;

    IloNumVarArray l; // Node current formulation

    IloArray<IloArray<IloNumVarArray>> z_depot; // arc-depot assignment
    IloArray<IloNumVarArray> z_node;            // master-sink assignment variable

    IloNumVar T;

    int M;

    virtual void add_objective_function();

    void add_flow_model_variables();

    void add_flow_limit_constraints();

    void add_flow_conservation_constraints();

    // constraints that uses the extra node
    void add_extra_node_constraints();

    void add_lower_bound_constraints();

    void add_leaf_constraints();

    void add_CastroAndrade2023_valid_inequalities();
    void add_adasme2023_valid_inequalities();

    void add_bektas2020_node_current_constraints();

    void add_arc_depot_assignment_constraints();

    void add_remove_symmetries();

    // constraints that balance the number of clusters on solution
    void add_balancing_constraints(int B = 1);

    // calculates an big-M
    double calculates_big_M();

    virtual IloModel create_relaxed();

    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_flow_model_2_1_sbc::WSN_flow_model_2_1_sbc(WSN_data &instance) : WSN(instance, "FlowModel2-1-sbc"),
                                                                     f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                     l(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)), // Node current formulation
                                                                     z_depot(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees)),
                                                                     z_node(IloArray<IloNumVarArray>(env, instance.number_trees)),
                                                                     T(IloNumVar(env, 0, IloInfinity, ILOFLOAT)),
                                                                     M(int(calculates_big_M()))
{
}

void WSN_flow_model_2_1_sbc::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

void WSN_flow_model_2_1_sbc::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp
    add_flow_limit_constraints();              // exp 5, 7, 8
    add_flow_conservation_constraints();       // exp 6

    add_extra_node_constraints();             // exp 9, 10
    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14

    add_lower_bound_constraints();     // exp 20
    add_master_neighbor_constraints(); // exp 21
    add_leaf_constraints();            // exp 22

    add_trivial_tree_constraints();

    add_arc_depot_assignment_constraints();

    add_remove_symmetries();
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();

    add_balancing_constraints(2);

    add_objective_function();
}

void WSN_flow_model_2_1_sbc::add_objective_function()
{
    model.add(IloMinimize(env, T));

    IloExpr expr(env);

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (auto &from : instance.adj_list_to_v[i])
            {
                expr += instance.weight[from][i] * z_depot[k][from][i];
            }
        }

        model.add(T >= expr);
        expr.end();

        expr = IloExpr(env);
    }
    // expr.end();
    // expr = IloExpr(env);

    // // Remove part of the symmetry ordering the trees by weight. So the first tree will be the heaviest
    // for (int k = 0; k < instance.number_trees - 1; k++)
    // {
    //     for (int i = 0; i < instance.n; i++)
    //     {
    //         for (auto &from : instance.adj_list_to_v[i])
    //         {
    //             expr += instance.weight[from][i] * z_depot[k][from][i] - instance.weight[from][i] * z_depot[k + 1][from][i];
    //         }
    //     }

    //     model.add(expr >= 0);
    //     expr.end();

    //     expr = IloExpr(env);
    // }

    // // calculates the weight of first tree
    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (auto &from : instance.adj_list_to_v[i])
    //     {
    //         expr += instance.weight[from][i] * z_depot[0][from][i];
    //     }
    // }

    // model.add(T >= expr);
    expr.end();
}

void WSN_flow_model_2_1_sbc::add_flow_model_variables()
{
    // Naming variables
    T.setName("T");
    for (int i = 0; i < instance.n; i++)
    {
        l[i].setName(("l(" + std::to_string(i) + ")").c_str());
    }

    for (int i = 0; i < instance.n + instance.number_trees; i++)
    {
        f[i] = IloNumVarArray(env, instance.n, 0, instance.n - instance.number_trees, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }

    // master-sink assignment variable
    for (int i = 0; i < instance.number_trees; i++)
    {
        z_node[i] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            z_node[i][j].setName(("z_node(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }

    // Creating arrays
    for (int k = 0; k < instance.number_trees; k++)
    {
        z_depot[k] = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);

        for (int i = 0; i < instance.n + instance.number_trees; i++)
        {
            z_depot[k][i] = IloNumVarArray(env, instance.n + instance.number_trees, 0, 1, ILOINT);

            // Naming variables
            for (int j = 0; j < instance.n; j++)
            {
                z_depot[k][i][j].setName(("z_depot(" + std::to_string(k) + ")(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
            }
        }
    }
}

void WSN_flow_model_2_1_sbc::add_flow_limit_constraints()
{

    IloExpr expr(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += f[instance.n + k][i];
        }
    }

    constraints.add(expr == N); // exp 5

    expr.end();

    // exp 7
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(x[i][to] <= f[i][to]);
        }

        // arc from node r
        // constraints.add(x[instance.n][i] <= f[instance.n][i]);

        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x[instance.n + k][i] <= f[instance.n + k][i]); // arc from node r
        }
    }

    // exp 8
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][to] <= (instance.n - instance.number_trees - 1) * x[i][to]);
        }

        // arc from node r
        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(f[instance.n + k][i] <= (instance.n - instance.number_trees - 1) * x[instance.n + k][i]);
        }
    }
}

void WSN_flow_model_2_1_sbc::add_flow_conservation_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            expr += f[instance.n + k][i];
        }

        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += f[from][i];
        }

        for (auto &to : instance.adj_list_from_v[i])
        {
            expr -= f[i][to];
        }

        constraints.add(expr == (y[i] + z[i]));

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_2_1_sbc::add_extra_node_constraints()
{
    // exp 9
    IloExpr expr(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += x[instance.n + k][i];
        }

        constraints.add(expr == 1);
        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_2_1_sbc::add_lower_bound_constraints()
{
    // exp 20
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            expr += instance.weight[i][to] * x[i][to];
        }
        constraints.add(T >= expr);

        expr.end();
        expr = IloExpr(env);
    }
    expr.end();
}

void WSN_flow_model_2_1_sbc::add_leaf_constraints()
{
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint doc 17
            constraints.add(z[i] == 0);
        }
    }
}

void WSN_flow_model_2_1_sbc::add_CastroAndrade2023_valid_inequalities()
{
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += (x[i][j] + x[j][i]);

            constraints.add((x[i][j] + x[j][i]) <= y[i] + z[i]); // constraints 15

            // constraint 16 and 17
            constraints.add(x[i][j] <= z[i] + z[j]);
            constraints.add(x[i][j] <= y[i] + y[j]);
        }

        constraints.add(2 * z[i] <= expr); // constraints 12

        expr.end();
        expr = IloExpr(env);
    }

    for (int u = 0; u < instance.n; u++)
    {
        std::set<int> neighbors(instance.adj_list_from_v[u]);
        neighbors.insert(u); // neighbors = N[u]

        expr -= (y[u] + z[u]);
        for (auto &v : instance.adj_list_from_v[u])
        {
            expr += x[u][v];
            expr -= (y[v] + z[v]);

            for (auto &l : instance.adj_list_from_v[v])
            {
                if (neighbors.find(l) != neighbors.end())
                {
                    expr += x[v][l];
                }
            }
        }

        constraints.add(expr <= -1); // constraints 10

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_flow_model_2_1_sbc::add_adasme2023_valid_inequalities()
{
    // constraints Adasme2023
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint 19
            // constraints.add(z[i] == 0);

            for (auto &from : instance.adj_list_to_v[i])
            {
                // constraint 20
                constraints.add(y[from] + y[i] == 1);

                // constraint 21
                // constraints.add(2 * z[from] <= y[i] + x[from][i]);
                constraints.add(2 * (x[from][i] + x[i][from]) <= y[i] + z[from]);
            }
        }
    }

    IloExpr exp_ad_28(env);

    for (int i = 0; i < instance.n; i++)
    {
        exp_ad_28 += (y[i] - z[i]);
    }

    constraints.add(exp_ad_28 >= instance.number_trees);
    exp_ad_28.end();

    IloExpr exp_ad_47(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {

            exp_ad_47 += f[from][i];
        }

        // Constraints 47
        constraints.add(2 * z[i] <= exp_ad_47);

        exp_ad_47.end();
        exp_ad_47 = IloExpr(env);
    }
    exp_ad_47.end();

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            // Constraints 50
            constraints.add(x[instance.n + k][i] <= y[i]);
        }
    }

    // IloExpr exp_ad_48(env);
    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (auto &j : instance.adj_list_from_v[i])
    //     {
    //         for (int h = 0; h < instance.n; h++)
    //         {
    //             exp_ad_48 += f_node[h][i][j];
    //         }

    //         // Constraints 48
    //         constraints.add(2 * x[i][j] - z[i] <= exp_ad_48);

    //         exp_ad_48.end();
    //         exp_ad_48 = IloExpr(env);
    //     }
    // }
    // exp_ad_48.end();

    // IloExpr exp_ad_49(env);
    // IloExpr exp_ad_49_1(env);
    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (int h = 0; h < instance.n; h++)
    //     {
    //         for (auto &from : instance.adj_list_to_v[i])
    //         {
    //             exp_ad_49 += f_node[h][from][i];
    //         }
    //         exp_ad_49 += f_node[h][instance.n][i];
    //     }

    //     for (auto &j : instance.adj_list_from_v[i])
    //     {
    //         for (int h = 0; h < instance.n; h++)
    //         {
    //             exp_ad_49_1 += f_node[h][i][j];
    //         }
    //         // Constraints 49
    //         constraints.add(exp_ad_49_1 + x[i][j] * (instance.n - 1) - exp_ad_49 <= instance.n - 2);

    //         exp_ad_49_1.end();
    //         exp_ad_49_1 = IloExpr(env);
    //     }

    //     exp_ad_49.end();
    //     exp_ad_49 = IloExpr(env);
    // }
    // exp_ad_49.end();
    // exp_ad_49_1.end();
}

void WSN_flow_model_2_1_sbc::add_remove_symmetries()
{
    // adapted from work of Robertty
    IloExpr expr(env);

    // for (int k = 0; k < instance.number_trees; k++)
    // {
    //     for (int i = k + 1; i < instance.number_trees; i++)
    //     {
    //         constraints.add(x[instance.n + k][i] == 0); // test_3
    //     }
    // }

    // for (int k = 0; k < instance.number_trees; k++)
    // {
    //     for (int v = 0; v < instance.n; v++)
    //     {
    //         if (v < k)
    //         {
    //             constraints.add(x[instance.n + k][v] == 0); // test_2
    //         }
    //         else
    //         {
    //             for (int s = instance.n; s < instance.n + k; s++)
    //             {
    //                 for (int u = 0; u < v; u++)
    //                 {
    //                     expr += x[s][u];
    //                 }

    //                 constraints.add(x[instance.n + k][v] <= expr); // test_2

    //                 expr.end();
    //                 expr = IloExpr(env);
    //             }
    //         }
    //     }
    // }

    // for (int u = 0; u < instance.n; u++)
    // {
    //     for (int i = 0; (i < u & i < instance.number_trees); i++)
    //     {
    //         for (int w = 0; w < u; w++)
    //         {
    //             for (int j = 0; j < i; j++)
    //             {
    //                 expr += x[instance.n + j][w];
    //             }
    //             for (int v = 0; v < u; v++)
    //             {
    //                 expr += x[instance.n + i][v];
    //             }

    //             constraints.add(x[instance.n + i][u] <= expr); // test_3

    //             expr.end();
    //             expr = IloExpr(env);
    //         }
    //     }
    // }

    // for (int k = 0; k < instance.number_trees - 1; k++)
    // {
    //     for (int v = 0; v < instance.n; v++)
    //     {
    //         expr += (v + 1) * (x[instance.n + k][v] - x[instance.n + k + 1][v]);
    //     }

    //     constraints.add(expr <= 0); // test_1

    //     expr.end();
    //     expr = IloExpr(env);
    // }

    // test_4
    // for (int k = 0; k < instance.number_trees; k++)
    // {
    //     for (int v = 0; v < instance.n; v++)
    //     {
    //         for (int s = instance.n; s < instance.n + k; s++)
    //         {
    //             for (int u = v + 1; u < instance.n; u++)
    //             {
    //                 expr += x[s][u];
    //             }

    //             constraints.add(x[instance.n + k][v] <= expr); // test_4

    //             expr.end();
    //             expr = IloExpr(env);
    //         }
    //     }
    // }

    // test_5
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            for (int s = instance.n; s < instance.n + k; s++)
            {
                for (int u = v + 1; u < instance.n; u++)
                {
                    expr += x[s][u];
                }

                for (int j = k; j < instance.number_trees; j++)
                {
                    expr -= x[instance.n + j][v];
                }

                constraints.add(expr >= 0); // test_5

                expr.end();
                expr = IloExpr(env);
            }
        }
    }

    // // test_6
    // for (int v = 0; v < instance.n; v++)
    // {
    //     for (int k = 0; k < instance.number_trees; k++)
    //     {
    //         for (auto &u : instance.adj_list_to_v[v])
    //         {
    //             expr += v * z_depot[k][u][v];
    //         }
    //         for (int u = 0; u < instance.n; u++)
    //         {
    //             expr -= u * z_depot[k][instance.n + k][u];
    //         }

    //         constraints.add(expr <= 0);

    //         expr.end();
    //         expr = IloExpr(env);
    //     }
    // }

    // test_7
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            for (auto &u : instance.adj_list_to_v[v])
            {
                expr += z_depot[k][u][v];
            }

            // z_node[k][v]=1 if node v it's a master and belongs to tree k
            constraints.add(expr + y[v] <= 1 + z_node[k][v]); // test_7 test_8
            constraints.add(expr + y[v] >= 2 * z_node[k][v]); // test_7 test_8

            expr.end();
            expr = IloExpr(env);
        }
    }

    // for (int k = 0; k < instance.number_trees; k++)
    // {
    //     for (int u = 0; u < instance.n; u++)
    //     {
    //         expr += u * z_depot[k][instance.n + k][u];
    //     }

    //     for (int v = 0; v < instance.n; v++)
    //     {
    //         // the node connected to sink have the bigger index of tree
    //         constraints.add(expr >= v * z_node[k][v]); // test_7
    //     }

    //     expr.end();
    //     expr = IloExpr(env);
    // }

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            // the node connected to sink have the bigger index of tree
            for (int u = v + 1; u < instance.n; u++)
            {
                expr += z_depot[k][instance.n + k][u];
            }

            constraints.add(z_node[k][v] <= expr); // test_8
        }

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_2_1_sbc::add_balancing_constraints(int B)
{
    IloExpr expr(env);

    for (int a = 0; a < instance.number_trees; a++)
    {
        for (int b = 0; b < instance.number_trees; b++)
        {
            if (a > b)
            {
                for (int u = 0; u < instance.n; u++)
                {
                    expr += (z_node[a][u] - z_node[b][u]);
                }

                constraints.add(expr <= B);
                constraints.add(-expr <= B);

                expr.end();
                expr = IloExpr(env);
            }
        }
    }

    expr.end();
}

void WSN_flow_model_2_1_sbc::add_bektas2020_node_current_constraints()
{
    // for (int k = 0; k < instance.number_trees; k++)
    // {
    //     constraints.add(l[instance.n + k] == (k + 1));
    // }

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // constraints.add(l[i] - l[j] <= (instance.number_trees - 1) * (1 - x[i][j] - x[j][i]));
            constraints.add(l[i] - l[j] <= (instance.number_trees) * (1 - x[j][i]));
            constraints.add(l[j] - l[i] <= (instance.number_trees) * (1 - x[i][j]));
        }
    }
}

void WSN_flow_model_2_1_sbc::add_arc_depot_assignment_constraints()
{
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (auto &j : instance.adj_list_from_v[i])
            {
                // constraints.add(l[i] - l[j] <= (instance.number_trees) * (1 - z_depot[k][i][j] - z_depot[k][j][i]));

                constraints.add((l[i] - (k + 1)) <= (instance.number_trees) * (1 - z_depot[k][i][j] - z_depot[k][j][i]));
                constraints.add(((k + 1) - l[i]) <= (instance.number_trees) * (1 - z_depot[k][i][j] - z_depot[k][j][i]));
            }

            constraints.add(l[i] >= (k + 1) * z_depot[k][instance.n + k][i]);
            constraints.add(l[i] <= (k + 1) + (instance.number_trees - 1) * (1 - z_depot[k][instance.n + k][i]));
        }
    }

    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                expr += z_depot[k][i][j];
            }

            constraints.add(expr == x[i][j]);

            expr.end();
            expr = IloExpr(env);
        }

        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(z_depot[k][instance.n + k][i] == x[instance.n + k][i]);
        }
    }
}

double WSN_flow_model_2_1_sbc::calculates_big_M()
{
    double M_weight = 1.0;
    std::vector<double> weights;

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            weights.push_back(instance.weight[i][j]);
        }
    }

    std::sort(weights.begin(), weights.end(), std::greater<double>());

    for (int i = 0; i < (instance.n - instance.number_trees) && i < weights.size(); i++)
    {
        M_weight += weights[i];
    }

    return M_weight;
}

inline IloModel WSN_flow_model_2_1_sbc::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_3_index(relaxed, z_depot);

    return relaxed;
}

void WSN_flow_model_2_1_sbc::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto read_matrix_3d = [](IloArray<IloArray<IloNumVarArray>> &matrix_3d, IloCplex &cplex, int sum_to_index = 0)
    {
        std::vector<std::vector<int>> vec;
        std::vector<double> values;

        for (int k = 0; k < matrix_3d.getSize(); k++)
        {
            for (int i = 0; i < matrix_3d[k].getSize(); i++)
            {
                for (int j = 0; j < matrix_3d[k][i].getSize(); j++)
                {
                    try
                    {
                        auto val = cplex.getValue(matrix_3d[k][i][j]);
                        if (val > 0)
                        {
                            vec.push_back({k + sum_to_index, i + sum_to_index, j + sum_to_index});
                            values.push_back(val);
                        }
                    }
                    catch (IloException &e)
                    {
                    }
                }
            }
        }

        return std::pair<std::vector<std::vector<int>>, std::vector<double>>({vec, values});
    };

    auto [z_depot_full, z_depot_values] = read_matrix_3d(z_depot, cplex, 1);

    print_matrix(z_depot_full, z_depot_values, "z_depot", cout);

    auto [l_full, l_values] = read_full_vec_to_matrix(l, cplex, 1);
    print_matrix(l_full, l_values, "l", cout);

    auto [z_node_full, z_node_values] = read_full_matrix(z_node, cplex, 1);
    print_matrix(z_node_full, z_node_values, "z_node", cout);
}