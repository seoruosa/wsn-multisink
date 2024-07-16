#pragma once

#include "WSN.h"

/**
 * @brief Multi-flow formulation where the flow is the weight of kth rooted tree
 *
 */
class WSN_mcf_weight_model_base : public WSN
{
public:
    WSN_mcf_weight_model_base(WSN_data &instance);

protected:
    virtual void build_model();

    IloArray<IloArray<IloNumVarArray>> x_sink; // arc-sink assignment
    IloArray<IloArray<IloNumVarArray>> f_node; // flow of commodity h (node h) along arc (i, j)

    IloArray<IloNumVarArray> y_sink; // master sink assignment
    IloArray<IloNumVarArray> z_sink; // bridge sink assignment

    IloNumVar T;
    int M;

    virtual void add_objective_function();

    // basic model
    void add_flow_model_variables();
    void add_ahani2019_mcf_constraints();
    void add_connect_sink_assignment_constraints();

    // Valid inequalities
    void add_CastroAndrade2023_valid_inequalities();
    void add_adasme2023_valid_inequalities();
    void add_mcf_valid_inequalities();
    void add_remove_symmetries();
    void add_conservation_inequalities_assignment();

    virtual IloModel create_relaxed();

    virtual void print_full(IloCplex &cplex, std::ostream &cout);

    virtual void set_params_cplex(IloCplex &cplex);

    double calculates_big_M();
};

WSN_mcf_weight_model_base::WSN_mcf_weight_model_base(WSN_data &instance) : WSN(instance, "MCF-Model-weightAsFlow-base"),
                                                                           T(IloNumVar(env, 0, IloInfinity, ILOFLOAT)),
                                                                           x_sink(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees)),
                                                                           f_node(IloArray<IloArray<IloNumVarArray>>(env, instance.n)),
                                                                           y_sink(IloArray<IloNumVarArray>(env)),
                                                                           z_sink(IloArray<IloNumVarArray>(env)),
                                                                           M(calculates_big_M())
{
}

inline void WSN_mcf_weight_model_base::add_objective_function()
{
    model.add(IloMinimize(env, T));

    // constraint used to min-max the tree weight (not yet defined, now it's the forest weight)
    IloExpr expr(env);

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += f_node[k][instance.n + k][i];
        }

        model.add(T >= expr);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_mcf_weight_model_base::build_model()
{
    // basic model
    add_decision_variables();
    add_number_dominating_nodes_constraints();
    add_number_forest_edges_constraints();
    add_in_coming_edge_constraints();
    add_node_master_or_bridge_constraints();
    add_master_neighbor_constraints();
    add_master_not_adj_master_constraints();
    add_bridges_not_neighbor_constraints();
    add_bridge_master_neighbor_constraints();

    add_trivial_tree_constraints();

    // mcf model
    add_flow_model_variables();
    add_ahani2019_mcf_constraints();

    add_connect_sink_assignment_constraints();

    // // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_mcf_valid_inequalities();
    add_remove_symmetries();
    add_conservation_inequalities_assignment();

    add_objective_function();
}

void WSN_mcf_weight_model_base::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

inline void WSN_mcf_weight_model_base::add_flow_model_variables()
{
    T.setName("T");

    y_sink = IloArray<IloNumVarArray>(env, instance.number_trees);
    z_sink = IloArray<IloNumVarArray>(env, instance.number_trees);

    // Creating arrays
    for (int k = 0; k < instance.number_trees; k++)
    {
        x_sink[k] = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);

        y_sink[k] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);
        z_sink[k] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);

        for (int i = 0; i < instance.n + instance.number_trees; i++)
        {
            x_sink[k][i] = IloNumVarArray(env, instance.n + instance.number_trees, 0, 1, ILOINT);

            // Naming variables
            for (int j = 0; j < instance.n; j++)
            {
                x_sink[k][i][j].setName(("x_sink(" + std::to_string(k) + ")(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
            }
        }

        for (int i = 0; i < instance.n; i++)
        {
            y_sink[k][i].setName(("y_sink(" + std::to_string(k) + ")(" + std::to_string(i) + ")").c_str());
            z_sink[k][i].setName(("z_sink(" + std::to_string(k) + ")(" + std::to_string(i) + ")").c_str());
        }
    }

    // Creating arrays
    for (int k = 0; k < instance.n; k++)
    {
        f_node[k] = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);

        for (int i = 0; i < instance.n + instance.number_trees; i++)
        {
            f_node[k][i] = IloNumVarArray(env, instance.n + instance.number_trees, 0, M, ILOFLOAT);

            // Naming variables
            for (int j = 0; j < instance.n; j++)
            {
                f_node[k][i][j].setName(("f_node(" + std::to_string(k) + ")(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
            }
        }
    }
}

inline void WSN_mcf_weight_model_base::add_ahani2019_mcf_constraints()
{
    IloExpr expr(env);

    auto clear_expr = [&]
    {
        expr.end();
        expr = IloExpr(env);
    };

    // do nó extra saem k arcos
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += x_sink[k][instance.n + k][i];
        }

        constraints.add(expr == 1);
        clear_expr();
    }

// force to zero variables 
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int l = 0; l < instance.number_trees; l++)
        {
            if (l != k)
            {
                for (int v = 0; v < instance.n; v++)
                {
                    expr += x_sink[k][instance.n + l][v];
                }

                constraints.add(expr == 0);
                clear_expr();
            }
        }
    }

    // constraints.add(expr == instance.number_trees);
    // clear_expr();

    // constraints 6 and 7
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                constraints.add(x_sink[k][i][j] <= (y_sink[k][i] + z_sink[k][i]));
                constraints.add(x_sink[k][i][j] <= (y_sink[k][j] + z_sink[k][j]));
            }
        }
    }

    // se um arco sai do sink k até o nó i, então este nó pertence à árvore k
    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x_sink[k][instance.n + k][i] <= (y_sink[k][i] + z_sink[k][i]));
        }
    }

    // flow conservation and limits
    for (int s = 0; s < instance.number_trees; s++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            expr += f_node[s][instance.n + s][v];
            constraints.add(f_node[s][instance.n + s][v] <= M * x_sink[s][instance.n + s][v]);

            for (auto &from : instance.adj_list_to_v[v])
            {
                expr += f_node[s][from][v];
                expr -= instance.weight[from][v] * x_sink[s][from][v];

                constraints.add(f_node[s][from][v] <= M * x_sink[s][from][v]);
                constraints.add(f_node[s][from][v] >= instance.weight[from][v] * x_sink[s][from][v]);
            }

            for (auto &to : instance.adj_list_from_v[v])
            {
                expr -= f_node[s][v][to];

                constraints.add(f_node[s][v][to] <= M * x_sink[s][v][to]);
                constraints.add(f_node[s][v][to] >= instance.weight[v][to] * x_sink[s][v][to]);
            }

            constraints.add(expr == 0);
            clear_expr();
        }
    }

    expr.end();
}

inline void WSN_mcf_weight_model_base::add_connect_sink_assignment_constraints()
{
    IloExpr expr_x(env);
    IloExpr expr_y(env);
    IloExpr expr_z(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            expr_y += y_sink[k][i];
            expr_z += z_sink[k][i];
        }

        constraints.add(y[i] == expr_y);
        constraints.add(z[i] == expr_z);

        expr_y.end();
        expr_y = IloExpr(env);

        expr_z.end();
        expr_z = IloExpr(env);

        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                expr_x += x_sink[k][i][j];
            }

            constraints.add(x[i][j] == expr_x);

            expr_x.end();
            expr_x = IloExpr(env);
        }

        // nodes from sinks
        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x[instance.n + k][i] == x_sink[k][instance.n + k][i]);
        }
    }

    expr_x.end();
    expr_y.end();
    expr_z.end();
}

inline void WSN_mcf_weight_model_base::add_CastroAndrade2023_valid_inequalities()
{
    // Constraints from CastroAndrade2023
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // constraint 15
            constraints.add(x[i][j] + x[j][i] <= (y[j] + z[j]));

            // constraint 16 and 17
            constraints.add(x[i][j] <= z[i] + z[j]);
            constraints.add(x[i][j] <= y[i] + y[j]);
        }
    }
}

inline void WSN_mcf_weight_model_base::add_adasme2023_valid_inequalities()
{
    // constraints Adasme2023
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint 19
            constraints.add(z[i] == 0);

            for (auto &from : instance.adj_list_to_v[i])
            {
                // constraint 20
                constraints.add(y[from] + y[i] == 1);
            }
        }
    }

    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                for (auto &to : instance.adj_list_from_v[i])
                {
                    // constraint 21
                    constraints.add(2 * z_sink[k][to] <= y_sink[k][i] + x_sink[k][i][to]);
                }
            }
        }
    }

    IloExpr exp_ad_28(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            exp_ad_28 += (y_sink[k][i] - z_sink[k][i]);
        }

        constraints.add(exp_ad_28 >= 1);

        exp_ad_28.end();
        exp_ad_28 = IloExpr(env);
    }

    exp_ad_28.end();
}

inline void WSN_mcf_weight_model_base::add_mcf_valid_inequalities()
{
    // Constraints 50 (Adasme2023)
    for (int i = 0; i < instance.n; i++)
    {

        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x_sink[k][instance.n + k][i] <= y_sink[k][i]);
        }
    }

    // WSN::add_in_coming_edge_constraints()
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                constraints.add(x_sink[k][i][j] + x_sink[k][j][i] <= (y_sink[k][j] + z_sink[k][j])); // check this for every arc
            }
        }
    }
}

inline void WSN_mcf_weight_model_base::add_remove_symmetries()
{
    // adapted from work of Robertty
    IloExpr expr(env);

    // test_5
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            for (int s = 0; s < k; s++)
            {
                for (int u = v + 1; u < instance.n; u++)
                {
                    expr += x_sink[s][instance.n + s][u];
                }

                for (int j = k; j < instance.number_trees; j++)
                {
                    expr -= x_sink[j][instance.n + j][v];
                }

                constraints.add(expr >= 0); // test_5

                expr.end();
                expr = IloExpr(env);
            }
        }
    }

    expr.end();
}

inline void WSN_mcf_weight_model_base::add_conservation_inequalities_assignment()
{
    // flow conservation of number of nodes and limits
    IloExpr expr(env);

    auto clear_expr = [&]
    {
        expr.end();
        expr = IloExpr(env);
    };

    for (int s = 0; s < instance.number_trees; s++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            expr += x_sink[s][instance.n + s][v];

            for (auto &from : instance.adj_list_to_v[v])
            {
                expr += x_sink[s][from][v];
            }

            for (auto &to : instance.adj_list_from_v[v])
            {
                expr -= x_sink[s][v][to];
            }

            constraints.add(expr <= (y_sink[s][v] + z_sink[s][v]));
            clear_expr();
        }
    }
}

inline IloModel WSN_mcf_weight_model_base::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_2_index(relaxed, y_sink);
    relaxed = relax_utils::relax_2_index(relaxed, z_sink);

    relaxed = relax_utils::relax_3_index(relaxed, x_sink);
    relaxed = relax_utils::relax_3_index(relaxed, f_node);

    return relaxed;
}

inline void WSN_mcf_weight_model_base::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto [x_sink_full, x_sink_values] = read_matrix_3d(x_sink, cplex, 1);

    print_matrix(x_sink_full, x_sink_values, "x_sink", cout);

    auto [f_node_full, f_node_values] = read_matrix_3d(f_node, cplex, 1);

    print_matrix(f_node_full, f_node_values, "f_node", cout);

    auto [y_sink_full, y_sink_values] = read_full_matrix(y_sink, cplex, 1);

    print_matrix(y_sink_full, y_sink_values, "y_sink", cout);
}

double WSN_mcf_weight_model_base::calculates_big_M()
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