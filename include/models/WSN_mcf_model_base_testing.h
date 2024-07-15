#pragma once

#include "WSN_mcf_model_base.h"

class WSN_mcf_model_testing : public WSN_mcf_model_base
{
public:
    WSN_mcf_model_testing(WSN_data &instance);

private:
    int M;
    virtual void add_objective_function() override;
    virtual void build_model() override;
    virtual void set_params_cplex(IloCplex &cplex) override;
    void add_ahani2019_mcf_constraints_refactor();
    void add_adasme2023_valid_inequalities_refactor();
    double calculates_big_M();
};

WSN_mcf_model_testing::WSN_mcf_model_testing(WSN_data &instance) : WSN_mcf_model_base(instance),
                                                                   M(calculates_big_M())
{
    WSN::formulation_name = "MCF-Model-base-testing";
}

inline void WSN_mcf_model_testing::add_objective_function()
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

inline void WSN_mcf_model_testing::build_model()
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
    add_ahani2019_mcf_constraints_refactor();

    add_connect_sink_assignment_constraints();

    // // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities_refactor();
    add_mcf_valid_inequalities();

    add_objective_function();
}

void WSN_mcf_model_testing::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

inline void WSN_mcf_model_testing::add_ahani2019_mcf_constraints_refactor()
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

    // flow conservation of number of nodes and limits
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

    expr.end();
}

inline void WSN_mcf_model_testing::add_adasme2023_valid_inequalities_refactor()
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

double WSN_mcf_model_testing::calculates_big_M()
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