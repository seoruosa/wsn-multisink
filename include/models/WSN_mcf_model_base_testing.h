#pragma once

#include "WSN_mcf_model_base.h"

class WSN_mcf_model_testing : public WSN_mcf_model_base
{
public:
    WSN_mcf_model_testing(WSN_data &instance);

private:
    virtual void build_model() override;
    virtual void set_params_cplex(IloCplex &cplex) override;
    void add_ahani2019_mcf_constraints_refactor();
};

WSN_mcf_model_testing::WSN_mcf_model_testing(WSN_data &instance) : WSN_mcf_model_base(instance)
{
    WSN::formulation_name = "MCF-Model-base-testing";
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

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_mcf_valid_inequalities();

    add_objective_function();
}

void WSN_mcf_model_testing::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
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
            expr += x_sink[k][instance.n][i];
        }
    }

    constraints.add(expr == instance.number_trees);
    clear_expr();

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
            constraints.add(x_sink[k][instance.n][i] <= (y_sink[k][i] + z_sink[k][i]));
        }
    }

    // constraints 8
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int h = 0; h < instance.n; h++)
        {
            for (int j = 0; j < instance.n; j++)
            {
                expr += f_node[h][instance.n][j];
            }

            constraints.add(expr >= (y_sink[k][h] + z_sink[k][h]));
            clear_expr();
        }
    }

    // constraints 9
    for (int h = 0; h < instance.n; h++)
    {
        for (auto &i : instance.adj_list_to_v[h])
        {
            expr += f_node[h][i][h];
        }
        expr += f_node[h][instance.n][h];
        for (auto &j : instance.adj_list_from_v[h])
        {
            expr -= f_node[h][h][j];
        }

        constraints.add(expr == (y[h] + z[h]));
        clear_expr();
    }

    // constraints 10
    for (int h = 0; h < instance.n; h++)
    {
        for (int j = 0; j < instance.n; j++)
        {
            if (h != j)
            {
                for (auto &i : instance.adj_list_to_v[j])
                {
                    expr += f_node[h][i][j];
                }
                expr += f_node[h][instance.n][j];
                for (auto &i : instance.adj_list_from_v[j])
                {
                    expr -= f_node[h][j][i];
                }

                constraints.add(expr == 0);
                clear_expr();
            }
        }
    }

    // constraints 11
    // só existe fluxo em um arco, caso este arco exista
    for (int h = 0; h < instance.n; h++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (auto &j : instance.adj_list_from_v[i])
            {
                for (int k = 0; k < instance.number_trees; k++)
                {
                    expr += x_sink[k][i][j];
                }

                // arcos de A
                constraints.add(f_node[h][i][j] <= expr);
                clear_expr();
            }
        }
    }

    // arcos que saem de um sink
    for (int h = 0; h < instance.n; h++)
    {
        for (int j = 0; j < instance.n; j++)
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                expr += x_sink[k][instance.n][j];
            }
            constraints.add(f_node[h][instance.n][j] <= expr);

            clear_expr();
        }
    }

    expr.end();
}