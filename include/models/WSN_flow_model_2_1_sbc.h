#pragma once

#include "WSN_flow_model_2_1_base.h"

/**
 * @brief Model WSN_flow_model_2_1_base modified to test constraints to break symmetries
 * and constraints to balance the number of clusters on solution.
 *
 */
class WSN_flow_model_2_1_sbc : public WSN_flow_model_2_1_base
{
public:
    WSN_flow_model_2_1_sbc(WSN_data &instance);

private:
    virtual void build_model();

    IloArray<IloNumVarArray> z_node; // master-sink assignment variable

    void add_remove_symmetries_variables();
    void add_remove_symmetries();

    // constraints that balance the number of clusters on solution
    void add_balancing_constraints(int B = 1);

    virtual IloModel create_relaxed();

    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_flow_model_2_1_sbc::WSN_flow_model_2_1_sbc(WSN_data &instance) : WSN_flow_model_2_1_base(instance),
                                                                     z_node(IloArray<IloNumVarArray>(env, instance.number_trees))
{
    WSN::formulation_name = "FlowModel2-1-sbc-test";
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
    add_remove_symmetries_variables();

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

void WSN_flow_model_2_1_sbc::add_remove_symmetries_variables()
{
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

inline IloModel WSN_flow_model_2_1_sbc::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_3_index(relaxed, z_depot);

    return relaxed;
}

void WSN_flow_model_2_1_sbc::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN_flow_model_2_1_base::print_full(cplex, cout);

    auto [z_node_full, z_node_values] = read_full_matrix(z_node, cplex, 1);
    print_matrix(z_node_full, z_node_values, "z_node", cout);
}