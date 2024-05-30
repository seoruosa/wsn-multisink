#pragma once

#include "WSN.h"
#include <limits>

#include "wsn_constructive_heur.h"

class WSN_arvore_rotulada_model_base : public WSN
{
public:
    WSN_arvore_rotulada_model_base(WSN_data &instance);

private:
    virtual void build_model();

    IloArray<IloArray<IloNumVarArray>> x_sink; // arc-sink assignment

    IloArray<IloNumVarArray> y_sink; // master sink assignment
    IloArray<IloNumVarArray> z_sink; // bridge sink assignment

    IloNumVar T;
    IloArray<IloNumVarArray> f;

    int p;
    IloNumVarArray pi;

    virtual void add_objective_function();

    // basic model
    void add_mar_model_variables();
    void add_flow_model_variables();
    void add_ahani2019_mcf_constraints();
    void add_connect_sink_assignment_constraints();

    // flow constraints
    void add_flow_limit_constraints();
    void add_flow_conservation_constraints();

    // mtz constraints
    void add_mtz_model_variables();
    void add_mtz_subtour_elimination_constraints();
    void add_mtz_valid_inequalities();
    void add_mtz_bektas2014_inequalities();

    // Valid inequalities
    void add_CastroAndrade2023_valid_inequalities();
    void add_adasme2023_valid_inequalities();
    void add_mcf_valid_inequalities();

    void add_testing_valid_inequalities();
    void add_remove_symmetries();

    virtual IloModel create_relaxed();

    virtual void print_full(IloCplex &cplex, std::ostream &cout);

    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_arvore_rotulada_model_base::WSN_arvore_rotulada_model_base(WSN_data &instance) : WSN(instance, "MAR-base"),
                                                                                     T(IloNumVar(env, 0, IloInfinity, ILOFLOAT)),
                                                                                     x_sink(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees)),
                                                                                     y_sink(IloArray<IloNumVarArray>(env)),
                                                                                     z_sink(IloArray<IloNumVarArray>(env)),
                                                                                     f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                                     p((instance.n - instance.number_trees + 1) / 2),
                                                                                     pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT))
{
}

inline void WSN_arvore_rotulada_model_base::build_model()
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

    // mar model
    add_mar_model_variables();
    add_ahani2019_mcf_constraints();
    add_connect_sink_assignment_constraints();

    // // remove subtours - flow-based
    // add_flow_model_variables();
    // add_flow_limit_constraints();
    // add_flow_conservation_constraints();

    // remove subtours - mtz
    add_mtz_model_variables();
    add_mtz_subtour_elimination_constraints();
    add_mtz_valid_inequalities();
    add_mtz_bektas2014_inequalities();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_mcf_valid_inequalities();

    add_testing_valid_inequalities();
    add_remove_symmetries();

    add_objective_function();
}

void WSN_arvore_rotulada_model_base::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

inline void WSN_arvore_rotulada_model_base::add_objective_function()
{

    model.add(IloMinimize(env, T));

    // constraint used to min-max the tree weight (not yet defined, now it's the forest weight)
    IloExpr expr(env);

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (auto &from : instance.adj_list_to_v[i])
            {
                expr += instance.weight[from][i] * x_sink[k][from][i];
            }
        }
        model.add(T >= expr);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_arvore_rotulada_model_base::add_mar_model_variables()
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
}

inline void WSN_arvore_rotulada_model_base::add_flow_model_variables()
{
    for (int i = 0; i < instance.n + instance.number_trees; i++)
    {
        f[i] = IloNumVarArray(env, instance.n, 0, instance.n - instance.number_trees, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN_arvore_rotulada_model_base::add_mtz_model_variables()
{
    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        pi[i].setName(("pi(" + std::to_string(i) + ")").c_str());
    }
}

void WSN_arvore_rotulada_model_base::add_mtz_subtour_elimination_constraints()
{
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // strengthened MTZ subtour elimination constraints
            constraints.add(pi[j] - pi[i] - p * x[i][j] - (p - 2) * x[j][i] >= 1 - p);
        }
    }
}

void WSN_arvore_rotulada_model_base::add_mtz_valid_inequalities()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        if (p - instance.number_trees + 1 >= 0)
        {
            constraints.add(pi[i] + z[i] <= p - instance.number_trees + 1); // constraint 23 (castroAndrade2023)
        }
    }

    for (int i = 0; i < instance.n; i++)
    {
        // Constraints 31
        constraints.add(pi[i] >= z[i]); // constraint 20 (castroAndrade2023)

        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += x[from][i];
        }

        // Constraints 32
        constraints.add(pi[i] >= 2 * expr - z[i]); // constraint 21 (castroAndrade2023)

        expr.end();
        expr = IloExpr(env);
    }

    // Contraints 30
    for (int i = 0; i < instance.n; i++)
    {
        expr += (pi[i] - z[i] - 2 * y[i]);
    }

    constraints.add(expr >= -2);

    expr.end();
    expr = IloExpr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            for (auto &k : instance.adj_list_to_v[from])
            {
                expr += x[k][from];
            }

            // Constraints 33
            constraints.add(pi[i] >= 2 * expr + (y[from] + z[i] - 1) - 2 * (1 - x[from][i])); // constraint 22 (castroAndrade2023)

            expr.end();
            expr = IloExpr(env);
        }
    }

    expr.end();
}

void WSN_arvore_rotulada_model_base::add_mtz_bektas2014_inequalities()
{
    int n = instance.n;
    // restricoes Bektas2014
    for (int i = 0; i < instance.n; i++)
    {
        for (int j = 0; j < instance.n; j++)
        {
            for (int k = 0; k < instance.n; k++)
            {
                if ((instance.is_connected[i][j] == 1) & (instance.is_connected[j][k] == 1) & (instance.is_connected[i][k] == 1))
                {
                    // 3.1
                    constraints.add(pi[i] - pi[k] - (n - 1) * (x[i][j] + x[j][k]) + (n - 3) * (x[k][j] + x[j][i]) + n * x[i][k] + (n - 4) * x[k][i] <= 2 * n - 4);
                    // // 3.5 e 3.6
                    constraints.add(pi[i] - pi[k] - (2 * n - 3) * x[i][k] + (n - 4) * x[k][i] + (n - 1) * (x[i][j] + x[j][k]) <= 2 * n - 4);
                    constraints.add(pi[k] - pi[i] - (2 * n - 7) * x[i][k] + (n - 1) * x[k][i] + (n - 4) * (x[i][j] + x[j][k]) <= 2 * n - 6);
                    // // 4.3
                    constraints.add((3 * n - 7) * (x[i][k] + x[k][i]) + (n - 1) * (x[i][j] + x[j][k] + x[k][j] + x[j][i]) <= 4 * n - 8);
                }
            }
        }
    }
}

inline void WSN_arvore_rotulada_model_base::add_ahani2019_mcf_constraints()
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

        constraints.add(expr == 1);
        clear_expr();
    }

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

    expr.end();
}

inline void WSN_arvore_rotulada_model_base::add_connect_sink_assignment_constraints()
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
    }

    expr_x.end();
    expr_y.end();
    expr_z.end();
}

void WSN_arvore_rotulada_model_base::add_flow_limit_constraints()
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
            constraints.add(x_sink[k][instance.n][i] <= f[instance.n + k][i]); // arc from node r
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
            constraints.add(f[instance.n + k][i] <= (instance.n - instance.number_trees - 1) * x_sink[k][instance.n][i]);
        }
    }
}

void WSN_arvore_rotulada_model_base::add_flow_conservation_constraints()
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

inline void WSN_arvore_rotulada_model_base::add_CastroAndrade2023_valid_inequalities()
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

            // for (int k = 0; k < instance.number_trees; k++)
            // {
            //     constraints.add(x_sink[k][i][j] <= z_sink[k][i] + z_sink[k][j]);
            //     constraints.add(x_sink[k][i][j] <= y_sink[k][i] + y_sink[k][j]);
            // }
        }
    }
}

inline void WSN_arvore_rotulada_model_base::add_adasme2023_valid_inequalities()
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

void WSN_arvore_rotulada_model_base::add_mcf_valid_inequalities()
{
    // Constraints 50 (Adasme2023)
    for (int i = 0; i < instance.n; i++)
    {

        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x_sink[k][instance.n][i] <= y_sink[k][i]);
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

void WSN_arvore_rotulada_model_base::add_testing_valid_inequalities()
{
    IloExpr expr(env);

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &u : instance.adj_list_from_v[v])
        {
            expr += y[u] + x[v][u] + x[u][v];
        }

        constraints.add(4 * z[v] <= expr); // des. val. 1

        expr.end();
        expr = IloExpr(env);
    }

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            constraints.add((x[i][j] + x[j][i]) <= z[i] + z[j]); // des. val. 2
        }
    }

    expr.end();
}

void WSN_arvore_rotulada_model_base::add_remove_symmetries()
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
                    expr += x_sink[s][instance.n][u];
                }

                for (int j = k; j < instance.number_trees; j++)
                {
                    expr -= x_sink[j][instance.n][v];
                }

                constraints.add(expr >= 0); // test_5

                expr.end();
                expr = IloExpr(env);
            }
        }
    }

    expr.end();
}

inline IloModel WSN_arvore_rotulada_model_base::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_2_index(relaxed, y_sink);
    relaxed = relax_utils::relax_2_index(relaxed, z_sink);

    relaxed = relax_utils::relax_3_index(relaxed, x_sink);

    return relaxed;
}

void WSN_arvore_rotulada_model_base::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto [x_sink_full, x_sink_values] = read_matrix_3d(x_sink, cplex, 1);

    print_matrix(x_sink_full, x_sink_values, "x_sink", cout);

    auto [y_sink_full, y_sink_values] = read_full_matrix(y_sink, cplex, 1);

    print_matrix(y_sink_full, y_sink_values, "y_sink", cout);
}