#pragma once

#include "WSN.h"
#include <limits>

#include "wsn_constructive_heur.h"

class WSN_representante_model_base : public WSN
{
public:
    WSN_representante_model_base(WSN_data &instance);
    WSN_representante_model_base(WSN_data &instance, double upper_bound);

protected:
    virtual void build_model();

    IloArray<IloArray<IloNumVarArray>> x_sink; // arc-sink assignment

    IloArray<IloNumVarArray> y_sink; // master sink assignment
    IloArray<IloNumVarArray> z_sink; // bridge sink assignment

    IloArray<IloNumVarArray> f;

    int p;
    IloNumVarArray pi;

    virtual void add_objective_function();

    // basic model
    void add_connect_sink_assignment_constraints();

    // flow constraints
    void add_flow_model_variables();
    void add_flow_limit_constraints();
    void add_flow_conservation_constraints();
    void add_flow_valid_inequalities();

    // repr constraints
    void add_repr_model_variables();
    void add_repr_constraints();
    void add_repr_valid_inequalities();

    // mtz constraints
    void add_mtz_model_variables();
    void add_mtz_subtour_elimination_constraints();
    void add_mtz_valid_inequalities();
    void add_mtz_bektas2014_inequalities();

    // Valid inequalities
    void add_CastroAndrade2023_valid_inequalities();
    void add_adasme2023_valid_inequalities();

    void add_testing_valid_inequalities();

    virtual IloModel create_relaxed();

    virtual void print_full(IloCplex &cplex, std::ostream &cout);

    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_representante_model_base::WSN_representante_model_base(WSN_data &instance) : WSN(instance, "REPR-base"),
                                                                                 x_sink(IloArray<IloArray<IloNumVarArray>>(env, instance.n)),
                                                                                 y_sink(IloArray<IloNumVarArray>(env)),
                                                                                 z_sink(IloArray<IloNumVarArray>(env)),
                                                                                 f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                                 p((instance.n - instance.number_trees)),
                                                                                 pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT))
{
}

WSN_representante_model_base::WSN_representante_model_base(WSN_data &instance,
                                                           double upper_bound) : WSN(instance, "REPR-base", upper_bound),
                                                                                 x_sink(IloArray<IloArray<IloNumVarArray>>(env, instance.n)),
                                                                                 y_sink(IloArray<IloNumVarArray>(env)),
                                                                                 z_sink(IloArray<IloNumVarArray>(env)),
                                                                                 f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                                 p((instance.n - instance.number_trees)),
                                                                                 pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT))
{
}

inline void WSN_representante_model_base::build_model()
{
    // basic model
    add_decision_variables();
    add_number_dominating_nodes_constraints();
    add_number_forest_edges_constraints();

    add_master_neighbor_constraints();
    add_master_not_adj_master_constraints();
    add_bridges_not_neighbor_constraints();
    add_bridge_master_neighbor_constraints();

    add_trivial_tree_constraints();

    // repr model
    add_repr_model_variables();
    add_repr_constraints();
    add_connect_sink_assignment_constraints();
    add_repr_valid_inequalities();

    // remove subtours - flow-based
    add_flow_model_variables();
    add_flow_limit_constraints();
    add_flow_conservation_constraints();
    add_flow_valid_inequalities();

    // // remove subtours - mtz
    // add_mtz_model_variables();
    // add_mtz_subtour_elimination_constraints();
    // add_mtz_valid_inequalities();
    // // add_mtz_bektas2014_inequalities();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_testing_valid_inequalities();

    add_objective_function();
}

void WSN_representante_model_base::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

inline void WSN_representante_model_base::add_objective_function()
{

    model.add(IloMinimize(env, T));

    // constraint used to min-max the tree weight (not yet defined, now it's the forest weight)
    IloExpr expr(env);

    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u; v < instance.n; v++)
        {
            for (auto &w : instance.adj_list_to_v[v])
            {
                if (w >= u && v > w)
                {
                    expr += instance.weight[w][v] * (x_sink[u][w][v] + x_sink[u][v][w]);
                }
            }
        }
        model.add(T >= expr);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_representante_model_base::add_repr_model_variables()
{
    T.setName("T");

    y_sink = IloArray<IloNumVarArray>(env, instance.n);
    z_sink = IloArray<IloNumVarArray>(env, instance.n);

    // Creating arrays
    for (int k = 0; k < instance.n; k++)
    {
        x_sink[k] = IloArray<IloNumVarArray>(env, instance.n);

        y_sink[k] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);
        z_sink[k] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);

        for (int i = 0; i < instance.n; i++)
        {
            x_sink[k][i] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);

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

void WSN_representante_model_base::add_mtz_model_variables()
{
    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        pi[i].setName(("pi(" + std::to_string(i) + ")").c_str());
    }
}

void WSN_representante_model_base::add_mtz_subtour_elimination_constraints()
{
    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u; v < instance.n; v++)
        {
            for (auto &w : instance.adj_list_from_v[v])
            {
                if (w > v)
                {
                    constraints.add(((p - 1) * x_sink[u][w][v] - (p + 1) * (1 - x_sink[u][v][w]) + 1) <= (pi[w] - pi[v]));  // 5.21
                    constraints.add((-(p + 1) * x_sink[u][w][v] + (p - 1) * (1 - x_sink[u][v][w]) + 1) >= (pi[w] - pi[v])); // 5.21
                }
            }
        }
    }

    for (int u = 0; u < instance.n; u++)
    {
        constraints.add(pi[u] <= p * (1 - y_sink[u][u])); // 5.22
    }
}

void WSN_representante_model_base::add_mtz_valid_inequalities()
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

void WSN_representante_model_base::add_mtz_bektas2014_inequalities()
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

inline void WSN_representante_model_base::add_connect_sink_assignment_constraints()
{
    IloExpr expr_x(env);
    IloExpr expr_y(env);
    IloExpr expr_z(env);
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k <= i; k++)
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
    }

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &w : instance.adj_list_to_v[v])
        {
            for (int u = 0; u <= v; u++)
            {
                if (w >= u)
                {
                    expr_x += x_sink[u][v][w];
                }
            }

            constraints.add(x[v][w] == expr_x);

            expr_x.end();
            expr_x = IloExpr(env);
        }
    }

    for (int v = 0; v < instance.n; v++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            expr += x[instance.n + k][v];
        }
    }

    constraints.add(expr == 0);

    expr_x.end();
    expr_y.end();
    expr_z.end();
    expr.end();
}

inline void WSN_representante_model_base::add_flow_model_variables()
{
    for (int i = 0; i < instance.n + 1; i++)
    {
        f[i] = IloNumVarArray(env, instance.n, 0, instance.n - instance.number_trees, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN_representante_model_base::add_flow_limit_constraints()
{

    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += f[instance.n][i];
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
        constraints.add((y_sink[i][i] + z_sink[i][i]) <= f[instance.n][i]);
    }

    // exp 8
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][to] <= (instance.n - instance.number_trees - 1) * x[i][to]);
        }

        constraints.add(f[instance.n][i] <= (instance.n - instance.number_trees - 1) * (y_sink[i][i] + z_sink[i][i]));
    }
}

void WSN_representante_model_base::add_flow_conservation_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += f[instance.n][i];

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

inline void WSN_representante_model_base::add_flow_valid_inequalities()
{
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {

            // expr += f[from][i];
            constraints.add(2 * x[from][i] - z[from] <= f[from][i]);
        }

        // Constraints 47
        // constraints.add(2 * z[i] <= expr); // TESTEI E NÂO É VALIDA

        expr.end();
        expr = IloExpr(env);
    }

    for (int u = 0; u < instance.n; u++)
    {
        for (auto &v : instance.adj_list_from_v[u])
        {
            expr += f[instance.n][u];

            for (auto &k : instance.adj_list_to_v[u])
            {
                expr += f[k][u];
            }

            constraints.add(f[u][v] + x[u][v] * (instance.n - 1) - expr <= (instance.n - 2));

            expr.end();
            expr = IloExpr(env);
        }
    }

    expr.end();
}

inline void WSN_representante_model_base::add_repr_constraints()
{
    IloExpr expr(env);

    auto clear_expr = [&]
    {
        expr.end();
        expr = IloExpr(env);
    };

    for (int u = 0; u < instance.n; u++)
    {
        expr += (y_sink[u][u] + z_sink[u][u]);
    }

    constraints.add(expr == instance.number_trees); // 5.15
    clear_expr();

    for (int v = 0; v < instance.n; v++)
    {
        for (int u = 0; u <= v; u++)
        {
            expr += (y_sink[u][v] + z_sink[u][v]);

            constraints.add(y_sink[u][u] + z_sink[u][u] >= y_sink[u][v] + z_sink[u][v]); // 5.17
        }
        constraints.add(expr <= 1); // 5.16

        clear_expr();
    }

    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u; v < instance.n; v++)
        {
            for (auto &w : instance.adj_list_from_v[v])
            {
                if (w >= u)
                {
                    constraints.add((x_sink[u][v][w] + x_sink[u][w][v]) <= (y_sink[u][v] + z_sink[u][v])); // 5.18
                    constraints.add((x_sink[u][v][w] + x_sink[u][w][v]) <= (y_sink[u][w] + z_sink[u][w])); // 5.18
                }
            }
        }
    }

    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u + 1; v < instance.n; v++)
        {
            expr += x_sink[u][v][u];
        }

        constraints.add(expr == 0); // 5.19

        clear_expr();
    }

    for (int u = 0; u < instance.n; u++)
    {
        for (int w = u + 1; w < instance.n; w++)
        {
            for (auto &v : instance.adj_list_to_v[w])
            {
                if (v >= u)
                {
                    expr += x_sink[u][v][w];
                }
            }

            constraints.add(expr == (y_sink[u][w] + z_sink[u][w])); // 5.20

            clear_expr();
        }
    }

    expr.end();
}

inline void WSN_representante_model_base::add_repr_valid_inequalities()
{
    IloExpr expr(env);

    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u + 1; v < instance.n; v++)
        {
            for (auto &w : instance.adj_list_from_v[u])
            {
                if (w > u)
                {
                    expr += x_sink[u][u][w];
                }
            }

            constraints.add((y_sink[u][v] + z_sink[u][v]) <= expr);

            expr.end();
            expr = IloExpr(env);
        }
    }

    expr.end();
}

inline void WSN_representante_model_base::add_CastroAndrade2023_valid_inequalities()
{
    IloExpr expr(env);

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

inline void WSN_representante_model_base::add_adasme2023_valid_inequalities()
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

                // constraint 21
                constraints.add(2 * (x[from][i] + x[i][from]) <= y[i] + z[from]);
            }
        }
    }

    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {

        expr += (y[i] - z[i]);
    }
    constraints.add(expr >= instance.number_trees);

    expr.end();
}

void WSN_representante_model_base::add_testing_valid_inequalities()
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

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += instance.weight[i][j] * x[i][j];
        }

        constraints.add(T >= expr); // constraints doc 16

        expr.end();
        expr = IloExpr(env);
    }

    for (int u = 0; u < instance.n; u++)
    {
        for (int v = u; v < instance.n; v++)
        {
            expr += (y_sink[u][v] - z_sink[u][v]);
        }

        constraints.add(expr >= (y_sink[u][u] - z_sink[u][u]));

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline IloModel WSN_representante_model_base::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_2_index(relaxed, y_sink);
    relaxed = relax_utils::relax_2_index(relaxed, z_sink);

    relaxed = relax_utils::relax_3_index(relaxed, x_sink);

    return relaxed;
}

void WSN_representante_model_base::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto [x_sink_full, x_sink_values] = read_matrix_3d(x_sink, cplex, 1);

    print_matrix(x_sink_full, x_sink_values, "x_sink", cout);

    auto [y_sink_full, y_sink_values] = read_full_matrix(y_sink, cplex, 1);

    print_matrix(y_sink_full, y_sink_values, "y_sink", cout);

    auto [z_sink_full, z_sink_values] = read_full_matrix(z_sink, cplex, 1);

    print_matrix(z_sink_full, z_sink_values, "z_sink", cout);
}