#pragma once

#include "WSN.h"
// #include "wsn_constructive_heur.h"

class WSN_flow_model_0_1 : public WSN
{
public:
    WSN_flow_model_0_1(WSN_data &instance);
    WSN_flow_model_0_1(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();

    IloArray<IloNumVarArray> f;
    IloArray<IloNumVarArray> w;
    IloNumVarArray t;

    double M;

    virtual void add_objective_function();

    void add_flow_model_variables();

    void add_flow_limit_constraints();

    void add_flow_conservation_constraints();

    // constraints that uses the extra node
    void add_extra_node_constraints();

    void add_calculate_weight_tree_constraints();

    void add_lower_bound_constraints();

    void add_leaf_constraints();

    void add_adasme2023_valid_inequalities();
    void add_CastroAndrade2023_valid_inequalities();

    // calculates an big-M
    double calculates_big_M();
    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_flow_model_0_1::WSN_flow_model_0_1(WSN_data &instance) : WSN(instance, "FlowModel0-1"),
                                                             f(IloArray<IloNumVarArray>(env, instance.n + 1)),
                                                             w(IloArray<IloNumVarArray>(env, instance.n + 1)),
                                                             t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                             M(calculates_big_M())
{
}

WSN_flow_model_0_1::WSN_flow_model_0_1(WSN_data &instance, double upper_bound) : WSN(instance, "FlowModel0-1", upper_bound),
                                                                                 f(IloArray<IloNumVarArray>(env, instance.n + 1)),
                                                                                 w(IloArray<IloNumVarArray>(env, instance.n + 1)),
                                                                                 t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                                                 M(calculates_big_M())
{
}

void WSN_flow_model_0_1::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

void WSN_flow_model_0_1::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_flow_limit_constraints();              // exp 5, 7, 8
    add_flow_conservation_constraints();       // exp 6

    add_extra_node_constraints();             // exp 9, 10
    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14
    add_trivial_tree_constraints();

    add_calculate_weight_tree_constraints(); // exp 15, 16, 17, 18, 19
    add_lower_bound_constraints();           // exp 20
    add_master_neighbor_constraints();       // exp 21
    add_leaf_constraints();                  // exp 22

    add_adasme2023_valid_inequalities();
    add_CastroAndrade2023_valid_inequalities();

    add_objective_function();
}

void WSN_flow_model_0_1::add_objective_function()
{
    model.add(IloMinimize(env, T));

    for (int i = 0; i < instance.n; i++)
    {
        constraints.add(T >= t[i]); // exp 2
    }
}

void WSN_flow_model_0_1::add_flow_model_variables()
{
    // Naming variables
    T.setName("T");
    for (int i = 0; i < instance.n; i++)
    {
        t[i].setName(("t(" + std::to_string(i) + ")").c_str());
    }

    for (int i = 0; i <= instance.n; i++)
    {
        w[i] = IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT);
        f[i] = IloNumVarArray(env, instance.n, 0, instance.n - instance.number_trees, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            w[i][j].setName(("w(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN_flow_model_0_1::add_flow_limit_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += f[instance.n][i];
    }

    constraints.add(expr == N);
    expr.end();

    // exp 7
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(x[i][to] <= f[i][to]);
        }

        // arc from node r
        constraints.add(x[instance.n][i] <= f[instance.n][i]);
    }

    // exp 8
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][to] <= (instance.n - instance.number_trees - 1) * x[i][to]);
        }

        // arc from node r
        constraints.add(f[instance.n][i] <= (instance.n - instance.number_trees - 1) * x[instance.n][i]);
    }
}

void WSN_flow_model_0_1::add_flow_conservation_constraints()
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

void WSN_flow_model_0_1::add_extra_node_constraints()
{
    // exp 9
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        expr += x[instance.n][i];
    }
    constraints.add(expr == instance.number_trees);
    expr.end();

    // exp 10
    expr = IloExpr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += x[from][i];
        }
        expr += x[instance.n][i];

        constraints.add(expr == (y[i] + z[i]));
        expr.end();
        expr = IloExpr(env);
    }
    expr.end();
}

void WSN_flow_model_0_1::add_calculate_weight_tree_constraints()
{
    // exp 15
    for (int i = 0; i < instance.n; i++)
    {
        constraints.add(t[i] <= M * (y[i] + z[i]));
    }

    // exp 16
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            constraints.add(w[from][i] <= M * x[from][i]);
        }
        constraints.add(w[instance.n][i] <= M * x[instance.n][i]);
    }

    // exp 17 e 18
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            constraints.add((t[i] + instance.weight[from][i] * x[from][i] + M * (1 - x[from][i])) >= w[from][i]);
            constraints.add((t[i] + instance.weight[from][i] * x[from][i] - M * (1 - x[from][i])) <= w[from][i]);
        }

        constraints.add((t[i] + M * (1 - x[instance.n][i])) >= w[instance.n][i]);
        constraints.add((t[i] - M * (1 - x[instance.n][i])) <= w[instance.n][i]);
    }

    // exp 19
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            expr += w[i][to];
        }
        constraints.add(t[i] >= expr);

        expr.end();
        expr = IloExpr(env);
    }
    expr.end();
}

void WSN_flow_model_0_1::add_lower_bound_constraints()
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

void WSN_flow_model_0_1::add_leaf_constraints()
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

inline void WSN_flow_model_0_1::add_adasme2023_valid_inequalities()
{
    // constraints Adasme2023
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            for (auto &from : instance.adj_list_to_v[i])
            {
                // constraint 20
                constraints.add(y[from] + y[i] == 1);

                // constraint 21
                constraints.add(2 * (x[from][i] + x[i][from]) <= y[i] + z[from]);
            }
        }
    }

    IloExpr exp_ad_28(env);
    for (int i = 0; i < instance.n; i++)
    {
        exp_ad_28 += (y[i] - z[i]);
    }

    constraints.add(exp_ad_28 >= 1);

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
        for (auto &j : instance.adj_list_from_v[i])
        {
            // Constraints 48
            constraints.add(2 * x[i][j] - z[i] <= f[i][j]);
        }
    }

    IloExpr exp_ad_49(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            exp_ad_49 += f[from][i];
        }
        exp_ad_49 += f[instance.n][i];

        // Constraints 49

        for (auto &j : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][j] + x[i][j] * (instance.n - 1) - exp_ad_49 <= instance.n - 2);
        }

        exp_ad_49.end();
        exp_ad_49 = IloExpr(env);
    }
    exp_ad_49.end();

    for (int i = 0; i < instance.n; i++)
    {
        // Constraints 50
        constraints.add(x[instance.n][i] <= y[i]);
    }
}

void WSN_flow_model_0_1::add_CastroAndrade2023_valid_inequalities()
{
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            constraints.add((x[i][j] + x[j][i]) <= y[i] + z[i]); // constraints 15
        }
    }
}

double WSN_flow_model_0_1::calculates_big_M()
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

    // WSNConstructiveHeuristic heur(instance);

    // auto sol = heur.solve();
    // M_weight = heur.weight_of_solution() + 1;

    return M_weight;
}