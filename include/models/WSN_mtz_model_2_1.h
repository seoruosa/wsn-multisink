#pragma once

#include "WSN.h"
#include "wsn_data.h"
#include "wsn_constructive_heur.h"

class WSN_mtz_model_2_1 : public WSN
{
public:
    WSN_mtz_model_2_1(WSN_data &instance);
    // ~WSN_mtz_model_2();

private:
    virtual void build_model();
    virtual void add_objective_function();

    IloArray<IloNumVarArray> w;
    IloNumVarArray t;
    IloNumVarArray pi;

    int p;
    double M;

    void add_mtz_model_variables();

    // MTZ subtour elimination constraints
    void add_subtour_constraints();

    void add_in_coming_edge_mtz_constraints();

    // constraints to calculate the weight of trees
    void add_calculate_weight_tree_constraints();

    // lower bound to the weight of node
    void add_lower_bound_weight_constraints();

    // leaf constraints
    void add_leaf_constraints();

    void add_bektas2014_constraints();

    void add_castrodeAndrade2023_constraints();

    // calculates an big-M
    double calculates_big_M();

    void add_adasme2023_valid_inequalities();

    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_mtz_model_2_1::WSN_mtz_model_2_1(WSN_data &instance) : WSN(instance, "MTZModelStrengthened2"),
                                                           w(IloArray<IloNumVarArray>(env, instance.n)),
                                                           t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                           pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                           p((instance.n - instance.number_trees + 1) / 2),
                                                           M(calculates_big_M())
{
}

void WSN_mtz_model_2_1::build_model()
{
    // create_basic_model_constraints(); // constraints 2, 4-9

    add_decision_variables();

    add_number_dominating_nodes_constraints();
    add_number_forest_edges_constraints();
    add_node_master_or_bridge_constraints();
    add_master_neighbor_constraints();
    add_master_not_adj_master_constraints();
    add_bridges_not_neighbor_constraints();
    add_bridge_master_neighbor_constraints();

    add_in_coming_edge_mtz_constraints();

    add_mtz_model_variables();
    add_subtour_constraints(); // constraints 3

    add_calculate_weight_tree_constraints();
    add_lower_bound_weight_constraints();
    add_leaf_constraints();

    add_bektas2014_constraints();
    add_castrodeAndrade2023_constraints();
    add_adasme2023_valid_inequalities();

    add_trivial_tree_constraints();

    add_objective_function();
}

void WSN_mtz_model_2_1::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

void WSN_mtz_model_2_1::add_objective_function()
{
    model.add(IloMinimize(env, T));

    for (int i = 0; i < instance.n; i++)
    {
        constraints.add(T >= t[i]);
    }
}

void WSN_mtz_model_2_1::add_mtz_model_variables()
{
    T.setName("T");

    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        pi[i].setName(("pi(" + std::to_string(i) + ")").c_str());
        t[i].setName(("t(" + std::to_string(i) + ")").c_str());
    }

    for (int i = 0; i < instance.n; i++)
    {
        w[i] = IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            w[i][j].setName(("w(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN_mtz_model_2_1::add_subtour_constraints()
{
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // strengthened MTZ subtour elimination constraints
            constraints.add(pi[j] - pi[i] - p * x[i][j] - (p - 2) * x[j][i] >= 1 - p);
            // constraints.add(pi[j] - pi[i] - (instance.n - 1) * x[i][j] - (instance.n - 3) * x[j][i] >= 2 - instance.n);
        }
    }
}

void WSN_mtz_model_2_1::add_in_coming_edge_mtz_constraints()
{
    // Constraints 4
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += x[from][i];
        }

        constraints.add(expr <= (y[i] + z[i]));

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_mtz_model_2_1::add_calculate_weight_tree_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += w[i][j];

            constraints.add(w[i][j] <= M * x[i][j]);                                                // constraints doc 12
            constraints.add(t[j] + instance.weight[i][j] * x[i][j] + M * (1 - x[i][j]) >= w[i][j]); // constraints doc 13
            constraints.add(t[j] + instance.weight[i][j] * x[i][j] - M * (1 - x[i][j]) <= w[i][j]); // constraints doc 14
        }

        constraints.add(t[i] <= M * (y[i] + z[i])); // constraints doc 11
        constraints.add(t[i] >= expr);              // constraints doc 15
        // constraints.add(); // constraints doc

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_mtz_model_2_1::add_lower_bound_weight_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += instance.weight[i][j] * x[i][j];
        }

        constraints.add(t[i] >= expr); // constraints doc 16

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_mtz_model_2_1::add_leaf_constraints()
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

double WSN_mtz_model_2_1::calculates_big_M()
{
    double M_weight = 1.0;
    // std::vector<double> weights;

    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (auto &j : instance.adj_list_from_v[i])
    //     {
    //         weights.push_back(instance.weight[i][j]);
    //     }
    // }

    // std::sort(weights.begin(), weights.end(), std::greater<double>());

    // for (int i = 0; i < (instance.n - instance.number_trees) && i < weights.size(); i++)
    // {
    //     M_weight += weights[i];
    // }

    WSNConstructiveHeuristic heur(instance);

    auto sol = heur.solve();
    M_weight = heur.weight_of_solution() + 1;

    return M_weight;
}

void WSN_mtz_model_2_1::add_bektas2014_constraints()
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

void WSN_mtz_model_2_1::add_castrodeAndrade2023_constraints()
{

    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += (x[i][j] + x[j][i]);

            constraints.add((x[i][j] + x[j][i]) <= y[i] + z[i]); // constraints 15
        }

        constraints.add(2 * z[i] <= expr); // constraints 12

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_mtz_model_2_1::add_adasme2023_valid_inequalities()
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

    constraints.add(exp_ad_28 >= 1);

    exp_ad_28.end();

    for (int i = 0; i < instance.n; i++)
    {
        // Constraints 50
        constraints.add(x[instance.n][i] <= y[i]);
    }

    // Contraints 29
    IloExpr exp_ad_29(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            exp_ad_29 += z[to];
        }

        constraints.add(exp_ad_29 <= y[i] + instance.adj_list_from_v[i].size() - 1);

        exp_ad_29.end();
        exp_ad_29 = IloExpr(env);
    }

    exp_ad_29.end();

    // Contraints 30
    IloExpr exp_ad_30(env);

    for (int i = 0; i < instance.n; i++)
    {
        exp_ad_30 += (pi[i] - z[i] - 2 * y[i]);
    }

    constraints.add(exp_ad_30 >= -2);
    exp_ad_30.end();

    IloExpr exp_ad_32(env);
    for (int i = 0; i < instance.n; i++)
    {
        // Constraints 31
        constraints.add(pi[i] >= z[i]);

        for (auto &from : instance.adj_list_to_v[i])
        {
            exp_ad_32 += x[from][i];
        }

        // Constraints 32
        constraints.add(pi[i] >= 2 * exp_ad_32 - z[i]);

        exp_ad_32.end();
        exp_ad_32 = IloExpr(env);
    }

    exp_ad_32.end();

    IloExpr exp_ad_33(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            for (auto &k : instance.adj_list_to_v[from])
            {
                exp_ad_33 += x[k][from];
            }

            // Constraints 33
            constraints.add(pi[i] >= 2 * exp_ad_33 + (y[from] + z[i] - 1) - 2 * (1 - x[from][i]));

            exp_ad_33.end();
            exp_ad_33 = IloExpr(env);
        }
    }

    exp_ad_33.end();
}