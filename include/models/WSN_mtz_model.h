#pragma once

#include "WSN.h"
#include "wsn_data.h"

class WSN_mtz_model : public WSN
{
public:
    WSN_mtz_model(WSN_data &instance);
    WSN_mtz_model(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
    virtual void add_objective_function();

    IloArray<IloNumVarArray> w;
    IloNumVarArray t;
    IloNumVarArray pi;

    int p;
    int M;

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
};

WSN_mtz_model::WSN_mtz_model(WSN_data &instance) : WSN(instance, "MTZModelBasic"),
                                                   w(IloArray<IloNumVarArray>(env, instance.n)),
                                                   t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                   pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                   p((instance.n - instance.number_trees + 1) / 2),
                                                   M(100000)
{
}

WSN_mtz_model::WSN_mtz_model(WSN_data &instance,
                             double upper_bound) : WSN(instance, "MTZModelBasic", upper_bound),
                                                   w(IloArray<IloNumVarArray>(env, instance.n)),
                                                   t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                   pi(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                   p((instance.n - instance.number_trees + 1) / 2),
                                                   M(100000)
{
}

void WSN_mtz_model::build_model()
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

    add_trivial_tree_constraints();

    add_objective_function();
}

void WSN_mtz_model::add_objective_function()
{
    model.add(IloMinimize(env, T));

    for (int i = 0; i < instance.n; i++)
    {
        constraints.add(T >= t[i]);
    }
}

void WSN_mtz_model::add_mtz_model_variables()
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

void WSN_mtz_model::add_subtour_constraints()
{
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // strengthened MTZ subtour elimination constraints
            // constraints.add(pi[j] - pi[i] - p * x[i][j] - (p - 2) * x[j][i] >= 1 - p);
            constraints.add(pi[j] - pi[i] - (instance.n - 1) * x[i][j] - (instance.n - 3) * x[j][i] >= 2 - instance.n);
        }
    }
}

void WSN_mtz_model::add_in_coming_edge_mtz_constraints()
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

void WSN_mtz_model::add_calculate_weight_tree_constraints()
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

void WSN_mtz_model::add_lower_bound_weight_constraints()
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

void WSN_mtz_model::add_leaf_constraints()
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