#pragma once

#include "WSN.h"
#include <limits>

#include "wsn_constructive_heur.h"
#include "WSN_mcf_model_base.h"

class WSN_mcf_model_weight_on_node : public WSN_mcf_model_base
{
public:
    WSN_mcf_model_weight_on_node(WSN_data &instance);
    WSN_mcf_model_weight_on_node(WSN_data &instance, double upper_bound);

private:
    virtual void build_model() override;
    void add_objective_function() override;

    IloArray<IloNumVarArray> w;
    IloNumVarArray t;
    double M;

    void add_weight_calculation_variables();
    // constraints to calculate the weight of trees
    void add_calculate_weight_tree_constraints();
    double calculates_big_M();
};

WSN_mcf_model_weight_on_node::WSN_mcf_model_weight_on_node(WSN_data &instance) : WSN_mcf_model_base(instance),
                                                                                 w(IloArray<IloNumVarArray>(env, instance.n)),
                                                                                 t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                                                 M(calculates_big_M())
{
    WSN::formulation_name = "MCF-Model-weight-node-refactor";
}

WSN_mcf_model_weight_on_node::WSN_mcf_model_weight_on_node(WSN_data &instance,
                                                           double upper_bound) : WSN_mcf_model_base(instance, upper_bound),
                                                                                 w(IloArray<IloNumVarArray>(env, instance.n)),
                                                                                 t(IloNumVarArray(env, instance.n, 0, IloInfinity, ILOFLOAT)),
                                                                                 M(calculates_big_M())
{
    WSN::formulation_name = "MCF-Model-weight-node-refactor";
}

inline void WSN_mcf_model_weight_on_node::build_model()
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
    add_weight_calculation_variables();
    add_ahani2019_mcf_constraints();

    add_connect_sink_assignment_constraints();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_mcf_valid_inequalities();

    add_objective_function();
}

inline void WSN_mcf_model_weight_on_node::add_objective_function()
{
    WSN_mcf_model_base::add_objective_function();

    for (int i = 0; i < instance.n; i++)
    {
        constraints.add(T >= t[i]);
    }
}

inline void WSN_mcf_model_weight_on_node::add_weight_calculation_variables()
{
    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
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

inline void WSN_mcf_model_weight_on_node::add_calculate_weight_tree_constraints()
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
        constraints.add(t[i] == expr);              // constraints doc 15

        expr.end();
        expr = IloExpr(env);
    }
}

inline double WSN_mcf_model_weight_on_node::calculates_big_M()
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