#pragma once

#include "WSN.h"
#include "WSN_flow_model_3_base.h"

/**
 * @brief Model that return the number of nodes that is a master or neighbor of master.
 * It is usefull to check if a instance is feasible
 *
 */
class WSN_flow_model_3_check_instance : public WSN_flow_model_3_base
{
public:
    WSN_flow_model_3_check_instance(WSN_data &instance);

private:
    IloNumVarArray t; // have a master as neighbor
    
    virtual void build_model();
    virtual void add_objective_function();

    void add_check_have_neighbors();
    void add_check_model_variables();

    virtual void set_params_cplex(IloCplex &cplex);
    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
};

WSN_flow_model_3_check_instance::WSN_flow_model_3_check_instance(WSN_data &instance) : WSN_flow_model_3_base(instance),
                                                                                       t(IloNumVarArray(env))
{
    WSN::formulation_name = "FlowModel3-checking-test";
}

void WSN_flow_model_3_check_instance::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
}

void WSN_flow_model_3_check_instance::build_model()
{
    add_decision_variables();
    add_flow_model_variables();
    add_check_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp

    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14

    add_trivial_tree_constraints();

    add_flow_limit_constraints();        // exp 5, 7, 8
    add_flow_conservation_constraints(); // exp 6
    add_extra_node_constraints();        // exp 9, 10

    add_check_have_neighbors();

    add_objective_function();
}

void WSN_flow_model_3_check_instance::add_objective_function()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += t[i];
    }

    model.add(IloMaximize(env, expr));

    expr.end();
}

void WSN_flow_model_3_check_instance::add_check_model_variables()
{
    t = IloNumVarArray(env, instance.n, 0, 1, ILOINT); // have a master as neighbor
    
    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        t[i].setName(("t(" + std::to_string(i) + ")").c_str());
    }
}

void WSN_flow_model_3_check_instance::add_check_have_neighbors()
{
    IloExpr expr(env);

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &u : instance.adj_list_from_v[v])
        {
            expr += y[u];
        }

        constraints.add(expr <= int(instance.adj_list_from_v[v].size()) * t[v]);
        constraints.add(t[v] <= expr + y[v]);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_3_check_instance::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto [t_full, t_values] = read_full_vec_to_matrix(t, cplex, 1);
    print_matrix(t_full, t_values, "t", cout);
}