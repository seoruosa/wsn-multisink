#pragma once

#include "WSN_mcf_model_base.h"

class WSN_mcf_model : public WSN_mcf_model_base
{
public:
    WSN_mcf_model(WSN_data &instance);
    WSN_mcf_model(WSN_data &instance, double upper_bound);

private:
    virtual void build_model() override;
    virtual void set_params_cplex(IloCplex &cplex) override;
};

WSN_mcf_model::WSN_mcf_model(WSN_data &instance) : WSN_mcf_model_base(instance)
{
    WSN::formulation_name = "MCF-Model";
}

WSN_mcf_model::WSN_mcf_model(WSN_data &instance, double upper_bound) : WSN_mcf_model_base(instance, upper_bound)
{
    WSN::formulation_name = "MCF-Model";
}

inline void WSN_mcf_model::build_model()
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
    add_ahani2019_mcf_constraints();

    add_connect_sink_assignment_constraints();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_mcf_valid_inequalities();

    add_objective_function();
}

void WSN_mcf_model::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}