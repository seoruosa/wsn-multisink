#pragma once

#include "WSN_repr_model_base.h"

class WSN_repr_model_flow_base : public WSN_representante_model_base
{
public:
    WSN_repr_model_flow_base(WSN_data &instance);
    WSN_repr_model_flow_base(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
};

WSN_repr_model_flow_base::WSN_repr_model_flow_base(WSN_data &instance) : WSN_representante_model_base(instance)
{
    WSN::formulation_name = "REPR-flow-base";
}

WSN_repr_model_flow_base::WSN_repr_model_flow_base(WSN_data &instance,
                                                   double upper_bound) : WSN_representante_model_base(instance, upper_bound)
{
    WSN::formulation_name = "REPR-flow-base";
}

inline void WSN_repr_model_flow_base::build_model()
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

    add_objective_function();
}

class WSN_repr_model_flow : public WSN_representante_model_base
{
public:
    WSN_repr_model_flow(WSN_data &instance);
    WSN_repr_model_flow(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
};

WSN_repr_model_flow::WSN_repr_model_flow(WSN_data &instance) : WSN_representante_model_base(instance)
{
    WSN::formulation_name = "REPR-flow";
}

WSN_repr_model_flow::WSN_repr_model_flow(WSN_data &instance,
                                         double upper_bound) : WSN_representante_model_base(instance, upper_bound)
{
    WSN::formulation_name = "REPR-flow";
}

inline void WSN_repr_model_flow::build_model()
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

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_testing_valid_inequalities();

    add_objective_function();
}
