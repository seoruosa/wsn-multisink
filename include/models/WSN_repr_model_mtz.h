#pragma once

#include "WSN_repr_model_base.h"

class WSN_repr_model_mtz_base : public WSN_representante_model_base
{
public:
    WSN_repr_model_mtz_base(WSN_data &instance);
    WSN_repr_model_mtz_base(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
};

WSN_repr_model_mtz_base::WSN_repr_model_mtz_base(WSN_data &instance) : WSN_representante_model_base(instance)
{
    WSN::formulation_name = "REPR-mtz-base";
}

WSN_repr_model_mtz_base::WSN_repr_model_mtz_base(WSN_data &instance,
                                                 double upper_bound) : WSN_representante_model_base(instance, upper_bound)
{
    WSN::formulation_name = "REPR-mtz-base";
}

inline void WSN_repr_model_mtz_base::build_model()
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

    // remove subtours - mtz
    add_mtz_model_variables();
    add_mtz_subtour_elimination_constraints();

    add_objective_function();
}

class WSN_repr_model_mtz : public WSN_representante_model_base
{
public:
    WSN_repr_model_mtz(WSN_data &instance);
    WSN_repr_model_mtz(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
};

WSN_repr_model_mtz::WSN_repr_model_mtz(WSN_data &instance) : WSN_representante_model_base(instance)
{
    WSN::formulation_name = "REPR-mtz";
}

WSN_repr_model_mtz::WSN_repr_model_mtz(WSN_data &instance,
                                       double upper_bound) : WSN_representante_model_base(instance, upper_bound)
{
    WSN::formulation_name = "REPR-mtz";
}

inline void WSN_repr_model_mtz::build_model()
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

    // remove subtours - mtz
    add_mtz_model_variables();
    add_mtz_subtour_elimination_constraints();
    add_mtz_valid_inequalities();
    // add_mtz_bektas2014_inequalities();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_testing_valid_inequalities();

    add_objective_function();
}