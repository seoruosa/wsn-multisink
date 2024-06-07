#pragma once

#include "WSN_arvore_rotulada_model_base.h"

class WSN_arv_rot_model_flow_base : public WSN_arvore_rotulada_model_base
{
public:
    WSN_arv_rot_model_flow_base(WSN_data &instance);

private:
    virtual void build_model();
};

WSN_arv_rot_model_flow_base::WSN_arv_rot_model_flow_base(WSN_data &instance) : WSN_arvore_rotulada_model_base(instance)
{
    WSN::formulation_name = "MAR-flow-base";
}

inline void WSN_arv_rot_model_flow_base::build_model()
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

    // remove subtours - flow-based
    add_flow_model_variables();
    add_flow_limit_constraints();
    add_flow_conservation_constraints();

    add_objective_function();
}

class WSN_arv_rot_model_flow : public WSN_arvore_rotulada_model_base
{
public:
    WSN_arv_rot_model_flow(WSN_data &instance);

private:
    virtual void build_model();
};

WSN_arv_rot_model_flow::WSN_arv_rot_model_flow(WSN_data &instance) : WSN_arvore_rotulada_model_base(instance)
{
    WSN::formulation_name = "MAR-flow";
}

inline void WSN_arv_rot_model_flow::build_model()
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

    // remove subtours - flow-based
    add_flow_model_variables();
    add_flow_limit_constraints();
    add_flow_conservation_constraints();

    // Valid inequalities
    add_CastroAndrade2023_valid_inequalities();
    add_adasme2023_valid_inequalities();
    add_arv_rotulada_valid_inequalities();

    add_testing_valid_inequalities();
    add_remove_symmetries();

    add_objective_function();
}