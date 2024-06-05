#pragma once

#include "WSN_flow_model_2_1_base.h"

class WSN_flow_model_2_1 : public WSN_flow_model_2_1_base
{
public:
    WSN_flow_model_2_1(WSN_data &instance);
    // ~WSN_flow_model_2_1() { WSN_flow_model_2_1_base::~WSN_flow_model_2_1_base(); };
    virtual void build_model();
};

WSN_flow_model_2_1::WSN_flow_model_2_1(WSN_data &instance) : WSN_flow_model_2_1_base(instance)
{
}

inline void WSN_flow_model_2_1::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp
    add_flow_limit_constraints();              // exp 5, 7, 8
    add_flow_conservation_constraints();       // exp 6

    add_extra_node_constraints();             // exp 9, 10
    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14

    add_lower_bound_constraints();     // exp 20
    add_master_neighbor_constraints(); // exp 21
    add_leaf_constraints();            // exp 22

    add_trivial_tree_constraints();

    add_CastroAndrade2023_valid_inequalities();
    // add_bektas2020_node_current_constraints();
    add_arc_depot_assignment_constraints();
    add_adasme2023_valid_inequalities();

    add_objective_function();
}