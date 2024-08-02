#pragma once

#include "WSN_flow_model_3_base.h"

class WSN_flow_model_3_testing_ineq : public WSN_flow_model_3_base
{
private:
    virtual void build_model();

public:
    WSN_flow_model_3_testing_ineq(WSN_data &instance);
    WSN_flow_model_3_testing_ineq(WSN_data &instance, double upper_bound);
};

WSN_flow_model_3_testing_ineq::WSN_flow_model_3_testing_ineq(WSN_data &instance) : WSN_flow_model_3_base(instance)
{
    WSN::formulation_name = "FlowModel3-testing-ineq";
}

WSN_flow_model_3_testing_ineq::WSN_flow_model_3_testing_ineq(WSN_data &instance,
                                                             double upper_bound) : WSN_flow_model_3_base(instance, upper_bound)
{
    WSN::formulation_name = "FlowModel3-testing-ineq";
}

void WSN_flow_model_3_testing_ineq::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp

    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14
    add_master_neighbor_constraints();        // exp 21
    add_trivial_tree_constraints();

    add_flow_limit_constraints();        // exp 5, 7, 8
    add_flow_conservation_constraints(); // exp 6
    add_extra_node_constraints();        // exp 9, 10
    add_lower_bound_constraints();       // exp 20
    add_leaf_constraints();              // exp 22

    // valid inequalities
    add_adasme2023_valid_inequalities();
    add_CastroAndrade2023_valid_inequalities();
    add_remove_symmetries();
    // add_testing_valid_inequalities();
    add_lower_bound_weight_2_levels();

    add_objective_function();
}

class WSN_flow_model_3_valid_ineq : public WSN_flow_model_3_base
{
private:
    virtual void build_model();

public:
    WSN_flow_model_3_valid_ineq(WSN_data &instance);
    WSN_flow_model_3_valid_ineq(WSN_data &instance, double upper_bound);
};

WSN_flow_model_3_valid_ineq::WSN_flow_model_3_valid_ineq(WSN_data &instance) : WSN_flow_model_3_base(instance)
{
    WSN::formulation_name = "FlowModel3-valid-ineq";
}

WSN_flow_model_3_valid_ineq::WSN_flow_model_3_valid_ineq(WSN_data &instance,
                                                         double upper_bound) : WSN_flow_model_3_base(instance, upper_bound)
{
    WSN::formulation_name = "FlowModel3-valid-ineq";
}

void WSN_flow_model_3_valid_ineq::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp

    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14
    add_master_neighbor_constraints();        // exp 21
    add_trivial_tree_constraints();

    add_flow_limit_constraints();        // exp 5, 7, 8
    add_flow_conservation_constraints(); // exp 6
    add_extra_node_constraints();        // exp 9, 10
    add_lower_bound_constraints();       // exp 20
    add_leaf_constraints();              // exp 22

    // valid inequalities
    add_adasme2023_valid_inequalities();
    add_CastroAndrade2023_valid_inequalities();
    add_remove_symmetries();

    add_objective_function();
}