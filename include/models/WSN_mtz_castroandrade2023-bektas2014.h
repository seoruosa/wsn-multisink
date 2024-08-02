#pragma once

#include "WSN_mtz_castroandrade2023-SBPO.h"

class WSN_mtz_castro_andrade_2023_bektas2014 : public WSN_mtz_castro_andrade_2023_sbpo
{
public:
    WSN_mtz_castro_andrade_2023_bektas2014(WSN_data &instance);
    WSN_mtz_castro_andrade_2023_bektas2014(WSN_data &instance, double upper_bound);

private:
    virtual void build_model() override;
};

WSN_mtz_castro_andrade_2023_bektas2014::WSN_mtz_castro_andrade_2023_bektas2014(WSN_data &instance) : WSN_mtz_castro_andrade_2023_sbpo(instance)
{
    WSN::formulation_name = "MTZ-castro2023-bektas";
}

WSN_mtz_castro_andrade_2023_bektas2014::WSN_mtz_castro_andrade_2023_bektas2014(WSN_data &instance,
                                                                               double upper_bound) : WSN_mtz_castro_andrade_2023_sbpo(instance, upper_bound)
{
    WSN::formulation_name = "MTZ-castro2023-bektas";
}

void WSN_mtz_castro_andrade_2023_bektas2014::build_model()
{
    // create_basic_model_constraints(); // constraints 2, 4-9

    add_decision_variables();

    add_number_dominating_nodes_constraints(); // Const CA2023 -> 1
    add_number_forest_edges_constraints();     // Const CA2023 -> 1
    add_node_master_or_bridge_constraints();   // Const CA2023 -> 6
    add_master_neighbor_constraints();         // Const CA2023 -> 4
    add_master_not_adj_master_constraints();   // Const CA2023 -> 5
    add_bridges_not_neighbor_constraints();    // Const CA2023 -> 7
    add_bridge_master_neighbor_constraints();  // Const CA2023 -> 8

    add_in_coming_edge_mtz_constraints(); // Const CA2023 -> 3

    add_mtz_model_variables();
    add_subtour_constraints(); // Const CA2023 ->  2

    add_calculate_weight_tree_constraints(); // Const CA2023 -> pag. 5
    add_lower_bound_weight_constraints();    // Const CA2023 -> 25
    add_leaf_constraints();                  // Const CA2023 -> 13
    add_trivial_tree_constraints();          // Const CA2023 -> 14

    add_castrodeAndrade2023_constraints(); // Const CA2023 -> 12, 15, 16, 17, 10 (implementation had been corrected),
    //                                                             18, 19, 20, 21, 22, 23

    // add_adasme2023_valid_inequalities();
    add_bektas2014_constraints();

    add_objective_function(); // Const CA2023 -> pag 5
}