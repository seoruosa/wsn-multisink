#include "flow_based.h"

#include <iostream>
#include "wsn_data.h"

#include "all_models.h"
#include "part_model.h"

ILOSTLBEGIN

int main(int argc, char *argv[])
{
    try
    {
        auto params = read_arguments(argc, argv);

        WSN_data instance(params.instance_path, params.number_sinks);
        std::cout << instance << std::endl;

        WSN_flow_model_3_base model(instance);
        PartModel<WSN_flow_model_3_base> part(model);

        part.addBasicFunction("a", &WSN_flow_model_3_base::add_decision_variables);
        part.addBasicFunction("b", &WSN_flow_model_3_base::add_flow_model_variables);
        part.addBasicFunction("c", &WSN_flow_model_3_base::add_number_dominating_nodes_constraints);
        part.addBasicFunction("d", &WSN_flow_model_3_base::add_in_coming_edge_constraints);
        part.addBasicFunction("e", &WSN_flow_model_3_base::add_master_not_adj_master_constraints);
        part.addBasicFunction("f", &WSN_flow_model_3_base::add_node_master_or_bridge_constraints);
        part.addBasicFunction("g", &WSN_flow_model_3_base::add_bridges_not_neighbor_constraints);
        part.addBasicFunction("h", &WSN_flow_model_3_base::add_bridge_master_neighbor_constraints);
        part.addBasicFunction("i", &WSN_flow_model_3_base::add_master_neighbor_constraints);
        part.addBasicFunction("j", &WSN_flow_model_3_base::add_trivial_tree_constraints);
        part.addBasicFunction("k", &WSN_flow_model_3_base::add_flow_limit_constraints);
        part.addBasicFunction("l", &WSN_flow_model_3_base::add_flow_conservation_constraints);
        part.addBasicFunction("m", &WSN_flow_model_3_base::add_extra_node_constraints);
        part.addBasicFunction("p", &WSN_flow_model_3_base::add_objective_function);

        part.addFunction("n", &WSN_flow_model_3_base::add_lower_bound_constraints);
        part.addFunction("o", &WSN_flow_model_3_base::add_leaf_constraints);
        // // part.addFunction("", &WSN_flow_model_3_base::);

        std::vector<std::string> abc({"n", "o"});

        part.create_model(abc);
        part.solve(params.relaxed);
    }
    catch (IloException &e)
    {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (const string &e)
    {
        cerr << "Exception caught: " << e << endl;
    }
    catch (exception &e)
    {
        cerr << "Exception caught: " << e.what() << endl;
    }
    catch (...)
    {
        cerr << "Unknown exception caught." << endl;
    }

    return 0;
}