#include "flow_based.h"

#include <iostream>
#include "wsn_data.h"

#include "all_models.h"

ILOSTLBEGIN

/**
 * PartModel
 *
 * PartModel sabe que que objeto de entrada é WSN
 *
 */

template <typename T>
class PartModel
{
    typedef void (T::*memberFunction)();
    typedef std::map<std::string, memberFunction> MethodMap;

private:
    MethodMap funcMap;
    T *obj_ptr;
    WSN *model;

    std::vector<std::string> basic_functions;

    void operator()(std::string funcName)
    {
        typename MethodMap::const_iterator it = funcMap.find(funcName);

        if (it != funcMap.end())
        {
            (obj_ptr->*it->second)();
        }
    }

public:
    PartModel(T &model) : obj_ptr(&model), model(&model), basic_functions({}){};

    void addFunction(std::string funcName, memberFunction func)
    {
        funcMap[funcName] = func;
    }

    void addBasicFunction(std::string funcName, memberFunction func)
    {
        funcMap[funcName] = func;
        basic_functions.push_back(funcName);
    }

    void create_model(std::vector<std::string> &func_list)
    {
        for (auto &el : basic_functions)
        {
            operator()(el);
        }

        for (auto &el : func_list)
        {
            operator()(el);
        }
    }

    void solve(bool relaxed)
    {
        auto name = model->name_model_instance();

        model->model.add(model->constraints);

        if (relaxed)
        {
            model->solve_relaxed(name, print::time_now());
        }
        else
        {
            model->solve_mip(name, print::time_now());
        }
    }
};

int main(int argc, char *argv[])
{
    try
    {
        auto params = read_arguments(argc, argv);

        WSN_data instance(params.instance_path, params.number_sinks);

        std::cout << params.instance_path << " | K: " << params.number_sinks << "   " << std::endl;
        std::cout << "n:" << instance.n << std::endl;

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
        std::cout << instance << std::endl;

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

// int main(int argc, char *argv[])
// {
//     try
//     {
//         auto params = read_arguments(argc, argv);

//         WSN_data instance(params.instance_path, params.number_sinks);

//         std::cout << params.instance_path << " | K: " << params.number_sinks << "   " << std::endl;
//         std::cout << "n:" << instance.n << std::endl;

//         WSN_flow_model_3_base model(instance);

//         Part<WSN_flow_model_3_base> part(model);

//         part.addFunction("a", &WSN_flow_model_3_base::add_decision_variables);
//         part.addFunction("b", &WSN_flow_model_3_base::add_flow_model_variables);
//         part.addFunction("c", &WSN_flow_model_3_base::add_number_dominating_nodes_constraints);
//         part.addFunction("d", &WSN_flow_model_3_base::add_in_coming_edge_constraints);
//         part.addFunction("e", &WSN_flow_model_3_base::add_master_not_adj_master_constraints);
//         part.addFunction("f", &WSN_flow_model_3_base::add_node_master_or_bridge_constraints);
//         part.addFunction("g", &WSN_flow_model_3_base::add_bridges_not_neighbor_constraints);
//         part.addFunction("h", &WSN_flow_model_3_base::add_bridge_master_neighbor_constraints);
//         part.addFunction("i", &WSN_flow_model_3_base::add_master_neighbor_constraints);
//         part.addFunction("j", &WSN_flow_model_3_base::add_trivial_tree_constraints);
//         part.addFunction("k", &WSN_flow_model_3_base::add_flow_limit_constraints);
//         part.addFunction("l", &WSN_flow_model_3_base::add_flow_conservation_constraints);
//         part.addFunction("m", &WSN_flow_model_3_base::add_extra_node_constraints);
//         part.addFunction("n", &WSN_flow_model_3_base::add_lower_bound_constraints);
//         part.addFunction("o", &WSN_flow_model_3_base::add_leaf_constraints);
//         part.addFunction("p", &WSN_flow_model_3_base::add_objective_function);
//         // part.addFunction("", &WSN_flow_model_3_base::);
//         // part.addFunction("", &WSN_flow_model_3_base::);
//         // std::unique_ptr<ModelRunner<WSN>> model_runner = initialize_all_models(instance);

//         std::vector<std::string> abc({"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p"});
//         std::cout << instance << std::endl;
//         for (auto &el : abc)
//         {
//             part(el);
//         }

//         // if (params.relaxed)
//         // {
//         auto name = model.name_model_instance();

//         //     model.solve_mip(name, print::time_now());
//         // }
//         // part.solve(params.relaxed);
//         part.obj_ptr->solve_mip(name, print::time_now());

//         // model.solve(params.relaxed);
//         // (*model_runner).run_model(params.model, params.relaxed);
//     }
//     catch (IloException &e)
//     {
//         cerr << "Concert exception caught: " << e << endl;
//     }
//     catch (const string &e)
//     {
//         cerr << "Exception caught: " << e << endl;
//     }
//     catch (exception &e)
//     {
//         cerr << "Exception caught: " << e.what() << endl;
//     }
//     catch (...)
//     {
//         cerr << "Unknown exception caught." << endl;
//     }

//     return 0;
// }