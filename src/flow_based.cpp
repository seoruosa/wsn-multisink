#include "flow_based.h"

#include <iostream>
#include <getopt.h>
#include "wsn_data.h"
// #include <ilcplex/ilocplex.h>

// #include "WSN_flow_model_3idx.h"
// #include "WSN_mtz_model.h"
// #include "WSN_mtz_model_2.h"
// #include "WSN_flow_model_0.h"
// #include "WSN_flow_model_1.h"
// #include "WSN_flow_model_2.h"
// #include "WSN_flow_model_0_1.h"
// #include "WSN_mtz_model_2_1.h"

// #include "WSN_mtz_castroandrade2023.h"
// #include "WSN_mtz_castroandrade2023-impSBPO.h"
#include "WSN_mtz_castroandrade2023-impSBPO-corrigido.h"
// #include "WSN_mtz_castroandrade2023-bektas2014.h"
// #include "WSN_mtz_castroandrade2023-new_constr.h"

// #include "WSN_mcf_model.h"
// #include "WSN_mcf_model_mcf_valid.h"
// #include "WSN_mcf_model_base.h"
// #include "WSN_mcf_model_adasme2023.h"
// #include "WSN_mcf_model_castro2023.h"

// #include "WSN_mcf_model_weight_on_node.h"
// #include "WSN_mcf_model_weight_on_node_benders.h"

// #include "WSN_flow_model_2_1.h"
// #include "WSN_flow_model_2_1_base.h"
// #include "WSN_flow_model_2_1_sbc.h"
// #include "WSN_flow_model_3_base.h"
// #include "WSN_flow_model_3_des.h"

// #include "WSN_flow_model_3_check_instance.h"
// #include "WSN_arvore_rotulada_model_base.h"

// #include "WSN_mcf_weight_arc.h"
// #include "WSN_repr_model_base.h"


// #include "flow_based_model2.h"

ILOSTLBEGIN

int main(int argc, char *argv[])
{
    try
    {
        auto params = read_arguments(argc, argv);

        std::string instance_path = params.instance_path;

        int K = params.number_sinks;
        bool relaxed = params.relaxed;

        WSN_data instance(instance_path, K);

        std::cout << instance_path << " | K: " << K << "   " << std::endl;
        std::cout << "n:" << instance.n << std::endl;


        // if (params.model == "check-instance")
        // {
        //     WSN_flow_model_3_check_instance(instance).solve(relaxed);
        // }
        // if (params.model == "FlowModel3idx")
        // {
        //     WSN_flow_model_3idx(instance).solve(relaxed);
        // }
        // else if (params.model == "MTZ")
        // {
        //     WSN_mtz_model(instance).solve(relaxed);
        // }
        // else if (params.model == "MTZ2")
        // {
        //     WSN_mtz_model_2(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel0")
        // {
        //     // IloEnv env;
        //     // create_model(env, instance.weight, instance.is_connected, instance.n, instance.number_trees);
        //     WSN_flow_model_0(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel1")
        // {
        //     WSN_flow_model_1(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel2")
        // {
        //     WSN_flow_model_2(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel0-1")
        // {
        //     WSN_flow_model_0_1(instance).solve(relaxed);
        // }
        // else if (params.model == "MTZ2-1")
        // {
        //     WSN_mtz_model_2_1(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel2-1-base")
        // {
        //     WSN_flow_model_2_1_base(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel2-1")
        // {
        //     WSN_flow_model_2_1(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel2-1-sbc")
        // {
        //     WSN_flow_model_2_1_sbc(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel3-base")
        // {
        //     WSN_flow_model_3_base(instance).solve(relaxed);
        // }
        // else if (params.model == "FlowModel3-des")
        // {
        //     WSN_flow_model_3_des(instance).solve(relaxed);
        // }
        // // ####################### MTZ Models #######################
        // else if (params.model == "MTZ-castro2023")
        // {
        //     WSN_mtz_castro_andrade_2023(instance).solve(relaxed);
        // }
        // else if (params.model == "MTZ-sbpo")
        // {
        //     WSN_mtz_castro_andrade_2023_imp_sbpo(instance).solve(relaxed);
        // }
        // else 
        if (params.model == "MTZ-sbpo-corr")
        {
            WSN_mtz_castro_andrade_2023_imp_sbpo_corrigido(instance).solve(relaxed);
        }
        // else if (params.model == "MTZ-castro2023-bektas")
        // {
        //     WSN_mtz_castro_andrade_2023_bektas2014(instance).solve(relaxed);
        // }
        // else if (params.model == "MTZ-castro2023-new-constr")
        // {
        //     WSN_mtz_castro_andrade_2023_new_constraints(instance).solve(relaxed);
        // }
        // // ####################### MCF Models #######################
        // else if (params.model == "MCFModel")
        // {
        //     WSN_mcf_model(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-mcf-valid")
        // {
        //     WSN_mcf_model_mcf_valid(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-castro2023")
        // {
        //     WSN_mcf_model_castro2023(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-adasme2023")
        // {
        //     WSN_mcf_model_adasme2023(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-base")
        // {
        //     WSN_mcf_model_base(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-weight-node")
        // {
        //     WSN_mcf_model_weight_on_node(instance).solve(relaxed);
        // }
        // else if (params.model == "MCFModel-weight-node-benders")
        // {
        //     WSN_mcf_model_weight_on_node_benders(instance).solve(relaxed);
        // }
        // else if (params.model == "MAR-base")
        // {
        //     WSN_arvore_rotulada_model_base(instance).solve(relaxed);
        // }
        // else if (params.model == "MCF-weight-arc-Model")
        // {
        //     WSN_mcf_weight_arc_model(instance).solve(relaxed);
        // }
        // else if (params.model == "REPR-base")
        // {
        //     WSN_representante_model_base(instance).solve(relaxed);
        // }
        // else if (params.model == "")
        // {
        //     (instance).solve(relaxed);
        // }
        
        
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

void PrintHelp()
{
    auto print_list = [](auto &list, std::string sep = " | ")
    {
        for (int i = 0; i < list.size(); i++)
        {
            std::cout << list[i];
            if (i < list.size() - 1)
            {
                std::cout << sep;
            }
        }
    };

    std::cout << "-i, --instance <path>:     Path for instance\n"
                 "-r, --relaxed:              Solve the relaxed version of model\n"
                 "-K, --num-sinks <n>:       Number of sinks\n"
                 "-m, --model [";
    print_list(valid_models, " | ");
    std::cout << "]:       choosen model\n"
                 "-h, --help:                Show help\n";
    exit(1);
}

Run_Params read_arguments(int argc, char **argv)
{
    const char *const short_opts = "K:ri:m:s:h";
    const option long_opts[] = {
        {"instance", required_argument, nullptr, 'i'},
        {"num-sinks", optional_argument, nullptr, 'K'},
        {"relaxed", no_argument, nullptr, 'r'},
        {"model", optional_argument, nullptr, 'm'},
        {"seed", optional_argument, nullptr, 's'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, no_argument, nullptr, 0}};

    std::string instance_path;
    std::string model;
    int number_sinks = 1;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    bool relaxed = false;

    while (true)
    {
        const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);
        bool model_is_valid = false;

        if (-1 == opt)
            break;

        switch (opt)
        {
        case 'K':
            number_sinks = (optarg == NULL) ? number_sinks : std::stoi(optarg);
            break;

        case 'i':
            instance_path = std::string(optarg);
            break;

        case 'r':
            relaxed = true;
            break;

        case 'm':
            model = std::string(optarg);

            for (auto &el : valid_models)
            {
                if (model == el)
                {
                    model_is_valid = true;
                }
            }

            if (!model_is_valid)
            {
                PrintHelp();
            }

            break;
        case 's':
            seed = (optarg == NULL) ? seed : std::stoul(optarg);
            break;
        case 'h': // -h or --help
        case '?': // Unrecognized option
        default:
            PrintHelp();
            break;
        }
    }

    return {instance_path, model, number_sinks, seed, relaxed};
}