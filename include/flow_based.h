#pragma once

#include <string>
#include <vector>
#include <chrono>

struct Run_Params
{
    std::string instance_path;
    std::string model = "FlowModel3idx";
    int number_sinks = 1;
    unsigned seed;
    bool relaxed = false;
};

const std::vector<std::string> valid_models({"check-instance",
                                             "MTZ", "MTZ2", "FlowModel3idx", "FlowModel0",
                                             "FlowModel1", "FlowModel2", "FlowModel0-1", "MTZ2-1", 
                                             "MCFModel", "MTZ-castro2023", "MTZ-sbpo",
                                             "MTZ-sbpo-corr", "MTZ-castro2023-bektas", "MTZ-castro2023-new-constr",
                                             "MCFModel-mcf-valid", "MCFModel-castro2023",
                                             "MCFModel-adasme2023", "MCFModel-base", "MCFModel-weight-node", 
                                             "MCFModel-weight-node-benders", "FlowModel2-1",
                                             "FlowModel2-1-base", "FlowModel2-1-sbc", "FlowModel3-base", "MAR-base",
                                             "FlowModel3-des", "MCF-weight-arc-Model", "REPR-base"});

void PrintHelp();

Run_Params read_arguments(int argc, char **argv);