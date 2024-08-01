#pragma once

#include <string>
#include <vector>

#include <getopt.h>
#include <iostream>
#include <chrono>

#include <regex>

/**
 * @brief Structure with necessary parameters from input
 *
 */
struct Run_Params
{
    std::string instance_path;
    std::string model = "FlowModel3idx";
    int number_sinks = 1;
    unsigned seed;
    bool relaxed = false;
    std::vector<std::string> constraints = {};
};

/**
 * @brief Print a help message
 *
 */
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
                 "-m, --model [model_name]:       choosen model\n"
                 "-c, --constraints [constr_list]:      list of constraints\n"
                 "-h, --help:                Show help\n";
    exit(1);
}

/**
 * @brief Parse the arguments from program call
 *
 * @param argc
 * @param argv
 * @return Run_Params is the structure with parsed arguments
 */
Run_Params read_arguments(int argc, char **argv)
{
    const char *const short_opts = "K:ri:m:s:c:h";
    const option long_opts[] = {
        {"instance", required_argument, nullptr, 'i'},
        {"num-sinks", optional_argument, nullptr, 'K'},
        {"relaxed", no_argument, nullptr, 'r'},
        {"model", optional_argument, nullptr, 'm'},
        {"seed", optional_argument, nullptr, 's'},
        {"help", no_argument, nullptr, 'h'},
        {"constraints", optional_argument, nullptr, 'c'},
        {nullptr, no_argument, nullptr, 0}};

    std::string instance_path;
    std::string model;
    int number_sinks = 1;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    bool relaxed = false;
    std::vector<std::string> constraints({});

    while (true)
    {
        const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);
        bool model_is_valid = false;

        auto read_constraints = [](auto optarg)
        {
            std::regex rgx("[-:\\w]+");
            std::string string(optarg);

            auto i = std::sregex_iterator(string.begin(), string.end(), rgx);
            auto end = std::sregex_iterator();

            std::vector<std::string> constraints_list({});

            while (i != end)
            {
                auto match = *i++;

                constraints_list.push_back(match.str());                
            }

            return constraints_list;
        };

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

            break;
        case 's':
            seed = (optarg == NULL) ? seed : std::stoul(optarg);
            break;
        case 'c':
            constraints = read_constraints(optarg);
            break;
        case 'h': // -h or --help
        case '?': // Unrecognized option
        default:
            PrintHelp();
            break;
        }
    }

    return {instance_path, model, number_sinks, seed, relaxed, constraints};
}