#include "arguments_parser.h"

#include <iostream>
#include "wsn_data.h"

#include "all_models.h"

ILOSTLBEGIN

int main(int argc, char *argv[])
{
    try
    {
        auto params = read_arguments(argc, argv);

        WSN_data instance(params.instance_path, params.number_sinks);

        std::cout << params.instance_path << " | K: " << params.number_sinks << "   " << std::endl;
        std::cout << "n:" << instance.n << std::endl;

        std::unique_ptr<ModelRunner<WSN>> model_runner;

        if (params.upper_bound > 0)
        {
            model_runner = initialize_all_models(instance, params.upper_bound);
        }
        else
        {
            model_runner = initialize_all_models(instance);
        }

        (*model_runner).run_model(params.model, params.relaxed);
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