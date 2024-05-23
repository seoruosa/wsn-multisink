#pragma once

#include <ilcplex/ilocplex.h>

/**
 * @brief Util methods for relaxing problems
 * 
 */
namespace relax_utils
{
    /**
     * @brief Convert the matrix of variables to float
     * 
     * @param model is the model
     * @param variables_matrix is the matrix of variables to be converted
     * @return IloModel the relaxed model
     */
    IloModel relax_2_index(IloModel &model, IloArray<IloNumVarArray> &variables_matrix)
    {
        auto env = model.getEnv();

        for (int i = 0; i < variables_matrix.getSize(); i++)
        {
            try
            {
                // relax integer variables
                model.add(IloConversion(env, variables_matrix[i], ILOFLOAT));
            }
            catch (IloException &e)
            {
            }
        }

        return model;
    }

    /**
     * @brief Convert the variables_matrix of variables with 3 index to float
     * 
     * @param model is the model
     * @param matrix_3d is the variables_matrix of variables (3 index)
     * @return IloModel the relaxed model
     */
    IloModel relax_3_index(IloModel &model, IloArray<IloArray<IloNumVarArray>> &matrix_3d)
    {
        auto env = model.getEnv();

        for (int k = 0; k < matrix_3d.getSize(); k++)
        {
            for (int i = 0; i < matrix_3d[k].getSize(); i++)
            {
                try
                {
                    model.add(IloConversion(env, matrix_3d[k][i], ILOFLOAT));
                }
                catch (IloException &e)
                {
                }
            }
        }

        return model;
    }

} // namespace relax_utils