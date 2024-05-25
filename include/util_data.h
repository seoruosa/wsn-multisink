#pragma once

#include <vector>
#include "wsn_data.h"
#include <chrono>
#include <ilcplex/ilocplex.h>

#include "util_results.h"

/**
 * @brief Read the values of boolean cplex variable that is represented as a matrix.
 * Read values of adjacency list and from sinks (nodes + k, k \in (0, ..., number_trees - 1)).
 * 
 * @param matrix is a boolean cplex variable represented as a matrix
 * @param nodes is the number of nodes associated with the variable
 * @param number_of_trees is the number of sinks
 * @param adj_list_from_v is the list of adjacency
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<std::vector<int>> is a matrix of arcs with value on solution greater than 0.01
 */
std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, 
                                              int nodes, 
                                              int number_of_trees, 
                                              std::vector<std::set<int>> &adj_list_from_v, 
                                              IloCplex &cplex, 
                                              int sum_to_index)
{
    std::vector<std::vector<int>> vec;
    for (int i = 0; i < nodes; i++)
    {
        for (auto &j : adj_list_from_v[i])
        {
            try
            {
                auto val = cplex.getValue(matrix[i][j]);
                if (val > 0.01)
                {
                    vec.push_back({i + sum_to_index, j + sum_to_index});
                }
            }
            catch (IloException &e)
            {
            }
        }

        for (int k = 0; k < number_of_trees; k++)
        {
            try
            {
                auto val = cplex.getValue(matrix[nodes + k][i]);
                if (val > 0.01)
                {
                    vec.push_back({nodes + k + sum_to_index, i + sum_to_index});
                }
            }
            catch (IloException &e)
            {
            }
        }
    }

    return vec;
}

/**
 * @brief Read the values of boolean cplex variable that is represented as a matrix.
 * Read values of adjacency list and from sinks (nodes + k, k \in (0, ..., number_trees - 1)).
 * 
 * @param matrix is a boolean cplex variable represented as a matrix
 * @param instance is the instance object (number of nodes, number of trees and adjacency list is used)
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<std::vector<int>> is a matrix of arcs with value on solution greater than 0.01
 */
std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, 
                                              WSN_data &instance, 
                                              IloCplex &cplex, 
                                              int sum_to_index)
{
    return read_bin_matrix(matrix, instance.n, instance.number_trees, instance.adj_list_from_v, cplex, sum_to_index);
}

/**
 * @brief Read the values of boolean cplex variable that is represented as a matrix.
 * Read values of adjacency list.
 * 
 * @param matrix is a boolean cplex variable represented as a matrix
 * @param nodes is the number of nodes associated with the variable
 * @param number_of_trees is the number of sinks
 * @param adj_list_from_v is the list of adjacency
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<std::vector<int>> is a matrix of arcs with value on solution greater than 0.01
 */
std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, 
                                                  int nodes, 
                                                  int number_of_trees, 
                                                  std::vector<std::set<int>> &adj_list_from_v, 
                                                  IloCplex &cplex, 
                                                  int sum_to_index)
{
    return read_bin_matrix(matrix, nodes, 0, adj_list_from_v, cplex, sum_to_index);
}

/**
 * @brief Read the values of boolean cplex variable that is represented as a matrix.
 * Read values of adjacency list.
 * 
 * @param matrix is a boolean cplex variable represented as a matrix
 * @param instance is the instance object (number of nodes, number of trees and adjacency list is used)
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<std::vector<int>> is a matrix of arcs with value on solution greater than 0.01
 */
std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, 
                                                  WSN_data &instance, 
                                                  IloCplex &cplex, 
                                                  int sum_to_index)
{
    return read_bin_matrix(matrix, instance.n, 0, instance.adj_list_from_v, cplex, sum_to_index);
}

/**
 * @brief Read the values of boolean cplex variable that is represented as a vector.
 * 
 * @param vec is a boolean cplex variable represented as a vector
 * @param nodes is the nodes of vector
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<int> is a vector of arcs with value on solution greater than 0.01
 */
std::vector<int> read_bin_vec(IloNumVarArray &vec, int nodes, IloCplex &cplex, int sum_to_index)
{
    std::vector<int> vec_out;

    for (int i = 0; i < nodes; i++)
    {
        try
        {
            auto val = cplex.getValue(vec[i]);

            if (val > 0.01)
            {
                vec_out.push_back(i + sum_to_index);
            }
        }
        catch (IloException &e)
        {
        }
    }

    return vec_out;
};

/**
 * @brief Read the values of boolean cplex variable that is represented as a vector.
 * 
 * @param vec is a boolean cplex variable represented as a vector
 * @param nodes is the nodes of vector
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::vector<std::vector<int>> is a vector of arcs with value on solution greater than 0.01
 */
std::vector<std::vector<int>> read_bin_vec_to_matrix(IloNumVarArray &vec, int nodes, IloCplex &cplex, int sum_to_index)
{
    std::vector<std::vector<int>> vec_out;

    for (int i = 0; i < nodes; i++)
    {
        try
        {
            auto val = cplex.getValue(vec[i]);

            if (val > 0.1)
            {
                vec_out.push_back({i + sum_to_index});
            }
        }
        catch (IloException &e)
        {
        }
    }

    return vec_out;
};

/**
 * @brief Read the values of cplex variable that is represented as a vector.
 * 
 * @param vec is a cplex variable represented as a vector
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::pair<std::vector<std::vector<int>>, std::vector<double>> is a pair of a matrix of index 
 * and a list of respective values, for solution values greater than zero
 */
std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_vec_to_matrix(IloNumVarArray &vec, 
                                                                                      IloCplex &cplex, 
                                                                                      int sum_to_index)
{
    std::vector<std::vector<int>> vec_out;
    std::vector<double> values;

    for (int i = 0; i < vec.getSize(); i++)
    {
        try
        {
            auto val = cplex.getValue(vec[i]);

            if (val > 0)
            {
                vec_out.push_back({i + sum_to_index});
                values.push_back(val);
            }
        }
        catch (IloException &e)
        {
        }
    }

    return {vec_out, values};
};

/**
 * @brief Read the values of cplex variable that is represented as a matrix.
 * 
 * @param matrix is a cplex variable represented as a matrix
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::pair<std::vector<std::vector<int>>, std::vector<double>> is a pair of a matrix of arcs 
 * and a list of respective values, for solution values greater than zero
 */
std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_matrix(IloArray<IloNumVarArray> &matrix, 
                                                                               IloCplex &cplex, 
                                                                               int sum_to_index)
{
    std::vector<std::vector<int>> vec;
    std::vector<double> values;

    for (int i = 0; i < matrix.getSize(); i++)
    {
        for (int j = 0; j < matrix[i].getSize(); j++)
        {
            try
            {
                auto val = cplex.getValue(matrix[i][j]);
                if (val > 0)
                {
                    vec.push_back({i + sum_to_index, j + sum_to_index});
                    values.push_back(val);
                }
            }
            catch (IloException &e)
            {
            }
        }
    }

    return {vec, values};
}

/**
 * @brief Read the values of cplex variable that is represented as a matrix.
 * 
 * @param matrix_3d is a cplex variable represented as a matrix with 3 index
 * @param cplex is the cplex object
 * @param sum_to_index is a constant to be summed to the index of variable
 * @return std::pair<std::vector<std::vector<int>>, std::vector<double>>  is a pair of a matrix of 3 index 
 * and a list of respective values, for solution values greater than zero
 */
std::pair<std::vector<std::vector<int>>, std::vector<double>> read_matrix_3d(IloArray<IloArray<IloNumVarArray>> &matrix_3d,
                                                                             IloCplex &cplex, 
                                                                             int sum_to_index)
{
    std::vector<std::vector<int>> vec;
    std::vector<double> values;

    for (int k = 0; k < matrix_3d.getSize(); k++)
    {
        for (int i = 0; i < matrix_3d[k].getSize(); i++)
        {
            for (int j = 0; j < matrix_3d[k][i].getSize(); j++)
            {
                try
                {
                    auto val = cplex.getValue(matrix_3d[k][i][j]);
                    if (val > 0)
                    {
                        vec.push_back({k + sum_to_index, i + sum_to_index, j + sum_to_index});
                        values.push_back(val);
                    }
                }
                catch (IloException &e)
                {
                }
            }
        }
    }

    return std::pair<std::vector<std::vector<int>>, std::vector<double>>({vec, values});
}

/**
 * @brief Print a solution values
 * 
 * @param cplex is the cplex object
 * @param x is the cplex variable of arcs of solution
 * @param y is the cplex variable represent a node is a master or not
 * @param z is the cplex variable represent a node is a bridge or not
 * @param nodes is the number of nodes
 * @param number_of_trees is the number of sinks
 * @param adj_list_from_v is the list of adjacency
 * @param sum_to_index is a constant to be summed to the index of variable
 * @param cout is stream output
 */
void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, 
                                              IloNumVarArray &y, 
                                              IloNumVarArray &z, 
                                              int nodes, 
                                              int number_of_trees,
                                              std::vector<std::set<int>> &adj_list_from_v, 
                                              int sum_to_index, 
                                              std::ostream &cout)
{
    auto matrix_x = read_bin_sol_matrix(x, nodes, number_of_trees, adj_list_from_v, cplex, 1);
    print_matrix(matrix_x, "X", cout);

    auto vec_y = read_bin_vec_to_matrix(y, nodes, cplex, 1);
    print_matrix(vec_y, "Y", cout);

    auto vec_z = read_bin_vec_to_matrix(z, nodes, cplex, 1);
    print_matrix(vec_z, "z", cout);

    cout << "EOF" << std::endl;
}

/**
 * @brief Print a solution values on the standard output
 * 
 * @param cplex is the cplex object
 * @param x is the cplex variable of arcs of solution
 * @param y is the cplex variable represent a node is a master or not
 * @param z is the cplex variable represent a node is a bridge or not
 * @param nodes is the number of nodes
 * @param number_of_trees is the number of sinks
 * @param adj_list_from_v is the list of adjacency
 * @param sum_to_index is a constant to be summed to the index of variable
 */
void print_solution(IloCplex &cplex, 
                    IloArray<IloNumVarArray> &x, 
                    IloNumVarArray &y, 
                    IloNumVarArray &z, 
                    int nodes, 
                    int number_of_trees,
                    std::vector<std::set<int>> &adj_list_from_v, 
                    int sum_to_index)
{
    print_solution(cplex, x, y, z, nodes, number_of_trees, adj_list_from_v, sum_to_index, std::cout);
}

/**
  * @brief Print a solution values
 * 
 * @param cplex is the cplex object
 * @param x is the cplex variable of arcs of solution
 * @param y is the cplex variable represent a node is a master or not
 * @param z is the cplex variable represent a node is a bridge or not
 * @param instance is the instance object (number of nodes, number of trees and adjacency list is used)
 * @param sum_to_index is a constant to be summed to the index of variable
 * @param cout is stream output
 */
void print_solution(IloCplex &cplex, 
                    IloArray<IloNumVarArray> &x, 
                    IloNumVarArray &y, 
                    IloNumVarArray &z, 
                    WSN_data &instance, 
                    int sum_to_index, 
                    std::ostream &cout)
{
    cout << instance << std::endl;

    print_solution(cplex, x, y, z, instance.n, instance.number_trees, instance.adj_list_from_v, sum_to_index, cout);
}

/**
 * @brief Print a solution values on the standard output
 * 
 * @param cplex is the cplex object
 * @param x is the cplex variable of arcs of solution
 * @param y is the cplex variable represent a node is a master or not
 * @param z is the cplex variable represent a node is a bridge or not
 * @param instance is the instance object (number of nodes, number of trees and adjacency list is used)
 * @param sum_to_index is a constant to be summed to the index of variable
 */
void print_solution(IloCplex &cplex, 
                    IloArray<IloNumVarArray> &x, 
                    IloNumVarArray &y, 
                    IloNumVarArray &z, 
                    WSN_data &instance, 
                    int sum_to_index)
{
    print_solution(cplex, x, y, z, instance, sum_to_index, std::cout);
}