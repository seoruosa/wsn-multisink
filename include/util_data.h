#pragma once

#include <vector>
#include "wsn_data.h"
#include <chrono>
#include <ilcplex/ilocplex.h>

#include "util_results.h"

std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, int size, int number_of_trees, std::vector<std::set<int>> &adj_list_from_v, IloCplex &cplex, int sum_to_index)
{
    std::vector<std::vector<int>> vec;
    for (int i = 0; i < size; i++)
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
                auto val = cplex.getValue(matrix[size + k][i]);
                if (val > 0.01)
                {
                    vec.push_back({size + k + sum_to_index, i + sum_to_index});
                }
            }
            catch (IloException &e)
            {
            }
        }
    }

    return vec;
}

std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, WSN_data &instance, IloCplex &cplex, int sum_to_index)
{
    return read_bin_matrix(matrix, instance.n, instance.number_trees, instance.adj_list_from_v, cplex, sum_to_index);
}

std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, int size, int number_of_trees, std::vector<std::set<int>> &adj_list_from_v, IloCplex &cplex, int sum_to_index)
{
    return read_bin_matrix(matrix, size, 0, adj_list_from_v, cplex, sum_to_index);
}

std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, WSN_data &instance, IloCplex &cplex, int sum_to_index)
{
    return read_bin_matrix(matrix, instance.n, 0, instance.adj_list_from_v, cplex, sum_to_index);
}

std::vector<int> read_bin_vec(IloNumVarArray &vec, int size, IloCplex &cplex, int sum_to_index)
{
    std::vector<int> vec_out;

    for (int i = 0; i < size; i++)
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

std::vector<std::vector<int>> read_bin_vec_to_matrix(IloNumVarArray &vec, int size, IloCplex &cplex, int sum_to_index)
{
    std::vector<std::vector<int>> vec_out;

    for (int i = 0; i < size; i++)
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

std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_vec_to_matrix(IloNumVarArray &vec, IloCplex &cplex, int sum_to_index)
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

// Read a matrix of variables and returns a pair of a vector with the pair of index and a list with values.
// Just have index on matrix or value, when value it's greater than zero.
std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_matrix(IloArray<IloNumVarArray> &matrix, IloCplex &cplex, int sum_to_index)
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

std::pair<std::vector<std::vector<int>>, std::vector<double>> read_matrix_3d(IloArray<IloArray<IloNumVarArray>> &matrix_3d,
                                                                             IloCplex &cplex, int sum_to_index)
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

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, int n, int number_of_trees,
                    std::vector<std::set<int>> &adj_list_from_v, int sum_to_index, std::ostream &cout)
{

    // auto read_int_3_idx_matrix = [](IloArray<IloArray<IloNumVarArray>> &matrix, int n_k, int n, auto adj_list_from_v, IloCplex &cplex, int sum_to_index = 0)
    // {
    //     std::vector<std::vector<double>> vec_out;

    //     for (int k = 0; k < n_k; k++)
    //     {
    //         for (int i = 0; i < n; i++)
    //         {
    //             for (auto &j : adj_list_from_v[i])
    //             {
    //                 auto val = cplex.getValue(matrix[k][i][j]);

    //                 if (val > 0.01)
    //                 {
    //                     vec_out.push_back({double(k + sum_to_index), double(i + sum_to_index), double(j + sum_to_index), val});
    //                 }
    //             }
    //         }
    //         for (int idx = 0; idx < n; idx++)
    //         {
    //             auto val = cplex.getValue(matrix[k][n + k][idx]);

    //             if (val > 0.01)
    //             {
    //                 vec_out.push_back({double(k + sum_to_index), double(n + k + sum_to_index), double(idx + sum_to_index), val});
    //             }
    //         }
    //     }

    //     return vec_out;
    // };

    auto matrix_x = read_bin_sol_matrix(x, n, number_of_trees, adj_list_from_v, cplex, 1);
    print_matrix(matrix_x, "X", cout);

    auto vec_y = read_bin_vec_to_matrix(y, n, cplex, 1);
    print_matrix(vec_y, "Y", cout);

    auto vec_z = read_bin_vec_to_matrix(z, n, cplex, 1);
    print_matrix(vec_z, "z", cout);

    // auto vec_f_depot = read_int_3_idx_matrix(f_depot, number_of_trees, n, adj_list_from_v, cplex, 1);
    // print_matrix(vec_f_depot, "F_SINK");

    cout << "EOF" << std::endl;
}

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, int n, int number_of_trees,
                    std::vector<std::set<int>> &adj_list_from_v, int sum_to_index)
{
    print_solution(cplex, x, y, z, n, number_of_trees, adj_list_from_v, sum_to_index, std::cout);
}

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, WSN_data &instance, int sum_to_index, std::ostream &cout)
{
    cout << instance << std::endl;

    print_solution(cplex, x, y, z, instance.n, instance.number_trees, instance.adj_list_from_v, sum_to_index, cout);
}

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, WSN_data &instance, int sum_to_index)
{
    print_solution(cplex, x, y, z, instance, sum_to_index, std::cout);
}