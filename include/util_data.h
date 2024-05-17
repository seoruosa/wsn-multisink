#pragma once

#include <vector>
#include <fstream>
#include "wsn_data.h"
#include <chrono>
#include <ilcplex/ilocplex.h>

#include <regex>

int get_number(std::string a);

std::pair<int, int> get_pair(std::string a);

std::vector<std::vector<int>> get_int_matrix(std::ifstream &file, int lines, int columns, int skip_columns);

template <typename T>
std::vector<std::vector<T>> get_float_matrix(std::ifstream &file, int lines, int columns, int skip_columns);

template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, std::vector<T2> &values, std::string var_name, std::ostream &cout);

template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, std::vector<T2> &values, std::string var_name);

template <typename T>
void print_matrix(std::vector<std::vector<T>> &vec, std::string var_name, std::ostream &cout);

template <typename T>
void print_matrix(std::vector<std::vector<T>> &vec, std::string var_name);

std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, int size, int number_of_trees, std::vector<std::set<int>> &adj_list_from_v, IloCplex &cplex, int sum_to_index = 0);

std::vector<std::vector<int>> read_bin_matrix(IloArray<IloNumVarArray> &matrix, WSN_data &instance, IloCplex &cplex, int sum_to_index = 0);

std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, int size, int number_of_trees, std::vector<std::set<int>> &adj_list_from_v, IloCplex &cplex, int sum_to_index = 0);

std::vector<std::vector<int>> read_bin_sol_matrix(IloArray<IloNumVarArray> &matrix, WSN_data &instance, IloCplex &cplex, int sum_to_index = 0);

std::vector<int> read_bin_vec(IloNumVarArray &vec, int size, IloCplex &cplex, int sum_to_index = 0);

std::vector<std::vector<int>> read_bin_vec_to_matrix(IloNumVarArray &vec, int size, IloCplex &cplex, int sum_to_index = 0);

std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_vec_to_matrix(IloNumVarArray &vec, IloCplex &cplex, int sum_to_index = 0);

// Read a matrix of variables and returns a pair of a vector with the pair of index and a list with values.
// Just have index on matrix or value, when value it's greater than zero.
std::pair<std::vector<std::vector<int>>, std::vector<double>> read_full_matrix(IloArray<IloNumVarArray> &matrix, IloCplex &cplex, int sum_to_index = 0);

std::pair<std::vector<std::vector<int>>, std::vector<double>> read_matrix_3d(IloArray<IloArray<IloNumVarArray>> &matrix_3d,
                                                                             IloCplex &cplex, int sum_to_index = 0);

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, int n, int number_of_trees,
                    std::vector<std::set<int>> &adj_list_from_v, int sum_to_index, std::ostream &cout);

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, int n, int number_of_trees,
                    std::vector<std::set<int>> &adj_list_from_v, int sum_to_index = 1);

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, WSN_data &instance, int sum_to_index, std::ostream &cout);

void print_solution(IloCplex &cplex, IloArray<IloNumVarArray> &x, IloNumVarArray &y, IloNumVarArray &z, WSN_data &instance, int sum_to_index = 1);

namespace print
{
    std::string time_now(const char *fmt);

    std::string time_now();

} // namespace print

namespace conversion
{
    IloModel relax_2_index(IloModel &model, IloArray<IloNumVarArray> &matrix);

    IloModel relax_3_index(IloModel &model, IloArray<IloArray<IloNumVarArray>> &matrix_3d);

} // namespace conversion


namespace perf
{
    namespace time
    {
        std::chrono::high_resolution_clock::time_point start();

        // template <typename Ratio=std::micro>
        std::chrono::duration<double> duration(std::chrono::high_resolution_clock::time_point start_);
    }
}

int get_number(std::string a)
{
    const std::regex number_reg("\\d+");
    std::smatch n_match;
    int output(INT16_MAX);

    if (std::regex_search(a, n_match, number_reg))
    {
        output = std::stoi(n_match[0].str());
    }

    return output;
}

std::pair<int, int> get_pair(std::string a)
{
    const std::regex pieces_regex("(\\d+)[\\s\\t]+(\\w+)");
    std::smatch pieces_match;

    int n = 0;
    int m = 0;

    if (std::regex_match(a, pieces_match, pieces_regex))
    {
        std::sub_match sub_match = pieces_match[1];
        n = std::stoi(pieces_match[1].str());
        m = std::stoi(pieces_match[2].str());
    }

    return {n, m};
}

std::vector<std::vector<int>> get_int_matrix(std::ifstream &file, int lines, int columns, int skip_columns)
{
    std::string string_line;
    std::regex number_reg("\\d+");
    auto node_coord = std::vector<std::vector<int>>(lines, std::vector<int>(columns, 0));

    for (int i = 0; i < lines; i++)
    {
        std::getline(file, string_line);

        auto words_begin = std::sregex_iterator(string_line.begin(), string_line.end(), number_reg);
        auto words_end = std::sregex_iterator();

        int j = 0;
        for (auto ind = words_begin; ind != words_end; ++ind)
            while ((j < (columns + skip_columns)) & (ind != words_end))
            {
                if (j >= skip_columns)
                {
                    auto number_match = *ind;
                    node_coord[i][j - skip_columns] = std::stoi(number_match.str());
                }
                j++;
                ++ind;
            }
    }

    return node_coord;
}

template <typename T>
std::vector<std::vector<T>> get_float_matrix(std::ifstream &file, int lines, int columns, int skip_columns)
{
    std::string string_line;
    std::regex number_reg("\\d+\\.{0,1}\\d*");
    auto node_coord = std::vector<std::vector<T>>(lines, std::vector<T>(columns, 0));

    for (int i = 0; i < lines; i++)
    {
        std::getline(file, string_line);

        auto words_begin = std::sregex_iterator(string_line.begin(), string_line.end(), number_reg);
        auto words_end = std::sregex_iterator();

        int j = 0;
        for (auto ind = words_begin; ind != words_end; ++ind)
            while ((j < (columns + skip_columns)) & (ind != words_end))
            {
                if (j >= skip_columns)
                {
                    auto number_match = *ind;
                    node_coord[i][j - skip_columns] = std::stof(number_match.str());
                }
                j++;
                ++ind;
            }
    }

    return node_coord;
}

template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, std::vector<T2> &values, std::string var_name, std::ostream &cout)
{
    cout << var_name << " : " << matrix.size() << " ";
    if (matrix.empty())
    {
        cout << matrix.size() << std::endl;
    }
    else
    {
        cout << matrix.front().size() << std::endl;
    }
    // assert matrix.size() == values.size();

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            if (j == matrix[i].size() - 1)
            {
                cout << matrix[i][j] << "\t: " << values[i] << std::endl;
            }
            else
            {
                cout << matrix[i][j] << "\t";
            }
        }
    }
    cout << std::endl;
}

template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, std::vector<T2> &values, std::string var_name)
{
    print_matrix(matrix, values, var_name, std::cout);
}

template <typename T>
void print_matrix(std::vector<std::vector<T>> &vec, std::string var_name, std::ostream &cout)
{
    cout << var_name << " : " << vec.size() << " ";
    if (vec.empty())
    {
        cout << vec.size() << std::endl;
    }
    else
    {
        cout << vec.front().size() << std::endl;
    }

    for (int i = 0; i < vec.size(); i++)
    {
        for (int j = 0; j < vec[i].size(); j++)
        {
            if (j == vec[i].size() - 1)
            {
                cout << vec[i][j] << std::endl;
            }
            else
            {
                cout << vec[i][j] << "\t";
            }
        }
    }
    cout << std::endl;
}

template <typename T>
void print_matrix(std::vector<std::vector<T>> &vec, std::string var_name)
{
    print_matrix(vec, var_name, std::cout);
}

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

namespace print
{
    std::string time_now(const char *fmt)
    {
        // https://www.programiz.com/cpp-programming/library-function/ctime/strftime
        auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        char date_string[100];

        std::strftime(date_string, 100, fmt, std::localtime(&time));
        std::string out(date_string);

        return out;
    };

    std::string time_now()
    {
        return time_now("%Y-%m-%d %H:%M:%S");
    }
} // namespace print

namespace conversion
{
    IloModel relax_2_index(IloModel &model, IloArray<IloNumVarArray> &matrix)
    {
        auto env = model.getEnv();

        for (int i = 0; i < matrix.getSize(); i++)
        {
            try
            {
                // relax integer variables
                model.add(IloConversion(env, matrix[i], ILOFLOAT));
            }
            catch (IloException &e)
            {
            }
        }

        return model;
    }

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

} // namespace conversion


namespace perf
{
    namespace time
    {
        std::chrono::high_resolution_clock::time_point start()
        {
            return std::chrono::high_resolution_clock::now();
        }

        // template <typename Ratio=std::micro>
        std::chrono::duration<double> duration(std::chrono::high_resolution_clock::time_point start_)
        {
            auto end_gen = start();
            std::chrono::duration<double> duration(end_gen - start_);

            return duration;
        }
    }
}