#pragma once

#include <string>
#include <vector>
#include <set>
#include <sstream>
#include <fstream>

/**
 * @brief Transform a string with a integer number written in a int number
 * 
 * @param num is a string input with a number
 * @return int  
 */
int numero(std::string num)
{
    std::stringstream ss(num);
    int retorno = 0;
    ss >> retorno;
    return retorno;
}

/**
 * @brief Transform a string with a double number written in a double number
 * 
 * @param num is a string input with a number
 * @return double 
 */
double numerodouble(std::string num)
{
    std::stringstream ss(num);
    double retorno = 0;
    ss >> retorno;
    return retorno;
}

/**
 * @brief Transform a matrix of adjacency in a list of adjacency. 
 * Each position is associated with a node i. Each position contains 
 * a set of nodes j, such that arc {i, j} exists.
 * 
 * @param adj_matrix is the matrix of adjacency
 * @return std::vector<std::set<int>> list of adjacency
 */
std::vector<std::set<int>> to_adj_list_out(std::vector<std::vector<int>> &adj_matrix)
{
    int n = adj_matrix.size();

    std::vector<std::set<int>> adj_list(n, std::set<int>());

    for (int from = 0; from < n; from++)
    {
        for (int to = 0; to < n; to++)
        {
            if (adj_matrix[from][to] == 1)
            {
                adj_list[from].insert(to);
            }
        }
    }

    return adj_list;
}

/**
 * @brief Transform a matrix of adjacency in a list of adjacency. 
 * Each position is associated with a node i. Each position contains 
 * a set of nodes j, such that arc {j, i} exists.
 * 
 * @param adj_matrix is the matrix of adjacency
 * @return std::vector<std::set<int>> list of adjacency
 */
std::vector<std::set<int>> to_adj_list_in(std::vector<std::vector<int>> &adj_matrix)
{
    int n = adj_matrix.size();

    std::vector<std::set<int>> adj_list(n, std::set<int>());

    for (int from = 0; from < n; from++)
    {
        for (int to = 0; to < n; to++)
        {
            if (adj_matrix[from][to] == 1)
            {
                adj_list[to].insert(from);
            }
        }
    }

    return adj_list;
}

/**
 * @brief Data from a instance of a graph
 * 
 */
struct Instance
{
    int number_nodes;
    int number_edges;
    std::vector<std::vector<double>> weight_matrix;
    std::vector<std::vector<int>> adjacency_matrix;
};


/**
 * @brief Read a instance of WSN problem, as defined in https://github.com/seoruosa/instances/tree/main/MSCWSN
 * 
 * @param instance_path is a path to the instance
 * @param weight_matrix pointer to a matrix of double where the weight matrix will be stored
 * @param adjacency_matrix pointer to a matrix of int where the matrix of adjacency will be stored
 * @return int the number of nodes of instance
 */
int read_instance_wsn(std::string instance_path,
                  std::vector<std::vector<double>> &weight_matrix,
                  std::vector<std::vector<int>> &adjacency_matrix)
{
    int number_nodes;
    int number_edges;
    int entrada;
    int saida;
    std::string auxiliar;
    std::ifstream arquivo(instance_path);

    if (arquivo.is_open())
    {
        arquivo >> auxiliar;
        number_nodes = numero(auxiliar);

        arquivo >> auxiliar;
        number_edges = numero(auxiliar);
        
        weight_matrix = std::vector<std::vector<double>>(number_nodes, std::vector<double>(number_nodes, 1000));
        adjacency_matrix = std::vector<std::vector<int>>(number_nodes, std::vector<int>(number_nodes, 0));

        for (int i = 0; i < number_edges; i++)
        {
            arquivo >> auxiliar;
            entrada = numero(auxiliar) - 1;

            arquivo >> auxiliar;
            saida = numero(auxiliar) - 1;

            arquivo >> auxiliar;
            weight_matrix[entrada][saida] = numerodouble(auxiliar);
            weight_matrix[saida][entrada] = numerodouble(auxiliar);
            
            adjacency_matrix[entrada][saida] = 1;
            adjacency_matrix[saida][entrada] = 1;
        }

        arquivo.close();
    }

    return number_nodes;
}