#pragma once

// #include <iostream>
#include <string>
#include <vector>
#include <set>
#include <sstream>
#include <fstream>

int numero(std::string num)
{
    std::stringstream ss(num);
    int retorno = 0;
    ss >> retorno;
    return retorno;
}

double numerodouble(std::string num)
{
    std::stringstream ss(num);
    double retorno = 0;
    ss >> retorno;
    return retorno;
}

/*Create a list of elements that have a arc from node v*/
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
                // adj_list[from].push_back(to);
                adj_list[from].insert(to);
            }
        }
    }

    return adj_list;
}

/*Create a list of elements that is connected to node v*/
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
                // adj_list[to].push_back(from);
                adj_list[to].insert(from);
            }
        }
    }

    return adj_list;
}

int read_instance(std::string instance_path,
                  std::vector<std::vector<double>> &c,
                  std::vector<std::vector<int>> &MA,
                  double &t)
{
    int n;
    int m;
    int entrada;
    int saida;
    std::string auxiliar;
    std::ifstream arquivo(instance_path);

    if (arquivo.is_open())
    {
        arquivo >> auxiliar;
        n = numero(auxiliar);

        arquivo >> auxiliar;
        m = numero(auxiliar);
        
        c = std::vector<std::vector<double>>(n, std::vector<double>(n, 1000));
        MA = std::vector<std::vector<int>>(n, std::vector<int>(n, 0));

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                c[i][j] = 1000;
                MA[i][j] = 0;
            }
        }

        for (int i = 0; i < m; i++)
        {
            arquivo >> auxiliar;
            entrada = numero(auxiliar) - 1;

            arquivo >> auxiliar;
            saida = numero(auxiliar) - 1;

            arquivo >> auxiliar;
            c[entrada][saida] = numerodouble(auxiliar);
            c[saida][entrada] = numerodouble(auxiliar);
            // cout<<"entrada "<<entrada+1<<" saida "<<saida+1<<" custo "<<numero(auxiliar)<<endl;
            MA[entrada][saida] = 1;
            MA[saida][entrada] = 1;
        }

        arquivo.close();
    }

    return n;
}

int read_instance(std::string instance_path,
                  std::vector<std::vector<double>> &c,
                  std::vector<std::vector<int>> &MA)
{
    double t;

    return read_instance(instance_path, c, MA, t);
}