#include "funcoes.h"

int minint(int a, int b)
{
    if (a < b)
    {
        return a;
    }
    return b;
}
int maxint(int a, int b)
{
    if (a > b)
    {
        return a;
    }
    return b;
}

double zeroeum()
{
    double a = rand() % 100000;
    a = a / 100000;
    return a;
}

int aleatorio(int min, int max)
{
    int k;
    double d;
    d = (double)rand() / ((double)RAND_MAX + 1);
    k = d * (max - min + 1);
    return min + k;
}

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

std::string nint(int k)
{
    std::string nint = "";
    std::stringstream resultado;
    resultado << k;
    nint = resultado.str();
    return nint;
}

std::string ndouble(double k)
{
    std::string nint = "";
    std::stringstream resultado;
    resultado << k;
    nint = resultado.str();
    return nint;
}

std::string nn(int k)
{
    std::string n = "";
    std::stringstream resultado;
    resultado << k;
    n = resultado.str();
    return n;
}

double min(double n, double m)
{
    if (n < m)
    {
        return n;
    }
    else
    {
        return m;
    }
}
void Floyd_Warshall(double **&c, int n)
{
    for (int k = 0; k < n; k++)
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                c[i][j] = min(c[i][j], c[i][k] + c[k][j]);
            }
        }
    }
    for (int i = 0; i < n; i++)
    {
        c[i][i] = 0;
    }
}
void Imprime_Matriz(int **c, int m, int n, std::string nome)
{
    std::cout << "--------------" << nome << "--------------------" << std::endl;
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            std::cout << std::setw(10);
            if (c[i][j] >= 900000)
            {
                std::cout << "-"
                     << " ";
            }
            else
            {
                std::cout << c[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }
}

int leitura(double **&c, int **&MA, double &t)
{
    int n;
    int m;
    int entrada;
    int saida;
    std::string auxiliar;
    std::ifstream arquivo(INSTANCE);
    // ifstream arquivo("grafo.txt");
    arquivo >> auxiliar;
    n = numero(auxiliar);
    arquivo >> auxiliar;
    m = numero(auxiliar);
    // arquivo>>auxiliar;
    // t=numerodouble(auxiliar);
    // getline(arquivo,auxiliar);
    c = new double *[n];
    MA = new int *[n];
    for (int i = 0; i < n; i++)
    {
        c[i] = new double[n];
        MA[i] = new int[n];
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
    return n;
}