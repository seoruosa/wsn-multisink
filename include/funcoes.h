#pragma once

#include <iostream>
#include <cstdint>
#include <vector>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <time.h>
#include <math.h>

#define MAXn 100

// #ifndef FUNCOES_H
// #define FUNCOES_H

double inicio;
double fim;
double tempototal;

std::string INSTANCE;
std::string SOLUCAOTOTAL;

int minint(int a, int b);

int maxint(int a, int b);

double zeroeum();

int aleatorio(int min, int max);

int numero(std::string num);

double numerodouble(std::string num);

std::string nint(int k);

std::string ndouble(double k);

std::string nn(int k);

double min(double n, double m);
void Floyd_Warshall(double **&c, int n);
void Imprime_Matriz(int **c, int m, int n, std::string nome);
int leitura(double **&c, int **&MA, double &t);

// #endif
