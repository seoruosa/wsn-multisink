#pragma once

#include "WSN.h"

class WSN_flow_model_3_des : public WSN
{
    // Model where the flow it's related to the weight of tree
    // Testing valid constraints
public:
    WSN_flow_model_3_des(WSN_data &instance);

private:
    virtual void build_model();

    IloArray<IloNumVarArray> f;

    IloNumVar T;

    int M;

    virtual void add_objective_function();

    void add_flow_model_variables();

    void add_flow_limit_constraints();

    void add_flow_conservation_constraints();

    // constraints that uses the extra node
    void add_extra_node_constraints();

    void add_lower_bound_constraints();

    void add_leaf_constraints();

    void add_CastroAndrade2023_valid_inequalities();
    void add_adasme2023_valid_inequalities();
    void add_testing_valid_inequalities();

    void add_remove_symmetries();

    // calculates an big-M
    double calculates_big_M();

    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_flow_model_3_des::WSN_flow_model_3_des(WSN_data &instance) : WSN(instance, "FlowModel3-des-1-2-4"),
                                                                 f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                 T(IloNumVar(env, 0, IloInfinity, ILOFLOAT)),
                                                                 M(int(calculates_big_M()))
{
}

void WSN_flow_model_3_des::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
    // cplex.setParam(IloCplex::Param::Benders::Strategy, 3);
}

void WSN_flow_model_3_des::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp

    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14
    add_master_neighbor_constraints();        // exp 21
    add_trivial_tree_constraints();

    add_flow_limit_constraints();        // exp 5, 7, 8
    add_flow_conservation_constraints(); // exp 6
    add_extra_node_constraints();        // exp 9, 10
    add_lower_bound_constraints();       // exp 20
    add_leaf_constraints();              // exp 22

    // valid inequalities
    add_adasme2023_valid_inequalities();
    add_CastroAndrade2023_valid_inequalities();
    add_remove_symmetries();
    add_testing_valid_inequalities();

    add_objective_function();
}

void WSN_flow_model_3_des::add_objective_function()
{
    model.add(IloMinimize(env, T));

    IloExpr expr(env);

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += f[instance.n + k][i];
        }

        model.add(T >= expr);
        expr.end();

        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_3_des::add_flow_model_variables()
{
    // Naming variables
    T.setName("T");

    for (int i = 0; i < instance.n + instance.number_trees; i++)
    {
        f[i] = IloNumVarArray(env, instance.n, 0, M, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN_flow_model_3_des::add_flow_limit_constraints()
{

    IloExpr expr(env);

    // exp 7
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][to] >= instance.weight[i][to] * x[i][to]);
        }
    }

    // exp 8
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            constraints.add(f[i][to] <= M * x[i][to]);
        }

        // arc from node r
        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(f[instance.n + k][i] <= M * x[instance.n + k][i]);
        }
    }

    expr.end();
}

void WSN_flow_model_3_des::add_flow_conservation_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            expr += f[instance.n + k][i];
        }

        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += f[from][i];
        }

        for (auto &to : instance.adj_list_from_v[i])
        {
            expr -= f[i][to];
        }

        for (auto &from : instance.adj_list_to_v[i])
        {
            expr -= instance.weight[from][i] * x[from][i];
        }

        constraints.add(expr == 0);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_3_des::add_extra_node_constraints()
{
    // exp 9
    IloExpr expr(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            expr += x[instance.n + k][i];
        }

        constraints.add(expr == 1);
        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN_flow_model_3_des::add_lower_bound_constraints()
{
    // exp 20
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            expr += instance.weight[i][to] * x[i][to];
        }
        constraints.add(T >= expr);

        expr.end();
        expr = IloExpr(env);
    }
    expr.end();
}

void WSN_flow_model_3_des::add_leaf_constraints()
{
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint doc 17
            constraints.add(z[i] == 0);
        }
    }
}

void WSN_flow_model_3_des::add_CastroAndrade2023_valid_inequalities()
{
    IloExpr expr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            expr += (x[i][j] + x[j][i]);

            constraints.add((x[i][j] + x[j][i]) <= y[i] + z[i]); // constraints 15

            // constraint 16 and 17
            constraints.add(x[i][j] <= z[i] + z[j]);
            constraints.add(x[i][j] <= y[i] + y[j]);
        }

        constraints.add(2 * z[i] <= expr); // constraints 12

        expr.end();
        expr = IloExpr(env);
    }

    for (int u = 0; u < instance.n; u++)
    {
        std::set<int> neighbors(instance.adj_list_from_v[u]);
        neighbors.insert(u); // neighbors = N[u]

        expr -= (y[u] + z[u]);
        for (auto &v : instance.adj_list_from_v[u])
        {
            expr += x[u][v];
            expr -= (y[v] + z[v]);

            for (auto &l : instance.adj_list_from_v[v])
            {
                if (neighbors.find(l) != neighbors.end())
                {
                    expr += x[v][l];
                }
            }
        }

        constraints.add(expr <= -1); // constraints 10

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

inline void WSN_flow_model_3_des::add_adasme2023_valid_inequalities()
{
    // constraints Adasme2023
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint 19
            // constraints.add(z[i] == 0);

            for (auto &from : instance.adj_list_to_v[i])
            {
                // constraint 20
                constraints.add(y[from] + y[i] == 1);

                // constraint 21
                // constraints.add(2 * z[from] <= y[i] + x[from][i]);
                constraints.add(2 * (x[from][i] + x[i][from]) <= y[i] + z[from]);
            }
        }
    }

    IloExpr exp_ad_28(env);

    for (int i = 0; i < instance.n; i++)
    {
        exp_ad_28 += (y[i] - z[i]);
    }

    constraints.add(exp_ad_28 >= instance.number_trees);
    exp_ad_28.end();

    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            // Constraints 50
            constraints.add(x[instance.n + k][i] <= y[i]);
        }
    }
}

void WSN_flow_model_3_des::add_testing_valid_inequalities()
{
    IloExpr expr(env);

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &u : instance.adj_list_from_v[v])
        {
            expr += y[u] + x[v][u] + x[u][v];
        }

        constraints.add(4 * z[v] <= expr); // des. val. 1

        expr.end();
        expr = IloExpr(env);
    }

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {

            constraints.add((x[i][j] + x[j][i]) <= z[i] + z[j]); // des. val. 2
        }
    }

    expr = IloExpr(env);

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &u : instance.adj_list_from_v[v])
        {
            expr += instance.weight[v][u]*(x[v][u] - z[v]);
        }
        
        for (int k = 0; k < instance.number_trees; k++)
        {
            // constraints.add(f[instance.n + k][v] >= expr); // des. val. 3 // falso
            constraints.add(f[instance.n + k][v] <= M*y[v]); // des. val. 4
        }
        
    }
    


    expr.end();
}

void WSN_flow_model_3_des::add_remove_symmetries()
{
    // adapted from work of Robertty
    IloExpr expr(env);

    // test_5
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int v = 0; v < instance.n; v++)
        {
            for (int s = instance.n; s < instance.n + k; s++)
            {
                for (int u = v + 1; u < instance.n; u++)
                {
                    expr += x[s][u];
                }

                for (int j = k; j < instance.number_trees; j++)
                {
                    expr -= x[instance.n + j][v];
                }

                constraints.add(expr >= 0); // test_5

                expr.end();
                expr = IloExpr(env);
            }
        }
    }

    expr.end();
}

double WSN_flow_model_3_des::calculates_big_M()
{
    double M_weight = 1.0;
    std::vector<double> weights;

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            weights.push_back(instance.weight[i][j]);
        }
    }

    std::sort(weights.begin(), weights.end(), std::greater<double>());

    for (int i = 0; i < (instance.n - instance.number_trees) && i < weights.size(); i++)
    {
        M_weight += weights[i];
    }

    return M_weight;
}

void WSN_flow_model_3_des::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto read_matrix_3d = [](IloArray<IloArray<IloNumVarArray>> &matrix_3d, IloCplex &cplex, int sum_to_index = 0)
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
    };
}