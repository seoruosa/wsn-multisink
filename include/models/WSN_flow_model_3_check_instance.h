#pragma once

#include "WSN.h"

/**
 * @brief Model that return the number of nodes that is a master or neighbor of master.
 * It is usefull to check if a instance is feasible
 * 
 */
class WSN_flow_model_3_check_instance : public WSN
{
public:
    WSN_flow_model_3_check_instance(WSN_data &instance);

private:
    virtual void build_model();

    IloArray<IloNumVarArray> f;
    IloNumVarArray t; // have a master as neighbor

    IloNumVar T;

    int M;

    virtual void add_objective_function();

    void add_flow_model_variables();

    void add_flow_limit_constraints();

    void add_flow_conservation_constraints();

    // constraints that uses the extra node
    void add_extra_node_constraints();

    void add_check_have_neighbors();

    // calculates an big-M
    double calculates_big_M();

    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
    virtual void set_params_cplex(IloCplex &cplex);
};

WSN_flow_model_3_check_instance::WSN_flow_model_3_check_instance(WSN_data &instance) : WSN(instance, "FlowModel3-checking"),
                                                                                       f(IloArray<IloNumVarArray>(env, instance.n + instance.number_trees)),
                                                                                       t(IloNumVarArray(env)),
                                                                                       T(IloNumVar(env, 0, IloInfinity, ILOFLOAT)),
                                                                                       M(int(calculates_big_M()))
{
}

void WSN_flow_model_3_check_instance::set_params_cplex(IloCplex &cplex)
{
    WSN::set_params_cplex(cplex);
}

void WSN_flow_model_3_check_instance::build_model()
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
    
    add_trivial_tree_constraints();

    add_flow_limit_constraints();        // exp 5, 7, 8
    add_flow_conservation_constraints(); // exp 6
    add_extra_node_constraints();        // exp 9, 10

    add_check_have_neighbors();

    add_objective_function();
}

void WSN_flow_model_3_check_instance::add_objective_function()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += t[i];
    }

    model.add(IloMaximize(env, expr));

    expr.end();
}

void WSN_flow_model_3_check_instance::add_flow_model_variables()
{
    // Naming variables
    T.setName("T");
    t = IloNumVarArray(env, instance.n, 0, 1, ILOINT); // have a master as neighbor

    for (int i = 0; i < instance.n + instance.number_trees; i++)
    {
        f[i] = IloNumVarArray(env, instance.n, 0, M, ILOFLOAT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            f[i][j].setName(("f(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }

    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        t[i].setName(("t(" + std::to_string(i) + ")").c_str());
    }
}

void WSN_flow_model_3_check_instance::add_flow_limit_constraints()
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

void WSN_flow_model_3_check_instance::add_flow_conservation_constraints()
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

void WSN_flow_model_3_check_instance::add_extra_node_constraints()
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

void WSN_flow_model_3_check_instance::add_check_have_neighbors()
{
    IloExpr expr(env);

    for (int v = 0; v < instance.n; v++)
    {
        for (auto &u : instance.adj_list_from_v[v])
        {
            expr += y[u];
        }

        constraints.add(expr <= int(instance.adj_list_from_v[v].size()) * t[v]);
        constraints.add(t[v] <= expr + y[v]);


        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

double WSN_flow_model_3_check_instance::calculates_big_M()
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

void WSN_flow_model_3_check_instance::print_full(IloCplex &cplex, std::ostream &cout)
{
    WSN::print_full(cplex, cout);

    auto [t_full, t_values] = read_full_vec_to_matrix(t, cplex, 1);
    print_matrix(t_full, t_values, "t", cout);
}