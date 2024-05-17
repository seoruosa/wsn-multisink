#pragma once

#include "util_data.h"
// #include "wsn_data.h"
#include <iostream>
#include "wsn_solution.h"

class WSN
{
public:
    WSN(WSN_data &instance, std::string formulation_name);
    // ~WSN();
    void solve(bool solve_relaxed = false);
    std::string name_model_instance();

protected:
    virtual void build_model() = 0;
    std::string formulation_name;
    WSN_data &instance;

    IloEnv env;
    IloConstraintArray constraints;

    IloModel model;

    IloNumVarArray z; // bridge
    IloNumVarArray y; // master

    IloArray<IloNumVarArray> x; // edges
    IloNumVar N;                // number of bridge and master nodes

    virtual void add_objective_function() = 0;
    void add_decision_variables();

    // Calculates the number of masters and bridges, the sum will be N
    void add_number_dominating_nodes_constraints();

    // The number of edges of dominating forest will be N - K
    void add_number_forest_edges_constraints();

    // Exist an incoming edge to a node, if this node is a master or bridge
    void add_in_coming_edge_constraints();

    // A node that is part of the dominating fores should be master or bridge
    void add_node_master_or_bridge_constraints();

    // Every node is a master or is neighbor of a master
    void add_master_neighbor_constraints();

    // Given two adjacent nodes, just one of then could be a master node
    void add_master_not_adj_master_constraints();

    // Given two adjacent nodes that are bridges, they cant be connected on the
    // final forest
    void add_bridges_not_neighbor_constraints();

    // If an arc is part of the dominating forest, one of their end is a master and
    // the other a bridge
    void add_bridge_master_neighbor_constraints();

    // a trivial tree should have just one master node
    void add_trivial_tree_constraints();

    // create decision variables and add basic model constraints
    void create_basic_model_constraints();

    // returns a copy of actual model with a relaxation of the integer variables
    virtual IloModel create_relaxed();

    virtual void create_start_solution(IloCplex &cplex);
    virtual void set_params_cplex(IloCplex &cplex);

    virtual void print_full(IloCplex &cplex, std::ostream &cout = std::cout);
};


WSN::WSN(WSN_data &instance, std::string formulation_name) : instance(instance),
                                                             formulation_name(formulation_name),
                                                             constraints(IloRangeArray(env)),
                                                             model(IloModel(env)),
                                                             z(IloNumVarArray(env)),
                                                             y(IloNumVarArray(env)),
                                                             x(IloArray<IloNumVarArray>(env)),
                                                             N(IloNumVar(env))
{
}

void WSN::solve(bool solve_relaxed)
{
    auto _name_model_instance = name_model_instance();
    std::string time_now = print::time_now();

    build_model();
    model.add(constraints);

    // double start, end, elapsed;
    auto start = perf::time::start();
    double gap;

    if (solve_relaxed)
    {
        // https://www.ibm.com/support/pages/solving-linear-relaxation-mip-concert
        std::ofstream relaxed_out((_name_model_instance + ".relaxed.out").c_str());
        std::ofstream relaxed_sol((_name_model_instance + ".relaxed.sol").c_str());

        relaxed_out << time_now << std::endl;
        relaxed_sol << time_now << std::endl;

        // SOLVE RELAXED
        auto relaxed = create_relaxed();
        IloCplex cplex_relax(relaxed);
        cplex_relax.setOut(relaxed_out);
        cplex_relax.setWarning(relaxed_out);
        cplex_relax.setError(relaxed_out);

        cplex_relax.exportModel((_name_model_instance + ".relaxed.lp").c_str());

        start = perf::time::start();
        cplex_relax.solve();

        gap = cplex_relax.getMIPRelativeGap();

        auto elapsed = perf::time::duration(start);

        auto out_info = [&elapsed, &cplex_relax, &gap](auto &out)
        {
            out << "time: " << elapsed.count() << std::endl;
            out << "obj: " << cplex_relax.getObjValue() << std::endl;
            out << "gap: " << (gap * 100) << " %" << std::endl;
        };

        out_info(relaxed_out);
        out_info(relaxed_sol);

        relaxed_sol << "***************************************" << std::endl;
        print_full(cplex_relax, relaxed_sol);

        cplex_relax.clear();
        cplex_relax.end();
    }
    else
    {
        std::ofstream cplex_out((_name_model_instance + ".log").c_str());
        std::ofstream cplex_warn_error((_name_model_instance + ".warn.log").c_str());
        std::ofstream solution((_name_model_instance + ".sol").c_str());
        std::ofstream cout((_name_model_instance + ".out").c_str());

        cout << time_now << std::endl;
        solution << time_now << std::endl;

        // SOLVE ORIGINAL MODEL
        IloCplex cplex(model);

        cplex.setOut(cplex_out);
        cplex.setWarning(cplex_warn_error);
        cplex.setError(cplex_warn_error);

        cplex.exportModel((_name_model_instance + ".lp").c_str());

        start = perf::time::start();
        set_params_cplex(cplex);

        create_start_solution(cplex);

        cplex.solve();

        gap = cplex.getMIPRelativeGap();

        auto elapsed = perf::time::duration(start).count();

        cout << "time: " << elapsed << std::endl;
        cout << "obj: " << cplex.getObjValue() << std::endl;
        cout << "best_obj: " << cplex.getBestObjValue() << std::endl;
        cout << "gap: " << (gap * 100) << " %" << std::endl;
        cout << "status: " << cplex.getStatus() << std::endl;

        print_solution(cplex, x, y, z, instance, 1, solution);

        WSN_solution solution_checker(instance);

        auto matrix_x = read_bin_sol_matrix(x, instance, cplex, 0);
        auto vec_y = read_bin_vec(y, instance.n, cplex, 0);
        auto vec_z = read_bin_vec(z, instance.n, cplex, 0);

        auto solution_valid = solution_checker.is_valid(matrix_x, vec_y, vec_z);

        cout << "Solution is" << (solution_valid ? "" : " not") << " valid" << std::endl;

        cout << "***************************************" << std::endl;
        print_full(cplex, cout);

        cplex.clear();
        cplex.end();
    }
}

void WSN::add_decision_variables()
{
    z = IloNumVarArray(env, instance.n, 0, 1, ILOINT); // bridge
    y = IloNumVarArray(env, instance.n, 0, 1, ILOINT); // master

    x = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);

    N = IloNumVar(env, 0, IloInfinity, ILOFLOAT);

    // Naming variables
    for (int i = 0; i < instance.n; i++)
    {
        z[i].setName(("z(" + std::to_string(i) + ")").c_str());
        y[i].setName(("y(" + std::to_string(i) + ")").c_str());
    }

    N.setName("N");

    for (int i = 0; i < instance.n + instance.number_trees; i++)
    {
        x[i] = IloNumVarArray(env, instance.n, 0, 1, ILOINT);

        // Naming variables
        for (int j = 0; j < instance.n; j++)
        {
            x[i][j].setName(("x(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
        }
    }
}

void WSN::add_number_dominating_nodes_constraints()
{
    // Constraints 2
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += (y[i] + z[i]);
    }

    constraints.add(expr == N);

    expr.end();
}

void WSN::add_number_forest_edges_constraints()
{
    // Constraints 3
    IloExpr expr(env);

    for (int from = 0; from < instance.n; from++)
    {
        for (auto &to : instance.adj_list_from_v[from])
        {
            expr += x[from][to];
        }
    }

    constraints.add(expr == N - instance.number_trees);

    expr.end();
}

void WSN::add_in_coming_edge_constraints()
{
    // Constraints 4
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += x[from][i];
        }
        for (int k = 0; k < instance.number_trees; k++)
        {
            expr += x[instance.n + k][i];
        }

        constraints.add(expr == (y[i] + z[i]));

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN::add_node_master_or_bridge_constraints()
{
    // Constraints 5
    for (int i = 0; i < instance.n; i++)
    {
        constraints.add((y[i] + z[i]) <= 1);
    }
}

void WSN::add_master_neighbor_constraints()
{
    // Constraints 6
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        expr += y[i];
        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += y[from];
        }
        constraints.add(expr >= 1);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN::add_master_not_adj_master_constraints()
{
    // Constraints 7
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            if (from > i)
            {
                constraints.add((y[from] + y[i]) <= 1); // Adasme2023 eq.20
            }
        }
    }
}

void WSN::add_bridges_not_neighbor_constraints()
{
    // Constraints 8
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            if (from > i)
            {
                constraints.add((z[from] + z[i] + x[from][i] + x[i][from]) <= 2);
            }
        }
    }
}

void WSN::add_bridge_master_neighbor_constraints()
{
    // Constraints 9
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            constraints.add(2 * x[from][i] <= (z[from] + z[i] + y[from] + y[i]));
        }
    }
}

void WSN::add_trivial_tree_constraints()
{
    IloExpr expr(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            expr += x[i][to];
        }

        for (auto &from : instance.adj_list_to_v[i])
        {
            expr += x[from][i];
        }

        constraints.add(2 * z[i] <= expr);

        expr.end();
        expr = IloExpr(env);
    }

    expr.end();
}

void WSN::create_basic_model_constraints()
{
    add_decision_variables();

    add_number_dominating_nodes_constraints();
    add_number_forest_edges_constraints();
    add_in_coming_edge_constraints();
    add_node_master_or_bridge_constraints();
    add_master_neighbor_constraints();
    add_master_not_adj_master_constraints();
    add_bridges_not_neighbor_constraints();
    add_bridge_master_neighbor_constraints();

    add_trivial_tree_constraints();
}

std::string WSN::name_model_instance()
{
    return std::string(formulation_name + "_" + instance.name() + "_" + std::to_string(instance.n) + "_" + std::to_string(instance.number_trees));
}

IloModel WSN::create_relaxed()
{
    IloModel relaxed(env);

    relaxed.add(model);

    relaxed = conversion::relax_2_index(relaxed, x);
    relaxed.add(IloConversion(env, y, ILOFLOAT));
    relaxed.add(IloConversion(env, z, ILOFLOAT));

    return relaxed;
}

void WSN::create_start_solution(IloCplex &cplex)
{
}

void WSN::set_params_cplex(IloCplex &cplex)
{
    cplex.setParam(IloCplex::Param::TimeLimit, 600);
    cplex.setParam(IloCplex::Param::Conflict::Display, 2);
    // cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10000);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 1e-6);
}

void WSN::print_full(IloCplex &cplex, std::ostream &cout)
{
    auto [x_full, x_values] = read_full_matrix(x, cplex, 1);
    auto [y_full, y_values] = read_full_vec_to_matrix(y, cplex, 1);
    auto [z_full, z_values] = read_full_vec_to_matrix(z, cplex, 1);

    print_matrix(x_full, x_values, "X", cout);
    print_matrix(y_full, y_values, "y", cout);
    print_matrix(z_full, z_values, "z", cout);
}