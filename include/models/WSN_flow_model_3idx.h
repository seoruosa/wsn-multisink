#pragma once

#include "WSN.h"
#include <limits>

#include "wsn_constructive_heur.h"

class WSN_flow_model_3idx : public WSN
{
public:
    WSN_flow_model_3idx(WSN_data &instance);
    WSN_flow_model_3idx(WSN_data &instance, double upper_bound);

private:
    virtual void build_model();
    IloArray<IloArray<IloNumVarArray>> f_depot; // flow formulation with 3 index
    IloArray<IloArray<IloNumVarArray>> z_depot; // arc-depot assignment

    virtual void add_objective_function();

    void add_flow_model_variables();

    void add_flow_3idx_elimination_constraints();
    void add_arc_depot_assignments_constraints();

    void add_adasme2023_valid_inequalities();
    void add_CastroAndrade2023_valid_inequalities();
    void add_lower_bound_constraints();

    virtual IloModel create_relaxed();
    void create_start_solution(IloCplex &cplex);
};

WSN_flow_model_3idx::WSN_flow_model_3idx(WSN_data &instance) : WSN(instance, "FlowModel3Index"),
                                                               f_depot(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees)),
                                                               z_depot(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees))
{
}

WSN_flow_model_3idx::WSN_flow_model_3idx(WSN_data &instance,
                                         double upper_bound) : WSN(instance, "FlowModel3Index", upper_bound),
                                                               f_depot(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees)),
                                                               z_depot(IloArray<IloArray<IloNumVarArray>>(env, instance.number_trees))
{
}

inline void WSN_flow_model_3idx::build_model()
{
    create_basic_model_constraints();

    add_flow_model_variables();

    add_flow_3idx_elimination_constraints();
    add_arc_depot_assignments_constraints();

    add_adasme2023_valid_inequalities();
    add_CastroAndrade2023_valid_inequalities();
    add_lower_bound_constraints();

    add_objective_function();
}

inline void WSN_flow_model_3idx::add_objective_function()
{

    model.add(IloMinimize(env, T));

    // constraint used to min-max the tree weight (not yet defined, now it's the forest weight)
    IloExpr expr(env);

    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (auto &from : instance.adj_list_to_v[i])
            {
                expr += instance.weight[from][i] * z_depot[k][from][i];
            }
        }

        model.add(T >= expr);
        expr.end();

        expr = IloExpr(env);
    }
    expr.end();
}

inline void WSN_flow_model_3idx::add_flow_model_variables()
{
    T.setName("T");

    // Creating arrays
    for (int k = 0; k < instance.number_trees; k++)
    {
        f_depot[k] = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);
        z_depot[k] = IloArray<IloNumVarArray>(env, instance.n + instance.number_trees);

        for (int i = 0; i < instance.n + instance.number_trees; i++)
        {
            z_depot[k][i] = IloNumVarArray(env, instance.n + instance.number_trees, 0, 1, ILOINT);
            f_depot[k][i] = IloNumVarArray(env, instance.n + instance.number_trees, 0, instance.n - instance.number_trees, ILOFLOAT);

            // Naming variables
            for (int j = 0; j < instance.n; j++)
            {
                z_depot[k][i][j].setName(("z_depot(" + std::to_string(k) + ")(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
                f_depot[k][i][j].setName(("f_depot(" + std::to_string(k) + ")(" + std::to_string(i) + ")(" + std::to_string(j) + ")").c_str());
            }
        }
    }
}

inline void WSN_flow_model_3idx::add_flow_3idx_elimination_constraints()
{
    // Imposes that N flow units leave from all sinks
    IloExpr exp2_5(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            exp2_5 += f_depot[k][instance.n + k][i];
        }
    }
    constraints.add(exp2_5 == N);

    exp2_5.end();

    // Flow conservation constraints
    IloExpr exp2_6(env);
    for (int sink = 0; sink < instance.number_trees; sink++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            for (int k = instance.n; k < instance.n + instance.number_trees; k++)
            {
                exp2_6 += f_depot[sink][k][i];
            }
            for (auto &from : instance.adj_list_to_v[i])
            {
                exp2_6 += f_depot[sink][from][i];
            }

            for (auto &to : instance.adj_list_from_v[i])
            {
                exp2_6 -= f_depot[sink][i][to];
            }

            constraints.add(exp2_6 == (y[i] + z[i]));

            exp2_6.end();
            exp2_6 = IloExpr(env);
        }
    }

    exp2_6.end();

    // Constraints that force the flow to null, if an arc doesn't belong to the forest
    IloExpr exp_f(env);
    for (int i = 0; i < instance.n; i++)
    {
        // Arcs that are "internal"
        for (auto &to : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                exp_f += f_depot[k][i][to];
            }
            constraints.add(x[i][to] <= exp_f);

            constraints.add(exp_f <= (instance.n - instance.number_trees - 1) * x[i][to]);

            exp_f.end();
            exp_f = IloExpr(env);
        }

        // Arcs from sinks
        for (int k = 0; k < instance.number_trees; k++)
        {
            constraints.add(x[instance.n + k][i] <= f_depot[k][instance.n + k][i]); // arc from node r

            constraints.add(f_depot[k][instance.n + k][i] <= (instance.n - instance.number_trees - 1) * x[instance.n + k][i]); // arc from node r
        }
    }
    exp_f.end();

    // Exactly one arc leave each sink
    IloExpr exp2_9(env);
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            exp2_9 += x[instance.n + k][i];
        }

        constraints.add(exp2_9 == 1);

        exp2_9.end();
        exp2_9 = IloExpr(env);
    }
    exp2_9.end();
}

inline void WSN_flow_model_3idx::add_arc_depot_assignments_constraints()
{
    for (int k = 0; k < instance.number_trees; k++)
    {
        for (int i = 0; i < instance.n; i++)
        {
            constraints.add(z_depot[k][instance.n + k][i] == x[instance.n + k][i]);

            for (auto &j : instance.adj_list_from_v[i])
            {
                constraints.add((instance.n - instance.number_trees - 1) * z_depot[k][i][j] >= f_depot[k][i][j]);
                // constraints.add(f_depot[k][i][j] >= z_depot[k][i][j]);
            }
        }
    }
}

inline void WSN_flow_model_3idx::add_adasme2023_valid_inequalities()
{
    // constraints Adasme2023
    for (int i = 0; i < instance.n; i++)
    {
        if (instance.adj_list_to_v[i].size() == 1)
        {
            // constraint 19
            constraints.add(z[i] == 0);

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

    constraints.add(exp_ad_28 >= 1);

    exp_ad_28.end();

    IloExpr exp_ad_47(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &from : instance.adj_list_to_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                exp_ad_47 += f_depot[k][from][i];
            }
        }

        // Constraints 47
        constraints.add(2 * z[i] <= exp_ad_47);

        exp_ad_47.end();
        exp_ad_47 = IloExpr(env);
    }
    exp_ad_47.end();

    IloExpr exp_ad_48(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                exp_ad_48 += f_depot[k][i][j];
            }

            // Constraints 48
            constraints.add(2 * x[i][j] - z[i] <= exp_ad_48);

            exp_ad_48.end();
            exp_ad_48 = IloExpr(env);
        }
    }
    exp_ad_48.end();

    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (int k = 0; k < instance.number_trees; k++)
    //     {
    //         // Constraints 50
    //         constraints.add(x[instance.n + k][i] <= y[i]);
    //     }
    // }

    IloExpr exp_ad_49(env);
    IloExpr exp_ad_49_1(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            for (auto &from : instance.adj_list_to_v[i])
            {
                exp_ad_49 += f_depot[k][from][i];
            }
            exp_ad_49 += f_depot[k][instance.n + k][i];
        }

        for (auto &j : instance.adj_list_from_v[i])
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                exp_ad_49_1 += f_depot[k][i][j];
            }
            // Constraints 49
            constraints.add(exp_ad_49_1 + x[i][j] * (instance.n - 1) - exp_ad_49 <= instance.n - 2);

            exp_ad_49_1.end();
            exp_ad_49_1 = IloExpr(env);
        }

        exp_ad_49.end();
        exp_ad_49 = IloExpr(env);
    }
    exp_ad_49.end();
    exp_ad_49_1.end();

    IloExpr exp_ad_50(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (int k = 0; k < instance.number_trees; k++)
        {
            exp_ad_50 += x[instance.n + k][i];
        }

        // Constraints 50
        constraints.add(exp_ad_50 <= y[i]);

        exp_ad_50.end();
        exp_ad_50 = IloExpr(env);
    }

    exp_ad_50.end();

    // // Constraints 51

    // IloExpr exp_ad_51(env);
    // IloExpr exp_ad_51_1(env);
    // for (int i = 0; i < instance.n; i++)
    // {
    //     for (auto &j : instance.adj_list_from_v[i])
    //     {
    //         if (i > j)
    //         {
    //             for (int k = 0; k < instance.number_trees; k++)
    //             {
    //                 exp_ad_51 += f_depot[k][i][j];
    //                 exp_ad_51_1 += x[instance.n + k][i];

    //                 // constraints.add(f_depot[k][i][j] <= instance.number_trees - 1 - 2 * (y[i] - x[instance.n + k][i]) - z[i]);
    //             }

    //             for (int k = 0; k < instance.number_trees; k++)
    //             {
    //                 // exp_ad_51 += f_depot[k][i][j];
    //                 // exp_ad_51_1 += x[instance.n + k][i];

    //                 // constraints.add(f_depot[k][i][j] <= instance.number_trees - 1 - 2 * (y[i] - exp_ad_51_1) - z[i]);
    //                 constraints.add(exp_ad_51 <= instance.number_trees - 1 - 2 * (y[i] - x[instance.n + k][i]) - z[i]);
    //             }
    //             // constraints.add(exp_ad_51 <= instance.number_trees - 1 - 2 * (y[i] - exp_ad_51_1) - z[i]);

    //             exp_ad_51.end();
    //             exp_ad_51 = IloExpr(env);
    //             exp_ad_51_1.end();
    //             exp_ad_51_1 = IloExpr(env);
    //         }
    //     }
    // }

    // exp_ad_51.end();
}

inline void WSN_flow_model_3idx::add_CastroAndrade2023_valid_inequalities()
{
    // Constraints from CastroAndrade2023
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &j : instance.adj_list_from_v[i])
        {
            // constraint 15
            constraints.add(x[i][j] + x[j][i] <= (y[j] + z[j]));

            // constraint 16 and 17
            constraints.add(x[i][j] <= z[i] + z[j]);
            constraints.add(x[i][j] <= y[i] + y[j]);
        }
    }
}

void WSN_flow_model_3idx::add_lower_bound_constraints()
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

    expr = IloExpr(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            expr += instance.weight[i][to] * x[i][to];
        }
    }
    constraints.add(T >= expr / instance.n);
    expr.end();

    auto minimum_weight = [&]()
    {
        double min = std::numeric_limits<double>::max();

        for (int i = 0; i < instance.n; i++)
        {
            for (auto &to : instance.adj_list_from_v[i])
            {
                if (instance.weight[i][to] < min)
                {
                    min = instance.weight[i][to];
                }
            }
        }
        return min;
    };

    auto avg_weight = [&]()
    {
        int count = 0;
        double avg = 0;

        for (int i = 0; i < instance.n; i++)
        {
            for (auto &to : instance.adj_list_from_v[i])
            {
                avg = (avg * count + instance.weight[i][to]) / (count + 1);
                ++count;
            }
        }

        return avg;
    };

    auto min_weight = minimum_weight();
    IloExpr expr_2(env);
    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_to_v[i]) // from ou to_v???
        {
            for (int k = 0; k < instance.number_trees; k++)
            {
                expr_2 += f_depot[k][i][to];
            }
        }

        constraints.add(T >= min_weight * expr_2);

        expr_2.end();
        expr_2 = IloExpr(env);
    }

    expr_2.end();
}

inline IloModel WSN_flow_model_3idx::create_relaxed()
{
    IloModel relaxed(WSN::create_relaxed());

    relaxed = relax_utils::relax_3_index(relaxed, f_depot);
    relaxed = relax_utils::relax_3_index(relaxed, z_depot);

    return relaxed;
}

void WSN_flow_model_3idx::create_start_solution(IloCplex &cplex)
{
    auto heur = WSNConstructiveHeuristic(instance);

    auto sol = heur.solve();
    std::cout << "heuristic weight: " << heur.weight_of_solution() << std::endl;

    IloNumVarArray startVar(env);
    IloNumArray startVal(env);

    for (int i = 0; i < instance.n; i++)
    {
        for (auto &to : instance.adj_list_from_v[i])
        {
            startVar.add(x[i][to]);
            startVal.add(0);
        }
    }

    for (auto &e : sol.edges)
    {
        startVar.add(x[e[0]][e[1]]);
        startVal.add(1);

        // std::cout << ">> " << e[0] << " -> " << e[1] << std::endl;
    }

    auto add_binary = [&](auto vec, IloNumVarArray &var)
    {
        for (int i = 0; i < vec.size(); i++)
        {
            startVar.add(var[i]);
            startVal.add(vec[i]);
        }
    };

    // add_binary(sol.bridges, z);
    // add_binary(sol.masters, y);

    for (int i = 0; i < instance.n; i++)
    {
        startVar.add(z[i]);
        startVal.add(sol.bridges[i]);
        startVar.add(y[i]);
        startVal.add(sol.masters[i]);
    }

    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
}