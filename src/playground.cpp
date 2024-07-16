#include <iostream>

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <stdexcept>

#include "models/WSN_flow_model_2_1_base.h"

class Base
{
public:
    virtual ~Base() {}
    virtual void f() = 0;
    virtual void g() { std::cout << "Run method g (base)" << std::endl; };
};

/********************************************************************************************/
/********************************************************************************************/

class Derived : public Base
{
private:
    /* data */
public:
    Derived(/* args */);
    ~Derived(){};
    void f() override { std::cout << "Run method f (derived)" << std::endl; };
    void g() override { std::cout << "Run method g (derived)" << std::endl; };
};

Derived::Derived(/* args */)
{
    std::cout << "Constructor Derived" << std::endl;
}

/********************************************************************************************/
/********************************************************************************************/

class Derived2 : public Base
{
private:
    /* data */
public:
    Derived2(/* args */);
    ~Derived2(){};
    void f() override { std::cout << "Run method f (derived2)" << std::endl; };
    // void g() override { std::cout << "Run method g (derived2)" << std::endl; };
    void h() { std::cout << "Run method h (derived2)" << std::endl; };
    void v() { std::cout << "Run method v (derived2)" << std::endl; };
};

Derived2::Derived2(/* args */)
{
    std::cout << "Constructor Derived2" << std::endl;
}

/********************************************************************************************/
/********************************************************************************************/

template <typename T>
class Part
{
    typedef void (T::*memberFunction)();
    typedef std::map<std::string, memberFunction> MethodMap;

private:
    MethodMap funcMap;
    T *obj_ptr;

public:
    Part(T &obj_ptr) : obj_ptr(&obj_ptr){};

    void addFunction(std::string funcName, memberFunction func)
    {
        funcMap[funcName] = func;
    }

    void operator()(std::string funcName)
    {
        typename MethodMap::const_iterator it = funcMap.find(funcName);

        if (it != funcMap.end())
        {
            (obj_ptr->*it->second)();
        }
    }
};

/********************************************************************************************/
/********************************************************************************************/

class MyModel : public WSN_flow_model_2_1_base
{
private:
    /* data */
public:
    MyModel(WSN_data &instance);
    // ~MyModel();
    virtual void build_model();
};

MyModel::MyModel(WSN_data &instance) : WSN_flow_model_2_1_base(instance)
{
    WSN::formulation_name = "my-model";
}

void MyModel::build_model()
{
    add_decision_variables();
    add_flow_model_variables();

    add_number_dominating_nodes_constraints(); // exp 3
    add_number_forest_edges_constraints();     // exp 4
    add_in_coming_edge_constraints();          // exp 
    add_flow_limit_constraints();              // exp 5, 7, 8
    add_flow_conservation_constraints();       // exp 6

    add_extra_node_constraints();             // exp 9, 10
    add_master_not_adj_master_constraints();  // exp 11
    add_node_master_or_bridge_constraints();  // exp 12
    add_bridges_not_neighbor_constraints();   // exp 13
    add_bridge_master_neighbor_constraints(); // exp 14

    add_lower_bound_constraints();     // exp 20
    add_master_neighbor_constraints(); // exp 21
    add_leaf_constraints();            // exp 22

    add_trivial_tree_constraints();

    add_CastroAndrade2023_valid_inequalities();
    // add_bektas2020_node_current_constraints();
    add_arc_depot_assignment_constraints();
    add_adasme2023_valid_inequalities();

    add_objective_function();
}

/********************************************************************************************/
/********************************************************************************************/

int main()
{
    auto line = [](auto s="~")
    {
        for (int i = 0; i < 60; i++)
        {
            std::cout << s;
        }

        std::cout << std::endl;
    };

    auto der = Derived();
    auto der2 = Derived2();


    line("-");

    std::cout << ">>> ADD FUNCTIONS <<<" << std::endl;

    Part<Derived2> part(der2);
    part.addFunction("f", &Derived2::f);
    part.addFunction("g", &Derived2::g);
    part.addFunction("h", &Derived2::h);

    line("-");

    std::cout << ">>> RUN FUNCTIONS <<<" << std::endl;

    std::vector<std::string> abc({"f", "g", "h"});

    for (auto &el : abc)
    {
        part(el);
    }

    line("#");
}