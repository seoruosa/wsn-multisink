#pragma once

#include <map>
#include <vector>
#include <string>
#include "models/WSN.h"
#include "wsn_data.h"

#include <stdexcept>
#include <sstream>


/**
 * @brief Class to make easy the testing of valid inequalities of a formulation
 *
 * @tparam T is the class that implements the formulation
 */
template <typename T>
class PartModel
{
    typedef void (T::*memberFunction)();
    typedef std::map<std::string, memberFunction> MethodMap;

private:
    MethodMap funcMap;
    T *obj_ptr;
    WSN *model;

    std::vector<std::string> basic_functions;

    /**
     * @brief Run the method related to funcName
     *
     * @param funcName is the name of the method that implements the constraint
     */
    void operator()(std::string funcName);

public:
    PartModel(T &model) : obj_ptr(&model), model(&model), basic_functions({}){};

    /**
     * @brief Add the constraints that belongs to the basic model
     *
     * @param funcName Name of the function
     * @param func refers to the method that implements the problems constraints
     */
    void addBasicFunction(std::string funcName, memberFunction func);

    /**
     * @brief Add the valid inequalities
     *
     * @param funcName Name of the function
     * @param func refers to the method that implements the problems constraints
     */
    void addFunction(std::string funcName, memberFunction func);

    /**
     * @brief Create the basic model with valid inequalities
     *
     * @param func_list is a list with valid inequalities that will be used on formulation
     */
    void create_model(std::vector<std::string> &func_list);

    /**
     * @brief Create the basic model
     *
     */
    void create_model();

    /**
     * @brief Solve the implemented model. The user need to create the model before solving them
     *
     * @param relaxed is true if the user want to solve the relaxed version of problem
     */
    void solve(bool relaxed = false);
};

template <typename T>
inline void PartModel<T>::operator()(std::string funcName)
{
    typename MethodMap::const_iterator it = funcMap.find(funcName);

    if (it != funcMap.end())
    {
        (obj_ptr->*it->second)();
    }
    else
    {
        std::ostringstream error_message;
        error_message << "Constraint name don't exist in the map";

        throw std::invalid_argument(error_message.str());
    }
}

template <typename T>
inline void PartModel<T>::addFunction(std::string funcName, memberFunction func)
{
    if (funcMap.find(funcName) == funcMap.end())
    {
        funcMap[funcName] = func;
    }
    else
    {
        std::ostringstream error_message;
        error_message << "Constraint name exist in the map";

        throw std::invalid_argument(error_message.str());
    }
}

template <typename T>
inline void PartModel<T>::addBasicFunction(std::string funcName, memberFunction func)
{
    addFunction(funcName, func);
    basic_functions.push_back(funcName);
}

template <typename T>
inline void PartModel<T>::create_model(std::vector<std::string> &func_list)
{
    for (auto &el : basic_functions)
    {
        operator()(el);
    }

    for (auto &el : func_list)
    {
        operator()(el);
    }
}

template <typename T>
inline void PartModel<T>::create_model()
{
    std::vector<std::string> empty({});

    create_model(empty);
}

template <typename T>
inline void PartModel<T>::solve(bool relaxed)
{
    auto name = model->name_model_instance();

    model->model.add(model->constraints);

    if (relaxed)
    {
        model->solve_relaxed(name, print::time_now());
    }
    else
    {
        model->solve_mip(name, print::time_now());
    }
}