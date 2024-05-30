#pragma once

#include <string>
#include <map>
#include <memory>
#include <stdexcept>
#include <sstream>

/**
 * @brief Class used to register models and run then.
 *
 * @tparam Base it is the class that models inherits
 */
template <class Base>
class ModelRunner
{
private:
    std::map<std::string, std::unique_ptr<Base>> map_models;

    /**
     * @brief Check if a model it was registered
     *
     * @param model_name is the name that will be checked
     * @return true if model is registered
     * @return false otherwise
     */
    bool is_model_valid(std::string model_name) { return (map_models.find(model_name) != map_models.end()); };

    /**
     * @brief Return a list of the registered models
     *
     * @return std::vector<std::string> list of the registered models
     */
    std::vector<std::string> list_of_models();

    /**
     * @brief Return a string with all model names separated by commas
     *
     * @return std::string
     */
    std::string name_of_models();

public:
    ModelRunner() {}
    ~ModelRunner(){};

    /**
     * @brief Insert a model to runner
     *
     * @tparam T is the class of model to be registered. Should inherit from Base
     * @param new_model is the new model to be registered
     * @param name is the name that will be associated with the model
     */
    template <class T>
    void insert_model(T new_model, std::string name);

    /**
     * @brief Runs a model by name
     *
     * @param model_name is the name of model that you can run
     * @param relaxed defines if it will solve the full or relaxed model
     */
    void run_model(std::string model_name, bool relaxed);
};

template <class Base>
inline std::vector<std::string> ModelRunner<Base>::list_of_models()
{
    std::vector<std::string> list;

    for (auto const &el : map_models)
    {
        list.push_back(el.first);
    }

    return list;
}

template <class Base>
inline std::string ModelRunner<Base>::name_of_models()
{
    std::ostringstream oostring;
    auto list = list_of_models();

    for (size_t i = 0; i < list.size(); i++)
    {
        if (i == list.size() - 1)
        {
            oostring << list[i];
        }
        else
        {
            oostring << list[i] << ", ";
        }
    }

    return oostring.str();
}

template <class Base>
inline void ModelRunner<Base>::run_model(std::string model_name, bool relaxed)
{
    if (is_model_valid(model_name))
    {
        (*map_models[model_name]).solve(relaxed);
    }
    else
    {
        std::ostringstream error_message;
        error_message << "Model name do not exist in [" << name_of_models() << "]";

        throw std::invalid_argument(error_message.str());
    }
}

template <class Base>
template <class T>
inline void ModelRunner<Base>::insert_model(T new_model, std::string name)
{
    if (!is_model_valid(name))
    {
        map_models[name] = std::make_unique<T>(new_model);
    }
    else
    {
        throw std::invalid_argument("Registering a new model with an existing name.");
    }
}