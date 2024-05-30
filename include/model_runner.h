#pragma once

#include <string>
#include <map>
#include <memory>
#include <stdexcept>
#include <sstream>

template <class Base>
class ModelRunner
{
private:
    std::map<std::string, std::unique_ptr<Base>> map_models;
    bool is_model_valid(std::string model_name) { return (map_models.find(model_name) != map_models.end()); };
    std::vector<std::string> list_of_models();
    std::string name_of_models();

public:
    ModelRunner() {}
    ~ModelRunner(){};

    template <class T>
    void insert_model(T abc, std::string name)
    {
        map_models[name] = std::make_unique<T>(abc);
    }

    void run_model(std::string model_name, bool relaxed)
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
    };
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
};
