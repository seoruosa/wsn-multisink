#pragma once

#include <string>
#include <vector>
#include <set>

#include "util.h"
#include <filesystem>
#include <algorithm>

/**
 * Contains data associated with an instance of WSN problem
 */
class WSN_data
{
private:
    void initialize_calculated_data();

    // path of the instance
    std::string instance_path;

    // is initialized by an file instance?
    bool is_file_instance;

public:
    // WSN_data();
    WSN_data(std::vector<std::vector<double>> &weight, std::vector<std::vector<int>> &is_connected,
             int n, int number_of_trees);
    WSN_data(std::vector<std::vector<double>> &weight, std::vector<std::vector<int>> &is_connected,
             int n);
    WSN_data(std::string path, int number_trees);
    WSN_data(std::string path);
    ~WSN_data();

    // matrix with the weight associated to an arc
    std::vector<std::vector<double>> weight;

    // matrix with 1 if the arc exists, 0 otherwise
    std::vector<std::vector<int>> is_connected;

    // number of nodes
    int n;

    // number of trees
    int number_trees;

    // set number of trees (update the calculated data)
    void set_number_trees(int number_of_trees);

    /**
     * Calculated data
     * */
    
    // list of elements that have a arc from node v
    std::vector<std::set<int>> adj_list_from_v;

    // list of elements that is connected to node v
    std::vector<std::set<int>> adj_list_to_v;

    std::string name();

    // implementing how to print WSN_data
    friend std::ostream &operator<<(std::ostream &os, const WSN_data &l);
};

// WSN_data::WSN_data(/* args */)
// {
// }

WSN_data::WSN_data(std::vector<std::vector<double>> &weight, std::vector<std::vector<int>> &is_connected,
                   int n, int number_of_trees) : weight(weight), is_connected(is_connected),
                                                 n(n), number_trees(number_of_trees),
                                                 is_file_instance(false)
{
    initialize_calculated_data();
}

WSN_data::WSN_data(std::vector<std::vector<double>> &weight, std::vector<std::vector<int>> &is_connected,
                   int n) : weight(weight), is_connected(is_connected),
                            n(n), number_trees(1), is_file_instance(false)
{
    initialize_calculated_data();
}

WSN_data::WSN_data(std::string path, int number_trees) : instance_path(path),
                                                         number_trees(number_trees),
                                                         is_file_instance(true)
{
    WSN_data::n = read_instance_wsn(instance_path, WSN_data::weight, WSN_data::is_connected);

    initialize_calculated_data();
}

WSN_data::WSN_data(std::string path) : instance_path(path),
                                       number_trees(1),
                                       is_file_instance(true)
{
    WSN_data::n = read_instance_wsn(instance_path, WSN_data::weight, WSN_data::is_connected);

    initialize_calculated_data();
}

WSN_data::~WSN_data()
{
}

void WSN_data::initialize_calculated_data()
{
    WSN_data::adj_list_from_v = to_adj_list_out(WSN_data::is_connected);
    WSN_data::adj_list_to_v = to_adj_list_in(WSN_data::is_connected);
}

void WSN_data::set_number_trees(int number_of_trees)
{
    WSN_data::number_trees = number_of_trees;

    WSN_data::initialize_calculated_data();
}

inline std::string WSN_data::name()
{
    auto instance_name = std::filesystem::path(instance_path).filename().stem().string();
    std::replace(instance_name.begin(), instance_name.end(), '_', '-');

    return instance_name;
}

std::ostream &operator<<(std::ostream &os, const WSN_data &l)
{
    os << "instance_path:\t" << (l.is_file_instance ? l.instance_path : "None") << std::endl;
    os << "number_nodes:\t" << l.n << std::endl;
    os << "number_trees:\t" << l.number_trees << std::endl;

    return os;
}