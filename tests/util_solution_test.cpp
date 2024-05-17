#include <gtest/gtest.h>
#include "util_solution.h"
#include "test_utils.h"
#include "wsn_solution.h"

TEST(SolutionIsValid, TestOneSinkValidSolution)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 1;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{}});
    std::set<int> masters({0});
    std::set<int> bridges({});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = true;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestOneSinkNumberSinkDifferents)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 2;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{}});
    std::set<int> masters({0});
    std::set<int> bridges({});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestOneSinkInvalidMasterAdjMaster)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {0, 2},
        {0, 3},
        {0, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 1;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{}});
    std::set<int> masters({0, 1});
    std::set<int> bridges({});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestOneSinkValidSolutionAllNodes)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 1;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}, {2, 3}, {3, 4}});
    std::set<int> masters({0, 2, 4});
    std::set<int> bridges({1, 3});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = true;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestOneSinkInvalidSolutionNodeUnreacheable)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 1;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}});
    std::set<int> masters({0, 2});
    std::set<int> bridges({1});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestOneSinkInvalidSolutionDifferentNumberTrees)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 1;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}});
    std::set<int> masters({0, 2});
    std::set<int> bridges({1});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestTwoSinksValidSolution)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 2;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}});
    std::set<int> masters({0, 2, 4});
    std::set<int> bridges({1});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = true;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestTwoSinksInvalidSolutionTrivialTreeIsNotMaster)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 2;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}});
    std::set<int> masters({0, 2});
    std::set<int> bridges({1, 4});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestTwoSinksInvalidSolutionMastersAdjacents)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 2;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}});
    std::set<int> masters({0, 2, 3});
    std::set<int> bridges({1});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

TEST(SolutionIsValid, TestTwoSinksInvalidSolutionCycle)
{
    int number_nodes = 5;
    auto adj_matrix = adj_matrix_from_edges({
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 4}}, number_nodes);
    auto weight = square_matrix(number_nodes, 0.0);

    int trees = 2;

    WSN_data instance(weight, adj_matrix, number_nodes, trees);

    std::vector<std::vector<int>> matrix_x({{0, 1}, {1, 2}, {2, 0}});
    std::set<int> masters({0, 2, 4});
    std::set<int> bridges({1});

    auto adj_forest = adj_list_forest(matrix_x, instance);
    auto actual = is_solution_valid(instance, adj_forest, masters, bridges);

    const auto expected = false;
    
    ASSERT_EQ(expected, actual);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}