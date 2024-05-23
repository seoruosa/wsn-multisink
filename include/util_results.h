#pragma once

#include <string>
#include <vector>
#include <regex>
#include <fstream>

/**
 * @brief Get the integer number from string input
 * 
 * @param a is a string with a integer number
 * @return int 
 */
int get_number(std::string a)
{
    const std::regex number_reg("\\d+");
    std::smatch n_match;
    int output(INT16_MAX);

    if (std::regex_search(a, n_match, number_reg))
    {
        output = std::stoi(n_match[0].str());
    }

    return output;
}

/**
 * @brief Get a pair of integer numbers separated by space
 * 
 * @param a is a string with a pair of integer numbers separated by space
 * @return std::pair<int, int> is the pair of integer numbers
 */
std::pair<int, int> get_pair(std::string a)
{
    const std::regex pieces_regex("(\\d+)[\\s\\t]+(\\w+)");
    std::smatch pieces_match;

    int n = 0;
    int m = 0;

    if (std::regex_match(a, pieces_match, pieces_regex))
    {
        std::sub_match sub_match = pieces_match[1];
        n = std::stoi(pieces_match[1].str());
        m = std::stoi(pieces_match[2].str());
    }

    return {n, m};
}

/**
 * @brief Read a matrix of integers from input file
 * 
 * @param input_file is the input input_file stream
 * @param rows is the number of rows to be read
 * @param columns is the number of columns to be read
 * @param skip_columns is the number of columns to be skipped
 * @return std::vector<std::vector<int>> is a matrix with size of (rows, columns)
 */
std::vector<std::vector<int>> get_int_matrix(std::ifstream &input_file, int rows, int columns, int skip_columns)
{
    std::string string_line;
    std::regex number_reg("\\d+");
    auto node_coord = std::vector<std::vector<int>>(rows, std::vector<int>(columns, 0));

    for (int i = 0; i < rows; i++)
    {
        std::getline(input_file, string_line);

        auto words_begin = std::sregex_iterator(string_line.begin(), string_line.end(), number_reg);
        auto words_end = std::sregex_iterator();

        int j = 0;
        for (auto ind = words_begin; ind != words_end; ++ind)
            while ((j < (columns + skip_columns)) & (ind != words_end))
            {
                if (j >= skip_columns)
                {
                    auto number_match = *ind;
                    node_coord[i][j - skip_columns] = std::stoi(number_match.str());
                }
                j++;
                ++ind;
            }
    }

    return node_coord;
}

/**
 * @brief Read a matrix of floats from input input_file
 * 
 * @tparam T is the type of the output (float, double, ...)
 * @param input_file 
 * @param rows is the number of rows to be read
 * @param columns is the number of columns to be read
 * @param skip_columns is the number of columns to be skipped
 * @return std::vector<std::vector<T>> is a matrix with size of (rows, columns)
 */
template <typename T>
std::vector<std::vector<T>> get_float_matrix(std::ifstream &input_file, int rows, int columns, int skip_columns)
{
    std::string string_line;
    std::regex number_reg("\\d+\\.{0,1}\\d*");
    auto node_coord = std::vector<std::vector<T>>(rows, std::vector<T>(columns, 0));

    for (int i = 0; i < rows; i++)
    {
        std::getline(input_file, string_line);

        auto words_begin = std::sregex_iterator(string_line.begin(), string_line.end(), number_reg);
        auto words_end = std::sregex_iterator();

        int j = 0;
        for (auto ind = words_begin; ind != words_end; ++ind)
            while ((j < (columns + skip_columns)) & (ind != words_end))
            {
                if (j >= skip_columns)
                {
                    auto number_match = *ind;
                    node_coord[i][j - skip_columns] = std::stof(number_match.str());
                }
                j++;
                ++ind;
            }
    }

    return node_coord;
}

/**
 * @brief Print in cout the rows of matrix and the respective value of values
 * 
 * @tparam T1 is the type of values of matrix
 * @tparam T2 is the type of values of values_list
 * @param matrix is a matrix of values, each row related to a value on values_list
 * @param values_list is a list of values related to the row of matrix
 * @param var_name is the name of variable that matrix and values_list are related
 * @param cout is the output stream
 */
template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, 
                  std::vector<T2> &values_list, 
                  std::string var_name, 
                  std::ostream &cout)
{
    cout << var_name << " : " << matrix.size() << " ";
    if (matrix.empty())
    {
        cout << matrix.size() << std::endl;
    }
    else
    {
        cout << matrix.front().size() << std::endl;
    }

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            if (j == matrix[i].size() - 1)
            {
                cout << matrix[i][j] << "\t: " << values_list[i] << std::endl;
            }
            else
            {
                cout << matrix[i][j] << "\t";
            }
        }
    }
    cout << std::endl;
}

/**
 * @brief Print in standard output the rows of matrix and the respective value of values
 * 
 * @tparam T1 is the type of values of matrix
 * @tparam T2 is the type of values of values_list
 * @param matrix is a matrix of values, each row related to a value on values_list
 * @param values_list is a list of values related to the row of matrix
 * @param var_name is the name of variable that matrix and values_list are related
 */
template <typename T1, typename T2>
void print_matrix(std::vector<std::vector<T1>> &matrix, 
                  std::vector<T2> &values_list, 
                  std::string var_name)
{
    print_matrix(matrix, values_list, var_name, std::cout);
}

/**
 * @brief Print in cout output the rows of matrix
 * 
 * @tparam T is the type of values of matrix
 * @param matrix is a matrix of values
 * @param var_name is the name of variable that matrix and values_list are related
 * @param cout is the output stream
 */
template <typename T>
void print_matrix(std::vector<std::vector<T>> &matrix, 
                  std::string var_name, 
                  std::ostream &cout)
{
    cout << var_name << " : " << matrix.size() << " ";
    if (matrix.empty())
    {
        cout << matrix.size() << std::endl;
    }
    else
    {
        cout << matrix.front().size() << std::endl;
    }

    for (int i = 0; i < matrix.size(); i++)
    {
        for (int j = 0; j < matrix[i].size(); j++)
        {
            if (j == matrix[i].size() - 1)
            {
                cout << matrix[i][j] << std::endl;
            }
            else
            {
                cout << matrix[i][j] << "\t";
            }
        }
    }
    cout << std::endl;
}

/**
 * @brief Print in standard output the rows of matrix
 * 
 * @tparam T is the type of values of matrix
 * @param matrix is a matrix of values
 * @param var_name is the name of variable that matrix and values_list are related
 */
template <typename T>
void print_matrix(std::vector<std::vector<T>> &matrix, 
                  std::string var_name)
{
    print_matrix(matrix, var_name, std::cout);
}

/**
 * @brief Methods for printing output
 * 
 */
namespace print
{
    /**
     * @brief Returns a string with actual time. More about the format at https://www.programiz.com/cpp-programming/library-function/ctime/strftime
     * 
     * @param fmt is the output format
     * @return std::string 
     */
    std::string time_now(const char *fmt)
    {
        auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        char date_string[100];

        std::strftime(date_string, 100, fmt, std::localtime(&time));
        std::string out(date_string);

        return out;
    };

    /**
     * @brief Returns a string with actual time with the format %Y-%m-%d %H:%M:%S
     * 
     * @return std::string 
     */
    std::string time_now()
    {
        return time_now("%Y-%m-%d %H:%M:%S");
    }
} // namespace print

/**
 * @brief Methods for measurement of the performance
 * 
 */
namespace perf
{
    namespace time
    {
        /**
         * @brief Return a starting point for the measurement of duration of event
         * 
         * @return std::chrono::high_resolution_clock::time_point 
         */
        std::chrono::high_resolution_clock::time_point start()
        {
            return std::chrono::high_resolution_clock::now();
        }

        /**
         * @brief Return the duration of the event that started at start_ and finished when method is called
         * 
         * @param start_ is the start of event
         * @return std::chrono::duration<double> returns the durations of event in seconds
         */
        std::chrono::duration<double> duration(std::chrono::high_resolution_clock::time_point start_)
        {
            auto end_gen = start();
            std::chrono::duration<double> duration(end_gen - start_);

            return duration;
        }
    }
}