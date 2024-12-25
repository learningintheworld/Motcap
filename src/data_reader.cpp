#include "data_reader.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

DataReader::DataReader(const std::string& filename) 
    : current_row(0), total_rows(34), cols_per_row(14) {
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    // 预分配内存以提高性能
    all_data.reserve(total_rows);

    // 逐行读取CSV文件
    while (std::getline(file, line)) {
        std::vector<double> row_data;
        row_data.reserve(cols_per_row);
        
        std::stringstream ss(line);
        std::string value;

        // 解析每行中的值
        while (std::getline(ss, value, ',')) {
            try {
                row_data.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "解析错误: " << e.what() << std::endl;
            }
        }

        if (row_data.size() == cols_per_row) {
            all_data.push_back(row_data);
        }
    }

    file.close();

    if (all_data.size() != total_rows) {
        std::cerr << "警告: 实际读取的行数 (" << all_data.size() 
                  << ") 与预期行数 (" << total_rows << ") 不符" << std::endl;
    }
}

std::vector<double> DataReader::getNextRow() {
    if (current_row >= all_data.size() - 1) {
        current_row = 0;  // 循环读取数据
        return all_data[all_data.size() - 1];
    }
    return all_data[current_row++];
}

std::vector<double> DataReader::getPreviousRow() {
    if (current_row == 0) {// 循环读取数据
        // 如果当前在第一行，则移动到最后一行
        current_row = all_data.size() - 1;
        return all_data[0];
    }
    return all_data[--current_row];
}

size_t DataReader::getTotalRows() const {
    return all_data.size();
}

size_t DataReader::getCurrentRow() const {
    return current_row;
}

void DataReader::reset() {
    current_row = 0;
}

void DataReader::setCurrentRow(size_t row) {
    if (isValidRow(row)) {
        current_row = row;
    } else {
        throw std::out_of_range("行号超出有效范围");
    }
}

bool DataReader::isValidRow(size_t row) const {
    return row < all_data.size();
}

