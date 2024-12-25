#ifndef __DATA_READER_H
#define __DATA_READER_H

#include <vector>
#include <string>

class DataReader {
public:
    // 构造函数
    DataReader(const std::string& filename);
    
    // // 析构函数（如果需要的话）
    // ~DataReader();
    
    // 获取下一行数据的接口
    std::vector<double> getNextRow();

    // 新增获取上一行数据的接口
    std::vector<double> getPreviousRow();
    
    // 其他可能需要的公共接口
    size_t getTotalRows() const;
    size_t getCurrentRow() const;
    void reset();  // 重置到文件开始
    void setCurrentRow(size_t row); // 新增：设置当前行

private:
    std::vector<std::vector<double>> all_data;  // 存储所有数据
    size_t current_row;                         // 当前读取的行
    const size_t total_rows;                    // 总行数
    const size_t cols_per_row;                  // 每行的列数

    // 新增：检查行号是否有效的辅助函数
    bool isValidRow(size_t row) const;
};






#endif



