/**@file    base_matrix.h
 * @brief   矩阵运算类头文件
 * @details 声明BaseMatrix类，即矩阵类，实现了矩阵的四则运算、转置、求逆等操作
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/25
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/25    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_BASETK_BASE_MATRIX_H
#define LOOSECOUPLED_SRC_BASETK_BASE_MATRIX_H


// c/c++系统文件

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件


/**@class   BaseMatrix
 * @brief   一维数组实现的矩阵类
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/25    <td>Zing Fong   <td>Initialize
 * <tr><td>2022/5/31    <td>Zing Fong   <td>增加了求反对称矩阵的函数
 * </table>
 */
class BaseMatrix
{
  public:
    BaseMatrix();  // 默认构造函数
    BaseMatrix(const std::vector<double> &mat, const int &row,
               const int &col);  // 构造函数
    BaseMatrix(const int &row, const int &col);  // 全零矩阵构造函数
    BaseMatrix(const BaseMatrix &src);  // 拷贝构造函数
    static BaseMatrix eye(const int &n);  // 单位阵
    static BaseMatrix zeros(const int &row, const int &col);  // 全零阵
    static BaseMatrix CalcAntisymmetryMat(
            const std::vector<double> vector);  // 求向量的反对称矩阵
    
    void disp(const int width = 9, const int presise = 4);  // 按照位宽和精度显示矩阵
    double read(const int &row, const int &col) const;  // 读取矩阵元素
    void write(const int &row, const int &col, const double &val);  // 向矩阵中写入值
    
    BaseMatrix &operator=(const BaseMatrix &src);  // 矩阵复制
    BaseMatrix operator+(const BaseMatrix &add_mat) const;  // 矩阵加法
    BaseMatrix operator-(const BaseMatrix &subtrahend) const;  // 矩阵减法
    BaseMatrix &operator+=(const BaseMatrix &add_mat);  // +=
    BaseMatrix &operator-=(const BaseMatrix &subtrahend);  // -=
    BaseMatrix operator*(const BaseMatrix &multiplier) const;  // 矩阵乘法
    BaseMatrix operator*(const double &scalar) const;  // 矩阵数乘
    
    BaseMatrix Inverse() const;  // 矩阵求逆, 返回该矩阵的逆矩阵
    BaseMatrix Trans() const;  // 矩阵转置, 返回该矩阵的转置矩阵
    void setZero();  // 将矩阵置零
    void addRow(const std::vector<double> &vec,
                const int &aim_row);  // 矩阵扩展, 加一行
    void addCol(const std::vector<double> &vec,
                const int &aim_col);  // 矩阵扩展, 加一列
    void subRow(const int &aim_row);  // 去掉一行
    void subCol(const int &aim_col);  // 去掉一列
    
    // row_的set() get()
    void set_row(const int &row);
    int get_row() const;
    // col_的set() get()
    void set_col(const int &col);
    int get_col() const;
    // mat_的get()
    std::vector<double> get_mat() const;
  
  private:
    int row_{};  // 矩阵行数
    int col_{};  // 矩阵列数
    std::vector<double> mat_{};  // 矩阵的一维数组存储
};


#endif // LOOSECOUPLED_SRC_BASETK_BASE_MATRIX_H
