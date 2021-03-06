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
 * <tr><td>2022/5/31    <td>Zing Fong   <td>增加了求反对称矩阵的函数, 更改了默认构造函数
 * <tr><td>2022/6/5     <td>Zing Fong   <td>加入了矩阵求迹函数
 * <tr><td>2022/6/9     <td>Zing Fong   <td>增加了三维列向量叉乘函数
 * <tr><td>2022/6/12    <td>Zing Fong   <td>增加了向量加减法
 * <tr><td>2022/6/18    <td>Zing Fong   <td>增加了向量求对角阵函数
 * </table>
 */
class BaseMatrix
{
  public:
    BaseMatrix() = default;  // 默认构造函数
    BaseMatrix(const std::vector<double> &mat,
               const int &row_num, const int &col_num);  // 构造函数
    BaseMatrix(const int &row_num, const int &col_num);  // 全零矩阵构造函数
    BaseMatrix(const BaseMatrix &src);  // 拷贝构造函数
    
    static BaseMatrix eye(const int &n);  // 单位阵
    static BaseMatrix zeros(const int &row_num, const int &col_num);  // 全零阵
    static BaseMatrix CalcAntisymmetryMat(
            const std::vector<double> &vec);  // 三维向量的反对称矩阵
    static std::vector<double> CrossProduct(const std::vector<double> &vec1,
                                   const std::vector<double> &vec2);  // 三维向量外积
    static std::vector<double> VectorAdd(const std::vector<double> &vec1,
                                         const std::vector<double> &vec2);  // 向量加法
    static std::vector<double> VectorSub(const std::vector<double> &vec1,
                                const std::vector<double> &vec2);  // 向量减法
    static BaseMatrix Diag(const std::vector<double> &vec);  // 向量求对角阵
    
    
    void disp(int width = 9, int precise = 4) const;  // 按照位宽和精度显示矩阵
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
    double Trace() const;  // 矩阵求迹
    void setZero();  // 将矩阵置零
    void InsertRow(const std::vector<double> &vec, const int &aim_row);  // 矩阵扩展, 加一行
    void InsertCol(const std::vector<double> &vec, const int &aim_col);  // 矩阵扩展, 加一列
    void EraseRow(const int &aim_row);  // 去掉一行
    void EraseCol(const int &aim_col);  // 去掉一列
    
    // get
    int get_row_num() const;
    int get_col_num() const;
    std::vector<double> get_mat() const;
    
    // set
    void set_row(const int &row);
    void set_col(const int &col);
  
  private:
    int row_num_ = 1;  // 矩阵行数
    int col_num_ = 1;  // 矩阵列数
    std::vector<double> mat_ = std::vector<double>(1, 0.0);  // 矩阵的一维数组存储
};


#endif // LOOSECOUPLED_SRC_BASETK_BASE_MATRIX_H
