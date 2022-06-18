/**@file    base_matrix.cc
 * @brief   矩阵运算类.cc文件
 * @details 矩阵的四则运算、转置、求逆等操作的具体实现
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/1
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/1    <td>1.0      <td>Zing Fong  <td>Initialize
 * <tr><td>2022/6/5    <td>1.1      <td>Zing Fong  <td>加入了矩阵求迹函数
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "base_matrix.h"
// c/c++系统文件
#include <iostream>
#include <iomanip>
// 其他库的 .h 文件
#include <cmath>

// 本项目内 .h 文件

/**@brief          构造函数
 * @param[in]      mat          用于构造矩阵的一维数组
 * @param[in]      row_num      矩阵行数
 * @param[in]      col_num      矩阵列数
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix::BaseMatrix(const std::vector<double> &mat,
                       const int &row_num, const int &col_num)
{
    if(row_num > 0 && col_num > 0 &&
       mat.size() == row_num*col_num)  // 行列数不为零, 且与数组元素匹配
    {
        row_num_ = row_num;
        col_num_ = col_num;
        mat_ = mat;
    }
    else
    {
        printf("Constructor error!\n");
        *this = BaseMatrix();  // 默认构造
    }
}

/**@brief          全零构造函数
 * @param[in]      row_num      矩阵行数
 * @param[in]      col_num      矩阵列数
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix::BaseMatrix(const int &row_num, const int &col_num)
{
    if(row_num > 0 && col_num > 0)  // 行列数均不为0
    {
        row_num_ = row_num;
        col_num_ = col_num;
        mat_ = std::vector<double>(row_num_*col_num_, 0.0);
    }
    else
    {
        printf("Constructor error!\n");
        *this = BaseMatrix();  // 默认构造
    }
}

/**@brief          拷贝构造函数
 * @param[in]      src          源矩阵
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix::BaseMatrix(const BaseMatrix &src)
{
    if(src.row_num_ > 0 && src.col_num_ > 0)  // 行列数均不为0
    {
        row_num_ = src.row_num_;
        col_num_ = src.col_num_;
        mat_ = src.mat_;
    }
    else
    {
        printf("Constructor error!\n");
        *this = BaseMatrix();  // 默认构造
    }
}

/**@brief           单位阵
 * @param[in]       n          单位阵维数, > 0
 * @return          返回结果\n
 * - n > 0(成功)      一个n×n维的单位阵\n
 * - n < 0(失败)      默认构造1×1矩阵\n
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::eye(const int &n)
{
    if(n > 0)
    {
        BaseMatrix eye_mat(n, n);
        for(int i = 0; i < n; ++i)
            eye_mat.write(i, i, 1);  // 对角线写入元素
        return eye_mat;
    }
    else
        return {};
}

/**@brief           全零阵
 * @param[in]       row_num         矩阵行数
 * @param[in]       col_num         矩阵列数
 * @return          返回结果\n
 * - n > 0(成功)     一个row_num×col_num维的全零阵\n
 * - n < 0(失败)     默认构造1×1矩阵\n
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::zeros(const int &row_num, const int &col_num)
{
    if(row_num > 0 && col_num > 0)
        return BaseMatrix(row_num, col_num);
    else
    {
        printf("Constructor error!\n");
        return {};
    }
}

/**@brief           计算三维向量的反对称矩阵
 * @param[in]       vec             3维向量
 * @return          返回结果\n
 * - 向量维数为3      向量的3×3维的反对称矩阵\n
 * - 向量维数不为3    3×3维的单位阵\n
 * @author          Zing Fong
 * @date            2022/6/1
 */
BaseMatrix BaseMatrix::CalcAntisymmetryMat(const std::vector<double> &vec)
{
    if(vec.size() == 3)  // 确实是三维向量
    {
        BaseMatrix result(3, 3);
        result.write(0, 1, -vec[2]);
        result.write(0, 2, vec[1]);
        result.write(1, 0, vec[2]);
        result.write(1, 2, -vec[0]);
        result.write(2, 0, -vec[1]);
        result.write(2, 1, vec[0]);
        return result;
    }
    else
    {
        printf("Calculate antisymmetry matrix error! Vector size: %d\n",
               vec.size());
        return eye(3);
    }
}

/**@brief           计算两个三维向量的外积
 * @param[in]       vec1             3维向量
 * @param[in]       vec2             3维向量
 * @return          返回结果\n
 * - 向量维数正确     两个向量的叉乘结果, 3×1的矩阵\n
 * - 向量维数错误     3×1的全零矩阵\n
 * @author          Zing Fong
 * @date            2022/6/12
 */
std::vector<double> BaseMatrix::CrossProduct(const std::vector<double> &vec1,
                                             const std::vector<double> &vec2)
{
    if(vec1.size() != 3 || vec2.size() != 3)
    {
        // 不是两个三维向量
        printf("CrossProduct error!\n");
        return std::vector<double>(3, 0.0);
    }
    std::vector<double> result(3, 0.0);
    result[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    result[1] = -(vec1[0]*vec2[2] - vec1[2]*vec2[0]);
    result[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    
    return result;
}

/**@brief           向量加法
 * @param[in]       vec1            待求和向量1
 * @param[in]       vec2            待求和向量2
 * @return          两个向量的和
 * @author          Zing Fong
 * @date            2022/6/12
 */
std::vector<double> BaseMatrix::VectorAdd(const std::vector<double> &vec1,
                                          const std::vector<double> &vec2)
{
    if(vec1.empty() || vec2.empty() || vec1.size() != vec2.size())
    {
        // 保证两个向量都不为空且维数相同
        printf("VectorAdd error. vector1 size: %d vector2 size: %d",
               vec1.size(), vec2.size());
        return {};  //
    }
    std::vector<double> result(vec1.size(), 0.0);
    for(int i = 0; i < vec1.size(); ++i)
        result[i] = vec1[i] + vec2[i];
    return result;
}

/**@brief           向量加法
 * @param[in]       vec1            被减数向量
 * @param[in]       vec2            减数向量
 * @return          两个向量的差
 * @author          Zing Fong
 * @date            2022/6/12
 */
std::vector<double> BaseMatrix::VectorSub(const std::vector<double> &vec1,
                                          const std::vector<double> &vec2)
{
    if(vec1.empty() || vec2.empty() || vec1.size() != vec2.size())
    {
        // 保证两个向量都不为空且维数相同
        printf("VectorSub error. vector1 size: %d vector2 size: %d",
               vec1.size(), vec2.size());
        return {};  //
    }
    std::vector<double> result(vec1.size(), 0.0);
    for(int i = 0; i < vec1.size(); ++i)
        result[i] = vec1[i] - vec2[i];
    return result;
}

/**@brief           向量求对角阵
 * @param[in]       vec            输入的向量
 * @return          以该向量为主对角线元素的对角阵
 * @author          Zing Fong
 * @date            2022/6/18
 */
BaseMatrix BaseMatrix::Diag(const std::vector<double> &vec)
{
    auto size = vec.size();
    BaseMatrix diag(size, size);
    for(int i = 0; i < size; ++i)
        diag.write(i, i, vec[i]);
    
    return diag;
}

/**@brief           矩阵显示函数
 * @param[in]       width            输出位宽, 默认为9
 * @param[in]       precise          输出精度, 默认为4
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::disp(int width, int precise) const
{
    for(int i = 0; i < row_num_; ++i)
    {
        for(int j = 0; j < col_num_; j++)
        {
            std::cout << std::setw(width) << std::setiosflags(std::ios::fixed)
                      << std::setprecision(precise) << this->read(i, j) << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

/**@brief           矩阵读操作
 * @param[in]       row          待读行号
 * @param[in]       col          待读列号
 * @return      矩阵row行col列的值
 * @author      Zing Fong
 * @date        2022/6/1
 */
double BaseMatrix::read(const int &row, const int &col) const
{
    if(row < row_num_ && col < col_num_)
        return mat_[row*col_num_ + col];
    else
    {
        printf("Read matrix error!\n");
        return -114514.0;
    }
}

/**@brief           矩阵写操作
 * @param[in]       row          待写行号
 * @param[in]       col          待写列号
 * @param[in]       val          待写入的值
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::write(const int &row, const int &col, const double &val)
{
    if(row < row_num_ && col < col_num_)
        mat_[row*col_num_ + col] = val;
    else
        printf("Write matrix error!\n");
}

/**@brief       “=”重载, 深拷贝
 * @param[in]   src          待拷贝的BaseMatrix对象
 * @return      拷贝后的this指针
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix &BaseMatrix::operator=(const BaseMatrix &src)
{
    if(this == &src) return *this;  // 身份检测
    col_num_ = src.col_num_;
    row_num_ = src.row_num_;
    mat_ = src.mat_;
    return *this;
}

/**@brief       “+”重载
 * @param[in]   add_mat        待加的矩阵
 * @return      "+"左右两个矩阵相加的结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::operator+(const BaseMatrix &add_mat) const
{
    if(row_num_ == add_mat.row_num_ && col_num_ == add_mat.col_num_ &&
       mat_.size() == add_mat.mat_.size())
    {
        // 矩阵维数一致才可以进行加法运算
        BaseMatrix result(row_num_, col_num_);
        for(int i = 0; i < mat_.size(); ++i)
            result.mat_[i] = this->mat_[i] + add_mat.mat_[i];
        return result;
    }
    else
    {
        printf("Matrix addition error! left size: %d×%d, right size: %d×%d\n",
               row_num_, col_num_, add_mat.row_num_, add_mat.col_num_);
        return *this;
    }
}

/**@brief       “-”重载
 * @param[in]   subtrahend        减数矩阵
 * @return      "-"左边矩阵减去右边矩阵的结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::operator-(const BaseMatrix &subtrahend) const
{
    if(row_num_ == subtrahend.row_num_ && col_num_ == subtrahend.col_num_ &&
       mat_.size() == subtrahend.mat_.size())
    {
        // 矩阵维数一致才可以进行减法运算
        BaseMatrix result(row_num_, col_num_);
        for(int i = 0; i < mat_.size(); ++i)
            result.mat_[i] = this->mat_[i] - subtrahend.mat_[i];
        return result;
    }
    else
    {
        printf("Matrix subtraction error! left size: %d×%d, right size: %d×%d\n",
               row_num_, col_num_, subtrahend.row_num_, subtrahend.col_num_);
        return *this;
    }
}

/**@brief       “+=”重载
 * @param[in]   add_mat        待加的矩阵
 * @return      "+"左右两个矩阵相加的结果, 并赋值给左边
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix &BaseMatrix::operator+=(const BaseMatrix &add_mat)
{
    if(row_num_ == add_mat.row_num_ && col_num_ == add_mat.col_num_ &&
       mat_.size() == add_mat.mat_.size())
    {
        // 矩阵维数一致才可以进行加法运算
        for(int i = 0; i < mat_.size(); ++i)
            this->mat_[i] += add_mat.mat_[i];
    }
    else
        printf("Matrix addition error! left size: %d×%d, right size: %d×%d\n",
               row_num_, col_num_, add_mat.row_num_, add_mat.col_num_);
    return *this;
}

/**@brief       “-=”重载
 * @param[in]   subtrahend        减数矩阵
 * @return      "-"左边矩阵减去右边矩阵的结果, 并赋值给左边
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix &BaseMatrix::operator-=(const BaseMatrix &subtrahend)
{
    if(row_num_ == subtrahend.row_num_ && col_num_ == subtrahend.col_num_ &&
       mat_.size() == subtrahend.mat_.size())
    {
        // 矩阵维数一致才可以进行减法运算
        for(int i = 0; i < mat_.size(); ++i)
            this->mat_[i] -= subtrahend.mat_[i];
    }
    else
        printf("Matrix subtraction error! left size: %d×%d, right size: %d×%d\n",
               row_num_, col_num_, subtrahend.row_num_, subtrahend.col_num_);
    return *this;
}

/**@brief       “*”重载, 矩阵与矩阵相乘
 * @param[in]   multiplier        乘数矩阵
 * @return      "*"左右两边矩阵相乘的结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::operator*(const BaseMatrix &multiplier) const
{
    if(col_num_ == multiplier.row_num_)
    {
        // 确认左矩阵的列数与右矩阵的行数是否一致
        int m = row_num_;
        int n = col_num_;
        int p = multiplier.col_num_;
        BaseMatrix result(m, p);  // 声明一个积矩阵
        // 实际上这里循环顺序并不重要，对于一维数组来说顺序读取和抽样读取速度差异不大
        for(int i = 0; i < m; ++i)
            for(int j = 0; j < n; j++)
                for(int k = 0; k < p; k++)
                    result.mat_[i*p + k] += (mat_[i*n + j]*
                                             multiplier.mat_[j*p + k]);
        return result;
    }
    else
    {
        printf("Matrix multiplication error! left size: %d×%d, right size: %d×%d\n",
               row_num_, col_num_, multiplier.row_num_, multiplier.col_num_);
        return *this;
    }
}

/**@brief       “*”重载, 矩阵数乘
 * @param[in]   scalar        数乘的系数
 * @return      矩阵数乘结果, 矩阵在"*"左边, 系数在右边
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::operator*(const double &scalar) const
{
    BaseMatrix result(*this);
    for(double &a_mat: result.mat_)
        a_mat *= scalar;
    return result;
}

/**@brief       高斯约当法矩阵求逆
 * @details     算法我也不太懂, 代码抄的, 能跑就行
 * @return      该矩阵求逆结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::Inverse() const
{
    int n = row_num_;
    BaseMatrix inv_mat(n, n);
    std::vector<double> a = mat_;
    std::vector<double> b = inv_mat.mat_;
    int *is = new int[n*n];
    int *js = new int[n*n];
    int i, j, k, l, u, v;
    double d, p;
    
    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    b = a;
    for(k = 0; k < n; k++)
    {
        d = 0.0;
        for(i = k; i < n; ++i)   /* 查找右下角方阵中主元素的位置 */
        {
            for(j = k; j < n; j++)
            {
                l = n*i + j;
                p = fabs(b[l]);
                if(p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }
        
        if(fabs(d) < 1.0E-15)
        {
            return eye(n);
        }
        
        if(is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for(j = 0; j < n; j++)
            {
                u = k*n + j;
                v = is[k]*n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        if(js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for(i = 0; i < n; ++i)
            {
                u = i*n + k;
                v = i*n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        
        l = k*n + k;
        b[l] = 1.0/b[l];  /* 初等行变换 */
        for(j = 0; j < n; j++)
        {
            if(j != k)
            {
                u = k*n + j;
                b[u] = b[u]*b[l];
            }
        }
        for(i = 0; i < n; ++i)
        {
            if(i != k)
            {
                for(j = 0; j < n; j++)
                {
                    if(j != k)
                    {
                        u = i*n + j;
                        b[u] = b[u] - b[i*n + k]*b[k*n + j];
                    }
                }
            }
        }
        for(i = 0; i < n; ++i)
        {
            if(i != k)
            {
                u = i*n + k;
                b[u] = -b[u]*b[l];
            }
        }
    }
    
    for(k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
    {
        if(js[k] != k)
        {
            for(j = 0; j < n; j++)
            {
                u = k*n + j;
                v = js[k]*n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if(is[k] != k)
        {
            for(i = 0; i < n; ++i)
            {
                u = i*n + k;
                v = is[k] + i*n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }
    inv_mat.mat_ = b;
    
    delete[] is;
    delete[] js;
    
    return inv_mat;
}

/**@brief       矩阵转置
 * @return      该矩阵转置结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
BaseMatrix BaseMatrix::Trans() const
{
    int m = row_num_, n = col_num_;
    BaseMatrix trans_mat(n, m);
    for(int i = 0; i < m; ++i)
        for(int j = 0; j < n; j++)
            trans_mat.mat_[j*m + i] = mat_[i*n + j];  // 原矩阵i行j列元素赋值到转置矩阵中j行i列处
    return trans_mat;
}

/**@brief       矩阵求迹
 * @return      该矩阵的迹
 * @author      Zing Fong
 * @date        2022/6/5
 */
double BaseMatrix::Trace() const
{
    if(row_num_ != col_num_)
    {
        printf("Calculation trace error: Not a square.\n");
        return 0.0;
    }
    double trace{};
    for(int i = 0; i < row_num_; ++i)
        trace += this->read(i, i);
    return trace;
}

/**@brief       矩阵置零
 * @details     将该矩阵元素全部置零
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::setZero()
{
    for(auto &a_mat: mat_)
        a_mat = 0.0;
}

/**@brief       向矩阵中插入一行
 * @param[in]   vec         要插入的行
 * @param[in]   aim_row     要插入的位置(行号)
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::InsertRow(const std::vector<double> &vec, const int &aim_row)
{
    if(aim_row > row_num_ || aim_row < 0)
    {
        // 要插入的行不在矩阵范围内
        printf("Matrix InsertRow function error! matrix size: %d×%d, aim_row: %d\n",
               row_num_, col_num_, aim_row);
        return;
    }
    ++row_num_;
    mat_.reserve((row_num_ + 1)*col_num_);
    mat_.insert(mat_.begin() + aim_row*col_num_, vec.cbegin(), vec.cend());
    
}

/**@brief       向矩阵中插入一列
 * @param[in]   vec         要插入的行
 * @param[in]   aim_col     要插入的位置(列号)
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::InsertCol(const std::vector<double> &vec, const int &aim_col)
{
    BaseMatrix tmp = this->Trans();
    tmp.InsertRow(vec, aim_col);
    *this = tmp.Trans();
}

/**@brief       将矩阵删除一行
 * @param[in]   aim_row     要删除的位置(行号)
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::EraseRow(const int &aim_row)
{
    if(aim_row > row_num_ - 1 || aim_row < 0)
    {
        // 要删除的行不在矩阵范围内
        printf("Matrix EraseRow function error! matrix size: %d×%d, aim_row: %d\n",
               row_num_, col_num_, aim_row);
        return;
    }
    --row_num_;
    mat_.erase(mat_.begin() + aim_row*col_num_,
               mat_.begin() + (aim_row + 1)*col_num_);
}

/**@brief       将矩阵删除一列
 * @param[in]   aim_col     要删除的位置(列号)
 * @author      Zing Fong
 * @date        2022/6/1
 */
void BaseMatrix::EraseCol(const int &aim_col)
{
    BaseMatrix tmp = this->Trans();
    tmp.EraseRow(aim_col);
    *this = tmp.Trans();
}

int BaseMatrix::get_row_num() const
{
    return row_num_;
}

int BaseMatrix::get_col_num() const
{
    return col_num_;
}

std::vector<double> BaseMatrix::get_mat() const
{
    return mat_;
}

void BaseMatrix::set_row(const int &row)
{
    if(row > 0)
        row_num_ = row;
    else
        printf("Set row number error!\n");
}

void BaseMatrix::set_col(const int &col)
{
    if(col > 0)
        col_num_ = col;
    else
        printf("Set column number error!\n");
}
