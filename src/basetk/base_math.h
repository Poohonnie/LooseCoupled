/**@file    base_math.h
 * @brief   数学方法类.h文件
 * @details 定义了各种常数以及GNSS系统的枚举类
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/27
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/27    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

#ifndef LOOSECOUPLED_SRC_BASETK_BASE_MATH_H
#define LOOSECOUPLED_SRC_BASETK_BASE_MATH_H

// c/c++系统文件

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "base_sdc.h"
#include "base_matrix.h"

/**@class   BaseMath
 * @brief   数学方法类, 定义了各种数学函数, 包括坐标转换, 四元数等
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseMath
{
public:
    // 坐标转换函数
    std::vector<double> XyzAdd(const std::vector<double> &add_xyz1,
                               const std::vector<double> &add_xyz2);  // 地心地固坐标加法
    std::vector<double> XyzSub(const std::vector<double> &minuend,
                               const std::vector<double> &subtrahend);  // 地心地固坐标减法
    std::vector<double> Blh2Xyz(const std::vector<double> &blh,
                                const CoorSys &coor_sys);  // 大地坐标转地心地固坐标
    std::vector<double> Xyz2Blh(const std::vector<double> &xyz,
                                const CoorSys &coor_sys);  // 地心地固坐标转大地坐标
    double Deg2Rad(const double &deg,
                   const double &min, const double &sec);  // 度分秒转弧度
    std::vector<double> CalcEnu(const std::vector<double> &ref_xyz,
                                const std::vector<double> &station_xyz);  // 计算测站在参考坐标下的ENU坐标(误差)
                                
    // 四元数相关运算
    std::vector<double> QuaternionMul(const std::vector<double> &quaternion1, const std::vector<double> &quaternion2);  // 四元数乘法
    double Norm(const std::vector<double> &vector);  // 向量取模
    void Normalize(std::vector<double> &vector);  // 四元数归一化
    
    // 角度转换
    BaseMatrix Euler2Cmn
    
};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_MATH_H
