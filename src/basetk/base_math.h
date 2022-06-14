/**@file    base_math.h
 * @brief   数学方法类.h文件
 * @details 定义了坐标转换, 四元数相关运算, 姿态转换等数学方法
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/27
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/27    <td>1.0      <td>Zing Fong  <td>Initialize
 * <tr><td>2022/6/5     <td>1.1      <td>Zing Fong  <td>修正了对constexpr变量引用的错误
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
 * @brief   数学方法类, 定义了各种数学函数, 包括坐标转换, 四元数, 姿态转换等
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * <tr><td>2022/6/10    <td>Zing Fong   <td>将max和min函数的参数类型更改为vector
 * <tr><td>2022/6/12    <td>Zing Fong   <td>增加了计算n系重力加速度矢量的函数
 * <tr><td>2022/6/14    <td>Zing Fong   <td>增加了NED系和ENU系相互转换的函数
 * </table>
 */
class BaseMath
{
  public:
    static double max(const std::vector<double> &list);  // 求最大值
    static double min(const std::vector<double> &list);  // 求最小值
    
    // 坐标转换函数
    static std::vector<double> XyzAdd(const std::vector<double> &add_xyz1,
                                      const std::vector<double> &add_xyz2);  // 地心地固坐标加法
    static std::vector<double> XyzSub(const std::vector<double> &minuend,
                                      const std::vector<double> &subtrahend);  // 地心地固坐标减法
    static std::vector<double> Blh2Xyz(const std::vector<double> &blh,
                                       CoorSys coor_sys = BaseSdc::wgs84);  // 大地坐标转地心地固坐标
    static std::vector<double> Xyz2Blh(const std::vector<double> &xyz,
                                       CoorSys coor_sys = BaseSdc::wgs84);  // 地心地固坐标转大地坐标
    static double Deg2Rad(const int &deg,
                          const int &min, const double &sec);  // 度分秒转弧度
    static std::vector<double> CalcDenu(const std::vector<double> &ref_xyz,
                                        const std::vector<double> &station_xyz);  // 计算测站在参考坐标下的ENU坐标(误差)
    static std::vector<double> Ned2Enu(const std::vector<double> &ned);  // NED转ENU
    static std::vector<double> Enu2Ned(const std::vector<double> &enu);  // ENU转NED
    
    
    // 四元数相关运算
    static std::vector<double> QuaternionMul(
            const std::vector<double> &quaternion1,
            const std::vector<double> &quaternion2);  // 四元数乘法
    static double Norm(const std::vector<double> &vector);  // 向量取模
    static void Normalize(std::vector<double> &vector);  // 向量归一化
    static void QuaternionNormalize(std::vector<double> &quaternion);  // 四元数归一化
    
    // 姿态转换
    // 欧拉角排列顺序roll, pitch, yaw
    // 旋转顺序R(yaw, pitch, roll)
    static BaseMatrix Euler2RotationMat(
            const std::vector<double> &euler);  // 欧拉角转旋转矩阵
    static std::vector<double> RotationMat2Euler(
            const BaseMatrix &rotation_mat);  // 旋转矩阵转欧拉角
    static std::vector<double> Euler2Quaternion(
            const std::vector<double> &euler);  // 欧拉角转四元数
    static std::vector<double> Quaternion2Euler(
            const std::vector<double> &quaternion);  // 四元数转欧拉角
    static BaseMatrix Quaternion2RotationMat(
            const std::vector<double> &quaternion);  // 四元数转旋转矩阵
    static std::vector<double> RotationMat2Quaternion(
            const BaseMatrix &rotation_mat);  // 旋转矩阵转四元数
    static std::vector<double> Quaternion2RotationVec(
            const std::vector<double> &quaternion);  // 四元数转旋转矢量
    static std::vector<double> RotationVec2Quaternion(
            const std::vector<double> &rotation_vec);  // 旋转矢量转四元数
    static BaseMatrix RotationVec2RotationMat(
            const std::vector<double> &rotation_vec);  // 旋转矢量转旋转矩阵
    static std::vector<double> RotationMat2RotationVec(
            const BaseMatrix &rotation_mat);  // 旋转矩阵转旋转矢量
    
    static std::vector<double> CalcGe(const std::vector<double> &blh);  // e系下的重力加速度矢量计算
    static std::vector<double> CalcGn(const std::vector<double> &blh);  // n系吓得重力加速度计算
};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_MATH_H
