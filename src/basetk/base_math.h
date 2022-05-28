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
 * @brief   数学方法类, 定义了各种数学函数, 包括坐标转换, 四元数, 姿态转换等
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/28    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseMath
{
public:
    /**@brief       地心地固坐标加法
     * @param[in]   add_xyz1        加数1
     * @param[in]   add_xyz2        加数2
     * @return      两个坐标的和
     */
    static std::vector<double> XyzAdd(const std::vector<double> &add_xyz1,
                                      const std::vector<double> &add_xyz2);
    
    /**@brief       地心地固坐标减法
     * @param[in]   minuend         待减坐标
     * @param[in]   subtrahend      被减坐标
     * @return      待减坐标与被减坐标的差
     */
    static std::vector<double> XyzSub(const std::vector<double> &minuend,
                                      const std::vector<double> &subtrahend);
    
    /**@brief       大地坐标转地心地固坐标
     * @param[in]    blh         大地坐标
     * @param[in]    coor_sys    大地坐标的坐标系统(WGS84、CGCS2000等)
     * @return       大地坐标对应的地心地固坐标
     */
    static std::vector<double> Blh2Xyz(const std::vector<double> &blh,
                                       const CoorSys &coor_sys);
    
    /**@brief       地心地固坐标转大地坐标
     * @param[in]   xyz             地心地固坐标
     * @param[in]   coor_sys        大地坐标
     * @return      地心地固坐标对应的大地坐标
     */
    static std::vector<double> Xyz2Blh(const std::vector<double> &xyz,
                                       const CoorSys &coor_sys);
    
    /**@brief       度分秒转弧度
     * @param[in]   deg     度
     * @param[in]   min     分
     * @param[in]   sec     秒
     * @return      度分秒对应的弧度
     */
    static double Deg2Rad(const double &deg,
                          const double &min, const double &sec);
    
    /**@brief       计算测站在参考坐标下的ENU坐标(误差)
     * @param[in]   ref_xyz     参考坐标
     * @param[in]   coor_sys    测站的地心地固坐标
     * @return      测站在参考坐标下的ENU坐标\n
     * - 如果参考坐标是测站的真实坐标, 则返回的ENU坐标可考虑为测量ENU三向误差\n
     * - 如果参考坐标是参考站的坐标, 则返回的ENU坐标可认为是基线向量\n
     * - 上面几行可以写在定义里的注释, 不要写在这里, 太过臃肿
     */
    static std::vector<double> CalcEnu(const std::vector<double> &ref_xyz,
                                       const std::vector<double> &station_xyz);
    
    /**@brief       四元数乘法
     * @param[in]   quaternion1     待乘的四元数1
     * @param[in]   quaternion2     待乘的四元数2
     * @return      两个四元数相乘的结果, 也是四元数
     */
    static std::vector<double>
    QuaternionMul(const std::vector<double> &quaternion1,
                  const std::vector<double> &quaternion2);
    
    /**@brief       向量取模
     * @param[in]   vector     待取模的向量
     * @return      向量的模长
     */
    static double Norm(const std::vector<double> &vector);
    
    /**@brief       四元数归一化
     * @param[in]   vector     待取模的向量
     * @return      向量的模长
     */
    static void Normalize(std::vector<double> &vector);

    // 欧拉角排列顺序roll, pitch, yaw
    // 旋转顺序R(yaw, pitch, roll)
    
    /**@brief       欧拉角转方向余弦矩阵
     * @param[in]   euler     欧拉角
     * @return      方向余弦矩阵
     */
    static BaseMatrix
    Euler2Rotation(const std::vector<double> &euler);
    
    /**@brief       方向余弦矩阵转欧拉角
     * @param[in]   rotation     方向余弦矩阵
     * @return      欧拉角
     */
    static std::vector<double>
    Rotation2Euler(const BaseMath &rotation);
    
    /**@brief       欧拉角转四元数
     * @param[in]   euler     欧拉角
     * @return      四元数
     */
    static std::vector<double>
    Euler2Quaternion(const std::vector<double> &euler);  // 欧拉角转四元数
    
    /**@brief       四元数转欧拉角
     * @param[in]   quaternion     四元数
     * @return      欧拉角
     */
    static std::vector<double>
    Quaternion2Euler(const std::vector<double> &quaternion);  // 四元数转欧拉角
    
    /**@brief       四元数转方向余弦矩阵
     * @param[in]   quaternion     四元数
     * @return      方向余弦矩阵
     */
    static BaseMatrix
    Quaternion2Rotation(const std::vector<double> &quaternion);  // 四元数转方向余弦矩阵
    
    /**@brief       方向余弦矩阵转四元数
     * @param[in]   rotation     方向余弦矩阵
     * @return      四元数
     */
    static std::vector<double>
    Rotation2Quaternion(const BaseMatrix &rotation);  // 方向余弦矩阵转四元数
    
    
    /**@brief       e系下的重力加速度向量计算
     * @param[in]   blh         大地坐标
     * @return      e系下的重力加速度向量
     */
    static std::vector<double> get_ge(const std::vector<double> &blh);
    
    /**@brief       e系下的重力加速度向量计算
     * @param[in]   b       大地纬度
     * @param[in]   l       大地经度
     * @param[in]   h       大地高
     * @return      e系下的重力加速度向量
     */
    static std::vector<double> get_ge(const double &b,
                                      const double &l, const double &h);
    
};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_MATH_H
