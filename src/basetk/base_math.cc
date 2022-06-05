/**@file    base_math.h
 * @brief   数学方法类.cc文件
 * @details 实现了坐标转换, 四元数相关运算, 姿态转换等数学方法
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/2
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/2     <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "base_math.h"
// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <cmath>

// 本项目内 .h 文件
#include "base_sdc.h"
#include "base_matrix.h"

/**@brief       地心地固坐标加法
 * @param[in]   add_xyz1        加数1
 * @param[in]   add_xyz2        加数2
 * @return      两个地心地固坐标的和
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::XyzAdd(const std::vector<double> &add_xyz1,
                                     const std::vector<double> &add_xyz2)
{
    std::vector<double> result(3, 0.0);
    if(add_xyz1.size() == add_xyz2.size() && add_xyz1.size() == 3)
    {
        for(int i = 0; i < 3; ++i)
            result[i] = add_xyz1[i] + add_xyz2[i];
    }
    else // 输入格式不正确
        printf("XYZ addition error! xyz1 size: %d, xyz2 size: %d\n",
               add_xyz1.size(), add_xyz2.size());
    return result;
}

/**@brief       地心地固坐标减法
 * @param[in]   minuend             被减数
 * @param[in]   subtrahend          减数
 * @return      两个地心地固坐标之差
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::XyzSub(const std::vector<double> &minuend,
                                     const std::vector<double> &subtrahend)
{
    std::vector<double> result(3, 0.0);
    if(minuend.size() == subtrahend.size() && minuend.size() == 3)
    {
        for(int i = 0; i < 3; ++i)
            result[i] = minuend[i] - subtrahend[i];
    }
    else // 输入格式不正确
        printf("XYZ subtraction error! xyz1 size: %d, xyz2 size: %d\n",
               minuend.size(), subtrahend.size());
    return result;
}

/**@brief       大地坐标转地心地固坐标
 * @param[in]   blh             大地坐标
 * @param[in]   coor_sys        大地坐标参考系统(WGS84/CGCS2000)
 * @return      地心地固坐标
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::Blh2Xyz(const std::vector<double> &blh,
                                      const CoorSys &coor_sys)
{
    std::vector<double> xyz(3, 0.0);
    double B = blh[0], L = blh[1], H = blh[2];
    double N = coor_sys.kA / sqrt(1 - coor_sys.kESquare * sin(B) * sin(B));  // 卯酉圈曲率半径
    //PPT 1-4 20页 公式
    xyz[0] = (N + H) * cos(B) * cos(L);
    xyz[0] = (N + H) * cos(B) * sin(L);
    xyz[0] = (N * (1 - coor_sys.kESquare) + H) * sin(B);
    
    return xyz;
}

/**@brief       地心地固坐标转大地坐标
 * @param[in]   xyz             地心地固坐标
 * @param[in]   coor_sys        大地坐标参考系统(WGS84/CGCS2000)
 * @return      大地坐标
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::Xyz2Blh(const std::vector<double> &xyz,
                                      const CoorSys &coor_sys)
{
    std::vector<double> blh(3, 0.0);
    if (sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]) < 1e+6)
        // 不在地球表面，认定为异常数值
        return blh;
    double x2 = xyz[0] * xyz[0];
    double y2 = xyz[1] * xyz[1];
    double z2 = xyz[2] * xyz[2];  // 把各值平方求出来方便后面计算
    
    double deltaZ = 0;  // 赋初值
    double deltaZ1 = coor_sys.kESquare*xyz[2];  // deltaZ n+1
    double sinB = ((xyz[2] + deltaZ1) + 1e-6) / (1e-6 + sqrt(x2 + y2 + (xyz[2] + deltaZ1) * (xyz[2] + deltaZ1)));
    double N = coor_sys.kA/sqrt(1 - coor_sys.kESquare*sinB*sinB);
    
    blh[1] = atan2(xyz[1], xyz[0]);  // 注意L取值范围，这里要用atan2
    
    for (int i = 0; i < 12 && fabs(deltaZ - deltaZ1) > 1e-10; ++i)
    {
        deltaZ = deltaZ1;
        sinB = (xyz[2] + deltaZ) / sqrt(x2 + y2 + (xyz[2] + deltaZ) * (xyz[2] + deltaZ));
        N = coor_sys.kA/sqrt(1 - coor_sys.kESquare*sinB*sinB);
        deltaZ1 = N*coor_sys.kESquare*sinB;
        
    }  // 迭代求ΔZ
    blh[0] = atan2(xyz[2] + deltaZ, sqrt(x2 + y2));
    blh[2] = sqrt(x2 + y2 + (xyz[2] + deltaZ) * (xyz[2] + deltaZ)) - N;
    
    return blh;
}

/**@brief       度分秒转弧度
 * @param[in]   deg          度
 * @param[in]   min          分
 * @param[in]   sec          秒
 * @return      转换后的弧度
 * @author      Zing Fong
 * @date        2022/6/2
 */
double BaseMath::Deg2Rad(const int &deg, const int &min,
                         const double &sec)
{
    double rad{};
    if (deg > 0 && min > 0 && sec > 0)
        rad = (deg + min / 60.0 + sec / 3600.0)*BaseSdc::kD2R;
    else
        printf("Deg2Rad error! deg: %d, min: %d, sec: %5.2f\n", deg, min, sec);
    return rad;
}

/**@brief       计算ENU坐标
 * @param[in]   ref_xyz             参考点坐标
 * @param[in]   station_xyz         测站坐标
 * @return      给定测站坐标在参考点ENU系下的坐标
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::CalcEnu(const std::vector<double> &ref_xyz,
                                      const std::vector<double> &station_xyz)
{
    std::vector<double> d_enu(3, 0.0);
    
    if(ref_xyz.size() != 3 || station_xyz.size() != 3)
    {
        // 输入格式错误
        printf("CalcEnu error!");
        return d_enu;
    }
    std::vector<double> ref_blh = Xyz2Blh(ref_xyz, BaseSdc::wgs84);
    std::vector<double> d_xyz(3, 0.0);
    d_xyz[0] = station_xyz[0] - ref_xyz[0];
    d_xyz[1] = station_xyz[1] - ref_xyz[1];
    d_xyz[2] = station_xyz[2] - ref_xyz[2];
    
    d_enu[0] = -sin(ref_blh[1]) * d_xyz[0] + cos(ref_blh[1]) * d_xyz[1];
    d_enu[1] = -sin(ref_blh[0]) * cos(ref_blh[1]) * d_xyz[0] - sin(ref_blh[0]) * sin(ref_blh[1]) * d_xyz[1]
              + cos(ref_blh[0]) * d_xyz[2];
    d_enu[2] = cos(ref_blh[0]) * cos(ref_blh[1]) * d_xyz[0] + cos(ref_blh[0]) * sin(ref_blh[1]) * d_xyz[1]
              + sin(ref_blh[0]) * d_xyz[2];
}

/**@brief       四元数乘法
 * @param[in]   quaternion1             乘数1
 * @param[in]   quaternion1             乘数2
 * @return      两个四元数相乘的结果
 * @author      Zing Fong
 * @date        2022/6/2
 */
std::vector<double> BaseMath::QuaternionMul(
        const std::vector<double> &quaternion1,
        const std::vector<double> &quaternion2)
{
    std::vector<double> result(4, 0.0);
    
    if(quaternion1.size() != 4 || quaternion2.size() != 4)
    {
        // 输入格式错误
        printf("Quaternion multiplication error! q1 size: %d, q2 size: %d", quaternion1.size(), quaternion2.size());
        return result;
    }
    
    // 起别名, 方便书写
    const std::vector<double> &p = quaternion1;
    const std::vector<double> &q = quaternion2;
    
    result[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    result[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    result[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    result[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    
    return result;
}

/**@brief       向量取模
 * @param[in]   vector          待取模向量
 * @return      向量模长
 * @author      Zing Fong
 * @date        2022/6/2
 */
double BaseMath::Norm(const std::vector<double> &vector)
{
    double norm{};
    for(const auto a_vector : vector)
        norm += a_vector*a_vector;  // 平方和
    norm = sqrt(norm);
    
    return norm;
}

/**@brief       四元数归一化
 * @param[in]   vector          待取模向量
 * @return      向量模长
 * @author      Zing Fong
 * @date        2022/6/2
 */
void BaseMath::Normalize(std::vector<double> &vector)
{

}

BaseMatrix BaseMath::Euler2RotationMat(const std::vector<double> &euler)
{
    return BaseMatrix();
}

std::vector<double> BaseMath::RotationMat2Euler(const BaseMath &rotation_mat)
{
    return std::vector<double>();
}

std::vector<double> BaseMath::Euler2Quaternion(const std::vector<double> &euler)
{
    return std::vector<double>();
}

std::vector<double> BaseMath::Quaternion2Euler(
        const std::vector<double> &quaternion)
{
    return std::vector<double>();
}

BaseMatrix BaseMath::Quaternion2RotationMat(
        const std::vector<double> &quaternion)
{
    return BaseMatrix();
}

std::vector<double> BaseMath::RotationMat2Quaternion(
        const BaseMatrix &rotation_mat)
{
    return std::vector<double>();
}

std::vector<double> BaseMath::Quaternion2RotationVec(
        const std::vector<double> &quaternion)
{
    return std::vector<double>();
}

BaseMatrix BaseMath::RotationVec2RotationMat(
        const std::vector<double> &rotation_vec)
{
    return BaseMatrix();
}

std::vector<double> BaseMath::Calc_ge(const std::vector<double> &blh)
{
    return std::vector<double>();
}

std::vector<double> BaseMath::Calc_ge(const double &b, const double &l,
                                      const double &h)
{
    return std::vector<double>();
}
