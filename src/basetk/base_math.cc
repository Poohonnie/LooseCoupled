/**@file    base_math.h
 * @brief   数学方法类.cc文件
 * @details 实现了坐标转换, 四元数相关运算, 姿态转换等常见数学函数
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/2
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/5     <td>1.0      <td>Zing Fong  <td>Initialize
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
#include "base_matrix.h"

/**@brief       求最大值
 * @param[in]   list        待求最大值的列表
 * @return      列表中的最大值
 * @author      Zing Fong
 * @date        2022/6/5
 */
double BaseMath::max(std::initializer_list<double> list)
{
    double max_num{};
    for(const auto &a_list: list)
        max_num = max_num > a_list ? max_num : a_list;
    return max_num;
}

/**@brief       求最小值
 * @param[in]   list        待求最小值的列表
 * @return      列表中的最小值
 * @author      Zing Fong
 * @date        2022/6/5
 */
double BaseMath::min(std::initializer_list<double> list)
{
    double min_num{};
    for(const auto &a_list: list)
        min_num = min_num < a_list ? min_num : a_list;
    return min_num;
}

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
                                      const CoorSys coor_sys)
{
    std::vector<double> xyz(3, 0.0);
    double B = blh[0], L = blh[1], H = blh[2];
    double N =
            coor_sys.kA/sqrt(1 - coor_sys.kESquare*sin(B)*sin(B));  // 卯酉圈曲率半径
    //PPT 1-4 20页 公式
    xyz[0] = (N + H)*cos(B)*cos(L);
    xyz[0] = (N + H)*cos(B)*sin(L);
    xyz[0] = (N*(1 - coor_sys.kESquare) + H)*sin(B);
    
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
                                      const CoorSys coor_sys)
{
    std::vector<double> blh(3, 0.0);
    if(sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]) < 1e+6)
        // 不在地球表面，认定为异常数值
        return blh;
    double x2 = xyz[0]*xyz[0];
    double y2 = xyz[1]*xyz[1];
    double z2 = xyz[2]*xyz[2];  // 把各值平方求出来方便后面计算
    
    double deltaZ = 0;  // 赋初值
    double deltaZ1 = coor_sys.kESquare*xyz[2];  // deltaZ n+1
    double sinB = ((xyz[2] + deltaZ1) + 1e-6)/(1e-6 +
                    sqrt(x2 + y2 + (xyz[2] + deltaZ1)*(xyz[2] + deltaZ1)));
    double N = coor_sys.kA/sqrt(1 - coor_sys.kESquare*sinB*sinB);
    
    blh[1] = atan2(xyz[1], xyz[0]);  // 注意L取值范围，这里要用atan2
    
    for(int i = 0; i < 12 && fabs(deltaZ - deltaZ1) > 1e-10; ++i)
    {
        deltaZ = deltaZ1;
        sinB = (xyz[2] + deltaZ)/
               sqrt(x2 + y2 + (xyz[2] + deltaZ)*(xyz[2] + deltaZ));
        N = coor_sys.kA/sqrt(1 - coor_sys.kESquare*sinB*sinB);
        deltaZ1 = N*coor_sys.kESquare*sinB;
        
    }  // 迭代求ΔZ
    blh[0] = atan2(xyz[2] + deltaZ, sqrt(x2 + y2));
    blh[2] = sqrt(x2 + y2 + (xyz[2] + deltaZ)*(xyz[2] + deltaZ)) - N;
    
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
    if(deg > 0 && min > 0 && sec > 0)
        rad = (deg + min/60.0 + sec/3600.0)*BaseSdc::kD2R;
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
        // 输入格式错误
        printf("CalcEnu error!");
    std::vector<double> ref_blh = Xyz2Blh(ref_xyz, BaseSdc::wgs84);
    std::vector<double> d_xyz(3, 0.0);
    d_xyz[0] = station_xyz[0] - ref_xyz[0];
    d_xyz[1] = station_xyz[1] - ref_xyz[1];
    d_xyz[2] = station_xyz[2] - ref_xyz[2];
    
    d_enu[0] = -sin(ref_blh[1])*d_xyz[0] + cos(ref_blh[1])*d_xyz[1];
    d_enu[1] = -sin(ref_blh[0])*cos(ref_blh[1])*d_xyz[0] -
               sin(ref_blh[0])*sin(ref_blh[1])*d_xyz[1]
               + cos(ref_blh[0])*d_xyz[2];
    d_enu[2] = cos(ref_blh[0])*cos(ref_blh[1])*d_xyz[0] +
               cos(ref_blh[0])*sin(ref_blh[1])*d_xyz[1]
               + sin(ref_blh[0])*d_xyz[2];
    return d_enu;
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
        printf("Quaternion multiplication error! q1 size: %d, q2 size: %d",
               quaternion1.size(), quaternion2.size());
        return result;
    }
    
    // 起别名, 方便书写
    const std::vector<double> &p = quaternion1;
    const std::vector<double> &q = quaternion2;
    
    result[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    result[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    result[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    result[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    
    if(result[0] < 0)
    {
        // 四元数实部为负
        for(auto a_result: result)
            a_result = -a_result;
    }
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
    for(const auto a_vector: vector)
        norm += a_vector*a_vector;  // 平方和
    norm = sqrt(norm);
    
    return norm;
}

/**@brief       向量归一化
 * @param[in]   vector          待归一化向量
 * @author      Zing Fong
 * @date        2022/6/5
 */
void BaseMath::Normalize(std::vector<double> &vector)
{
    double norm = Norm(vector);;  // 取模
    for(auto a_vec: vector)
        a_vec /= norm;
}

/**@brief       四元数归一化, 同时保证实部非负
 * @param[in]   quaternion          待归一化四元数
 * @author      Zing Fong
 * @date        2022/6/5
 */
void BaseMath::QuaternionNormalize(std::vector<double> &quaternion)
{
    if(quaternion[0] < 0)
    {
        // 四元数实部为负
        for(auto a_q: quaternion)
            a_q = -a_q;
    }
    Normalize(quaternion);
}

/**@brief       欧拉角转旋转矩阵
 * @details     欧拉角排列顺序roll, pitch, yaw. 旋转顺序R(yaw, pitch, roll), 即ZYX\n
 * - 旋转矩阵为C_b_R, 即b系相对于R系转动的角度(或者说R系转动到b系的旋转矩阵)
 * @param[in]   euler          待转换欧拉角
 * @return      旋转矩阵
 * @author      Zing Fong
 * @date        2022/6/5
 */
BaseMatrix BaseMath::Euler2RotationMat(const std::vector<double> &euler)
{
    if(euler.size() != 3)
    {
        // 传入参数错误
        printf("Euler2RotationMat error.\n");
        return BaseMatrix::eye(3);
    }
    const double roll = euler[0];
    const double pitch = euler[1];
    const double yaw = euler[2];
    // PPT上公式用的字母, 这样方便书写和检查
    const double &psi = yaw;
    const double &theta = pitch;
    const double &phi = roll;
    
    BaseMatrix rotation(3, 3);
    rotation.write(0, 0, cos(theta)*cos(psi));
    rotation.write(0, 1, -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi));
    rotation.write(0, 2, sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
    
    rotation.write(1, 0, cos(theta)*sin(psi));
    rotation.write(1, 1, cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi));
    rotation.write(1, 2, -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));
    
    rotation.write(2, 0, -sin(theta));
    rotation.write(2, 1, sin(phi)*cos(theta));
    rotation.write(2, 2, cos(phi)*cos(theta));
    
    return rotation;
}

/**@brief       旋转矩阵转欧拉角
 * @details     欧拉角排列顺序roll, pitch, yaw. 旋转顺序R(yaw, pitch, roll), 即ZYX\n
 * - 旋转矩阵为C_b_R, 即b系相对于R系转动的角度(或者说R系转动到b系的旋转矩阵)\n
 * @note        俯仰角pitch在π/2附近时无法解出结果, 所以尽量 !避免! 使用此函数
 * @param[in]   rotation_mat          待转换旋转矩阵
 * @return      欧拉角, 以roll, pitch, yaw的顺序存储的vector
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::RotationMat2Euler(const BaseMatrix &rotation_mat)
{
    if(rotation_mat.get_col_num() != 3 || rotation_mat.get_row_num() != 3)
    {
        // 传入参数错误
        printf("RotationMat2Euler error. rotation_mat size: %d×%d\n",
               rotation_mat.get_row_num(), rotation_mat.get_col_num());
        return std::vector<double>(3, 0.0);
    }
    std::vector<double> euler(3, 0.0);
    const BaseMatrix &R = rotation_mat;  // PPT上公式所用字母, 方便书写与检查
    euler[1] = atan2(-R.read(2, 0),
                     sqrt(R.read(2, 1)*R.read(2, 1) +
                          R.read(2, 2)*R.read(2, 2)));
    euler[0] = atan2(R.read(2, 1), R.read(2, 2));
    euler[2] = atan2(R.read(1, 0), R.read(0, 0));
    return euler;
}

/**@brief       欧拉角转四元数
 * @details     欧拉角排列顺序roll, pitch, yaw. 旋转顺序R(yaw, pitch, roll), 即ZYX
 * @param[in]   euler          欧拉角
 * @return      欧拉角转化得到的四元数
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::Euler2Quaternion(const std::vector<double> &euler)
{
    if(euler.size() != 3)
    {
        // 传入参数错误
        printf("Euler2Quaternion error.\n");
        return std::vector<double>(4, 0.0);
    }
    const double &phi = euler[0];
    const double &theta = euler[1];
    const double &psi = euler[2];
    
    std::vector<double> q(4, 0.0);  // b系相对于R系的姿态四元数
    q[0] = cos(phi/2)*cos(theta/2)*cos(psi/2)
           + sin(phi/2)*sin(theta/2)*sin(psi/2);
    q[1] = sin(phi/2)*cos(theta/2)*cos(psi/2)
           - cos(phi/2)*sin(theta/2)*sin(psi/2);
    q[2] = cos(phi/2)*sin(theta/2)*cos(psi/2)
           + sin(phi/2)*cos(theta/2)*sin(psi/2);
    q[3] = cos(phi/2)*cos(theta/2)*sin(psi/2)
           - sin(phi/2)*sin(theta/2)*cos(psi/2);
    QuaternionNormalize(q);  // 标准化
    return q;
}

/**@brief       四元数转欧拉角
 * @param[in]   quaternion          待转换四元数
 * @details     欧拉角排列顺序roll, pitch, yaw
 * @return      四元数转换得到的欧拉角
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::Quaternion2Euler(
        const std::vector<double> &quaternion)
{
    if(quaternion.size() != 4)
    {
        // 传入参数错误
        printf("Quaternion2Euler error.\n");
        return std::vector<double>(3, 0.0);
    }
    const std::vector<double> &q = quaternion;  // 四元数, 别名
    std::vector<double> euler(3, 0.0);  // 欧拉角
    euler[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
    euler[1] = asin(2*(q[0]*q[2] - q[3]*q[1]));
    euler[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
    
    return euler;
}

/**@brief       四元数转方向余弦矩阵
 * @param[in]   quaternion          待转换四元数
 * @return      四元数转换得到的方向余弦矩阵
 * @author      Zing Fong
 * @date        2022/6/5
 */
BaseMatrix BaseMath::Quaternion2RotationMat(
        const std::vector<double> &quaternion)
{
    if(quaternion.size() != 4)
    {
        // 传入参数错误
        printf("Quaternion2RotationMat error.\n");
        return BaseMatrix(3, 3);
    }
    BaseMatrix C_b_R(3, 3);
    const std::vector<double> &q = quaternion;
    // 方便书写
    double q1q1 = q[0]*q[0], q2q2 = q[1]*q[1],
            q3q3 = q[2]*q[2], q4q4 = q[3]*q[3];
    double q1q2 = q[0]*q[1], q1q3 = q[0]*q[2], q1q4 = q[0]*q[3];
    double q2q3 = q[1]*q[2], q2q4 = q[1]*q[3];
    double q3q4 = q[2]*q[3];
    // 为方向余弦矩阵各项赋值
    C_b_R.write(0, 0, q1q1 + q2q2 - q3q3 - q4q4);
    C_b_R.write(0, 1, 2*(q2q3 - q1q4));
    C_b_R.write(0, 2, 2*(q2q4 + q1q3));
    
    C_b_R.write(1, 0, 2*(q2q3 + q1q4));
    C_b_R.write(1, 1, q1q1 - q2q2 + q3q3 - q4q4);
    C_b_R.write(1, 2, 2*(q3q4 - q1q2));
    
    C_b_R.write(2, 0, 2*(q2q4 - q1q3));
    C_b_R.write(2, 1, 2*(q3q4 + q1q2));
    C_b_R.write(2, 2, q1q1 - q2q2 - q3q3 + q4q4);
    
    return C_b_R;
}

/**@brief       方向余弦矩阵转四元数
 * @param[in]   rotation_mat          待转换的方向余弦矩阵
 * @return      转换得到的四元数
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::RotationMat2Quaternion(
        const BaseMatrix &rotation_mat)
{
    if(rotation_mat.get_row_num() != 3 || rotation_mat.get_col_num() != 3)
    {
        printf("RotationMat2Quaternion error.\n");
        return std::vector<double>(4, 0.0);
    }
    const BaseMatrix &c = rotation_mat;
    double p1 = 1 + c.Trace();
    double p2 = 1 + 2*c.read(0, 0) - c.Trace();
    double p3 = 1 + 2*c.read(1, 1) - c.Trace();
    double p4 = 1 + 2*c.read(2, 2) - c.Trace();
    double q1{}, q2{}, q3{}, q4{};  // 四元数
    if(p1 == max({p1, p2, p3, p4}))
    {
        q1 = 0.5*sqrt(p1);
        q2 = (c.read(2, 1) - c.read(1, 2))/(4*q1);
        q3 = (c.read(0, 2) - c.read(2, 0))/(4*q1);
        q4 = (c.read(1, 0) - c.read(0, 1))/(4*q1);
    }
    else if(p2 == max({p1, p2, p3, p4}))
    {
        q2 = 0.5*sqrt(p2);
        q3 = (c.read(1, 0) + c.read(0, 1))/(4*q2);
        q4 = (c.read(0, 2) + c.read(2, 0))/(4*q2);
        q1 = (c.read(2, 1) - c.read(1, 2))/(4*q2);
    }
    else if(p3 == max({p1, p2, p3, p4}))
    {
        q3 = 0.5*sqrt(p3);
        q4 = (c.read(2, 1) + c.read(1, 2))/(4*q3);
        q1 = (c.read(0, 2) - c.read(2, 0))/(4*q3);
        q2 = (c.read(0, 1) + c.read(1, 0))/(4*q3);
    }
    else if(p4 == max({p1, p2, p3, p4}))
    {
        q4 = 0.5*sqrt(p4);
        q1 = (c.read(1, 0) - c.read(0, 1))/(4*q4);
        q2 = (c.read(0, 2) + c.read(2, 0))/(4*q4);
        q3 = (c.read(2, 1) + c.read(1, 2))/(4*q4);
    }
    std::vector<double> result{q1, q2, q3, q4};
    QuaternionNormalize(result);
    return result;
}

/**@brief       四元数转旋转矢量
 * @param[in]   quaternion          待转换的四元数
 * @return      转换得到的旋转矢量
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::Quaternion2RotationVec(
        const std::vector<double> &quaternion)
{
    if(quaternion.size() != 4)
    {
        // 传入参数错误
        printf("Quaternion2RotationVec error.\n");
        return std::vector<double>(3, 0.0);
    }
    const std::vector<double> &q = quaternion;
    const double &q1 = q[0], &q2 = q[1], &q3 = q[2], &q4 = q[3];  // 方便书写
    double half_phi_norm = sqrt(q2*q2 + q3*q3 + q4*q4)/q1;  // 模长的一半
    double f = sin(half_phi_norm)/(2*half_phi_norm);
    double pi = BaseSdc::kPi;  // 方便书写
    if(q1 == 0)
        return std::vector<double>{q2/pi, q3/pi, q4/pi};
    else
        return std::vector<double>{q2/f, q3/f, q4/f};
}

/**@brief       旋转矢量转四元数
 * @param[in]   rotation_vec          待转换的旋转矢量
 * @return      转换得到的四元数
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::RotationVec2Quaternion(
        const std::vector<double> &rotation_vec)
{
    if(rotation_vec.size() != 3)
    {
        // 传入参数错误
        printf("RotationVec2Quaternion error.\n");
        return std::vector<double>(4, 0.0);
    }
    const std::vector<double> &phi = rotation_vec;  // 方便书写
    std::vector<double> q(4, 0.0);  // 返回的结果
    double half_phi_norm = 0.5*Norm(phi);
    double cos_half_phi_norm = cos(half_phi_norm);
    double f = sin(half_phi_norm)/(2*half_phi_norm);  // 注意这里把书上公式里的0.5约掉了
    q[0] = cos_half_phi_norm;
    q[1] = f*phi[0];
    q[2] = f*phi[1];
    q[3] = f*phi[2];
    QuaternionNormalize(q);
    return q;
}

/**@brief       旋转矢量转方向余弦矩阵
 * @param[in]   rotation_vec          待转换的旋转矢量
 * @return      转换得到的方向余弦矩阵
 * @author      Zing Fong
 * @date        2022/6/5
 */
BaseMatrix BaseMath::RotationVec2RotationMat(
        const std::vector<double> &rotation_vec)
{
    if(rotation_vec.size() != 3)
    {
        // 传入参数错误
        printf("RotationVec2RotationMat error.\n");
        return BaseMatrix(3, 3);
    }
    const std::vector<double> &phi = rotation_vec;
    BaseMatrix antisymmetric_mat =
            BaseMatrix::CalcAntisymmetryMat(phi);  // 反对称矩阵
    double norm = Norm(phi);  // 模长
    double scalar1 = sin(norm)/norm;
    double scalar2 = (1 - cos(norm))/(norm*norm);
    BaseMatrix C_b_R = BaseMatrix::eye(3) + antisymmetric_mat*scalar1
                       + antisymmetric_mat*antisymmetric_mat*scalar2;
    return C_b_R;
}

/**@brief       方向余弦矩阵转旋转矢量
 * @param[in]   rotation_mat          待转换的方向余弦矩阵
 * @return      转换得到的旋转矢量
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::RotationMat2RotationVec(
        const BaseMatrix &rotation_mat)
{
    if(rotation_mat.get_row_num() != 3 || rotation_mat.get_col_num() != 3)
    {
        printf("RotationMat2RotationVec error.\n");
        return std::vector<double>(3, 0.0);
    }
    // 这里没有直接实现的路径, 只能以四元数为媒介
    std::vector<double> q = RotationMat2Quaternion(rotation_mat);
    return Quaternion2RotationVec(q);
}

/**@brief       e系下的重力加速度矢量计算
 * @param[in]   blh          已知大地坐标BLH
 * @return      e系下的重力加速度矢量
 * @author      Zing Fong
 * @date        2022/6/5
 */
std::vector<double> BaseMath::Calc_ge(const std::vector<double> &blh)
{
    if(blh.size() != 3)
    {
        printf("Calc_ge error.\n");
        return std::vector<double>(3, 0.0);
    }
    double g0 = 9.7803267715;
    double a1 = 0.0052790414;
    double a2 = 0.0000232718;
    double b1 = -3.087691891e-6;
    double b2 = 4.3977311e-10;
    double b3 = 7.211e-13;
    double b = blh[0], l = blh[1], h = blh[2];
    // 由已知位置BLH计算当地重力加速度g
    double sin_b_2 = sin(b)*sin(b);
    double sin_b_4 = sin_b_2*sin_b_2;
    double g = g0*(1 + a1*sin_b_2 + a2*sin_b_4) + (b1 + b2*sin_b_2)*h + b3*h*h;
    // 计算e系下的重力加速度矢量
    std::vector<double> ge(3, 0.0);
    ge[0] = cos(l)*cos(b)*(-g);
    ge[1] = sin(l)*cos(b)*(-g);
    ge[2] = sin(b)*(-g);
    return ge;
}

