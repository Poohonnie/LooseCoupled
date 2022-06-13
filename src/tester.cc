/**@file    tester.cc
 * @brief   测试器
 * @details 实现了项目中各类对应的测试类
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/10
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/10    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "tester.h"
// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <cmath>

// 本项目内 .h 文件

/**@brief       最大最小值测试器
 * @author      Zing Fong
 * @date        2022/6/10
 */
void BaseMathTester::MaxAndMinTester()
{
    // 随机数
    static std::default_random_engine e(time(nullptr));
    static std::uniform_real_distribution<double> u(-20, 20);
    std::vector<double> vec(10, 0.0);
    
    printf("double list:");
    for(auto &a_vec: vec)
    {
        a_vec = u(e);
        printf(" %5.2f", a_vec);
    }
    printf("\n");
    printf("max num: %5.2f, min num: %5.2f\n",
           BaseMath::max(vec), BaseMath::min(vec));
}

/**@brief       坐标转换测试器
 * @author      Zing Fong
 * @date        2022/6/10
 */
void BaseMathTester::CoordinateTransformationTester()
{
    // 随机数
    static std::default_random_engine e(time(nullptr));
    static std::uniform_real_distribution<double> u_b(0, 90);
    static std::uniform_real_distribution<double> u_l(-180, 180);
    static std::uniform_real_distribution<double> u_h(0, 100);
    
    std::vector<double> blh = {u_b(e)*BaseSdc::kD2R, u_l(e)*BaseSdc::kD2R,
                               u_h(e)};  // 初始大地坐标
    std::vector<double> xyz = BaseMath::Blh2Xyz(blh,
                                                BaseSdc::wgs84);  // 初始地心地固坐标
    printf("initial:\n blh: %12.8f %12.8f %12.8f\n xyz: %18.8f %18.8f %18.8f\n",
           blh[0]*BaseSdc::kR2D, blh[1]*BaseSdc::kR2D, blh[2]*BaseSdc::kR2D,
           xyz[0], xyz[1], xyz[2]);
    std::vector<double> blh_final(blh);  // 最终大地坐标
    std::vector<double> xyz_final(xyz);  // 最终地心地固坐标
    for(int i = 0; i < 1000; ++i)
    {
        xyz_final = BaseMath::Blh2Xyz(blh_final, BaseSdc::wgs84);
        blh_final = BaseMath::Xyz2Blh(xyz_final, BaseSdc::wgs84);
    }
    printf("result:\n blh: %12.8f %12.8f %12.8f\n xyz: %18.8f %18.8f %18.8f\n",
           blh_final[0]*BaseSdc::kR2D, blh_final[1]*BaseSdc::kR2D,
           blh_final[2]*BaseSdc::kR2D,
           xyz_final[0], xyz_final[1], xyz_final[2]);
    printf("error:\n blh: %12.8f %12.8f %12.8f\n xyz: %18.8f %18.8f %18.8f\n",
           blh[0] - blh_final[0], blh[1] - blh_final[1], blh[2] - blh_final[2],
           xyz[0] - xyz_final[0], xyz[1] - xyz_final[1], xyz[2] - xyz_final[2]);
}

/**@brief       姿态转换测试器
 * @author      Zing Fong
 * @date        2022/6/10
 */
void BaseMathTester::AttitudeTransformationTester()
{
    // 随机数
    static std::default_random_engine e(time(nullptr));
    static std::uniform_real_distribution<double> u(0, 90);
    std::vector<double> euler = {u(e)*BaseSdc::kD2R, u(e)*BaseSdc::kD2R,
                                 u(e)*BaseSdc::kD2R};  // 欧拉角
    printf("initial euler angle: %12.8f %12.8f %12.8f\n",
           euler[0]*BaseSdc::kR2D, euler[1]*BaseSdc::kR2D,
           euler[2]*BaseSdc::kR2D);  // 先打印出初始欧拉角
    std::vector<double> euler_result = euler;  // 最后转换结果
    for(int i = 0; i < 10000; ++i)
    {
        // 正向转换
        BaseMatrix rotation_mat = BaseMath::Euler2RotationMat(
                euler_result);  // 方向余弦矩阵
        std::vector<double> q = BaseMath::RotationMat2Quaternion(
                rotation_mat);  // 四元数
        std::vector<double> euler_tmp = BaseMath::Quaternion2Euler(q);  // 欧拉角
        q = BaseMath::Euler2Quaternion(euler_tmp);  // 欧拉角转四元数
        std::vector<double> rotation_vec = BaseMath::Quaternion2RotationVec(
                q);  // 旋转矢量

        // 逆向转换
        q = BaseMath::RotationVec2Quaternion(rotation_vec);
        rotation_mat = BaseMath::Quaternion2RotationMat(q);
        rotation_vec = BaseMath::RotationMat2RotationVec(rotation_mat);
        rotation_mat = BaseMath::RotationVec2RotationMat(rotation_vec);
        euler_result = BaseMath::RotationMat2Euler(rotation_mat);
    }
    printf("euler angle error: %12.8f %12.8f %12.8f\n",
           (euler[0] - euler_result[0])*BaseSdc::kR2D,
           (euler[1] - euler_result[1])*BaseSdc::kR2D,
           (euler[2] - euler_result[2])*BaseSdc::kR2D);
}
