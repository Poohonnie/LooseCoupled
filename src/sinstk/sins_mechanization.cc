/**@file    sins_mechanization.cc
 * @brief   惯导机械编排
 * @details 读取imu文件, 进行纯惯导的机械编排
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/31
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/31    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "sins_mechanization.h"
// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <cmath>

// 本项目内 .h 文件

//constexpr int


/**@brief       状态初始化
 * @param[in]   initial_state          载体的初始状态量
 * @author      Zing Fong
 * @date        2022/6/10
 */
void SinsMechanization::Init(const StateInfo &initial_state)
{
    cur_state_ = initial_state;  // 初始状态
    ksub1_state_ = initial_state;
    t_ = cur_state_.time;  // 时间
    
}

/**@brief       机械编排准备
 * @param[in]   imu_data          当前历元imu读数
 * @return      返回结果\n
 * -  0         说明不是第一个历元, 可以进行机械编排
 * -  -114514   说明是第一个历元, 不进行机械编排
 * @author      Zing Fong
 * @date        2022/6/11
 */
int SinsMechanization::PrepareUpdate(const ImuData &imu_data)
{
    ++cur_epoch_;  // 历元数+1
    // 上一历元数据前移
    ksub2_state_ = ksub1_state_;
    ksub1_state_ = cur_state_;
    cur_state_ = {};
    
    ksub1_imu_data_ = cur_imu_data_;
    cur_imu_data_ = imu_data;
    
    r_m_ksub1_ = r_m_;
    r_m_ = {};
    r_n_ksub1_ = r_n_;
    r_n_ = {};
    
    g_n_ksub2_ = g_n_ksub1_;
    g_n_ksub1_ = g_n_;
    g_n_ = {};
    
    omega_en_n_ksub2_ = omega_en_n_ksub1_;
    omega_en_n_ksub1_ = omega_en_n_;
    omega_en_n_ = {};
    omega_ie_n_ksub2_ = omega_ie_n_ksub1_;
    omega_ie_n_ksub1_ = omega_ie_n_;
    omega_ie_n_ = {};
    
    // 引用参数
    const double &a = BaseSdc::wgs84.kA;
    const double &e_2 = BaseSdc::wgs84.kESquare;
    const double &omega_e = BaseSdc::wgs84.kOmega;
    const double &phi = cur_state_.blh[0];  // 纬度
    const double &h = cur_state_.blh[2];  // 高程
    const double &v_e = cur_state_.v_ned[1];  // 东向速度  2022/6/14更改为NED
    const double &v_n = cur_state_.v_ned[0];  // 北向速度
    
    // 求子午圈半径和卯酉圈半径, 以及两个不知道啥名的omega, 一个可能是地球自转角速度
    r_m_ = a*(1 - e_2)/
           sqrt(pow(1 - e_2*sin(phi)*sin(phi), 3));
    r_n_ = a/sqrt(1 - e_2*sin(phi)*sin(phi));
    
    omega_ie_n_[0] = omega_e*cos(phi);
    omega_ie_n_[1] = 0;
    omega_ie_n_[2] = -omega_e*sin(phi);
    
    omega_en_n_[0] = v_e/(r_n_ + h);
    omega_en_n_[1] = -v_n/(r_m_ + h);
    omega_en_n_[2] = -v_e*tan(phi)/(r_n_ + h);
    
    g_n_ = BaseMath::CalcGn(cur_state_.blh);  // 计算e系下的重力加速度
    
    // 时间
    cur_state_.time = t_ = imu_data.t;
    delta_t_ = cur_state_.time - ksub1_state_.time;  // 当前历元与上一历元时间间隔
    
    if(cur_epoch_ == 1)
    {
        // 说明是第一个历元, 前两个历元数据与当前历元统一
        ksub2_state_ = ksub1_state_ = cur_state_;
        ksub1_imu_data_ = cur_imu_data_;
        r_m_ksub1_ = r_m_;
        r_n_ksub1_ = r_n_;
        g_n_ksub2_ = g_n_ksub1_ = g_n_;
        omega_en_n_ksub2_ = omega_en_n_ksub1_ = omega_en_n_;
        omega_ie_n_ksub2_ = omega_ie_n_ksub1_ = omega_ie_n_;
        delta_t_ = 0;
        return -114514;
    }
    
    return 0;
}

/**@brief       姿态更新
 * @details
 * - 已知: 上一历元姿态、上一历元和当前历元陀螺输出角增量\n
 * - 待求: 当前历元姿态
 * @author      Zing Fong
 * @date        2022/6/11
 */
void SinsMechanization::AttitudeUpdate()
{
    // 求b系变化的等效旋转矢量
    auto delta_theta_k = cur_imu_data_.gyro;  // 当前历元陀螺输出
    auto delta_theta_ksub1 = ksub1_imu_data_.gyro;  // 上一历元陀螺输出
    auto cross_product = BaseMatrix::CrossProduct(
            delta_theta_ksub1, delta_theta_k);  // 陀螺读数的外积
    for(auto &a_product: cross_product)
        a_product *= 1.0/12;  // 乘系数
    auto phi_k = BaseMatrix::VectorAdd(delta_theta_k,
                                       cross_product);  // b系变化的等效旋转矢量
//    double norm_phi_k = BaseMath::Norm(phi_k);  // φk的模
//    double f1 = sin(0.5*norm_phi_k)/norm_phi_k;  // 系数
    
    // 求b系姿态变化四元数
    auto q_bk_bksub1 = BaseMath::RotationVec2Quaternion(phi_k);
//    std::vector<double> q_bk_bksub1(4, 0.0);
//    q_bk_bksub1[0] = cos(0.5*norm_phi_k);
//    q_bk_bksub1[1] = f1*phi_k[0];
//    q_bk_bksub1[2] = f1*phi_k[1];
//    q_bk_bksub1[3] = f1*phi_k[2];
    
    // 求n系变化对应的等效旋转矢量
    auto zeta_k = BaseMatrix::VectorAdd(omega_ie_n_, omega_en_n_);
    for(auto &a_zeta_k: zeta_k)
        a_zeta_k *= delta_t_;  // 乘时间间隔
//    double norm_zeta_k = BaseMath::Norm(zeta_k);  // ζk的模
//    double f2 = -sin(0.5*norm_zeta_k)/norm_zeta_k;  // 系数
    
    //求n系姿态变化四元数
    auto q_nksub1_nk = BaseMath::RotationVec2Quaternion(zeta_k);
    q_nksub1_nk[1] *= -1;  // 这里是负的, 也不知道为啥
    q_nksub1_nk[2] *= -1;
    q_nksub1_nk[3] *= -1;
//    std::vector<double> q_nksub1_nk(4, 0.0);
//    q_nksub1_nk[0] = cos(0.5*norm_zeta_k);
//    q_nksub1_nk[1] = f2*zeta_k[0];
//    q_nksub1_nk[2] = f2*zeta_k[1];
//    q_nksub1_nk[3] = f2*zeta_k[2];
    
    // 姿态更新的递推
    auto tmp = BaseMath::QuaternionMul(q_nksub1_nk,
                                       ksub1_state_.q);
    cur_state_.q = BaseMath::QuaternionMul(tmp, q_bk_bksub1);
    cur_state_.c_b_n = BaseMath::Quaternion2RotationMat(
            cur_state_.q);  // 方向余弦矩阵
}

/**@brief       速度更新
 * @details
 * - 已知: 上一历元速度、上一历元和当前历元加速度输出\n
 * - 待求: 当前历元速度
 * @author      Zing Fong
 * @date        2022/6/14
 */
void SinsMechanization::VelocityUpdate()
{
    // 对omega_ie_n_和omega_en_e_作线性外推
    auto omega_ie_n_mid = LinearExtrapolation(omega_ie_n_ksub1_,
                                              omega_ie_n_ksub2_);
    auto omega_en_n_mid = LinearExtrapolation(omega_en_n_ksub1_,
                                              omega_en_n_ksub2_);
    // 对速度作线性外推, 计算tk-1/2时刻的速度
    auto v_n_mid = LinearExtrapolation(ksub1_state_.v_ned, ksub2_state_.v_ned);
    // 线性外推计算tk-1/2时刻的重力
    auto g_n_mid = LinearExtrapolation(g_n_ksub1_, g_n_ksub2_);
    
    // 计算a_gc_k-1/2
    auto double_omega_ie_n_mid = omega_ie_n_mid;
    for(auto &a_omega: double_omega_ie_n_mid)
        a_omega *= 2.0;
    auto omega_sum = BaseMatrix::VectorAdd(double_omega_ie_n_mid,
                                           omega_en_n_mid);
    auto cross_product_omega_v = BaseMatrix::CrossProduct(omega_sum, v_n_mid);
    auto a_gc_mid = BaseMatrix::VectorSub(g_n_mid, cross_product_omega_v);
    
    // 计算哥氏重力积分项
    auto delta_v_g_n = a_gc_mid;
    for(auto &a_v_g_n: delta_v_g_n)
        a_v_g_n *= delta_t_;
    
    // 单子样假设计算δv_f,k_b(k-1)
    auto cross_product_theta_v = BaseMatrix::CrossProduct(cur_imu_data_.gyro,
                                                          cur_imu_data_.acc);
    auto half_cross_product_theta_v = cross_product_theta_v;
    for(auto &a_cross: half_cross_product_theta_v)
        a_cross *= 0.5;
    auto delta_v_fk_bksub1 = BaseMatrix::VectorAdd(cur_imu_data_.acc,
                                                   half_cross_product_theta_v);
    BaseMatrix mat_delta_v_fk_bksub1(delta_v_fk_bksub1, 3, 1);  // 矩阵版
    
    // 计算n(k-1)系转动到n(k)系对应的等效旋转矢量ζn(k-1)n(k)
    auto zeta_nksub1_nk = BaseMatrix::VectorAdd(omega_ie_n_mid, omega_en_n_mid);
    for(auto &a_zeta: zeta_nksub1_nk)
        a_zeta *= delta_t_;
    
    // 计算比力积分项
    auto antisymmetry = BaseMatrix::CalcAntisymmetryMat(zeta_nksub1_nk)*0.5;
    auto delta_v_fk_n =
            (BaseMatrix::eye(3) - antisymmetry)*ksub1_state_.c_b_n*
            mat_delta_v_fk_bksub1;
    
    // 速度更新
    auto sum_f_gcor = BaseMatrix::VectorAdd(delta_v_fk_n.get_mat(),
                                            delta_v_g_n);  // 哥氏重力积分项和比力积分项的和
    cur_state_.v_ned = BaseMatrix::VectorAdd(ksub1_state_.v_ned, sum_f_gcor);
}

/**@brief       位置更新
 * @details
 * - 已知: 上一历元位置BLH、上一历元和当前历元的速度\n
 * - 待求: 当前历元位置BLH
 * @author      Zing Fong
 * @date        2022/6/14
 */
void SinsMechanization::PositionUpdate()
{
    // 高程更新
    cur_state_.blh[2] = ksub1_state_.blh[2]
                        - 0.5*(ksub1_state_.v_ned[3] + cur_state_.v_ned[3])*
                          delta_t_;
    // 纬度更新
    double h_bar = 0.5*(cur_state_.blh[2] + ksub1_state_.blh[2]);  // 积分周期内平均高程
    cur_state_.blh[0] = ksub1_state_.blh[0] +
                        (cur_state_.v_ned[0] + ksub1_state_.v_ned[0])/
                        (2*(r_m_ksub1_ + h_bar))*delta_t_;
    // 经度更新
    double r_n_mid = 0.5*(r_n_ + r_n_ksub1_);  // 中间时刻Rn
    double phi_bar = 0.5*(cur_state_.blh[0] + ksub1_state_.blh[0]);  // 中间时刻纬度
    cur_state_.blh[1] = ksub1_state_.blh[1] +
                        (cur_state_.v_ned[1] + cur_state_.v_ned[1])/
                        (2*(r_n_mid + h_bar)*cos(phi_bar))*delta_t_;
    
    // 更新xyz
    cur_state_.xyz = BaseMath::Blh2Xyz(cur_state_.blh);
}

/**@brief       一个历元的惯导机械编排
 * @details
 * - 已知: 上一历元位置BLH、上一历元和当前历元的速度\n
 * - 待求: 当前历元位置BLH
 * @author      Zing Fong
 * @date        2022/6/14
 */
int SinsMechanization::ImuMechanization(const ImuData &imu_data)
{
    return 0;
}

double SinsMechanization::get_t() const
{
    return t_;
}

StateInfo SinsMechanization::get_cur_state() const
{
    return cur_state_;
}

/**@brief       线性外推
 * @param[in]   ksub1       k-1历元某一矢量
 * @param[in]   ksub2       k-2历元某一矢量
 * @return      线性外推k-1/2历元结果
 * @author      Zing Fong
 * @date        2022/6/13
 */
std::vector<double> SinsMechanization::LinearExtrapolation(
        std::vector<double> &ksub1, std::vector<double> &ksub2)
{
    auto tmp = BaseMatrix::VectorSub(ksub1, ksub2);
    for(auto &a_tmp: tmp)
        a_tmp /= 2.0;  // 除以2
    auto ksubhalf = BaseMatrix::VectorAdd(ksub1, tmp);
    return ksubhalf;
}
