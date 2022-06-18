/**@file    sins_mechanization.h
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
#ifndef LOOSECOUPLED_SRC_SINSTK_SINS_MECHANIZATION_H
#define LOOSECOUPLED_SRC_SINSTK_SINS_MECHANIZATION_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_math.h"
#include "../basetk/base_time.h"
#include "../basetk/base_app.h"
#include "sins_file_stream.h"

/**@struct      StateInfo
 * @brief       载体位姿状态信息, 包括三轴位置、速度、姿态
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * <tr><td>2022/6/9     <td>Zing Fong   <td>更改了位置和速度的描述
 * </table>
 */
struct StateInfo
{
    double time{};  // 时间信息, GPS周秒
    std::vector<double> q = std::vector<double>(4, 0.0);  // 姿态四元数
    BaseMatrix c_b_n = BaseMatrix(3, 3);  // 姿态方向余弦矩阵
    std::vector<double> v_ecef = std::vector<double>(3, 0.0);  // ECEF系下的速度
    std::vector<double> v_ned = std::vector<double>(3, 0.0);  // n系下的速度(NED)
    std::vector<double> v_enu = std::vector<double>(3, 0.0);  // ENU系下的速度
    std::vector<double> xyz = std::vector<double>(3, 0.0);  // ECEF系下的位置
    std::vector<double> blh = std::vector<double>(3, 0.0);  // 大地坐标
};

/**@class   SinsMechanization
 * @brief   惯导机械编排类
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * <tr><td>2022/6/12    <td>Zing Fong   <td>修改了位姿更新函数的传入参数
 * </table>
 */
class SinsMechanization
{
  public:
    SinsMechanization() = default;  // 默认构造函数
    
    void Init(const StateInfo &initial_state);  // 状态初始化
    int ImuMechanization(const ImuData &imu_data);  // 进行一次机械编排
    
    // get
    double get_t() const;
    double get_delta_t() const;
    StateInfo get_cur_state() const;
    double get_r_m() const;
    double get_r_n() const;
    std::vector<double> get_g_n() const;
    std::vector<double> get_omega_ie_n() const;
    std::vector<double> get_omega_en_n() const;
    std::vector<double> get_omega_in_n() const;
  
  private:
    int PrepareUpdate(const ImuData &imu_data);  // 更新前准备, 将惯性传感器数据存储起来
    void AttitudeUpdate();  // 姿态更新
    void VelocityUpdate();  // 速度更新
    void PositionUpdate();  // 位置更新
    std::vector<double> LinearExtrapolation(std::vector<double> &ksub1,
                                            std::vector<double> &ksub2);  // 线性外推
    int cur_epoch_{};  // 累计经过了多少个历元
    double t_{};  // 当前历元时间, GPS周秒
    double delta_t_{};  // 当前历元和上一历元的时间间隔
    
    double r_m_{};  // 子午圈半径
    double r_n_{};  // 卯酉圈半径
    double r_m_ksub1_{};  // k-1时刻子午圈半径
    double r_n_ksub1_{};  // k-1时刻卯酉圈半径
    
    std::vector<double> g_n_ = std::vector<double>(3, 0.0);  // n系下的重力加速度
    std::vector<double> g_n_ksub1_ = std::vector<double>(3,
                                                         0.0);  // k-1时刻n系下的重力加速度
    std::vector<double> g_n_ksub2_ = std::vector<double>(3,
                                                         0.0);  // k-2时刻n系下的重力加速度
    
    std::vector<double> omega_ie_n_ = std::vector<double>(3,
                                                          0.0);  // e系下的地球自转角速度
    std::vector<double> omega_en_n_ = std::vector<double>(3, 0.0);
    std::vector<double> omega_ie_n_ksub1_ = std::vector<double>(3,
                                                                0.0);  // k-1时刻e系下的地球自转角速度
    std::vector<double> omega_en_n_ksub1_ = std::vector<double>(3,
                                                                0.0);  // k-1时刻的内个, 具体名字我也不知道
    std::vector<double> omega_ie_n_ksub2_ = std::vector<double>(3,
                                                                0.0);  // k-2时刻e系下的地球自转角速度
    std::vector<double> omega_en_n_ksub2_ = std::vector<double>(3,
                                                                0.0);  // k-2时刻的内个, 具体名字我也不知道
    
    StateInfo cur_state_{};  // 当前时刻位姿状态信息
    StateInfo ksub1_state_{};  // k-1时刻位姿信息
    StateInfo ksub2_state_{};  // k-2时刻位姿信息
    
    ImuData cur_imu_data_{};  // 当前时刻传感器数据
    ImuData ksub1_imu_data_{};  // k-1时刻传感器数据
};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_MECHANIZATION_H
