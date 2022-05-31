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
 * </table>
 */
struct StateInfo
{
    std::vector<double> quaternion = std::vector<double>(4, 0.0);  // 姿态四元数
    BaseMatrix rotation_mat = BaseMatrix(3, 3);  // e系到b系的旋转矩阵
    std::vector<double> velocity = std::vector<double>(3, 0.0);  // e系下的速度
    std::vector<double> pos = std::vector<double>(3, 0.0);  // e系下的位置
};

/**@class   SinsMechanization
 * @brief   惯导机械编排类
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class SinsMechanization
{
  public:
    int PrepareUpdate();  // 更新前准备
    void AttitudeUpdate(const ImuData &imu_data);  // 姿态更新
    void VelocityUpdate(const ImuData &imu_data);  // 速度更新
    void PositionUpdate(const ImuData &imu_data);  // 位置更新
    int ImuMechanization(const ImuData &imu_data);  // 进行一次机械编排
  
    // get
    StateInfo get_cur_state() const;
  
  private:
    int cur_calendar{};  // 当前推算历元数
    std::vector<double> ge_ = std::vector<double>(3, 0.0);  // e系下的重力加速度
    std::vector<double> omega_ie_e_ = std::vector<double>(3, 0.0);  // e系下的地球自转角速度
    
    StateInfo cur_state_{};  // 当前时刻位姿状态信息
    StateInfo last_state_{};  // 上一时刻位姿状态信息
    
};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_MECHANIZATION_H
