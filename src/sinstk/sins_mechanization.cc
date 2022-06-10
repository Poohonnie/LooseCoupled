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

/**@brief       状态初始化
 * @param[in]   initial_state          载体的初始状态量
 * @author      Zing Fong
 * @date        2022/6/9
 */
void SinsMechanization::Init(const StateInfo &initial_state)
{


}

int SinsMechanization::PrepareUpdate()
{
    return 0;
}

void SinsMechanization::AttitudeUpdate(const ImuData &imu_data)
{

}

void SinsMechanization::VelocityUpdate(const ImuData &imu_data)
{

}

void SinsMechanization::PositionUpdate(const ImuData &imu_data)
{

}

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
