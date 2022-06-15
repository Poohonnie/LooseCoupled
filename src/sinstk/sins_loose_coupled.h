/**@file    sins_loose_coupled.h
 * @brief   GNSS/INS松组合解算
 * @details 输入RTK和惯导机械编排的位姿信息, 进行松组合解算
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
#ifndef LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H
#define LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_math.h"
#include "../basetk/base_time.h"
#include "../basetk/base_app.h"
#include "../gnsstk/gnss_pos.h"
#include "sins_file_stream.h"
#include "sins_mechanization.h"

// TODO: 完成松组合算法设计
class SinsLooseCoupled
{
  public:
  
    void Predict(const ImuData &imu_data);  // 一步预测
    void Update(const ImuData &imu_data, const StateInfo &gnss_state);  // 测量更新(在有GPS输入的情况下)
    
  private:
    BaseMatrix CalcF();  // 计算F矩阵
    BaseMatrix CalcFrr();  // 计算Frr矩阵
    BaseMatrix CalcFvr();  // 计算Fvr矩阵
    BaseMatrix CalcFphir();  // 计算Fφr矩阵
    BaseMatrix CalcFvv();  // 计算Fvv矩阵
    BaseMatrix CalcFphiv();  // 计算Fφv矩阵
    
    SinsMechanization sins_mechanization_{};  // 机械编排对象, 包含位置、速度、姿态等信息
    
    BaseMatrix x_k_{};  // k时刻系统状态
    
    BaseMatrix x_k_ksub1{};  // 一步预测状态
    BaseMatrix p_k_ksub1_{};  // 一步预测协方差阵
    
    BaseMatrix K_k_{};  // 增益矩阵
    
    
    BaseMatrix phi_k_ksub1_{};  // 离散形式状态转移矩阵
    BaseMatrix G_{};  // 不知道啥
    BaseMatrix Q_{};  // 协因数阵
    BaseMatrix P_{};  // 协方差阵
};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H
