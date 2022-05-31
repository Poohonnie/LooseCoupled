/**@file    gnss_rtk.h
 * @brief   GNSS实时相对定位.h文件
 * @details 实现了GNSS实时相对定位
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
#ifndef LOOSECOUPLED_SRC_GNSSTK_GNSS_RTK_H
#define LOOSECOUPLED_SRC_GNSSTK_GNSS_RTK_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_time.h"
#include "../basetk/base_sdc.h"
#include "../basetk/base_app.h"
#include "gnss_file_stream.h"
#include "gnss_spp.h"

extern int lambda(int n, int m, const double *a, const double *Q, double *F,
                  double *s);  // lambda模糊度搜索

/**@class       SatSd
 * @brief       单个卫星站间单差观测值
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class SatSd
{
    friend class SdObs;
  
  public:
    bool valid_{};  // 观测值可用性标志
    
    // get
    Gnss get_sys() const;
    int get_prn() const;
    std::vector<double> get_psr_sd() const;
    std::vector<double> get_cp_sd() const;
    int get_r_id() const;
    int get_b_id() const;
  
  private:
    Gnss sys_{};  // 卫星系统
    int prn_{};  // 卫星号
    std::vector<double> psr_sd_ = std::vector<double>(2, 0.0);  // 双频伪距单差
    std::vector<double> cp_sd_ = std::vector<double>(2, 0.0);  // 双频载波相位单差
    int r_id_{}, b_id_{};  // 流动站和基站卫星观测值索引号
    
};

/**@class       SdObs
 * @brief       全部卫星站间单差观测值集合
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct SdObs  // 站间单差
{
  public:
    void GetSdObs(EpochObs &rover_obs, EpochObs &base_obs,
                  GnssSpp &rover_spp, GnssSpp &base_spp);  // 单差求解
    
    // get
    GpsTime get_t() const;
    int get_sd_num() const;
    std::vector<SatSd> get_sat_sd() const;
  
  private:
    GpsTime t_{};  // 时间
    int sd_num_{};  // 单差观测值数目
    std::vector<SatSd> sat_sd = std::vector<SatSd>(BaseSdc::kMaxChannelNum,
                                                   SatSd{});  // 卫星单差观测值数组
    
};

/**@class       CycleSlipDetector
 * @brief       周跳探测器
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class CycleSlipDetector
{
  public:
    void DetectCycleSlip(SdObs &sd_obs);  // 周跳探测
  
  private:
    EpochGfmw last_epk_{};  // 上一历元GF和MW组合
    EpochGfmw cur_epk_{};  // 当前历元GF和MW组合
};

/**@struct      DdObs
 * @brief       站星双差
 * @par         修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct DdObs
{
    // TODO:改下变量名
    int refSatPrn[4]{}, refSatIndex[4]{};  // 参考星的prn号, 以及在单差数组中的下标; 0:GPS 1:BDS 2:GLONASS 3:Galileo
    bool selected[4]{};  // 参考星是否选取成功
    
    int ddPrn[BaseSdc::kMaxChannelNum]{};  // 双差对应的卫星prn号
    Gnss ddSys[BaseSdc::kMaxChannelNum]{};  // 双差对应的卫星系统
    int ddNum{};  // 双差观测值数目
    int sysNum[4]{};  // 各卫星系统可用双差观测值数目
    double dd[BaseSdc::kMaxChannelNum][4]{};  // 双差观测值; 0:L1 1:L2 2:P1 3:P2
    double fixedAmb[BaseSdc::kMaxChannelNum*4]{};  // 固定解模糊度最优和次优组合
};

/**@class       GnssRtk
 * @brief       GNSS实时相对定位
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class GnssRtk
{
  public:
    int CalFixedSolution(RawData &rover_raw, RawData &base_raw,
                         EpochGfmw &r_epoch_gfmw, EpochGfmw &b_epoch_gfmw,
                         Config &config);  // 固定解解算
    
    // get
    GpsTime get_t() const;
    std::vector<double> get_pos() const;
    std::vector<double> get_res_amb() const;
    bool get_valid() const;
    int get_sol() const;
    double get_ratio() const;
  
  private:
    void SelectRefSat();  // 参考星选取
    void GetDDObs();  // 获取双差观测值
    GpsTime t_{};  // 信号发射时刻
    std::vector<double> pos_ = std::vector<double>(3, 0.0);  // 流动站最终定位结果
    double res_amb_[2]{};  // 浮点解中的模糊度残差
    bool valid_{};  // true为有解
    int sol_{};  // 0:single 1:float 2:fixed
    double ratio_{};  // 模糊度固定情况ratio > 3即为可用
    
    SdObs sd_obs_{};  // 站间单差
    DdObs dd_obs_{};  // 站星双差
    std::vector<GnssSpp> gnss_spp_ = std::vector<GnssSpp>(2,
                                                          GnssSpp{});  // 不同接收机的解算(定位)结果  0:rover 1:base;
    CycleSlipDetector detector_;  // 单差观测值周跳探测
    
};


#endif //LOOSECOUPLED_SRC_GNSSTK_GNSS_RTK_H
