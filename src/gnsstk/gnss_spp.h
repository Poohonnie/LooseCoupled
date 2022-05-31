/**@file    gnss_spp.h
 * @brief   GNSS单点定位.h文件
 * @details 实现了GNSS单点定位以及粗差探测
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/30
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/30    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

#ifndef LOOSECOUPLED_SRC_GNSSTK_GNSS_SPP_H
#define LOOSECOUPLED_SRC_GNSSTK_GNSS_SPP_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_time.h"
#include "../basetk/base_sdc.h"
#include "../basetk/base_matrix.h"
#include "../basetk/base_app.h"
#include "gnss_file_stream.h"

/**@struct      TmpParam
 * @brief       卫星位置计算临时变量集合
 * @note        原封不动抄过来的, 省的后面改代码麻烦, 变量名就这样了
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct TmpParam
{
    double A{};  // 轨道长半轴
    double n0{};  // 平均运动角速度
    double tk{};  // 相对于星历参考历元的时间
    double n{};  // 对平均运动角速度进行改正
    double mk{};  // 平近点角
    double ek{};  // 偏近点角，迭代求解
    double vk{};  // 真近点角
    double phik{};  // 升交角距
    // 二阶调和改正数
    double deltaUk{};  // 计算升交角距的改正数
    double deltaRk{};  // 计算向径的改正数
    double deltaIk{};  // 计算轨道倾角改证数
    // 计算经过改正的升交角距，向径和轨道倾角
    double uk{};  // 改正过的升交角距
    double rk{};  // 改正过的向径
    double ik{};  // 改正过的轨道倾角
    double omegak{};  // 改正后的升交点经度
    double xy0[2]{};  // 卫星在轨道平面上的位置
    
    // 各参变量对时间的导数
    double mkDot{};
    double ekDot{}, vkDot{}, phikDot{};
    
    double deltaUkDot{}, deltaRkDot{}, deltaIkDot{};
    
    double omegakDot{};
    double ikDot{}, rkDot{}, ukDot{};
    
    // 卫星在轨道平面内的速度
    double xkDot{}, ykDot{};
};

/**@class       SatPos
 * @brief       单个卫星的位置
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
class SatPos
{
    friend class EpochPos;
  
  public:
    bool valid{};  // 定位结果是否可用 例如高度角限制等等
    
    // TODO:确认这里的时间哪些是没有更改的, make it const
    static TmpParam CalTmpParam(GpsTime t/*卫星钟表面时*/, const Ephemeris &ephem);
    
    void CalSat(GpsTime t, const Ephemeris &ephem);  // 计算卫星相关数据
    void CalPosVel(const Ephemeris &ephem, TmpParam &para);  // 卫星位置速度计算
    void ClockBias(GpsTime t, double ek, const Ephemeris &ephem);  // 钟差计算
    void ClockRate(GpsTime t, double ek, double ekDot,
                   const Ephemeris &ephem);  // 钟速计算
    static bool Overdue(GpsTime t, const Ephemeris &ephem);  // 判断星历是否过期
    
    void CalcSatElevation(const std::vector<double> &recv,
                          CoorSys &coor_sys);  // 卫星高度角的计算  rad
    void CalcHopefield(const std::vector<double> &recv,
                       CoorSys &coor_sys);  // 对流层Hopefield模型改正
    
    // get
    Gnss get_sys() const;
    int get_prn() const;
    std::vector<double> get_sat_xyz() const;
    std::vector<double> get_sat_v() const;
    double get_clk_bias() const;
    double get_clk_rate() const;
    double get_elevation() const;
    double get_trop_delay() const;
    double get_obs_times() const;
    
    // set
    void set_clk_bias();
  
  private:
    Gnss sys_{};
    int prn_{};
    std::vector<double> sat_xyz_ = std::vector<double>(3,
                                                       0.0);  // 卫星在地心地固坐标系下的坐标
    std::vector<double> sat_v_ = std::vector<double>(3, 0.0);  // 卫星运动速度
    double clk_bias_{};  // 卫星钟差
    double clk_rate_{};  // 卫星钟速
    double elevation{};  // 卫星高度角
    double trop_delay_{};  // 对流层延迟
    double obs_times_{};  // 观测历元数
    
};

/**@class       EpochPos
 * @brief       某一历元全部卫星位置集合
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
class EpochPos
{
  public:
    int FindSatPosIndex(const int &prn, const Gnss &sys);  // 查找卫星
    
    // get
    int get_sat_num() const;
    std::vector<SatPos> get_sat_pos() const;
  
  private:
    int sat_num_{};  // 卫星数
    std::vector<SatPos> sat_pos_ = std::vector<SatPos>(BaseSdc::kMaxBdsNum,
                                                       SatPos{});
};

/**@struct      Gfmw
 * @brief       单个卫星观测值GF和MW组合, 也包括IF组合
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct Gfmw
{
    Gnss sys{};
    int prn{};
    double l_mw{};  // 伪距MW组合
    double l_gf{};  // 伪距GF组合
    double l_if{};  // 伪距IF组合
    double p_if{};  // 载波相位IF组合
    int n{};  // 平滑历元数，1表示第一个历元
    bool valid{};  // 可用情况, 是否有粗差等等
};

/**@class       EpochGfmw
 * @brief       某一历元全部卫星的观测值GFMW组合
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
class EpochGfmw
{
  public:
    int FindGfmwIndex(const int &prn,
                      const Gnss &sys);  // 搜索某个prn号的卫星在epkGfmw中的下标
    
    // get
    std::vector<Gfmw> get_gfmw() const;
  
  private:
    std::vector<Gfmw> gfmw_ = std::vector<Gfmw>(BaseSdc::kMaxChannelNum,
                                                Gfmw{});  // 单个历元所有GFMW组合观测值
    
};

/**@class       OutlierDetector
 * @brief       粗差探测器, 放入原始数据即可检测观测值粗差
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
class OutlierDetector
{
  public:
    void DetectOutlier(RawData raw_data);
    
    // get
    EpochGfmw get_cur_epoch() const;
  
  private:
    EpochGfmw last_epoch{};  // 上一历元GF和MW组合
    EpochGfmw cur_epoch{};  // 当前历元GF和MW组合
};

/**@class       GnssSpp
 * @brief       单点定位类, 根据原始观测值, GFMW组合进行标准单点定位
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
class GnssSpp
{
  public:
    void ExtendMatB(BaseMatrix &B,
                    const int total);  // 将设计矩阵B根据GPS以及BDS卫星数目情况进行扩展
    void ExtendDeltaX(BaseMatrix &deltaX);  // 将deltaX根据GPS以及BDS卫星数目情况进行扩展
    
    int StdPointPositioning(RawData &raw_data, EpochGfmw &epk_efmw,
                            const Config &config);  // 单点定位
    void CalcPointVelocity(RawData &raw_data, EpochGfmw &epk_efmw,
                           const Config &config);  // 单点测速
    
    // get
    GpsTime get_t() const;
    std::vector<double> get_station_xyz() const;
    std::vector<double> get_station_blh() const;
    double get_clk_g() const;
    double get_clk_b() const;
    double get_p_dop() const;
    double get_sigma_p() const;
    std::vector<double> get_station_v() const;
    std::vector<double> get_sigma_v() const;
    std::vector<int> get_sys_num() const;
    EpochPos get_epoch_pos() const;
  
  private:
    GpsTime t_{};  // 信号发射时刻
    
    std::vector<double> station_xyz_ = std::vector<double>(3,
                                                           0.0);  // 测站在地心地固坐标系下的坐标
    std::vector<double> station_blh_ = std::vector<double>(3,
                                                           0.0);  // 测站在WGS84坐标系下的坐标
    
    double clk_g_{};  // 测站GPS钟差
    double clg_b_{};  // 测站BDS钟差
    double p_dop_{};  // PDOP值
    double sigma_p_{};  // 定位标准差
    
    std::vector<double> station_v_ = std::vector<double>(3, 0.0);  // 测站三向速度
    double sigma_v_{};  // 测速标准差
    
    std::vector<int> sys_num = std::vector<int>(4, 0);  // 各系统卫星数
    EpochPos epk_pos;  // 卫星位置数据
    
};


#endif //LOOSECOUPLED_SRC_GNSSTK_GNSS_SPP_H
