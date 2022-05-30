/**@file    gnss_file_stream.h
 * @brief   GNSS文件读取
 * @details 包括o文件和p文件的读取,
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/29
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/29    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_GNSSTK_GNSS_FILE_STREAM_H
#define LOOSECOUPLED_SRC_GNSSTK_GNSS_FILE_STREAM_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_time.h"
#include "../basetk/base_sdc.h"

/**@struct      SatObs
 * @brief       单个卫星的观测值, 包括双频伪距, 双频载波相位, 多普勒频移
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/29    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct SatObs
{
    // 单个卫星的观测值
    Gnss sys{};  // 卫星系统
    int prn{};  // 卫星编号
    double P[2]{};  // 双频伪距观测值, GPS为L1, L2; BDS为B1, B3
    double L[2]{};  // 双频载波相位观测值
    double D[2]{};  // 多普勒频移
    
    bool valid{};  // 观测值是否可用(是否双频)
};

/**@class       EpochObs
 * @brief       一个历元所有卫星观测值的集合
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/29    <td>Zing Fong   <td>Initialize
 * </table>
 */
class EpochObs
{
public:

    int FindSatObsIndex(const int &prn, const Gnss &sys);  // 搜索某个prn号的卫星在epkObs中的下标
    
    GpsTime get_time() const;
    int get_sat_num() const;
    std::vector<SatObs> get_sat_obs() const;
    
private:
    GpsTime time_{};  // 该历元的时间
    int sat_num_{};  // 观测值数目
    std::vector<SatObs> sat_obs_ = std::vector<SatObs>(BaseSdc::kMaxChannelNum, SatObs{});  // 单个历元所有卫星观测值
};

/**@struct       Ephemeris
 * @brief        单个卫星的星历
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct Ephemeris
{
    //星历
    Gnss sys{};
    int prn{};  // 卫星编号
    int health{};  // 卫星健康状态 0: healthy   others: unhealthy
    GpsTime toeG{};  // GPS时  包含week 和 toe
    BdsTime toeB{};  // BDS时  包含week 和 toe
    double tgd[2]{};  // Equipment group delay differential
    
    double A{};  // 长半轴 A
    double delta_n{};  // Mean anomaly correction (rad/sec)
    double m0{};  // Mean anomaly at reference time (semicircles)
    double ecc{};  // 卫星轨道偏心率
    
    double omega{};  // 近地点角距
    double omega0{};  // 升交点经度
    double omega_dot{};  // 赤经率
    
    double cuc{}, cus{};  // 谐波修正项 单位
    double crc{}, crs{};  // 谐波修正项 单位rad   m
    double cic{}, cis{};  // 谐波修正项 单位
    
    double i0{}, iDot{};  // 倾角 磁倾角变化率
    double toc{};  // SV clock correction term
    double af[3]{};  // 时钟改正值 单位s s/s s/s²
    
    // TODO: 将是否为GEO卫星写在读取星历处, 这样每次调用的时候不用再判断一遍
    bool is_geo{};  // 是否为GEO卫星
};

/**@struct       RawData
 * @brief        原始数据打包
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/30    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct RawData
{
    EpochObs epoch_obs{};
    std::vector<Ephemeris> gps_ephem = std::vector<Ephemeris>(BaseSdc::kMaxGpsNum, Ephemeris{});
    std::vector<Ephemeris> bds_ephem = std::vector<Ephemeris>(BaseSdc::kMaxBdsNum, Ephemeris{});
};

/**@class   GnssFileStream
 * @brief   GNSS文件流类, 实现了O文件和P文件的读取并保存
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/29    <td>Zing Fong   <td>Initialize
 * </table>
 */
class GnssFileStream
{
public:
    int ReadOFile(const char *file_name);  // O文件中读取一秒的观测值
    
    int ReadPFile(const char *file_name);  // 读取P文件, 注意只有在星历过期的时候才会读
    int ReadGpsEphemeris();  // 从当前指针所指位置读取GPS星历
    int ReadBdsEphemeris();  // 从当前文件指针所指位置读取BDS星历
    
    GpsTime get_time() const;
    RawData get_raw_data() const;
    
private:
    FILE *o_file_ptr_{};  // o文件指针
    FILE *p_file_ptr_{};  // p文件指针
    GpsTime time_{};  // 该历元的时间
    RawData raw_data_{};  // 每个历元的原始观测值和星历数据
};


#endif //LOOSECOUPLED_SRC_GNSSTK_GNSS_FILE_STREAM_H
