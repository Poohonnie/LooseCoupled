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
    EpochObs()
    {
        sat_obs_.reserve(BaseSdc::kMaxChannelNum);  // 预先分配内存
    }
    
    int FindSatObsIndex(const int &prn, const Gnss &sys);  // 搜索某个prn号的卫星在epkObs中的下标
    
    GpsTime get_time() const;
    int get_sat_num() const;
    std::vector<SatObs> get_sat_obs() const;
    
private:
    GpsTime time_{};  // 该历元的时间
    int sat_num_{};  // 观测值数目
    std::vector<SatObs> sat_obs_{};  // 单个历元所有卫星观测值
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
    int OFileRead(const char *file_name);  // O文件中读取一秒的观测值
    int PFileRead(const char *file_name);  // 读取P文件, 注意只有在星历过期的时候才会读
    
    GpsTime get_time() const;
    
private:
    FILE *o_file_{};  // o文件指针
    FILE *p_file_{};  // p文件指针
    GpsTime time_{};  // 该历元的时间
};


#endif //LOOSECOUPLED_SRC_GNSSTK_GNSS_FILE_STREAM_H
