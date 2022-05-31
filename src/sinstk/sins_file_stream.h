/**@file    sins_file_stream.h
 * @brief   捷联惯导输出文件读取
 * @details 支持对txt格式的imu输出文件的读取
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
#ifndef LOOSECOUPLED_SRC_SINSTK_SINS_FILE_STREAM_H
#define LOOSECOUPLED_SRC_SINSTK_SINS_FILE_STREAM_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_math.h"
#include "../basetk/base_time.h"
#include "../basetk/base_app.h"

/**@struct      ImuData
 * @brief       读取的单个历元的原始IMU数据
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct ImuData
{
    double time{};  // 时间
    std::vector<double> acc = std::vector<double>(3, 0.0);  // 加表输出 初始为右前上b系
    std::vector<double> gyro = std::vector<double>(3, 0.0);  // 陀螺输出 初始为右前上b系
};

/**@class   SinsFileStream
 * @brief   捷联惯导输出文件流, 实现了imu文件的读取操作
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class SinsFileStream
{
  public:
    int Init(const Config &config);  // 打开文件, 初始化
    int ReadImuFile();  // 读一个历元的IMU数据
    
    // get
    GpsTime get_time() const;
    ImuData get_raw_data() const;
    
  private:
    FILE *imu_file_ptr_{};  // imu文件指针
    GpsTime time_{};  // 时间
    ImuData raw_data_{};  // imu原始数据 右前上b系
    
};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_FILE_STREAM_H
