/**@file    sins_file_stream.cc
 * @brief   捷联惯导输出文件读取.cc文件
 * @details 支持对txt格式的imu输出文件的读取
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/7
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/7     <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "sins_file_stream.h"
// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <string>

// 本项目内 .h 文件

/**@brief           读取一行, 并对一行进行分析
 * @param[in]       config        配置表
 * @return          打开文件正常符号0, 打开失败的话直接程序终止
 * @author          Zing Fong
 * @date            2022/6/7
 */
int SinsFileStream::Init(const Config &config)
{
    std::string imu_file_path = config.ReadString("SINS", "imu_file_path",
                                                  "imu.txt");
    file_ptr_ = fopen(imu_file_path.c_str(), "r");
    if(file_ptr_ == nullptr)
    {
        // 这里不用fopen_s的异常返回值, 而是通过判断file_ptr是否为空指针的方式判断是否打开成功
        printf("Cannot open imu file! file path: %s\n", imu_file_path.c_str());
        std::abort();
    }
    return 0;
}

/**@brief           读取一行, 并对一行进行分析
 * @param[in]       config        配置表
 * @return          返回结果:\n
 * -     0          读取正常
 * -    -1          到达文件末尾
 * @author          Zing Fong
 * @date            2022/6/7
 */
int SinsFileStream::ReadImuFile()
{
    if(feof(file_ptr_))  // 已到达文件末尾
        return -1;
    // 读取一行
    fscanf(file_ptr_, "%f %f %f %f %f %f %f", time_, raw_data_.acc[0],
           raw_data_.acc[1], raw_data_.acc[2], raw_data_.gyro[0],
           raw_data_.gyro[1], raw_data_.gyro[2]);
    return 0;
}

SinsFileStream::~SinsFileStream()
{
    fclose(file_ptr_);
}

GpsTime SinsFileStream::get_time() const
{
    return time_;
}

ImuData SinsFileStream::get_raw_data() const
{
    return raw_data_;
}
