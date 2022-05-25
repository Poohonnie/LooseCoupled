/**@file    base_time.h
 * @brief   时间转换类.h文件
 * @details 声明了通用时、儒略日时、GPS时、BDS时以及其相互之间的转换
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/25
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/25    <td>1.0      <td>Zing Fong  <td>To be initialized
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_BASETK_BASE_TIME_H
#define LOOSECOUPLED_SRC_BASETK_BASE_TIME_H

struct CommonTime
{
    int year{};
    int month{};
    int day{};
    int hour{};
    int minute{};
    double second{};
};

struct MjdTime
{
    int day{};
    double frac_day{};  // 简化儒略日的小数部分(天), 精度损失严重
    double sec_of_day{};  // 小数天的具体秒数
};

class GpsTime
{
public:
    int week{};  // GPS周
    double sec_of_week{};  // GPS周秒
    
    
};

class BdsTime : public GpsTime
{};

class BaseTime
{
public:

};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_TIME_H
