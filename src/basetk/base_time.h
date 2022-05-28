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
    int week_{};  // GPS周
    double sec_of_week_{};  // GPS周秒
    
    void normalize();  // 标准化, 即周内秒在[0, 604800)范围内
    double operator-(const GpsTime &subtrahend) const;  // 两个GpsTime的时间间隔(s)
    GpsTime operator-(const double &subtrahend) const;  // GpsTime减去一定秒数
};

class BdsTime : public GpsTime
{
};

class BaseTime
{
public:
    double GpstimeSub(const double &t, const double toc);  // 两个GPS周内秒的减法, 用于获取tk
    double CommonTime2UT(const CommonTime &common_time);  // 通用时转世界时
    
    MjdTime CommonTime2MjdTime(const CommonTime &common_time);  // 通用时转简化儒略日
    CommonTime MjdTime2CommonTime(const MjdTime &mjd_time);  // 简化儒略日转通用时
    
    GpsTime MjdTime2GpsTime(const MjdTime &mjd_time);  // 简化儒略日转GPS时
    MjdTime GpsTime2MjdTime(const GpsTime &gps_time);  // GPS时转简化儒略日
    
    GpsTime CommonTime2GpsTime(const CommonTime &common_time);  // 通用时转GPS时
    CommonTime GpsTime2CommonTime(const GpsTime &gps_time);  // GPS时转通用时
    
    BdsTime GpsTime2BdsTime(const GpsTime &gps_time);  // GPS时 转 BDS时
    GpsTime BdsTime2GpsTime(const BdsTime &bds_time);  // BDS时 转 GPS时
    
    BdsTime MjdTime2BdsTime(const MjdTime &mjd_time);  // 简化儒略日转BDS时
    MjdTime BdsTime2MjdTime(const BdsTime &bds_time);  // BDS时转简化儒略日
    
    BdsTime CommonTime2BdsTime(const CommonTime &common_time);  // 通用时转BDS时
    CommonTime BdsTime2CommonTime(const BdsTime &bds_time);  // BDS时转通用时
};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_TIME_H
