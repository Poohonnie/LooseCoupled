/**@file    base_time.cc
 * @brief   时间转换类cc文件
 * @details 实现了通用时、儒略日时、GPS时、BDS时相互之间的转换算法
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/1
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/1    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "base_time.h"
// c/c++系统文件

// 其他库的 .h 文件
#include <cmath>

// 本项目内 .h 文件

/**@brief       GPS时自检, 将周内秒控制在[0, 604800)范围内
 * @author      Zing Fong
 * @date        2022/6/1
 */
void GpsTime::normalize()
{
    for (int i = 0; i < 3 && sec_of_week_ < 0; ++i)
    {
        // 最多循环3次，防止死循环
        sec_of_week_ += 604800.0;
        --week_;
    }
    for(int i = 0; i < 3 && sec_of_week_ >= 604800.0; ++i)
    {
        sec_of_week_ -= 604800.0;
        --week_;
    }
}

/**@brief       “-”重载, 两个GPS时作差
 * @param[in]   subtrahend        减数GPS时
 * @return      "-"左右GPS时相减结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
double GpsTime::operator-(const GpsTime &subtrahend) const
{
    return BaseTime::GpstimeSub(week_ * 604800.0 + sec_of_week_,
                        subtrahend.week_ * 604800.0 + subtrahend.sec_of_week_);
}

/**@brief       “-”重载, GPS时减去一定秒数
 * @param[in]   subtrahend        待减去的秒数
 * @return      GPS时减去一定秒数的结果
 * @author      Zing Fong
 * @date        2022/6/1
 */
GpsTime GpsTime::operator-(const double &subtrahend) const
{
    GpsTime result(*this);
    result.sec_of_week_ -= subtrahend;
    result.normalize();
    return result;
}

/**@brief       GPS周秒减法, 仅支持大减小, 否则结果会出错
 * @param[in]   t       时间线上靠前的时间点
 * @param[in]   toc     时间线上靠后的时间点
 * @return      两个时间点的时间差(s)
 * @author      Zing Fong
 * @date        2022/6/1
 */
double BaseTime::GpstimeSub(const double &t, const double &toc)
{
    double result = t - toc;
    if (result > 302400)
        result -= 604800;
    else if (result < -302400)
        result += 604800;
    return result;
}

/**@brief       通用时转世界时, 世界时单位为h
 * @param[in]   common_time       通用时
 * @return      世界时
 * @author      Zing Fong
 * @date        2022/6/1
 */
double BaseTime::CommonTime2UT(const CommonTime &common_time)
{
    // 以小时为单位的UT
    double UT = common_time.hour + common_time.minute / 60.0 + common_time.second / 3600.0;
    return UT;
}

/**@brief       通用时转简化儒略日
 * @param[in]   common_time      通用时
 * @return      简化儒略日
 * @author      Zing Fong
 * @date        2022/6/1
 */
MjdTime BaseTime::CommonTime2MjdTime(const CommonTime &common_time)
{
    //通用时转简化儒略日
    MjdTime mjd_time{};
    int y;  // 年
    int m;  // 月
    if (common_time.month <= 2)
    {
        y = common_time.year - 1;
        m = common_time.month + 12;
    }
    else
    {
        y = common_time.year;
        m = common_time.month;
    }
    double MJD = int(365.25 * y) + int(30.6001 * (m + 1)) + common_time.day + CommonTime2UT(common_time)/24 + 1720981.5 - 2400000.5;  // PPT1-4第 9 页MJD计算公式，单位不出意外应该为 天
    mjd_time.day = int(MJD);
    mjd_time.frac_day = MJD - int(MJD);  // mjd_time 的小数部分，单位为天
    mjd_time.sec_of_day = common_time.hour*3600.0 + common_time.minute*60.0 + common_time.second;  // mjd_time 一天中的具体秒数
    return mjd_time;
}

/**@brief       简化儒略日转通用时
 * @param[in]   mjd_time      简化儒略日
 * @return      通用时
 * @author      Zing Fong
 * @date        2022/6/1
 */
CommonTime BaseTime::MjdTime2CommonTime(const MjdTime &mjd_time)
{
    CommonTime common_time{};  // 作为返回值
    int a, b, c, d, e;  // 转换中间参数，见PPT 1-4 第 10 页
    double JD = mjd_time.day + mjd_time.frac_day + 2400000.5;
    a = lround(JD + 0.5); b = a + 1537;
    c = int((b - 122.1) / 365.25);
    d = int(365.25 * c);
    e = int((b - d) / 30.6001);
    double D = b - d - int(30.6001 * e) + JD + 0.5 - lround(JD + 0.5);  // 中间参数一年中的日，单位天
    int M = e - 1 - 12 * int(e / 14);  // 月
    int Y = c - 4715 - int((M + 7) / 10);  // 年
    
    common_time.day = (unsigned short)D;  // D取整
    
    common_time.hour = (unsigned short)(mjd_time.sec_of_day/3600.0);  // 秒数除以3600再取整，得小时数
    common_time.minute = (unsigned short)((mjd_time.sec_of_day - common_time.hour*3600.0)/60.0);  // 一小时内剩余秒数除以60取整，得分钟数
    common_time.second = mjd_time.sec_of_day - common_time.hour*3600.0 - common_time.minute*60.0;  // 秒数
    
    common_time.month = (unsigned char)M;
    common_time.year = short(Y);
    
    return common_time;
}

/**@brief       简化儒略日转GPS时
 * @param[in]   mjd_time      简化儒略日
 * @return      GPS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
GpsTime BaseTime::MjdTime2GpsTime(const MjdTime &mjd_time)
{
    GpsTime gps_time{};
    gps_time.week_ = (unsigned short)((mjd_time.day - 44244)/7);
    gps_time.sec_of_week_ = int(mjd_time.day - 44244 - int((mjd_time.day - 44244)/7)*7)/*这是算出GPS周内天数的整数部分*/ *86400 + mjd_time.sec_of_day;
    return gps_time;
}

/**@brief       GPS时转简化儒略日
 * @param[in]   gps_time      GPS时
 * @return      简化儒略日
 * @author      Zing Fong
 * @date        2022/6/1
 */
MjdTime BaseTime::GpsTime2MjdTime(const GpsTime &gps_time)
{
    MjdTime mjd_time{};
    mjd_time.day = 44244 + gps_time.week_*7 + int(gps_time.sec_of_week_/86400);
    mjd_time.frac_day = fmod(gps_time.sec_of_week_, 86400.0)/86400.0;
    /*gps_time.secOfWeek / 86400 - int(gps_time.secOfWeek / 86400); 可用上面代码替换*/
    mjd_time.sec_of_day = gps_time.sec_of_week_ - int(gps_time.sec_of_week_/86400)*86400.0;
    return mjd_time;
}

/**@brief       通用时转GPS时
 * @param[in]   common_time      通用时
 * @return      GPS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
GpsTime BaseTime::CommonTime2GpsTime(const CommonTime &common_time)
{
    MjdTime mjd_time = CommonTime2MjdTime(common_time);
    GpsTime gps_time = MjdTime2GpsTime(mjd_time);
    return gps_time;
}

/**@brief       GPS时转通用时
 * @param[in]   gps_time      GPS时
 * @return      通用时
 * @author      Zing Fong
 * @date        2022/6/1
 */
CommonTime BaseTime::GpsTime2CommonTime(const GpsTime &gps_time)
{
    MjdTime mjd_time = GpsTime2MjdTime(gps_time);
    CommonTime common_time = MjdTime2CommonTime(mjd_time);
    return common_time;
}

/**@brief       GPS时转BDS时
 * @param[in]   gps_time      GPS时
 * @return      BDS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
BdsTime BaseTime::GpsTime2BdsTime(const GpsTime &gps_time)
{
    BdsTime bds_time{};
    bds_time.week_ = gps_time.week_ - 1356;
    bds_time.sec_of_week_ = gps_time.sec_of_week_ - 14.0;
    bds_time.normalize();
    return bds_time;
}

/**@brief       BDS时转GPS时
 * @param[in]   bds_time      BDS时
 * @return      GPS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
GpsTime BaseTime::BdsTime2GpsTime(const BdsTime &bds_time)
{
    GpsTime gps_time{};
    gps_time.week_ = bds_time.week_ + 1356;
    gps_time.sec_of_week_ = bds_time.sec_of_week_ + 14.0;
    gps_time.normalize();
    return gps_time;
}

/**@brief       简化儒略日转BDS时
 * @param[in]   mjd_time      简化儒略日
 * @return      BDS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
BdsTime BaseTime::MjdTime2BdsTime(const MjdTime &mjd_time)
{
    GpsTime gps_time = MjdTime2GpsTime(mjd_time);
    BdsTime bds_time = GpsTime2BdsTime(gps_time);
    return bds_time;
}

/**@brief       BDS时转简化儒略日
 * @param[in]   bds_time      BDS时
 * @return      简化儒略日
 * @author      Zing Fong
 * @date        2022/6/1
 */
MjdTime BaseTime::BdsTime2MjdTime(const BdsTime &bds_time)
{
    GpsTime gps_time = BdsTime2GpsTime(bds_time);
    MjdTime mjd_time = GpsTime2MjdTime(gps_time);
    return mjd_time;
}

/**@brief       通用时转BDS时
 * @param[in]   common_time      通用时
 * @return      BDS时
 * @author      Zing Fong
 * @date        2022/6/1
 */
BdsTime BaseTime::CommonTime2BdsTime(const CommonTime &common_time)
{
    GpsTime gps_time = CommonTime2GpsTime(common_time);
    BdsTime bds_time = GpsTime2BdsTime(gps_time);
    return bds_time;
}

/**@brief       BDS时转通用时
 * @param[in]   bds_time      BDS时
 * @return      通用时
 * @author      Zing Fong
 * @date        2022/6/1
 */
CommonTime BaseTime::BdsTime2CommonTime(const BdsTime &bds_time)
{
    GpsTime gps_time = BdsTime2GpsTime(bds_time);
    CommonTime common_time = GpsTime2CommonTime(gps_time);
    return common_time;
}

