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

/**@struct      CommonTime
 * @brief       通用时, 年月日时分秒
 * @par         修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct CommonTime
{
    int year{};
    int month{};
    int day{};
    int hour{};
    int minute{};
    double second{};
};

/**@struct      MjdTime
 * @brief       简化儒略日
 * @par         修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct MjdTime
{
    int day{};
    double frac_day{};  // 简化儒略日的小数部分(天), 精度损失严重
    double sec_of_day{};  // 小数天的具体秒数
};

/**@class   GpsTime
 * @brief   GPS时, 包括GPS周和周秒
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
class GpsTime
{
public:
    int week_{};  // GPS周
    double sec_of_week_{};  // GPS周秒
    
    void normalize();  // 标准化, 即周内秒在[0, 604800)范围内
    double operator-(const GpsTime &subtrahend) const;  // 两个GpsTime的时间间隔(s)
    GpsTime operator-(const double &subtrahend) const;  // GpsTime减去一定秒数
};

/**@class   BdsTime
 * @brief   BDS时, 直接从GpsTime继承过来, 因为量纲都是一样的, 只不过起始时间不一样
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BdsTime : public GpsTime
{
};

/**@class   BaseTime
 * @brief   时间运算类, 实现了通用时、儒略日时、GPS时、BDS时相互之间的转换算法
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseTime
{
public:
    static double GpstimeSub(const double &t, const double &toc);  // 两个GPS周内秒的减法, 用于获取tk
    static double CommonTime2UT(const CommonTime &common_time);  // 通用时转世界时
    
    static MjdTime CommonTime2MjdTime(const CommonTime &common_time);  // 通用时转简化儒略日
    static CommonTime MjdTime2CommonTime(const MjdTime &mjd_time);  // 简化儒略日转通用时
    
    static GpsTime MjdTime2GpsTime(const MjdTime &mjd_time);  // 简化儒略日转GPS时
    static MjdTime GpsTime2MjdTime(const GpsTime &gps_time);  // GPS时转简化儒略日
    
    static GpsTime CommonTime2GpsTime(const CommonTime &common_time);  // 通用时转GPS时
    static CommonTime GpsTime2CommonTime(const GpsTime &gps_time);  // GPS时转通用时
    
    static BdsTime GpsTime2BdsTime(const GpsTime &gps_time);  // GPS时 转 BDS时
    static GpsTime BdsTime2GpsTime(const BdsTime &bds_time);  // BDS时 转 GPS时
    
    static BdsTime MjdTime2BdsTime(const MjdTime &mjd_time);  // 简化儒略日转BDS时
    static MjdTime BdsTime2MjdTime(const BdsTime &bds_time);  // BDS时转简化儒略日
    
    static BdsTime CommonTime2BdsTime(const CommonTime &common_time);  // 通用时转BDS时
    static CommonTime BdsTime2CommonTime(const BdsTime &bds_time);  // BDS时转通用时
};


#endif //LOOSECOUPLED_SRC_BASETK_BASE_TIME_H
