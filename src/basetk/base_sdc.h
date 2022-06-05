/**@file    base_sdc.h
 * @brief   科学资料中心sdc.h文件
 * @details 声明了各种常数以及GNSS系统的枚举类
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/27
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/27    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

#ifndef LOOSECOUPLED_SRC_BASETK_BASE_SDC_H
#define LOOSECOUPLED_SRC_BASETK_BASE_SDC_H

// c/c++系统文件

// 其他库的 .h 文件

// 本项目内 .h 文件

/**@enum    Gnss
 * @brief   GNSS系统枚举类, 定义了四个卫星系统
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
enum class Gnss
{
    kGps, kBds, kGlonass, kGalileo
};

/**@struct      CoorSys
 * @brief       坐标系统, 用于存储WGS84和CGCS2000坐标参数
 * @par         修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
struct CoorSys
{
    double kA{};  // 长半轴 单位m
    double kB{};  // 短半轴 单位m
    double kESquare{};  // 第一偏心率的平方
    double kE2Square{};  // 第二偏心率的平方
    double kF{};  // 扁率
    double kGm{};  // 地心引力常数 单位m³s^-2
    double kOmega{};  // 自转角速度 单位rad/s
};

/**@class   BaseSdc
 * @brief   科学资料中心SDC类, 声明并定义了各种常量
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/27    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseSdc
{
  public:
    static constexpr double kPi = 3.14159265358979323846;  // 圆周率
    static constexpr int kVOfLight = 299792458;  // 光速c, 单位 m/s
    static constexpr double kD2R = kPi/180.0;  // 角度转弧度
    static constexpr double kR2D = 180.0/kPi;  // 弧度转角度
    
    static constexpr double kFreqL1 = 1575.420e+6;  // GPS L1信号频率
    static constexpr double kFreqL2 = 1227.600e+6;  // GPS L2信号频率
    static constexpr double kFreqB1 = 1561.098e+6;  // BDS B1信号频率
    static constexpr double kFreqB3 = 1268.520e+6;  // BDS B3信号频率
    
    static constexpr double kWavelengthL1 = kVOfLight/kFreqL1;  // GPS L1波长(m)
    static constexpr double kWavelengthL2 = kVOfLight/kFreqL2;  // GPS L2波长(m)
    static constexpr double kWavelengthB1 = kVOfLight/kFreqB1;  // BDS B1波长(m)
    static constexpr double kWavelengthB3 = kVOfLight/kFreqB3;  // BDS B3波长(m)
    
    static constexpr CoorSys wgs84
            {
                    6378137.0,
                    6356752.3142,
                    0.00669437999013,
                    0.006739496742227,
                    1/298.257223563,
                    3.986004418e+14,
                    7.292115e-5
            };  // WGS84坐标系参数
    static constexpr CoorSys cgcs2000
            {
                    6378137.0,
                    6356752.3142,
                    0.00669437999013,
                    0.006739496742227,
                    1/298.257223563,
                    3.986004418e+14,
                    7.292115e-5
            };  // CGCS2000坐标系参数
    
    static constexpr int kMaxChannelNum = 36;  // 一秒最多可观测到的卫星数
    static constexpr int kMaxGpsNum = 32;  // GPS最大卫星数
    static constexpr int kMaxBdsNum = 63;  // BDS最大卫星数
};

#endif //LOOSECOUPLED_SRC_BASETK_BASE_SDC_H
