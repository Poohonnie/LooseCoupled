/**@file    base_app.h
 * @brief   客户端头文件
 * @details 程序入口处, 读取配置文件, 并根据配置参数选择解算类型
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/28
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/5/28   <td>1.0      <td>Zing Fong   <td>Initialize
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_BASETK_BASE_APP_H
#define LOOSECOUPLED_SRC_BASETK_BASE_APP_H


// c/c++系统文件

// 其他库的 .h 文件
#include <map>
#include <string>

// 本项目内 .h 文件

/**@class   Config
 * @brief   配置表类, 声明了各种配置参数, 以及配置文件的创建读取等操作
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/28    <td>Zing Fong   <td>Initialize
 * </table>
 */
class Config
{
    /** ini文件示例
    [BASE]
    #可选解算模式spp, rtk, loose coupled
    mode=rtk
    
    [SPP]
    o_file_path=
    p_file_path=
    
    [RTK]
    #
    base_o_file_path=
    base_p_file_path=
    rover_o_file_path=
    rover_p_file_path=
    #高度角阈值单位为°
    elevation_threshold=15
    ratio_threshold=3.0
    
    [SINS]
    imr_file_path=
     */
     friend class BaseApp;
     
public:
    bool ReadConfig(const std::string &filename);  // 读取配置文件并保存配置参数
    std::string ReadString(const char *section, const char *item,
                           const char *default_value);  // 读取类型为string的参数
    int ReadInt(const char *section, const char *item,
                const int &default_value);  // 读取类型为int的参数
    float ReadFloat(const char *section, const char *item,
                    const float &default_value);  // 读取类型为float的参数

private:
    bool isSpace(char c);  // 判断一个char类型变量是否为' '字符
    bool isCommentChar(char c);  // 判断一个char类型变量是否为注释标识符
    void Trim(std::string &str);  // 清除字符串前方和后方的多余空格
    bool AnalyseLine(const std::string &line, std::string &section,
                     std::string &key, std::string &value);  // 读取一行, 并对一行进行分析
    bool CreateDefaultConfig();  // 创建默认配置文件
    
    
    // 参数设置, 支持块->参数项->参数值的存储模式
    std::map<std::string, std::map<std::string, std::string> > settings_;

    


};


/**@class   BaseApp
 * @brief   应用程序类, 负责程序运行模式的管理, 是整个应用的唯一入口
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/29    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseApp
{
public:
    void run();  // 启动函数, 在此读取配置表并选择解算类型等
    
private:
    Config config_{};  // 配置表
    // GnssApp gnss_app_{};
    // SinsApp sins_app_{};

};


#endif // LOOSECOUPLED_SRC_BASETK_BASE_APP_H
