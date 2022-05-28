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
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/5/28  <td>1.0      <td>Zing Fong  <td>Initialize
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
public:
    bool ReadConfig(const std::string &filename);
    std::string ReadString(const char *section, const char *item,
                           const char *default_value);
    int ReadInt(const char *section, const char *item, const int &default_value);
    float ReadFloat(const char *section, const char *item, const float &default_value);

private:
    bool isSpace(char c);
    bool isCommentChar(char c);
    void Trim(std::string &str);
    bool AnalyseLine(const std::string &line, std::string &section,
                     std::string &key, std::string &value);
    bool CreateDefaultConfig();
    
    //std::map<std::string, std::string> settings_;
    std::map<std::string, std::map<std::string, std::string> > settings_;
};

class BaseApp
{

};


#endif // LOOSECOUPLED_SRC_BASETK_BASE_APP_H
