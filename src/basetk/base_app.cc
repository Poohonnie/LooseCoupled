/**@file    base_matrix.cc
 * @brief   客户端.cc文件
 * @details 实现了配置文件的创建、读取操作, 以及配置参数的读取函数。程序的入口处。
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/28
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/28    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

// 本类对应的.h文件
#include "base_app.h"

// c/c++系统文件
#include <fstream>

// 其他库的 .h 文件
#include <cstdlib>

// 本项目内 .h 文件

/**@brief       判断一个char类型变量是否为' '字符
 * @param[in]   c    要判断的字符
 * @return      判断结果\n
 * - true       该字符为空格\n
 * - false      该字符不为空格\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
bool Config::isSpace(char c)
{
    if(' ' == c || '\t' == c)
        return true;
    return false;
}

/**@brief       判断一个char类型变量是否为注释标识符
 * @param[in]   c    要判断的字符
 * @return      判断结果\n
 * - true       该字符为注释标识符\n
 * - false      该字符不为注释标识符\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
bool Config::isCommentChar(char c)
{
    switch(c)
    {
        case '#':return true;
        case ';':return true;
        default:return false;
    }
}

/**@brief       清除字符串前方和后方的多余空格
 * @param[in,out]   str     待裁剪的字符串
 * @author      Zing Fong
 * @date        2022/5/29
 * @par 示例:
 * @code
 *  std::string str = "     There are some blank spaces     ";
 *  Trim(str);
 *  std::cout << str;
 * @endcode
 * 最终输出结果为
 * @code
 *  There are some blank spaces
 * @endcode
 */
void Config::Trim(std::string &str)
{
    if(str.empty())
    {
        return;  // 字符串为空
    }
    int i, start_pos, end_pos;
    for(i = 0; i < str.size(); ++i)
    {
        if(!isSpace(str[i]))
        {
            break;
        }
    }
    if(i == str.size())
    {
        str = "";  // 字符串为空格
        return;
    }
    start_pos = i;
    for(i = str.size() - 1; i >= 0; --i)
    {
        if(!isSpace(str[i]))
        {
            break;
        }
    }
    end_pos = i;
    str = str.substr(start_pos, end_pos - start_pos + 1);
}

/**@brief           读取一行, 并对一行进行分析
 * @param[in]       line        待分析的字符串
 * @param[out]      section     配置参数大类section
 * @param[out]      key         配置参数名key
 * @param[out]      value       参数值value
 * @return          返回结果:\n
 * - true           该行不为空, 已分析完毕\n
 * - false          该行为空\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
bool Config::AnalyseLine(const std::string &line, std::string &section,
                         std::string &key, std::string &value)
{
    if(line.empty())
        return false;
    int start_pos = 0, end_pos = line.size() - 1, pos, s_startpos, s_endpos;
    if((pos = line.find("#")) != -1)
    {
        if(0 == pos)
        {
            return false;
        }
        end_pos = pos - 1;
    }
    if(((s_startpos = line.find("[")) != -1) &&
       ((s_endpos = line.find("]"))) != -1)
    {
        section = line.substr(s_startpos + 1, s_endpos - 1);
        return true;
    }
    std::string new_line = line.substr(start_pos, start_pos + 1 - end_pos);
    if((pos = new_line.find('=')) == -1)
        return false;
    key = new_line.substr(0, pos);
    value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));
    Trim(key);
    if(key.empty())
    {
        return false;
    }
    Trim(value);
    if((pos = value.find("\r")) > 0)
    {
        value.replace(pos, 1, "");
    }
    if((pos = value.find("\n")) > 0)
    {
        value.replace(pos, 1, "");
    }
    return true;
}

/**@brief       创建默认配置文件
 * @return      判断结果\n
 * - true       默认配置文件创建成功\n
 * - false      默认配置文件创建失败\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
bool Config::CreateDefaultConfig()
{
    
    return true;
}

/**@brief       读取配置文件并保存配置参数
 * @param[in]   filename     待读取的配置文件名
 * @note        如果当前文件夹下没找到配置文件, 则会创建一个默认的
 * @author      Zing Fong
 * @date        2022/5/29
 */
bool Config::ReadConfig(const std::string &filename)
{
    settings_.clear();
    std::ifstream infile(filename.c_str());//构造默认调用open,所以可以不调用open
    if(!infile)
    {
        bool set = CreateDefaultConfig();  // 创建默认配置表
        if(!set)
        {
            printf("FATAL ERROR!! Create default .ini file failed.\n");
            return false;
        }
        infile.open("config.ini");
    }
    std::string line, key, value, section;
    std::map<std::string, std::string> k_v;
    std::map<std::string, std::map<std::string, std::string> >::iterator it;
    while(getline(infile, line))
    {
        if(AnalyseLine(line, section, key, value))
        {
            it = settings_.find(section);
            if(it != settings_.end())
            {
                k_v[key] = value;
                it->second = k_v;
            }
            else
            {
                k_v.clear();
                settings_.insert(std::make_pair(section, k_v));
            }
        }
        key.clear();
        value.clear();
    }
    infile.close();
    return true;
}

/**@brief          读取类型为string的参数
 * @param[in]      section              要查找的块
 * @param[in]      item                 要查找的参数项
 * @param[in]      default_value        该参数项的预设默认值
 * @return         返回结果:\n
 * - 如果配置文件中该参数有值       返回读取到的值\n
 * - 该参数值为空                 返回预设默认值\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
std::string Config::ReadString(const char *section, const char *item,
                               const char *default_value) const
{
    std::string tmp_s(section);
    std::string tmp_i(item);
    std::string def(default_value);
    std::map<std::string, std::string> k_v;
    std::map<std::string, std::string>::iterator it_item;
    std::map<std::string, std::map<std::string, std::string> >::const_iterator it;
    it = settings_.find(tmp_s);
    if(it == settings_.end())
    {
        //printf("111");
        return def;
    }
    k_v = it->second;
    it_item = k_v.find(tmp_i);
    if(it_item == k_v.end())
    {
        //printf("222");
        return def;
    }
    return it_item->second;
}

/**@brief          读取类型为int的参数
 * @param[in]      section              要查找的块
 * @param[in]      item                 要查找的参数项
 * @param[in]      default_value        该参数项的预设默认值
 * @return         返回结果:\n
 * - 如果配置文件中该参数有值       返回读取到的值\n
 * - 该参数值为空                 返回预设默认值\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
int Config::ReadInt(const char *section, const char *item,
                    const int &default_value) const
{
    std::string tmp_s(section);
    std::string tmp_i(item);
    std::map<std::string, std::string> k_v;
    std::map<std::string, std::string>::iterator it_item;
    std::map<std::string, std::map<std::string, std::string> >::const_iterator it;
    it = settings_.find(tmp_s);
    if(it == settings_.end())
    {
        return default_value;
    }
    k_v = it->second;
    it_item = k_v.find(tmp_i);
    if(it_item == k_v.end())
    {
        return default_value;
    }
    return atoi(it_item->second.c_str());
}

/**@brief          读取类型为float的参数
 * @param[in]      section              要查找的块
 * @param[in]      item                 要查找的参数项
 * @param[in]      default_value        该参数项的预设默认值
 * @return         返回结果:\n
 * - 如果配置文件中该参数有值       返回读取到的值\n
 * - 该参数值为空                 返回预设默认值\n
 * @author      Zing Fong
 * @date        2022/5/29
 */
float Config::ReadFloat(const char *section, const char *item,
                        const float &default_value) const
{
    std::string tmp_s(section);
    std::string tmp_i(item);
    std::map<std::string, std::string> k_v;
    std::map<std::string, std::string>::iterator it_item;
    std::map<std::string, std::map<std::string, std::string> >::const_iterator it;
    it = settings_.find(tmp_s);
    if(it == settings_.end())
    {
        return default_value;
    }
    k_v = it->second;
    it_item = k_v.find(tmp_i);
    if(it_item == k_v.end())
    {
        return default_value;
    }
    return atof(it_item->second.c_str());
}
