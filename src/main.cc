/**@mainpage GNSS/INS松组合解算程序
 * <table>
 * <tr><th>Project  <td>LooseCoupled
 * <tr><th>Author   <td>Zing Fong
 * <tr><th>Source   <td>https://github.com/Poohonnie/LooseCoupled
 * </table>
 * @section   项目详细描述
 * 通过读取GNSS o文件和p文件，进行RTK解算。再通过读取SINS imr文件，利用结算出来的RTK结果，进行松组合解算。
 *
 * @section   功能描述  
 * -# 功能1
 * -# 功能2
 *
 * @section   用法描述 
 * -# 用法1
 * -# 用法2
 * 
 * @section   软件更新
 * <table>
 * <tr><th>Date        <th>Version    <th>Author      <th>Description </tr>
 * <tr><td>2018/08/17  <td>1.0          <td>Zing Fong   <td>创建初始版本  </tr>
 * </table>
 **********************************************************************************
 */
#include <iostream>

#include <iostream>
#include "basetk/base_app.h"
#include <fstream>
#include <cassert>

int main()
{
    Config config{};
    bool ret = config.ReadConfig("config.ini.txt");
    if (!ret)
    {
        return 1;
    }
    std::string HostName = config.ReadString("MYSQL", "HostName", "");
    int Port = config.ReadInt("MYSQL", "Port", 0);
    std::string UserName = config.ReadString("MYSQL", "UserName", "");
    
    std::cout << "HostName=" << HostName << std::endl;
    std::cout << "Port=" << Port << std::endl;
    std::cout << "UserName=" << UserName << std::endl;
    
    system("Pause");
    return 0;
}

