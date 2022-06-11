/**@mainpage GNSS/INS松组合解算程序
 * <table>
 * <tr><th>Project  <td>LooseCoupled
 * <tr><th>Author   <td>Zing Fong
 * <tr><th>Source   <td>https://github.com/Poohonnie/LooseCoupled
 * </table>
 * @section   项目详细描述
 * 通过读取GNSS pos文件，获取载体的ECEF坐标。再通过读取SINS imr文件，与GNSS解算结果进行松组合解算。
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
 * <tr><th>Date         <th>Version         <th>Author      <th>Description </tr>
 * <tr><td>2022/6/7     <td>1.0             <td>Zing Fong   <td>创建初始版本  </tr>
 * </table>
 **********************************************************************************
 */

#include <iostream>
#include "basetk/base_app.h"
#include "basetk/base_matrix.h"
#include "basetk/base_math.h"
#include "tester.h"
#include <fstream>

int main()
{
    std::vector<double> q{0.5, 0.5, 0.5, 0.5};
    std::vector<double> rotation_vec = BaseMath::Quaternion2RotationVec(q);
    std::cout << rotation_vec[0] << ' ' << rotation_vec[1] << ' ' << rotation_vec[2] << std::endl;
    q = BaseMath::RotationVec2Quaternion(rotation_vec);
    printf("%f %f %f %f\n", q[0], q[1], q[2], q[3]);
//    Config config{};
//    bool ret = config.ReadConfig("config.ini");
//    if (!ret)
//    {
//        return 1;
//    }
//    std::string HostName = config.ReadString("MYSQL", "HostName", "");
//    int Port = config.ReadInt("MYSQL", "Port", 0);
//    std::string UserName = config.ReadString("MYSQL", "UserName", "");
//
//    std::cout << "HostName=" << HostName << std::endl;
//    std::cout << "Port=" << Port << std::endl;
//    std::cout << "UserName=" << UserName << std::endl;
//
//    system("Pause");
    return 0;
}

