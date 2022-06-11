/**@file    tester.h
 * @brief   测试器
 * @details 实现了项目中各类对应的测试类
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/10
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/10    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

#ifndef LOOSECOUPLED_SRC_TESTER_H
#define LOOSECOUPLED_SRC_TESTER_H

// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <random>

// 本项目内 .h 文件
#include "basetk/base_math.h"

/**@class   BaseMathTester
 * @brief   BaseMath类的测试类
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/5/31    <td>Zing Fong   <td>Initialize
 * </table>
 */
class BaseMathTester
{
  public:
    static void MaxAndMinTester();  // 最大最小值测试器
    static void CoordinateTransformationTester();  // 坐标转换测试器
    static void AttitudeTransformationTester();  // 姿态转换函数测试器
    
  private:
  
};



class Tester
{

};


#endif //LOOSECOUPLED_SRC_TESTER_H
