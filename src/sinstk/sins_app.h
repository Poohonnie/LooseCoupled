/**@file    sins_app.h
 * @brief   惯导应用程序.h文件
 * @details 提供了惯导机械编排算法的demo
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/6/15
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/15    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */

#ifndef LOOSECOUPLED_SRC_SINSTK_SINS_APP_H
#define LOOSECOUPLED_SRC_SINSTK_SINS_APP_H

// c/c++系统文件
// 其他库的 .h 文件

// 本项目内 .h 文件
#include "../basetk/base_math.h"
#include "../basetk/base_time.h"
#include "sins_file_stream.h"
#include "sins_mechanization.h"

class SinsApp
{
  public:
    void SinsMechanizationDemo();  // 测试用例
  
  private:
    
    SinsFileStream sins_file_stream_{};  // IMU文件流对象
    SinsMechanization sins_mechanization_{};  // IMU机械编排对象
    
};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_APP_H
