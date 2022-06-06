/**@file    gnss_pos.h
 * @brief   pos文件的读取.h文件
 * @details 读取pos文件并保存位置, 以供松组合解算使用
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/29
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/6/6     <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_GNSSTK_GNSS_POS_H
#define LOOSECOUPLED_SRC_GNSSTK_GNSS_POS_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_app.h"

/**@class   GnssPos
 * @brief   读取GNSSpos文件, 并将结果保存起来
 * @par     修改日志:
 * <table>
 * <tr><th>Date         <th>Author      <th>Description
 * <tr><td>2022/6/6     <td>Zing Fong   <td>Initialize
 * </table>
 */
class GnssPos
{
  public:
    int Init(const Config &config);  // 初始化, 打开文件等
    
    int ReadOneSec();  // 读取一秒钟的观测值
  
    // get
    double get_t() const;
    std::vector<double> get_pos() const;
  private:
    double t_{};  // GPS周秒
    std::vector<double> pos_ = std::vector<double>(3, 0.0);  // 当前历元的位置
    
    FILE *file_ptr_{};  // 文件指针
};


#endif //LOOSECOUPLED_SRC_GNSSTK_GNSS_POS_H
