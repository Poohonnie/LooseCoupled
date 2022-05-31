/**@file    sins_loose_coupled.h
 * @brief   GNSS/INS松组合解算
 * @details 输入RTK和惯导机械编排的位姿信息, 进行松组合解算
 * @author  Zing Fong\n
 *          zing.fong.whu\@outlook.com
 * @date    2022/5/31
 * @version V1.0
 **********************************************************************************
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version  <th>Author     <th>Description
 * <tr><td>2022/5/31    <td>1.0      <td>Zing Fong  <td>Initialize
 * </table>
 **********************************************************************************
 */
#ifndef LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H
#define LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H

// c/c++系统文件
#include <iostream>

// 其他库的 .h 文件
#include <vector>

// 本项目内 .h 文件
#include "../basetk/base_math.h"
#include "../basetk/base_time.h"
#include "../basetk/base_app.h"
#include "sins_file_stream.h"
#include "sins_mechanization.h"


class SinsLooseCoupled
{


};


#endif //LOOSECOUPLED_SRC_SINSTK_SINS_LOOSE_COUPLED_H
