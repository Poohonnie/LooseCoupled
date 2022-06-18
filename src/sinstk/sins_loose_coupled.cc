/**@file    sins_loose_coupled.cc
 * @brief   GNSS/INS松组合解算
 * @details 输入RTK和惯导机械编排的位姿信息, 进行松组合解算
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

// 本类对应的.h文件
#include "sins_loose_coupled.h"
// c/c++系统文件
#include <iostream>
// 其他库的 .h 文件
#include <vector>
#include <cmath>

// 本项目内 .h 文件

void SinsLooseCoupled::Predict(const ImuData &imu_data)
{

}

void SinsLooseCoupled::Update(const ImuData &imu_data,
                              const StateInfo &gnss_state)
{

}

/**@brief       F阵的计算
 * @param[in]   imu_data        惯性传感器读数
 * @return      F矩阵
 * @author      Zing Fong
 * @date        2022/6/18
 */
BaseMatrix SinsLooseCoupled::CalcF(const ImuData &imu_data)
{
    BaseMatrix F(21, 21);  // 21×21维矩阵, 因为状态是21×1维
    auto frr = CalcFrr();  // Frr阵, 3×3维
    auto fvr = CalcFvr();  // Fvr阵, 3×3维
    auto fphir = CalcFphir();  // Fφr阵, 3×3维
    auto fvv = CalcFvv();  // Fvv阵, 3×3维
    auto fphiv = CalcFphiv();  // Fφr阵, 3×3维
    
    const auto &c_b_n = sins_mechanization_.get_cur_state().c_b_n;
    const auto &f_b = imu_data.acc;
    auto omega_ib_b = imu_data.gyro;  // omega_ib_b = gyro / delta_t
    for(auto &a_omega: omega_ib_b)
        a_omega /= sins_mechanization_.get_delta_t();
    
    BaseMatrix mat_fb(f_b, 3, 1);
    const auto &omega_in_n = sins_mechanization_.get_omega_in_n();
    
    // 向F矩阵中写入3×3矩阵的函数
    auto WriteF = [&F](int row_begin, int col_begin, const BaseMatrix &sub_mat)
    {
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                F.write(row_begin + i, col_begin + j, sub_mat.read(i, j));
    };
    
    // (0:2, 0:2) Frr
    WriteF(0, 0, frr);
    // (0:2, 3:5) I3×3
    WriteF(0, 3, BaseMatrix::eye(3));
    // (3:5, 0:2) Fvr
    WriteF(3, 0, fvr);
    // (3:5, 3:5) Fvv
    WriteF(3, 3, fvv);
    // (3:5, 6:8) (Cbn*fb)×, Cbn和fb相乘的反对称阵
    auto product_cbn_fb = c_b_n*mat_fb;
    auto antisymmetry_cbn_fb = BaseMatrix::CalcAntisymmetryMat(
            product_cbn_fb.get_mat());
    WriteF(3, 6, antisymmetry_cbn_fb);
    
    // (3:5, 12:14) Cbn
    WriteF(3, 12, c_b_n);
    // (3:5, 18:20) Cbn*diag(fb)
    auto product_cbn_diagfb = c_b_n*BaseMatrix::Diag(f_b);
    WriteF(3, 18, product_cbn_diagfb);
    // (6:8, 0:2) Fφr
    WriteF(6, 0, fphir);
    // (6:8, 3:5) Fφv
    WriteF(6, 3, fphiv);
    // (6:8, 6:8) -(omega_in_n×)
    auto neg_antisymmetry_omega =
            BaseMatrix::CalcAntisymmetryMat(omega_in_n)*-1.0;
    WriteF(6, 6, neg_antisymmetry_omega);
    // (6:8, 9:11) -Cbn
    WriteF(6, 9, c_b_n*-1.0);
    // (6:8, 15:17) -Cbn*diag(omega_ib_b)
    auto neg_product_cbn_diag_omega = c_b_n*BaseMatrix::Diag(omega_ib_b)*-1.0;
    
    // Tgb, Tab, Tgs, Tas 一阶高斯马尔科夫过程相关事件, 都设为3600s
    double t_gb = 3600.0, t_ab = 3600.0, t_gs = 3600.0, t_as = 3600.0;
    // (9:11, 9:11) (12:14, 12:14) (15:17, 15:17) (18:20, 18:20)
    WriteF(9, 9, BaseMatrix::eye(3)*(-1.0/t_gb));
    WriteF(12, 12, BaseMatrix::eye(3)*(-1.0/t_ab));
    WriteF(15, 15, BaseMatrix::eye(3)*(-1.0/t_gs));
    WriteF(18, 18, BaseMatrix::eye(3)*(-1.0/t_as));
    
    return F;
}

/**@brief       Frr阵的计算
 * @author      Zing Fong
 * @date        2022/6/16
 */
BaseMatrix SinsLooseCoupled::CalcFrr()
{
    BaseMatrix frr(3, 3);
    // 需要用到的量
    auto state = sins_mechanization_.get_cur_state();
    auto v_ned = state.v_ned;
    const double &vn = v_ned[0], &ve = v_ned[1], &vd = v_ned[2];
    auto blh = state.blh;
    const double &b = blh[0], &l = blh[1], &h = blh[2];
    auto rm = sins_mechanization_.get_r_m();
    auto rn = sins_mechanization_.get_r_n();
    const double &omega_e = BaseSdc::wgs84.kOmega;
    
    frr.write(0, 0, -vd/(rm + h));
    frr.write(0, 1, 0);
    frr.write(0, 2, vn/(rm + h));
    
    frr.write(1, 0, ve*tan(b)/(rn + h));
    frr.write(1, 1, -(vd + vn*tan(b))/(rn + h));
    frr.write(1, 2, ve/(rn + h));
    // 剩下一行全为0, 不用管
    
    return frr;
}

/**@brief       Fvr阵的计算
 * @return      Fvr矩阵
 * @author      Zing Fong
 * @date        2022/6/16
 */
BaseMatrix SinsLooseCoupled::CalcFvr()
{
    BaseMatrix fvr(3, 3);
    // 需要用到的量
    auto state = sins_mechanization_.get_cur_state();
    auto v_ned = state.v_ned;
    const double &vn = v_ned[0], &ve = v_ned[1], &vd = v_ned[2];
    auto blh = state.blh;
    const double &b = blh[0], &l = blh[1], &h = blh[2];
    auto rm = sins_mechanization_.get_r_m();
    auto rn = sins_mechanization_.get_r_n();
    auto g_n = sins_mechanization_.get_g_n();
    const double &gp = g_n[2];
    const double &omega_e = BaseSdc::wgs84.kOmega;
    
    fvr.write(0, 0, (-2*ve*omega_e*cos(b))/(rm + h) -
                    (ve/cos(b))*(ve/cos(b)/((rm + h)*(rn + h))));
    fvr.write(0, 1, 0);
    fvr.write(0, 2, vn*vd/((rm + h)*(rm + h)) -
                    (ve*ve*tan(b))/((rn + h)*(rn + h)));
    
    fvr.write(1, 0, 2*omega_e*(vn*cos(b) - vd*sin(b))/
                    (rm + h) + vn*ve/(cos(b)*cos(b))/((rm + h)*(rn + h)));
    fvr.write(1, 1, 0);
    fvr.write(1, 2, (ve*vd + vn*ve*tan(b))/((rn + h)*(rn + h)));
    
    fvr.write(2, 0, 2*omega_e*ve*sin(b)/(rm + h));
    fvr.write(2, 1, 0);
    fvr.write(2, 2, -ve*ve/((rn + h)*(rn + h)) -
                    vn*vn/((rm + h)*(rm + h)) + 2*gp/(sqrt(rm*rn) + h));
    
    return fvr;
}

/**@brief       F阵的计算
 * @return      Fφv矩阵
 * @author      Zing Fong
 * @date        2022/6/16
 */
BaseMatrix SinsLooseCoupled::CalcFphir()
{
    BaseMatrix fphir(3, 3);
    // 需要用到的量
    auto state = sins_mechanization_.get_cur_state();
    auto v_ned = state.v_ned;
    const double &vn = v_ned[0], &ve = v_ned[1], &vd = v_ned[2];
    auto blh = state.blh;
    const double &b = blh[0], &l = blh[1], &h = blh[2];
    auto rm = sins_mechanization_.get_r_m();
    auto rn = sins_mechanization_.get_r_n();
    const double &omega_e = BaseSdc::wgs84.kOmega;
    
    // 写入矩阵
    fphir.write(0, 0, -omega_e*sin(b)/(rm + h));
    fphir.write(0, 1, 0);
    fphir.write(0, 2, ve/((rn + h)*(rn + h)));
    
    fphir.write(1, 0, 0);
    fphir.write(1, 1, 0);
    fphir.write(1, 2, -vn/((rm + h)*(rm + h)));
    
    fphir.write(2, 0, -omega_e*cos(b)/(rm + h) -
                      ve/(cos(b)*cos(b))/((rm + h)*(rn + h)));
    fphir.write(2, 1, 0);
    fphir.write(2, 2, -ve*tan(b)/((rn + h)*(rn + h)));
    
    return fphir;
}

/**@brief       Fvv阵的计算
 * @return      Fvv矩阵
 * @author      Zing Fong
 * @date        2022/6/16
 */
BaseMatrix SinsLooseCoupled::CalcFvv()
{
    BaseMatrix fvv(3, 3);
    // 需要用到的量
    auto state = sins_mechanization_.get_cur_state();
    auto v_ned = state.v_ned;
    const double &vn = v_ned[0], &ve = v_ned[1], &vd = v_ned[2];
    auto blh = state.blh;
    const double &b = blh[0], &l = blh[1], &h = blh[2];
    auto rm = sins_mechanization_.get_r_m();
    auto rn = sins_mechanization_.get_r_n();
    const double &omega_e = BaseSdc::wgs84.kOmega;
    
    // 写矩阵
    fvv.write(0, 0, vd/(rm + h));
    fvv.write(0, 1, -2*(omega_e*sin(b) + ve*tan(b)/(rn + h)));
    fvv.write(0, 2, vn/(rm + h));
    
    fvv.write(1, 0, 2*omega_e*sin(b) + ve*tan(b)*(rn + h));
    fvv.write(1, 1, (vd + vn*tan(b))/(rn + h));
    fvv.write(1, 2, 2*omega_e*cos(b) + ve/(rn + h));
    
    fvv.write(2, 0, -2*vn/(rm + h));
    fvv.write(2, 1, -2*(omega_e*cos(b) + ve/(rn + h)));
    fvv.write(2, 2, 0);
    
    return fvv;
}

/**@brief       Fphiv阵的计算
 * @return      Fphiv矩阵
 * @author      Zing Fong
 * @date        2022/6/16
 */
BaseMatrix SinsLooseCoupled::CalcFphiv()
{
    BaseMatrix fphiv(3, 3);
    
    // 需要用到的量
    auto state = sins_mechanization_.get_cur_state();
    auto v_ned = state.v_ned;
    const double &vn = v_ned[0], &ve = v_ned[1], &vd = v_ned[2];
    auto blh = state.blh;
    const double &b = blh[0], &l = blh[1], &h = blh[2];
    auto rm = sins_mechanization_.get_r_m();
    auto rn = sins_mechanization_.get_r_n();
    const double &omega_e = BaseSdc::wgs84.kOmega;
    
    // 写入矩阵
    fphiv.write(0, 0, 0);
    fphiv.write(0, 1, 1.0/(rn + h));
    fphiv.write(0, 2, 0);
    
    fphiv.write(1, 0, -1.0/(rm + h));
    fphiv.write(1, 1, 0);
    fphiv.write(1, 2, 0);
    
    fphiv.write(2, 0, 0);
    fphiv.write(2, 1, -tan(b)/(rn + h));
    fphiv.write(2, 2, 0);
    
    return fphiv;
}
