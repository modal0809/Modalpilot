#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6678050384484087612);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6915426896439217469);
void car_H_mod_fun(double *state, double *out_6424634700770478609);
void car_f_fun(double *state, double dt, double *out_1961072051082692586);
void car_F_fun(double *state, double dt, double *out_3343064474476800318);
void car_h_25(double *state, double *unused, double *out_8441344765247916904);
void car_H_25(double *state, double *unused, double *out_7543773704230038944);
void car_h_24(double *state, double *unused, double *out_7608609832333167846);
void car_H_24(double *state, double *unused, double *out_5629476086566500078);
void car_h_30(double *state, double *unused, double *out_8716538827532422793);
void car_H_30(double *state, double *unused, double *out_3016077374102430746);
void car_h_26(double *state, double *unused, double *out_3714865559871577148);
void car_H_26(double *state, double *unused, double *out_3802270385355982720);
void car_h_27(double *state, double *unused, double *out_4779492092626309765);
void car_H_27(double *state, double *unused, double *out_841314062302005835);
void car_h_29(double *state, double *unused, double *out_4504298030341803876);
void car_H_29(double *state, double *unused, double *out_3526308718416822930);
void car_h_28(double *state, double *unused, double *out_1098793394932405323);
void car_H_28(double *state, double *unused, double *out_5489938989982149181);
void car_h_31(double *state, double *unused, double *out_4803895236993894554);
void car_H_31(double *state, double *unused, double *out_3176062283122631244);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}