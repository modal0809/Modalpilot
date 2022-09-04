#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9052257119879677051);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9030534445731092372);
void gnss_H_mod_fun(double *state, double *out_2627594616369667156);
void gnss_f_fun(double *state, double dt, double *out_904766291043479982);
void gnss_F_fun(double *state, double dt, double *out_5659544920538399114);
void gnss_h_6(double *state, double *sat_pos, double *out_6682955905749598678);
void gnss_H_6(double *state, double *sat_pos, double *out_4085749273610108535);
void gnss_h_20(double *state, double *sat_pos, double *out_8912925592750663323);
void gnss_H_20(double *state, double *sat_pos, double *out_466141655324552523);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6302367303281942640);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7349408904144907925);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6302367303281942640);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7349408904144907925);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}