#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_622721346943683685);
void live_err_fun(double *nom_x, double *delta_x, double *out_773547643175572684);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5115024931453247445);
void live_H_mod_fun(double *state, double *out_2117137093686112568);
void live_f_fun(double *state, double dt, double *out_3393411830256892093);
void live_F_fun(double *state, double dt, double *out_7303582802215372833);
void live_h_4(double *state, double *unused, double *out_4618606153516215947);
void live_H_4(double *state, double *unused, double *out_2869943118884432623);
void live_h_9(double *state, double *unused, double *out_8434652701568218375);
void live_H_9(double *state, double *unused, double *out_3111132765514023268);
void live_h_10(double *state, double *unused, double *out_8387767613653762802);
void live_H_10(double *state, double *unused, double *out_5440438466562031607);
void live_h_12(double *state, double *unused, double *out_5528931727572573892);
void live_H_12(double *state, double *unused, double *out_7889399526916394418);
void live_h_35(double *state, double *unused, double *out_4128968949380736912);
void live_H_35(double *state, double *unused, double *out_7811781514468143489);
void live_h_32(double *state, double *unused, double *out_1037016206198413416);
void live_H_32(double *state, double *unused, double *out_2010477142260530931);
void live_h_13(double *state, double *unused, double *out_3929842276086482884);
void live_H_13(double *state, double *unused, double *out_8738577330018849522);
void live_h_14(double *state, double *unused, double *out_8434652701568218375);
void live_H_14(double *state, double *unused, double *out_3111132765514023268);
void live_h_33(double *state, double *unused, double *out_2180366763639033166);
void live_H_33(double *state, double *unused, double *out_4661224509829285885);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}