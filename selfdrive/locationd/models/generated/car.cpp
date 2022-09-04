#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6678050384484087612) {
   out_6678050384484087612[0] = delta_x[0] + nom_x[0];
   out_6678050384484087612[1] = delta_x[1] + nom_x[1];
   out_6678050384484087612[2] = delta_x[2] + nom_x[2];
   out_6678050384484087612[3] = delta_x[3] + nom_x[3];
   out_6678050384484087612[4] = delta_x[4] + nom_x[4];
   out_6678050384484087612[5] = delta_x[5] + nom_x[5];
   out_6678050384484087612[6] = delta_x[6] + nom_x[6];
   out_6678050384484087612[7] = delta_x[7] + nom_x[7];
   out_6678050384484087612[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6915426896439217469) {
   out_6915426896439217469[0] = -nom_x[0] + true_x[0];
   out_6915426896439217469[1] = -nom_x[1] + true_x[1];
   out_6915426896439217469[2] = -nom_x[2] + true_x[2];
   out_6915426896439217469[3] = -nom_x[3] + true_x[3];
   out_6915426896439217469[4] = -nom_x[4] + true_x[4];
   out_6915426896439217469[5] = -nom_x[5] + true_x[5];
   out_6915426896439217469[6] = -nom_x[6] + true_x[6];
   out_6915426896439217469[7] = -nom_x[7] + true_x[7];
   out_6915426896439217469[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6424634700770478609) {
   out_6424634700770478609[0] = 1.0;
   out_6424634700770478609[1] = 0;
   out_6424634700770478609[2] = 0;
   out_6424634700770478609[3] = 0;
   out_6424634700770478609[4] = 0;
   out_6424634700770478609[5] = 0;
   out_6424634700770478609[6] = 0;
   out_6424634700770478609[7] = 0;
   out_6424634700770478609[8] = 0;
   out_6424634700770478609[9] = 0;
   out_6424634700770478609[10] = 1.0;
   out_6424634700770478609[11] = 0;
   out_6424634700770478609[12] = 0;
   out_6424634700770478609[13] = 0;
   out_6424634700770478609[14] = 0;
   out_6424634700770478609[15] = 0;
   out_6424634700770478609[16] = 0;
   out_6424634700770478609[17] = 0;
   out_6424634700770478609[18] = 0;
   out_6424634700770478609[19] = 0;
   out_6424634700770478609[20] = 1.0;
   out_6424634700770478609[21] = 0;
   out_6424634700770478609[22] = 0;
   out_6424634700770478609[23] = 0;
   out_6424634700770478609[24] = 0;
   out_6424634700770478609[25] = 0;
   out_6424634700770478609[26] = 0;
   out_6424634700770478609[27] = 0;
   out_6424634700770478609[28] = 0;
   out_6424634700770478609[29] = 0;
   out_6424634700770478609[30] = 1.0;
   out_6424634700770478609[31] = 0;
   out_6424634700770478609[32] = 0;
   out_6424634700770478609[33] = 0;
   out_6424634700770478609[34] = 0;
   out_6424634700770478609[35] = 0;
   out_6424634700770478609[36] = 0;
   out_6424634700770478609[37] = 0;
   out_6424634700770478609[38] = 0;
   out_6424634700770478609[39] = 0;
   out_6424634700770478609[40] = 1.0;
   out_6424634700770478609[41] = 0;
   out_6424634700770478609[42] = 0;
   out_6424634700770478609[43] = 0;
   out_6424634700770478609[44] = 0;
   out_6424634700770478609[45] = 0;
   out_6424634700770478609[46] = 0;
   out_6424634700770478609[47] = 0;
   out_6424634700770478609[48] = 0;
   out_6424634700770478609[49] = 0;
   out_6424634700770478609[50] = 1.0;
   out_6424634700770478609[51] = 0;
   out_6424634700770478609[52] = 0;
   out_6424634700770478609[53] = 0;
   out_6424634700770478609[54] = 0;
   out_6424634700770478609[55] = 0;
   out_6424634700770478609[56] = 0;
   out_6424634700770478609[57] = 0;
   out_6424634700770478609[58] = 0;
   out_6424634700770478609[59] = 0;
   out_6424634700770478609[60] = 1.0;
   out_6424634700770478609[61] = 0;
   out_6424634700770478609[62] = 0;
   out_6424634700770478609[63] = 0;
   out_6424634700770478609[64] = 0;
   out_6424634700770478609[65] = 0;
   out_6424634700770478609[66] = 0;
   out_6424634700770478609[67] = 0;
   out_6424634700770478609[68] = 0;
   out_6424634700770478609[69] = 0;
   out_6424634700770478609[70] = 1.0;
   out_6424634700770478609[71] = 0;
   out_6424634700770478609[72] = 0;
   out_6424634700770478609[73] = 0;
   out_6424634700770478609[74] = 0;
   out_6424634700770478609[75] = 0;
   out_6424634700770478609[76] = 0;
   out_6424634700770478609[77] = 0;
   out_6424634700770478609[78] = 0;
   out_6424634700770478609[79] = 0;
   out_6424634700770478609[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1961072051082692586) {
   out_1961072051082692586[0] = state[0];
   out_1961072051082692586[1] = state[1];
   out_1961072051082692586[2] = state[2];
   out_1961072051082692586[3] = state[3];
   out_1961072051082692586[4] = state[4];
   out_1961072051082692586[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1961072051082692586[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1961072051082692586[7] = state[7];
   out_1961072051082692586[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3343064474476800318) {
   out_3343064474476800318[0] = 1;
   out_3343064474476800318[1] = 0;
   out_3343064474476800318[2] = 0;
   out_3343064474476800318[3] = 0;
   out_3343064474476800318[4] = 0;
   out_3343064474476800318[5] = 0;
   out_3343064474476800318[6] = 0;
   out_3343064474476800318[7] = 0;
   out_3343064474476800318[8] = 0;
   out_3343064474476800318[9] = 0;
   out_3343064474476800318[10] = 1;
   out_3343064474476800318[11] = 0;
   out_3343064474476800318[12] = 0;
   out_3343064474476800318[13] = 0;
   out_3343064474476800318[14] = 0;
   out_3343064474476800318[15] = 0;
   out_3343064474476800318[16] = 0;
   out_3343064474476800318[17] = 0;
   out_3343064474476800318[18] = 0;
   out_3343064474476800318[19] = 0;
   out_3343064474476800318[20] = 1;
   out_3343064474476800318[21] = 0;
   out_3343064474476800318[22] = 0;
   out_3343064474476800318[23] = 0;
   out_3343064474476800318[24] = 0;
   out_3343064474476800318[25] = 0;
   out_3343064474476800318[26] = 0;
   out_3343064474476800318[27] = 0;
   out_3343064474476800318[28] = 0;
   out_3343064474476800318[29] = 0;
   out_3343064474476800318[30] = 1;
   out_3343064474476800318[31] = 0;
   out_3343064474476800318[32] = 0;
   out_3343064474476800318[33] = 0;
   out_3343064474476800318[34] = 0;
   out_3343064474476800318[35] = 0;
   out_3343064474476800318[36] = 0;
   out_3343064474476800318[37] = 0;
   out_3343064474476800318[38] = 0;
   out_3343064474476800318[39] = 0;
   out_3343064474476800318[40] = 1;
   out_3343064474476800318[41] = 0;
   out_3343064474476800318[42] = 0;
   out_3343064474476800318[43] = 0;
   out_3343064474476800318[44] = 0;
   out_3343064474476800318[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3343064474476800318[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3343064474476800318[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3343064474476800318[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3343064474476800318[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3343064474476800318[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3343064474476800318[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3343064474476800318[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3343064474476800318[53] = -9.8000000000000007*dt;
   out_3343064474476800318[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3343064474476800318[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3343064474476800318[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3343064474476800318[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3343064474476800318[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3343064474476800318[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3343064474476800318[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3343064474476800318[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3343064474476800318[62] = 0;
   out_3343064474476800318[63] = 0;
   out_3343064474476800318[64] = 0;
   out_3343064474476800318[65] = 0;
   out_3343064474476800318[66] = 0;
   out_3343064474476800318[67] = 0;
   out_3343064474476800318[68] = 0;
   out_3343064474476800318[69] = 0;
   out_3343064474476800318[70] = 1;
   out_3343064474476800318[71] = 0;
   out_3343064474476800318[72] = 0;
   out_3343064474476800318[73] = 0;
   out_3343064474476800318[74] = 0;
   out_3343064474476800318[75] = 0;
   out_3343064474476800318[76] = 0;
   out_3343064474476800318[77] = 0;
   out_3343064474476800318[78] = 0;
   out_3343064474476800318[79] = 0;
   out_3343064474476800318[80] = 1;
}
void h_25(double *state, double *unused, double *out_8441344765247916904) {
   out_8441344765247916904[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7543773704230038944) {
   out_7543773704230038944[0] = 0;
   out_7543773704230038944[1] = 0;
   out_7543773704230038944[2] = 0;
   out_7543773704230038944[3] = 0;
   out_7543773704230038944[4] = 0;
   out_7543773704230038944[5] = 0;
   out_7543773704230038944[6] = 1;
   out_7543773704230038944[7] = 0;
   out_7543773704230038944[8] = 0;
}
void h_24(double *state, double *unused, double *out_7608609832333167846) {
   out_7608609832333167846[0] = state[4];
   out_7608609832333167846[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5629476086566500078) {
   out_5629476086566500078[0] = 0;
   out_5629476086566500078[1] = 0;
   out_5629476086566500078[2] = 0;
   out_5629476086566500078[3] = 0;
   out_5629476086566500078[4] = 1;
   out_5629476086566500078[5] = 0;
   out_5629476086566500078[6] = 0;
   out_5629476086566500078[7] = 0;
   out_5629476086566500078[8] = 0;
   out_5629476086566500078[9] = 0;
   out_5629476086566500078[10] = 0;
   out_5629476086566500078[11] = 0;
   out_5629476086566500078[12] = 0;
   out_5629476086566500078[13] = 0;
   out_5629476086566500078[14] = 1;
   out_5629476086566500078[15] = 0;
   out_5629476086566500078[16] = 0;
   out_5629476086566500078[17] = 0;
}
void h_30(double *state, double *unused, double *out_8716538827532422793) {
   out_8716538827532422793[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3016077374102430746) {
   out_3016077374102430746[0] = 0;
   out_3016077374102430746[1] = 0;
   out_3016077374102430746[2] = 0;
   out_3016077374102430746[3] = 0;
   out_3016077374102430746[4] = 1;
   out_3016077374102430746[5] = 0;
   out_3016077374102430746[6] = 0;
   out_3016077374102430746[7] = 0;
   out_3016077374102430746[8] = 0;
}
void h_26(double *state, double *unused, double *out_3714865559871577148) {
   out_3714865559871577148[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3802270385355982720) {
   out_3802270385355982720[0] = 0;
   out_3802270385355982720[1] = 0;
   out_3802270385355982720[2] = 0;
   out_3802270385355982720[3] = 0;
   out_3802270385355982720[4] = 0;
   out_3802270385355982720[5] = 0;
   out_3802270385355982720[6] = 0;
   out_3802270385355982720[7] = 1;
   out_3802270385355982720[8] = 0;
}
void h_27(double *state, double *unused, double *out_4779492092626309765) {
   out_4779492092626309765[0] = state[3];
}
void H_27(double *state, double *unused, double *out_841314062302005835) {
   out_841314062302005835[0] = 0;
   out_841314062302005835[1] = 0;
   out_841314062302005835[2] = 0;
   out_841314062302005835[3] = 1;
   out_841314062302005835[4] = 0;
   out_841314062302005835[5] = 0;
   out_841314062302005835[6] = 0;
   out_841314062302005835[7] = 0;
   out_841314062302005835[8] = 0;
}
void h_29(double *state, double *unused, double *out_4504298030341803876) {
   out_4504298030341803876[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3526308718416822930) {
   out_3526308718416822930[0] = 0;
   out_3526308718416822930[1] = 1;
   out_3526308718416822930[2] = 0;
   out_3526308718416822930[3] = 0;
   out_3526308718416822930[4] = 0;
   out_3526308718416822930[5] = 0;
   out_3526308718416822930[6] = 0;
   out_3526308718416822930[7] = 0;
   out_3526308718416822930[8] = 0;
}
void h_28(double *state, double *unused, double *out_1098793394932405323) {
   out_1098793394932405323[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5489938989982149181) {
   out_5489938989982149181[0] = 1;
   out_5489938989982149181[1] = 0;
   out_5489938989982149181[2] = 0;
   out_5489938989982149181[3] = 0;
   out_5489938989982149181[4] = 0;
   out_5489938989982149181[5] = 0;
   out_5489938989982149181[6] = 0;
   out_5489938989982149181[7] = 0;
   out_5489938989982149181[8] = 0;
}
void h_31(double *state, double *unused, double *out_4803895236993894554) {
   out_4803895236993894554[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3176062283122631244) {
   out_3176062283122631244[0] = 0;
   out_3176062283122631244[1] = 0;
   out_3176062283122631244[2] = 0;
   out_3176062283122631244[3] = 0;
   out_3176062283122631244[4] = 0;
   out_3176062283122631244[5] = 0;
   out_3176062283122631244[6] = 0;
   out_3176062283122631244[7] = 0;
   out_3176062283122631244[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6678050384484087612) {
  err_fun(nom_x, delta_x, out_6678050384484087612);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6915426896439217469) {
  inv_err_fun(nom_x, true_x, out_6915426896439217469);
}
void car_H_mod_fun(double *state, double *out_6424634700770478609) {
  H_mod_fun(state, out_6424634700770478609);
}
void car_f_fun(double *state, double dt, double *out_1961072051082692586) {
  f_fun(state,  dt, out_1961072051082692586);
}
void car_F_fun(double *state, double dt, double *out_3343064474476800318) {
  F_fun(state,  dt, out_3343064474476800318);
}
void car_h_25(double *state, double *unused, double *out_8441344765247916904) {
  h_25(state, unused, out_8441344765247916904);
}
void car_H_25(double *state, double *unused, double *out_7543773704230038944) {
  H_25(state, unused, out_7543773704230038944);
}
void car_h_24(double *state, double *unused, double *out_7608609832333167846) {
  h_24(state, unused, out_7608609832333167846);
}
void car_H_24(double *state, double *unused, double *out_5629476086566500078) {
  H_24(state, unused, out_5629476086566500078);
}
void car_h_30(double *state, double *unused, double *out_8716538827532422793) {
  h_30(state, unused, out_8716538827532422793);
}
void car_H_30(double *state, double *unused, double *out_3016077374102430746) {
  H_30(state, unused, out_3016077374102430746);
}
void car_h_26(double *state, double *unused, double *out_3714865559871577148) {
  h_26(state, unused, out_3714865559871577148);
}
void car_H_26(double *state, double *unused, double *out_3802270385355982720) {
  H_26(state, unused, out_3802270385355982720);
}
void car_h_27(double *state, double *unused, double *out_4779492092626309765) {
  h_27(state, unused, out_4779492092626309765);
}
void car_H_27(double *state, double *unused, double *out_841314062302005835) {
  H_27(state, unused, out_841314062302005835);
}
void car_h_29(double *state, double *unused, double *out_4504298030341803876) {
  h_29(state, unused, out_4504298030341803876);
}
void car_H_29(double *state, double *unused, double *out_3526308718416822930) {
  H_29(state, unused, out_3526308718416822930);
}
void car_h_28(double *state, double *unused, double *out_1098793394932405323) {
  h_28(state, unused, out_1098793394932405323);
}
void car_H_28(double *state, double *unused, double *out_5489938989982149181) {
  H_28(state, unused, out_5489938989982149181);
}
void car_h_31(double *state, double *unused, double *out_4803895236993894554) {
  h_31(state, unused, out_4803895236993894554);
}
void car_H_31(double *state, double *unused, double *out_3176062283122631244) {
  H_31(state, unused, out_3176062283122631244);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
