#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9052257119879677051) {
   out_9052257119879677051[0] = delta_x[0] + nom_x[0];
   out_9052257119879677051[1] = delta_x[1] + nom_x[1];
   out_9052257119879677051[2] = delta_x[2] + nom_x[2];
   out_9052257119879677051[3] = delta_x[3] + nom_x[3];
   out_9052257119879677051[4] = delta_x[4] + nom_x[4];
   out_9052257119879677051[5] = delta_x[5] + nom_x[5];
   out_9052257119879677051[6] = delta_x[6] + nom_x[6];
   out_9052257119879677051[7] = delta_x[7] + nom_x[7];
   out_9052257119879677051[8] = delta_x[8] + nom_x[8];
   out_9052257119879677051[9] = delta_x[9] + nom_x[9];
   out_9052257119879677051[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9030534445731092372) {
   out_9030534445731092372[0] = -nom_x[0] + true_x[0];
   out_9030534445731092372[1] = -nom_x[1] + true_x[1];
   out_9030534445731092372[2] = -nom_x[2] + true_x[2];
   out_9030534445731092372[3] = -nom_x[3] + true_x[3];
   out_9030534445731092372[4] = -nom_x[4] + true_x[4];
   out_9030534445731092372[5] = -nom_x[5] + true_x[5];
   out_9030534445731092372[6] = -nom_x[6] + true_x[6];
   out_9030534445731092372[7] = -nom_x[7] + true_x[7];
   out_9030534445731092372[8] = -nom_x[8] + true_x[8];
   out_9030534445731092372[9] = -nom_x[9] + true_x[9];
   out_9030534445731092372[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2627594616369667156) {
   out_2627594616369667156[0] = 1.0;
   out_2627594616369667156[1] = 0;
   out_2627594616369667156[2] = 0;
   out_2627594616369667156[3] = 0;
   out_2627594616369667156[4] = 0;
   out_2627594616369667156[5] = 0;
   out_2627594616369667156[6] = 0;
   out_2627594616369667156[7] = 0;
   out_2627594616369667156[8] = 0;
   out_2627594616369667156[9] = 0;
   out_2627594616369667156[10] = 0;
   out_2627594616369667156[11] = 0;
   out_2627594616369667156[12] = 1.0;
   out_2627594616369667156[13] = 0;
   out_2627594616369667156[14] = 0;
   out_2627594616369667156[15] = 0;
   out_2627594616369667156[16] = 0;
   out_2627594616369667156[17] = 0;
   out_2627594616369667156[18] = 0;
   out_2627594616369667156[19] = 0;
   out_2627594616369667156[20] = 0;
   out_2627594616369667156[21] = 0;
   out_2627594616369667156[22] = 0;
   out_2627594616369667156[23] = 0;
   out_2627594616369667156[24] = 1.0;
   out_2627594616369667156[25] = 0;
   out_2627594616369667156[26] = 0;
   out_2627594616369667156[27] = 0;
   out_2627594616369667156[28] = 0;
   out_2627594616369667156[29] = 0;
   out_2627594616369667156[30] = 0;
   out_2627594616369667156[31] = 0;
   out_2627594616369667156[32] = 0;
   out_2627594616369667156[33] = 0;
   out_2627594616369667156[34] = 0;
   out_2627594616369667156[35] = 0;
   out_2627594616369667156[36] = 1.0;
   out_2627594616369667156[37] = 0;
   out_2627594616369667156[38] = 0;
   out_2627594616369667156[39] = 0;
   out_2627594616369667156[40] = 0;
   out_2627594616369667156[41] = 0;
   out_2627594616369667156[42] = 0;
   out_2627594616369667156[43] = 0;
   out_2627594616369667156[44] = 0;
   out_2627594616369667156[45] = 0;
   out_2627594616369667156[46] = 0;
   out_2627594616369667156[47] = 0;
   out_2627594616369667156[48] = 1.0;
   out_2627594616369667156[49] = 0;
   out_2627594616369667156[50] = 0;
   out_2627594616369667156[51] = 0;
   out_2627594616369667156[52] = 0;
   out_2627594616369667156[53] = 0;
   out_2627594616369667156[54] = 0;
   out_2627594616369667156[55] = 0;
   out_2627594616369667156[56] = 0;
   out_2627594616369667156[57] = 0;
   out_2627594616369667156[58] = 0;
   out_2627594616369667156[59] = 0;
   out_2627594616369667156[60] = 1.0;
   out_2627594616369667156[61] = 0;
   out_2627594616369667156[62] = 0;
   out_2627594616369667156[63] = 0;
   out_2627594616369667156[64] = 0;
   out_2627594616369667156[65] = 0;
   out_2627594616369667156[66] = 0;
   out_2627594616369667156[67] = 0;
   out_2627594616369667156[68] = 0;
   out_2627594616369667156[69] = 0;
   out_2627594616369667156[70] = 0;
   out_2627594616369667156[71] = 0;
   out_2627594616369667156[72] = 1.0;
   out_2627594616369667156[73] = 0;
   out_2627594616369667156[74] = 0;
   out_2627594616369667156[75] = 0;
   out_2627594616369667156[76] = 0;
   out_2627594616369667156[77] = 0;
   out_2627594616369667156[78] = 0;
   out_2627594616369667156[79] = 0;
   out_2627594616369667156[80] = 0;
   out_2627594616369667156[81] = 0;
   out_2627594616369667156[82] = 0;
   out_2627594616369667156[83] = 0;
   out_2627594616369667156[84] = 1.0;
   out_2627594616369667156[85] = 0;
   out_2627594616369667156[86] = 0;
   out_2627594616369667156[87] = 0;
   out_2627594616369667156[88] = 0;
   out_2627594616369667156[89] = 0;
   out_2627594616369667156[90] = 0;
   out_2627594616369667156[91] = 0;
   out_2627594616369667156[92] = 0;
   out_2627594616369667156[93] = 0;
   out_2627594616369667156[94] = 0;
   out_2627594616369667156[95] = 0;
   out_2627594616369667156[96] = 1.0;
   out_2627594616369667156[97] = 0;
   out_2627594616369667156[98] = 0;
   out_2627594616369667156[99] = 0;
   out_2627594616369667156[100] = 0;
   out_2627594616369667156[101] = 0;
   out_2627594616369667156[102] = 0;
   out_2627594616369667156[103] = 0;
   out_2627594616369667156[104] = 0;
   out_2627594616369667156[105] = 0;
   out_2627594616369667156[106] = 0;
   out_2627594616369667156[107] = 0;
   out_2627594616369667156[108] = 1.0;
   out_2627594616369667156[109] = 0;
   out_2627594616369667156[110] = 0;
   out_2627594616369667156[111] = 0;
   out_2627594616369667156[112] = 0;
   out_2627594616369667156[113] = 0;
   out_2627594616369667156[114] = 0;
   out_2627594616369667156[115] = 0;
   out_2627594616369667156[116] = 0;
   out_2627594616369667156[117] = 0;
   out_2627594616369667156[118] = 0;
   out_2627594616369667156[119] = 0;
   out_2627594616369667156[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_904766291043479982) {
   out_904766291043479982[0] = dt*state[3] + state[0];
   out_904766291043479982[1] = dt*state[4] + state[1];
   out_904766291043479982[2] = dt*state[5] + state[2];
   out_904766291043479982[3] = state[3];
   out_904766291043479982[4] = state[4];
   out_904766291043479982[5] = state[5];
   out_904766291043479982[6] = dt*state[7] + state[6];
   out_904766291043479982[7] = dt*state[8] + state[7];
   out_904766291043479982[8] = state[8];
   out_904766291043479982[9] = state[9];
   out_904766291043479982[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5659544920538399114) {
   out_5659544920538399114[0] = 1;
   out_5659544920538399114[1] = 0;
   out_5659544920538399114[2] = 0;
   out_5659544920538399114[3] = dt;
   out_5659544920538399114[4] = 0;
   out_5659544920538399114[5] = 0;
   out_5659544920538399114[6] = 0;
   out_5659544920538399114[7] = 0;
   out_5659544920538399114[8] = 0;
   out_5659544920538399114[9] = 0;
   out_5659544920538399114[10] = 0;
   out_5659544920538399114[11] = 0;
   out_5659544920538399114[12] = 1;
   out_5659544920538399114[13] = 0;
   out_5659544920538399114[14] = 0;
   out_5659544920538399114[15] = dt;
   out_5659544920538399114[16] = 0;
   out_5659544920538399114[17] = 0;
   out_5659544920538399114[18] = 0;
   out_5659544920538399114[19] = 0;
   out_5659544920538399114[20] = 0;
   out_5659544920538399114[21] = 0;
   out_5659544920538399114[22] = 0;
   out_5659544920538399114[23] = 0;
   out_5659544920538399114[24] = 1;
   out_5659544920538399114[25] = 0;
   out_5659544920538399114[26] = 0;
   out_5659544920538399114[27] = dt;
   out_5659544920538399114[28] = 0;
   out_5659544920538399114[29] = 0;
   out_5659544920538399114[30] = 0;
   out_5659544920538399114[31] = 0;
   out_5659544920538399114[32] = 0;
   out_5659544920538399114[33] = 0;
   out_5659544920538399114[34] = 0;
   out_5659544920538399114[35] = 0;
   out_5659544920538399114[36] = 1;
   out_5659544920538399114[37] = 0;
   out_5659544920538399114[38] = 0;
   out_5659544920538399114[39] = 0;
   out_5659544920538399114[40] = 0;
   out_5659544920538399114[41] = 0;
   out_5659544920538399114[42] = 0;
   out_5659544920538399114[43] = 0;
   out_5659544920538399114[44] = 0;
   out_5659544920538399114[45] = 0;
   out_5659544920538399114[46] = 0;
   out_5659544920538399114[47] = 0;
   out_5659544920538399114[48] = 1;
   out_5659544920538399114[49] = 0;
   out_5659544920538399114[50] = 0;
   out_5659544920538399114[51] = 0;
   out_5659544920538399114[52] = 0;
   out_5659544920538399114[53] = 0;
   out_5659544920538399114[54] = 0;
   out_5659544920538399114[55] = 0;
   out_5659544920538399114[56] = 0;
   out_5659544920538399114[57] = 0;
   out_5659544920538399114[58] = 0;
   out_5659544920538399114[59] = 0;
   out_5659544920538399114[60] = 1;
   out_5659544920538399114[61] = 0;
   out_5659544920538399114[62] = 0;
   out_5659544920538399114[63] = 0;
   out_5659544920538399114[64] = 0;
   out_5659544920538399114[65] = 0;
   out_5659544920538399114[66] = 0;
   out_5659544920538399114[67] = 0;
   out_5659544920538399114[68] = 0;
   out_5659544920538399114[69] = 0;
   out_5659544920538399114[70] = 0;
   out_5659544920538399114[71] = 0;
   out_5659544920538399114[72] = 1;
   out_5659544920538399114[73] = dt;
   out_5659544920538399114[74] = 0;
   out_5659544920538399114[75] = 0;
   out_5659544920538399114[76] = 0;
   out_5659544920538399114[77] = 0;
   out_5659544920538399114[78] = 0;
   out_5659544920538399114[79] = 0;
   out_5659544920538399114[80] = 0;
   out_5659544920538399114[81] = 0;
   out_5659544920538399114[82] = 0;
   out_5659544920538399114[83] = 0;
   out_5659544920538399114[84] = 1;
   out_5659544920538399114[85] = dt;
   out_5659544920538399114[86] = 0;
   out_5659544920538399114[87] = 0;
   out_5659544920538399114[88] = 0;
   out_5659544920538399114[89] = 0;
   out_5659544920538399114[90] = 0;
   out_5659544920538399114[91] = 0;
   out_5659544920538399114[92] = 0;
   out_5659544920538399114[93] = 0;
   out_5659544920538399114[94] = 0;
   out_5659544920538399114[95] = 0;
   out_5659544920538399114[96] = 1;
   out_5659544920538399114[97] = 0;
   out_5659544920538399114[98] = 0;
   out_5659544920538399114[99] = 0;
   out_5659544920538399114[100] = 0;
   out_5659544920538399114[101] = 0;
   out_5659544920538399114[102] = 0;
   out_5659544920538399114[103] = 0;
   out_5659544920538399114[104] = 0;
   out_5659544920538399114[105] = 0;
   out_5659544920538399114[106] = 0;
   out_5659544920538399114[107] = 0;
   out_5659544920538399114[108] = 1;
   out_5659544920538399114[109] = 0;
   out_5659544920538399114[110] = 0;
   out_5659544920538399114[111] = 0;
   out_5659544920538399114[112] = 0;
   out_5659544920538399114[113] = 0;
   out_5659544920538399114[114] = 0;
   out_5659544920538399114[115] = 0;
   out_5659544920538399114[116] = 0;
   out_5659544920538399114[117] = 0;
   out_5659544920538399114[118] = 0;
   out_5659544920538399114[119] = 0;
   out_5659544920538399114[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6682955905749598678) {
   out_6682955905749598678[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4085749273610108535) {
   out_4085749273610108535[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4085749273610108535[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4085749273610108535[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4085749273610108535[3] = 0;
   out_4085749273610108535[4] = 0;
   out_4085749273610108535[5] = 0;
   out_4085749273610108535[6] = 1;
   out_4085749273610108535[7] = 0;
   out_4085749273610108535[8] = 0;
   out_4085749273610108535[9] = 0;
   out_4085749273610108535[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8912925592750663323) {
   out_8912925592750663323[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_466141655324552523) {
   out_466141655324552523[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_466141655324552523[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_466141655324552523[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_466141655324552523[3] = 0;
   out_466141655324552523[4] = 0;
   out_466141655324552523[5] = 0;
   out_466141655324552523[6] = 1;
   out_466141655324552523[7] = 0;
   out_466141655324552523[8] = 0;
   out_466141655324552523[9] = 1;
   out_466141655324552523[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6302367303281942640) {
   out_6302367303281942640[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7349408904144907925) {
   out_7349408904144907925[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[6] = 0;
   out_7349408904144907925[7] = 1;
   out_7349408904144907925[8] = 0;
   out_7349408904144907925[9] = 0;
   out_7349408904144907925[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6302367303281942640) {
   out_6302367303281942640[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7349408904144907925) {
   out_7349408904144907925[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7349408904144907925[6] = 0;
   out_7349408904144907925[7] = 1;
   out_7349408904144907925[8] = 0;
   out_7349408904144907925[9] = 0;
   out_7349408904144907925[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9052257119879677051) {
  err_fun(nom_x, delta_x, out_9052257119879677051);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9030534445731092372) {
  inv_err_fun(nom_x, true_x, out_9030534445731092372);
}
void gnss_H_mod_fun(double *state, double *out_2627594616369667156) {
  H_mod_fun(state, out_2627594616369667156);
}
void gnss_f_fun(double *state, double dt, double *out_904766291043479982) {
  f_fun(state,  dt, out_904766291043479982);
}
void gnss_F_fun(double *state, double dt, double *out_5659544920538399114) {
  F_fun(state,  dt, out_5659544920538399114);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6682955905749598678) {
  h_6(state, sat_pos, out_6682955905749598678);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4085749273610108535) {
  H_6(state, sat_pos, out_4085749273610108535);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8912925592750663323) {
  h_20(state, sat_pos, out_8912925592750663323);
}
void gnss_H_20(double *state, double *sat_pos, double *out_466141655324552523) {
  H_20(state, sat_pos, out_466141655324552523);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6302367303281942640) {
  h_7(state, sat_pos_vel, out_6302367303281942640);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7349408904144907925) {
  H_7(state, sat_pos_vel, out_7349408904144907925);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6302367303281942640) {
  h_21(state, sat_pos_vel, out_6302367303281942640);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7349408904144907925) {
  H_21(state, sat_pos_vel, out_7349408904144907925);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
