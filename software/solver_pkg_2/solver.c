/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solver.c
 *
 * Code generation for function 'solver'
 *
 */

/* Include files */
#include "solver.h"
#include "rt_nonfinite.h"
#include "solver_emxutil.h"
#include "solver_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_Transducer3D
#define typedef_Transducer3D

typedef struct {
  double x;
  double y;
  double z;
} Transducer3D;

#endif                                 /* typedef_Transducer3D */

/* Variable Definitions */
static unsigned int state[625];
static boolean_T isInitialized_solver = false;

/* Function Declarations */
static void TryThisOneNoNorm(const double P[3], const double T[3], creal_T *M,
  creal_T *Mx, creal_T *My, creal_T *Mz, creal_T *Mxx, creal_T *Mxy, creal_T
  *Mxz, creal_T *Myy, creal_T *Myz, creal_T *Mzz, creal_T *Mxxx, creal_T *Mxyy,
  creal_T *Mxzz, creal_T *Myxx, creal_T *Myyy, creal_T *Myzz, creal_T *Mzxx,
  creal_T *Mzyy, creal_T *Mzzz);
static void b_rand(double varargin_2, emxArray_real_T *r);
static void b_sqrt(creal_T *x);
static void c_eml_rand_mt19937ar_stateful_i(void);
static int casyi(const creal_T z, double fnu, creal_T *y);
static void cbesj(const creal_T z, double fnu, creal_T *cy, int *nz, int *ierr);
static int cmlri(const creal_T z, double fnu, creal_T *y);
static void cospiAndSinpi(double x, double *c, double *s);
static double fminsearch(double funfcn_workspace_P_x, double
  funfcn_workspace_P_y, double funfcn_workspace_P_z, const Transducer3D
  funfcn_workspace_Tarray_active[1442], double funfcn_workspace_numT,
  emxArray_real_T *x);
static void gammaln(double *x);
static creal_T power(const creal_T a);
static double rt_atan2d_snf(double u0, double u1);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static double solver_anonFcn1(double P_x, double P_y, double P_z, const
  Transducer3D Tarray_active[1442], double numT, const emxArray_real_T *phi);
static void sortIdx(const emxArray_real_T *x, emxArray_int32_T *idx);
static creal_T sum(const emxArray_creal_T *x);
static void times(emxArray_creal_T *pzzz, const emxArray_creal_T *activation);

/* Function Definitions */
static void TryThisOneNoNorm(const double P[3], const double T[3], creal_T *M,
  creal_T *Mx, creal_T *My, creal_T *Mz, creal_T *Mxx, creal_T *Mxy, creal_T
  *Mxz, creal_T *Myy, creal_T *Myz, creal_T *Mzz, creal_T *Mxxx, creal_T *Mxyy,
  creal_T *Mxzz, creal_T *Myxx, creal_T *Myyy, creal_T *Myzz, creal_T *Mzxx,
  creal_T *Mzyy, creal_T *Mzzz)
{
  creal_T t128;
  creal_T t129;
  creal_T t132;
  creal_T t183;
  creal_T t184;
  creal_T t188;
  creal_T t190;
  creal_T t97;
  double ab_t42_re_tmp;
  double ai;
  double b_im;
  double b_im_tmp;
  double b_re;
  double b_re_tmp;
  double b_re_tmp_tmp;
  double b_re_tmp_tmp_tmp;
  double b_t104_im_tmp;
  double b_t104_re_tmp;
  double b_t42_im;
  double b_t42_im_tmp;
  double b_t42_re;
  double b_t42_re_tmp;
  double b_t42_re_tmp_tmp;
  double b_t42_re_tmp_tmp_tmp;
  double b_t42_re_tmp_tmp_tmp_tmp;
  double b_t43_im;
  double b_t43_im_tmp;
  double b_t43_re;
  double b_t43_re_tmp;
  double b_t43_re_tmp_tmp;
  double b_t43_re_tmp_tmp_tmp;
  double b_t44_im;
  double b_t44_im_tmp;
  double b_t44_re;
  double b_t44_re_tmp;
  double b_t44_re_tmp_tmp;
  double b_t44_re_tmp_tmp_tmp;
  double b_t48_im;
  double b_t48_im_tmp;
  double b_t48_re;
  double b_t48_re_tmp;
  double b_t48_re_tmp_tmp;
  double b_t50_im_tmp;
  double b_t50_re_tmp;
  double b_t52_im_tmp;
  double b_t52_re_tmp;
  double b_t54_im;
  double b_t54_im_tmp;
  double b_t54_re;
  double b_t54_re_tmp;
  double b_t54_re_tmp_tmp;
  double b_t55_im;
  double b_t55_re;
  double b_t55_re_tmp_tmp;
  double b_t56_im_tmp;
  double b_t56_re;
  double b_t56_re_tmp;
  double b_t58_im_tmp;
  double b_t58_re_tmp;
  double b_t68_im_tmp;
  double b_t68_re_tmp;
  double b_t68_re_tmp_tmp;
  double b_t73_im_tmp;
  double b_t73_re_tmp;
  double b_t93_im_tmp;
  double b_t93_re;
  double b_t93_re_tmp;
  double bb_t42_re_tmp;
  double c_im;
  double c_im_tmp;
  double c_re;
  double c_re_tmp;
  double c_re_tmp_tmp;
  double c_re_tmp_tmp_tmp;
  double c_t104_im_tmp;
  double c_t104_re_tmp;
  double c_t42_im;
  double c_t42_im_tmp;
  double c_t42_re;
  double c_t42_re_tmp;
  double c_t42_re_tmp_tmp;
  double c_t42_re_tmp_tmp_tmp;
  double c_t43_im;
  double c_t43_im_tmp;
  double c_t43_re;
  double c_t43_re_tmp;
  double c_t43_re_tmp_tmp;
  double c_t44_im_tmp;
  double c_t44_re_tmp;
  double c_t44_re_tmp_tmp;
  double c_t48_im;
  double c_t48_im_tmp;
  double c_t48_re;
  double c_t48_re_tmp;
  double c_t48_re_tmp_tmp;
  double c_t50_re_tmp;
  double c_t52_re_tmp;
  double c_t54_im;
  double c_t54_im_tmp;
  double c_t54_re;
  double c_t54_re_tmp;
  double c_t55_re;
  double c_t56_re_tmp;
  double c_t58_re_tmp;
  double c_t68_im_tmp;
  double c_t68_re_tmp;
  double c_t73_im_tmp;
  double c_t73_re_tmp;
  double c_t93_im_tmp;
  double c_t93_re_tmp;
  double cb_t42_re_tmp;
  double d;
  double d1;
  double d_im;
  double d_im_tmp;
  double d_re;
  double d_re_tmp;
  double d_re_tmp_tmp;
  double d_re_tmp_tmp_tmp;
  double d_t104_im_tmp;
  double d_t104_re_tmp;
  double d_t42_im;
  double d_t42_im_tmp;
  double d_t42_re;
  double d_t42_re_tmp;
  double d_t42_re_tmp_tmp;
  double d_t43_im_tmp;
  double d_t43_re;
  double d_t43_re_tmp;
  double d_t43_re_tmp_tmp;
  double d_t44_im_tmp;
  double d_t44_re_tmp;
  double d_t48_im;
  double d_t48_re;
  double d_t48_re_tmp;
  double d_t48_re_tmp_tmp;
  double d_t50_re_tmp;
  double d_t54_im;
  double d_t54_im_tmp;
  double d_t54_re;
  double d_t54_re_tmp;
  double d_t55_re;
  double d_t56_re_tmp;
  double d_t68_im_tmp;
  double d_t68_re_tmp;
  double d_t73_im_tmp;
  double d_t73_re_tmp;
  double d_t93_im_tmp;
  double d_t93_re_tmp;
  double db_t42_re_tmp;
  double e_im;
  double e_im_tmp;
  double e_re;
  double e_re_tmp;
  double e_re_tmp_tmp;
  double e_re_tmp_tmp_tmp;
  double e_t104_im_tmp;
  double e_t104_re_tmp;
  double e_t42_im;
  double e_t42_im_tmp;
  double e_t42_re;
  double e_t42_re_tmp;
  double e_t42_re_tmp_tmp;
  double e_t43_im_tmp;
  double e_t43_re;
  double e_t43_re_tmp;
  double e_t43_re_tmp_tmp;
  double e_t44_im_tmp;
  double e_t44_re_tmp;
  double e_t48_im;
  double e_t48_re;
  double e_t48_re_tmp;
  double e_t54_im;
  double e_t54_re;
  double e_t54_re_tmp;
  double e_t55_re;
  double e_t68_im_tmp;
  double e_t68_re_tmp;
  double e_t73_im_tmp;
  double e_t73_re_tmp;
  double e_t93_im_tmp;
  double e_t93_re_tmp;
  double f_im;
  double f_im_tmp;
  double f_re;
  double f_re_tmp;
  double f_re_tmp_tmp;
  double f_re_tmp_tmp_tmp;
  double f_t104_im_tmp;
  double f_t104_re_tmp;
  double f_t42_im_tmp;
  double f_t42_re;
  double f_t42_re_tmp;
  double f_t42_re_tmp_tmp;
  double f_t43_im_tmp;
  double f_t43_re_tmp;
  double f_t44_im_tmp;
  double f_t44_re_tmp;
  double f_t48_re;
  double f_t48_re_tmp;
  double f_t54_re;
  double f_t54_re_tmp;
  double f_t68_im_tmp;
  double f_t68_re_tmp;
  double f_t73_re_tmp;
  double f_t93_im_tmp;
  double f_t93_re_tmp;
  double g_im;
  double g_im_tmp;
  double g_re;
  double g_re_tmp;
  double g_re_tmp_tmp;
  double g_t104_im_tmp;
  double g_t104_re_tmp;
  double g_t42_im_tmp;
  double g_t42_re;
  double g_t42_re_tmp;
  double g_t42_re_tmp_tmp;
  double g_t43_im_tmp;
  double g_t43_re_tmp;
  double g_t44_im_tmp;
  double g_t44_re_tmp;
  double g_t48_re;
  double g_t48_re_tmp;
  double g_t68_im_tmp;
  double g_t68_re_tmp;
  double g_t93_im_tmp;
  double g_t93_re_tmp;
  double h_im;
  double h_im_tmp;
  double h_re;
  double h_re_tmp;
  double h_re_tmp_tmp;
  double h_t42_im_tmp;
  double h_t42_re;
  double h_t42_re_tmp;
  double h_t43_im_tmp;
  double h_t43_re_tmp;
  double h_t44_im_tmp;
  double h_t44_re_tmp;
  double h_t48_re;
  double h_t68_im_tmp;
  double h_t68_re_tmp;
  double h_t93_im_tmp;
  double h_t93_re_tmp;
  double i_im;
  double i_im_tmp;
  double i_re;
  double i_re_tmp;
  double i_re_tmp_tmp;
  double i_t42_im_tmp;
  double i_t42_re;
  double i_t42_re_tmp;
  double i_t43_im_tmp;
  double i_t43_re_tmp;
  double i_t44_im_tmp;
  double i_t44_re_tmp;
  double i_t48_re;
  double i_t68_im_tmp;
  double i_t68_re_tmp;
  double i_t93_im_tmp;
  double i_t93_re_tmp;
  double im;
  double im_tmp;
  double j_im;
  double j_im_tmp;
  double j_re;
  double j_re_tmp;
  double j_re_tmp_tmp;
  double j_t42_im_tmp;
  double j_t42_re_tmp;
  double j_t43_im_tmp;
  double j_t43_re_tmp;
  double j_t44_re_tmp;
  double j_t68_re_tmp;
  double j_t93_re_tmp;
  double k_im_tmp;
  double k_re_tmp;
  double k_t42_im_tmp;
  double k_t42_re_tmp;
  double k_t43_im_tmp;
  double k_t43_re_tmp;
  double k_t68_re_tmp;
  double l_im_tmp;
  double l_re_tmp;
  double l_t42_im_tmp;
  double l_t42_re_tmp;
  double l_t43_im_tmp;
  double l_t43_re_tmp;
  double m_im_tmp;
  double m_re_tmp;
  double m_t42_im_tmp;
  double m_t42_re_tmp;
  double m_t43_im_tmp;
  double m_t43_re_tmp;
  double n_re_tmp;
  double n_t42_im_tmp;
  double n_t42_re_tmp;
  double n_t43_im_tmp;
  double n_t43_re_tmp;
  double o_re_tmp;
  double o_t42_im_tmp;
  double o_t42_re_tmp;
  double o_t43_im_tmp;
  double o_t43_re_tmp;
  double p_t42_im_tmp;
  double p_t42_re_tmp;
  double p_t43_re_tmp;
  double q_t42_im_tmp;
  double q_t42_re_tmp;
  double q_t43_re_tmp;
  double r_t42_im_tmp;
  double r_t42_re_tmp;
  double r_t43_re_tmp;
  double re;
  double re_tmp;
  double re_tmp_tmp;
  double re_tmp_tmp_tmp;
  double s_t42_im_tmp;
  double s_t42_re_tmp;
  double s_t43_re_tmp;
  double t100;
  double t101;
  double t103;
  double t104;
  double t104_im_tmp;
  double t104_re_tmp;
  double t105;
  double t106;
  double t107;
  double t108_tmp;
  double t109_re;
  double t111;
  double t112_re;
  double t113_im;
  double t113_re;
  double t114_im;
  double t114_re;
  double t115;
  double t116_im;
  double t116_re;
  double t117_im;
  double t117_re;
  double t118;
  double t119;
  double t120;
  double t121;
  double t123_im;
  double t123_re;
  double t130_im;
  double t130_re;
  double t187_im;
  double t187_re;
  double t189_im;
  double t189_re;
  double t209_im;
  double t209_re;
  double t210_im;
  double t210_re;
  double t221_im;
  double t221_re;
  double t222_im;
  double t222_re;
  double t223_im;
  double t223_re;
  double t224_im;
  double t224_re;
  double t229_im_tmp;
  double t229_re_tmp;
  double t241_im;
  double t241_re;
  double t242_im;
  double t242_re;
  double t243_im;
  double t243_re;
  double t244_im;
  double t244_re;
  double t249_im;
  double t249_re;
  double t250_im;
  double t250_re;
  double t254_im;
  double t254_re;
  double t255_im;
  double t255_re;
  double t256_im;
  double t256_re;
  double t41_tmp;
  double t42;
  double t42_im;
  double t42_im_tmp;
  double t42_re;
  double t42_re_tmp;
  double t42_re_tmp_tmp;
  double t42_re_tmp_tmp_tmp;
  double t42_re_tmp_tmp_tmp_tmp;
  double t43;
  double t43_im;
  double t43_im_tmp;
  double t43_re;
  double t43_re_tmp;
  double t43_re_tmp_tmp;
  double t43_re_tmp_tmp_tmp;
  double t43_re_tmp_tmp_tmp_tmp;
  double t44;
  double t44_im;
  double t44_im_tmp;
  double t44_re;
  double t44_re_tmp;
  double t44_re_tmp_tmp;
  double t44_re_tmp_tmp_tmp;
  double t48;
  double t48_im;
  double t48_im_tmp;
  double t48_re;
  double t48_re_tmp;
  double t48_re_tmp_tmp;
  double t48_re_tmp_tmp_tmp;
  double t49;
  double t49_im_tmp;
  double t49_re;
  double t49_re_tmp;
  double t49_re_tmp_tmp;
  double t50;
  double t50_im;
  double t50_im_tmp;
  double t50_re;
  double t50_re_tmp;
  double t50_re_tmp_tmp;
  double t50_re_tmp_tmp_tmp;
  double t50_re_tmp_tmp_tmp_tmp;
  double t51;
  double t52;
  double t52_im_tmp;
  double t52_re_tmp;
  double t52_re_tmp_tmp;
  double t52_re_tmp_tmp_tmp;
  double t52_re_tmp_tmp_tmp_tmp;
  double t53;
  double t54_im;
  double t54_im_tmp;
  double t54_re;
  double t54_re_tmp;
  double t54_re_tmp_tmp;
  double t54_re_tmp_tmp_tmp;
  double t54_tmp;
  double t55;
  double t55_im;
  double t55_re;
  double t55_re_tmp;
  double t55_re_tmp_tmp;
  double t56_im_tmp;
  double t56_re;
  double t56_re_tmp;
  double t56_re_tmp_tmp;
  double t56_re_tmp_tmp_tmp;
  double t56_tmp;
  double t57;
  double t58_im;
  double t58_im_tmp;
  double t58_re_tmp;
  double t58_re_tmp_tmp;
  double t58_re_tmp_tmp_tmp;
  double t58_tmp;
  double t59;
  double t61;
  double t63;
  double t65;
  double t66;
  double t67;
  double t68;
  double t68_im;
  double t68_im_tmp;
  double t68_im_tmp_tmp;
  double t68_re;
  double t68_re_tmp;
  double t68_re_tmp_tmp;
  double t69;
  double t70;
  double t71;
  double t72;
  double t73;
  double t73_im;
  double t73_im_tmp;
  double t73_re;
  double t73_re_tmp;
  double t73_re_tmp_tmp;
  double t73_re_tmp_tmp_tmp;
  double t74;
  double t78;
  double t79;
  double t80;
  double t81;
  double t82;
  double t83;
  double t85;
  double t86;
  double t88;
  double t89;
  double t91;
  double t92;
  double t93;
  double t93_im;
  double t93_im_tmp;
  double t93_im_tmp_tmp;
  double t93_re;
  double t93_re_tmp;
  double t93_re_tmp_tmp;
  double t94_tmp;
  double t95;
  double t96;
  double t98;
  double t99;
  double t_t42_im_tmp;
  double t_t42_re_tmp;
  double u_t42_im_tmp;
  double u_t42_re_tmp;
  double v_t42_im_tmp;
  double v_t42_re_tmp;
  double w_t42_re_tmp;
  double x_t42_re_tmp;
  double y_t42_re_tmp;
  int a__2;
  int ierr;

  /* TryThisOneNoNorm */
  /*     [M,Mx,My,Mz,Mxx,Mxy,Mxz,Myy,Myz,Mzz,Mxxx,Mxyy,Mxzz,Myxx,Myyy,Myzz,Mzxx,Mzyy,Mzzz] = TryThisOneNoNorm(P1,P2,P3,T1,T2,T3) */
  /*     This function was generated by the Symbolic Math Toolbox version 9.2. */
  /*     04-Mar-2024 13:01:52 */
  t98 = P[0] + -T[0];
  t103 = P[1] + -T[1];
  t41_tmp = P[2] + -T[2];
  t42 = fabs(t98);
  t43 = fabs(t103);
  t44 = fabs(t41_tmp);
  t54_tmp = 1.0 / t98;
  t56_tmp = 1.0 / t103;
  t58_tmp = 1.0 / t41_tmp;
  t48 = t42 * t42;
  t49 = rt_powd_snf(t42, 3.0);
  t50 = t43 * t43;
  t51 = rt_powd_snf(t43, 3.0);
  t52 = t44 * t44;
  t53 = rt_powd_snf(t44, 3.0);
  t55 = t54_tmp * t54_tmp;
  t57 = t56_tmp * t56_tmp;
  t59 = t58_tmp * t58_tmp;
  t66 = t98 + t98;
  t67 = t103 + t103;
  t68 = t41_tmp + t41_tmp;
  t78 = ((P[0] * 4.0 + P[0] * 4.0) + -(T[0] * 4.0)) + -(T[0] * 4.0);
  t79 = ((P[1] * 4.0 + P[1] * 4.0) + -(T[1] * 4.0)) + -(T[1] * 4.0);
  t80 = ((P[2] * 4.0 + P[2] * 4.0) + -(T[2] * 4.0)) + -(T[2] * 4.0);
  t61 = t54_tmp * t54_tmp;
  t63 = t56_tmp * t56_tmp;
  t65 = t58_tmp * t58_tmp;
  t69 = t66 * t66;
  t70 = rt_powd_snf(t66, 3.0);
  t71 = t67 * t67;
  t72 = rt_powd_snf(t67, 3.0);
  t73 = t68 * t68;
  t74 = rt_powd_snf(t68, 3.0);
  t81 = t48 + t50;
  t82 = 1.0 / sqrt(t98 * t98);
  t85 = 1.0 / sqrt(t103 * t103);
  t88 = 1.0 / sqrt(t41_tmp * t41_tmp);
  t83 = rt_powd_snf(t82, 3.0);
  t86 = rt_powd_snf(t85, 3.0);
  t89 = rt_powd_snf(t88, 3.0);
  t91 = 1.0 / t81;
  t93 = sqrt(t81);
  t98 = t52 + t81;
  t92 = t91 * t91;
  t94_tmp = 1.0 / t93;
  t97.re = t93 * 0.0;
  t99 = 1.0 / t98;
  t103 = sqrt(t98);
  t95 = rt_powd_snf(t94_tmp, 3.0);
  t96 = rt_powd_snf(t94_tmp, 5.0);
  t100 = t99 * t99;
  t101 = rt_powd_snf(t99, 3.0);
  t104 = 1.0 / t103;
  t108_tmp = t93 * t93;
  t109_re = (T[2] + -P[2]) + t97.re;
  t112_re = t41_tmp + t97.re;
  t105 = rt_powd_snf(t104, 3.0);
  t106 = rt_powd_snf(t104, 5.0);
  t107 = rt_powd_snf(t104, 7.0);
  t111 = 1.0 / t108_tmp;
  if (t93 == 0.0) {
    t113_re = 1.0 / t109_re;
    t113_im = 0.0;
  } else if (t109_re == 0.0) {
    t113_re = 0.0;
    t113_im = -(1.0 / t93);
  } else {
    t41_tmp = fabs(t109_re);
    if (t41_tmp > t93) {
      t81 = t93 / t109_re;
      t98 = t109_re + t81 * t93;
      t113_re = (t81 * 0.0 + 1.0) / t98;
      t113_im = (0.0 - t81) / t98;
    } else if (t93 == t41_tmp) {
      if (t109_re > 0.0) {
        t98 = 0.5;
      } else {
        t98 = -0.5;
      }

      t113_re = t98 / t41_tmp;
      t113_im = -0.5 / t41_tmp;
    } else {
      t81 = t109_re / t93;
      t98 = t93 + t81 * t109_re;
      t113_re = t81 / t98;
      t113_im = (t81 * 0.0 - 1.0) / t98;
    }
  }

  t115 = rt_hypotd_snf(t112_re, t93);
  if (t93 == 0.0) {
    t116_re = 1.0 / t112_re;
    t116_im = 0.0;
  } else if (t112_re == 0.0) {
    t116_re = 0.0;
    t116_im = -(1.0 / t93);
  } else {
    t41_tmp = fabs(t112_re);
    if (t41_tmp > t93) {
      t81 = t93 / t112_re;
      t98 = t112_re + t81 * t93;
      t116_re = (t81 * 0.0 + 1.0) / t98;
      t116_im = (0.0 - t81) / t98;
    } else if (t93 == t41_tmp) {
      if (t112_re > 0.0) {
        t98 = 0.5;
      } else {
        t98 = -0.5;
      }

      t116_re = t98 / t41_tmp;
      t116_im = -0.5 / t41_tmp;
    } else {
      t81 = t112_re / t93;
      t98 = t93 + t81 * t112_re;
      t116_re = t81 / t98;
      t116_im = (t81 * 0.0 - 1.0) / t98;
    }
  }

  t97.re = t103 * 3.1415926535897931 * 0.0;
  t97.im = t103 * 3.1415926535897931 * 233.23615160349851;
  if (t97.im == 0.0) {
    t123_re = exp(t97.re);
    t123_im = 0.0;
  } else {
    t81 = exp(t97.re / 2.0);
    t123_re = t81 * (t81 * cos(t97.im));
    t123_im = t81 * (t81 * sin(t97.im));
  }

  t114_re = t113_re * t113_re - t113_im * t113_im;
  t103 = t113_re * t113_im;
  t114_im = t103 + t103;
  t117_re = t116_re * t116_re - t116_im * t116_im;
  t81 = t116_re * t116_im;
  t117_im = t81 + t81;
  t118 = 1.0 / t115;
  t42_re_tmp_tmp_tmp = t42 * t66;
  t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t82;
  t42_re_tmp = t42_re_tmp_tmp * t94_tmp;
  t42_re = t42_re_tmp * t109_re;
  t42_im_tmp = t42_re_tmp * t93;
  b_t42_re = t42_re_tmp * t112_re;
  t183.re = (t42_re * 0.0 - t42_im_tmp * 0.5) + (b_t42_re * 0.0 - t42_im_tmp *
    0.5);
  t183.im = (t42_re * 0.5 + t42_im_tmp * 0.0) + (b_t42_re * 0.5 + t42_im_tmp *
    0.0);
  t43_re_tmp_tmp_tmp = t43 * t67;
  t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t85;
  t43_re_tmp = t43_re_tmp_tmp * t94_tmp;
  t43_re = t43_re_tmp * t109_re;
  t43_im_tmp = t43_re_tmp * t93;
  b_t43_re = t43_re_tmp * t112_re;
  t184.re = (t43_re * 0.0 - t43_im_tmp * 0.5) + (b_t43_re * 0.0 - t43_im_tmp *
    0.5);
  t184.im = (t43_re * 0.5 + t43_im_tmp * 0.0) + (b_t43_re * 0.5 + t43_im_tmp *
    0.0);
  t42_re_tmp_tmp_tmp_tmp = t42 * t43;
  b_t42_re_tmp_tmp_tmp = t42_re_tmp_tmp_tmp_tmp * t66;
  b_t42_re_tmp_tmp = b_t42_re_tmp_tmp_tmp * t67 * t82 * t85;
  b_t42_re_tmp = b_t42_re_tmp_tmp * t95;
  t42_re = b_t42_re_tmp * t109_re;
  b_t42_im_tmp = b_t42_re_tmp * t93;
  b_t42_re = b_t42_re_tmp * t112_re;
  t221_re = (b_t42_re_tmp_tmp * t91 / 2.0 + (t42_re * 0.0 - b_t42_im_tmp * 0.25))
    + (b_t42_re * 0.0 - b_t42_im_tmp * 0.25);
  t221_im = (t42_re * 0.25 + b_t42_im_tmp * 0.0) + (b_t42_re * 0.25 +
    b_t42_im_tmp * 0.0);
  t119 = t118 * t118;
  t120 = rt_powd_snf(t118, 3.0);
  t97.re = -(t109_re * t112_re - t108_tmp);
  t97.im = -(t109_re * t93 + t93 * t112_re);
  b_sqrt(&t97);
  if (t97.im == 0.0) {
    t128.re = 1.0 / t97.re;
    t128.im = 0.0;
  } else if (t97.re == 0.0) {
    t128.re = 0.0;
    t128.im = -(1.0 / t97.im);
  } else {
    t41_tmp = fabs(t97.re);
    t81 = fabs(t97.im);
    if (t41_tmp > t81) {
      t81 = t97.im / t97.re;
      t98 = t97.re + t81 * t97.im;
      t128.re = (t81 * 0.0 + 1.0) / t98;
      t128.im = (0.0 - t81) / t98;
    } else if (t81 == t41_tmp) {
      if (t97.re > 0.0) {
        t81 = 0.5;
      } else {
        t81 = -0.5;
      }

      if (t97.im > 0.0) {
        t98 = 0.5;
      } else {
        t98 = -0.5;
      }

      t128.re = (t81 + 0.0 * t98) / t41_tmp;
      t128.im = (0.0 * t81 - t98) / t41_tmp;
    } else {
      t81 = t97.re / t97.im;
      t98 = t97.im + t81 * t97.re;
      t128.re = t81 / t98;
      t128.im = (t81 * 0.0 - 1.0) / t98;
    }
  }

  t98 = t93 * t118 * 3.1415926535897931 * 1.1428571428571428;
  t187_re = t183.re * t183.re - t183.im * t183.im;
  t81 = t183.re * t183.im;
  t187_im = t81 + t81;
  t188 = power(t183);
  t189_re = t184.re * t184.re - t184.im * t184.im;
  t81 = t184.re * t184.im;
  t189_im = t81 + t81;
  t190 = power(t184);
  c_t42_re_tmp_tmp = t42 * t82;
  c_t42_re_tmp = c_t42_re_tmp_tmp * t94_tmp;
  t42_re = c_t42_re_tmp * t109_re;
  b_t42_im_tmp = c_t42_re_tmp * t93;
  b_t42_re = c_t42_re_tmp * t112_re;
  t54_re_tmp_tmp_tmp = t54_tmp * t54_tmp;
  t54_re_tmp_tmp = t54_re_tmp_tmp_tmp * t69;
  t54_re_tmp = t54_re_tmp_tmp * t94_tmp;
  t54_re = t54_re_tmp * t109_re;
  t54_im_tmp = t54_re_tmp * t93;
  b_t54_re = t54_re_tmp * t112_re;
  d_t42_re_tmp_tmp = t42 * t69 * t83;
  d_t42_re_tmp = d_t42_re_tmp_tmp * t94_tmp;
  c_t42_re = d_t42_re_tmp * t109_re;
  c_t42_im_tmp = d_t42_re_tmp * t93;
  t41_tmp = t48 * t54_tmp;
  t48_re_tmp_tmp_tmp = t41_tmp * t54_tmp;
  t48_re_tmp_tmp = t48_re_tmp_tmp_tmp * t69;
  t48_re_tmp = t48_re_tmp_tmp * t95;
  t48_re = t48_re_tmp * t109_re;
  t48_im_tmp = t48_re_tmp * t93;
  d_t42_re = d_t42_re_tmp * t112_re;
  b_t48_re = t48_re_tmp * t112_re;
  t241_re = (((((((t48_re_tmp_tmp * t91 / 2.0 + -(t42_re * 0.0 - b_t42_im_tmp))
                  + -(b_t42_re * 0.0 - b_t42_im_tmp)) + -(t54_re * 0.0 -
    t54_im_tmp * 0.25)) + -(b_t54_re * 0.0 - t54_im_tmp * 0.25)) + (c_t42_re *
    0.0 - c_t42_im_tmp * 0.25)) + (t48_re * 0.0 - t48_im_tmp * 0.25)) +
             (d_t42_re * 0.0 - c_t42_im_tmp * 0.25)) + (b_t48_re * 0.0 -
    t48_im_tmp * 0.25);
  t241_im = ((((((-(t42_re + b_t42_im_tmp * 0.0) + -(b_t42_re + b_t42_im_tmp *
    0.0)) + -(t54_re * 0.25 + t54_im_tmp * 0.0)) + -(b_t54_re * 0.25 +
    t54_im_tmp * 0.0)) + (c_t42_re * 0.25 + c_t42_im_tmp * 0.0)) + (t48_re *
    0.25 + t48_im_tmp * 0.0)) + (d_t42_re * 0.25 + c_t42_im_tmp * 0.0)) +
    (b_t48_re * 0.25 + t48_im_tmp * 0.0);
  b_t43_re_tmp_tmp = t43 * t85;
  b_t43_re_tmp = b_t43_re_tmp_tmp * t94_tmp;
  t43_re = b_t43_re_tmp * t109_re;
  b_t43_im_tmp = b_t43_re_tmp * t93;
  b_t43_re = b_t43_re_tmp * t112_re;
  t56_re_tmp_tmp_tmp = t56_tmp * t56_tmp;
  t56_re_tmp_tmp = t56_re_tmp_tmp_tmp * t71;
  t56_re_tmp = t56_re_tmp_tmp * t94_tmp;
  t56_re = t56_re_tmp * t109_re;
  t56_im_tmp = t56_re_tmp * t93;
  b_t56_re = t56_re_tmp * t112_re;
  c_t43_re_tmp_tmp = t43 * t71 * t86;
  c_t43_re_tmp = c_t43_re_tmp_tmp * t94_tmp;
  c_t43_re = c_t43_re_tmp * t109_re;
  c_t43_im_tmp = c_t43_re_tmp * t93;
  t50_re_tmp_tmp_tmp_tmp = t50 * t56_tmp;
  t50_re_tmp_tmp_tmp = t50_re_tmp_tmp_tmp_tmp * t56_tmp;
  t50_re_tmp_tmp = t50_re_tmp_tmp_tmp * t71;
  t50_re_tmp = t50_re_tmp_tmp * t95;
  t48_re = t50_re_tmp * t109_re;
  t50_im_tmp = t50_re_tmp * t93;
  d_t43_re = c_t43_re_tmp * t112_re;
  t50_re = t50_re_tmp * t112_re;
  t242_re = (((((((t50_re_tmp_tmp * t91 / 2.0 + -(t43_re * 0.0 - b_t43_im_tmp))
                  + -(b_t43_re * 0.0 - b_t43_im_tmp)) + -(t56_re * 0.0 -
    t56_im_tmp * 0.25)) + -(b_t56_re * 0.0 - t56_im_tmp * 0.25)) + (c_t43_re *
    0.0 - c_t43_im_tmp * 0.25)) + (t48_re * 0.0 - t50_im_tmp * 0.25)) +
             (d_t43_re * 0.0 - c_t43_im_tmp * 0.25)) + (t50_re * 0.0 -
    t50_im_tmp * 0.25);
  t242_im = ((((((-(t43_re + b_t43_im_tmp * 0.0) + -(b_t43_re + b_t43_im_tmp *
    0.0)) + -(t56_re * 0.25 + t56_im_tmp * 0.0)) + -(b_t56_re * 0.25 +
    t56_im_tmp * 0.0)) + (c_t43_re * 0.25 + c_t43_im_tmp * 0.0)) + (t48_re *
    0.25 + t50_im_tmp * 0.0)) + (d_t43_re * 0.25 + c_t43_im_tmp * 0.0)) +
    (t50_re * 0.25 + t50_im_tmp * 0.0);
  t121 = t119 * t119;
  t129 = power(t128);
  if ((t128.im == 0.0) && (t128.re >= 0.0)) {
    t130_re = rt_powd_snf(t128.re, 5.0);
    t130_im = 0.0;
  } else if (t128.re == 0.0) {
    t130_re = 0.0;
    t130_im = rt_powd_snf(t128.im, 5.0);
  } else {
    if (t128.im == 0.0) {
      if (t128.re < 0.0) {
        t97.re = log(fabs(t128.re));
        t97.im = 3.1415926535897931;
      } else {
        t97.re = log(t128.re);
        t97.im = 0.0;
      }
    } else if ((fabs(t128.re) > 8.9884656743115785E+307) || (fabs(t128.im) >
                8.9884656743115785E+307)) {
      t97.re = log(rt_hypotd_snf(t128.re / 2.0, t128.im / 2.0)) +
        0.69314718055994529;
      t97.im = rt_atan2d_snf(t128.im, t128.re);
    } else {
      t97.re = log(rt_hypotd_snf(t128.re, t128.im));
      t97.im = rt_atan2d_snf(t128.im, t128.re);
    }

    t130_re = 5.0 * t97.re;
    t130_im = 5.0 * t97.im;
    if (t130_im == 0.0) {
      t130_re = exp(t130_re);
      t130_im = 0.0;
    } else if (rtIsInf(t130_im) && rtIsInf(t130_re) && (t130_re < 0.0)) {
      t130_re = 0.0;
      t130_im = 0.0;
    } else {
      t81 = exp(t130_re / 2.0);
      t130_re = t81 * (t81 * cos(t130_im));
      t130_im = t81 * (t81 * sin(t130_im));
    }
  }

  t97.re = t98;
  t97.im = 0.0;
  ierr = 0;
  if (rtIsNaN(t98)) {
    t132.re = rtNaN;
  } else {
    cbesj(t97, -0.0, &t132, &a__2, &ierr);
  }

  if (ierr == 5) {
    t132.re = rtNaN;
  } else if (ierr == 2) {
    t132.re = rtInf;
  }

  d = t132.re;
  t104_re_tmp = t104 * t123_re;
  t104_im_tmp = t104 * t123_im;
  M->re = 1.0669676460233539 * (t104_re_tmp * d - t104_im_tmp * 0.0);
  M->im = 1.0669676460233539 * (t104_re_tmp * 0.0 + t104_im_tmp * d);
  t97.re = t98;
  t97.im = 0.0;
  ierr = 0;
  if (rtIsNaN(t98)) {
    t132.re = rtNaN;
  } else {
    cbesj(t97, 1.0, &t132, &a__2, &ierr);
  }

  if (ierr == 5) {
    t132.re = rtNaN;
  } else if (ierr == 2) {
    t132.re = rtInf;
  }

  d1 = t132.re;
  t68_re_tmp = t68 * t93;
  b_t68_re_tmp = t68_re_tmp * t113_re;
  t68_im_tmp = t68_re_tmp * t113_im;
  t68_re_tmp_tmp = b_t68_re_tmp * t116_re - t68_im_tmp * t116_im;
  c_t68_re_tmp = t120 * t68_re_tmp_tmp;
  t68_im_tmp_tmp = b_t68_re_tmp * t116_im + t68_im_tmp * t116_re;
  b_t68_im_tmp = t120 * t68_im_tmp_tmp;
  t209_re = 0.5714285714285714 * (3.1415926535897931 * (c_t68_re_tmp * t183.re -
    b_t68_im_tmp * t183.im));
  t209_im = 0.5714285714285714 * (3.1415926535897931 * (c_t68_re_tmp * t183.im +
    b_t68_im_tmp * t183.re));
  t210_re = 0.5714285714285714 * (3.1415926535897931 * (c_t68_re_tmp * t184.re -
    b_t68_im_tmp * t184.im));
  t210_im = 0.5714285714285714 * (3.1415926535897931 * (c_t68_re_tmp * t184.im +
    b_t68_im_tmp * t184.re));
  t81 = t118 * t128.re;
  t98 = t118 * t128.im;
  t97.re = t81 * d1 - t98 * 0.0;
  t97.im = t81 * 0.0 + t98 * d1;
  t93_re_tmp = t93 * t119;
  b_t93_re_tmp = t93_re_tmp * t128.re;
  t93_im_tmp = t93_re_tmp * t128.im;
  t223_re = t42_re_tmp * t118 * 3.1415926535897931 * 0.5714285714285714 +
    3.1415926535897931 * (b_t93_re_tmp * t183.re - t93_im_tmp * t183.im) *
    0.5714285714285714;
  t223_im = 3.1415926535897931 * (b_t93_re_tmp * t183.im + t93_im_tmp * t183.re)
    * 0.5714285714285714;
  b_t104_re_tmp = t104_re_tmp * d1 - t104_im_tmp * 0.0;
  b_t104_im_tmp = t104_re_tmp * 0.0 + t104_im_tmp * d1;
  e_t42_re_tmp = t42_re_tmp_tmp * t105;
  f_t42_re_tmp = e_t42_re_tmp * t123_re;
  b_t42_im_tmp = e_t42_re_tmp * t123_im;
  e_t42_re_tmp = t42_re_tmp_tmp * t99;
  g_t42_re_tmp = e_t42_re_tmp * t123_re;
  c_t42_im_tmp = e_t42_re_tmp * t123_im;
  t42_re = 3.1415926535897931 * (g_t42_re_tmp * d - c_t42_im_tmp * 0.0);
  t42_im = 3.1415926535897931 * (g_t42_re_tmp * 0.0 + c_t42_im_tmp * d);
  Mx->re = ((b_t104_re_tmp * t223_re - b_t104_im_tmp * t223_im) *
            -1.0669676460233539 - (f_t42_re_tmp * d - b_t42_im_tmp * 0.0) *
            0.53348382301167685) + (t42_re * 0.0 - t42_im * 124.4277138219654);
  Mx->im = ((b_t104_re_tmp * t223_im + b_t104_im_tmp * t223_re) *
            -1.0669676460233539 - (f_t42_re_tmp * 0.0 + b_t42_im_tmp * d) *
            0.53348382301167685) + (t42_re * 124.4277138219654 + t42_im * 0.0);
  t224_re = t43_re_tmp * t118 * 3.1415926535897931 * 0.5714285714285714 +
    3.1415926535897931 * (b_t93_re_tmp * t184.re - t93_im_tmp * t184.im) *
    0.5714285714285714;
  t224_im = 3.1415926535897931 * (b_t93_re_tmp * t184.im + t93_im_tmp * t184.re)
    * 0.5714285714285714;
  d_t43_re_tmp = t43_re_tmp_tmp * t105;
  e_t43_re_tmp = d_t43_re_tmp * t123_re;
  b_t43_im_tmp = d_t43_re_tmp * t123_im;
  d_t43_re_tmp = t43_re_tmp_tmp * t99;
  f_t43_re_tmp = d_t43_re_tmp * t123_re;
  c_t43_im_tmp = d_t43_re_tmp * t123_im;
  t43_re = 3.1415926535897931 * (f_t43_re_tmp * d - c_t43_im_tmp * 0.0);
  t43_im = 3.1415926535897931 * (f_t43_re_tmp * 0.0 + c_t43_im_tmp * d);
  My->re = ((b_t104_re_tmp * t224_re - b_t104_im_tmp * t224_im) *
            -1.0669676460233539 - (e_t43_re_tmp * d - b_t43_im_tmp * 0.0) *
            0.53348382301167685) + (t43_re * 0.0 - t43_im * 124.4277138219654);
  My->im = ((b_t104_re_tmp * t224_im + b_t104_im_tmp * t224_re) *
            -1.0669676460233539 - (e_t43_re_tmp * 0.0 + b_t43_im_tmp * d) *
            0.53348382301167685) + (t43_re * 124.4277138219654 + t43_im * 0.0);
  t44_re_tmp_tmp_tmp = t44 * t68;
  t44_re_tmp_tmp = t44_re_tmp_tmp_tmp * t88;
  t44_re_tmp = t44_re_tmp_tmp * t105;
  b_t44_re_tmp = t44_re_tmp * t123_re;
  t44_im_tmp = t44_re_tmp * t123_im;
  t44_re_tmp = t44_re_tmp_tmp * t99;
  c_t44_re_tmp = t44_re_tmp * t123_re;
  b_t44_im_tmp = t44_re_tmp * t123_im;
  t44_re = 3.1415926535897931 * (c_t44_re_tmp * d - b_t44_im_tmp * 0.0);
  t44_im = 3.1415926535897931 * (c_t44_re_tmp * 0.0 + b_t44_im_tmp * d);
  b_t68_re_tmp_tmp = t68_re_tmp * t104;
  d_t68_re_tmp = b_t68_re_tmp_tmp * t119;
  e_t68_re_tmp = d_t68_re_tmp * t123_re;
  c_t68_im_tmp = d_t68_re_tmp * t123_im;
  d_t68_re_tmp = e_t68_re_tmp * t128.re - c_t68_im_tmp * t128.im;
  d_t68_im_tmp = e_t68_re_tmp * t128.im + c_t68_im_tmp * t128.re;
  Mz->re = ((b_t44_re_tmp * d - t44_im_tmp * 0.0) * -0.53348382301167685 +
            (t44_re * 0.0 - t44_im * 124.4277138219654)) + 3.1415926535897931 *
    (d_t68_re_tmp * d1 - d_t68_im_tmp * 0.0) * 0.60969579772763072;
  Mz->im = ((b_t44_re_tmp * 0.0 + t44_im_tmp * d) * -0.53348382301167685 +
            (t44_re * 124.4277138219654 + t44_im * 0.0)) + 3.1415926535897931 *
    (d_t68_re_tmp * 0.0 + d_t68_im_tmp * d1) * 0.60969579772763072;
  re_tmp = 0.31830988618379069 * t43 * t67 * t85 * t94_tmp * t111 * t115;
  re = re_tmp * d1;
  im = re_tmp * 0.0;
  re_tmp = 0.31830988618379069 * t94_tmp * t128.re;
  im_tmp = 0.31830988618379069 * t94_tmp * t128.im;
  b_re_tmp = re_tmp * d1 - im_tmp * 0.0;
  im_tmp = re_tmp * 0.0 + im_tmp * d1;
  b_re = b_re_tmp * t184.re - im_tmp * t184.im;
  b_im = b_re_tmp * t184.im + im_tmp * t184.re;
  e_t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t68 * t82;
  f_t42_re_tmp_tmp = e_t42_re_tmp_tmp * t94_tmp;
  e_t42_re_tmp = f_t42_re_tmp_tmp * t119;
  h_t42_re_tmp = e_t42_re_tmp * t128.re;
  d_t42_im_tmp = e_t42_re_tmp * t128.im;
  f_t68_re_tmp = t68_re_tmp * t119;
  g_t68_re_tmp = f_t68_re_tmp * t129.re;
  e_t68_im_tmp = f_t68_re_tmp * t129.im;
  t243_re = (3.1415926535897931 * h_t42_re_tmp * 0.2857142857142857 + -t209_re)
    + 3.1415926535897931 * (g_t68_re_tmp * t183.re - e_t68_im_tmp * t183.im) *
    0.2857142857142857;
  t243_im = (3.1415926535897931 * d_t42_im_tmp * 0.2857142857142857 + -t209_im)
    + 3.1415926535897931 * (g_t68_re_tmp * t183.im + e_t68_im_tmp * t183.re) *
    0.2857142857142857;
  d_t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t68 * t85;
  e_t43_re_tmp_tmp = d_t43_re_tmp_tmp * t94_tmp;
  d_t43_re_tmp = e_t43_re_tmp_tmp * t119;
  g_t43_re_tmp = d_t43_re_tmp * t128.re;
  d_t43_im_tmp = d_t43_re_tmp * t128.im;
  t244_re = (3.1415926535897931 * g_t43_re_tmp * 0.2857142857142857 + -t210_re)
    + 3.1415926535897931 * (g_t68_re_tmp * t184.re - e_t68_im_tmp * t184.im) *
    0.2857142857142857;
  t244_im = (3.1415926535897931 * d_t43_im_tmp * 0.2857142857142857 + -t210_im)
    + 3.1415926535897931 * (g_t68_re_tmp * t184.im + e_t68_im_tmp * t184.re) *
    0.2857142857142857;
  c_t93_re_tmp = t93 * t113_re;
  b_t93_im_tmp = t93 * t113_im;
  t93_re_tmp_tmp = c_t93_re_tmp * t116_re - b_t93_im_tmp * t116_im;
  d_t93_re_tmp = t120 * t93_re_tmp_tmp;
  t93_im_tmp_tmp = c_t93_re_tmp * t116_im + b_t93_im_tmp * t116_re;
  b_t93_im_tmp = t120 * t93_im_tmp_tmp;
  c_t93_re_tmp = t93_re_tmp * t129.re;
  c_t93_im_tmp = t93_re_tmp * t129.im;
  i_t42_re_tmp = t42_re_tmp * t119;
  j_t42_re_tmp = i_t42_re_tmp * t128.re;
  e_t42_im_tmp = i_t42_re_tmp * t128.im;
  t254_re = ((((((c_t42_re_tmp * t118 * 3.1415926535897931 * 1.1428571428571428
                  + t54_re_tmp * t118 * 3.1415926535897931 * 0.2857142857142857)
                 + -(d_t42_re_tmp * t118 * 3.1415926535897931 *
                     0.2857142857142857)) + -(t48_re_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857)) + -(3.1415926535897931 *
    (d_t93_re_tmp * t187_re - b_t93_im_tmp * t187_im) * 0.5714285714285714)) +
              3.1415926535897931 * (c_t93_re_tmp * t187_re - c_t93_im_tmp *
    t187_im) * 0.2857142857142857) + 3.1415926535897931 * (j_t42_re_tmp *
              t183.re - e_t42_im_tmp * t183.im) * 0.5714285714285714) +
    -(3.1415926535897931 * (b_t93_re_tmp * t241_re - t93_im_tmp * t241_im) *
      0.5714285714285714);
  t254_im = ((-(3.1415926535897931 * (d_t93_re_tmp * t187_im + b_t93_im_tmp *
    t187_re) * 0.5714285714285714) + 3.1415926535897931 * (c_t93_re_tmp *
    t187_im + c_t93_im_tmp * t187_re) * 0.2857142857142857) + 3.1415926535897931
             * (j_t42_re_tmp * t183.im + e_t42_im_tmp * t183.re) *
             0.5714285714285714) + -(3.1415926535897931 * (b_t93_re_tmp *
    t241_im + t93_im_tmp * t241_re) * 0.5714285714285714);
  h_t43_re_tmp = t43_re_tmp * t119;
  i_t43_re_tmp = h_t43_re_tmp * t128.re;
  e_t43_im_tmp = h_t43_re_tmp * t128.im;
  t255_re = ((((((b_t43_re_tmp * t118 * 3.1415926535897931 * 1.1428571428571428
                  + t56_re_tmp * t118 * 3.1415926535897931 * 0.2857142857142857)
                 + -(c_t43_re_tmp * t118 * 3.1415926535897931 *
                     0.2857142857142857)) + -(t50_re_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857)) + -(3.1415926535897931 *
    (d_t93_re_tmp * t189_re - b_t93_im_tmp * t189_im) * 0.5714285714285714)) +
              3.1415926535897931 * (c_t93_re_tmp * t189_re - c_t93_im_tmp *
    t189_im) * 0.2857142857142857) + 3.1415926535897931 * (i_t43_re_tmp *
              t184.re - e_t43_im_tmp * t184.im) * 0.5714285714285714) +
    -(3.1415926535897931 * (b_t93_re_tmp * t242_re - t93_im_tmp * t242_im) *
      0.5714285714285714);
  t255_im = ((-(3.1415926535897931 * (d_t93_re_tmp * t189_im + b_t93_im_tmp *
    t189_re) * 0.5714285714285714) + 3.1415926535897931 * (c_t93_re_tmp *
    t189_im + c_t93_im_tmp * t189_re) * 0.2857142857142857) + 3.1415926535897931
             * (i_t43_re_tmp * t184.im + e_t43_im_tmp * t184.re) *
             0.5714285714285714) + -(3.1415926535897931 * (b_t93_re_tmp *
    t242_im + t93_im_tmp * t242_re) * 0.5714285714285714);
  b_t48_re = t68 * t97.re;
  ai = t68 * t97.im;
  if (ai == 0.0) {
    t68_re = b_t48_re / 2.0;
    t68_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t68_re = 0.0;
    t68_im = ai / 2.0;
  } else {
    t68_re = b_t48_re / 2.0;
    t68_im = ai / 2.0;
  }

  h_t68_re_tmp = f_t68_re_tmp * t128.re;
  f_t68_im_tmp = f_t68_re_tmp * t128.im;
  t222_re = t68_re + -(3.1415926535897931 * (h_t68_re_tmp * d - f_t68_im_tmp *
    0.0) * 0.5714285714285714);
  t222_im = t68_im + -(3.1415926535897931 * (h_t68_re_tmp * 0.0 + f_t68_im_tmp *
    d) * 0.5714285714285714);
  t50_re = t224_re * t224_re - t224_im * t224_im;
  t81 = t224_re * t224_im;
  b_t56_re = t81 + t81;
  t93_re = d_t93_re_tmp * t183.re - b_t93_im_tmp * t183.im;
  t93_im = d_t93_re_tmp * t183.im + b_t93_im_tmp * t183.re;
  e_t93_re_tmp = c_t93_re_tmp * t183.re - c_t93_im_tmp * t183.im;
  d_t93_im_tmp = c_t93_re_tmp * t183.im + c_t93_im_tmp * t183.re;
  t256_re = ((((b_t42_re_tmp * t118 * 3.1415926535897931 * 0.2857142857142857 +
                -(3.1415926535897931 * (i_t43_re_tmp * t183.re - e_t43_im_tmp *
    t183.im) * 0.2857142857142857)) + -(3.1415926535897931 * (j_t42_re_tmp *
    t184.re - e_t42_im_tmp * t184.im) * 0.2857142857142857)) +
              3.1415926535897931 * (b_t93_re_tmp * t221_re - t93_im_tmp *
    t221_im) * 0.5714285714285714) + 3.1415926535897931 * (t93_re * t184.re -
              t93_im * t184.im) * 0.5714285714285714) + -(3.1415926535897931 *
    (e_t93_re_tmp * t184.re - d_t93_im_tmp * t184.im) * 0.2857142857142857);
  t256_im = (((-(3.1415926535897931 * (i_t43_re_tmp * t183.im + e_t43_im_tmp *
    t183.re) * 0.2857142857142857) + -(3.1415926535897931 * (j_t42_re_tmp *
    t184.im + e_t42_im_tmp * t184.re) * 0.2857142857142857)) +
              3.1415926535897931 * (b_t93_re_tmp * t221_im + t93_im_tmp *
    t221_re) * 0.5714285714285714) + 3.1415926535897931 * (t93_re * t184.im +
              t93_im * t184.re) * 0.5714285714285714) + -(3.1415926535897931 *
    (e_t93_re_tmp * t184.im + d_t93_im_tmp * t184.re) * 0.2857142857142857);
  re_tmp = 0.31830988618379069 * t94_tmp * t115;
  c_re_tmp = re_tmp * d1;
  b_im_tmp = re_tmp * 0.0;
  t249_re = (d * t223_re - 0.0 * t223_im) + -((c_re_tmp * t223_re - b_im_tmp *
    t223_im) * 0.875);
  t249_im = (d * t223_im + 0.0 * t223_re) + -((c_re_tmp * t223_im + b_im_tmp *
    t223_re) * 0.875);
  c_t104_re_tmp = t104_re_tmp * t223_re - t104_im_tmp * t223_im;
  c_t104_im_tmp = t104_re_tmp * t223_im + t104_im_tmp * t223_re;
  k_t42_re_tmp = c_t42_re_tmp_tmp * t105;
  l_t42_re_tmp = k_t42_re_tmp * t123_re;
  f_t42_im_tmp = k_t42_re_tmp * t123_im;
  b_t54_re_tmp = t54_re_tmp_tmp * t105;
  c_t54_re_tmp = b_t54_re_tmp * t123_re;
  t54_im_tmp = b_t54_re_tmp * t123_im;
  k_t42_re_tmp = d_t42_re_tmp_tmp * t105;
  m_t42_re_tmp = k_t42_re_tmp * t123_re;
  g_t42_im_tmp = k_t42_re_tmp * t123_im;
  k_t42_re_tmp = c_t42_re_tmp_tmp * t99;
  n_t42_re_tmp = k_t42_re_tmp * t123_re;
  h_t42_im_tmp = k_t42_re_tmp * t123_im;
  t42_re = 3.1415926535897931 * (n_t42_re_tmp * d - h_t42_im_tmp * 0.0);
  t42_im = 3.1415926535897931 * (n_t42_re_tmp * 0.0 + h_t42_im_tmp * d);
  b_t54_re_tmp = t54_re_tmp_tmp * t99;
  d_t54_re_tmp = b_t54_re_tmp * t123_re;
  b_t54_im_tmp = b_t54_re_tmp * t123_im;
  t54_re = 3.1415926535897931 * (d_t54_re_tmp * d - b_t54_im_tmp * 0.0);
  t56_re = 3.1415926535897931 * (d_t54_re_tmp * 0.0 + b_t54_im_tmp * d);
  k_t42_re_tmp = d_t42_re_tmp_tmp * t99;
  o_t42_re_tmp = k_t42_re_tmp * t123_re;
  i_t42_im_tmp = k_t42_re_tmp * t123_im;
  b_t42_re = 3.1415926535897931 * (o_t42_re_tmp * d - i_t42_im_tmp * 0.0);
  b_t42_im = 3.1415926535897931 * (o_t42_re_tmp * 0.0 + i_t42_im_tmp * d);
  b_t48_re_tmp = t48_re_tmp_tmp * t106;
  c_t48_re_tmp = b_t48_re_tmp * t123_re;
  t48_im_tmp = b_t48_re_tmp * t123_im;
  k_t42_re_tmp = f_t42_re_tmp * d1 - b_t42_im_tmp * 0.0;
  j_t42_im_tmp = f_t42_re_tmp * 0.0 + b_t42_im_tmp * d1;
  b_t48_re_tmp = t48_re_tmp_tmp * t100;
  d_t48_re_tmp = b_t48_re_tmp * t123_re;
  b_t48_im_tmp = b_t48_re_tmp * t123_im;
  t48_re = 3.1415926535897931 * (d_t48_re_tmp * d - b_t48_im_tmp * 0.0);
  t48_im = 3.1415926535897931 * (d_t48_re_tmp * 0.0 + b_t48_im_tmp * d);
  p_t42_re_tmp = g_t42_re_tmp * d1 - c_t42_im_tmp * 0.0;
  k_t42_im_tmp = g_t42_re_tmp * 0.0 + c_t42_im_tmp * d1;
  c_t42_re = 3.1415926535897931 * (p_t42_re_tmp * t223_re - k_t42_im_tmp *
    t223_im);
  c_t42_im = 3.1415926535897931 * (p_t42_re_tmp * t223_im + k_t42_im_tmp *
    t223_re);
  re_tmp_tmp_tmp = 9.869604401089358 * t48 * t54_tmp;
  re_tmp_tmp = re_tmp_tmp_tmp * t54_tmp;
  d_re_tmp = re_tmp_tmp * t69 * t105;
  e_re_tmp = d_re_tmp * t123_re;
  c_im_tmp = d_re_tmp * t123_im;
  Mxx->re = ((((((b_t104_re_tmp * t254_re - b_t104_im_tmp * t254_im) *
                 -1.0669676460233539 - (c_t104_re_tmp * t249_re - c_t104_im_tmp *
    t249_im) * 1.0669676460233539) - (l_t42_re_tmp * d - f_t42_im_tmp * 0.0) *
                1.0669676460233539) - (c_t54_re_tmp * d - t54_im_tmp * 0.0) *
               0.26674191150583842) + ((((m_t42_re_tmp * d - g_t42_im_tmp * 0.0)
    * 0.26674191150583842 + (t42_re * 0.0 - t42_im * 248.85542764393091)) +
    (t54_re * 0.0 - t56_re * 62.21385691098272)) - (b_t42_re * 0.0 - b_t42_im *
    62.21385691098272))) + ((((c_t48_re_tmp * d - t48_im_tmp * 0.0) *
    0.80022573451751533 + (k_t42_re_tmp * t223_re - j_t42_im_tmp * t223_im) *
    1.0669676460233539) - (t48_re * 0.0 - t48_im * 186.6415707329482)) -
              (c_t42_re * 0.0 - c_t42_im * 248.85542764393091))) + (e_re_tmp * d
    - c_im_tmp * 0.0) * -14510.520562328329;
  Mxx->im = ((((((b_t104_re_tmp * t254_im + b_t104_im_tmp * t254_re) *
                 -1.0669676460233539 - (c_t104_re_tmp * t249_im + c_t104_im_tmp *
    t249_re) * 1.0669676460233539) - (l_t42_re_tmp * 0.0 + f_t42_im_tmp * d) *
                1.0669676460233539) - (c_t54_re_tmp * 0.0 + t54_im_tmp * d) *
               0.26674191150583842) + ((((m_t42_re_tmp * 0.0 + g_t42_im_tmp * d)
    * 0.26674191150583842 + (t42_re * 248.85542764393091 + t42_im * 0.0)) +
    (t54_re * 62.21385691098272 + t56_re * 0.0)) - (b_t42_re * 62.21385691098272
    + b_t42_im * 0.0))) + ((((c_t48_re_tmp * 0.0 + t48_im_tmp * d) *
    0.80022573451751533 + (k_t42_re_tmp * t223_im + j_t42_im_tmp * t223_re) *
    1.0669676460233539) - (t48_re * 186.6415707329482 + t48_im * 0.0)) -
              (c_t42_re * 248.85542764393091 + c_t42_im * 0.0))) + (e_re_tmp *
    0.0 + c_im_tmp * d) * -14510.520562328329;
  t250_re = (d * t224_re - 0.0 * t224_im) + -((c_re_tmp * t224_re - b_im_tmp *
    t224_im) * 0.875);
  t250_im = (d * t224_im + 0.0 * t224_re) + -((c_re_tmp * t224_im + b_im_tmp *
    t224_re) * 0.875);
  j_t43_re_tmp = e_t43_re_tmp * d1 - b_t43_im_tmp * 0.0;
  f_t43_im_tmp = e_t43_re_tmp * 0.0 + b_t43_im_tmp * d1;
  t42_re = 3.1415926535897931 * (p_t42_re_tmp * t224_re - k_t42_im_tmp * t224_im);
  t42_im = 3.1415926535897931 * (p_t42_re_tmp * t224_im + k_t42_im_tmp * t224_re);
  k_t43_re_tmp = f_t43_re_tmp * d1 - c_t43_im_tmp * 0.0;
  g_t43_im_tmp = f_t43_re_tmp * 0.0 + c_t43_im_tmp * d1;
  t43_re = 3.1415926535897931 * (k_t43_re_tmp * t223_re - g_t43_im_tmp * t223_im);
  t43_im = 3.1415926535897931 * (k_t43_re_tmp * t223_im + g_t43_im_tmp * t223_re);
  q_t42_re_tmp = b_t42_re_tmp_tmp * t106;
  r_t42_re_tmp = q_t42_re_tmp * t123_re;
  l_t42_im_tmp = q_t42_re_tmp * t123_im;
  q_t42_re_tmp = b_t42_re_tmp_tmp * t100;
  s_t42_re_tmp = q_t42_re_tmp * t123_re;
  m_t42_im_tmp = q_t42_re_tmp * t123_im;
  b_t42_re = 3.1415926535897931 * (s_t42_re_tmp * d - m_t42_im_tmp * 0.0);
  b_t42_im = 3.1415926535897931 * (s_t42_re_tmp * 0.0 + m_t42_im_tmp * d);
  b_re_tmp_tmp_tmp = 9.869604401089358 * t42 * t43;
  b_re_tmp_tmp = b_re_tmp_tmp_tmp * t66;
  d_re_tmp = b_re_tmp_tmp * t67 * t82 * t85 * t105;
  f_re_tmp = d_re_tmp * t123_re;
  d_im_tmp = d_re_tmp * t123_im;
  Mxy->re = (((((b_t104_re_tmp * t256_re - b_t104_im_tmp * t256_im) *
                1.0669676460233539 - (c_t104_re_tmp * t250_re - c_t104_im_tmp *
    t250_im) * 1.0669676460233539) + (k_t42_re_tmp * t224_re - j_t42_im_tmp *
    t224_im) * 0.53348382301167685) + (j_t43_re_tmp * t223_re - f_t43_im_tmp *
    t223_im) * 0.53348382301167685) + ((((t42_re * -0.0 - t42_im *
    -124.4277138219654) - (t43_re * 0.0 - t43_im * 124.4277138219654)) +
    (r_t42_re_tmp * d - l_t42_im_tmp * 0.0) * 0.80022573451751533) - (b_t42_re *
    0.0 - b_t42_im * 186.6415707329482))) + (f_re_tmp * d - d_im_tmp * 0.0) *
    -14510.520562328329;
  Mxy->im = (((((b_t104_re_tmp * t256_im + b_t104_im_tmp * t256_re) *
                1.0669676460233539 - (c_t104_re_tmp * t250_im + c_t104_im_tmp *
    t250_re) * 1.0669676460233539) + (k_t42_re_tmp * t224_im + j_t42_im_tmp *
    t224_re) * 0.53348382301167685) + (j_t43_re_tmp * t223_im + f_t43_im_tmp *
    t223_re) * 0.53348382301167685) + ((((t42_re * -124.4277138219654 + t42_im *
    -0.0) - (t43_re * 124.4277138219654 + t43_im * 0.0)) + (r_t42_re_tmp * 0.0 +
    l_t42_im_tmp * d) * 0.80022573451751533) - (b_t42_re * 186.6415707329482 +
    b_t42_im * 0.0))) + (f_re_tmp * 0.0 + d_im_tmp * d) * -14510.520562328329;
  d_t104_re_tmp = t104_re_tmp * t222_re - t104_im_tmp * t222_im;
  d_t104_im_tmp = t104_re_tmp * t222_im + t104_im_tmp * t222_re;
  t44_re_tmp = b_t44_re_tmp * d1 - t44_im_tmp * 0.0;
  c_t44_im_tmp = b_t44_re_tmp * 0.0 + t44_im_tmp * d1;
  d_t44_re_tmp = c_t44_re_tmp * d1 - b_t44_im_tmp * 0.0;
  d_t44_im_tmp = c_t44_re_tmp * 0.0 + b_t44_im_tmp * d1;
  t44_re = 3.1415926535897931 * (d_t44_re_tmp * t223_re - d_t44_im_tmp * t223_im);
  t44_im = 3.1415926535897931 * (d_t44_re_tmp * t223_im + d_t44_im_tmp * t223_re);
  b_t42_re_tmp_tmp_tmp_tmp = t42 * t44;
  c_t42_re_tmp_tmp_tmp = b_t42_re_tmp_tmp_tmp_tmp * t66;
  b_t42_re_tmp_tmp = c_t42_re_tmp_tmp_tmp * t68 * t82 * t88;
  q_t42_re_tmp = b_t42_re_tmp_tmp * t106;
  t_t42_re_tmp = q_t42_re_tmp * t123_re;
  n_t42_im_tmp = q_t42_re_tmp * t123_im;
  q_t42_re_tmp = b_t42_re_tmp_tmp * t100;
  u_t42_re_tmp = q_t42_re_tmp * t123_re;
  o_t42_im_tmp = q_t42_re_tmp * t123_im;
  t42_re = 3.1415926535897931 * (u_t42_re_tmp * d - o_t42_im_tmp * 0.0);
  t42_im = 3.1415926535897931 * (u_t42_re_tmp * 0.0 + o_t42_im_tmp * d);
  c_re_tmp_tmp_tmp = 9.869604401089358 * t42 * t44;
  c_re_tmp_tmp = c_re_tmp_tmp_tmp * t66;
  d_re_tmp = c_re_tmp_tmp * t68 * t82 * t88 * t105;
  g_re_tmp = d_re_tmp * t123_re;
  e_im_tmp = d_re_tmp * t123_im;
  b_t42_re_tmp_tmp = e_t42_re_tmp_tmp * t93 * t105;
  q_t42_re_tmp = b_t42_re_tmp_tmp * t119;
  v_t42_re_tmp = q_t42_re_tmp * t123_re;
  p_t42_im_tmp = q_t42_re_tmp * t123_im;
  q_t42_re_tmp = v_t42_re_tmp * t128.re - p_t42_im_tmp * t128.im;
  q_t42_im_tmp = v_t42_re_tmp * t128.im + p_t42_im_tmp * t128.re;
  d_re_tmp_tmp = 9.869604401089358 * t42 * t66;
  e_re_tmp_tmp = d_re_tmp_tmp * t68 * t82 * t99 * t93;
  d_re_tmp = e_re_tmp_tmp * t119;
  h_re_tmp = d_re_tmp * t123_re;
  f_im_tmp = d_re_tmp * t123_im;
  d_re_tmp = h_re_tmp * t128.re - f_im_tmp * t128.im;
  g_im_tmp = h_re_tmp * t128.im + f_im_tmp * t128.re;
  c_re = d_re_tmp * d1 - g_im_tmp * 0.0;
  c_im = d_re_tmp * 0.0 + g_im_tmp * d1;
  Mxz->re = (((((b_t104_re_tmp * t243_re - b_t104_im_tmp * t243_im) *
                1.0669676460233539 - (d_t104_re_tmp * t223_re - d_t104_im_tmp *
    t223_im) * 1.0669676460233539) + (t44_re_tmp * t223_re - c_t44_im_tmp *
    t223_im) * 0.53348382301167685) - (t44_re * 0.0 - t44_im * 124.4277138219654))
             + ((((t_t42_re_tmp * d - n_t42_im_tmp * 0.0) * 0.80022573451751533
                  - (t42_re * 0.0 - t42_im * 186.6415707329482)) - (g_re_tmp * d
    - e_im_tmp * 0.0) * 14510.520562328329) - 3.1415926535897931 * (q_t42_re_tmp
    * d1 - q_t42_im_tmp * 0.0) * 0.3048478988638153)) + (c_re * 0.0 - c_im *
    71.101550755408823);
  Mxz->im = (((((b_t104_re_tmp * t243_im + b_t104_im_tmp * t243_re) *
                1.0669676460233539 - (d_t104_re_tmp * t223_im + d_t104_im_tmp *
    t223_re) * 1.0669676460233539) + (t44_re_tmp * t223_im + c_t44_im_tmp *
    t223_re) * 0.53348382301167685) - (t44_re * 124.4277138219654 + t44_im * 0.0))
             + ((((t_t42_re_tmp * 0.0 + n_t42_im_tmp * d) * 0.80022573451751533
                  - (t42_re * 186.6415707329482 + t42_im * 0.0)) - (g_re_tmp *
    0.0 + e_im_tmp * d) * 14510.520562328329) - 3.1415926535897931 *
                (q_t42_re_tmp * 0.0 + q_t42_im_tmp * d1) * 0.3048478988638153))
    + (c_re * 71.101550755408823 + c_im * 0.0);
  e_t104_re_tmp = t104_re_tmp * t224_re - t104_im_tmp * t224_im;
  e_t104_im_tmp = t104_re_tmp * t224_im + t104_im_tmp * t224_re;
  l_t43_re_tmp = b_t43_re_tmp_tmp * t105;
  m_t43_re_tmp = l_t43_re_tmp * t123_re;
  h_t43_im_tmp = l_t43_re_tmp * t123_im;
  b_t56_re_tmp = t56_re_tmp_tmp * t105;
  c_t56_re_tmp = b_t56_re_tmp * t123_re;
  t56_im_tmp = b_t56_re_tmp * t123_im;
  l_t43_re_tmp = c_t43_re_tmp_tmp * t105;
  n_t43_re_tmp = l_t43_re_tmp * t123_re;
  i_t43_im_tmp = l_t43_re_tmp * t123_im;
  l_t43_re_tmp = b_t43_re_tmp_tmp * t99;
  o_t43_re_tmp = l_t43_re_tmp * t123_re;
  j_t43_im_tmp = l_t43_re_tmp * t123_im;
  t43_re = 3.1415926535897931 * (o_t43_re_tmp * d - j_t43_im_tmp * 0.0);
  t43_im = 3.1415926535897931 * (o_t43_re_tmp * 0.0 + j_t43_im_tmp * d);
  b_t56_re_tmp = t56_re_tmp_tmp * t99;
  d_t56_re_tmp = b_t56_re_tmp * t123_re;
  b_t56_im_tmp = b_t56_re_tmp * t123_im;
  t56_re = 3.1415926535897931 * (d_t56_re_tmp * d - b_t56_im_tmp * 0.0);
  t48_re_tmp_tmp = 3.1415926535897931 * (d_t56_re_tmp * 0.0 + b_t56_im_tmp * d);
  l_t43_re_tmp = c_t43_re_tmp_tmp * t99;
  p_t43_re_tmp = l_t43_re_tmp * t123_re;
  k_t43_im_tmp = l_t43_re_tmp * t123_im;
  b_t43_re = 3.1415926535897931 * (p_t43_re_tmp * d - k_t43_im_tmp * 0.0);
  b_t43_im = 3.1415926535897931 * (p_t43_re_tmp * 0.0 + k_t43_im_tmp * d);
  b_t50_re_tmp = t50_re_tmp_tmp * t106;
  c_t50_re_tmp = b_t50_re_tmp * t123_re;
  t50_im_tmp = b_t50_re_tmp * t123_im;
  b_t50_re_tmp = t50_re_tmp_tmp * t100;
  d_t50_re_tmp = b_t50_re_tmp * t123_re;
  b_t50_im_tmp = b_t50_re_tmp * t123_im;
  t48_re = 3.1415926535897931 * (d_t50_re_tmp * d - b_t50_im_tmp * 0.0);
  t50_im = 3.1415926535897931 * (d_t50_re_tmp * 0.0 + b_t50_im_tmp * d);
  c_t43_re = 3.1415926535897931 * (k_t43_re_tmp * t224_re - g_t43_im_tmp *
    t224_im);
  c_t43_im = 3.1415926535897931 * (k_t43_re_tmp * t224_im + g_t43_im_tmp *
    t224_re);
  d_re_tmp_tmp_tmp = 9.869604401089358 * t50 * t56_tmp;
  f_re_tmp_tmp = d_re_tmp_tmp_tmp * t56_tmp;
  i_re_tmp = f_re_tmp_tmp * t71 * t105;
  j_re_tmp = i_re_tmp * t123_re;
  h_im_tmp = i_re_tmp * t123_im;
  Myy->re = ((((((b_t104_re_tmp * t255_re - b_t104_im_tmp * t255_im) *
                 -1.0669676460233539 - (e_t104_re_tmp * t250_re - e_t104_im_tmp *
    t250_im) * 1.0669676460233539) - (m_t43_re_tmp * d - h_t43_im_tmp * 0.0) *
                1.0669676460233539) - (c_t56_re_tmp * d - t56_im_tmp * 0.0) *
               0.26674191150583842) + ((((n_t43_re_tmp * d - i_t43_im_tmp * 0.0)
    * 0.26674191150583842 + (t43_re * 0.0 - t43_im * 248.85542764393091)) +
    (t56_re * 0.0 - t48_re_tmp_tmp * 62.21385691098272)) - (b_t43_re * 0.0 -
    b_t43_im * 62.21385691098272))) + ((((c_t50_re_tmp * d - t50_im_tmp * 0.0) *
    0.80022573451751533 + (j_t43_re_tmp * t224_re - f_t43_im_tmp * t224_im) *
    1.0669676460233539) - (t48_re * 0.0 - t50_im * 186.6415707329482)) -
              (c_t43_re * 0.0 - c_t43_im * 248.85542764393091))) + (j_re_tmp * d
    - h_im_tmp * 0.0) * -14510.520562328329;
  Myy->im = ((((((b_t104_re_tmp * t255_im + b_t104_im_tmp * t255_re) *
                 -1.0669676460233539 - (e_t104_re_tmp * t250_im + e_t104_im_tmp *
    t250_re) * 1.0669676460233539) - (m_t43_re_tmp * 0.0 + h_t43_im_tmp * d) *
                1.0669676460233539) - (c_t56_re_tmp * 0.0 + t56_im_tmp * d) *
               0.26674191150583842) + ((((n_t43_re_tmp * 0.0 + i_t43_im_tmp * d)
    * 0.26674191150583842 + (t43_re * 248.85542764393091 + t43_im * 0.0)) +
    (t56_re * 62.21385691098272 + t48_re_tmp_tmp * 0.0)) - (b_t43_re *
    62.21385691098272 + b_t43_im * 0.0))) + ((((c_t50_re_tmp * 0.0 + t50_im_tmp *
    d) * 0.80022573451751533 + (j_t43_re_tmp * t224_im + f_t43_im_tmp * t224_re)
    * 1.0669676460233539) - (t48_re * 186.6415707329482 + t50_im * 0.0)) -
              (c_t43_re * 248.85542764393091 + c_t43_im * 0.0))) + (j_re_tmp *
    0.0 + h_im_tmp * d) * -14510.520562328329;
  t44_re = 3.1415926535897931 * (d_t44_re_tmp * t224_re - d_t44_im_tmp * t224_im);
  t44_im = 3.1415926535897931 * (d_t44_re_tmp * t224_im + d_t44_im_tmp * t224_re);
  t43_re_tmp_tmp_tmp_tmp = t43 * t44;
  b_t43_re_tmp_tmp_tmp = t43_re_tmp_tmp_tmp_tmp * t67;
  b_t43_re_tmp_tmp = b_t43_re_tmp_tmp_tmp * t68 * t85 * t88;
  l_t43_re_tmp = b_t43_re_tmp_tmp * t106;
  q_t43_re_tmp = l_t43_re_tmp * t123_re;
  l_t43_im_tmp = l_t43_re_tmp * t123_im;
  l_t43_re_tmp = b_t43_re_tmp_tmp * t100;
  r_t43_re_tmp = l_t43_re_tmp * t123_re;
  m_t43_im_tmp = l_t43_re_tmp * t123_im;
  t43_re = 3.1415926535897931 * (r_t43_re_tmp * d - m_t43_im_tmp * 0.0);
  t43_im = 3.1415926535897931 * (r_t43_re_tmp * 0.0 + m_t43_im_tmp * d);
  e_re_tmp_tmp_tmp = 9.869604401089358 * t43 * t44;
  g_re_tmp_tmp = e_re_tmp_tmp_tmp * t67;
  i_re_tmp = g_re_tmp_tmp * t68 * t85 * t88 * t105;
  k_re_tmp = i_re_tmp * t123_re;
  i_im_tmp = i_re_tmp * t123_im;
  b_t43_re_tmp_tmp = d_t43_re_tmp_tmp * t93 * t105;
  l_t43_re_tmp = b_t43_re_tmp_tmp * t119;
  s_t43_re_tmp = l_t43_re_tmp * t123_re;
  n_t43_im_tmp = l_t43_re_tmp * t123_im;
  l_t43_re_tmp = s_t43_re_tmp * t128.re - n_t43_im_tmp * t128.im;
  o_t43_im_tmp = s_t43_re_tmp * t128.im + n_t43_im_tmp * t128.re;
  h_re_tmp_tmp = 9.869604401089358 * t43 * t67;
  i_re_tmp_tmp = h_re_tmp_tmp * t68 * t85 * t99 * t93;
  i_re_tmp = i_re_tmp_tmp * t119;
  l_re_tmp = i_re_tmp * t123_re;
  j_im_tmp = i_re_tmp * t123_im;
  i_re_tmp = l_re_tmp * t128.re - j_im_tmp * t128.im;
  k_im_tmp = l_re_tmp * t128.im + j_im_tmp * t128.re;
  c_re = i_re_tmp * d1 - k_im_tmp * 0.0;
  c_im = i_re_tmp * 0.0 + k_im_tmp * d1;
  Myz->re = (((((b_t104_re_tmp * t244_re - b_t104_im_tmp * t244_im) *
                1.0669676460233539 - (d_t104_re_tmp * t224_re - d_t104_im_tmp *
    t224_im) * 1.0669676460233539) + (t44_re_tmp * t224_re - c_t44_im_tmp *
    t224_im) * 0.53348382301167685) - (t44_re * 0.0 - t44_im * 124.4277138219654))
             + ((((q_t43_re_tmp * d - l_t43_im_tmp * 0.0) * 0.80022573451751533
                  - (t43_re * 0.0 - t43_im * 186.6415707329482)) - (k_re_tmp * d
    - i_im_tmp * 0.0) * 14510.520562328329) - 3.1415926535897931 * (l_t43_re_tmp
    * d1 - o_t43_im_tmp * 0.0) * 0.3048478988638153)) + (c_re * 0.0 - c_im *
    71.101550755408823);
  Myz->im = (((((b_t104_re_tmp * t244_im + b_t104_im_tmp * t244_re) *
                1.0669676460233539 - (d_t104_re_tmp * t224_im + d_t104_im_tmp *
    t224_re) * 1.0669676460233539) + (t44_re_tmp * t224_im + c_t44_im_tmp *
    t224_re) * 0.53348382301167685) - (t44_re * 124.4277138219654 + t44_im * 0.0))
             + ((((q_t43_re_tmp * 0.0 + l_t43_im_tmp * d) * 0.80022573451751533
                  - (t43_re * 186.6415707329482 + t43_im * 0.0)) - (k_re_tmp *
    0.0 + i_im_tmp * d) * 14510.520562328329) - 3.1415926535897931 *
                (l_t43_re_tmp * 0.0 + o_t43_im_tmp * d1) * 0.3048478988638153))
    + (c_re * 71.101550755408823 + c_im * 0.0);
  b_t44_re_tmp_tmp = t44 * t88;
  e_t44_re_tmp = b_t44_re_tmp_tmp * t105;
  f_t44_re_tmp = e_t44_re_tmp * t123_re;
  e_t44_im_tmp = e_t44_re_tmp * t123_im;
  t58_re_tmp_tmp_tmp = t58_tmp * t58_tmp;
  t58_re_tmp_tmp = t58_re_tmp_tmp_tmp * t73;
  t58_re_tmp = t58_re_tmp_tmp * t105;
  b_t58_re_tmp = t58_re_tmp * t123_re;
  t58_im_tmp = t58_re_tmp * t123_im;
  b_t44_re_tmp_tmp_tmp = t44 * t73;
  c_t44_re_tmp_tmp = b_t44_re_tmp_tmp_tmp * t89;
  e_t44_re_tmp = c_t44_re_tmp_tmp * t105;
  g_t44_re_tmp = e_t44_re_tmp * t123_re;
  f_t44_im_tmp = e_t44_re_tmp * t123_im;
  e_t44_re_tmp = b_t44_re_tmp_tmp * t99;
  h_t44_re_tmp = e_t44_re_tmp * t123_re;
  g_t44_im_tmp = e_t44_re_tmp * t123_im;
  t44_re = 3.1415926535897931 * (h_t44_re_tmp * d - g_t44_im_tmp * 0.0);
  t44_im = 3.1415926535897931 * (h_t44_re_tmp * 0.0 + g_t44_im_tmp * d);
  t58_re_tmp = t58_re_tmp_tmp * t99;
  c_t58_re_tmp = t58_re_tmp * t123_re;
  b_t58_im_tmp = t58_re_tmp * t123_im;
  t48_im = 3.1415926535897931 * (c_t58_re_tmp * d - b_t58_im_tmp * 0.0);
  t58_im = 3.1415926535897931 * (c_t58_re_tmp * 0.0 + b_t58_im_tmp * d);
  e_t44_re_tmp = c_t44_re_tmp_tmp * t99;
  i_t44_re_tmp = e_t44_re_tmp * t123_re;
  h_t44_im_tmp = e_t44_re_tmp * t123_im;
  b_t44_re = 3.1415926535897931 * (i_t44_re_tmp * d - h_t44_im_tmp * 0.0);
  b_t44_im = 3.1415926535897931 * (i_t44_re_tmp * 0.0 + h_t44_im_tmp * d);
  f_t93_re_tmp = t93 * t104 * t119;
  g_t93_re_tmp = f_t93_re_tmp * t123_re;
  e_t93_im_tmp = f_t93_re_tmp * t123_im;
  f_t93_re_tmp = g_t93_re_tmp * t128.re - e_t93_im_tmp * t128.im;
  e_t93_im_tmp = g_t93_re_tmp * t128.im + e_t93_im_tmp * t128.re;
  t52_re_tmp_tmp_tmp_tmp = t52 * t58_tmp;
  t52_re_tmp_tmp_tmp = t52_re_tmp_tmp_tmp_tmp * t58_tmp;
  t52_re_tmp_tmp = t52_re_tmp_tmp_tmp * t73;
  t52_re_tmp = t52_re_tmp_tmp * t106;
  b_t52_re_tmp = t52_re_tmp * t123_re;
  t52_im_tmp = t52_re_tmp * t123_im;
  t52_re_tmp = t52_re_tmp_tmp * t100;
  c_t52_re_tmp = t52_re_tmp * t123_re;
  b_t52_im_tmp = t52_re_tmp * t123_im;
  t48_re = 3.1415926535897931 * (c_t52_re_tmp * d - b_t52_im_tmp * 0.0);
  t54_re = 3.1415926535897931 * (c_t52_re_tmp * 0.0 + b_t52_im_tmp * d);
  t73_re_tmp_tmp_tmp = t73 * t93;
  t73_re_tmp_tmp = t73_re_tmp_tmp_tmp * t104;
  t73_re_tmp = t73_re_tmp_tmp * t119;
  b_t73_re_tmp = t73_re_tmp * t123_re;
  t73_im_tmp = t73_re_tmp * t123_im;
  t73_re_tmp = b_t73_re_tmp * t129.re - t73_im_tmp * t129.im;
  t73_im_tmp = b_t73_re_tmp * t129.im + t73_im_tmp * t129.re;
  f_re_tmp_tmp_tmp = 9.869604401089358 * t52 * t58_tmp;
  j_re_tmp_tmp = f_re_tmp_tmp_tmp * t58_tmp;
  m_re_tmp = j_re_tmp_tmp * t73 * t105;
  n_re_tmp = m_re_tmp * t123_re;
  l_im_tmp = m_re_tmp * t123_im;
  b_t73_re_tmp = t73_re_tmp_tmp * t113_re;
  b_t73_im_tmp = t73_re_tmp_tmp * t113_im;
  c_t73_re_tmp = t120 * (b_t73_re_tmp * t116_re - b_t73_im_tmp * t116_im);
  c_t73_im_tmp = t120 * (b_t73_re_tmp * t116_im + b_t73_im_tmp * t116_re);
  t73_re = c_t73_re_tmp * t123_re - c_t73_im_tmp * t123_im;
  t73_im = c_t73_re_tmp * t123_im + c_t73_im_tmp * t123_re;
  e_t44_re_tmp = b_t44_re_tmp_tmp_tmp * t88 * t93 * t105 * t119;
  j_t44_re_tmp = e_t44_re_tmp * t123_re;
  i_t44_im_tmp = e_t44_re_tmp * t123_im;
  e_t44_re_tmp = j_t44_re_tmp * t128.re - i_t44_im_tmp * t128.im;
  i_t44_im_tmp = j_t44_re_tmp * t128.im + i_t44_im_tmp * t128.re;
  m_re_tmp = 9.869604401089358 * t44 * t73 * t88 * t99 * t93 * t119;
  o_re_tmp = m_re_tmp * t123_re;
  m_im_tmp = m_re_tmp * t123_im;
  m_re_tmp = o_re_tmp * t128.re - m_im_tmp * t128.im;
  m_im_tmp = o_re_tmp * t128.im + m_im_tmp * t128.re;
  c_re = m_re_tmp * d1 - m_im_tmp * 0.0;
  c_im = m_re_tmp * 0.0 + m_im_tmp * d1;
  Mzz->re = ((((((f_t44_re_tmp * d - e_t44_im_tmp * 0.0) * -1.0669676460233539 -
                 (b_t58_re_tmp * d - t58_im_tmp * 0.0) * 0.26674191150583842) +
                (g_t44_re_tmp * d - f_t44_im_tmp * 0.0) * 0.26674191150583842) +
               (t44_re * 0.0 - t44_im * 248.85542764393091)) + ((((t48_im * 0.0
    - t58_im * 62.21385691098272) - (b_t44_re * 0.0 - b_t44_im *
    62.21385691098272)) + 3.1415926535897931 * (f_t93_re_tmp * d1 - e_t93_im_tmp
    * 0.0) * 1.219391595455261) + (b_t52_re_tmp * d - t52_im_tmp * 0.0) *
    0.80022573451751533)) + ((((t48_re * -0.0 - t54_re * -186.6415707329482) -
    3.1415926535897931 * (t73_re_tmp * d1 - t73_im_tmp * 0.0) *
    0.3048478988638153) + 3.1415926535897931 * (d_t68_re_tmp * t222_re -
    d_t68_im_tmp * t222_im) * 0.60969579772763072) - (n_re_tmp * d - l_im_tmp *
    0.0) * 14510.520562328329)) + ((3.1415926535897931 * (t73_re * d1 - t73_im *
    0.0) * 0.60969579772763072 - 3.1415926535897931 * (e_t44_re_tmp * d1 -
    i_t44_im_tmp * 0.0) * 0.60969579772763072) + (c_re * 0.0 - c_im *
    142.2031015108177));
  Mzz->im = ((((((f_t44_re_tmp * 0.0 + e_t44_im_tmp * d) * -1.0669676460233539 -
                 (b_t58_re_tmp * 0.0 + t58_im_tmp * d) * 0.26674191150583842) +
                (g_t44_re_tmp * 0.0 + f_t44_im_tmp * d) * 0.26674191150583842) +
               (t44_re * 248.85542764393091 + t44_im * 0.0)) + ((((t48_im *
    62.21385691098272 + t58_im * 0.0) - (b_t44_re * 62.21385691098272 + b_t44_im
    * 0.0)) + 3.1415926535897931 * (f_t93_re_tmp * 0.0 + e_t93_im_tmp * d1) *
    1.219391595455261) + (b_t52_re_tmp * 0.0 + t52_im_tmp * d) *
    0.80022573451751533)) + ((((t48_re * -186.6415707329482 + t54_re * -0.0) -
    3.1415926535897931 * (t73_re_tmp * 0.0 + t73_im_tmp * d1) *
    0.3048478988638153) + 3.1415926535897931 * (d_t68_re_tmp * t222_im +
    d_t68_im_tmp * t222_re) * 0.60969579772763072) - (n_re_tmp * 0.0 + l_im_tmp *
    d) * 14510.520562328329)) + ((3.1415926535897931 * (t73_re * 0.0 + t73_im *
    d1) * 0.60969579772763072 - 3.1415926535897931 * (e_t44_re_tmp * 0.0 +
    i_t44_im_tmp * d1) * 0.60969579772763072) + (c_re * 142.2031015108177 + c_im
    * 0.0));
  f_t104_re_tmp = t104_re_tmp * t249_re - t104_im_tmp * t249_im;
  f_t104_im_tmp = t104_re_tmp * t249_im + t104_im_tmp * t249_re;
  t54_re_tmp_tmp = t54_re_tmp_tmp_tmp * t66;
  b_t54_re_tmp = t54_re_tmp_tmp * t94_tmp;
  t54_re = b_t54_re_tmp * t109_re;
  c_t54_im_tmp = b_t54_re_tmp * t93;
  b_t54_re = b_t54_re_tmp * t112_re;
  c_t44_re_tmp_tmp = t54_tmp * t61 * t69;
  d_t43_re_tmp_tmp = c_t44_re_tmp_tmp * t94_tmp;
  c_t54_re = d_t43_re_tmp_tmp * t109_re;
  b_t44_re_tmp_tmp_tmp = d_t43_re_tmp_tmp * t93;
  t55_re_tmp_tmp = t55 * t54_tmp * t69;
  t55_re_tmp = t55_re_tmp_tmp * t94_tmp;
  t55_re = t55_re_tmp * t109_re;
  b_t44_re_tmp_tmp = t55_re_tmp * t93;
  b_t55_re_tmp_tmp = t55 * t61 * t70;
  c_t43_re_tmp_tmp = b_t55_re_tmp_tmp * t94_tmp;
  b_t55_re = c_t43_re_tmp_tmp * t109_re;
  t58_re_tmp_tmp = c_t43_re_tmp_tmp * t93;
  d_t54_re = d_t43_re_tmp_tmp * t112_re;
  c_t55_re = t55_re_tmp * t112_re;
  d_t55_re = c_t43_re_tmp_tmp * t112_re;
  c_t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t83;
  w_t42_re_tmp = c_t42_re_tmp_tmp * t94_tmp;
  t42_re = w_t42_re_tmp * t109_re;
  r_t42_im_tmp = w_t42_re_tmp * t93;
  b_t54_re_tmp_tmp = t54_re_tmp_tmp_tmp * t78;
  e_t54_re_tmp = b_t54_re_tmp_tmp * t94_tmp;
  e_t54_re = e_t54_re_tmp * t109_re;
  d_t54_im_tmp = e_t54_re_tmp * t93;
  b_t42_re = w_t42_re_tmp * t112_re;
  f_t54_re = e_t54_re_tmp * t112_re;
  d_t42_re_tmp_tmp = t42 * t70 * rt_powd_snf(t82, 5.0);
  x_t42_re_tmp = d_t42_re_tmp_tmp * t94_tmp;
  c_t42_re = x_t42_re_tmp * t109_re;
  s_t42_im_tmp = x_t42_re_tmp * t93;
  d_t42_re = x_t42_re_tmp * t112_re;
  e_t42_re_tmp_tmp = t42 * t78 * t83;
  y_t42_re_tmp = e_t42_re_tmp_tmp * t94_tmp;
  e_t42_re = y_t42_re_tmp * t109_re;
  t_t42_im_tmp = y_t42_re_tmp * t93;
  f_t42_re = y_t42_re_tmp * t112_re;
  t48_re_tmp_tmp = t48_re_tmp_tmp_tmp * t66;
  b_t48_re_tmp = t48_re_tmp_tmp * t95;
  t48_re = b_t48_re_tmp * t109_re;
  c_t48_im_tmp = b_t48_re_tmp * t93;
  b_t48_re = b_t48_re_tmp * t112_re;
  b_t48_re_tmp_tmp = t41_tmp * t61 * t69;
  t56_re_tmp_tmp = b_t48_re_tmp_tmp * t95;
  c_t48_re = t56_re_tmp_tmp * t109_re;
  t81 = t56_re_tmp_tmp * t93;
  c_t48_re_tmp_tmp = t48 * t55;
  t52_re_tmp_tmp = c_t48_re_tmp_tmp * t54_tmp * t69;
  t50_re_tmp_tmp = t52_re_tmp_tmp * t95;
  b_t56_re_tmp = t50_re_tmp_tmp * t109_re;
  t98 = t50_re_tmp_tmp * t93;
  c_t48_re_tmp_tmp = c_t48_re_tmp_tmp * t61 * t70;
  e_t48_re_tmp = c_t48_re_tmp_tmp * t95;
  b_t50_re_tmp = e_t48_re_tmp * t109_re;
  t41_tmp = e_t48_re_tmp * t93;
  d_t48_re = t56_re_tmp_tmp * t112_re;
  e_t48_re = t50_re_tmp_tmp * t112_re;
  f_t48_re = e_t48_re_tmp * t112_re;
  d_t48_re_tmp_tmp = t48_re_tmp_tmp_tmp * t78;
  f_t48_re_tmp = d_t48_re_tmp_tmp * t95;
  g_t48_re = f_t48_re_tmp * t109_re;
  t103 = f_t48_re_tmp * t93;
  h_t48_re = f_t48_re_tmp * t112_re;
  g_t42_re_tmp_tmp = t42 * t54_tmp * t54_tmp * t70 * t82;
  ab_t42_re_tmp = g_t42_re_tmp_tmp * t95;
  g_t42_re = ab_t42_re_tmp * t109_re;
  u_t42_im_tmp = ab_t42_re_tmp * t93;
  h_t42_re = ab_t42_re_tmp * t112_re;
  t49_re_tmp_tmp = t49 * t54_tmp * t54_tmp * t70 * t82;
  t49_re_tmp = t49_re_tmp_tmp * t96;
  t49_re = t49_re_tmp * t109_re;
  t49_im_tmp = t49_re_tmp * t93;
  o_re_tmp = t49_re_tmp * t112_re;
  g_t93_re_tmp = 3.1415926535897931 * b_t93_re_tmp;
  f_t93_im_tmp = 3.1415926535897931 * t93_im_tmp;
  i_t48_re = ((((((((((((((((((t48_re_tmp_tmp * t91 - b_t48_re_tmp_tmp * t91 /
    2.0) - t52_re_tmp_tmp * t91 / 2.0) - c_t48_re_tmp_tmp * t91 / 4.0) +
    d_t48_re_tmp_tmp * t91 / 2.0) - (t54_re * 0.0 - c_t54_im_tmp * 0.5)) -
    (b_t54_re * 0.0 - c_t54_im_tmp * 0.5)) + (c_t54_re * 0.0 -
    b_t44_re_tmp_tmp_tmp * 0.25)) + (t55_re * 0.0 - b_t44_re_tmp_tmp * 0.25)) +
                       (b_t55_re * 0.0 - t58_re_tmp_tmp * 0.125)) + (d_t54_re *
    0.0 - b_t44_re_tmp_tmp_tmp * 0.25)) + (c_t55_re * 0.0 - b_t44_re_tmp_tmp *
    0.25)) + (d_t55_re * 0.0 - t58_re_tmp_tmp * 0.125)) + (t42_re * 0.0 -
    r_t42_im_tmp * 0.5)) - (e_t54_re * 0.0 - d_t54_im_tmp * 0.25)) + (b_t42_re *
    0.0 - r_t42_im_tmp * 0.5)) - (f_t54_re * 0.0 - d_t54_im_tmp * 0.25)) -
               (c_t42_re * 0.0 - s_t42_im_tmp * 0.375)) +
              ((((((((((((((((d_t42_re * -0.0 - s_t42_im_tmp * -0.375) +
    (e_t42_re * 0.0 - t_t42_im_tmp * 0.25)) + (f_t42_re * 0.0 - t_t42_im_tmp *
    0.25)) + g_t42_re_tmp_tmp * t91 * 0.75) - t49_re_tmp_tmp * t92 * 0.75) +
    (t48_re * 0.0 - c_t48_im_tmp * 0.5)) + (b_t48_re * 0.0 - c_t48_im_tmp * 0.5))
                       - (c_t48_re * 0.0 - t81 * 0.25)) - (b_t56_re_tmp * 0.0 -
    t98 * 0.25)) - (b_t50_re_tmp * 0.0 - t41_tmp * 0.125)) - (d_t48_re * 0.0 -
    t81 * 0.25)) - (e_t48_re * 0.0 - t98 * 0.25)) - (f_t48_re * 0.0 - t41_tmp *
    0.125)) + (g_t48_re * 0.0 - t103 * 0.25)) + (h_t48_re * 0.0 - t103 * 0.25))
               + (g_t42_re * 0.0 - u_t42_im_tmp * 0.375))) + (((h_t42_re * 0.0 -
    u_t42_im_tmp * 0.375) - (t49_re * 0.0 - t49_im_tmp * 0.375)) - (o_re_tmp *
    0.0 - t49_im_tmp * 0.375));
  t48_im = ((((((((((((((0.0 - (t54_re * 0.5 + c_t54_im_tmp * 0.0)) - (b_t54_re *
    0.5 + c_t54_im_tmp * 0.0)) + (c_t54_re * 0.25 + b_t44_re_tmp_tmp_tmp * 0.0))
                      + (t55_re * 0.25 + b_t44_re_tmp_tmp * 0.0)) + (b_t55_re *
    0.125 + t58_re_tmp_tmp * 0.0)) + (d_t54_re * 0.25 + b_t44_re_tmp_tmp_tmp *
    0.0)) + (c_t55_re * 0.25 + b_t44_re_tmp_tmp * 0.0)) + (d_t55_re * 0.125 +
    t58_re_tmp_tmp * 0.0)) + (t42_re * 0.5 + r_t42_im_tmp * 0.0)) - (e_t54_re *
    0.25 + d_t54_im_tmp * 0.0)) + (b_t42_re * 0.5 + r_t42_im_tmp * 0.0)) -
              (f_t54_re * 0.25 + d_t54_im_tmp * 0.0)) - (c_t42_re * 0.375 +
              s_t42_im_tmp * 0.0)) + ((((((((((((((d_t42_re * -0.375 +
    s_t42_im_tmp * -0.0) + (e_t42_re * 0.25 + t_t42_im_tmp * 0.0)) + (f_t42_re *
    0.25 + t_t42_im_tmp * 0.0)) + (t48_re * 0.5 + c_t48_im_tmp * 0.0)) +
    (b_t48_re * 0.5 + c_t48_im_tmp * 0.0)) - (c_t48_re * 0.25 + t81 * 0.0)) -
    (b_t56_re_tmp * 0.25 + t98 * 0.0)) - (b_t50_re_tmp * 0.125 + t41_tmp * 0.0))
    - (d_t48_re * 0.25 + t81 * 0.0)) - (e_t48_re * 0.25 + t98 * 0.0)) -
    (f_t48_re * 0.125 + t41_tmp * 0.0)) + (g_t48_re * 0.25 + t103 * 0.0)) +
              (h_t48_re * 0.25 + t103 * 0.0)) + (g_t42_re * 0.375 + u_t42_im_tmp
              * 0.0))) + (((h_t42_re * 0.375 + u_t42_im_tmp * 0.0) - (t49_re *
    0.375 + t49_im_tmp * 0.0)) - (o_re_tmp * 0.375 + t49_im_tmp * 0.0));
  h_t93_re_tmp = t93_re_tmp * t130_re;
  g_t93_im_tmp = t93_re_tmp * t130_im;
  t93_re_tmp = t93 * t114_re;
  h_t93_im_tmp = t93 * t114_im;
  i_t93_re_tmp = t120 * (t93_re_tmp * t117_re - h_t93_im_tmp * t117_im);
  h_t93_im_tmp = t120 * (t93_re_tmp * t117_im + h_t93_im_tmp * t117_re);
  c_t42_re_tmp *= t119;
  bb_t42_re_tmp = c_t42_re_tmp * t128.re;
  r_t42_im_tmp = c_t42_re_tmp * t128.im;
  t93_re_tmp = t121 * t93_re_tmp_tmp;
  i_t93_im_tmp = t121 * t93_im_tmp_tmp;
  t93_re = t93_re_tmp * t128.re - i_t93_im_tmp * t128.im;
  t93_im = t93_re_tmp * t128.im + i_t93_im_tmp * t128.re;
  t49_im_tmp = d_t93_re_tmp * t183.re - b_t93_im_tmp * t183.im;
  d_t54_im_tmp = d_t93_re_tmp * t183.im + b_t93_im_tmp * t183.re;
  t54_re_tmp *= t119;
  f_t54_re_tmp = t54_re_tmp * t128.re;
  c_t54_im_tmp = t54_re_tmp * t128.im;
  c_t42_re_tmp = d_t42_re_tmp * t119;
  d_t42_re_tmp = c_t42_re_tmp * t128.re;
  s_t42_im_tmp = c_t42_re_tmp * t128.im;
  c_t42_re_tmp = i_t42_re_tmp * t129.re;
  t_t42_im_tmp = i_t42_re_tmp * t129.im;
  t48_re_tmp *= t119;
  g_t48_re_tmp = t48_re_tmp * t128.re;
  c_t48_im_tmp = t48_re_tmp * t128.im;
  i_t42_re_tmp = t42_re_tmp * t113_re;
  u_t42_im_tmp = t42_re_tmp * t113_im;
  t42_re_tmp = t120 * (i_t42_re_tmp * t116_re - u_t42_im_tmp * t116_im);
  u_t42_im_tmp = t120 * (i_t42_re_tmp * t116_im + u_t42_im_tmp * t116_re);
  i_t42_re_tmp = t42_im_tmp * t113_re;
  v_t42_im_tmp = t42_im_tmp * t113_im;
  cb_t42_re_tmp = t120 * (i_t42_re_tmp * t117_re - v_t42_im_tmp * t117_im);
  v_t42_im_tmp = t120 * (i_t42_re_tmp * t117_im + v_t42_im_tmp * t117_re);
  t42_re = 3.1415926535897931 * (cb_t42_re_tmp * t187_re - v_t42_im_tmp *
    t187_im);
  t42_im = 3.1415926535897931 * (cb_t42_re_tmp * t187_im + v_t42_im_tmp *
    t187_re);
  i_t42_re_tmp = t42_im_tmp * t114_re;
  t42_im_tmp *= t114_im;
  db_t42_re_tmp = t120 * (i_t42_re_tmp * t116_re - t42_im_tmp * t116_im);
  t42_im_tmp = t120 * (i_t42_re_tmp * t116_im + t42_im_tmp * t116_re);
  b_t42_re = 3.1415926535897931 * (db_t42_re_tmp * t187_re - t42_im_tmp *
    t187_im);
  b_t42_im = 3.1415926535897931 * (db_t42_re_tmp * t187_im + t42_im_tmp *
    t187_re);
  b_t93_re = ((((((((((((((((((g_t93_re_tmp * i_t48_re - f_t93_im_tmp * t48_im) *
    0.5714285714285714 - 3.1415926535897931 * (h_t93_re_tmp * t188.re -
    g_t93_im_tmp * t188.im) * 0.42857142857142855) - b_t54_re_tmp * t118 *
    3.1415926535897931 * 0.5714285714285714) + d_t43_re_tmp_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857) + t55_re_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857) + c_t43_re_tmp_tmp * t118 *
    3.1415926535897931 / 7.0) + w_t42_re_tmp * t118 * 3.1415926535897931 *
    0.5714285714285714) - e_t54_re_tmp * t118 * 3.1415926535897931 *
                        0.2857142857142857) - x_t42_re_tmp * t118 *
                       3.1415926535897931 * 0.42857142857142855) + y_t42_re_tmp *
                      t118 * 3.1415926535897931 * 0.2857142857142857) -
                     3.1415926535897931 * (i_t93_re_tmp * t188.re - h_t93_im_tmp
    * t188.im) * 0.2857142857142857) + 3.1415926535897931 * (e_t93_re_tmp *
    t241_re - d_t93_im_tmp * t241_im) * 0.8571428571428571) + b_t48_re_tmp *
                   t118 * 3.1415926535897931 * 0.5714285714285714) -
                  t56_re_tmp_tmp * t118 * 3.1415926535897931 *
                  0.2857142857142857) - t50_re_tmp_tmp * t118 *
                 3.1415926535897931 * 0.2857142857142857) - e_t48_re_tmp * t118 *
                3.1415926535897931 / 7.0) + f_t48_re_tmp * t118 *
               3.1415926535897931 * 0.2857142857142857) - 3.1415926535897931 *
              (bb_t42_re_tmp * t183.re - r_t42_im_tmp * t183.im) *
              1.7142857142857142) + (((((((((((3.1415926535897931 * (t93_re *
    t188.re - t93_im * t188.im) * 0.8571428571428571 - 3.1415926535897931 *
    (t49_im_tmp * t241_re - d_t54_im_tmp * t241_im) * 1.7142857142857142) +
    ab_t42_re_tmp * t118 * 3.1415926535897931 * 0.42857142857142855) -
    t49_re_tmp * t118 * 3.1415926535897931 * 0.42857142857142855) -
    3.1415926535897931 * (f_t54_re_tmp * t183.re - c_t54_im_tmp * t183.im) *
    0.42857142857142855) + 3.1415926535897931 * (d_t42_re_tmp * t183.re -
    s_t42_im_tmp * t183.im) * 0.42857142857142855) - 3.1415926535897931 *
    (c_t42_re_tmp * t187_re - t_t42_im_tmp * t187_im) * 0.42857142857142855) +
    3.1415926535897931 * (j_t42_re_tmp * t241_re - e_t42_im_tmp * t241_im) *
    0.8571428571428571) + 3.1415926535897931 * (g_t48_re_tmp * t183.re -
    c_t48_im_tmp * t183.im) * 0.42857142857142855) + 3.1415926535897931 *
    (t42_re_tmp * t187_re - u_t42_im_tmp * t187_im) * 0.8571428571428571) -
    (t42_re * 0.0 - t42_im * 0.2857142857142857)) - (b_t42_re * 0.0 - b_t42_im *
    0.2857142857142857));
  t93_im = (((((g_t93_re_tmp * t48_im + f_t93_im_tmp * i_t48_re) *
               0.5714285714285714 - 3.1415926535897931 * (h_t93_re_tmp * t188.im
    + g_t93_im_tmp * t188.re) * 0.42857142857142855) - 3.1415926535897931 *
              (i_t93_re_tmp * t188.im + h_t93_im_tmp * t188.re) *
              0.2857142857142857) + 3.1415926535897931 * (e_t93_re_tmp * t241_im
              + d_t93_im_tmp * t241_re) * 0.8571428571428571) -
            3.1415926535897931 * (bb_t42_re_tmp * t183.im + r_t42_im_tmp *
             t183.re) * 1.7142857142857142) + (((((((((3.1415926535897931 *
    (t93_re * t188.im + t93_im * t188.re) * 0.8571428571428571 -
    3.1415926535897931 * (t49_im_tmp * t241_im + d_t54_im_tmp * t241_re) *
    1.7142857142857142) - 3.1415926535897931 * (f_t54_re_tmp * t183.im +
    c_t54_im_tmp * t183.re) * 0.42857142857142855) + 3.1415926535897931 *
    (d_t42_re_tmp * t183.im + s_t42_im_tmp * t183.re) * 0.42857142857142855) -
    3.1415926535897931 * (c_t42_re_tmp * t187_im + t_t42_im_tmp * t187_re) *
    0.42857142857142855) + 3.1415926535897931 * (j_t42_re_tmp * t241_im +
    e_t42_im_tmp * t241_re) * 0.8571428571428571) + 3.1415926535897931 *
    (g_t48_re_tmp * t183.im + c_t48_im_tmp * t183.re) * 0.42857142857142855) +
    3.1415926535897931 * (t42_re_tmp * t187_im + u_t42_im_tmp * t187_re) *
    0.8571428571428571) - (t42_re * 0.2857142857142857 + t42_im * 0.0)) -
    (b_t42_re * 0.2857142857142857 + b_t42_im * 0.0));
  t98 = t223_re * t223_re - t223_im * t223_im;
  t81 = t223_re * t223_im;
  t81 += t81;
  c_re = re_tmp * t223_re;
  c_im = re_tmp * t223_im;
  d_re = b_re_tmp * t183.re - im_tmp * t183.im;
  d_im = b_re_tmp * t183.im + im_tmp * t183.re;
  o_re_tmp = 0.31830988618379069 * t42 * t66 * t82 * t94_tmp * t111 * t115;
  t111 = o_re_tmp * d1;
  t115 = o_re_tmp * 0.0;
  t58_im = (((((d * t254_re - 0.0 * t254_im) - (d1 * t98 - 0.0 * t81)) -
              (c_re_tmp * t254_re - b_im_tmp * t254_im) * 0.875) - (c_re *
              t249_re - c_im * t249_im) * 0.875) + (d_re * t223_re - d_im *
             t223_im) * 0.4375) + (t111 * t223_re - t115 * t223_im) * 0.4375;
  e_t48_re_tmp = (((((d * t254_im + 0.0 * t254_re) - (d1 * t81 + 0.0 * t98)) -
                    (c_re_tmp * t254_im + b_im_tmp * t254_re) * 0.875) - (c_re *
    t249_im + c_im * t249_re) * 0.875) + (d_re * t223_im + d_im * t223_re) *
                  0.4375) + (t111 * t223_im + t115 * t223_re) * 0.4375;
  t54_re_tmp = t54_re_tmp_tmp * t105;
  t54_re = t54_re_tmp * t123_re;
  t56_re = t54_re_tmp * t123_im;
  t54_re_tmp = c_t44_re_tmp_tmp * t105;
  b_t54_re = t54_re_tmp * t123_re;
  t54_im = t54_re_tmp * t123_im;
  t55_re_tmp = t55_re_tmp_tmp * t105;
  t55_re = t55_re_tmp * t123_re;
  t55_im = t55_re_tmp * t123_im;
  t55_re_tmp = b_t55_re_tmp_tmp * t105;
  b_t55_re = t55_re_tmp * t123_re;
  t50_im = t55_re_tmp * t123_im;
  i_t42_re_tmp = c_t42_re_tmp_tmp * t105;
  t42_re = i_t42_re_tmp * t123_re;
  t42_im = i_t42_re_tmp * t123_im;
  t54_re_tmp = b_t54_re_tmp_tmp * t105;
  c_t54_re = t54_re_tmp * t123_re;
  b_t54_im = t54_re_tmp * t123_im;
  i_t42_re_tmp = d_t42_re_tmp_tmp * t105;
  b_t42_re = i_t42_re_tmp * t123_re;
  b_t42_im = i_t42_re_tmp * t123_im;
  i_t42_re_tmp = e_t42_re_tmp_tmp * t105;
  c_t42_re = i_t42_re_tmp * t123_re;
  c_t42_im = i_t42_re_tmp * t123_im;
  i_t42_re_tmp = l_t42_re_tmp * d1 - f_t42_im_tmp * 0.0;
  f_t42_im_tmp = l_t42_re_tmp * 0.0 + f_t42_im_tmp * d1;
  t54_re_tmp = t54_re_tmp_tmp * t99;
  d_t54_re = t54_re_tmp * t123_re;
  c_t54_im = t54_re_tmp * t123_im;
  e_t54_re = 3.1415926535897931 * (d_t54_re * d - c_t54_im * 0.0);
  c_t54_im = 3.1415926535897931 * (d_t54_re * 0.0 + c_t54_im * d);
  t54_re_tmp = c_t44_re_tmp_tmp * t99;
  d_t54_re = t54_re_tmp * t123_re;
  d_t54_im = t54_re_tmp * t123_im;
  f_t54_re = 3.1415926535897931 * (d_t54_re * d - d_t54_im * 0.0);
  d_t54_im = 3.1415926535897931 * (d_t54_re * 0.0 + d_t54_im * d);
  t55_re_tmp = t55_re_tmp_tmp * t99;
  c_t55_re = t55_re_tmp * t123_re;
  b_t55_im = t55_re_tmp * t123_im;
  d_t55_re = 3.1415926535897931 * (c_t55_re * d - b_t55_im * 0.0);
  b_t55_im = 3.1415926535897931 * (c_t55_re * 0.0 + b_t55_im * d);
  t55_re_tmp = b_t55_re_tmp_tmp * t99;
  c_t55_re = t55_re_tmp * t123_re;
  y_t42_re_tmp = t55_re_tmp * t123_im;
  e_t55_re = 3.1415926535897931 * (c_t55_re * d - y_t42_re_tmp * 0.0);
  y_t42_re_tmp = 3.1415926535897931 * (c_t55_re * 0.0 + y_t42_re_tmp * d);
  l_t42_re_tmp = c_t42_re_tmp_tmp * t99;
  d_t42_re = l_t42_re_tmp * t123_re;
  d_t42_im = l_t42_re_tmp * t123_im;
  e_t42_re = 3.1415926535897931 * (d_t42_re * d - d_t42_im * 0.0);
  d_t42_im = 3.1415926535897931 * (d_t42_re * 0.0 + d_t42_im * d);
  t54_re_tmp = b_t54_re_tmp_tmp * t99;
  d_t54_re = t54_re_tmp * t123_re;
  e_t54_im = t54_re_tmp * t123_im;
  c_t42_re_tmp_tmp = 3.1415926535897931 * (d_t54_re * d - e_t54_im * 0.0);
  e_t54_im = 3.1415926535897931 * (d_t54_re * 0.0 + e_t54_im * d);
  l_t42_re_tmp = d_t42_re_tmp_tmp * t99;
  d_t42_re = l_t42_re_tmp * t123_re;
  t73_im = l_t42_re_tmp * t123_im;
  f_t42_re = 3.1415926535897931 * (d_t42_re * d - t73_im * 0.0);
  t73_im = 3.1415926535897931 * (d_t42_re * 0.0 + t73_im * d);
  l_t42_re_tmp = e_t42_re_tmp_tmp * t99;
  d_t42_re = l_t42_re_tmp * t123_re;
  e_t42_im = l_t42_re_tmp * t123_im;
  g_t42_re = 3.1415926535897931 * (d_t42_re * d - e_t42_im * 0.0);
  e_t42_im = 3.1415926535897931 * (d_t42_re * 0.0 + e_t42_im * d);
  l_t42_re_tmp = n_t42_re_tmp * d1 - h_t42_im_tmp * 0.0;
  h_t42_im_tmp = n_t42_re_tmp * 0.0 + h_t42_im_tmp * d1;
  d_t42_re = 3.1415926535897931 * (l_t42_re_tmp * t223_re - h_t42_im_tmp *
    t223_im);
  t93_re_tmp_tmp = 3.1415926535897931 * (l_t42_re_tmp * t223_im + h_t42_im_tmp *
    t223_re);
  t48_re_tmp = t48_re_tmp_tmp * t106;
  t48_re = t48_re_tmp * t123_re;
  t48_im = t48_re_tmp * t123_im;
  t48_re_tmp = b_t48_re_tmp_tmp * t106;
  b_t48_re = t48_re_tmp * t123_re;
  t50_re_tmp_tmp = t48_re_tmp * t123_im;
  t48_re_tmp = t52_re_tmp_tmp * t106;
  c_t48_re = t48_re_tmp * t123_re;
  b_t48_im = t48_re_tmp * t123_im;
  t48_re_tmp = c_t48_re_tmp_tmp * t106;
  b_t56_re_tmp = t48_re_tmp * t123_re;
  c_t48_im = t48_re_tmp * t123_im;
  t48_re_tmp = d_t48_re_tmp_tmp * t106;
  b_t50_re_tmp = t48_re_tmp * t123_re;
  d_t48_im = t48_re_tmp * t123_im;
  t54_re_tmp = c_t54_re_tmp * d1 - t54_im_tmp * 0.0;
  t54_im_tmp = c_t54_re_tmp * 0.0 + t54_im_tmp * d1;
  n_t42_re_tmp = m_t42_re_tmp * d1 - g_t42_im_tmp * 0.0;
  g_t42_im_tmp = m_t42_re_tmp * 0.0 + g_t42_im_tmp * d1;
  m_t42_re_tmp = f_t42_re_tmp * t223_re - b_t42_im_tmp * t223_im;
  b_t44_re_tmp_tmp = f_t42_re_tmp * t223_im + b_t42_im_tmp * t223_re;
  t48_re_tmp = t48_re_tmp_tmp * t100;
  d_t48_re = t48_re_tmp * t123_re;
  e_t48_im = t48_re_tmp * t123_im;
  e_t48_re = 3.1415926535897931 * (d_t48_re * d - e_t48_im * 0.0);
  e_t48_im = 3.1415926535897931 * (d_t48_re * 0.0 + e_t48_im * d);
  t48_re_tmp = b_t48_re_tmp_tmp * t100;
  d_t48_re = t48_re_tmp * t123_re;
  d_t42_re_tmp_tmp = t48_re_tmp * t123_im;
  f_t48_re = 3.1415926535897931 * (d_t48_re * d - d_t42_re_tmp_tmp * 0.0);
  d_t42_re_tmp_tmp = 3.1415926535897931 * (d_t48_re * 0.0 + d_t42_re_tmp_tmp * d);
  t48_re_tmp = t52_re_tmp_tmp * t100;
  d_t48_re = t48_re_tmp * t123_re;
  t93_re = t48_re_tmp * t123_im;
  g_t48_re = 3.1415926535897931 * (d_t48_re * d - t93_re * 0.0);
  t93_re = 3.1415926535897931 * (d_t48_re * 0.0 + t93_re * d);
  t48_re_tmp = c_t48_re_tmp_tmp * t100;
  d_t48_re = t48_re_tmp * t123_re;
  t48_re_tmp_tmp = t48_re_tmp * t123_im;
  h_t48_re = 3.1415926535897931 * (d_t48_re * d - t48_re_tmp_tmp * 0.0);
  t48_re_tmp_tmp = 3.1415926535897931 * (d_t48_re * 0.0 + t48_re_tmp_tmp * d);
  t48_re_tmp = d_t48_re_tmp_tmp * t100;
  d_t48_re = t48_re_tmp * t123_re;
  c_t44_re_tmp_tmp = t48_re_tmp * t123_im;
  i_t48_re = 3.1415926535897931 * (d_t48_re * d - c_t44_re_tmp_tmp * 0.0);
  c_t44_re_tmp_tmp = 3.1415926535897931 * (d_t48_re * 0.0 + c_t44_re_tmp_tmp * d);
  b_t54_re_tmp = d_t54_re_tmp * d1 - b_t54_im_tmp * 0.0;
  b_t54_im_tmp = d_t54_re_tmp * 0.0 + b_t54_im_tmp * d1;
  d_t54_re = 3.1415926535897931 * (b_t54_re_tmp * t223_re - b_t54_im_tmp *
    t223_im);
  t58_re_tmp_tmp = 3.1415926535897931 * (b_t54_re_tmp * t223_im + b_t54_im_tmp *
    t223_re);
  w_t42_re_tmp = o_t42_re_tmp * d1 - i_t42_im_tmp * 0.0;
  i_t42_im_tmp = o_t42_re_tmp * 0.0 + i_t42_im_tmp * d1;
  h_t42_re = 3.1415926535897931 * (w_t42_re_tmp * t223_re - i_t42_im_tmp *
    t223_im);
  b_t48_re_tmp_tmp = 3.1415926535897931 * (w_t42_re_tmp * t223_im + i_t42_im_tmp
    * t223_re);
  i_t42_re = 3.1415926535897931 * (p_t42_re_tmp * t254_re - k_t42_im_tmp *
    t254_im);
  b_t54_re_tmp_tmp = 3.1415926535897931 * (p_t42_re_tmp * t254_im + k_t42_im_tmp
    * t254_re);
  o_t42_re_tmp = g_t42_re_tmp * t223_re - c_t42_im_tmp * t223_im;
  b_t44_re_tmp_tmp_tmp = g_t42_re_tmp * t223_im + c_t42_im_tmp * t223_re;
  t55_re_tmp_tmp = 3.1415926535897931 * (o_t42_re_tmp * t249_re -
    b_t44_re_tmp_tmp_tmp * t249_im);
  b_t55_re_tmp_tmp = 3.1415926535897931 * (o_t42_re_tmp * t249_im +
    b_t44_re_tmp_tmp_tmp * t249_re);
  o_re_tmp = re_tmp_tmp * t66 * t105;
  c_re = o_re_tmp * t123_re;
  c_im = o_re_tmp * t123_im;
  o_re_tmp = re_tmp_tmp_tmp * t61 * t69 * t105;
  d_re = o_re_tmp * t123_re;
  d_im = o_re_tmp * t123_im;
  t81 = 9.869604401089358 * t48 * t55;
  o_re_tmp = t81 * t54_tmp * t69 * t105;
  e_re = o_re_tmp * t123_re;
  e_im = o_re_tmp * t123_im;
  o_re_tmp = t81 * t61 * t70 * t105;
  f_re = o_re_tmp * t123_re;
  f_im = o_re_tmp * t123_im;
  o_re_tmp = re_tmp_tmp * t78 * t105;
  g_re = o_re_tmp * t123_re;
  g_im = o_re_tmp * t123_im;
  x_t42_re_tmp = g_t42_re_tmp_tmp * t106;
  t55_re_tmp = x_t42_re_tmp * t123_re;
  ai = x_t42_re_tmp * t123_im;
  t49_re_tmp = t49_re_tmp_tmp * t107;
  t49_re = t49_re_tmp * t123_re;
  t103 = t49_re_tmp * t123_im;
  t48_re_tmp = c_t48_re_tmp * d1 - t48_im_tmp * 0.0;
  t48_im_tmp = c_t48_re_tmp * 0.0 + t48_im_tmp * d1;
  x_t42_re_tmp = g_t42_re_tmp_tmp * t100;
  d_t43_re_tmp_tmp = x_t42_re_tmp * t123_re;
  c_t43_re_tmp_tmp = x_t42_re_tmp * t123_im;
  c_t55_re = 3.1415926535897931 * (d_t43_re_tmp_tmp * d - c_t43_re_tmp_tmp * 0.0);
  c_t43_re_tmp_tmp = 3.1415926535897931 * (d_t43_re_tmp_tmp * 0.0 +
    c_t43_re_tmp_tmp * d);
  t49_re_tmp = t49_re_tmp_tmp * t101;
  o_re_tmp = t49_re_tmp * t123_re;
  t98 = t49_re_tmp * t123_im;
  t41_tmp = 3.1415926535897931 * (o_re_tmp * d - t98 * 0.0);
  t98 = 3.1415926535897931 * (o_re_tmp * 0.0 + t98 * d);
  b_t48_re_tmp = d_t48_re_tmp * d1 - b_t48_im_tmp * 0.0;
  b_t48_im_tmp = d_t48_re_tmp * 0.0 + b_t48_im_tmp * d1;
  d_t48_re = 3.1415926535897931 * (b_t48_re_tmp * t223_re - b_t48_im_tmp *
    t223_im);
  t81 = 3.1415926535897931 * (b_t48_re_tmp * t223_im + b_t48_im_tmp * t223_re);
  o_re_tmp = 9.869604401089358 * t42 * t54_tmp * t54_tmp * t70 * t82 * t105;
  h_re = o_re_tmp * t123_re;
  h_im = o_re_tmp * t123_im;
  o_re_tmp = 31.006276680299816 * t49 * t54_tmp * t54_tmp * t70 * t82 * t100;
  i_re = o_re_tmp * t123_re;
  i_im = o_re_tmp * t123_im;
  j_re = i_re * d - i_im * 0.0;
  i_im = i_re * 0.0 + i_im * d;
  o_re_tmp = 9.869604401089358 * t49 * t54_tmp * t54_tmp * t70 * t82 * t106;
  i_re = o_re_tmp * t123_re;
  j_im = o_re_tmp * t123_im;
  o_re_tmp = e_re_tmp * d1 - c_im_tmp * 0.0;
  c_im_tmp = e_re_tmp * 0.0 + c_im_tmp * d1;
  Mxxx->re = ((((((((((((((((f_t104_re_tmp * t254_re - f_t104_im_tmp * t254_im) *
    -2.133935292046707 + (b_t104_re_tmp * b_t93_re - b_t104_im_tmp * t93_im) *
    1.0669676460233539) - (c_t104_re_tmp * t58_im - c_t104_im_tmp * e_t48_re_tmp)
    * 1.0669676460233539) + ((((t54_re * d - t56_re * 0.0) *
    -0.53348382301167685 + (b_t54_re * d - t54_im * 0.0) * 0.26674191150583842)
    + (t55_re * d - t55_im * 0.0) * 0.26674191150583842) + (b_t55_re * d -
    t50_im * 0.0) * 0.13337095575291921)) + ((((t42_re * d - t42_im * 0.0) *
    0.53348382301167685 - (c_t54_re * d - b_t54_im * 0.0) * 0.26674191150583842)
    - (b_t42_re * d - b_t42_im * 0.0) * 0.40011286725875761) + (c_t42_re * d -
    c_t42_im * 0.0) * 0.26674191150583842)) + ((((i_t42_re_tmp * t223_re -
    f_t42_im_tmp * t223_im) * 3.2009029380700609 + (e_t54_re * 0.0 - c_t54_im *
    124.4277138219654)) - (f_t54_re * 0.0 - d_t54_im * 62.21385691098272)) -
    (d_t55_re * 0.0 - b_t55_im * 62.21385691098272))) + ((((e_t55_re * -0.0 -
    y_t42_re_tmp * -31.10692845549136) - (e_t42_re * 0.0 - d_t42_im *
    124.4277138219654)) + (c_t42_re_tmp_tmp * 0.0 - e_t54_im * 62.21385691098272))
    + (f_t42_re * 0.0 - t73_im * 93.320785366474084))) + ((((g_t42_re * -0.0 -
    e_t42_im * -62.21385691098272) - (d_t42_re * 0.0 - t93_re_tmp_tmp *
    746.56628293179267)) + (t48_re * d - t48_im * 0.0) * 1.6004514690350311) -
    (b_t48_re * d - t50_re_tmp_tmp * 0.0) * 0.80022573451751533)) + ((((c_t48_re
    * d - b_t48_im * 0.0) * -0.80022573451751533 - (b_t56_re_tmp * d - c_t48_im *
    0.0) * 0.40011286725875761) + (b_t50_re_tmp * d - d_t48_im * 0.0) *
    0.80022573451751533) + (t54_re_tmp * t223_re - t54_im_tmp * t223_im) *
    0.80022573451751533)) + ((((n_t42_re_tmp * t223_re - g_t42_im_tmp * t223_im)
    * -0.80022573451751533 + (k_t42_re_tmp * t254_re - j_t42_im_tmp * t254_im) *
    1.6004514690350311) + (m_t42_re_tmp * t249_re - b_t44_re_tmp_tmp * t249_im) *
    1.6004514690350311) - (e_t48_re * 0.0 - e_t48_im * 373.28314146589628))) +
                   ((((f_t48_re * 0.0 - d_t42_re_tmp_tmp * 186.6415707329482) +
                      (g_t48_re * 0.0 - t93_re * 186.6415707329482)) + (h_t48_re
    * 0.0 - t48_re_tmp_tmp * 93.320785366474084)) - (i_t48_re * 0.0 -
    c_t44_re_tmp_tmp * 186.6415707329482))) + ((((d_t54_re * -0.0 -
    t58_re_tmp_tmp * -186.6415707329482) + (h_t42_re * 0.0 - b_t48_re_tmp_tmp *
    186.6415707329482)) - (i_t42_re * 0.0 - b_t54_re_tmp_tmp *
    373.28314146589628)) - (t55_re_tmp_tmp * 0.0 - b_t55_re_tmp_tmp *
    373.28314146589628))) + ((((c_re * d - c_im * 0.0) * -29021.041124656658 +
    (d_re * d - d_im * 0.0) * 14510.520562328329) + (e_re * d - e_im * 0.0) *
    14510.520562328329) + (f_re * d - f_im * 0.0) * 7255.2602811641664)) +
                ((((g_re * d - g_im * 0.0) * -14510.520562328329 + (t55_re_tmp *
    d - ai * 0.0) * 1.2003386017762729) - (t49_re * d - t103 * 0.0) *
                  2.0005643362937882) - (t48_re_tmp * t223_re - t48_im_tmp *
    t223_im) * 2.4006772035525459)) + (((c_t55_re * -0.0 - c_t43_re_tmp_tmp *
    -279.96235609942232) + (t41_tmp * 0.0 - t98 * 466.60392683237041)) +
    (d_t48_re * 0.0 - t81 * 559.92471219884453))) + (((h_re * d - h_im * 0.0) *
    -21765.7808434925 - (j_re * 0.0 - i_im * 1.6921889868604471E+6)) + (i_re * d
    - j_im * 0.0) * 43531.561686985)) + (o_re_tmp * t223_re - c_im_tmp * t223_im)
    * 43531.561686985;
  Mxxx->im = ((((((((((((((((f_t104_re_tmp * t254_im + f_t104_im_tmp * t254_re) *
    -2.133935292046707 + (b_t104_re_tmp * t93_im + b_t104_im_tmp * b_t93_re) *
    1.0669676460233539) - (c_t104_re_tmp * e_t48_re_tmp + c_t104_im_tmp * t58_im)
    * 1.0669676460233539) + ((((t54_re * 0.0 + t56_re * d) *
    -0.53348382301167685 + (b_t54_re * 0.0 + t54_im * d) * 0.26674191150583842)
    + (t55_re * 0.0 + t55_im * d) * 0.26674191150583842) + (b_t55_re * 0.0 +
    t50_im * d) * 0.13337095575291921)) + ((((t42_re * 0.0 + t42_im * d) *
    0.53348382301167685 - (c_t54_re * 0.0 + b_t54_im * d) * 0.26674191150583842)
    - (b_t42_re * 0.0 + b_t42_im * d) * 0.40011286725875761) + (c_t42_re * 0.0 +
    c_t42_im * d) * 0.26674191150583842)) + ((((i_t42_re_tmp * t223_im +
    f_t42_im_tmp * t223_re) * 3.2009029380700609 + (e_t54_re * 124.4277138219654
    + c_t54_im * 0.0)) - (f_t54_re * 62.21385691098272 + d_t54_im * 0.0)) -
    (d_t55_re * 62.21385691098272 + b_t55_im * 0.0))) + ((((e_t55_re *
    -31.10692845549136 + y_t42_re_tmp * -0.0) - (e_t42_re * 124.4277138219654 +
    d_t42_im * 0.0)) + (c_t42_re_tmp_tmp * 62.21385691098272 + e_t54_im * 0.0))
    + (f_t42_re * 93.320785366474084 + t73_im * 0.0))) + ((((g_t42_re *
    -62.21385691098272 + e_t42_im * -0.0) - (d_t42_re * 746.56628293179267 +
    t93_re_tmp_tmp * 0.0)) + (t48_re * 0.0 + t48_im * d) * 1.6004514690350311) -
    (b_t48_re * 0.0 + t50_re_tmp_tmp * d) * 0.80022573451751533)) + ((((c_t48_re
    * 0.0 + b_t48_im * d) * -0.80022573451751533 - (b_t56_re_tmp * 0.0 +
    c_t48_im * d) * 0.40011286725875761) + (b_t50_re_tmp * 0.0 + d_t48_im * d) *
    0.80022573451751533) + (t54_re_tmp * t223_im + t54_im_tmp * t223_re) *
    0.80022573451751533)) + ((((n_t42_re_tmp * t223_im + g_t42_im_tmp * t223_re)
    * -0.80022573451751533 + (k_t42_re_tmp * t254_im + j_t42_im_tmp * t254_re) *
    1.6004514690350311) + (m_t42_re_tmp * t249_im + b_t44_re_tmp_tmp * t249_re) *
    1.6004514690350311) - (e_t48_re * 373.28314146589628 + e_t48_im * 0.0))) +
                   ((((f_t48_re * 186.6415707329482 + d_t42_re_tmp_tmp * 0.0) +
                      (g_t48_re * 186.6415707329482 + t93_re * 0.0)) + (h_t48_re
    * 93.320785366474084 + t48_re_tmp_tmp * 0.0)) - (i_t48_re *
    186.6415707329482 + c_t44_re_tmp_tmp * 0.0))) + ((((d_t54_re *
    -186.6415707329482 + t58_re_tmp_tmp * -0.0) + (h_t42_re * 186.6415707329482
    + b_t48_re_tmp_tmp * 0.0)) - (i_t42_re * 373.28314146589628 +
    b_t54_re_tmp_tmp * 0.0)) - (t55_re_tmp_tmp * 373.28314146589628 +
    b_t55_re_tmp_tmp * 0.0))) + ((((c_re * 0.0 + c_im * d) * -29021.041124656658
    + (d_re * 0.0 + d_im * d) * 14510.520562328329) + (e_re * 0.0 + e_im * d) *
    14510.520562328329) + (f_re * 0.0 + f_im * d) * 7255.2602811641664)) +
                ((((g_re * 0.0 + g_im * d) * -14510.520562328329 + (t55_re_tmp *
    0.0 + ai * d) * 1.2003386017762729) - (t49_re * 0.0 + t103 * d) *
                  2.0005643362937882) - (t48_re_tmp * t223_im + t48_im_tmp *
    t223_re) * 2.4006772035525459)) + (((c_t55_re * -279.96235609942232 +
    c_t43_re_tmp_tmp * -0.0) + (t41_tmp * 466.60392683237041 + t98 * 0.0)) +
    (d_t48_re * 559.92471219884453 + t81 * 0.0))) + (((h_re * 0.0 + h_im * d) *
    -21765.7808434925 - (j_re * 1.6921889868604471E+6 + i_im * 0.0)) + (i_re *
    0.0 + j_im * d) * 43531.561686985)) + (o_re_tmp * t223_im + c_im_tmp *
    t223_re) * 43531.561686985;
  e_re_tmp = re_tmp * t224_re;
  e_t48_re_tmp = re_tmp * t224_im;
  g_t104_re_tmp = t104_re_tmp * t250_re - t104_im_tmp * t250_im;
  g_t104_im_tmp = t104_re_tmp * t250_im + t104_im_tmp * t250_re;
  c_t42_re_tmp_tmp = b_t42_re_tmp_tmp_tmp * t82 * t85;
  x_t42_re_tmp = c_t42_re_tmp_tmp * t95;
  t42_re = x_t42_re_tmp * t109_re;
  t56_re_tmp_tmp = x_t42_re_tmp * t93;
  b_t42_re = x_t42_re_tmp * t112_re;
  d_t42_re_tmp_tmp = t42 * t56_tmp * t56_tmp * t66 * t71 * t82;
  y_t42_re_tmp = d_t42_re_tmp_tmp * t95;
  c_t42_re = y_t42_re_tmp * t109_re;
  f_t48_re_tmp = y_t42_re_tmp * t93;
  d_t42_re = y_t42_re_tmp * t112_re;
  e_t42_re_tmp_tmp = b_t42_re_tmp_tmp_tmp * t71 * t82 * t86;
  ab_t42_re_tmp = e_t42_re_tmp_tmp * t95;
  e_t42_re = ab_t42_re_tmp * t109_re;
  t50_re_tmp_tmp = ab_t42_re_tmp * t93;
  f_t42_re = ab_t42_re_tmp * t112_re;
  g_t42_re_tmp_tmp = t42 * t50 * t56_tmp * t56_tmp * t66 * t71 * t82;
  t81 = g_t42_re_tmp_tmp * t96;
  g_t42_re = t81 * t109_re;
  t98 = t81 * t93;
  h_t42_re = t81 * t112_re;
  i_t42_re = ((((((((((c_t42_re_tmp_tmp * t91 + d_t42_re_tmp_tmp * t91 / 4.0) -
                      e_t42_re_tmp_tmp * t91 / 4.0) + (t42_re * 0.0 -
    t56_re_tmp_tmp * 0.5)) + (b_t42_re * 0.0 - t56_re_tmp_tmp * 0.5)) -
                   g_t42_re_tmp_tmp * t92 * 0.75) + (c_t42_re * 0.0 -
    f_t48_re_tmp * 0.125)) + (d_t42_re * 0.0 - f_t48_re_tmp * 0.125)) -
                (e_t42_re * 0.0 - t50_re_tmp_tmp * 0.125)) - (f_t42_re * 0.0 -
    t50_re_tmp_tmp * 0.125)) - (g_t42_re * 0.0 - t98 * 0.375)) - (h_t42_re * 0.0
    - t98 * 0.375);
  t42_im = (((((((t42_re * 0.5 + t56_re_tmp_tmp * 0.0) + (b_t42_re * 0.5 +
    t56_re_tmp_tmp * 0.0)) + (c_t42_re * 0.125 + f_t48_re_tmp * 0.0)) +
               (d_t42_re * 0.125 + f_t48_re_tmp * 0.0)) - (e_t42_re * 0.125 +
    t50_re_tmp_tmp * 0.0)) - (f_t42_re * 0.125 + t50_re_tmp_tmp * 0.0)) -
            (g_t42_re * 0.375 + t98 * 0.0)) - (h_t42_re * 0.375 + t98 * 0.0);
  t93_re = h_t93_re_tmp * t183.re - g_t93_im_tmp * t183.im;
  t93_im = h_t93_re_tmp * t183.im + g_t93_im_tmp * t183.re;
  j_t93_re_tmp = c_t93_re_tmp * t184.re - c_t93_im_tmp * t184.im;
  c_t93_im_tmp = c_t93_re_tmp * t184.im + c_t93_im_tmp * t184.re;
  b_t43_re_tmp *= t119;
  b_t48_im = b_t43_re_tmp * t128.re;
  c_t48_im = b_t43_re_tmp * t128.im;
  t49_im_tmp = i_t93_re_tmp * t183.re - h_t93_im_tmp * t183.im;
  d_t54_im_tmp = i_t93_re_tmp * t183.im + h_t93_im_tmp * t183.re;
  b_t93_re = d_t93_re_tmp * t184.re - b_t93_im_tmp * t184.im;
  t98 = d_t93_re_tmp * t184.im + b_t93_im_tmp * t184.re;
  t41_tmp = d_t93_re_tmp * t183.re - b_t93_im_tmp * t183.im;
  t103 = d_t93_re_tmp * t183.im + b_t93_im_tmp * t183.re;
  t56_re_tmp *= t119;
  b_t56_re_tmp = t56_re_tmp * t128.re;
  d_t48_im = t56_re_tmp * t128.im;
  b_t48_re = (b_t56_re_tmp * t183.re - d_t48_im * t183.im) * 3.1415926535897931;
  ai = (b_t56_re_tmp * t183.im + d_t48_im * t183.re) * 3.1415926535897931;
  if (ai == 0.0) {
    t56_re = b_t48_re / 7.0;
    t48_re_tmp_tmp = 0.0;
  } else if (b_t48_re == 0.0) {
    t56_re = 0.0;
    t48_re_tmp_tmp = ai / 7.0;
  } else {
    t56_re = b_t48_re / 7.0;
    t48_re_tmp_tmp = ai / 7.0;
  }

  b_t48_re = (c_t42_re_tmp * t189_re - t_t42_im_tmp * t189_im) *
    3.1415926535897931;
  ai = (c_t42_re_tmp * t189_im + t_t42_im_tmp * t189_re) * 3.1415926535897931;
  if (ai == 0.0) {
    t42_re = b_t48_re / 7.0;
    b_t42_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t42_re = 0.0;
    b_t42_im = ai / 7.0;
  } else {
    t42_re = b_t48_re / 7.0;
    b_t42_im = ai / 7.0;
  }

  b_t43_re_tmp = c_t43_re_tmp * t119;
  c_t43_re_tmp = b_t43_re_tmp * t128.re;
  t93_im_tmp_tmp = b_t43_re_tmp * t128.im;
  b_t48_re = (c_t43_re_tmp * t183.re - t93_im_tmp_tmp * t183.im) *
    3.1415926535897931;
  ai = (c_t43_re_tmp * t183.im + t93_im_tmp_tmp * t183.re) * 3.1415926535897931;
  if (ai == 0.0) {
    t43_re = b_t48_re / 7.0;
    t43_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t43_re = 0.0;
    t43_im = ai / 7.0;
  } else {
    t43_re = b_t48_re / 7.0;
    t43_im = ai / 7.0;
  }

  c_t93_re_tmp = t93_re_tmp * t128.re - i_t93_im_tmp * t128.im;
  t58_re_tmp_tmp = t93_re_tmp * t128.im + i_t93_im_tmp * t128.re;
  c_t44_re_tmp_tmp = c_t93_re_tmp * t183.re - t58_re_tmp_tmp * t183.im;
  t58_re_tmp_tmp = c_t93_re_tmp * t183.im + t58_re_tmp_tmp * t183.re;
  t50_re_tmp *= t119;
  b_t50_re_tmp = t50_re_tmp * t128.re;
  c_t48_re_tmp_tmp = t50_re_tmp * t128.im;
  b_t48_re = (b_t50_re_tmp * t183.re - c_t48_re_tmp_tmp * t183.im) *
    3.1415926535897931;
  ai = (b_t50_re_tmp * t183.im + c_t48_re_tmp_tmp * t183.re) *
    3.1415926535897931;
  if (ai == 0.0) {
    t48_re = b_t48_re / 7.0;
    t50_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t48_re = 0.0;
    t50_im = ai / 7.0;
  } else {
    t48_re = b_t48_re / 7.0;
    t50_im = ai / 7.0;
  }

  b_t43_re_tmp = h_t43_re_tmp * t129.re;
  b_t55_im = h_t43_re_tmp * t129.im;
  b_t43_re = b_t43_re_tmp * t183.re - b_t55_im * t183.im;
  b_t43_im = b_t43_re_tmp * t183.im + b_t55_im * t183.re;
  h_t43_re_tmp = t43_re_tmp * t113_re;
  d_t48_re_tmp_tmp = t43_re_tmp * t113_im;
  t43_re_tmp = t120 * (h_t43_re_tmp * t116_re - d_t48_re_tmp_tmp * t116_im);
  d_t48_re_tmp_tmp = t120 * (h_t43_re_tmp * t116_im + d_t48_re_tmp_tmp * t116_re);
  c_t43_re = t43_re_tmp * t183.re - d_t48_re_tmp_tmp * t183.im;
  c_t43_im = t43_re_tmp * t183.im + d_t48_re_tmp_tmp * t183.re;
  b_t42_re_tmp *= t119;
  t52_re_tmp_tmp = b_t42_re_tmp * t128.re;
  t56_re_tmp_tmp = b_t42_re_tmp * t128.im;
  h_t43_re_tmp = t43_im_tmp * t113_re;
  d_t54_im = t43_im_tmp * t113_im;
  c_t54_im = t120 * (h_t43_re_tmp * t117_re - d_t54_im * t117_im);
  d_t54_im = t120 * (h_t43_re_tmp * t117_im + d_t54_im * t117_re);
  d_t43_re = c_t54_im * t183.re - d_t54_im * t183.im;
  e_t55_re = c_t54_im * t183.im + d_t54_im * t183.re;
  e_t43_re = 3.1415926535897931 * (d_t43_re * t184.re - e_t55_re * t184.im);
  e_t55_re = 3.1415926535897931 * (d_t43_re * t184.im + e_t55_re * t184.re);
  h_t43_re_tmp = t43_im_tmp * t114_re;
  t43_im_tmp *= t114_im;
  t54_im = t120 * (h_t43_re_tmp * t116_re - t43_im_tmp * t116_im);
  t43_im_tmp = t120 * (h_t43_re_tmp * t116_im + t43_im_tmp * t116_re);
  d_t43_re = t54_im * t183.re - t43_im_tmp * t183.im;
  e_t54_im = t54_im * t183.im + t43_im_tmp * t183.re;
  b_t42_re_tmp_tmp_tmp = 3.1415926535897931 * (d_t43_re * t184.re - e_t54_im *
    t184.im);
  e_t54_im = 3.1415926535897931 * (d_t43_re * t184.im + e_t54_im * t184.re);
  c_t93_re_tmp = ((((g_t93_re_tmp * i_t42_re - f_t93_im_tmp * t42_im) *
                    0.5714285714285714 - 3.1415926535897931 * (t93_re * t189_re
    - t93_im * t189_im) * 0.42857142857142855) + 3.1415926535897931 *
                   (j_t93_re_tmp * t221_re - c_t93_im_tmp * t221_im) *
                   0.5714285714285714) + (((((((((((((((3.1415926535897931 *
    (e_t93_re_tmp * t242_re - d_t93_im_tmp * t242_im) * 0.2857142857142857 -
    3.1415926535897931 * (b_t48_im * t183.re - c_t48_im * t183.im) *
    0.5714285714285714) - 3.1415926535897931 * (t49_im_tmp * t189_re -
    d_t54_im_tmp * t189_im) * 0.2857142857142857) - 3.1415926535897931 *
    (b_t93_re * t221_re - t98 * t221_im) * 1.1428571428571428) -
    3.1415926535897931 * (t41_tmp * t242_re - t103 * t242_im) *
    0.5714285714285714) + x_t42_re_tmp * t118 * 3.1415926535897931 *
    0.5714285714285714) - t56_re) - t42_re) + t43_re) + 3.1415926535897931 *
    (i_t43_re_tmp * t221_re - e_t43_im_tmp * t221_im) * 0.5714285714285714) +
    3.1415926535897931 * (j_t42_re_tmp * t242_re - e_t42_im_tmp * t242_im) *
    0.2857142857142857) + 3.1415926535897931 * (c_t44_re_tmp_tmp * t189_re -
    t58_re_tmp_tmp * t189_im) * 0.8571428571428571) + y_t42_re_tmp * t118 *
    3.1415926535897931 / 7.0) - ab_t42_re_tmp * t118 * 3.1415926535897931 / 7.0)
    + t48_re) + 3.1415926535897931 * (t42_re_tmp * t189_re - u_t42_im_tmp *
    t189_im) * 0.2857142857142857)) + (((((3.1415926535897931 * (b_t43_re *
    t184.re - b_t43_im * t184.im) * -0.2857142857142857 - t81 * t118 *
    3.1415926535897931 * 0.42857142857142855) + 3.1415926535897931 * (c_t43_re *
    t184.re - c_t43_im * t184.im) * 0.5714285714285714) + 3.1415926535897931 *
    (t52_re_tmp_tmp * t184.re - t56_re_tmp_tmp * t184.im) * 0.2857142857142857)
    - (e_t43_re * 0.0 - e_t55_re * 0.2857142857142857)) - (b_t42_re_tmp_tmp_tmp *
    0.0 - e_t54_im * 0.2857142857142857));
  t93_im = ((((g_t93_re_tmp * t42_im + f_t93_im_tmp * i_t42_re) *
              0.5714285714285714 - 3.1415926535897931 * (t93_re * t189_im +
    t93_im * t189_re) * 0.42857142857142855) + 3.1415926535897931 *
             (j_t93_re_tmp * t221_im + c_t93_im_tmp * t221_re) *
             0.5714285714285714) + ((((((((((((3.1415926535897931 *
    (e_t93_re_tmp * t242_im + d_t93_im_tmp * t242_re) * 0.2857142857142857 -
    3.1415926535897931 * (b_t48_im * t183.im + c_t48_im * t183.re) *
    0.5714285714285714) - 3.1415926535897931 * (t49_im_tmp * t189_im +
    d_t54_im_tmp * t189_re) * 0.2857142857142857) - 3.1415926535897931 *
    (b_t93_re * t221_im + t98 * t221_re) * 1.1428571428571428) -
    3.1415926535897931 * (t41_tmp * t242_im + t103 * t242_re) *
    0.5714285714285714) - t48_re_tmp_tmp) - b_t42_im) + t43_im) +
    3.1415926535897931 * (i_t43_re_tmp * t221_im + e_t43_im_tmp * t221_re) *
    0.5714285714285714) + 3.1415926535897931 * (j_t42_re_tmp * t242_im +
    e_t42_im_tmp * t242_re) * 0.2857142857142857) + 3.1415926535897931 *
    (c_t44_re_tmp_tmp * t189_im + t58_re_tmp_tmp * t189_re) * 0.8571428571428571)
              + t50_im) + 3.1415926535897931 * (t42_re_tmp * t189_im +
              u_t42_im_tmp * t189_re) * 0.2857142857142857)) +
    ((((3.1415926535897931 * (b_t43_re * t184.im + b_t43_im * t184.re) *
        -0.2857142857142857 + 3.1415926535897931 * (c_t43_re * t184.im +
         c_t43_im * t184.re) * 0.5714285714285714) + 3.1415926535897931 *
       (t52_re_tmp_tmp * t184.im + t56_re_tmp_tmp * t184.re) *
       0.2857142857142857) - (e_t43_re * 0.2857142857142857 + e_t55_re * 0.0)) -
     (b_t42_re_tmp_tmp_tmp * 0.2857142857142857 + e_t54_im * 0.0));
  t229_re_tmp = (((((d1 * t50_re - 0.0 * b_t56_re) - 0.4375 * (re * t224_re - im
    * t224_im)) - 0.4375 * (b_re * t224_re - b_im * t224_im)) + 0.875 *
                  (e_re_tmp * t250_re - e_t48_re_tmp * t250_im)) - (d * t255_re
    - 0.0 * t255_im)) + 0.875 * (c_re_tmp * t255_re - b_im_tmp * t255_im);
  t229_im_tmp = (((((d1 * b_t56_re + 0.0 * t50_re) - 0.4375 * (re * t224_im + im
    * t224_re)) - 0.4375 * (b_re * t224_im + b_im * t224_re)) + 0.875 *
                  (e_re_tmp * t250_im + e_t48_re_tmp * t250_re)) - (d * t255_im
    + 0.0 * t255_re)) + 0.875 * (c_re_tmp * t255_im + b_im_tmp * t255_re);
  h_t43_re_tmp = m_t43_re_tmp * d1 - h_t43_im_tmp * 0.0;
  h_t43_im_tmp = m_t43_re_tmp * 0.0 + h_t43_im_tmp * d1;
  m_t43_re_tmp = o_t43_re_tmp * d1 - j_t43_im_tmp * 0.0;
  j_t43_im_tmp = o_t43_re_tmp * 0.0 + j_t43_im_tmp * d1;
  t43_re = 3.1415926535897931 * (m_t43_re_tmp * t223_re - j_t43_im_tmp * t223_im);
  t43_im = 3.1415926535897931 * (m_t43_re_tmp * t223_im + j_t43_im_tmp * t223_re);
  t56_re_tmp = c_t56_re_tmp * d1 - t56_im_tmp * 0.0;
  t56_im_tmp = c_t56_re_tmp * 0.0 + t56_im_tmp * d1;
  o_t43_re_tmp = n_t43_re_tmp * d1 - i_t43_im_tmp * 0.0;
  i_t43_im_tmp = n_t43_re_tmp * 0.0 + i_t43_im_tmp * d1;
  b_t42_re_tmp = f_t42_re_tmp * t224_re - b_t42_im_tmp * t224_im;
  f_t48_re_tmp = f_t42_re_tmp * t224_im + b_t42_im_tmp * t224_re;
  n_t43_re_tmp = e_t43_re_tmp * t223_re - b_t43_im_tmp * t223_im;
  e_t48_im = e_t43_re_tmp * t223_im + b_t43_im_tmp * t223_re;
  c_t56_re_tmp = d_t56_re_tmp * d1 - b_t56_im_tmp * 0.0;
  b_t56_im_tmp = d_t56_re_tmp * 0.0 + b_t56_im_tmp * d1;
  t56_re = 3.1415926535897931 * (c_t56_re_tmp * t223_re - b_t56_im_tmp * t223_im);
  t48_re_tmp_tmp = 3.1415926535897931 * (c_t56_re_tmp * t223_im + b_t56_im_tmp *
    t223_re);
  b_t54_im = p_t43_re_tmp * d1 - k_t43_im_tmp * 0.0;
  k_t43_im_tmp = p_t43_re_tmp * 0.0 + k_t43_im_tmp * d1;
  b_t43_re = 3.1415926535897931 * (b_t54_im * t223_re - k_t43_im_tmp * t223_im);
  b_t43_im = 3.1415926535897931 * (b_t54_im * t223_im + k_t43_im_tmp * t223_re);
  t42_re = 3.1415926535897931 * (p_t42_re_tmp * t255_re - k_t42_im_tmp * t255_im);
  t42_im = 3.1415926535897931 * (p_t42_re_tmp * t255_im + k_t42_im_tmp * t255_re);
  c_t43_re = 3.1415926535897931 * (k_t43_re_tmp * t256_re - g_t43_im_tmp *
    t256_im);
  c_t43_im = 3.1415926535897931 * (k_t43_re_tmp * t256_im + g_t43_im_tmp *
    t256_re);
  x_t42_re_tmp = g_t42_re_tmp * t224_re - c_t42_im_tmp * t224_im;
  t50_re_tmp_tmp = g_t42_re_tmp * t224_im + c_t42_im_tmp * t224_re;
  b_t42_re = 3.1415926535897931 * (x_t42_re_tmp * t250_re - t50_re_tmp_tmp *
    t250_im);
  b_t42_im = 3.1415926535897931 * (x_t42_re_tmp * t250_im + t50_re_tmp_tmp *
    t250_re);
  p_t43_re_tmp = f_t43_re_tmp * t223_re - c_t43_im_tmp * t223_im;
  e_t54_re_tmp = f_t43_re_tmp * t223_im + c_t43_im_tmp * t223_re;
  d_t43_re = 3.1415926535897931 * (p_t43_re_tmp * t250_re - e_t54_re_tmp *
    t250_im);
  e_t55_re = 3.1415926535897931 * (p_t43_re_tmp * t250_im + e_t54_re_tmp *
    t250_re);
  y_t42_re_tmp = c_t42_re_tmp_tmp * t106;
  c_t42_re = y_t42_re_tmp * t123_re;
  c_t42_im = y_t42_re_tmp * t123_im;
  t50_re_tmp = c_t50_re_tmp * d1 - t50_im_tmp * 0.0;
  t50_im_tmp = c_t50_re_tmp * 0.0 + t50_im_tmp * d1;
  y_t42_re_tmp = c_t42_re_tmp_tmp * t100;
  d_t42_re = y_t42_re_tmp * t123_re;
  d_t42_im = y_t42_re_tmp * t123_im;
  e_t42_re = 3.1415926535897931 * (d_t42_re * d - d_t42_im * 0.0);
  d_t42_im = 3.1415926535897931 * (d_t42_re * 0.0 + d_t42_im * d);
  c_t50_re_tmp = d_t50_re_tmp * d1 - b_t50_im_tmp * 0.0;
  b_t50_im_tmp = d_t50_re_tmp * 0.0 + b_t50_im_tmp * d1;
  t48_re = 3.1415926535897931 * (c_t50_re_tmp * t223_re - b_t50_im_tmp * t223_im);
  t50_im = 3.1415926535897931 * (c_t50_re_tmp * t223_im + b_t50_im_tmp * t223_re);
  re_tmp = b_re_tmp_tmp * t82 * t85 * t105;
  re = re_tmp * t123_re;
  im = re_tmp * t123_im;
  y_t42_re_tmp = d_t42_re_tmp_tmp * t106;
  d_t42_re = y_t42_re_tmp * t123_re;
  t73_im = y_t42_re_tmp * t123_im;
  y_t42_re_tmp = e_t42_re_tmp_tmp * t106;
  f_t42_re = y_t42_re_tmp * t123_re;
  e_t42_im = y_t42_re_tmp * t123_im;
  re_tmp = j_re_tmp * d1 - h_im_tmp * 0.0;
  h_im_tmp = j_re_tmp * 0.0 + h_im_tmp * d1;
  y_t42_re_tmp = d_t42_re_tmp_tmp * t100;
  g_t42_re = y_t42_re_tmp * t123_re;
  t93_re_tmp_tmp = y_t42_re_tmp * t123_im;
  h_t42_re = 3.1415926535897931 * (g_t42_re * d - t93_re_tmp_tmp * 0.0);
  t93_re_tmp_tmp = 3.1415926535897931 * (g_t42_re * 0.0 + t93_re_tmp_tmp * d);
  y_t42_re_tmp = e_t42_re_tmp_tmp * t100;
  g_t42_re = y_t42_re_tmp * t123_re;
  b_t48_re_tmp_tmp = y_t42_re_tmp * t123_im;
  i_t42_re = 3.1415926535897931 * (g_t42_re * d - b_t48_re_tmp_tmp * 0.0);
  b_t48_re_tmp_tmp = 3.1415926535897931 * (g_t42_re * 0.0 + b_t48_re_tmp_tmp * d);
  j_re_tmp = 9.869604401089358 * t42 * t56_tmp * t56_tmp * t66 * t71 * t82 *
    t105;
  b_re = j_re_tmp * t123_re;
  b_im = j_re_tmp * t123_im;
  j_re_tmp = b_re_tmp_tmp * t71 * t82 * t86 * t105;
  c_re = j_re_tmp * t123_re;
  c_im = j_re_tmp * t123_im;
  y_t42_re_tmp = g_t42_re_tmp_tmp * t107;
  g_t42_re = y_t42_re_tmp * t123_re;
  b_t54_re_tmp_tmp = y_t42_re_tmp * t123_im;
  y_t42_re_tmp = r_t42_re_tmp * d1 - l_t42_im_tmp * 0.0;
  l_t42_im_tmp = r_t42_re_tmp * 0.0 + l_t42_im_tmp * d1;
  r_t42_re_tmp = g_t42_re_tmp_tmp * t101;
  t55_re_tmp_tmp = r_t42_re_tmp * t123_re;
  b_t55_re_tmp_tmp = r_t42_re_tmp * t123_im;
  t55_re_tmp = 3.1415926535897931 * (t55_re_tmp_tmp * d - b_t55_re_tmp_tmp * 0.0);
  b_t55_re_tmp_tmp = 3.1415926535897931 * (t55_re_tmp_tmp * 0.0 +
    b_t55_re_tmp_tmp * d);
  r_t42_re_tmp = s_t42_re_tmp * d1 - m_t42_im_tmp * 0.0;
  m_t42_im_tmp = s_t42_re_tmp * 0.0 + m_t42_im_tmp * d1;
  t55_re_tmp_tmp = 3.1415926535897931 * (r_t42_re_tmp * t224_re - m_t42_im_tmp *
    t224_im);
  ai = 3.1415926535897931 * (r_t42_re_tmp * t224_im + m_t42_im_tmp * t224_re);
  j_re_tmp = 31.006276680299816 * t42 * t50 * t56_tmp * t56_tmp * t66 * t71 *
    t82 * t100;
  d_re = j_re_tmp * t123_re;
  d_im = j_re_tmp * t123_im;
  e_re = d_re * d - d_im * 0.0;
  d_im = d_re * 0.0 + d_im * d;
  j_re_tmp = 9.869604401089358 * t42 * t50 * t56_tmp * t56_tmp * t66 * t71 * t82
    * t106;
  d_re = j_re_tmp * t123_re;
  e_im = j_re_tmp * t123_im;
  j_re_tmp = f_re_tmp * d1 - d_im_tmp * 0.0;
  d_im_tmp = f_re_tmp * 0.0 + d_im_tmp * d1;
  Mxyy->re = ((((((((((((g_t104_re_tmp * t256_re - g_t104_im_tmp * t256_im) *
                        2.133935292046707 + (b_t104_re_tmp * c_t93_re_tmp -
    b_t104_im_tmp * t93_im) * 1.0669676460233539) + (c_t104_re_tmp * t229_re_tmp
    - c_t104_im_tmp * t229_im_tmp) * 1.0669676460233539) + (h_t43_re_tmp *
    t223_re - h_t43_im_tmp * t223_im) * 1.0669676460233539) + ((((t43_re * -0.0
    - t43_im * -248.85542764393091) + (t56_re_tmp * t223_re - t56_im_tmp *
    t223_im) * 0.26674191150583842) - (o_t43_re_tmp * t223_re - i_t43_im_tmp *
    t223_im) * 0.26674191150583842) + (k_t42_re_tmp * t255_re - j_t42_im_tmp *
    t255_im) * 0.53348382301167685)) + ((((j_t43_re_tmp * t256_re - f_t43_im_tmp
    * t256_im) * -1.0669676460233539 + (b_t42_re_tmp * t250_re - f_t48_re_tmp *
    t250_im) * 0.53348382301167685) + (n_t43_re_tmp * t250_re - e_t48_im *
    t250_im) * 1.0669676460233539) - (t56_re * 0.0 - t48_re_tmp_tmp *
    62.21385691098272))) + ((((b_t43_re * 0.0 - b_t43_im * 62.21385691098272) -
    (t42_re * 0.0 - t42_im * 124.4277138219654)) + (c_t43_re * 0.0 - c_t43_im *
    248.85542764393091)) - (b_t42_re * 0.0 - b_t42_im * 124.4277138219654))) +
                  ((((d_t43_re * -0.0 - e_t55_re * -248.85542764393091) +
                     (c_t42_re * d - c_t42_im * 0.0) * 1.6004514690350311) -
                    (t50_re_tmp * t223_re - t50_im_tmp * t223_im) *
                    0.80022573451751533) - (e_t42_re * 0.0 - d_t42_im *
    373.28314146589628))) + ((((t48_re * 0.0 - t50_im * 186.6415707329482) - (re
    * d - im * 0.0) * 29021.041124656658) + (d_t42_re * d - t73_im * 0.0) *
    0.40011286725875761) - (f_t42_re * d - e_t42_im * 0.0) * 0.40011286725875761))
                + (((re_tmp * t223_re - h_im_tmp * t223_im) * 14510.520562328329
                    - (h_t42_re * 0.0 - t93_re_tmp_tmp * 93.320785366474084)) +
                   (i_t42_re * 0.0 - b_t48_re_tmp_tmp * 93.320785366474084))) +
               ((((b_re * d - b_im * 0.0) * -7255.2602811641664 + (c_re * d -
    c_im * 0.0) * 7255.2602811641664) - (g_t42_re * d - b_t54_re_tmp_tmp * 0.0) *
                 2.0005643362937882) - (y_t42_re_tmp * t224_re - l_t42_im_tmp *
    t224_im) * 1.6004514690350311)) + (((t55_re_tmp * 0.0 - b_t55_re_tmp_tmp *
    466.60392683237041) + (t55_re_tmp_tmp * 0.0 - ai * 373.28314146589628)) -
    (e_re * 0.0 - d_im * 1.6921889868604471E+6))) + ((d_re * d - e_im * 0.0) *
    43531.561686985 + (j_re_tmp * t224_re - d_im_tmp * t224_im) *
    29021.041124656658);
  Mxyy->im = ((((((((((((g_t104_re_tmp * t256_im + g_t104_im_tmp * t256_re) *
                        2.133935292046707 + (b_t104_re_tmp * t93_im +
    b_t104_im_tmp * c_t93_re_tmp) * 1.0669676460233539) + (c_t104_re_tmp *
    t229_im_tmp + c_t104_im_tmp * t229_re_tmp) * 1.0669676460233539) +
                      (h_t43_re_tmp * t223_im + h_t43_im_tmp * t223_re) *
                      1.0669676460233539) + ((((t43_re * -248.85542764393091 +
    t43_im * -0.0) + (t56_re_tmp * t223_im + t56_im_tmp * t223_re) *
    0.26674191150583842) - (o_t43_re_tmp * t223_im + i_t43_im_tmp * t223_re) *
    0.26674191150583842) + (k_t42_re_tmp * t255_im + j_t42_im_tmp * t255_re) *
    0.53348382301167685)) + ((((j_t43_re_tmp * t256_im + f_t43_im_tmp * t256_re)
    * -1.0669676460233539 + (b_t42_re_tmp * t250_im + f_t48_re_tmp * t250_re) *
    0.53348382301167685) + (n_t43_re_tmp * t250_im + e_t48_im * t250_re) *
    1.0669676460233539) - (t56_re * 62.21385691098272 + t48_re_tmp_tmp * 0.0)))
                   + ((((b_t43_re * 62.21385691098272 + b_t43_im * 0.0) -
                        (t42_re * 124.4277138219654 + t42_im * 0.0)) + (c_t43_re
    * 248.85542764393091 + c_t43_im * 0.0)) - (b_t42_re * 124.4277138219654 +
    b_t42_im * 0.0))) + ((((d_t43_re * -248.85542764393091 + e_t55_re * -0.0) +
    (c_t42_re * 0.0 + c_t42_im * d) * 1.6004514690350311) - (t50_re_tmp *
    t223_im + t50_im_tmp * t223_re) * 0.80022573451751533) - (e_t42_re *
    373.28314146589628 + d_t42_im * 0.0))) + ((((t48_re * 186.6415707329482 +
    t50_im * 0.0) - (re * 0.0 + im * d) * 29021.041124656658) + (d_t42_re * 0.0
    + t73_im * d) * 0.40011286725875761) - (f_t42_re * 0.0 + e_t42_im * d) *
    0.40011286725875761)) + (((re_tmp * t223_im + h_im_tmp * t223_re) *
    14510.520562328329 - (h_t42_re * 93.320785366474084 + t93_re_tmp_tmp * 0.0))
    + (i_t42_re * 93.320785366474084 + b_t48_re_tmp_tmp * 0.0))) + ((((b_re *
    0.0 + b_im * d) * -7255.2602811641664 + (c_re * 0.0 + c_im * d) *
    7255.2602811641664) - (g_t42_re * 0.0 + b_t54_re_tmp_tmp * d) *
    2.0005643362937882) - (y_t42_re_tmp * t224_im + l_t42_im_tmp * t224_re) *
    1.6004514690350311)) + (((t55_re_tmp * 466.60392683237041 + b_t55_re_tmp_tmp
    * 0.0) + (t55_re_tmp_tmp * 373.28314146589628 + ai * 0.0)) - (e_re *
    1.6921889868604471E+6 + d_im * 0.0))) + ((d_re * 0.0 + e_im * d) *
    43531.561686985 + (j_re_tmp * t224_im + d_im_tmp * t224_re) *
    29021.041124656658);
  t73_re = t73 * t113_re;
  t73_im = t73 * t113_im;
  b_t56_re = t119 * (t73_re * t116_re - t73_im * t116_im);
  t73_im = t119 * (t73_re * t116_im + t73_im * t116_re);
  t73_re = b_t56_re * d1 - t73_im * 0.0;
  t73_im = b_t56_re * 0.0 + t73_im * d1;
  if (t73_im == 0.0) {
    t73_re /= 4.0;
    t73_im = 0.0;
  } else if (t73_re == 0.0) {
    t73_re = 0.0;
    t73_im /= 4.0;
  } else {
    t73_re /= 4.0;
    t73_im /= 4.0;
  }

  d_t73_re_tmp = t73 * t118;
  b_t56_re = d_t73_re_tmp * t129.re;
  t50_re = d_t73_re_tmp * t129.im;
  e_t42_im = b_t56_re * d1 - t50_re * 0.0;
  t50_re = b_t56_re * 0.0 + t50_re * d1;
  if (t50_re == 0.0) {
    b_t56_re = e_t42_im / 4.0;
    t50_re = 0.0;
  } else if (e_t42_im == 0.0) {
    b_t56_re = 0.0;
    t50_re /= 4.0;
  } else {
    b_t56_re = e_t42_im / 4.0;
    t50_re /= 4.0;
  }

  d_t73_re_tmp = t73_re_tmp_tmp_tmp * t113_re;
  d_t73_im_tmp = t73_re_tmp_tmp_tmp * t113_im;
  t41_tmp = d_t73_re_tmp * t116_re - d_t73_im_tmp * t116_im;
  e_t42_im = t120 * t41_tmp;
  d_t73_im_tmp = d_t73_re_tmp * t116_im + d_t73_im_tmp * t116_re;
  i_t42_re = t120 * d_t73_im_tmp;
  d_t73_re_tmp = t73_re_tmp_tmp_tmp * t119;
  t81 = d_t73_re_tmp * t129.re;
  t98 = d_t73_re_tmp * t129.im;
  f_re_tmp = 9.869604401089358 * t73 * t108_tmp;
  re = f_re_tmp * t113_re;
  im = f_re_tmp * t113_im;
  b_re = t121 * (re * t116_re - im * t116_im);
  im = t121 * (re * t116_im + im * t116_re);
  i_t68_re_tmp = t68 * t118;
  j_t68_re_tmp = i_t68_re_tmp * t128.re;
  g_t68_im_tmp = i_t68_re_tmp * t128.im;
  t68_re = j_t68_re_tmp * t222_re - g_t68_im_tmp * t222_im;
  t68_im = j_t68_re_tmp * t222_im + g_t68_im_tmp * t222_re;
  if (t68_im == 0.0) {
    t68_re /= 2.0;
    t68_im = 0.0;
  } else if (t68_re == 0.0) {
    t68_re = 0.0;
    t68_im /= 2.0;
  } else {
    t68_re /= 2.0;
    t68_im /= 2.0;
  }

  t97.re = ((((((t97.re + t73_re) + -b_t56_re) + -(3.1415926535897931 *
    (b_t93_re_tmp * d - t93_im_tmp * 0.0) * 1.1428571428571428)) +
              -(3.1415926535897931 * (e_t42_im * d - i_t42_re * 0.0) *
                0.5714285714285714)) + 3.1415926535897931 * (t81 * d - t98 * 0.0)
             * 0.2857142857142857) + (b_re * d1 - im * 0.0) *
            0.32653061224489793) + t68_re;
  t97.im = ((((((t97.im + t73_im) + -t50_re) + -(3.1415926535897931 *
    (b_t93_re_tmp * 0.0 + t93_im_tmp * d) * 1.1428571428571428)) +
              -(3.1415926535897931 * (e_t42_im * 0.0 + i_t42_re * d) *
                0.5714285714285714)) + 3.1415926535897931 * (t81 * 0.0 + t98 * d)
             * 0.2857142857142857) + (b_re * 0.0 + im * d1) *
            0.32653061224489793) + t68_im;
  e_t73_re_tmp = d_t73_re_tmp * t130_re;
  e_t73_im_tmp = d_t73_re_tmp * t130_im;
  k_t68_re_tmp = t120 * (b_t68_re_tmp * t117_re - t68_im_tmp * t117_im);
  t68_im_tmp = t120 * (b_t68_re_tmp * t117_im + t68_im_tmp * t117_re);
  b_t68_re_tmp = t68_re_tmp * t114_re;
  h_t68_im_tmp = t68_re_tmp * t114_im;
  t68_re_tmp = t120 * (b_t68_re_tmp * t116_re - h_t68_im_tmp * t116_im);
  i_t68_im_tmp = t120 * (b_t68_re_tmp * t116_im + h_t68_im_tmp * t116_re);
  d_t73_re_tmp = t73_re_tmp_tmp_tmp * t114_re;
  t68_im = t73_re_tmp_tmp_tmp * t114_im;
  f_t73_re_tmp = t120 * (d_t73_re_tmp * t117_re - t68_im * t117_im);
  t68_im = t120 * (d_t73_re_tmp * t117_im + t68_im * t117_re);
  t42_re_tmp_tmp_tmp = t42_re_tmp_tmp_tmp * t73 * t82;
  c_t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t94_tmp;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t119;
  b_t48_re = s_t42_re_tmp * t129.re * 3.1415926535897931;
  ai = s_t42_re_tmp * t129.im * 3.1415926535897931;
  if (ai == 0.0) {
    t42_re = b_t48_re / 7.0;
    t42_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t42_re = 0.0;
    t42_im = ai / 7.0;
  } else {
    t42_re = b_t48_re / 7.0;
    t42_im = ai / 7.0;
  }

  d_t73_re_tmp = t121 * t41_tmp;
  d_t73_im_tmp *= t121;
  t73_re = d_t73_re_tmp * t128.re - d_t73_im_tmp * t128.im;
  t73_im = d_t73_re_tmp * t128.im + d_t73_im_tmp * t128.re;
  b_t42_re = c_t42_re_tmp_tmp * t113_re;
  b_t42_im = c_t42_re_tmp_tmp * t113_im;
  t93_re = ((((((((3.1415926535897931 * e_t93_re_tmp * 0.5714285714285714 -
                   3.1415926535897931 * (e_t73_re_tmp * t183.re - e_t73_im_tmp *
    t183.im) * 0.42857142857142855) - 3.1415926535897931 * (d_t93_re_tmp *
    t183.re - b_t93_im_tmp * t183.im) * 1.1428571428571428) + 3.1415926535897931
                 * j_t42_re_tmp * 0.5714285714285714) + 3.1415926535897931 *
                (k_t68_re_tmp * t183.re - t68_im_tmp * t183.im) *
                0.5714285714285714) - 3.1415926535897931 * (t68_re_tmp * t183.re
    - i_t68_im_tmp * t183.im) * 0.5714285714285714) - 3.1415926535897931 *
              (f_t73_re_tmp * t183.re - t68_im * t183.im) * 0.2857142857142857)
             - t42_re) + 3.1415926535897931 * (t73_re * t183.re - t73_im *
             t183.im) * 0.8571428571428571) + 3.1415926535897931 * (t120 *
    (b_t42_re * t116_re - b_t42_im * t116_im)) * 0.2857142857142857;
  t93_im = ((((((((3.1415926535897931 * d_t93_im_tmp * 0.5714285714285714 -
                   3.1415926535897931 * (e_t73_re_tmp * t183.im + e_t73_im_tmp *
    t183.re) * 0.42857142857142855) - 3.1415926535897931 * (d_t93_re_tmp *
    t183.im + b_t93_im_tmp * t183.re) * 1.1428571428571428) + 3.1415926535897931
                 * e_t42_im_tmp * 0.5714285714285714) + 3.1415926535897931 *
                (k_t68_re_tmp * t183.im + t68_im_tmp * t183.re) *
                0.5714285714285714) - 3.1415926535897931 * (t68_re_tmp * t183.im
    + i_t68_im_tmp * t183.re) * 0.5714285714285714) - 3.1415926535897931 *
              (f_t73_re_tmp * t183.im + t68_im * t183.re) * 0.2857142857142857)
             - t42_im) + 3.1415926535897931 * (t73_re * t183.im + t73_im *
             t183.re) * 0.8571428571428571) + 3.1415926535897931 * (t120 *
    (b_t42_re * t116_im + b_t42_im * t116_re)) * 0.2857142857142857;
  j_t44_re_tmp = f_t44_re_tmp * d1 - e_t44_im_tmp * 0.0;
  e_t44_im_tmp = f_t44_re_tmp * 0.0 + e_t44_im_tmp * d1;
  f_t44_re_tmp = h_t44_re_tmp * d1 - g_t44_im_tmp * 0.0;
  g_t44_im_tmp = h_t44_re_tmp * 0.0 + g_t44_im_tmp * d1;
  t44_re = 3.1415926535897931 * (f_t44_re_tmp * t223_re - g_t44_im_tmp * t223_im);
  t44_im = 3.1415926535897931 * (f_t44_re_tmp * t223_im + g_t44_im_tmp * t223_re);
  t58_re_tmp = b_t58_re_tmp * d1 - t58_im_tmp * 0.0;
  t58_im_tmp = b_t58_re_tmp * 0.0 + t58_im_tmp * d1;
  h_t44_re_tmp = g_t44_re_tmp * d1 - f_t44_im_tmp * 0.0;
  f_t44_im_tmp = g_t44_re_tmp * 0.0 + f_t44_im_tmp * d1;
  g_t44_re_tmp = b_t44_re_tmp * t222_re - t44_im_tmp * t222_im;
  t108_tmp = b_t44_re_tmp * t222_im + t44_im_tmp * t222_re;
  b_t58_re_tmp = c_t58_re_tmp * d1 - b_t58_im_tmp * 0.0;
  b_t58_im_tmp = c_t58_re_tmp * 0.0 + b_t58_im_tmp * d1;
  t48_im = 3.1415926535897931 * (b_t58_re_tmp * t223_re - b_t58_im_tmp * t223_im);
  t58_im = 3.1415926535897931 * (b_t58_re_tmp * t223_im + b_t58_im_tmp * t223_re);
  t78 = i_t44_re_tmp * d1 - h_t44_im_tmp * 0.0;
  h_t44_im_tmp = i_t44_re_tmp * 0.0 + h_t44_im_tmp * d1;
  b_t44_re = 3.1415926535897931 * (t78 * t223_re - h_t44_im_tmp * t223_im);
  b_t44_im = 3.1415926535897931 * (t78 * t223_im + h_t44_im_tmp * t223_re);
  d_t56_re_tmp = 3.1415926535897931 * (d_t44_re_tmp * t243_re - d_t44_im_tmp *
    t243_im);
  d_t50_re_tmp = 3.1415926535897931 * (d_t44_re_tmp * t243_im + d_t44_im_tmp *
    t243_re);
  i_t44_re_tmp = c_t44_re_tmp * t222_re - b_t44_im_tmp * t222_im;
  t68_re = c_t44_re_tmp * t222_im + b_t44_im_tmp * t222_re;
  c_t58_re_tmp = 3.1415926535897931 * (i_t44_re_tmp * t223_re - t68_re * t223_im);
  b_t93_re = 3.1415926535897931 * (i_t44_re_tmp * t223_im + t68_re * t223_re);
  c_t42_re_tmp_tmp = c_t42_re_tmp_tmp_tmp * t82 * t88;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t106;
  t42_re = s_t42_re_tmp * t123_re;
  t42_im = s_t42_re_tmp * t123_im;
  t52_re_tmp = b_t52_re_tmp * d1 - t52_im_tmp * 0.0;
  t52_im_tmp = b_t52_re_tmp * 0.0 + t52_im_tmp * d1;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t100;
  b_t42_re = s_t42_re_tmp * t123_re;
  b_t42_im = s_t42_re_tmp * t123_im;
  c_t42_re = 3.1415926535897931 * (b_t42_re * d - b_t42_im * 0.0);
  b_t42_im = 3.1415926535897931 * (b_t42_re * 0.0 + b_t42_im * d);
  b_t52_re_tmp = c_t52_re_tmp * d1 - b_t52_im_tmp * 0.0;
  b_t52_im_tmp = c_t52_re_tmp * 0.0 + b_t52_im_tmp * d1;
  t48_re = 3.1415926535897931 * (b_t52_re_tmp * t223_re - b_t52_im_tmp * t223_im);
  t54_re = 3.1415926535897931 * (b_t52_re_tmp * t223_im + b_t52_im_tmp * t223_re);
  f_re_tmp = c_re_tmp_tmp * t82 * t88 * t105;
  re = f_re_tmp * t123_re;
  im = f_re_tmp * t123_im;
  c_t42_re_tmp_tmp = t42 * t58_tmp * t58_tmp * t66 * t73 * t82;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t106;
  b_t42_re = s_t42_re_tmp * t123_re;
  c_t42_im = s_t42_re_tmp * t123_im;
  b_t42_re_tmp_tmp_tmp = c_t42_re_tmp_tmp_tmp * t73 * t82;
  d_t42_re_tmp_tmp = b_t42_re_tmp_tmp_tmp * t89;
  s_t42_re_tmp = d_t42_re_tmp_tmp * t106;
  d_t42_re = s_t42_re_tmp * t123_re;
  d_t42_im = s_t42_re_tmp * t123_im;
  f_re_tmp = n_re_tmp * d1 - l_im_tmp * 0.0;
  l_im_tmp = n_re_tmp * 0.0 + l_im_tmp * d1;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t100;
  e_t42_re = s_t42_re_tmp * t123_re;
  t73_im = s_t42_re_tmp * t123_im;
  f_t42_re = 3.1415926535897931 * (e_t42_re * d - t73_im * 0.0);
  t73_im = 3.1415926535897931 * (e_t42_re * 0.0 + t73_im * d);
  s_t42_re_tmp = d_t42_re_tmp_tmp * t100;
  e_t42_re = s_t42_re_tmp * t123_re;
  e_t42_im = s_t42_re_tmp * t123_im;
  g_t42_re = 3.1415926535897931 * (e_t42_re * d - e_t42_im * 0.0);
  e_t42_im = 3.1415926535897931 * (e_t42_re * 0.0 + e_t42_im * d);
  s_t42_re_tmp = t42_re_tmp_tmp * t93 * t105 * t119;
  e_t42_re = s_t42_re_tmp * t123_re;
  t93_re_tmp_tmp = s_t42_re_tmp * t123_im;
  h_t42_re = e_t42_re * t128.re - t93_re_tmp_tmp * t128.im;
  t93_re_tmp_tmp = e_t42_re * t128.im + t93_re_tmp_tmp * t128.re;
  n_re_tmp = 9.869604401089358 * t42 * t58_tmp * t58_tmp * t66 * t73 * t82 *
    t105;
  b_re = n_re_tmp * t123_re;
  b_im = n_re_tmp * t123_im;
  b_re_tmp_tmp = c_re_tmp_tmp * t73 * t82;
  n_re_tmp = b_re_tmp_tmp * t89 * t105;
  c_re = n_re_tmp * t123_re;
  c_im = n_re_tmp * t123_im;
  t42_re_tmp_tmp = t42 * t52 * t58_tmp * t58_tmp * t66 * t73 * t82;
  s_t42_re_tmp = t42_re_tmp_tmp * t107;
  e_t42_re = s_t42_re_tmp * t123_re;
  b_t48_re_tmp_tmp = s_t42_re_tmp * t123_im;
  n_re_tmp = d_re_tmp_tmp * t82 * t99 * t93 * t119;
  d_re = n_re_tmp * t123_re;
  d_im = n_re_tmp * t123_im;
  e_re = d_re * t128.re - d_im * t128.im;
  d_im = d_re * t128.im + d_im * t128.re;
  d_re = e_re * d1 - d_im * 0.0;
  d_im = e_re * 0.0 + d_im * d1;
  s_t42_re_tmp = t42_re_tmp_tmp * t101;
  i_t42_re = s_t42_re_tmp * t123_re;
  b_t54_re_tmp_tmp = s_t42_re_tmp * t123_im;
  t55_re_tmp_tmp = 3.1415926535897931 * (i_t42_re * d - b_t54_re_tmp_tmp * 0.0);
  b_t54_re_tmp_tmp = 3.1415926535897931 * (i_t42_re * 0.0 + b_t54_re_tmp_tmp * d);
  t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t93 * t105;
  s_t42_re_tmp = t42_re_tmp_tmp * t119;
  i_t42_re = s_t42_re_tmp * t123_re;
  b_t55_re_tmp_tmp = s_t42_re_tmp * t123_im;
  t55_re_tmp = i_t42_re * t129.re - b_t55_re_tmp_tmp * t129.im;
  b_t55_re_tmp_tmp = i_t42_re * t129.im + b_t55_re_tmp_tmp * t129.re;
  n_re_tmp = 31.006276680299816 * t42 * t52 * t58_tmp * t58_tmp * t66 * t73 *
    t82 * t100;
  e_re = n_re_tmp * t123_re;
  e_im = n_re_tmp * t123_im;
  f_re = e_re * d - e_im * 0.0;
  e_im = e_re * 0.0 + e_im * d;
  n_re_tmp = 9.869604401089358 * t42 * t52 * t58_tmp * t58_tmp * t66 * t73 * t82
    * t106;
  e_re = n_re_tmp * t123_re;
  f_im = n_re_tmp * t123_im;
  c_re_tmp_tmp = d_re_tmp_tmp * t73 * t82 * t99 * t93;
  n_re_tmp = c_re_tmp_tmp * t119;
  g_re = n_re_tmp * t123_re;
  g_im = n_re_tmp * t123_im;
  h_re = g_re * t129.re - g_im * t129.im;
  g_im = g_re * t129.im + g_im * t129.re;
  g_re = h_re * d1 - g_im * 0.0;
  g_im = h_re * 0.0 + g_im * d1;
  h_re = d_re_tmp * t222_re - g_im_tmp * t222_im;
  h_im = d_re_tmp * t222_im + g_im_tmp * t222_re;
  i_t42_re = t42_re_tmp_tmp * t113_re;
  ai = t42_re_tmp_tmp * t113_im;
  d_t43_re_tmp_tmp = t120 * (i_t42_re * t116_re - ai * t116_im);
  ai = t120 * (i_t42_re * t116_im + ai * t116_re);
  i_t42_re = d_t43_re_tmp_tmp * t123_re - ai * t123_im;
  ai = d_t43_re_tmp_tmp * t123_im + ai * t123_re;
  i_re = c_re_tmp_tmp * t113_re;
  i_im = c_re_tmp_tmp * t113_im;
  j_re = t120 * (i_re * t116_re - i_im * t116_im);
  i_im = t120 * (i_re * t116_im + i_im * t116_re);
  i_re = j_re * t123_re - i_im * t123_im;
  i_im = j_re * t123_im + i_im * t123_re;
  j_re = i_re * d1 - i_im * 0.0;
  i_im = i_re * 0.0 + i_im * d1;
  s_t42_re_tmp = b_t42_re_tmp_tmp_tmp * t88 * t93 * t106 * t119;
  d_t43_re_tmp_tmp = s_t42_re_tmp * t123_re;
  c_t43_re_tmp_tmp = s_t42_re_tmp * t123_im;
  c_t55_re = d_t43_re_tmp_tmp * t128.re - c_t43_re_tmp_tmp * t128.im;
  c_t43_re_tmp_tmp = d_t43_re_tmp_tmp * t128.im + c_t43_re_tmp_tmp * t128.re;
  n_re_tmp = b_re_tmp_tmp * t88 * t100 * t93 * t119;
  i_re = n_re_tmp * t123_re;
  j_im = n_re_tmp * t123_im;
  t93_im_tmp = i_re * t128.re - j_im * t128.im;
  j_im = i_re * t128.im + j_im * t128.re;
  i_re = t93_im_tmp * d1 - j_im * 0.0;
  j_im = t93_im_tmp * 0.0 + j_im * d1;
  n_re_tmp = 31.006276680299816 * t42 * t44 * t66 * t73 * t82 * t88 * t93 * t105
    * t119;
  t93_im_tmp = n_re_tmp * t123_re;
  t73_re_tmp_tmp_tmp = n_re_tmp * t123_im;
  b_t93_re_tmp = t93_im_tmp * t128.re - t73_re_tmp_tmp_tmp * t128.im;
  t73_re_tmp_tmp_tmp = t93_im_tmp * t128.im + t73_re_tmp_tmp_tmp * t128.re;
  Mxzz->re = ((((((((((((b_t104_re_tmp * t93_re - b_t104_im_tmp * t93_im) *
                        1.0669676460233539 + (d_t104_re_tmp * t243_re -
    d_t104_im_tmp * t243_im) * 2.133935292046707) + ((((c_t104_re_tmp * t97.re -
    c_t104_im_tmp * t97.im) * -1.0669676460233539 + (j_t44_re_tmp * t223_re -
    e_t44_im_tmp * t223_im) * 1.0669676460233539) - (t44_re * 0.0 - t44_im *
    248.85542764393091)) + (t58_re_tmp * t223_re - t58_im_tmp * t223_im) *
    0.26674191150583842)) + ((((h_t44_re_tmp * t223_re - f_t44_im_tmp * t223_im)
    * -0.26674191150583842 - (t44_re_tmp * t243_re - c_t44_im_tmp * t243_im) *
    1.0669676460233539) + (g_t44_re_tmp * t223_re - t108_tmp * t223_im) *
    1.0669676460233539) - (t48_im * 0.0 - t58_im * 62.21385691098272))) +
                     ((((b_t44_re * 0.0 - b_t44_im * 62.21385691098272) +
                        (d_t56_re_tmp * 0.0 - d_t50_re_tmp * 248.85542764393091))
                       - (c_t58_re_tmp * 0.0 - b_t93_re * 248.85542764393091)) +
                      (t42_re * d - t42_im * 0.0) * 1.6004514690350311)) +
                    ((((t52_re_tmp * t223_re - t52_im_tmp * t223_im) *
                       -0.80022573451751533 - (c_t42_re * 0.0 - b_t42_im *
    373.28314146589628)) + (t48_re * 0.0 - t54_re * 186.6415707329482)) - (re *
    d - im * 0.0) * 29021.041124656658)) + ((((b_t42_re * d - c_t42_im * 0.0) *
    0.40011286725875761 - (d_t42_re * d - d_t42_im * 0.0) * 0.40011286725875761)
    + (f_re_tmp * t223_re - l_im_tmp * t223_im) * 14510.520562328329) -
    (f_t42_re * 0.0 - t73_im * 93.320785366474084))) + (((g_t42_re * 0.0 -
    e_t42_im * 93.320785366474084) - 3.1415926535897931 * (h_t42_re * d1 -
    t93_re_tmp_tmp * 0.0) * 0.60969579772763072) - (b_re * d - b_im * 0.0) *
    7255.2602811641664)) + (((c_re * d - c_im * 0.0) * 7255.2602811641664 -
    (e_t42_re * d - b_t48_re_tmp_tmp * 0.0) * 2.0005643362937882) + (d_re * 0.0
    - d_im * 142.2031015108177))) + (((t55_re_tmp_tmp * 0.0 - b_t54_re_tmp_tmp *
    466.60392683237041) + 3.1415926535897931 * (t55_re_tmp * d1 -
    b_t55_re_tmp_tmp * 0.0) * 0.15242394943190771) - 3.1415926535897931 *
    (q_t42_re_tmp * t222_re - q_t42_im_tmp * t222_im) * 0.3048478988638153)) +
               (((f_re * -0.0 - e_im * -1.6921889868604471E+6) + (e_re * d -
    f_im * 0.0) * 43531.561686985) - (g_re * 0.0 - g_im * 35.550775377704412)))
              + ((((h_re * 0.0 - h_im * 71.101550755408823) - 3.1415926535897931
                   * (i_t42_re * d1 - ai * 0.0) * 0.3048478988638153) + (j_re *
    0.0 - i_im * 71.101550755408823)) + 3.1415926535897931 * (c_t55_re * d1 -
    c_t43_re_tmp_tmp * 0.0) * 0.914543696591446)) + ((i_re * -0.0 - j_im *
    -213.3046522662265) - (b_t93_re_tmp * d1 - t73_re_tmp_tmp_tmp * 0.0) *
    16583.452071232379);
  Mxzz->im = ((((((((((((b_t104_re_tmp * t93_im + b_t104_im_tmp * t93_re) *
                        1.0669676460233539 + (d_t104_re_tmp * t243_im +
    d_t104_im_tmp * t243_re) * 2.133935292046707) + ((((c_t104_re_tmp * t97.im +
    c_t104_im_tmp * t97.re) * -1.0669676460233539 + (j_t44_re_tmp * t223_im +
    e_t44_im_tmp * t223_re) * 1.0669676460233539) - (t44_re * 248.85542764393091
    + t44_im * 0.0)) + (t58_re_tmp * t223_im + t58_im_tmp * t223_re) *
    0.26674191150583842)) + ((((h_t44_re_tmp * t223_im + f_t44_im_tmp * t223_re)
    * -0.26674191150583842 - (t44_re_tmp * t243_im + c_t44_im_tmp * t243_re) *
    1.0669676460233539) + (g_t44_re_tmp * t223_im + t108_tmp * t223_re) *
    1.0669676460233539) - (t48_im * 62.21385691098272 + t58_im * 0.0))) +
                     ((((b_t44_re * 62.21385691098272 + b_t44_im * 0.0) +
                        (d_t56_re_tmp * 248.85542764393091 + d_t50_re_tmp * 0.0))
                       - (c_t58_re_tmp * 248.85542764393091 + b_t93_re * 0.0)) +
                      (t42_re * 0.0 + t42_im * d) * 1.6004514690350311)) +
                    ((((t52_re_tmp * t223_im + t52_im_tmp * t223_re) *
                       -0.80022573451751533 - (c_t42_re * 373.28314146589628 +
    b_t42_im * 0.0)) + (t48_re * 186.6415707329482 + t54_re * 0.0)) - (re * 0.0
    + im * d) * 29021.041124656658)) + ((((b_t42_re * 0.0 + c_t42_im * d) *
    0.40011286725875761 - (d_t42_re * 0.0 + d_t42_im * d) * 0.40011286725875761)
    + (f_re_tmp * t223_im + l_im_tmp * t223_re) * 14510.520562328329) -
    (f_t42_re * 93.320785366474084 + t73_im * 0.0))) + (((g_t42_re *
    93.320785366474084 + e_t42_im * 0.0) - 3.1415926535897931 * (h_t42_re * 0.0
    + t93_re_tmp_tmp * d1) * 0.60969579772763072) - (b_re * 0.0 + b_im * d) *
    7255.2602811641664)) + (((c_re * 0.0 + c_im * d) * 7255.2602811641664 -
    (e_t42_re * 0.0 + b_t48_re_tmp_tmp * d) * 2.0005643362937882) + (d_re *
    142.2031015108177 + d_im * 0.0))) + (((t55_re_tmp_tmp * 466.60392683237041 +
    b_t54_re_tmp_tmp * 0.0) + 3.1415926535897931 * (t55_re_tmp * 0.0 +
    b_t55_re_tmp_tmp * d1) * 0.15242394943190771) - 3.1415926535897931 *
    (q_t42_re_tmp * t222_im + q_t42_im_tmp * t222_re) * 0.3048478988638153)) +
               (((f_re * -1.6921889868604471E+6 + e_im * -0.0) + (e_re * 0.0 +
    f_im * d) * 43531.561686985) - (g_re * 35.550775377704412 + g_im * 0.0))) +
              ((((h_re * 71.101550755408823 + h_im * 0.0) - 3.1415926535897931 *
                 (i_t42_re * 0.0 + ai * d1) * 0.3048478988638153) + (j_re *
    71.101550755408823 + i_im * 0.0)) + 3.1415926535897931 * (c_t55_re * 0.0 +
    c_t43_re_tmp_tmp * d1) * 0.914543696591446)) + ((i_re * -213.3046522662265 +
    j_im * -0.0) - (b_t93_re_tmp * 0.0 + t73_re_tmp_tmp_tmp * d1) *
    16583.452071232379);
  t42_re_tmp_tmp = t42_re_tmp_tmp_tmp_tmp * t67;
  c_t42_re_tmp_tmp = t42_re_tmp_tmp * t82 * t85;
  s_t42_re_tmp = c_t42_re_tmp_tmp * t95;
  t42_re = s_t42_re_tmp * t109_re;
  t98 = s_t42_re_tmp * t93;
  b_t42_re = s_t42_re_tmp * t112_re;
  c_t43_re_tmp_tmp = t43 * t54_tmp * t54_tmp * t67 * t69 * t85;
  t49 = c_t43_re_tmp_tmp * t95;
  t43_re = t49 * t109_re;
  c_t48_re = t49 * t93;
  b_t43_re = t49 * t112_re;
  t42_re_tmp_tmp = t42_re_tmp_tmp * t69 * t83 * t85;
  ab_t42_re_tmp = t42_re_tmp_tmp * t95;
  c_t42_re = ab_t42_re_tmp * t109_re;
  t81 = ab_t42_re_tmp * t93;
  d_t42_re = ab_t42_re_tmp * t112_re;
  d_t43_re_tmp_tmp = t43 * t48 * t54_tmp * t54_tmp * t67 * t69 * t85;
  t70 = d_t43_re_tmp_tmp * t96;
  c_t43_re = t70 * t109_re;
  b_t54_re = t70 * t93;
  d_t43_re = t70 * t112_re;
  e_t42_re = ((((((((((c_t42_re_tmp_tmp * t91 + c_t43_re_tmp_tmp * t91 / 4.0) -
                      t42_re_tmp_tmp * t91 / 4.0) + (t42_re * 0.0 - t98 * 0.5))
                    + (b_t42_re * 0.0 - t98 * 0.5)) - d_t43_re_tmp_tmp * t92 *
                   0.75) + (t43_re * 0.0 - c_t48_re * 0.125)) + (b_t43_re * 0.0
    - c_t48_re * 0.125)) - (c_t42_re * 0.0 - t81 * 0.125)) - (d_t42_re * 0.0 -
    t81 * 0.125)) - (c_t43_re * 0.0 - b_t54_re * 0.375)) - (d_t43_re * 0.0 -
    b_t54_re * 0.375);
  t42_im = (((((((t42_re * 0.5 + t98 * 0.0) + (b_t42_re * 0.5 + t98 * 0.0)) +
                (t43_re * 0.125 + c_t48_re * 0.0)) + (b_t43_re * 0.125 +
    c_t48_re * 0.0)) - (c_t42_re * 0.125 + t81 * 0.0)) - (d_t42_re * 0.125 + t81
              * 0.0)) - (c_t43_re * 0.375 + b_t54_re * 0.0)) - (d_t43_re * 0.375
    + b_t54_re * 0.0);
  t93_re = h_t93_re_tmp * t184.re - g_t93_im_tmp * t184.im;
  t93_im = h_t93_re_tmp * t184.im + g_t93_im_tmp * t184.re;
  t49_im_tmp = i_t93_re_tmp * t184.re - h_t93_im_tmp * t184.im;
  d_t54_im_tmp = i_t93_re_tmp * t184.im + h_t93_im_tmp * t184.re;
  b_t93_re = d_t93_re_tmp * t183.re - b_t93_im_tmp * t183.im;
  t98 = d_t93_re_tmp * t183.im + b_t93_im_tmp * t183.re;
  t41_tmp = d_t93_re_tmp * t184.re - b_t93_im_tmp * t184.im;
  t103 = d_t93_re_tmp * t184.im + b_t93_im_tmp * t184.re;
  b_t48_re = (f_t54_re_tmp * t184.re - c_t54_im_tmp * t184.im) *
    3.1415926535897931;
  ai = (f_t54_re_tmp * t184.im + c_t54_im_tmp * t184.re) * 3.1415926535897931;
  if (ai == 0.0) {
    t54_re = b_t48_re / 7.0;
    t56_re = 0.0;
  } else if (b_t48_re == 0.0) {
    t54_re = 0.0;
    t56_re = ai / 7.0;
  } else {
    t54_re = b_t48_re / 7.0;
    t56_re = ai / 7.0;
  }

  b_t48_re = (d_t42_re_tmp * t184.re - s_t42_im_tmp * t184.im) *
    3.1415926535897931;
  ai = (d_t42_re_tmp * t184.im + s_t42_im_tmp * t184.re) * 3.1415926535897931;
  if (ai == 0.0) {
    t42_re = b_t48_re / 7.0;
    b_t42_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t42_re = 0.0;
    b_t42_im = ai / 7.0;
  } else {
    t42_re = b_t48_re / 7.0;
    b_t42_im = ai / 7.0;
  }

  b_t48_re = (b_t43_re_tmp * t187_re - b_t55_im * t187_im) * 3.1415926535897931;
  ai = (b_t43_re_tmp * t187_im + b_t55_im * t187_re) * 3.1415926535897931;
  if (ai == 0.0) {
    t43_re = b_t48_re / 7.0;
    t43_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t43_re = 0.0;
    t43_im = ai / 7.0;
  } else {
    t43_re = b_t48_re / 7.0;
    t43_im = ai / 7.0;
  }

  c_t93_re_tmp = t93_re_tmp * t128.re - i_t93_im_tmp * t128.im;
  t58_re_tmp_tmp = t93_re_tmp * t128.im + i_t93_im_tmp * t128.re;
  c_t44_re_tmp_tmp = c_t93_re_tmp * t184.re - t58_re_tmp_tmp * t184.im;
  t58_re_tmp_tmp = c_t93_re_tmp * t184.im + t58_re_tmp_tmp * t184.re;
  b_t48_re = (g_t48_re_tmp * t184.re - c_t48_im_tmp * t184.im) *
    3.1415926535897931;
  ai = (g_t48_re_tmp * t184.im + c_t48_im_tmp * t184.re) * 3.1415926535897931;
  if (ai == 0.0) {
    t48_re = b_t48_re / 7.0;
    t48_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t48_re = 0.0;
    t48_im = ai / 7.0;
  } else {
    t48_re = b_t48_re / 7.0;
    t48_im = ai / 7.0;
  }

  b_t42_re = c_t42_re_tmp * t183.re - t_t42_im_tmp * t183.im;
  c_t42_im = c_t42_re_tmp * t183.im + t_t42_im_tmp * t183.re;
  c_t42_re = t42_re_tmp * t183.re - u_t42_im_tmp * t183.im;
  d_t42_im = t42_re_tmp * t183.im + u_t42_im_tmp * t183.re;
  d_t42_re = cb_t42_re_tmp * t183.re - v_t42_im_tmp * t183.im;
  t73_im = cb_t42_re_tmp * t183.im + v_t42_im_tmp * t183.re;
  f_t42_re = 3.1415926535897931 * (d_t42_re * t184.re - t73_im * t184.im);
  t73_im = 3.1415926535897931 * (d_t42_re * t184.im + t73_im * t184.re);
  d_t42_re = db_t42_re_tmp * t183.re - t42_im_tmp * t183.im;
  e_t42_im = db_t42_re_tmp * t183.im + t42_im_tmp * t183.re;
  g_t42_re = 3.1415926535897931 * (d_t42_re * t184.re - e_t42_im * t184.im);
  e_t42_im = 3.1415926535897931 * (d_t42_re * t184.im + e_t42_im * t184.re);
  c_t93_re_tmp = ((((g_t93_re_tmp * e_t42_re - f_t93_im_tmp * t42_im) *
                    0.5714285714285714 - 3.1415926535897931 * (t93_re * t187_re
    - t93_im * t187_im) * 0.42857142857142855) + 3.1415926535897931 *
                   (e_t93_re_tmp * t221_re - d_t93_im_tmp * t221_im) *
                   0.5714285714285714) + (((((((((((((((3.1415926535897931 *
    (j_t93_re_tmp * t241_re - c_t93_im_tmp * t241_im) * 0.2857142857142857 -
    3.1415926535897931 * (bb_t42_re_tmp * t184.re - r_t42_im_tmp * t184.im) *
    0.5714285714285714) - 3.1415926535897931 * (t49_im_tmp * t187_re -
    d_t54_im_tmp * t187_im) * 0.2857142857142857) - 3.1415926535897931 *
    (b_t93_re * t221_re - t98 * t221_im) * 1.1428571428571428) -
    3.1415926535897931 * (t41_tmp * t241_re - t103 * t241_im) *
    0.5714285714285714) + s_t42_re_tmp * t118 * 3.1415926535897931 *
    0.5714285714285714) - t54_re) + t42_re) - t43_re) + 3.1415926535897931 *
    (j_t42_re_tmp * t221_re - e_t42_im_tmp * t221_im) * 0.5714285714285714) +
    3.1415926535897931 * (i_t43_re_tmp * t241_re - e_t43_im_tmp * t241_im) *
    0.2857142857142857) + 3.1415926535897931 * (c_t44_re_tmp_tmp * t187_re -
    t58_re_tmp_tmp * t187_im) * 0.8571428571428571) + t49 * t118 *
    3.1415926535897931 / 7.0) - ab_t42_re_tmp * t118 * 3.1415926535897931 / 7.0)
    + t48_re) + 3.1415926535897931 * (t43_re_tmp * t187_re - d_t48_re_tmp_tmp *
    t187_im) * 0.2857142857142857)) + (((((3.1415926535897931 * (b_t42_re *
    t184.re - c_t42_im * t184.im) * -0.2857142857142857 - t70 * t118 *
    3.1415926535897931 * 0.42857142857142855) + 3.1415926535897931 * (c_t42_re *
    t184.re - d_t42_im * t184.im) * 0.5714285714285714) + 3.1415926535897931 *
    (t52_re_tmp_tmp * t183.re - t56_re_tmp_tmp * t183.im) * 0.2857142857142857)
    - (f_t42_re * 0.0 - t73_im * 0.2857142857142857)) - (g_t42_re * 0.0 -
    e_t42_im * 0.2857142857142857));
  t93_im = ((((g_t93_re_tmp * t42_im + f_t93_im_tmp * e_t42_re) *
              0.5714285714285714 - 3.1415926535897931 * (t93_re * t187_im +
    t93_im * t187_re) * 0.42857142857142855) + 3.1415926535897931 *
             (e_t93_re_tmp * t221_im + d_t93_im_tmp * t221_re) *
             0.5714285714285714) + ((((((((((((3.1415926535897931 *
    (j_t93_re_tmp * t241_im + c_t93_im_tmp * t241_re) * 0.2857142857142857 -
    3.1415926535897931 * (bb_t42_re_tmp * t184.im + r_t42_im_tmp * t184.re) *
    0.5714285714285714) - 3.1415926535897931 * (t49_im_tmp * t187_im +
    d_t54_im_tmp * t187_re) * 0.2857142857142857) - 3.1415926535897931 *
    (b_t93_re * t221_im + t98 * t221_re) * 1.1428571428571428) -
    3.1415926535897931 * (t41_tmp * t241_im + t103 * t241_re) *
    0.5714285714285714) - t56_re) + b_t42_im) - t43_im) + 3.1415926535897931 *
    (j_t42_re_tmp * t221_im + e_t42_im_tmp * t221_re) * 0.5714285714285714) +
    3.1415926535897931 * (i_t43_re_tmp * t241_im + e_t43_im_tmp * t241_re) *
    0.2857142857142857) + 3.1415926535897931 * (c_t44_re_tmp_tmp * t187_im +
    t58_re_tmp_tmp * t187_re) * 0.8571428571428571) + t48_im) +
             3.1415926535897931 * (t43_re_tmp * t187_im + d_t48_re_tmp_tmp *
              t187_re) * 0.2857142857142857)) + ((((3.1415926535897931 *
    (b_t42_re * t184.im + c_t42_im * t184.re) * -0.2857142857142857 +
    3.1415926535897931 * (c_t42_re * t184.im + d_t42_im * t184.re) *
    0.5714285714285714) + 3.1415926535897931 * (t52_re_tmp_tmp * t183.im +
    t56_re_tmp_tmp * t183.re) * 0.2857142857142857) - (f_t42_re *
    0.2857142857142857 + t73_im * 0.0)) - (g_t42_re * 0.2857142857142857 +
    e_t42_im * 0.0));
  t81 = d1 * t223_re - 0.0 * t223_im;
  t98 = d1 * t223_im + 0.0 * t223_re;
  re = b_re_tmp * t183.re - im_tmp * t183.im;
  im = b_re_tmp * t183.im + im_tmp * t183.re;
  t58_im = (((((d * t256_re - 0.0 * t256_im) + (t81 * t224_re - t98 * t224_im))
              - (c_re_tmp * t256_re - b_im_tmp * t256_im) * 0.875) + (e_re_tmp *
              t249_re - e_t48_re_tmp * t249_im) * 0.875) - (re * t224_re - im *
             t224_im) * 0.4375) - (t111 * t224_re - t115 * t224_im) * 0.4375;
  e_t48_re_tmp = (((((d * t256_im + 0.0 * t256_re) + (t81 * t224_im + t98 *
    t224_re)) - (c_re_tmp * t256_im + b_im_tmp * t256_re) * 0.875) + (e_re_tmp *
    t249_im + e_t48_re_tmp * t249_re) * 0.875) - (re * t224_im + im * t224_re) *
                  0.4375) - (t111 * t224_im + t115 * t224_re) * 0.4375;
  t42_re = 3.1415926535897931 * (l_t42_re_tmp * t224_re - h_t42_im_tmp * t224_im);
  t42_im = 3.1415926535897931 * (l_t42_re_tmp * t224_im + h_t42_im_tmp * t224_re);
  t54_re = 3.1415926535897931 * (b_t54_re_tmp * t224_re - b_t54_im_tmp * t224_im);
  t56_re = 3.1415926535897931 * (b_t54_re_tmp * t224_im + b_t54_im_tmp * t224_re);
  b_t42_re = 3.1415926535897931 * (w_t42_re_tmp * t224_re - i_t42_im_tmp *
    t224_im);
  b_t42_im = 3.1415926535897931 * (w_t42_re_tmp * t224_im + i_t42_im_tmp *
    t224_re);
  c_t42_re = 3.1415926535897931 * (p_t42_re_tmp * t256_re - k_t42_im_tmp *
    t256_im);
  c_t42_im = 3.1415926535897931 * (p_t42_re_tmp * t256_im + k_t42_im_tmp *
    t256_re);
  t43_re = 3.1415926535897931 * (k_t43_re_tmp * t254_re - g_t43_im_tmp * t254_im);
  t43_im = 3.1415926535897931 * (k_t43_re_tmp * t254_im + g_t43_im_tmp * t254_re);
  d_t42_re = 3.1415926535897931 * (o_t42_re_tmp * t250_re - b_t44_re_tmp_tmp_tmp
    * t250_im);
  d_t42_im = 3.1415926535897931 * (o_t42_re_tmp * t250_im + b_t44_re_tmp_tmp_tmp
    * t250_re);
  e_t42_re = 3.1415926535897931 * (x_t42_re_tmp * t249_re - t50_re_tmp_tmp *
    t249_im);
  t73_im = 3.1415926535897931 * (x_t42_re_tmp * t249_im + t50_re_tmp_tmp *
    t249_re);
  b_t43_re = 3.1415926535897931 * (p_t43_re_tmp * t249_re - e_t54_re_tmp *
    t249_im);
  b_t43_im = 3.1415926535897931 * (p_t43_re_tmp * t249_im + e_t54_re_tmp *
    t249_re);
  t42_re_tmp = c_t42_re_tmp_tmp * t106;
  f_t42_re = t42_re_tmp * t123_re;
  e_t42_im = t42_re_tmp * t123_im;
  t42_re_tmp = c_t42_re_tmp_tmp * t100;
  g_t42_re = t42_re_tmp * t123_re;
  t93_re_tmp_tmp = t42_re_tmp * t123_im;
  h_t42_re = 3.1415926535897931 * (g_t42_re * d - t93_re_tmp_tmp * 0.0);
  t93_re_tmp_tmp = 3.1415926535897931 * (g_t42_re * 0.0 + t93_re_tmp_tmp * d);
  t48_re = 3.1415926535897931 * (b_t48_re_tmp * t224_re - b_t48_im_tmp * t224_im);
  t48_im = 3.1415926535897931 * (b_t48_re_tmp * t224_im + b_t48_im_tmp * t224_re);
  b_re_tmp_tmp = b_re_tmp_tmp_tmp * t67;
  b_re_tmp = b_re_tmp_tmp * t82 * t85 * t105;
  re = b_re_tmp * t123_re;
  im = b_re_tmp * t123_im;
  p_t43_re_tmp = c_t43_re_tmp_tmp * t106;
  c_t43_re = p_t43_re_tmp * t123_re;
  c_t43_im = p_t43_re_tmp * t123_im;
  t42_re_tmp = t42_re_tmp_tmp * t106;
  g_t42_re = t42_re_tmp * t123_re;
  b_t48_re_tmp_tmp = t42_re_tmp * t123_im;
  p_t43_re_tmp = c_t43_re_tmp_tmp * t100;
  d_t43_re = p_t43_re_tmp * t123_re;
  e_t55_re = p_t43_re_tmp * t123_im;
  e_t43_re = 3.1415926535897931 * (d_t43_re * d - e_t55_re * 0.0);
  e_t55_re = 3.1415926535897931 * (d_t43_re * 0.0 + e_t55_re * d);
  t42_re_tmp = t42_re_tmp_tmp * t100;
  i_t42_re = t42_re_tmp * t123_re;
  b_t54_re_tmp_tmp = t42_re_tmp * t123_im;
  t55_re_tmp_tmp = 3.1415926535897931 * (i_t42_re * d - b_t54_re_tmp_tmp * 0.0);
  b_t54_re_tmp_tmp = 3.1415926535897931 * (i_t42_re * 0.0 + b_t54_re_tmp_tmp * d);
  b_re_tmp = 9.869604401089358 * t43 * t54_tmp * t54_tmp * t67 * t69 * t85 *
    t105;
  b_re = b_re_tmp * t123_re;
  b_im = b_re_tmp * t123_im;
  b_re_tmp = b_re_tmp_tmp * t69 * t83 * t85 * t105;
  c_re = b_re_tmp * t123_re;
  c_im = b_re_tmp * t123_im;
  p_t43_re_tmp = d_t43_re_tmp_tmp * t107;
  d_t43_re = p_t43_re_tmp * t123_re;
  e_t54_im = p_t43_re_tmp * t123_im;
  p_t43_re_tmp = d_t43_re_tmp_tmp * t101;
  b_t42_re_tmp_tmp_tmp = p_t43_re_tmp * t123_re;
  ai = p_t43_re_tmp * t123_im;
  t73_re = 3.1415926535897931 * (b_t42_re_tmp_tmp_tmp * d - ai * 0.0);
  ai = 3.1415926535897931 * (b_t42_re_tmp_tmp_tmp * 0.0 + ai * d);
  i_t42_re = 3.1415926535897931 * (r_t42_re_tmp * t223_re - m_t42_im_tmp *
    t223_im);
  b_t55_re_tmp_tmp = 3.1415926535897931 * (r_t42_re_tmp * t223_im + m_t42_im_tmp
    * t223_re);
  b_re_tmp = 31.006276680299816 * t43 * t48 * t54_tmp * t54_tmp * t67 * t69 *
    t85 * t100;
  d_re = b_re_tmp * t123_re;
  d_im = b_re_tmp * t123_im;
  e_re = d_re * d - d_im * 0.0;
  d_im = d_re * 0.0 + d_im * d;
  b_re_tmp = 9.869604401089358 * t43 * t48 * t54_tmp * t54_tmp * t67 * t69 * t85
    * t106;
  d_re = b_re_tmp * t123_re;
  e_im = b_re_tmp * t123_im;
  Myxx->re = ((((((((((((g_t104_re_tmp * t254_re - g_t104_im_tmp * t254_im) *
                        -1.0669676460233539 + (f_t104_re_tmp * t256_re -
    f_t104_im_tmp * t256_im) * 1.0669676460233539) + (b_t104_re_tmp *
    c_t93_re_tmp - b_t104_im_tmp * t93_im) * 1.0669676460233539) +
                      (((c_t104_re_tmp * t58_im - c_t104_im_tmp * e_t48_re_tmp) *
                        1.0669676460233539 + (i_t42_re_tmp * t224_re -
    f_t42_im_tmp * t224_im) * 1.0669676460233539) - (t42_re * 0.0 - t42_im *
    248.85542764393091))) + ((((t54_re_tmp * t224_re - t54_im_tmp * t224_im) *
    0.26674191150583842 - (n_t42_re_tmp * t224_re - g_t42_im_tmp * t224_im) *
    0.26674191150583842) - (k_t42_re_tmp * t256_re - j_t42_im_tmp * t256_im) *
    1.0669676460233539) + (j_t43_re_tmp * t254_re - f_t43_im_tmp * t254_im) *
    0.53348382301167685)) + ((((m_t42_re_tmp * t250_re - b_t44_re_tmp_tmp *
    t250_im) * 0.53348382301167685 + (b_t42_re_tmp * t249_re - f_t48_re_tmp *
    t249_im) * 0.53348382301167685) + (n_t43_re_tmp * t249_re - e_t48_im *
    t249_im) * 0.53348382301167685) - (t54_re * 0.0 - t56_re * 62.21385691098272)))
                   + ((((b_t42_re * 0.0 - b_t42_im * 62.21385691098272) +
                        (c_t42_re * 0.0 - c_t42_im * 248.85542764393091)) -
                       (t43_re * 0.0 - t43_im * 124.4277138219654)) - (d_t42_re *
    0.0 - d_t42_im * 124.4277138219654))) + ((((e_t42_re * -0.0 - t73_im *
    -124.4277138219654) - (b_t43_re * 0.0 - b_t43_im * 124.4277138219654)) +
    (f_t42_re * d - e_t42_im * 0.0) * 1.6004514690350311) - (t48_re_tmp *
    t224_re - t48_im_tmp * t224_im) * 0.80022573451751533)) + ((((h_t42_re *
    -0.0 - t93_re_tmp_tmp * -373.28314146589628) + (t48_re * 0.0 - t48_im *
    186.6415707329482)) - (re * d - im * 0.0) * 29021.041124656658) + (c_t43_re *
    d - c_t43_im * 0.0) * 0.40011286725875761)) + ((((g_t42_re * d -
    b_t48_re_tmp_tmp * 0.0) * -0.40011286725875761 + (o_re_tmp * t224_re -
    c_im_tmp * t224_im) * 14510.520562328329) - (e_t43_re * 0.0 - e_t55_re *
    93.320785366474084)) + (t55_re_tmp_tmp * 0.0 - b_t54_re_tmp_tmp *
    93.320785366474084))) + ((((b_re * d - b_im * 0.0) * -7255.2602811641664 +
    (c_re * d - c_im * 0.0) * 7255.2602811641664) - (d_t43_re * d - e_t54_im *
    0.0) * 2.0005643362937882) - (y_t42_re_tmp * t223_re - l_t42_im_tmp *
    t223_im) * 1.6004514690350311)) + (((t73_re * 0.0 - ai * 466.60392683237041)
    + (i_t42_re * 0.0 - b_t55_re_tmp_tmp * 373.28314146589628)) - (e_re * 0.0 -
    d_im * 1.6921889868604471E+6))) + ((d_re * d - e_im * 0.0) * 43531.561686985
    + (j_re_tmp * t223_re - d_im_tmp * t223_im) * 29021.041124656658);
  Myxx->im = ((((((((((((g_t104_re_tmp * t254_im + g_t104_im_tmp * t254_re) *
                        -1.0669676460233539 + (f_t104_re_tmp * t256_im +
    f_t104_im_tmp * t256_re) * 1.0669676460233539) + (b_t104_re_tmp * t93_im +
    b_t104_im_tmp * c_t93_re_tmp) * 1.0669676460233539) + (((c_t104_re_tmp *
    e_t48_re_tmp + c_t104_im_tmp * t58_im) * 1.0669676460233539 + (i_t42_re_tmp *
    t224_im + f_t42_im_tmp * t224_re) * 1.0669676460233539) - (t42_re *
    248.85542764393091 + t42_im * 0.0))) + ((((t54_re_tmp * t224_im + t54_im_tmp
    * t224_re) * 0.26674191150583842 - (n_t42_re_tmp * t224_im + g_t42_im_tmp *
    t224_re) * 0.26674191150583842) - (k_t42_re_tmp * t256_im + j_t42_im_tmp *
    t256_re) * 1.0669676460233539) + (j_t43_re_tmp * t254_im + f_t43_im_tmp *
    t254_re) * 0.53348382301167685)) + ((((m_t42_re_tmp * t250_im +
    b_t44_re_tmp_tmp * t250_re) * 0.53348382301167685 + (b_t42_re_tmp * t249_im
    + f_t48_re_tmp * t249_re) * 0.53348382301167685) + (n_t43_re_tmp * t249_im +
    e_t48_im * t249_re) * 0.53348382301167685) - (t54_re * 62.21385691098272 +
    t56_re * 0.0))) + ((((b_t42_re * 62.21385691098272 + b_t42_im * 0.0) +
    (c_t42_re * 248.85542764393091 + c_t42_im * 0.0)) - (t43_re *
    124.4277138219654 + t43_im * 0.0)) - (d_t42_re * 124.4277138219654 +
    d_t42_im * 0.0))) + ((((e_t42_re * -124.4277138219654 + t73_im * -0.0) -
    (b_t43_re * 124.4277138219654 + b_t43_im * 0.0)) + (f_t42_re * 0.0 +
    e_t42_im * d) * 1.6004514690350311) - (t48_re_tmp * t224_im + t48_im_tmp *
    t224_re) * 0.80022573451751533)) + ((((h_t42_re * -373.28314146589628 +
    t93_re_tmp_tmp * -0.0) + (t48_re * 186.6415707329482 + t48_im * 0.0)) - (re *
    0.0 + im * d) * 29021.041124656658) + (c_t43_re * 0.0 + c_t43_im * d) *
    0.40011286725875761)) + ((((g_t42_re * 0.0 + b_t48_re_tmp_tmp * d) *
    -0.40011286725875761 + (o_re_tmp * t224_im + c_im_tmp * t224_re) *
    14510.520562328329) - (e_t43_re * 93.320785366474084 + e_t55_re * 0.0)) +
    (t55_re_tmp_tmp * 93.320785366474084 + b_t54_re_tmp_tmp * 0.0))) + ((((b_re *
    0.0 + b_im * d) * -7255.2602811641664 + (c_re * 0.0 + c_im * d) *
    7255.2602811641664) - (d_t43_re * 0.0 + e_t54_im * d) * 2.0005643362937882)
    - (y_t42_re_tmp * t223_im + l_t42_im_tmp * t223_re) * 1.6004514690350311)) +
              (((t73_re * 466.60392683237041 + ai * 0.0) + (i_t42_re *
    373.28314146589628 + b_t55_re_tmp_tmp * 0.0)) - (e_re *
    1.6921889868604471E+6 + d_im * 0.0))) + ((d_re * 0.0 + e_im * d) *
    43531.561686985 + (j_re_tmp * t223_im + d_im_tmp * t223_re) *
    29021.041124656658);
  t56_re_tmp_tmp = t56_re_tmp_tmp_tmp * t67;
  d_t56_re_tmp = t56_re_tmp_tmp * t94_tmp;
  t56_re = d_t56_re_tmp * t109_re;
  t55_re_tmp_tmp = d_t56_re_tmp * t93;
  b_t56_re = d_t56_re_tmp * t112_re;
  t55_im = t56_tmp * t63 * t71;
  y_t42_re_tmp = t55_im * t94_tmp;
  b_t44_re_tmp_tmp_tmp = y_t42_re_tmp * t109_re;
  b_t55_re_tmp_tmp = y_t42_re_tmp * t93;
  o_re_tmp = t57 * t56_tmp * t71;
  t49_re_tmp = o_re_tmp * t94_tmp;
  b_t44_re_tmp_tmp = t49_re_tmp * t109_re;
  b_t54_re_tmp_tmp = t49_re_tmp * t93;
  t49_re_tmp_tmp = t57 * t63 * t72;
  e_t55_re = t49_re_tmp_tmp * t94_tmp;
  d_t54_im_tmp = e_t55_re * t109_re;
  t48_re_tmp_tmp = e_t55_re * t93;
  d_t48_re_tmp = y_t42_re_tmp * t112_re;
  t50_im = t49_re_tmp * t112_re;
  g_t42_re_tmp_tmp = e_t55_re * t112_re;
  c_t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t86;
  n_t43_re_tmp = c_t43_re_tmp_tmp * t94_tmp;
  t43_re = n_t43_re_tmp * t109_re;
  e_t48_im = n_t43_re_tmp * t93;
  d_t48_re = t56_re_tmp_tmp_tmp * t79;
  t93_re_tmp_tmp = d_t48_re * t94_tmp;
  t55 = t93_re_tmp_tmp * t109_re;
  t93_re = t93_re_tmp_tmp * t93;
  b_t43_re = n_t43_re_tmp * t112_re;
  t61 = t93_re_tmp_tmp * t112_re;
  d_t43_re_tmp_tmp = t43 * t72 * rt_powd_snf(t85, 5.0);
  p_t43_re_tmp = d_t43_re_tmp_tmp * t94_tmp;
  c_t43_re = p_t43_re_tmp * t109_re;
  e_t54_re_tmp = p_t43_re_tmp * t93;
  d_t43_re = p_t43_re_tmp * t112_re;
  t49_re = t43 * t79 * t86;
  t49 = t49_re * t94_tmp;
  e_t43_re = t49 * t109_re;
  c_t48_re = t49 * t93;
  b_t42_re_tmp_tmp_tmp = t49 * t112_re;
  t50_re_tmp_tmp = t50_re_tmp_tmp_tmp * t67;
  d_t50_re_tmp = t50_re_tmp_tmp * t95;
  t48_re = d_t50_re_tmp * t109_re;
  d_t42_re_tmp_tmp = d_t50_re_tmp * t93;
  t50_re = d_t50_re_tmp * t112_re;
  h_t48_re = t50_re_tmp_tmp_tmp_tmp * t63 * t71;
  e_t54_im = h_t48_re * t95;
  i_t42_re = e_t54_im * t109_re;
  t98 = e_t54_im * t93;
  t55_re = t50 * t57;
  c_t54_re = t55_re * t56_tmp * t71;
  c_t42_re_tmp_tmp = c_t54_re * t95;
  re_tmp_tmp_tmp = c_t42_re_tmp_tmp * t109_re;
  t41_tmp = c_t42_re_tmp_tmp * t93;
  t55_re = t55_re * t63 * t72;
  c_t93_re_tmp = t55_re * t95;
  ab_t42_re_tmp = c_t93_re_tmp * t109_re;
  t103 = c_t93_re_tmp * t93;
  c_t44_re_tmp_tmp = e_t54_im * t112_re;
  t52_re_tmp_tmp = c_t42_re_tmp_tmp * t112_re;
  c_t48_re_tmp = c_t93_re_tmp * t112_re;
  d_t55_re = t50_re_tmp_tmp_tmp * t79;
  b_t48_re_tmp_tmp = d_t55_re * t95;
  e_t42_re_tmp_tmp = b_t48_re_tmp_tmp * t109_re;
  ai = b_t48_re_tmp_tmp * t93;
  t49_im_tmp = b_t48_re_tmp_tmp * t112_re;
  t115 = t43 * t56_tmp * t56_tmp * t72 * t85;
  t70 = t115 * t95;
  t73_re = t70 * t109_re;
  b_t54_re = t70 * t93;
  e_t42_im = t70 * t112_re;
  c_t54_re_tmp = t51 * t56_tmp * t56_tmp * t72 * t85;
  t111 = c_t54_re_tmp * t96;
  d_t54_re_tmp = t111 * t109_re;
  t81 = t111 * t93;
  t58_re_tmp_tmp = t111 * t112_re;
  t54_re_tmp_tmp = ((((((((((((((((((t50_re_tmp_tmp * t91 - h_t48_re * t91 / 2.0)
    - c_t54_re * t91 / 2.0) - t55_re * t91 / 4.0) + d_t55_re * t91 / 2.0) -
    (t56_re * 0.0 - t55_re_tmp_tmp * 0.5)) - (b_t56_re * 0.0 - t55_re_tmp_tmp *
    0.5)) + (b_t44_re_tmp_tmp_tmp * 0.0 - b_t55_re_tmp_tmp * 0.25)) +
    (b_t44_re_tmp_tmp * 0.0 - b_t54_re_tmp_tmp * 0.25)) + (d_t54_im_tmp * 0.0 -
    t48_re_tmp_tmp * 0.125)) + (d_t48_re_tmp * 0.0 - b_t55_re_tmp_tmp * 0.25)) +
    (t50_im * 0.0 - b_t54_re_tmp_tmp * 0.25)) + (g_t42_re_tmp_tmp * 0.0 -
    t48_re_tmp_tmp * 0.125)) + (t43_re * 0.0 - e_t48_im * 0.5)) - (t55 * 0.0 -
    t93_re * 0.25)) + (b_t43_re * 0.0 - e_t48_im * 0.5)) - (t61 * 0.0 - t93_re *
    0.25)) - (c_t43_re * 0.0 - e_t54_re_tmp * 0.375)) + ((((((((((((((((d_t43_re
    * -0.0 - e_t54_re_tmp * -0.375) + (e_t43_re * 0.0 - c_t48_re * 0.25)) +
    (b_t42_re_tmp_tmp_tmp * 0.0 - c_t48_re * 0.25)) + t115 * t91 * 0.75) -
    c_t54_re_tmp * t92 * 0.75) + (t48_re * 0.0 - d_t42_re_tmp_tmp * 0.5)) +
    (t50_re * 0.0 - d_t42_re_tmp_tmp * 0.5)) - (i_t42_re * 0.0 - t98 * 0.25)) -
    (re_tmp_tmp_tmp * 0.0 - t41_tmp * 0.25)) - (ab_t42_re_tmp * 0.0 - t103 *
    0.125)) - (c_t44_re_tmp_tmp * 0.0 - t98 * 0.25)) - (t52_re_tmp_tmp * 0.0 -
    t41_tmp * 0.25)) - (c_t48_re_tmp * 0.0 - t103 * 0.125)) + (e_t42_re_tmp_tmp *
    0.0 - ai * 0.25)) + (t49_im_tmp * 0.0 - ai * 0.25)) + (t73_re * 0.0 -
    b_t54_re * 0.375))) + (((e_t42_im * 0.0 - b_t54_re * 0.375) - (d_t54_re_tmp *
    0.0 - t81 * 0.375)) - (t58_re_tmp_tmp * 0.0 - t81 * 0.375));
  t50_im = ((((((((((((((0.0 - (t56_re * 0.5 + t55_re_tmp_tmp * 0.0)) -
                        (b_t56_re * 0.5 + t55_re_tmp_tmp * 0.0)) +
                       (b_t44_re_tmp_tmp_tmp * 0.25 + b_t55_re_tmp_tmp * 0.0)) +
                      (b_t44_re_tmp_tmp * 0.25 + b_t54_re_tmp_tmp * 0.0)) +
                     (d_t54_im_tmp * 0.125 + t48_re_tmp_tmp * 0.0)) +
                    (d_t48_re_tmp * 0.25 + b_t55_re_tmp_tmp * 0.0)) + (t50_im *
    0.25 + b_t54_re_tmp_tmp * 0.0)) + (g_t42_re_tmp_tmp * 0.125 + t48_re_tmp_tmp
    * 0.0)) + (t43_re * 0.5 + e_t48_im * 0.0)) - (t55 * 0.25 + t93_re * 0.0)) +
               (b_t43_re * 0.5 + e_t48_im * 0.0)) - (t61 * 0.25 + t93_re * 0.0))
             - (c_t43_re * 0.375 + e_t54_re_tmp * 0.0)) + ((((((((((((((d_t43_re
    * -0.375 + e_t54_re_tmp * -0.0) + (e_t43_re * 0.25 + c_t48_re * 0.0)) +
    (b_t42_re_tmp_tmp_tmp * 0.25 + c_t48_re * 0.0)) + (t48_re * 0.5 +
    d_t42_re_tmp_tmp * 0.0)) + (t50_re * 0.5 + d_t42_re_tmp_tmp * 0.0)) -
    (i_t42_re * 0.25 + t98 * 0.0)) - (re_tmp_tmp_tmp * 0.25 + t41_tmp * 0.0)) -
    (ab_t42_re_tmp * 0.125 + t103 * 0.0)) - (c_t44_re_tmp_tmp * 0.25 + t98 * 0.0))
    - (t52_re_tmp_tmp * 0.25 + t41_tmp * 0.0)) - (c_t48_re_tmp * 0.125 + t103 *
    0.0)) + (e_t42_re_tmp_tmp * 0.25 + ai * 0.0)) + (t49_im_tmp * 0.25 + ai *
    0.0)) + (t73_re * 0.375 + b_t54_re * 0.0))) + (((e_t42_im * 0.375 + b_t54_re
    * 0.0) - (d_t54_re_tmp * 0.375 + t81 * 0.0)) - (t58_re_tmp_tmp * 0.375 + t81
    * 0.0));
  t93_re = t93_re_tmp * t128.re - i_t93_im_tmp * t128.im;
  t93_im = t93_re_tmp * t128.im + i_t93_im_tmp * t128.re;
  t49_im_tmp = d_t93_re_tmp * t184.re - b_t93_im_tmp * t184.im;
  d_t54_im_tmp = d_t93_re_tmp * t184.im + b_t93_im_tmp * t184.re;
  t43_re = 3.1415926535897931 * (c_t54_im * t189_re - d_t54_im * t189_im);
  t43_im = 3.1415926535897931 * (c_t54_im * t189_im + d_t54_im * t189_re);
  b_t43_re = 3.1415926535897931 * (t54_im * t189_re - t43_im_tmp * t189_im);
  b_t43_im = 3.1415926535897931 * (t54_im * t189_im + t43_im_tmp * t189_re);
  b_t93_re = ((((((((((((((((((g_t93_re_tmp * t54_re_tmp_tmp - f_t93_im_tmp *
    t50_im) * 0.5714285714285714 - 3.1415926535897931 * (h_t93_re_tmp * t190.re
    - g_t93_im_tmp * t190.im) * 0.42857142857142855) - d_t56_re_tmp * t118 *
    3.1415926535897931 * 0.5714285714285714) + y_t42_re_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857) + t49_re_tmp * t118 *
    3.1415926535897931 * 0.2857142857142857) + e_t55_re * t118 *
    3.1415926535897931 / 7.0) + n_t43_re_tmp * t118 * 3.1415926535897931 *
    0.5714285714285714) - t93_re_tmp_tmp * t118 * 3.1415926535897931 *
                        0.2857142857142857) - p_t43_re_tmp * t118 *
                       3.1415926535897931 * 0.42857142857142855) + t49 * t118 *
                      3.1415926535897931 * 0.2857142857142857) -
                     3.1415926535897931 * (i_t93_re_tmp * t190.re - h_t93_im_tmp
    * t190.im) * 0.2857142857142857) + 3.1415926535897931 * (j_t93_re_tmp *
    t242_re - c_t93_im_tmp * t242_im) * 0.8571428571428571) + d_t50_re_tmp *
                   t118 * 3.1415926535897931 * 0.5714285714285714) - e_t54_im *
                  t118 * 3.1415926535897931 * 0.2857142857142857) -
                 c_t42_re_tmp_tmp * t118 * 3.1415926535897931 *
                 0.2857142857142857) - c_t93_re_tmp * t118 * 3.1415926535897931 /
                7.0) + b_t48_re_tmp_tmp * t118 * 3.1415926535897931 *
               0.2857142857142857) - 3.1415926535897931 * (b_t48_im * t184.re -
    c_t48_im * t184.im) * 1.7142857142857142) + (((((((((((3.1415926535897931 *
    (t93_re * t190.re - t93_im * t190.im) * 0.8571428571428571 -
    3.1415926535897931 * (t49_im_tmp * t242_re - d_t54_im_tmp * t242_im) *
    1.7142857142857142) + t70 * t118 * 3.1415926535897931 * 0.42857142857142855)
    - t111 * t118 * 3.1415926535897931 * 0.42857142857142855) -
    3.1415926535897931 * (b_t56_re_tmp * t184.re - d_t48_im * t184.im) *
    0.42857142857142855) + 3.1415926535897931 * (c_t43_re_tmp * t184.re -
    t93_im_tmp_tmp * t184.im) * 0.42857142857142855) - 3.1415926535897931 *
    (b_t43_re_tmp * t189_re - b_t55_im * t189_im) * 0.42857142857142855) +
    3.1415926535897931 * (i_t43_re_tmp * t242_re - e_t43_im_tmp * t242_im) *
    0.8571428571428571) + 3.1415926535897931 * (b_t50_re_tmp * t184.re -
    c_t48_re_tmp_tmp * t184.im) * 0.42857142857142855) + 3.1415926535897931 *
    (t43_re_tmp * t189_re - d_t48_re_tmp_tmp * t189_im) * 0.8571428571428571) -
    (t43_re * 0.0 - t43_im * 0.2857142857142857)) - (b_t43_re * 0.0 - b_t43_im *
    0.2857142857142857));
  t93_im = (((((g_t93_re_tmp * t50_im + f_t93_im_tmp * t54_re_tmp_tmp) *
               0.5714285714285714 - 3.1415926535897931 * (h_t93_re_tmp * t190.im
    + g_t93_im_tmp * t190.re) * 0.42857142857142855) - 3.1415926535897931 *
              (i_t93_re_tmp * t190.im + h_t93_im_tmp * t190.re) *
              0.2857142857142857) + 3.1415926535897931 * (j_t93_re_tmp * t242_im
              + c_t93_im_tmp * t242_re) * 0.8571428571428571) -
            3.1415926535897931 * (b_t48_im * t184.im + c_t48_im * t184.re) *
            1.7142857142857142) + (((((((((3.1415926535897931 * (t93_re *
    t190.im + t93_im * t190.re) * 0.8571428571428571 - 3.1415926535897931 *
    (t49_im_tmp * t242_im + d_t54_im_tmp * t242_re) * 1.7142857142857142) -
    3.1415926535897931 * (b_t56_re_tmp * t184.im + d_t48_im * t184.re) *
    0.42857142857142855) + 3.1415926535897931 * (c_t43_re_tmp * t184.im +
    t93_im_tmp_tmp * t184.re) * 0.42857142857142855) - 3.1415926535897931 *
    (b_t43_re_tmp * t189_im + b_t55_im * t189_re) * 0.42857142857142855) +
    3.1415926535897931 * (i_t43_re_tmp * t242_im + e_t43_im_tmp * t242_re) *
    0.8571428571428571) + 3.1415926535897931 * (b_t50_re_tmp * t184.im +
    c_t48_re_tmp_tmp * t184.re) * 0.42857142857142855) + 3.1415926535897931 *
    (t43_re_tmp * t189_im + d_t48_re_tmp_tmp * t189_re) * 0.8571428571428571) -
    (t43_re * 0.2857142857142857 + t43_im * 0.0)) - (b_t43_re *
    0.2857142857142857 + b_t43_im * 0.0));
  b_t56_re_tmp = t56_re_tmp_tmp * t105;
  t56_re = b_t56_re_tmp * t123_re;
  t48_re_tmp_tmp = b_t56_re_tmp * t123_im;
  b_t56_re_tmp = t55_im * t105;
  b_t56_re = b_t56_re_tmp * t123_re;
  d_t48_re_tmp_tmp = b_t56_re_tmp * t123_im;
  t49_re_tmp = o_re_tmp * t105;
  b_t44_re_tmp_tmp = t49_re_tmp * t123_re;
  d_t54_im = t49_re_tmp * t123_im;
  t49_re_tmp = t49_re_tmp_tmp * t105;
  d_t54_im_tmp = t49_re_tmp * t123_re;
  c_t54_im = t49_re_tmp * t123_im;
  t43_re_tmp = c_t43_re_tmp_tmp * t105;
  t43_re = t43_re_tmp * t123_re;
  t43_im = t43_re_tmp * t123_im;
  b_t56_re_tmp = d_t48_re * t105;
  b_t44_re_tmp_tmp_tmp = b_t56_re_tmp * t123_re;
  t54_im = b_t56_re_tmp * t123_im;
  t43_re_tmp = d_t43_re_tmp_tmp * t105;
  b_t43_re = t43_re_tmp * t123_re;
  b_t43_im = t43_re_tmp * t123_im;
  t43_re_tmp = t49_re * t105;
  c_t43_re = t43_re_tmp * t123_re;
  c_t43_im = t43_re_tmp * t123_im;
  b_t56_re_tmp = t56_re_tmp_tmp * t99;
  d_t48_re_tmp = b_t56_re_tmp * t123_re;
  b_t55_im = b_t56_re_tmp * t123_im;
  t55 = 3.1415926535897931 * (d_t48_re_tmp * d - b_t55_im * 0.0);
  b_t55_im = 3.1415926535897931 * (d_t48_re_tmp * 0.0 + b_t55_im * d);
  b_t56_re_tmp = t55_im * t99;
  d_t48_re_tmp = b_t56_re_tmp * t123_re;
  c_t48_re_tmp_tmp = b_t56_re_tmp * t123_im;
  t61 = 3.1415926535897931 * (d_t48_re_tmp * d - c_t48_re_tmp_tmp * 0.0);
  c_t48_re_tmp_tmp = 3.1415926535897931 * (d_t48_re_tmp * 0.0 + c_t48_re_tmp_tmp
    * d);
  t49_re_tmp = o_re_tmp * t99;
  t50_im = t49_re_tmp * t123_re;
  t93_im_tmp_tmp = t49_re_tmp * t123_im;
  g_t42_re_tmp_tmp = 3.1415926535897931 * (t50_im * d - t93_im_tmp_tmp * 0.0);
  t93_im_tmp_tmp = 3.1415926535897931 * (t50_im * 0.0 + t93_im_tmp_tmp * d);
  t49_re_tmp = t49_re_tmp_tmp * t99;
  t50_im = t49_re_tmp * t123_re;
  c_t48_im = t49_re_tmp * t123_im;
  d_t48_im = 3.1415926535897931 * (t50_im * d - c_t48_im * 0.0);
  c_t48_im = 3.1415926535897931 * (t50_im * 0.0 + c_t48_im * d);
  t43_re_tmp = c_t43_re_tmp_tmp * t99;
  d_t43_re = t43_re_tmp * t123_re;
  e_t55_re = t43_re_tmp * t123_im;
  e_t43_re = 3.1415926535897931 * (d_t43_re * d - e_t55_re * 0.0);
  e_t55_re = 3.1415926535897931 * (d_t43_re * 0.0 + e_t55_re * d);
  b_t56_re_tmp = d_t48_re * t99;
  d_t48_re_tmp = b_t56_re_tmp * t123_re;
  e_t48_im = b_t56_re_tmp * t123_im;
  b_t48_im = 3.1415926535897931 * (d_t48_re_tmp * d - e_t48_im * 0.0);
  e_t48_im = 3.1415926535897931 * (d_t48_re_tmp * 0.0 + e_t48_im * d);
  t43_re_tmp = d_t43_re_tmp_tmp * t99;
  d_t43_re = t43_re_tmp * t123_re;
  e_t54_im = t43_re_tmp * t123_im;
  b_t42_re_tmp_tmp_tmp = 3.1415926535897931 * (d_t43_re * d - e_t54_im * 0.0);
  e_t54_im = 3.1415926535897931 * (d_t43_re * 0.0 + e_t54_im * d);
  t43_re_tmp = t49_re * t99;
  d_t43_re = t43_re_tmp * t123_re;
  ai = t43_re_tmp * t123_im;
  t73_re = 3.1415926535897931 * (d_t43_re * d - ai * 0.0);
  ai = 3.1415926535897931 * (d_t43_re * 0.0 + ai * d);
  d_t43_re = 3.1415926535897931 * (m_t43_re_tmp * t224_re - j_t43_im_tmp *
    t224_im);
  i_t48_re = 3.1415926535897931 * (m_t43_re_tmp * t224_im + j_t43_im_tmp *
    t224_re);
  b_t50_re_tmp = t50_re_tmp_tmp * t106;
  t48_re = b_t50_re_tmp * t123_re;
  t50_im = b_t50_re_tmp * t123_im;
  b_t50_re_tmp = h_t48_re * t106;
  t50_re = b_t50_re_tmp * t123_re;
  b_t55_re = b_t50_re_tmp * t123_im;
  b_t50_re_tmp = c_t54_re * t106;
  i_t42_re = b_t50_re_tmp * t123_re;
  c_t48_re = b_t50_re_tmp * t123_im;
  b_t50_re_tmp = t55_re * t106;
  re_tmp_tmp_tmp = b_t50_re_tmp * t123_re;
  b_t54_re = b_t50_re_tmp * t123_im;
  b_t50_re_tmp = d_t55_re * t106;
  ab_t42_re_tmp = b_t50_re_tmp * t123_re;
  b_t48_re = b_t50_re_tmp * t123_im;
  e_t42_im = e_t43_re_tmp * t224_re - b_t43_im_tmp * t224_im;
  f_t54_re = e_t43_re_tmp * t224_im + b_t43_im_tmp * t224_re;
  b_t50_re_tmp = t50_re_tmp_tmp * t100;
  c_t44_re_tmp_tmp = b_t50_re_tmp * t123_re;
  t55_re_tmp = b_t50_re_tmp * t123_im;
  t52_re_tmp_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * d - t55_re_tmp * 0.0);
  t55_re_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 + t55_re_tmp * d);
  b_t50_re_tmp = h_t48_re * t100;
  c_t44_re_tmp_tmp = b_t50_re_tmp * t123_re;
  c_t55_re = b_t50_re_tmp * t123_im;
  c_t48_re_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * d - c_t55_re * 0.0);
  c_t55_re = 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 + c_t55_re * d);
  b_t50_re_tmp = c_t54_re * t100;
  c_t44_re_tmp_tmp = b_t50_re_tmp * t123_re;
  c_t43_re_tmp_tmp = b_t50_re_tmp * t123_im;
  e_t42_re_tmp_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * d -
    c_t43_re_tmp_tmp * 0.0);
  c_t43_re_tmp_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 +
    c_t43_re_tmp_tmp * d);
  b_t50_re_tmp = t55_re * t100;
  c_t44_re_tmp_tmp = b_t50_re_tmp * t123_re;
  d_t43_re_tmp_tmp = b_t50_re_tmp * t123_im;
  t49_im_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * d - d_t43_re_tmp_tmp *
    0.0);
  d_t43_re_tmp_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 +
    d_t43_re_tmp_tmp * d);
  b_t50_re_tmp = d_t55_re * t100;
  c_t44_re_tmp_tmp = b_t50_re_tmp * t123_re;
  e_t48_re_tmp = b_t50_re_tmp * t123_im;
  t54_re_tmp_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * d - e_t48_re_tmp *
    0.0);
  e_t48_re_tmp = 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 + e_t48_re_tmp * d);
  d_t48_re_tmp = 3.1415926535897931 * (c_t56_re_tmp * t224_re - b_t56_im_tmp *
    t224_im);
  t50_re_tmp_tmp = 3.1415926535897931 * (c_t56_re_tmp * t224_im + b_t56_im_tmp *
    t224_re);
  e_t54_re = 3.1415926535897931 * (b_t54_im * t224_re - k_t43_im_tmp * t224_im);
  d_t54_re = 3.1415926535897931 * (b_t54_im * t224_im + k_t43_im_tmp * t224_re);
  b_t56_re_tmp = 3.1415926535897931 * (k_t43_re_tmp * t255_re - g_t43_im_tmp *
    t255_im);
  b_t50_re_tmp = 3.1415926535897931 * (k_t43_re_tmp * t255_im + g_t43_im_tmp *
    t255_re);
  e_t48_re = f_t43_re_tmp * t224_re - c_t43_im_tmp * t224_im;
  f_t48_re = f_t43_re_tmp * t224_im + c_t43_im_tmp * t224_re;
  g_t48_re = 3.1415926535897931 * (e_t48_re * t250_re - f_t48_re * t250_im);
  f_t48_re = 3.1415926535897931 * (e_t48_re * t250_im + f_t48_re * t250_re);
  b_re_tmp = f_re_tmp_tmp * t67 * t105;
  re = b_re_tmp * t123_re;
  im = b_re_tmp * t123_im;
  b_re_tmp = d_re_tmp_tmp_tmp * t63 * t71 * t105;
  b_re = b_re_tmp * t123_re;
  b_im = b_re_tmp * t123_im;
  b_re_tmp_tmp = 9.869604401089358 * t50 * t57;
  b_re_tmp = b_re_tmp_tmp * t56_tmp * t71 * t105;
  c_re = b_re_tmp * t123_re;
  c_im = b_re_tmp * t123_im;
  b_re_tmp = b_re_tmp_tmp * t63 * t72 * t105;
  d_re = b_re_tmp * t123_re;
  d_im = b_re_tmp * t123_im;
  b_re_tmp = f_re_tmp_tmp * t79 * t105;
  e_re = b_re_tmp * t123_re;
  e_im = b_re_tmp * t123_im;
  t43_re_tmp = t115 * t106;
  e_t48_re = t43_re_tmp * t123_re;
  t70 = t43_re_tmp * t123_im;
  t111 = c_t54_re_tmp * t107;
  d_t54_re_tmp = t111 * t123_re;
  t103 = t111 * t123_im;
  t43_re_tmp = t115 * t100;
  t81 = t43_re_tmp * t123_re;
  b_t54_im = t43_re_tmp * t123_im;
  t49 = 3.1415926535897931 * (t81 * d - b_t54_im * 0.0);
  b_t54_im = 3.1415926535897931 * (t81 * 0.0 + b_t54_im * d);
  t111 = c_t54_re_tmp * t101;
  t58_re_tmp_tmp = t111 * t123_re;
  t98 = t111 * t123_im;
  t41_tmp = 3.1415926535897931 * (t58_re_tmp_tmp * d - t98 * 0.0);
  t98 = 3.1415926535897931 * (t58_re_tmp_tmp * 0.0 + t98 * d);
  c_t44_re_tmp_tmp = 3.1415926535897931 * (c_t50_re_tmp * t224_re - b_t50_im_tmp
    * t224_im);
  t81 = 3.1415926535897931 * (c_t50_re_tmp * t224_im + b_t50_im_tmp * t224_re);
  b_re_tmp = 9.869604401089358 * t43 * t56_tmp * t56_tmp * t72 * t85 * t105;
  f_re = b_re_tmp * t123_re;
  f_im = b_re_tmp * t123_im;
  b_re_tmp = 31.006276680299816 * t51 * t56_tmp * t56_tmp * t72 * t85 * t100;
  g_re = b_re_tmp * t123_re;
  g_im = b_re_tmp * t123_im;
  h_re = g_re * d - g_im * 0.0;
  g_im = g_re * 0.0 + g_im * d;
  b_re_tmp = 9.869604401089358 * t51 * t56_tmp * t56_tmp * t72 * t85 * t106;
  g_re = b_re_tmp * t123_re;
  h_im = b_re_tmp * t123_im;
  Myyy->re = (((((((((((((((((g_t104_re_tmp * t255_re - g_t104_im_tmp * t255_im)
    * -2.133935292046707 + (b_t104_re_tmp * b_t93_re - b_t104_im_tmp * t93_im) *
    1.0669676460233539) + (e_t104_re_tmp * t229_re_tmp - e_t104_im_tmp *
    t229_im_tmp) * 1.0669676460233539) - (t56_re * d - t48_re_tmp_tmp * 0.0) *
    0.53348382301167685) + ((((b_t56_re * d - d_t48_re_tmp_tmp * 0.0) *
    0.26674191150583842 + (b_t44_re_tmp_tmp * d - d_t54_im * 0.0) *
    0.26674191150583842) + (d_t54_im_tmp * d - c_t54_im * 0.0) *
    0.13337095575291921) + (t43_re * d - t43_im * 0.0) * 0.53348382301167685)) +
    ((((b_t44_re_tmp_tmp_tmp * d - t54_im * 0.0) * -0.26674191150583842 -
       (b_t43_re * d - b_t43_im * 0.0) * 0.40011286725875761) + (c_t43_re * d -
    c_t43_im * 0.0) * 0.26674191150583842) + (h_t43_re_tmp * t224_re -
    h_t43_im_tmp * t224_im) * 3.2009029380700609)) + ((((t55 * 0.0 - b_t55_im *
    124.4277138219654) - (t61 * 0.0 - c_t48_re_tmp_tmp * 62.21385691098272)) -
    (g_t42_re_tmp_tmp * 0.0 - t93_im_tmp_tmp * 62.21385691098272)) - (d_t48_im *
    0.0 - c_t48_im * 31.10692845549136))) + ((((e_t43_re * -0.0 - e_t55_re *
    -124.4277138219654) + (b_t48_im * 0.0 - e_t48_im * 62.21385691098272)) +
    (b_t42_re_tmp_tmp_tmp * 0.0 - e_t54_im * 93.320785366474084)) - (t73_re *
    0.0 - ai * 62.21385691098272))) + ((((d_t43_re * -0.0 - i_t48_re *
    -746.56628293179267) + (t48_re * d - t50_im * 0.0) * 1.6004514690350311) -
    (t50_re * d - b_t55_re * 0.0) * 0.80022573451751533) - (i_t42_re * d -
    c_t48_re * 0.0) * 0.80022573451751533)) + ((((re_tmp_tmp_tmp * d - b_t54_re *
    0.0) * -0.40011286725875761 + (ab_t42_re_tmp * d - b_t48_re * 0.0) *
    0.80022573451751533) + (t56_re_tmp * t224_re - t56_im_tmp * t224_im) *
    0.80022573451751533) - (o_t43_re_tmp * t224_re - i_t43_im_tmp * t224_im) *
    0.80022573451751533)) + ((((j_t43_re_tmp * t255_re - f_t43_im_tmp * t255_im)
    * 1.6004514690350311 + (e_t42_im * t250_re - f_t54_re * t250_im) *
    1.6004514690350311) - (t52_re_tmp_tmp * 0.0 - t55_re_tmp *
    373.28314146589628)) + (c_t48_re_tmp * 0.0 - c_t55_re * 186.6415707329482)))
                   + ((((e_t42_re_tmp_tmp * 0.0 - c_t43_re_tmp_tmp *
    186.6415707329482) + (t49_im_tmp * 0.0 - d_t43_re_tmp_tmp *
    93.320785366474084)) - (t54_re_tmp_tmp * 0.0 - e_t48_re_tmp *
    186.6415707329482)) - (d_t48_re_tmp * 0.0 - t50_re_tmp_tmp *
    186.6415707329482))) + ((((e_t54_re * 0.0 - d_t54_re * 186.6415707329482) -
    (b_t56_re_tmp * 0.0 - b_t50_re_tmp * 373.28314146589628)) - (g_t48_re * 0.0
    - f_t48_re * 373.28314146589628)) - (re * d - im * 0.0) * 29021.041124656658))
                 + (((b_re * d - b_im * 0.0) * 14510.520562328329 + (c_re * d -
    c_im * 0.0) * 14510.520562328329) + (d_re * d - d_im * 0.0) *
                    7255.2602811641664)) + ((((e_re * d - e_im * 0.0) *
    -14510.520562328329 + (e_t48_re * d - t70 * 0.0) * 1.2003386017762729) -
    (d_t54_re_tmp * d - t103 * 0.0) * 2.0005643362937882) - (t50_re_tmp *
    t224_re - t50_im_tmp * t224_im) * 2.4006772035525459)) + (((t49 * -0.0 -
    b_t54_im * -279.96235609942232) + (t41_tmp * 0.0 - t98 * 466.60392683237041))
    + (c_t44_re_tmp_tmp * 0.0 - t81 * 559.92471219884453))) + (((f_re * d - f_im
    * 0.0) * -21765.7808434925 - (h_re * 0.0 - g_im * 1.6921889868604471E+6)) +
    (g_re * d - h_im * 0.0) * 43531.561686985)) + (re_tmp * t224_re - h_im_tmp *
    t224_im) * 43531.561686985;
  Myyy->im = (((((((((((((((((g_t104_re_tmp * t255_im + g_t104_im_tmp * t255_re)
    * -2.133935292046707 + (b_t104_re_tmp * t93_im + b_t104_im_tmp * b_t93_re) *
    1.0669676460233539) + (e_t104_re_tmp * t229_im_tmp + e_t104_im_tmp *
    t229_re_tmp) * 1.0669676460233539) - (t56_re * 0.0 + t48_re_tmp_tmp * d) *
    0.53348382301167685) + ((((b_t56_re * 0.0 + d_t48_re_tmp_tmp * d) *
    0.26674191150583842 + (b_t44_re_tmp_tmp * 0.0 + d_t54_im * d) *
    0.26674191150583842) + (d_t54_im_tmp * 0.0 + c_t54_im * d) *
    0.13337095575291921) + (t43_re * 0.0 + t43_im * d) * 0.53348382301167685)) +
    ((((b_t44_re_tmp_tmp_tmp * 0.0 + t54_im * d) * -0.26674191150583842 -
       (b_t43_re * 0.0 + b_t43_im * d) * 0.40011286725875761) + (c_t43_re * 0.0
    + c_t43_im * d) * 0.26674191150583842) + (h_t43_re_tmp * t224_im +
    h_t43_im_tmp * t224_re) * 3.2009029380700609)) + ((((t55 * 124.4277138219654
    + b_t55_im * 0.0) - (t61 * 62.21385691098272 + c_t48_re_tmp_tmp * 0.0)) -
    (g_t42_re_tmp_tmp * 62.21385691098272 + t93_im_tmp_tmp * 0.0)) - (d_t48_im *
    31.10692845549136 + c_t48_im * 0.0))) + ((((e_t43_re * -124.4277138219654 +
    e_t55_re * -0.0) + (b_t48_im * 62.21385691098272 + e_t48_im * 0.0)) +
    (b_t42_re_tmp_tmp_tmp * 93.320785366474084 + e_t54_im * 0.0)) - (t73_re *
    62.21385691098272 + ai * 0.0))) + ((((d_t43_re * -746.56628293179267 +
    i_t48_re * -0.0) + (t48_re * 0.0 + t50_im * d) * 1.6004514690350311) -
    (t50_re * 0.0 + b_t55_re * d) * 0.80022573451751533) - (i_t42_re * 0.0 +
    c_t48_re * d) * 0.80022573451751533)) + ((((re_tmp_tmp_tmp * 0.0 + b_t54_re *
    d) * -0.40011286725875761 + (ab_t42_re_tmp * 0.0 + b_t48_re * d) *
    0.80022573451751533) + (t56_re_tmp * t224_im + t56_im_tmp * t224_re) *
    0.80022573451751533) - (o_t43_re_tmp * t224_im + i_t43_im_tmp * t224_re) *
    0.80022573451751533)) + ((((j_t43_re_tmp * t255_im + f_t43_im_tmp * t255_re)
    * 1.6004514690350311 + (e_t42_im * t250_im + f_t54_re * t250_re) *
    1.6004514690350311) - (t52_re_tmp_tmp * 373.28314146589628 + t55_re_tmp *
    0.0)) + (c_t48_re_tmp * 186.6415707329482 + c_t55_re * 0.0))) +
                   ((((e_t42_re_tmp_tmp * 186.6415707329482 + c_t43_re_tmp_tmp *
                       0.0) + (t49_im_tmp * 93.320785366474084 +
    d_t43_re_tmp_tmp * 0.0)) - (t54_re_tmp_tmp * 186.6415707329482 +
    e_t48_re_tmp * 0.0)) - (d_t48_re_tmp * 186.6415707329482 + t50_re_tmp_tmp *
    0.0))) + ((((e_t54_re * 186.6415707329482 + d_t54_re * 0.0) - (b_t56_re_tmp *
    373.28314146589628 + b_t50_re_tmp * 0.0)) - (g_t48_re * 373.28314146589628 +
    f_t48_re * 0.0)) - (re * 0.0 + im * d) * 29021.041124656658)) + (((b_re *
    0.0 + b_im * d) * 14510.520562328329 + (c_re * 0.0 + c_im * d) *
    14510.520562328329) + (d_re * 0.0 + d_im * d) * 7255.2602811641664)) +
                ((((e_re * 0.0 + e_im * d) * -14510.520562328329 + (e_t48_re *
    0.0 + t70 * d) * 1.2003386017762729) - (d_t54_re_tmp * 0.0 + t103 * d) *
                  2.0005643362937882) - (t50_re_tmp * t224_im + t50_im_tmp *
    t224_re) * 2.4006772035525459)) + (((t49 * -279.96235609942232 + b_t54_im *
    -0.0) + (t41_tmp * 466.60392683237041 + t98 * 0.0)) + (c_t44_re_tmp_tmp *
    559.92471219884453 + t81 * 0.0))) + (((f_re * 0.0 + f_im * d) *
    -21765.7808434925 - (h_re * 1.6921889868604471E+6 + g_im * 0.0)) + (g_re *
    0.0 + h_im * d) * 43531.561686985)) + (re_tmp * t224_im + h_im_tmp * t224_re)
    * 43531.561686985;
  t43_re_tmp_tmp_tmp = t43_re_tmp_tmp_tmp * t73 * t85;
  c_t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t94_tmp;
  t43_re_tmp = c_t43_re_tmp_tmp * t119;
  b_t48_re = t43_re_tmp * t129.re * 3.1415926535897931;
  ai = t43_re_tmp * t129.im * 3.1415926535897931;
  if (ai == 0.0) {
    t43_re = b_t48_re / 7.0;
    t43_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t43_re = 0.0;
    t43_im = ai / 7.0;
  } else {
    t43_re = b_t48_re / 7.0;
    t43_im = ai / 7.0;
  }

  t73_re = d_t73_re_tmp * t128.re - d_t73_im_tmp * t128.im;
  t73_im = d_t73_re_tmp * t128.im + d_t73_im_tmp * t128.re;
  b_t43_re = c_t43_re_tmp_tmp * t113_re;
  b_t43_im = c_t43_re_tmp_tmp * t113_im;
  t93_re = ((((((((3.1415926535897931 * j_t93_re_tmp * 0.5714285714285714 -
                   3.1415926535897931 * (e_t73_re_tmp * t184.re - e_t73_im_tmp *
    t184.im) * 0.42857142857142855) - 3.1415926535897931 * (d_t93_re_tmp *
    t184.re - b_t93_im_tmp * t184.im) * 1.1428571428571428) + 3.1415926535897931
                 * i_t43_re_tmp * 0.5714285714285714) + 3.1415926535897931 *
                (k_t68_re_tmp * t184.re - t68_im_tmp * t184.im) *
                0.5714285714285714) - 3.1415926535897931 * (t68_re_tmp * t184.re
    - i_t68_im_tmp * t184.im) * 0.5714285714285714) - 3.1415926535897931 *
              (f_t73_re_tmp * t184.re - t68_im * t184.im) * 0.2857142857142857)
             - t43_re) + 3.1415926535897931 * (t73_re * t184.re - t73_im *
             t184.im) * 0.8571428571428571) + 3.1415926535897931 * (t120 *
    (b_t43_re * t116_re - b_t43_im * t116_im)) * 0.2857142857142857;
  t93_im = ((((((((3.1415926535897931 * c_t93_im_tmp * 0.5714285714285714 -
                   3.1415926535897931 * (e_t73_re_tmp * t184.im + e_t73_im_tmp *
    t184.re) * 0.42857142857142855) - 3.1415926535897931 * (d_t93_re_tmp *
    t184.im + b_t93_im_tmp * t184.re) * 1.1428571428571428) + 3.1415926535897931
                 * e_t43_im_tmp * 0.5714285714285714) + 3.1415926535897931 *
                (k_t68_re_tmp * t184.im + t68_im_tmp * t184.re) *
                0.5714285714285714) - 3.1415926535897931 * (t68_re_tmp * t184.im
    + i_t68_im_tmp * t184.re) * 0.5714285714285714) - 3.1415926535897931 *
              (f_t73_re_tmp * t184.im + t68_im * t184.re) * 0.2857142857142857)
             - t43_im) + 3.1415926535897931 * (t73_re * t184.im + t73_im *
             t184.re) * 0.8571428571428571) + 3.1415926535897931 * (t120 *
    (b_t43_re * t116_im + b_t43_im * t116_re)) * 0.2857142857142857;
  t44_re = 3.1415926535897931 * (f_t44_re_tmp * t224_re - g_t44_im_tmp * t224_im);
  t44_im = 3.1415926535897931 * (f_t44_re_tmp * t224_im + g_t44_im_tmp * t224_re);
  t48_im = 3.1415926535897931 * (b_t58_re_tmp * t224_re - b_t58_im_tmp * t224_im);
  t58_im = 3.1415926535897931 * (b_t58_re_tmp * t224_im + b_t58_im_tmp * t224_re);
  b_t44_re = 3.1415926535897931 * (t78 * t224_re - h_t44_im_tmp * t224_im);
  b_t44_im = 3.1415926535897931 * (t78 * t224_im + h_t44_im_tmp * t224_re);
  d_t56_re_tmp = 3.1415926535897931 * (d_t44_re_tmp * t244_re - d_t44_im_tmp *
    t244_im);
  d_t50_re_tmp = 3.1415926535897931 * (d_t44_re_tmp * t244_im + d_t44_im_tmp *
    t244_re);
  c_t58_re_tmp = 3.1415926535897931 * (i_t44_re_tmp * t224_re - t68_re * t224_im);
  b_t93_re = 3.1415926535897931 * (i_t44_re_tmp * t224_im + t68_re * t224_re);
  c_t43_re_tmp_tmp = b_t43_re_tmp_tmp_tmp * t85 * t88;
  t43_re_tmp = c_t43_re_tmp_tmp * t106;
  t43_re = t43_re_tmp * t123_re;
  t43_im = t43_re_tmp * t123_im;
  t43_re_tmp = c_t43_re_tmp_tmp * t100;
  b_t43_re = t43_re_tmp * t123_re;
  b_t43_im = t43_re_tmp * t123_im;
  c_t43_re = 3.1415926535897931 * (b_t43_re * d - b_t43_im * 0.0);
  b_t43_im = 3.1415926535897931 * (b_t43_re * 0.0 + b_t43_im * d);
  t48_re = 3.1415926535897931 * (b_t52_re_tmp * t224_re - b_t52_im_tmp * t224_im);
  t54_re = 3.1415926535897931 * (b_t52_re_tmp * t224_im + b_t52_im_tmp * t224_re);
  re_tmp = g_re_tmp_tmp * t85 * t88 * t105;
  re = re_tmp * t123_re;
  im = re_tmp * t123_im;
  c_t43_re_tmp_tmp = t43 * t58_tmp * t58_tmp * t67 * t73 * t85;
  t43_re_tmp = c_t43_re_tmp_tmp * t106;
  b_t43_re = t43_re_tmp * t123_re;
  c_t43_im = t43_re_tmp * t123_im;
  b_t43_re_tmp_tmp_tmp = b_t43_re_tmp_tmp_tmp * t73 * t85;
  d_t43_re_tmp_tmp = b_t43_re_tmp_tmp_tmp * t89;
  t43_re_tmp = d_t43_re_tmp_tmp * t106;
  d_t43_re = t43_re_tmp * t123_re;
  e_t55_re = t43_re_tmp * t123_im;
  t43_re_tmp = c_t43_re_tmp_tmp * t100;
  e_t43_re = t43_re_tmp * t123_re;
  e_t54_im = t43_re_tmp * t123_im;
  b_t42_re_tmp_tmp_tmp = 3.1415926535897931 * (e_t43_re * d - e_t54_im * 0.0);
  e_t54_im = 3.1415926535897931 * (e_t43_re * 0.0 + e_t54_im * d);
  t43_re_tmp = d_t43_re_tmp_tmp * t100;
  e_t43_re = t43_re_tmp * t123_re;
  ai = t43_re_tmp * t123_im;
  t73_re = 3.1415926535897931 * (e_t43_re * d - ai * 0.0);
  ai = 3.1415926535897931 * (e_t43_re * 0.0 + ai * d);
  t43_re_tmp = t43_re_tmp_tmp * t93 * t105 * t119;
  e_t43_re = t43_re_tmp * t123_re;
  i_t48_re = t43_re_tmp * t123_im;
  e_t42_im = e_t43_re * t128.re - i_t48_re * t128.im;
  i_t48_re = e_t43_re * t128.im + i_t48_re * t128.re;
  re_tmp = 9.869604401089358 * t43 * t58_tmp * t58_tmp * t67 * t73 * t85 * t105;
  b_re = re_tmp * t123_re;
  b_im = re_tmp * t123_im;
  b_re_tmp_tmp = g_re_tmp_tmp * t73 * t85;
  re_tmp = b_re_tmp_tmp * t89 * t105;
  c_re = re_tmp * t123_re;
  c_im = re_tmp * t123_im;
  t43_re_tmp_tmp = t43 * t52 * t58_tmp * t58_tmp * t67 * t73 * t85;
  t43_re_tmp = t43_re_tmp_tmp * t107;
  e_t43_re = t43_re_tmp * t123_re;
  f_t54_re = t43_re_tmp * t123_im;
  re_tmp = h_re_tmp_tmp * t85 * t99 * t93 * t119;
  d_re = re_tmp * t123_re;
  d_im = re_tmp * t123_im;
  e_re = d_re * t128.re - d_im * t128.im;
  d_im = d_re * t128.im + d_im * t128.re;
  d_re = e_re * d1 - d_im * 0.0;
  d_im = e_re * 0.0 + d_im * d1;
  t43_re_tmp = t43_re_tmp_tmp * t101;
  e_t54_re = t43_re_tmp * t123_re;
  d_t54_re = t43_re_tmp * t123_im;
  b_t56_re_tmp = 3.1415926535897931 * (e_t54_re * d - d_t54_re * 0.0);
  d_t54_re = 3.1415926535897931 * (e_t54_re * 0.0 + d_t54_re * d);
  t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t93 * t105;
  t43_re_tmp = t43_re_tmp_tmp * t119;
  e_t54_re = t43_re_tmp * t123_re;
  b_t50_re_tmp = t43_re_tmp * t123_im;
  e_t48_re = e_t54_re * t129.re - b_t50_re_tmp * t129.im;
  b_t50_re_tmp = e_t54_re * t129.im + b_t50_re_tmp * t129.re;
  re_tmp = 31.006276680299816 * t43 * t52 * t58_tmp * t58_tmp * t67 * t73 * t85 *
    t100;
  e_re = re_tmp * t123_re;
  e_im = re_tmp * t123_im;
  f_re = e_re * d - e_im * 0.0;
  e_im = e_re * 0.0 + e_im * d;
  re_tmp = 9.869604401089358 * t43 * t52 * t58_tmp * t58_tmp * t67 * t73 * t85 *
    t106;
  e_re = re_tmp * t123_re;
  f_im = re_tmp * t123_im;
  c_re_tmp_tmp = h_re_tmp_tmp * t73 * t85 * t99 * t93;
  re_tmp = c_re_tmp_tmp * t119;
  g_re = re_tmp * t123_re;
  g_im = re_tmp * t123_im;
  h_re = g_re * t129.re - g_im * t129.im;
  g_im = g_re * t129.im + g_im * t129.re;
  g_re = h_re * d1 - g_im * 0.0;
  g_im = h_re * 0.0 + g_im * d1;
  h_re = i_re_tmp * t222_re - k_im_tmp * t222_im;
  h_im = i_re_tmp * t222_im + k_im_tmp * t222_re;
  e_t54_re = t43_re_tmp_tmp * t113_re;
  f_t48_re = t43_re_tmp_tmp * t113_im;
  g_t48_re = t120 * (e_t54_re * t116_re - f_t48_re * t116_im);
  f_t48_re = t120 * (e_t54_re * t116_im + f_t48_re * t116_re);
  e_t54_re = g_t48_re * t123_re - f_t48_re * t123_im;
  f_t48_re = g_t48_re * t123_im + f_t48_re * t123_re;
  i_re = c_re_tmp_tmp * t113_re;
  i_im = c_re_tmp_tmp * t113_im;
  j_re = t120 * (i_re * t116_re - i_im * t116_im);
  i_im = t120 * (i_re * t116_im + i_im * t116_re);
  i_re = j_re * t123_re - i_im * t123_im;
  i_im = j_re * t123_im + i_im * t123_re;
  j_re = i_re * d1 - i_im * 0.0;
  i_im = i_re * 0.0 + i_im * d1;
  t43_re_tmp = b_t43_re_tmp_tmp_tmp * t88 * t93 * t106 * t119;
  g_t48_re = t43_re_tmp * t123_re;
  t70 = t43_re_tmp * t123_im;
  t81 = g_t48_re * t128.re - t70 * t128.im;
  t70 = g_t48_re * t128.im + t70 * t128.re;
  re_tmp = b_re_tmp_tmp * t88 * t100 * t93 * t119;
  i_re = re_tmp * t123_re;
  j_im = re_tmp * t123_im;
  t93_im_tmp = i_re * t128.re - j_im * t128.im;
  j_im = i_re * t128.im + j_im * t128.re;
  i_re = t93_im_tmp * d1 - j_im * 0.0;
  j_im = t93_im_tmp * 0.0 + j_im * d1;
  re_tmp = 31.006276680299816 * t43 * t44 * t67 * t73 * t85 * t88 * t93 * t105 *
    t119;
  t93_im_tmp = re_tmp * t123_re;
  t73_re_tmp_tmp_tmp = re_tmp * t123_im;
  b_t93_re_tmp = t93_im_tmp * t128.re - t73_re_tmp_tmp_tmp * t128.im;
  t73_re_tmp_tmp_tmp = t93_im_tmp * t128.im + t73_re_tmp_tmp_tmp * t128.re;
  Myzz->re = ((((((((((((b_t104_re_tmp * t93_re - b_t104_im_tmp * t93_im) *
                        1.0669676460233539 + (d_t104_re_tmp * t244_re -
    d_t104_im_tmp * t244_im) * 2.133935292046707) + ((((e_t104_re_tmp * t97.re -
    e_t104_im_tmp * t97.im) * -1.0669676460233539 + (j_t44_re_tmp * t224_re -
    e_t44_im_tmp * t224_im) * 1.0669676460233539) - (t44_re * 0.0 - t44_im *
    248.85542764393091)) + (t58_re_tmp * t224_re - t58_im_tmp * t224_im) *
    0.26674191150583842)) + ((((h_t44_re_tmp * t224_re - f_t44_im_tmp * t224_im)
    * -0.26674191150583842 - (t44_re_tmp * t244_re - c_t44_im_tmp * t244_im) *
    1.0669676460233539) + (g_t44_re_tmp * t224_re - t108_tmp * t224_im) *
    1.0669676460233539) - (t48_im * 0.0 - t58_im * 62.21385691098272))) +
                     ((((b_t44_re * 0.0 - b_t44_im * 62.21385691098272) +
                        (d_t56_re_tmp * 0.0 - d_t50_re_tmp * 248.85542764393091))
                       - (c_t58_re_tmp * 0.0 - b_t93_re * 248.85542764393091)) +
                      (t43_re * d - t43_im * 0.0) * 1.6004514690350311)) +
                    ((((t52_re_tmp * t224_re - t52_im_tmp * t224_im) *
                       -0.80022573451751533 - (c_t43_re * 0.0 - b_t43_im *
    373.28314146589628)) + (t48_re * 0.0 - t54_re * 186.6415707329482)) - (re *
    d - im * 0.0) * 29021.041124656658)) + ((((b_t43_re * d - c_t43_im * 0.0) *
    0.40011286725875761 - (d_t43_re * d - e_t55_re * 0.0) * 0.40011286725875761)
    + (f_re_tmp * t224_re - l_im_tmp * t224_im) * 14510.520562328329) -
    (b_t42_re_tmp_tmp_tmp * 0.0 - e_t54_im * 93.320785366474084))) + (((t73_re *
    0.0 - ai * 93.320785366474084) - 3.1415926535897931 * (e_t42_im * d1 -
    i_t48_re * 0.0) * 0.60969579772763072) - (b_re * d - b_im * 0.0) *
    7255.2602811641664)) + (((c_re * d - c_im * 0.0) * 7255.2602811641664 -
    (e_t43_re * d - f_t54_re * 0.0) * 2.0005643362937882) + (d_re * 0.0 - d_im *
    142.2031015108177))) + (((b_t56_re_tmp * 0.0 - d_t54_re * 466.60392683237041)
    + 3.1415926535897931 * (e_t48_re * d1 - b_t50_re_tmp * 0.0) *
    0.15242394943190771) - 3.1415926535897931 * (l_t43_re_tmp * t222_re -
    o_t43_im_tmp * t222_im) * 0.3048478988638153)) + (((f_re * -0.0 - e_im *
    -1.6921889868604471E+6) + (e_re * d - f_im * 0.0) * 43531.561686985) - (g_re
    * 0.0 - g_im * 35.550775377704412))) + ((((h_re * 0.0 - h_im *
    71.101550755408823) - 3.1415926535897931 * (e_t54_re * d1 - f_t48_re * 0.0) *
    0.3048478988638153) + (j_re * 0.0 - i_im * 71.101550755408823)) +
    3.1415926535897931 * (t81 * d1 - t70 * 0.0) * 0.914543696591446)) + ((i_re *
    -0.0 - j_im * -213.3046522662265) - (b_t93_re_tmp * d1 - t73_re_tmp_tmp_tmp *
    0.0) * 16583.452071232379);
  Myzz->im = ((((((((((((b_t104_re_tmp * t93_im + b_t104_im_tmp * t93_re) *
                        1.0669676460233539 + (d_t104_re_tmp * t244_im +
    d_t104_im_tmp * t244_re) * 2.133935292046707) + ((((e_t104_re_tmp * t97.im +
    e_t104_im_tmp * t97.re) * -1.0669676460233539 + (j_t44_re_tmp * t224_im +
    e_t44_im_tmp * t224_re) * 1.0669676460233539) - (t44_re * 248.85542764393091
    + t44_im * 0.0)) + (t58_re_tmp * t224_im + t58_im_tmp * t224_re) *
    0.26674191150583842)) + ((((h_t44_re_tmp * t224_im + f_t44_im_tmp * t224_re)
    * -0.26674191150583842 - (t44_re_tmp * t244_im + c_t44_im_tmp * t244_re) *
    1.0669676460233539) + (g_t44_re_tmp * t224_im + t108_tmp * t224_re) *
    1.0669676460233539) - (t48_im * 62.21385691098272 + t58_im * 0.0))) +
                     ((((b_t44_re * 62.21385691098272 + b_t44_im * 0.0) +
                        (d_t56_re_tmp * 248.85542764393091 + d_t50_re_tmp * 0.0))
                       - (c_t58_re_tmp * 248.85542764393091 + b_t93_re * 0.0)) +
                      (t43_re * 0.0 + t43_im * d) * 1.6004514690350311)) +
                    ((((t52_re_tmp * t224_im + t52_im_tmp * t224_re) *
                       -0.80022573451751533 - (c_t43_re * 373.28314146589628 +
    b_t43_im * 0.0)) + (t48_re * 186.6415707329482 + t54_re * 0.0)) - (re * 0.0
    + im * d) * 29021.041124656658)) + ((((b_t43_re * 0.0 + c_t43_im * d) *
    0.40011286725875761 - (d_t43_re * 0.0 + e_t55_re * d) * 0.40011286725875761)
    + (f_re_tmp * t224_im + l_im_tmp * t224_re) * 14510.520562328329) -
    (b_t42_re_tmp_tmp_tmp * 93.320785366474084 + e_t54_im * 0.0))) + (((t73_re *
    93.320785366474084 + ai * 0.0) - 3.1415926535897931 * (e_t42_im * 0.0 +
    i_t48_re * d1) * 0.60969579772763072) - (b_re * 0.0 + b_im * d) *
    7255.2602811641664)) + (((c_re * 0.0 + c_im * d) * 7255.2602811641664 -
    (e_t43_re * 0.0 + f_t54_re * d) * 2.0005643362937882) + (d_re *
    142.2031015108177 + d_im * 0.0))) + (((b_t56_re_tmp * 466.60392683237041 +
    d_t54_re * 0.0) + 3.1415926535897931 * (e_t48_re * 0.0 + b_t50_re_tmp * d1) *
    0.15242394943190771) - 3.1415926535897931 * (l_t43_re_tmp * t222_im +
    o_t43_im_tmp * t222_re) * 0.3048478988638153)) + (((f_re *
    -1.6921889868604471E+6 + e_im * -0.0) + (e_re * 0.0 + f_im * d) *
    43531.561686985) - (g_re * 35.550775377704412 + g_im * 0.0))) + ((((h_re *
    71.101550755408823 + h_im * 0.0) - 3.1415926535897931 * (e_t54_re * 0.0 +
    f_t48_re * d1) * 0.3048478988638153) + (j_re * 71.101550755408823 + i_im *
    0.0)) + 3.1415926535897931 * (t81 * 0.0 + t70 * d1) * 0.914543696591446)) +
    ((i_re * -213.3046522662265 + j_im * -0.0) - (b_t93_re_tmp * 0.0 +
      t73_re_tmp_tmp_tmp * d1) * 16583.452071232379);
  d_t54_im_tmp = t104_re_tmp * t243_re - t104_im_tmp * t243_im;
  t111 = t104_re_tmp * t243_im + t104_im_tmp * t243_re;
  t68_re = j_t68_re_tmp * t249_re - g_t68_im_tmp * t249_im;
  t68_im = j_t68_re_tmp * t249_im + g_t68_im_tmp * t249_re;
  if (t68_im == 0.0) {
    t68_re /= 2.0;
    t68_im = 0.0;
  } else if (t68_re == 0.0) {
    t68_re = 0.0;
    t68_im /= 2.0;
  } else {
    t68_re /= 2.0;
    t68_im /= 2.0;
  }

  t68_re_tmp = i_t68_re_tmp * t129.re;
  t68_im_tmp = i_t68_re_tmp * t129.im;
  i_t68_re_tmp = t68_re_tmp * d1 - t68_im_tmp * 0.0;
  t68_im_tmp = t68_re_tmp * 0.0 + t68_im_tmp * d1;
  t78 = i_t68_re_tmp * t183.re - t68_im_tmp * t183.im;
  b_t44_re_tmp_tmp = i_t68_re_tmp * t183.im + t68_im_tmp * t183.re;
  if (b_t44_re_tmp_tmp == 0.0) {
    t78 /= 4.0;
    b_t44_re_tmp_tmp = 0.0;
  } else if (t78 == 0.0) {
    t78 = 0.0;
    b_t44_re_tmp_tmp /= 4.0;
  } else {
    t78 /= 4.0;
    b_t44_re_tmp_tmp /= 4.0;
  }

  t68_re_tmp = t68 * t113_re;
  i_t68_im_tmp = t68 * t113_im;
  k_t68_re_tmp = t119 * (t68_re_tmp * t116_re - i_t68_im_tmp * t116_im);
  i_t68_im_tmp = t119 * (t68_re_tmp * t116_im + i_t68_im_tmp * t116_re);
  i_t48_re = k_t68_re_tmp * d1 - i_t68_im_tmp * 0.0;
  t98 = k_t68_re_tmp * 0.0 + i_t68_im_tmp * d1;
  t81 = i_t48_re * t183.re - t98 * t183.im;
  t98 = i_t48_re * t183.im + t98 * t183.re;
  if (t98 == 0.0) {
    i_t48_re = t81 / 4.0;
    t98 = 0.0;
  } else if (t81 == 0.0) {
    i_t48_re = 0.0;
    t98 /= 4.0;
  } else {
    i_t48_re = t81 / 4.0;
    t98 /= 4.0;
  }

  t68_re_tmp = g_t68_re_tmp * d - e_t68_im_tmp * 0.0;
  t41_tmp = g_t68_re_tmp * 0.0 + e_t68_im_tmp * d;
  t103 = h_t68_re_tmp * d1 - f_t68_im_tmp * 0.0;
  f_t68_im_tmp = h_t68_re_tmp * 0.0 + f_t68_im_tmp * d1;
  t58_im = ((((((d * t209_re - 0.0 * t209_im) + t68_re) + t78) - i_t48_re) -
             3.1415926535897931 * (t68_re_tmp * t183.re - t41_tmp * t183.im) *
             0.2857142857142857) + 3.1415926535897931 * (t103 * t223_re -
             f_t68_im_tmp * t223_im) * 0.5714285714285714) - 3.1415926535897931 *
    (h_t42_re_tmp * d - d_t42_im_tmp * 0.0) * 0.2857142857142857;
  e_t48_re_tmp = ((((((d * t209_im + 0.0 * t209_re) + t68_im) + b_t44_re_tmp_tmp)
                    - t98) - 3.1415926535897931 * (t68_re_tmp * t183.im +
    t41_tmp * t183.re) * 0.2857142857142857) + 3.1415926535897931 * (t103 *
    t223_im + f_t68_im_tmp * t223_re) * 0.5714285714285714) - 3.1415926535897931
    * (h_t42_re_tmp * 0.0 + d_t42_im_tmp * d) * 0.2857142857142857;
  h_t68_re_tmp = f_t68_re_tmp * t130_re;
  b_t44_re_tmp_tmp_tmp = f_t68_re_tmp * t130_im;
  t42_re_tmp_tmp = t42 * t68;
  c_t42_re_tmp_tmp = t42_re_tmp_tmp * t82;
  t42_re_tmp = c_t42_re_tmp_tmp * t94_tmp * t119;
  f_t68_re_tmp = t120 * (b_t68_re_tmp * t117_re - h_t68_im_tmp * t117_im);
  h_t68_im_tmp = t120 * (b_t68_re_tmp * t117_im + h_t68_im_tmp * t117_re);
  t54_re_tmp_tmp = t54_re_tmp_tmp_tmp * t68 * t69;
  t54_re_tmp = t54_re_tmp_tmp * t94_tmp * t119;
  b_t48_re = t54_re_tmp * t128.re * 3.1415926535897931;
  ai = t54_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t54_re = b_t48_re / 7.0;
    t56_re = 0.0;
  } else if (b_t48_re == 0.0) {
    t54_re = 0.0;
    t56_re = ai / 7.0;
  } else {
    t54_re = b_t48_re / 7.0;
    t56_re = ai / 7.0;
  }

  t42_re_tmp_tmp = t42_re_tmp_tmp * t69 * t83;
  b_t42_re_tmp = t42_re_tmp_tmp * t94_tmp * t119;
  b_t48_re = b_t42_re_tmp * t128.re * 3.1415926535897931;
  ai = b_t42_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t42_re = b_t48_re / 7.0;
    t42_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t42_re = 0.0;
    t42_im = ai / 7.0;
  } else {
    t42_re = b_t48_re / 7.0;
    t42_im = ai / 7.0;
  }

  b_t68_re_tmp = t121 * t68_re_tmp_tmp;
  t49_im_tmp = t121 * t68_im_tmp_tmp;
  t68_re = b_t68_re_tmp * t128.re - t49_im_tmp * t128.im;
  t68_im = b_t68_re_tmp * t128.im + t49_im_tmp * t128.re;
  t48_re_tmp_tmp = t48_re_tmp_tmp_tmp * t68 * t69;
  t48_re_tmp = t48_re_tmp_tmp * t95 * t119;
  b_t48_re = t48_re_tmp * t128.re * 3.1415926535897931;
  ai = t48_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t48_re = b_t48_re / 7.0;
    t48_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t48_re = 0.0;
    t48_im = ai / 7.0;
  } else {
    t48_re = b_t48_re / 7.0;
    t48_im = ai / 7.0;
  }

  b_t42_re = e_t42_re_tmp * t129.re;
  b_t42_im = e_t42_re_tmp * t129.im;
  c_t42_re = f_t42_re_tmp_tmp * t113_re;
  c_t42_im = f_t42_re_tmp_tmp * t113_im;
  d_t42_re = t120 * (c_t42_re * t116_re - c_t42_im * t116_im);
  c_t42_im = t120 * (c_t42_re * t116_im + c_t42_im * t116_re);
  b_t42_re_tmp = f_t42_re_tmp_tmp * t93;
  c_t42_re = b_t42_re_tmp * t113_re;
  d_t42_im = b_t42_re_tmp * t113_im;
  e_t42_re = t120 * (c_t42_re * t117_re - d_t42_im * t117_im);
  d_t42_im = t120 * (c_t42_re * t117_im + d_t42_im * t117_re);
  c_t42_re = 3.1415926535897931 * (e_t42_re * t183.re - d_t42_im * t183.im);
  d_t42_im = 3.1415926535897931 * (e_t42_re * t183.im + d_t42_im * t183.re);
  e_t42_re = b_t42_re_tmp * t114_re;
  t73_im = b_t42_re_tmp * t114_im;
  f_t42_re = t120 * (e_t42_re * t116_re - t73_im * t116_im);
  t73_im = t120 * (e_t42_re * t116_im + t73_im * t116_re);
  e_t42_re = 3.1415926535897931 * (f_t42_re * t183.re - t73_im * t183.im);
  t73_im = 3.1415926535897931 * (f_t42_re * t183.im + t73_im * t183.re);
  t78 = (((((((((((3.1415926535897931 * (h_t68_re_tmp * t187_re -
    b_t44_re_tmp_tmp_tmp * t187_im) * 0.42857142857142855 - 3.1415926535897931 *
                   (g_t68_re_tmp * t241_re - e_t68_im_tmp * t241_im) *
                   0.2857142857142857) + 3.1415926535897931 * (t42_re_tmp *
    t128.re) * 0.5714285714285714) + 3.1415926535897931 * (f_t68_re_tmp *
    t187_re - h_t68_im_tmp * t187_im) * 0.2857142857142857) + 3.1415926535897931
                * (c_t68_re_tmp * t241_re - b_t68_im_tmp * t241_im) *
                0.5714285714285714) + t54_re) - t42_re) - 3.1415926535897931 *
             (t68_re * t187_re - t68_im * t187_im) * 0.8571428571428571) -
            t48_re) + 3.1415926535897931 * (b_t42_re * t183.re - b_t42_im *
            t183.im) * 0.2857142857142857) - 3.1415926535897931 * (d_t42_re *
           t183.re - c_t42_im * t183.im) * 0.5714285714285714) + (c_t42_re * 0.0
          - d_t42_im * 0.2857142857142857)) + (e_t42_re * 0.0 - t73_im *
    0.2857142857142857);
  t68_im = (((((((((((3.1415926535897931 * (h_t68_re_tmp * t187_im +
    b_t44_re_tmp_tmp_tmp * t187_re) * 0.42857142857142855 - 3.1415926535897931 *
                      (g_t68_re_tmp * t241_im + e_t68_im_tmp * t241_re) *
                      0.2857142857142857) + 3.1415926535897931 * (t42_re_tmp *
    t128.im) * 0.5714285714285714) + 3.1415926535897931 * (f_t68_re_tmp *
    t187_im + h_t68_im_tmp * t187_re) * 0.2857142857142857) + 3.1415926535897931
                   * (c_t68_re_tmp * t241_im + b_t68_im_tmp * t241_re) *
                   0.5714285714285714) + t56_re) - t42_im) - 3.1415926535897931 *
                (t68_re * t187_im + t68_im * t187_re) * 0.8571428571428571) -
               t48_im) + 3.1415926535897931 * (b_t42_re * t183.im + b_t42_im *
    t183.re) * 0.2857142857142857) - 3.1415926535897931 * (d_t42_re * t183.im +
              c_t42_im * t183.re) * 0.5714285714285714) + (c_t42_re *
             0.2857142857142857 + d_t42_im * 0.0)) + (e_t42_re *
    0.2857142857142857 + t73_im * 0.0);
  t42_re = f_t42_re_tmp * t222_re - b_t42_im_tmp * t222_im;
  t42_im = f_t42_re_tmp * t222_im + b_t42_im_tmp * t222_re;
  t44_re = b_t44_re_tmp * t223_re - t44_im_tmp * t223_im;
  t44_im = b_t44_re_tmp * t223_im + t44_im_tmp * t223_re;
  b_t42_re = 3.1415926535897931 * (p_t42_re_tmp * t243_re - k_t42_im_tmp *
    t243_im);
  b_t42_im = 3.1415926535897931 * (p_t42_re_tmp * t243_im + k_t42_im_tmp *
    t243_re);
  b_t44_re = 3.1415926535897931 * (d_t44_re_tmp * t254_re - d_t44_im_tmp *
    t254_im);
  b_t44_im = 3.1415926535897931 * (d_t44_re_tmp * t254_im + d_t44_im_tmp *
    t254_re);
  c_t42_re = g_t42_re_tmp * t222_re - c_t42_im_tmp * t222_im;
  c_t42_im = g_t42_re_tmp * t222_im + c_t42_im_tmp * t222_re;
  d_t42_re = 3.1415926535897931 * (c_t42_re * t223_re - c_t42_im * t223_im);
  c_t42_im = 3.1415926535897931 * (c_t42_re * t223_im + c_t42_im * t223_re);
  d_t56_re_tmp = c_t44_re_tmp * t223_re - b_t44_im_tmp * t223_im;
  d_t50_re_tmp = c_t44_re_tmp * t223_im + b_t44_im_tmp * t223_re;
  c_t58_re_tmp = 3.1415926535897931 * (d_t56_re_tmp * t249_re - d_t50_re_tmp *
    t249_im);
  d_t50_re_tmp = 3.1415926535897931 * (d_t56_re_tmp * t249_im + d_t50_re_tmp *
    t249_re);
  t42_re_tmp_tmp_tmp = b_t42_re_tmp_tmp_tmp_tmp * t68;
  d_t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t82 * t88;
  t42_re_tmp = d_t42_re_tmp_tmp * t106;
  c_t42_re = t42_re_tmp * t123_re;
  d_t42_im = t42_re_tmp * t123_im;
  t42_re_tmp = d_t42_re_tmp_tmp * t100;
  e_t42_re = t42_re_tmp * t123_re;
  t73_im = t42_re_tmp * t123_im;
  f_t42_re = 3.1415926535897931 * (e_t42_re * d - t73_im * 0.0);
  t73_im = 3.1415926535897931 * (e_t42_re * 0.0 + t73_im * d);
  b_re_tmp_tmp = c_re_tmp_tmp_tmp * t68;
  re_tmp = b_re_tmp_tmp * t82 * t88 * t105;
  re = re_tmp * t123_re;
  im = re_tmp * t123_im;
  b_t44_re_tmp_tmp = t44 * t54_tmp * t54_tmp * t68 * t69 * t88;
  f_t44_re_tmp = b_t44_re_tmp_tmp * t106;
  d_t56_re_tmp = f_t44_re_tmp * t123_re;
  b_t93_re = f_t44_re_tmp * t123_im;
  d_t42_re_tmp_tmp = t42_re_tmp_tmp_tmp * t69 * t83 * t88;
  t42_re_tmp = d_t42_re_tmp_tmp * t106;
  e_t42_re = t42_re_tmp * t123_re;
  e_t42_im = t42_re_tmp * t123_im;
  f_t44_re_tmp = b_t44_re_tmp_tmp * t100;
  t61 = f_t44_re_tmp * t123_re;
  g_t42_re_tmp_tmp = f_t44_re_tmp * t123_im;
  t55 = 3.1415926535897931 * (t61 * d - g_t42_re_tmp_tmp * 0.0);
  g_t42_re_tmp_tmp = 3.1415926535897931 * (t61 * 0.0 + g_t42_re_tmp_tmp * d);
  t42_re_tmp = d_t42_re_tmp_tmp * t100;
  g_t42_re = t42_re_tmp * t123_re;
  t93_re_tmp_tmp = t42_re_tmp * t123_im;
  h_t42_re = 3.1415926535897931 * (g_t42_re * d - t93_re_tmp_tmp * 0.0);
  t93_re_tmp_tmp = 3.1415926535897931 * (g_t42_re * 0.0 + t93_re_tmp_tmp * d);
  t42_re_tmp = c_t42_re_tmp_tmp * t93 * t105 * t119;
  g_t42_re = t42_re_tmp * t123_re;
  b_t48_re_tmp_tmp = t42_re_tmp * t123_im;
  i_t42_re = g_t42_re * t128.re - b_t48_re_tmp_tmp * t128.im;
  b_t48_re_tmp_tmp = g_t42_re * t128.im + b_t48_re_tmp_tmp * t128.re;
  re_tmp = 9.869604401089358 * t44 * t54_tmp * t54_tmp * t68 * t69 * t88 * t105;
  b_re = re_tmp * t123_re;
  b_im = re_tmp * t123_im;
  re_tmp = b_re_tmp_tmp * t69 * t83 * t88 * t105;
  c_re = re_tmp * t123_re;
  c_im = re_tmp * t123_im;
  b_t44_re_tmp_tmp = t44 * t48 * t54_tmp * t54_tmp * t68 * t69 * t88;
  f_t44_re_tmp = b_t44_re_tmp_tmp * t107;
  t61 = f_t44_re_tmp * t123_re;
  d_t48_re_tmp = f_t44_re_tmp * t123_im;
  b_re_tmp_tmp = 9.869604401089358 * t42 * t68;
  re_tmp = b_re_tmp_tmp * t82 * t99 * t93 * t119;
  d_re = re_tmp * t123_re;
  d_im = re_tmp * t123_im;
  e_re = d_re * t128.re - d_im * t128.im;
  d_im = d_re * t128.im + d_im * t128.re;
  d_re = e_re * d1 - d_im * 0.0;
  d_im = e_re * 0.0 + d_im * d1;
  g_t42_re = t_t42_re_tmp * d1 - n_t42_im_tmp * 0.0;
  b_t54_re_tmp_tmp = t_t42_re_tmp * 0.0 + n_t42_im_tmp * d1;
  f_t44_re_tmp = b_t44_re_tmp_tmp * t101;
  ab_t42_re_tmp = f_t44_re_tmp * t123_re;
  e_t42_re_tmp_tmp = f_t44_re_tmp * t123_im;
  c_t48_re_tmp = 3.1415926535897931 * (ab_t42_re_tmp * d - e_t42_re_tmp_tmp *
    0.0);
  e_t42_re_tmp_tmp = 3.1415926535897931 * (ab_t42_re_tmp * 0.0 +
    e_t42_re_tmp_tmp * d);
  t54_re_tmp = t54_re_tmp_tmp * t93 * t105 * t119;
  t54_re = t54_re_tmp * t123_re;
  t56_re = t54_re_tmp * t123_im;
  b_t54_re = t54_re * t128.re - t56_re * t128.im;
  t56_re = t54_re * t128.im + t56_re * t128.re;
  t55_re_tmp_tmp = u_t42_re_tmp * d1 - o_t42_im_tmp * 0.0;
  b_t55_re_tmp_tmp = u_t42_re_tmp * 0.0 + o_t42_im_tmp * d1;
  t55_re_tmp = 3.1415926535897931 * (t55_re_tmp_tmp * t223_re - b_t55_re_tmp_tmp
    * t223_im);
  b_t55_re_tmp_tmp = 3.1415926535897931 * (t55_re_tmp_tmp * t223_im +
    b_t55_re_tmp_tmp * t223_re);
  t42_re_tmp = t42_re_tmp_tmp * t93 * t105 * t119;
  t55_re_tmp_tmp = t42_re_tmp * t123_re;
  ai = t42_re_tmp * t123_im;
  d_t43_re_tmp_tmp = t55_re_tmp_tmp * t128.re - ai * t128.im;
  ai = t55_re_tmp_tmp * t128.im + ai * t128.re;
  re_tmp = 31.006276680299816 * t44 * t48 * t54_tmp * t54_tmp * t68 * t69 * t88 *
    t100;
  e_re = re_tmp * t123_re;
  e_im = re_tmp * t123_im;
  f_re = e_re * d - e_im * 0.0;
  e_im = e_re * 0.0 + e_im * d;
  re_tmp = 9.869604401089358 * t44 * t48 * t54_tmp * t54_tmp * t68 * t69 * t88 *
    t106;
  e_re = re_tmp * t123_re;
  f_im = re_tmp * t123_im;
  re_tmp = 9.869604401089358 * t54_tmp * t54_tmp * t68 * t69 * t99 * t93 * t119;
  g_re = re_tmp * t123_re;
  g_im = re_tmp * t123_im;
  h_re = g_re * t128.re - g_im * t128.im;
  g_im = g_re * t128.im + g_im * t128.re;
  g_re = h_re * d1 - g_im * 0.0;
  g_im = h_re * 0.0 + g_im * d1;
  re_tmp = b_re_tmp_tmp * t69 * t83 * t99 * t93 * t119;
  h_re = re_tmp * t123_re;
  h_im = re_tmp * t123_im;
  i_re = h_re * t128.re - h_im * t128.im;
  h_im = h_re * t128.im + h_im * t128.re;
  h_re = i_re * d1 - h_im * 0.0;
  h_im = i_re * 0.0 + h_im * d1;
  i_re = g_re_tmp * d1 - e_im_tmp * 0.0;
  i_im = g_re_tmp * 0.0 + e_im_tmp * d1;
  j_re = d_re_tmp * t249_re - g_im_tmp * t249_im;
  j_im = d_re_tmp * t249_im + g_im_tmp * t249_re;
  t48_re_tmp = t48_re_tmp_tmp * t94_tmp * t105 * t119;
  t48_re = t48_re_tmp * t123_re;
  t48_im = t48_re_tmp * t123_im;
  b_t48_re = t48_re * t128.re - t48_im * t128.im;
  t48_im = t48_re * t128.im + t48_im * t128.re;
  t48_re_tmp = t48_re_tmp_tmp * t93 * t106 * t119;
  t48_re = t48_re_tmp * t123_re;
  t50_re_tmp_tmp = t48_re_tmp * t123_im;
  c_t48_re = t48_re * t128.re - t50_re_tmp_tmp * t128.im;
  t50_re_tmp_tmp = t48_re * t128.im + t50_re_tmp_tmp * t128.re;
  t55_re_tmp_tmp = v_t42_re_tmp * t129.re - p_t42_im_tmp * t129.im;
  c_t43_re_tmp_tmp = v_t42_re_tmp * t129.im + p_t42_im_tmp * t129.re;
  c_t55_re = t55_re_tmp_tmp * d1 - c_t43_re_tmp_tmp * 0.0;
  c_t43_re_tmp_tmp = t55_re_tmp_tmp * 0.0 + c_t43_re_tmp_tmp * d1;
  re_tmp_tmp = re_tmp_tmp * t68 * t69;
  re_tmp = re_tmp_tmp * t94_tmp * t99 * t119;
  t93_im_tmp = re_tmp * t123_re;
  t73_re_tmp_tmp_tmp = re_tmp * t123_im;
  b_t93_re_tmp = t93_im_tmp * t128.re - t73_re_tmp_tmp_tmp * t128.im;
  t73_re_tmp_tmp_tmp = t93_im_tmp * t128.im + t73_re_tmp_tmp_tmp * t128.re;
  t93_im_tmp = b_t93_re_tmp * d1 - t73_re_tmp_tmp_tmp * 0.0;
  t73_re_tmp_tmp_tmp = b_t93_re_tmp * 0.0 + t73_re_tmp_tmp_tmp * d1;
  re_tmp = re_tmp_tmp * t100 * t93 * t119;
  b_t93_re_tmp = re_tmp * t123_re;
  t54_re_tmp_tmp = re_tmp * t123_im;
  re_tmp_tmp_tmp = b_t93_re_tmp * t128.re - t54_re_tmp_tmp * t128.im;
  t54_re_tmp_tmp = b_t93_re_tmp * t128.im + t54_re_tmp_tmp * t128.re;
  b_t93_re_tmp = re_tmp_tmp_tmp * d1 - t54_re_tmp_tmp * 0.0;
  t54_re_tmp_tmp = re_tmp_tmp_tmp * 0.0 + t54_re_tmp_tmp * d1;
  re_tmp = 31.006276680299816 * t48 * t54_tmp * t54_tmp * t68 * t69 * t93 * t105
    * t119;
  re_tmp_tmp_tmp = re_tmp * t123_re;
  e_t54_re_tmp = re_tmp * t123_im;
  d_t54_re_tmp = re_tmp_tmp_tmp * t128.re - e_t54_re_tmp * t128.im;
  e_t54_re_tmp = re_tmp_tmp_tmp * t128.im + e_t54_re_tmp * t128.re;
  re_tmp_tmp_tmp = h_re_tmp * t129.re - f_im_tmp * t129.im;
  f_t48_re_tmp = h_re_tmp * t129.im + f_im_tmp * t129.re;
  t56_re_tmp_tmp = re_tmp_tmp_tmp * d1 - f_t48_re_tmp * 0.0;
  f_t48_re_tmp = re_tmp_tmp_tmp * 0.0 + f_t48_re_tmp * d1;
  re_tmp_tmp_tmp = t56_re_tmp_tmp * t183.re - f_t48_re_tmp * t183.im;
  f_t48_re_tmp = t56_re_tmp_tmp * t183.im + f_t48_re_tmp * t183.re;
  t55_re_tmp_tmp = b_t42_re_tmp_tmp * t113_re;
  t81 = b_t42_re_tmp_tmp * t113_im;
  t98 = t120 * (t55_re_tmp_tmp * t116_re - t81 * t116_im);
  t81 = t120 * (t55_re_tmp_tmp * t116_im + t81 * t116_re);
  t55_re_tmp_tmp = t98 * t123_re - t81 * t123_im;
  t81 = t98 * t123_im + t81 * t123_re;
  t98 = t55_re_tmp_tmp * d1 - t81 * 0.0;
  t81 = t55_re_tmp_tmp * 0.0 + t81 * d1;
  t56_re_tmp_tmp = e_re_tmp_tmp * t113_re;
  t115 = e_re_tmp_tmp * t113_im;
  c_t54_re_tmp = t120 * (t56_re_tmp_tmp * t116_re - t115 * t116_im);
  t115 = t120 * (t56_re_tmp_tmp * t116_im + t115 * t116_re);
  t56_re_tmp_tmp = c_t54_re_tmp * t123_re - t115 * t123_im;
  t115 = c_t54_re_tmp * t123_im + t115 * t123_re;
  c_t54_re_tmp = t56_re_tmp_tmp * d1 - t115 * 0.0;
  t115 = t56_re_tmp_tmp * 0.0 + t115 * d1;
  t56_re_tmp_tmp = c_t54_re_tmp * t183.re - t115 * t183.im;
  t115 = c_t54_re_tmp * t183.im + t115 * t183.re;
  Mzxx->re = (((((((((((((((d_t104_re_tmp * t254_re - d_t104_im_tmp * t254_im) *
    -1.0669676460233539 + (d_t54_im_tmp * t249_re - t111 * t249_im) *
    1.0669676460233539) - (c_t104_re_tmp * t58_im - c_t104_im_tmp * e_t48_re_tmp)
    * 1.0669676460233539) + (b_t104_re_tmp * t78 - b_t104_im_tmp * t68_im) *
    1.0669676460233539) + ((((k_t42_re_tmp * t243_re - j_t42_im_tmp * t243_im) *
    -0.53348382301167685 + (t44_re_tmp * t254_re - c_t44_im_tmp * t254_im) *
    0.53348382301167685) + (t42_re * t223_re - t42_im * t223_im) *
    0.53348382301167685) + (t44_re * t249_re - t44_im * t249_im) *
    0.53348382301167685)) + ((((b_t42_re * 0.0 - b_t42_im * 124.4277138219654) -
    (b_t44_re * 0.0 - b_t44_im * 124.4277138219654)) - (d_t42_re * 0.0 -
    c_t42_im * 124.4277138219654)) - (c_t58_re_tmp * 0.0 - d_t50_re_tmp *
    124.4277138219654))) + ((((c_t42_re * d - d_t42_im * 0.0) *
    1.6004514690350311 - (f_t42_re * 0.0 - t73_im * 373.28314146589628)) - (re *
    d - im * 0.0) * 29021.041124656658) + (d_t56_re_tmp * d - b_t93_re * 0.0) *
    0.40011286725875761)) + ((((e_t42_re * d - e_t42_im * 0.0) *
    -0.40011286725875761 - (t55 * 0.0 - g_t42_re_tmp_tmp * 93.320785366474084))
    + (h_t42_re * 0.0 - t93_re_tmp_tmp * 93.320785366474084)) -
    3.1415926535897931 * (i_t42_re * d1 - b_t48_re_tmp_tmp * 0.0) *
    0.60969579772763072)) + (((b_re * d - b_im * 0.0) * -7255.2602811641664 +
    (c_re * d - c_im * 0.0) * 7255.2602811641664) - (t61 * d - d_t48_re_tmp *
    0.0) * 2.0005643362937882)) + ((((d_re * 0.0 - d_im * 142.2031015108177) -
    (g_t42_re * t223_re - b_t54_re_tmp_tmp * t223_im) * 1.6004514690350311) +
    (c_t48_re_tmp * 0.0 - e_t42_re_tmp_tmp * 466.60392683237041)) -
    3.1415926535897931 * (b_t54_re * d1 - t56_re * 0.0) * 0.15242394943190771))
                  + (((t55_re_tmp * 0.0 - b_t55_re_tmp_tmp * 373.28314146589628)
                      + 3.1415926535897931 * (d_t43_re_tmp_tmp * d1 - ai * 0.0) *
                      0.15242394943190771) - 3.1415926535897931 * (q_t42_re_tmp *
    t249_re - q_t42_im_tmp * t249_im) * 0.3048478988638153)) + (((f_re * -0.0 -
    e_im * -1.6921889868604471E+6) + (e_re * d - f_im * 0.0) * 43531.561686985)
    + (g_re * 0.0 - g_im * 35.550775377704412))) + (((h_re * -0.0 - h_im *
    -35.550775377704412) + (i_re * t223_re - i_im * t223_im) *
    29021.041124656658) + (j_re * 0.0 - j_im * 71.101550755408823))) +
               (((3.1415926535897931 * (b_t48_re * d1 - t48_im * 0.0) *
                  -0.15242394943190771 + 3.1415926535897931 * (c_t48_re * d1 -
    t50_re_tmp_tmp * 0.0) * 0.457271848295723) - 3.1415926535897931 * (c_t55_re *
    t183.re - c_t43_re_tmp_tmp * t183.im) * 0.15242394943190771) + (t93_im_tmp *
    0.0 - t73_re_tmp_tmp_tmp * 35.550775377704412))) + (((b_t93_re_tmp * -0.0 -
    t54_re_tmp_tmp * -106.65232613311321) - (d_t54_re_tmp * d1 - e_t54_re_tmp *
    0.0) * 8291.72603561619) + (re_tmp_tmp_tmp * 0.0 - f_t48_re_tmp *
    35.550775377704412))) + (3.1415926535897931 * (t98 * t183.re - t81 * t183.im)
    * 0.3048478988638153 - (t56_re_tmp_tmp * 0.0 - t115 * 71.101550755408823));
  Mzxx->im = (((((((((((((((d_t104_re_tmp * t254_im + d_t104_im_tmp * t254_re) *
    -1.0669676460233539 + (d_t54_im_tmp * t249_im + t111 * t249_re) *
    1.0669676460233539) - (c_t104_re_tmp * e_t48_re_tmp + c_t104_im_tmp * t58_im)
    * 1.0669676460233539) + (b_t104_re_tmp * t68_im + b_t104_im_tmp * t78) *
    1.0669676460233539) + ((((k_t42_re_tmp * t243_im + j_t42_im_tmp * t243_re) *
    -0.53348382301167685 + (t44_re_tmp * t254_im + c_t44_im_tmp * t254_re) *
    0.53348382301167685) + (t42_re * t223_im + t42_im * t223_re) *
    0.53348382301167685) + (t44_re * t249_im + t44_im * t249_re) *
    0.53348382301167685)) + ((((b_t42_re * 124.4277138219654 + b_t42_im * 0.0) -
    (b_t44_re * 124.4277138219654 + b_t44_im * 0.0)) - (d_t42_re *
    124.4277138219654 + c_t42_im * 0.0)) - (c_t58_re_tmp * 124.4277138219654 +
    d_t50_re_tmp * 0.0))) + ((((c_t42_re * 0.0 + d_t42_im * d) *
    1.6004514690350311 - (f_t42_re * 373.28314146589628 + t73_im * 0.0)) - (re *
    0.0 + im * d) * 29021.041124656658) + (d_t56_re_tmp * 0.0 + b_t93_re * d) *
    0.40011286725875761)) + ((((e_t42_re * 0.0 + e_t42_im * d) *
    -0.40011286725875761 - (t55 * 93.320785366474084 + g_t42_re_tmp_tmp * 0.0))
    + (h_t42_re * 93.320785366474084 + t93_re_tmp_tmp * 0.0)) -
    3.1415926535897931 * (i_t42_re * 0.0 + b_t48_re_tmp_tmp * d1) *
    0.60969579772763072)) + (((b_re * 0.0 + b_im * d) * -7255.2602811641664 +
    (c_re * 0.0 + c_im * d) * 7255.2602811641664) - (t61 * 0.0 + d_t48_re_tmp *
    d) * 2.0005643362937882)) + ((((d_re * 142.2031015108177 + d_im * 0.0) -
    (g_t42_re * t223_im + b_t54_re_tmp_tmp * t223_re) * 1.6004514690350311) +
    (c_t48_re_tmp * 466.60392683237041 + e_t42_re_tmp_tmp * 0.0)) -
    3.1415926535897931 * (b_t54_re * 0.0 + t56_re * d1) * 0.15242394943190771))
                  + (((t55_re_tmp * 373.28314146589628 + b_t55_re_tmp_tmp * 0.0)
                      + 3.1415926535897931 * (d_t43_re_tmp_tmp * 0.0 + ai * d1) *
                      0.15242394943190771) - 3.1415926535897931 * (q_t42_re_tmp *
    t249_im + q_t42_im_tmp * t249_re) * 0.3048478988638153)) + (((f_re *
    -1.6921889868604471E+6 + e_im * -0.0) + (e_re * 0.0 + f_im * d) *
    43531.561686985) + (g_re * 35.550775377704412 + g_im * 0.0))) + (((h_re *
    -35.550775377704412 + h_im * -0.0) + (i_re * t223_im + i_im * t223_re) *
    29021.041124656658) + (j_re * 71.101550755408823 + j_im * 0.0))) +
               (((3.1415926535897931 * (b_t48_re * 0.0 + t48_im * d1) *
                  -0.15242394943190771 + 3.1415926535897931 * (c_t48_re * 0.0 +
    t50_re_tmp_tmp * d1) * 0.457271848295723) - 3.1415926535897931 * (c_t55_re *
    t183.im + c_t43_re_tmp_tmp * t183.re) * 0.15242394943190771) + (t93_im_tmp *
    35.550775377704412 + t73_re_tmp_tmp_tmp * 0.0))) + (((b_t93_re_tmp *
    -106.65232613311321 + t54_re_tmp_tmp * -0.0) - (d_t54_re_tmp * 0.0 +
    e_t54_re_tmp * d1) * 8291.72603561619) + (re_tmp_tmp_tmp *
    35.550775377704412 + f_t48_re_tmp * 0.0))) + (3.1415926535897931 * (t98 *
    t183.im + t81 * t183.re) * 0.3048478988638153 - (t56_re_tmp_tmp *
    71.101550755408823 + t115 * 0.0));
  d_t54_im_tmp = t104_re_tmp * t244_re - t104_im_tmp * t244_im;
  t111 = t104_re_tmp * t244_im + t104_im_tmp * t244_re;
  t68_re = j_t68_re_tmp * t250_re - g_t68_im_tmp * t250_im;
  t68_im = j_t68_re_tmp * t250_im + g_t68_im_tmp * t250_re;
  if (t68_im == 0.0) {
    t68_re /= 2.0;
    t68_im = 0.0;
  } else if (t68_re == 0.0) {
    t68_re = 0.0;
    t68_im /= 2.0;
  } else {
    t68_re /= 2.0;
    t68_im /= 2.0;
  }

  t78 = i_t68_re_tmp * t184.re - t68_im_tmp * t184.im;
  b_t44_re_tmp_tmp = i_t68_re_tmp * t184.im + t68_im_tmp * t184.re;
  if (b_t44_re_tmp_tmp == 0.0) {
    t78 /= 4.0;
    b_t44_re_tmp_tmp = 0.0;
  } else if (t78 == 0.0) {
    t78 = 0.0;
    b_t44_re_tmp_tmp /= 4.0;
  } else {
    t78 /= 4.0;
    b_t44_re_tmp_tmp /= 4.0;
  }

  i_t48_re = k_t68_re_tmp * d1 - i_t68_im_tmp * 0.0;
  t98 = k_t68_re_tmp * 0.0 + i_t68_im_tmp * d1;
  t81 = i_t48_re * t184.re - t98 * t184.im;
  t98 = i_t48_re * t184.im + t98 * t184.re;
  if (t98 == 0.0) {
    i_t48_re = t81 / 4.0;
    t98 = 0.0;
  } else if (t81 == 0.0) {
    i_t48_re = 0.0;
    t98 /= 4.0;
  } else {
    i_t48_re = t81 / 4.0;
    t98 /= 4.0;
  }

  t58_im = ((((((d * t210_re - 0.0 * t210_im) + t68_re) + t78) - i_t48_re) -
             3.1415926535897931 * (t68_re_tmp * t184.re - t41_tmp * t184.im) *
             0.2857142857142857) + 3.1415926535897931 * (t103 * t224_re -
             f_t68_im_tmp * t224_im) * 0.5714285714285714) - 3.1415926535897931 *
    (g_t43_re_tmp * d - d_t43_im_tmp * 0.0) * 0.2857142857142857;
  e_t48_re_tmp = ((((((d * t210_im + 0.0 * t210_re) + t68_im) + b_t44_re_tmp_tmp)
                    - t98) - 3.1415926535897931 * (t68_re_tmp * t184.im +
    t41_tmp * t184.re) * 0.2857142857142857) + 3.1415926535897931 * (t103 *
    t224_im + f_t68_im_tmp * t224_re) * 0.5714285714285714) - 3.1415926535897931
    * (g_t43_re_tmp * 0.0 + d_t43_im_tmp * d) * 0.2857142857142857;
  t43_re_tmp_tmp = t43 * t68;
  c_t43_re_tmp_tmp = t43_re_tmp_tmp * t85;
  t43_re_tmp = c_t43_re_tmp_tmp * t94_tmp * t119;
  t56_re_tmp_tmp = t56_re_tmp_tmp_tmp * t68 * t71;
  t56_re_tmp = t56_re_tmp_tmp * t94_tmp * t119;
  b_t48_re = t56_re_tmp * t128.re * 3.1415926535897931;
  ai = t56_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t56_re = b_t48_re / 7.0;
    t48_re_tmp_tmp = 0.0;
  } else if (b_t48_re == 0.0) {
    t56_re = 0.0;
    t48_re_tmp_tmp = ai / 7.0;
  } else {
    t56_re = b_t48_re / 7.0;
    t48_re_tmp_tmp = ai / 7.0;
  }

  t43_re_tmp_tmp = t43_re_tmp_tmp * t71 * t86;
  b_t43_re_tmp = t43_re_tmp_tmp * t94_tmp * t119;
  b_t48_re = b_t43_re_tmp * t128.re * 3.1415926535897931;
  ai = b_t43_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t43_re = b_t48_re / 7.0;
    t43_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t43_re = 0.0;
    t43_im = ai / 7.0;
  } else {
    t43_re = b_t48_re / 7.0;
    t43_im = ai / 7.0;
  }

  t68_re = b_t68_re_tmp * t128.re - t49_im_tmp * t128.im;
  t68_im = b_t68_re_tmp * t128.im + t49_im_tmp * t128.re;
  t50_re_tmp_tmp = t50_re_tmp_tmp_tmp * t68 * t71;
  t50_re_tmp = t50_re_tmp_tmp * t95 * t119;
  b_t48_re = t50_re_tmp * t128.re * 3.1415926535897931;
  ai = t50_re_tmp * t128.im * 3.1415926535897931;
  if (ai == 0.0) {
    t48_re = b_t48_re / 7.0;
    t50_im = 0.0;
  } else if (b_t48_re == 0.0) {
    t48_re = 0.0;
    t50_im = ai / 7.0;
  } else {
    t48_re = b_t48_re / 7.0;
    t50_im = ai / 7.0;
  }

  b_t43_re = d_t43_re_tmp * t129.re;
  b_t43_im = d_t43_re_tmp * t129.im;
  c_t43_re = e_t43_re_tmp_tmp * t113_re;
  c_t43_im = e_t43_re_tmp_tmp * t113_im;
  d_t43_re = t120 * (c_t43_re * t116_re - c_t43_im * t116_im);
  c_t43_im = t120 * (c_t43_re * t116_im + c_t43_im * t116_re);
  b_t43_re_tmp = e_t43_re_tmp_tmp * t93;
  c_t43_re = b_t43_re_tmp * t113_re;
  e_t55_re = b_t43_re_tmp * t113_im;
  e_t43_re = t120 * (c_t43_re * t117_re - e_t55_re * t117_im);
  e_t55_re = t120 * (c_t43_re * t117_im + e_t55_re * t117_re);
  c_t43_re = 3.1415926535897931 * (e_t43_re * t184.re - e_t55_re * t184.im);
  e_t55_re = 3.1415926535897931 * (e_t43_re * t184.im + e_t55_re * t184.re);
  e_t43_re = b_t43_re_tmp * t114_re;
  e_t54_im = b_t43_re_tmp * t114_im;
  b_t42_re_tmp_tmp_tmp = t120 * (e_t43_re * t116_re - e_t54_im * t116_im);
  e_t54_im = t120 * (e_t43_re * t116_im + e_t54_im * t116_re);
  e_t43_re = 3.1415926535897931 * (b_t42_re_tmp_tmp_tmp * t184.re - e_t54_im *
    t184.im);
  e_t54_im = 3.1415926535897931 * (b_t42_re_tmp_tmp_tmp * t184.im + e_t54_im *
    t184.re);
  t78 = (((((((((((3.1415926535897931 * (h_t68_re_tmp * t189_re -
    b_t44_re_tmp_tmp_tmp * t189_im) * 0.42857142857142855 - 3.1415926535897931 *
                   (g_t68_re_tmp * t242_re - e_t68_im_tmp * t242_im) *
                   0.2857142857142857) + 3.1415926535897931 * (t43_re_tmp *
    t128.re) * 0.5714285714285714) + 3.1415926535897931 * (f_t68_re_tmp *
    t189_re - h_t68_im_tmp * t189_im) * 0.2857142857142857) + 3.1415926535897931
                * (c_t68_re_tmp * t242_re - b_t68_im_tmp * t242_im) *
                0.5714285714285714) + t56_re) - t43_re) - 3.1415926535897931 *
             (t68_re * t189_re - t68_im * t189_im) * 0.8571428571428571) -
            t48_re) + 3.1415926535897931 * (b_t43_re * t184.re - b_t43_im *
            t184.im) * 0.2857142857142857) - 3.1415926535897931 * (d_t43_re *
           t184.re - c_t43_im * t184.im) * 0.5714285714285714) + (c_t43_re * 0.0
          - e_t55_re * 0.2857142857142857)) + (e_t43_re * 0.0 - e_t54_im *
    0.2857142857142857);
  t68_im = (((((((((((3.1415926535897931 * (h_t68_re_tmp * t189_im +
    b_t44_re_tmp_tmp_tmp * t189_re) * 0.42857142857142855 - 3.1415926535897931 *
                      (g_t68_re_tmp * t242_im + e_t68_im_tmp * t242_re) *
                      0.2857142857142857) + 3.1415926535897931 * (t43_re_tmp *
    t128.im) * 0.5714285714285714) + 3.1415926535897931 * (f_t68_re_tmp *
    t189_im + h_t68_im_tmp * t189_re) * 0.2857142857142857) + 3.1415926535897931
                   * (c_t68_re_tmp * t242_im + b_t68_im_tmp * t242_re) *
                   0.5714285714285714) + t48_re_tmp_tmp) - t43_im) -
                3.1415926535897931 * (t68_re * t189_im + t68_im * t189_re) *
                0.8571428571428571) - t50_im) + 3.1415926535897931 * (b_t43_re *
    t184.im + b_t43_im * t184.re) * 0.2857142857142857) - 3.1415926535897931 *
             (d_t43_re * t184.im + c_t43_im * t184.re) * 0.5714285714285714) +
            (c_t43_re * 0.2857142857142857 + e_t55_re * 0.0)) + (e_t43_re *
    0.2857142857142857 + e_t54_im * 0.0);
  t43_re = e_t43_re_tmp * t222_re - b_t43_im_tmp * t222_im;
  t43_im = e_t43_re_tmp * t222_im + b_t43_im_tmp * t222_re;
  t44_re = b_t44_re_tmp * t224_re - t44_im_tmp * t224_im;
  t44_im = b_t44_re_tmp * t224_im + t44_im_tmp * t224_re;
  b_t43_re = 3.1415926535897931 * (k_t43_re_tmp * t244_re - g_t43_im_tmp *
    t244_im);
  b_t43_im = 3.1415926535897931 * (k_t43_re_tmp * t244_im + g_t43_im_tmp *
    t244_re);
  b_t44_re = 3.1415926535897931 * (d_t44_re_tmp * t255_re - d_t44_im_tmp *
    t255_im);
  b_t44_im = 3.1415926535897931 * (d_t44_re_tmp * t255_im + d_t44_im_tmp *
    t255_re);
  c_t43_re = f_t43_re_tmp * t222_re - c_t43_im_tmp * t222_im;
  c_t43_im = f_t43_re_tmp * t222_im + c_t43_im_tmp * t222_re;
  d_t43_re = 3.1415926535897931 * (c_t43_re * t224_re - c_t43_im * t224_im);
  c_t43_im = 3.1415926535897931 * (c_t43_re * t224_im + c_t43_im * t224_re);
  d_t56_re_tmp = c_t44_re_tmp * t224_re - b_t44_im_tmp * t224_im;
  d_t50_re_tmp = c_t44_re_tmp * t224_im + b_t44_im_tmp * t224_re;
  c_t58_re_tmp = 3.1415926535897931 * (d_t56_re_tmp * t250_re - d_t50_re_tmp *
    t250_im);
  d_t50_re_tmp = 3.1415926535897931 * (d_t56_re_tmp * t250_im + d_t50_re_tmp *
    t250_re);
  t43_re_tmp_tmp_tmp = t43_re_tmp_tmp_tmp_tmp * t68;
  d_t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t85 * t88;
  t43_re_tmp = d_t43_re_tmp_tmp * t106;
  c_t43_re = t43_re_tmp * t123_re;
  e_t55_re = t43_re_tmp * t123_im;
  t43_re_tmp = d_t43_re_tmp_tmp * t100;
  e_t43_re = t43_re_tmp * t123_re;
  e_t54_im = t43_re_tmp * t123_im;
  b_t42_re_tmp_tmp_tmp = 3.1415926535897931 * (e_t43_re * d - e_t54_im * 0.0);
  e_t54_im = 3.1415926535897931 * (e_t43_re * 0.0 + e_t54_im * d);
  re_tmp_tmp = e_re_tmp_tmp_tmp * t68;
  re_tmp = re_tmp_tmp * t85 * t88 * t105;
  re = re_tmp * t123_re;
  im = re_tmp * t123_im;
  b_t44_re_tmp_tmp = t44 * t56_tmp * t56_tmp * t68 * t71 * t88;
  b_t44_re_tmp = b_t44_re_tmp_tmp * t106;
  d_t56_re_tmp = b_t44_re_tmp * t123_re;
  b_t93_re = b_t44_re_tmp * t123_im;
  d_t43_re_tmp_tmp = t43_re_tmp_tmp_tmp * t71 * t86 * t88;
  t43_re_tmp = d_t43_re_tmp_tmp * t106;
  e_t43_re = t43_re_tmp * t123_re;
  ai = t43_re_tmp * t123_im;
  b_t44_re_tmp = b_t44_re_tmp_tmp * t100;
  t61 = b_t44_re_tmp * t123_re;
  g_t42_re_tmp_tmp = b_t44_re_tmp * t123_im;
  t55 = 3.1415926535897931 * (t61 * d - g_t42_re_tmp_tmp * 0.0);
  g_t42_re_tmp_tmp = 3.1415926535897931 * (t61 * 0.0 + g_t42_re_tmp_tmp * d);
  t43_re_tmp = d_t43_re_tmp_tmp * t100;
  t73_re = t43_re_tmp * t123_re;
  i_t48_re = t43_re_tmp * t123_im;
  e_t42_im = 3.1415926535897931 * (t73_re * d - i_t48_re * 0.0);
  i_t48_re = 3.1415926535897931 * (t73_re * 0.0 + i_t48_re * d);
  t43_re_tmp = c_t43_re_tmp_tmp * t93 * t105 * t119;
  t73_re = t43_re_tmp * t123_re;
  f_t54_re = t43_re_tmp * t123_im;
  e_t54_re = t73_re * t128.re - f_t54_re * t128.im;
  f_t54_re = t73_re * t128.im + f_t54_re * t128.re;
  re_tmp = 9.869604401089358 * t44 * t56_tmp * t56_tmp * t68 * t71 * t88 * t105;
  b_re = re_tmp * t123_re;
  b_im = re_tmp * t123_im;
  re_tmp = re_tmp_tmp * t71 * t86 * t88 * t105;
  c_re = re_tmp * t123_re;
  c_im = re_tmp * t123_im;
  b_t44_re_tmp_tmp = t44 * t50 * t56_tmp * t56_tmp * t68 * t71 * t88;
  b_t44_re_tmp = b_t44_re_tmp_tmp * t107;
  t61 = b_t44_re_tmp * t123_re;
  d_t48_re_tmp = b_t44_re_tmp * t123_im;
  re_tmp_tmp = 9.869604401089358 * t43 * t68;
  re_tmp = re_tmp_tmp * t85 * t99 * t93 * t119;
  d_re = re_tmp * t123_re;
  d_im = re_tmp * t123_im;
  e_re = d_re * t128.re - d_im * t128.im;
  d_im = d_re * t128.im + d_im * t128.re;
  d_re = e_re * d1 - d_im * 0.0;
  d_im = e_re * 0.0 + d_im * d1;
  t73_re = q_t43_re_tmp * d1 - l_t43_im_tmp * 0.0;
  d_t54_re = q_t43_re_tmp * 0.0 + l_t43_im_tmp * d1;
  b_t44_re_tmp = b_t44_re_tmp_tmp * t101;
  ab_t42_re_tmp = b_t44_re_tmp * t123_re;
  e_t42_re_tmp_tmp = b_t44_re_tmp * t123_im;
  c_t48_re_tmp = 3.1415926535897931 * (ab_t42_re_tmp * d - e_t42_re_tmp_tmp *
    0.0);
  e_t42_re_tmp_tmp = 3.1415926535897931 * (ab_t42_re_tmp * 0.0 +
    e_t42_re_tmp_tmp * d);
  t56_re_tmp = t56_re_tmp_tmp * t93 * t105 * t119;
  t56_re = t56_re_tmp * t123_re;
  t48_re_tmp_tmp = t56_re_tmp * t123_im;
  b_t56_re = t56_re * t128.re - t48_re_tmp_tmp * t128.im;
  t48_re_tmp_tmp = t56_re * t128.im + t48_re_tmp_tmp * t128.re;
  b_t56_re_tmp = r_t43_re_tmp * d1 - m_t43_im_tmp * 0.0;
  b_t50_re_tmp = r_t43_re_tmp * 0.0 + m_t43_im_tmp * d1;
  e_t48_re = 3.1415926535897931 * (b_t56_re_tmp * t224_re - b_t50_re_tmp *
    t224_im);
  b_t50_re_tmp = 3.1415926535897931 * (b_t56_re_tmp * t224_im + b_t50_re_tmp *
    t224_re);
  t43_re_tmp = t43_re_tmp_tmp * t93 * t105 * t119;
  b_t56_re_tmp = t43_re_tmp * t123_re;
  f_t48_re = t43_re_tmp * t123_im;
  g_t48_re = b_t56_re_tmp * t128.re - f_t48_re * t128.im;
  f_t48_re = b_t56_re_tmp * t128.im + f_t48_re * t128.re;
  re_tmp = 31.006276680299816 * t44 * t50 * t56_tmp * t56_tmp * t68 * t71 * t88 *
    t100;
  e_re = re_tmp * t123_re;
  e_im = re_tmp * t123_im;
  f_re = e_re * d - e_im * 0.0;
  e_im = e_re * 0.0 + e_im * d;
  re_tmp = 9.869604401089358 * t44 * t50 * t56_tmp * t56_tmp * t68 * t71 * t88 *
    t106;
  e_re = re_tmp * t123_re;
  f_im = re_tmp * t123_im;
  re_tmp = 9.869604401089358 * t56_tmp * t56_tmp * t68 * t71 * t99 * t93 * t119;
  g_re = re_tmp * t123_re;
  g_im = re_tmp * t123_im;
  h_re = g_re * t128.re - g_im * t128.im;
  g_im = g_re * t128.im + g_im * t128.re;
  g_re = h_re * d1 - g_im * 0.0;
  g_im = h_re * 0.0 + g_im * d1;
  re_tmp = re_tmp_tmp * t71 * t86 * t99 * t93 * t119;
  h_re = re_tmp * t123_re;
  h_im = re_tmp * t123_im;
  i_re = h_re * t128.re - h_im * t128.im;
  h_im = h_re * t128.im + h_im * t128.re;
  h_re = i_re * d1 - h_im * 0.0;
  h_im = i_re * 0.0 + h_im * d1;
  i_re = k_re_tmp * d1 - i_im_tmp * 0.0;
  i_im = k_re_tmp * 0.0 + i_im_tmp * d1;
  j_re = i_re_tmp * t250_re - k_im_tmp * t250_im;
  j_im = i_re_tmp * t250_im + k_im_tmp * t250_re;
  t50_re_tmp = t50_re_tmp_tmp * t94_tmp * t105 * t119;
  t48_re = t50_re_tmp * t123_re;
  t50_im = t50_re_tmp * t123_im;
  t50_re = t48_re * t128.re - t50_im * t128.im;
  t50_im = t48_re * t128.im + t50_im * t128.re;
  t50_re_tmp = t50_re_tmp_tmp * t93 * t106 * t119;
  t48_re = t50_re_tmp * t123_re;
  b_t55_re = t50_re_tmp * t123_im;
  i_t42_re = t48_re * t128.re - b_t55_re * t128.im;
  b_t55_re = t48_re * t128.im + b_t55_re * t128.re;
  b_t56_re_tmp = s_t43_re_tmp * t129.re - n_t43_im_tmp * t129.im;
  t70 = s_t43_re_tmp * t129.im + n_t43_im_tmp * t129.re;
  t81 = b_t56_re_tmp * d1 - t70 * 0.0;
  t70 = b_t56_re_tmp * 0.0 + t70 * d1;
  re_tmp_tmp = f_re_tmp_tmp * t68 * t71;
  re_tmp = re_tmp_tmp * t94_tmp * t99 * t119;
  t93_im_tmp = re_tmp * t123_re;
  t73_re_tmp_tmp_tmp = re_tmp * t123_im;
  b_t93_re_tmp = t93_im_tmp * t128.re - t73_re_tmp_tmp_tmp * t128.im;
  t73_re_tmp_tmp_tmp = t93_im_tmp * t128.im + t73_re_tmp_tmp_tmp * t128.re;
  t93_im_tmp = b_t93_re_tmp * d1 - t73_re_tmp_tmp_tmp * 0.0;
  t73_re_tmp_tmp_tmp = b_t93_re_tmp * 0.0 + t73_re_tmp_tmp_tmp * d1;
  re_tmp = re_tmp_tmp * t100 * t93 * t119;
  b_t93_re_tmp = re_tmp * t123_re;
  t54_re_tmp_tmp = re_tmp * t123_im;
  re_tmp_tmp_tmp = b_t93_re_tmp * t128.re - t54_re_tmp_tmp * t128.im;
  t54_re_tmp_tmp = b_t93_re_tmp * t128.im + t54_re_tmp_tmp * t128.re;
  b_t93_re_tmp = re_tmp_tmp_tmp * d1 - t54_re_tmp_tmp * 0.0;
  t54_re_tmp_tmp = re_tmp_tmp_tmp * 0.0 + t54_re_tmp_tmp * d1;
  re_tmp = 31.006276680299816 * t50 * t56_tmp * t56_tmp * t68 * t71 * t93 * t105
    * t119;
  re_tmp_tmp_tmp = re_tmp * t123_re;
  e_t54_re_tmp = re_tmp * t123_im;
  d_t54_re_tmp = re_tmp_tmp_tmp * t128.re - e_t54_re_tmp * t128.im;
  e_t54_re_tmp = re_tmp_tmp_tmp * t128.im + e_t54_re_tmp * t128.re;
  re_tmp_tmp_tmp = l_re_tmp * t129.re - j_im_tmp * t129.im;
  f_t48_re_tmp = l_re_tmp * t129.im + j_im_tmp * t129.re;
  t56_re_tmp_tmp = re_tmp_tmp_tmp * d1 - f_t48_re_tmp * 0.0;
  f_t48_re_tmp = re_tmp_tmp_tmp * 0.0 + f_t48_re_tmp * d1;
  re_tmp_tmp_tmp = t56_re_tmp_tmp * t184.re - f_t48_re_tmp * t184.im;
  f_t48_re_tmp = t56_re_tmp_tmp * t184.im + f_t48_re_tmp * t184.re;
  b_t56_re_tmp = b_t43_re_tmp_tmp * t113_re;
  b_t54_im = b_t43_re_tmp_tmp * t113_im;
  t49 = t120 * (b_t56_re_tmp * t116_re - b_t54_im * t116_im);
  b_t54_im = t120 * (b_t56_re_tmp * t116_im + b_t54_im * t116_re);
  b_t56_re_tmp = t49 * t123_re - b_t54_im * t123_im;
  b_t54_im = t49 * t123_im + b_t54_im * t123_re;
  t49 = b_t56_re_tmp * d1 - b_t54_im * 0.0;
  b_t54_im = b_t56_re_tmp * 0.0 + b_t54_im * d1;
  t56_re_tmp_tmp = i_re_tmp_tmp * t113_re;
  t115 = i_re_tmp_tmp * t113_im;
  c_t54_re_tmp = t120 * (t56_re_tmp_tmp * t116_re - t115 * t116_im);
  t115 = t120 * (t56_re_tmp_tmp * t116_im + t115 * t116_re);
  t56_re_tmp_tmp = c_t54_re_tmp * t123_re - t115 * t123_im;
  t115 = c_t54_re_tmp * t123_im + t115 * t123_re;
  c_t54_re_tmp = t56_re_tmp_tmp * d1 - t115 * 0.0;
  t115 = t56_re_tmp_tmp * 0.0 + t115 * d1;
  t56_re_tmp_tmp = c_t54_re_tmp * t184.re - t115 * t184.im;
  t115 = c_t54_re_tmp * t184.im + t115 * t184.re;
  Mzyy->re = (((((((((((((((d_t104_re_tmp * t255_re - d_t104_im_tmp * t255_im) *
    -1.0669676460233539 + (d_t54_im_tmp * t250_re - t111 * t250_im) *
    1.0669676460233539) - (e_t104_re_tmp * t58_im - e_t104_im_tmp * e_t48_re_tmp)
    * 1.0669676460233539) + (b_t104_re_tmp * t78 - b_t104_im_tmp * t68_im) *
    1.0669676460233539) + ((((j_t43_re_tmp * t244_re - f_t43_im_tmp * t244_im) *
    -0.53348382301167685 + (t44_re_tmp * t255_re - c_t44_im_tmp * t255_im) *
    0.53348382301167685) + (t43_re * t224_re - t43_im * t224_im) *
    0.53348382301167685) + (t44_re * t250_re - t44_im * t250_im) *
    0.53348382301167685)) + ((((b_t43_re * 0.0 - b_t43_im * 124.4277138219654) -
    (b_t44_re * 0.0 - b_t44_im * 124.4277138219654)) - (d_t43_re * 0.0 -
    c_t43_im * 124.4277138219654)) - (c_t58_re_tmp * 0.0 - d_t50_re_tmp *
    124.4277138219654))) + ((((c_t43_re * d - e_t55_re * 0.0) *
    1.6004514690350311 - (b_t42_re_tmp_tmp_tmp * 0.0 - e_t54_im *
    373.28314146589628)) - (re * d - im * 0.0) * 29021.041124656658) +
    (d_t56_re_tmp * d - b_t93_re * 0.0) * 0.40011286725875761)) + ((((e_t43_re *
    d - ai * 0.0) * -0.40011286725875761 - (t55 * 0.0 - g_t42_re_tmp_tmp *
    93.320785366474084)) + (e_t42_im * 0.0 - i_t48_re * 93.320785366474084)) -
    3.1415926535897931 * (e_t54_re * d1 - f_t54_re * 0.0) * 0.60969579772763072))
                    + (((b_re * d - b_im * 0.0) * -7255.2602811641664 + (c_re *
    d - c_im * 0.0) * 7255.2602811641664) - (t61 * d - d_t48_re_tmp * 0.0) *
                       2.0005643362937882)) + ((((d_re * 0.0 - d_im *
    142.2031015108177) - (t73_re * t224_re - d_t54_re * t224_im) *
    1.6004514690350311) + (c_t48_re_tmp * 0.0 - e_t42_re_tmp_tmp *
    466.60392683237041)) - 3.1415926535897931 * (b_t56_re * d1 - t48_re_tmp_tmp *
    0.0) * 0.15242394943190771)) + (((e_t48_re * 0.0 - b_t50_re_tmp *
    373.28314146589628) + 3.1415926535897931 * (g_t48_re * d1 - f_t48_re * 0.0) *
    0.15242394943190771) - 3.1415926535897931 * (l_t43_re_tmp * t250_re -
    o_t43_im_tmp * t250_im) * 0.3048478988638153)) + (((f_re * -0.0 - e_im *
    -1.6921889868604471E+6) + (e_re * d - f_im * 0.0) * 43531.561686985) + (g_re
    * 0.0 - g_im * 35.550775377704412))) + (((h_re * -0.0 - h_im *
    -35.550775377704412) + (i_re * t224_re - i_im * t224_im) *
    29021.041124656658) + (j_re * 0.0 - j_im * 71.101550755408823))) +
               (((3.1415926535897931 * (t50_re * d1 - t50_im * 0.0) *
                  -0.15242394943190771 + 3.1415926535897931 * (i_t42_re * d1 -
    b_t55_re * 0.0) * 0.457271848295723) - 3.1415926535897931 * (t81 * t184.re -
    t70 * t184.im) * 0.15242394943190771) + (t93_im_tmp * 0.0 -
    t73_re_tmp_tmp_tmp * 35.550775377704412))) + (((b_t93_re_tmp * -0.0 -
    t54_re_tmp_tmp * -106.65232613311321) - (d_t54_re_tmp * d1 - e_t54_re_tmp *
    0.0) * 8291.72603561619) + (re_tmp_tmp_tmp * 0.0 - f_t48_re_tmp *
    35.550775377704412))) + (3.1415926535897931 * (t49 * t184.re - b_t54_im *
    t184.im) * 0.3048478988638153 - (t56_re_tmp_tmp * 0.0 - t115 *
    71.101550755408823));
  Mzyy->im = (((((((((((((((d_t104_re_tmp * t255_im + d_t104_im_tmp * t255_re) *
    -1.0669676460233539 + (d_t54_im_tmp * t250_im + t111 * t250_re) *
    1.0669676460233539) - (e_t104_re_tmp * e_t48_re_tmp + e_t104_im_tmp * t58_im)
    * 1.0669676460233539) + (b_t104_re_tmp * t68_im + b_t104_im_tmp * t78) *
    1.0669676460233539) + ((((j_t43_re_tmp * t244_im + f_t43_im_tmp * t244_re) *
    -0.53348382301167685 + (t44_re_tmp * t255_im + c_t44_im_tmp * t255_re) *
    0.53348382301167685) + (t43_re * t224_im + t43_im * t224_re) *
    0.53348382301167685) + (t44_re * t250_im + t44_im * t250_re) *
    0.53348382301167685)) + ((((b_t43_re * 124.4277138219654 + b_t43_im * 0.0) -
    (b_t44_re * 124.4277138219654 + b_t44_im * 0.0)) - (d_t43_re *
    124.4277138219654 + c_t43_im * 0.0)) - (c_t58_re_tmp * 124.4277138219654 +
    d_t50_re_tmp * 0.0))) + ((((c_t43_re * 0.0 + e_t55_re * d) *
    1.6004514690350311 - (b_t42_re_tmp_tmp_tmp * 373.28314146589628 + e_t54_im *
    0.0)) - (re * 0.0 + im * d) * 29021.041124656658) + (d_t56_re_tmp * 0.0 +
    b_t93_re * d) * 0.40011286725875761)) + ((((e_t43_re * 0.0 + ai * d) *
    -0.40011286725875761 - (t55 * 93.320785366474084 + g_t42_re_tmp_tmp * 0.0))
    + (e_t42_im * 93.320785366474084 + i_t48_re * 0.0)) - 3.1415926535897931 *
    (e_t54_re * 0.0 + f_t54_re * d1) * 0.60969579772763072)) + (((b_re * 0.0 +
    b_im * d) * -7255.2602811641664 + (c_re * 0.0 + c_im * d) *
    7255.2602811641664) - (t61 * 0.0 + d_t48_re_tmp * d) * 2.0005643362937882))
                   + ((((d_re * 142.2031015108177 + d_im * 0.0) - (t73_re *
    t224_im + d_t54_re * t224_re) * 1.6004514690350311) + (c_t48_re_tmp *
    466.60392683237041 + e_t42_re_tmp_tmp * 0.0)) - 3.1415926535897931 *
                      (b_t56_re * 0.0 + t48_re_tmp_tmp * d1) *
                      0.15242394943190771)) + (((e_t48_re * 373.28314146589628 +
    b_t50_re_tmp * 0.0) + 3.1415926535897931 * (g_t48_re * 0.0 + f_t48_re * d1) *
    0.15242394943190771) - 3.1415926535897931 * (l_t43_re_tmp * t250_im +
    o_t43_im_tmp * t250_re) * 0.3048478988638153)) + (((f_re *
    -1.6921889868604471E+6 + e_im * -0.0) + (e_re * 0.0 + f_im * d) *
    43531.561686985) + (g_re * 35.550775377704412 + g_im * 0.0))) + (((h_re *
    -35.550775377704412 + h_im * -0.0) + (i_re * t224_im + i_im * t224_re) *
    29021.041124656658) + (j_re * 71.101550755408823 + j_im * 0.0))) +
               (((3.1415926535897931 * (t50_re * 0.0 + t50_im * d1) *
                  -0.15242394943190771 + 3.1415926535897931 * (i_t42_re * 0.0 +
    b_t55_re * d1) * 0.457271848295723) - 3.1415926535897931 * (t81 * t184.im +
    t70 * t184.re) * 0.15242394943190771) + (t93_im_tmp * 35.550775377704412 +
    t73_re_tmp_tmp_tmp * 0.0))) + (((b_t93_re_tmp * -106.65232613311321 +
    t54_re_tmp_tmp * -0.0) - (d_t54_re_tmp * 0.0 + e_t54_re_tmp * d1) *
    8291.72603561619) + (re_tmp_tmp_tmp * 35.550775377704412 + f_t48_re_tmp *
    0.0))) + (3.1415926535897931 * (t49 * t184.im + b_t54_im * t184.re) *
              0.3048478988638153 - (t56_re_tmp_tmp * 71.101550755408823 + t115 *
    0.0));
  t58_re_tmp_tmp = t58_re_tmp_tmp_tmp * t68;
  t58_re_tmp = t58_re_tmp_tmp * t105;
  t48_im = t58_re_tmp * t123_re;
  t58_im = t58_re_tmp * t123_im;
  t81 = t58_tmp * t65 * t73;
  t58_re_tmp = t81 * t105;
  h_t48_re = t58_re_tmp * t123_re;
  c_t54_re = t58_re_tmp * t123_im;
  t98 = t59 * t58_tmp * t73;
  t41_tmp = t98 * t105;
  t55_re = t41_tmp * t123_re;
  b_t55_re = t41_tmp * t123_im;
  t103 = t59 * t65 * t74;
  t41_tmp = t103 * t105;
  d_t54_re = t41_tmp * t123_re;
  d_t55_re = t41_tmp * t123_im;
  b_t44_re_tmp_tmp = t44_re_tmp_tmp_tmp * t89;
  t44_re_tmp = b_t44_re_tmp_tmp * t105;
  t44_re = t44_re_tmp * t123_re;
  t44_im = t44_re_tmp * t123_im;
  i_t48_re = t58_re_tmp_tmp_tmp * t80;
  t58_re_tmp = i_t48_re * t105;
  e_t54_re = t58_re_tmp * t123_re;
  f_t54_re = t58_re_tmp * t123_im;
  t44_re_tmp_tmp_tmp = t44 * t74;
  c_t44_re_tmp_tmp = t44_re_tmp_tmp_tmp * rt_powd_snf(t88, 5.0);
  t44_re_tmp = c_t44_re_tmp_tmp * t105;
  b_t44_re = t44_re_tmp * t123_re;
  b_t44_im = t44_re_tmp * t123_im;
  b_t44_re_tmp_tmp_tmp = t44 * t80;
  t49_im_tmp = b_t44_re_tmp_tmp_tmp * t89;
  t44_re_tmp = t49_im_tmp * t105;
  d_t56_re_tmp = t44_re_tmp * t123_re;
  d_t50_re_tmp = t44_re_tmp * t123_im;
  t58_re_tmp = t58_re_tmp_tmp * t99;
  t41_tmp = t58_re_tmp * t123_re;
  f_t48_re = t58_re_tmp * t123_im;
  g_t48_re = 3.1415926535897931 * (t41_tmp * d - f_t48_re * 0.0);
  f_t48_re = 3.1415926535897931 * (t41_tmp * 0.0 + f_t48_re * d);
  t58_re_tmp = t81 * t99;
  t41_tmp = t58_re_tmp * t123_re;
  b_t50_re_tmp = t58_re_tmp * t123_im;
  e_t48_re = 3.1415926535897931 * (t41_tmp * d - b_t50_re_tmp * 0.0);
  b_t50_re_tmp = 3.1415926535897931 * (t41_tmp * 0.0 + b_t50_re_tmp * d);
  t41_tmp = t98 * t99;
  t81 = t41_tmp * t123_re;
  t70 = t41_tmp * t123_im;
  b_t56_re_tmp = 3.1415926535897931 * (t81 * d - t70 * 0.0);
  t70 = 3.1415926535897931 * (t81 * 0.0 + t70 * d);
  t41_tmp = t103 * t99;
  t81 = t41_tmp * t123_re;
  d_t48_re = t41_tmp * t123_im;
  t49_re = 3.1415926535897931 * (t81 * d - d_t48_re * 0.0);
  d_t48_re = 3.1415926535897931 * (t81 * 0.0 + d_t48_re * d);
  t44_re_tmp = b_t44_re_tmp_tmp * t99;
  c_t58_re_tmp = t44_re_tmp * t123_re;
  b_t93_re = t44_re_tmp * t123_im;
  t61 = 3.1415926535897931 * (c_t58_re_tmp * d - b_t93_re * 0.0);
  b_t93_re = 3.1415926535897931 * (c_t58_re_tmp * 0.0 + b_t93_re * d);
  t58_re_tmp = i_t48_re * t99;
  t41_tmp = t58_re_tmp * t123_re;
  t49_re_tmp = t58_re_tmp * t123_im;
  t49_re_tmp_tmp = 3.1415926535897931 * (t41_tmp * d - t49_re_tmp * 0.0);
  t49_re_tmp = 3.1415926535897931 * (t41_tmp * 0.0 + t49_re_tmp * d);
  t44_re_tmp = c_t44_re_tmp_tmp * t99;
  c_t58_re_tmp = t44_re_tmp * t123_re;
  g_t42_re_tmp_tmp = t44_re_tmp * t123_im;
  t55 = 3.1415926535897931 * (c_t58_re_tmp * d - g_t42_re_tmp_tmp * 0.0);
  g_t42_re_tmp_tmp = 3.1415926535897931 * (c_t58_re_tmp * 0.0 + g_t42_re_tmp_tmp
    * d);
  t44_re_tmp = t49_im_tmp * t99;
  c_t58_re_tmp = t44_re_tmp * t123_re;
  d_t48_re_tmp = t44_re_tmp * t123_im;
  ab_t42_re_tmp = 3.1415926535897931 * (c_t58_re_tmp * d - d_t48_re_tmp * 0.0);
  d_t48_re_tmp = 3.1415926535897931 * (c_t58_re_tmp * 0.0 + d_t48_re_tmp * d);
  t52_re_tmp_tmp = t52_re_tmp_tmp_tmp * t68;
  t52_re_tmp = t52_re_tmp_tmp * t106;
  t48_re = t52_re_tmp * t123_re;
  t54_re = t52_re_tmp * t123_im;
  t81 = t52_re_tmp_tmp_tmp_tmp * t65 * t73;
  t52_re_tmp = t81 * t106;
  d_t54_im = t52_re_tmp * t123_re;
  c_t54_im = t52_re_tmp * t123_im;
  t41_tmp = t52 * t59;
  t98 = t41_tmp * t58_tmp * t73;
  t52_re_tmp = t98 * t106;
  t54_im = t52_re_tmp * t123_re;
  t55_im = t52_re_tmp * t123_im;
  t41_tmp = t41_tmp * t65 * t74;
  t52_re_tmp = t41_tmp * t106;
  t50_im = t52_re_tmp * t123_re;
  b_t54_im = t52_re_tmp * t123_im;
  t103 = t52_re_tmp_tmp_tmp * t80;
  t52_re_tmp = t103 * t106;
  t49 = t52_re_tmp * t123_re;
  o_re_tmp = t52_re_tmp * t123_im;
  t52_re_tmp = t52_re_tmp_tmp * t100;
  i_t48_re = t52_re_tmp * t123_re;
  b_t55_im = t52_re_tmp * t123_im;
  d_t48_re_tmp_tmp = 3.1415926535897931 * (i_t48_re * d - b_t55_im * 0.0);
  b_t55_im = 3.1415926535897931 * (i_t48_re * 0.0 + b_t55_im * d);
  t52_re_tmp = t81 * t100;
  i_t48_re = t52_re_tmp * t123_re;
  t52_re_tmp_tmp = t52_re_tmp * t123_im;
  c_t48_re_tmp_tmp = 3.1415926535897931 * (i_t48_re * d - t52_re_tmp_tmp * 0.0);
  t52_re_tmp_tmp = 3.1415926535897931 * (i_t48_re * 0.0 + t52_re_tmp_tmp * d);
  t52_re_tmp = t98 * t100;
  i_t48_re = t52_re_tmp * t123_re;
  t93_re_tmp_tmp = t52_re_tmp * t123_im;
  t93_im_tmp_tmp = 3.1415926535897931 * (i_t48_re * d - t93_re_tmp_tmp * 0.0);
  t93_re_tmp_tmp = 3.1415926535897931 * (i_t48_re * 0.0 + t93_re_tmp_tmp * d);
  t52_re_tmp = t41_tmp * t100;
  i_t48_re = t52_re_tmp * t123_re;
  y_t42_re_tmp = t52_re_tmp * t123_im;
  e_t55_re = 3.1415926535897931 * (i_t48_re * d - y_t42_re_tmp * 0.0);
  y_t42_re_tmp = 3.1415926535897931 * (i_t48_re * 0.0 + y_t42_re_tmp * d);
  t52_re_tmp = t103 * t100;
  i_t48_re = t52_re_tmp * t123_re;
  c_t93_re_tmp = t52_re_tmp * t123_im;
  b_t48_re_tmp_tmp = 3.1415926535897931 * (i_t48_re * d - c_t93_re_tmp * 0.0);
  c_t93_re_tmp = 3.1415926535897931 * (i_t48_re * 0.0 + c_t93_re_tmp * d);
  t68_re = e_t68_re_tmp * t129.re - c_t68_im_tmp * t129.im;
  t68_im = e_t68_re_tmp * t129.im + c_t68_im_tmp * t129.re;
  t103 = t74 * t93 * t104;
  t81 = t103 * t119;
  b_t48_im = t81 * t123_re;
  e_t54_im = t81 * t123_im;
  c_t42_re_tmp_tmp = b_t48_im * t130_re - e_t54_im * t130_im;
  e_t54_im = b_t48_im * t130_im + e_t54_im * t130_re;
  t41_tmp = t80 * t93 * t104;
  t81 = t41_tmp * t119;
  e_t48_im = t81 * t123_re;
  c_t48_im = t81 * t123_im;
  d_t48_im = e_t48_im * t129.re - c_t48_im * t129.im;
  c_t48_im = e_t48_im * t129.im + c_t48_im * t129.re;
  re_tmp = j_re_tmp_tmp * t68 * t105;
  re = re_tmp * t123_re;
  im = re_tmp * t123_im;
  re_tmp = f_re_tmp_tmp_tmp * t65 * t73 * t105;
  b_re = re_tmp * t123_re;
  b_im = re_tmp * t123_im;
  re_tmp_tmp = 9.869604401089358 * t52 * t59;
  re_tmp = re_tmp_tmp * t58_tmp * t73 * t105;
  c_re = re_tmp * t123_re;
  c_im = re_tmp * t123_im;
  re_tmp = re_tmp_tmp * t65 * t74 * t105;
  d_re = re_tmp * t123_re;
  d_im = re_tmp * t123_im;
  re_tmp = j_re_tmp_tmp * t80 * t105;
  e_re = re_tmp * t123_re;
  e_im = re_tmp * t123_im;
  b_t44_re_tmp_tmp = t44 * t58_tmp * t58_tmp * t74 * t88;
  t44_re_tmp = b_t44_re_tmp_tmp * t106;
  c_t58_re_tmp = t44_re_tmp * t123_re;
  e_t42_re_tmp_tmp = t44_re_tmp * t123_im;
  t81 = t53 * t58_tmp * t58_tmp * t74 * t88;
  t98 = t81 * t107;
  t93_re = t98 * t123_re;
  d_t42_re_tmp_tmp = t98 * t123_im;
  t44_re_tmp = b_t44_re_tmp_tmp * t100;
  c_t48_re_tmp = t44_re_tmp * t123_re;
  b_t54_re_tmp_tmp = t44_re_tmp * t123_im;
  t48_re_tmp_tmp = 3.1415926535897931 * (c_t48_re_tmp * d - b_t54_re_tmp_tmp *
    0.0);
  b_t54_re_tmp_tmp = 3.1415926535897931 * (c_t48_re_tmp * 0.0 + b_t54_re_tmp_tmp
    * d);
  t98 = t81 * t101;
  t81 = t98 * t123_re;
  t55_re_tmp_tmp = t98 * t123_im;
  b_t55_re_tmp_tmp = 3.1415926535897931 * (t81 * d - t55_re_tmp_tmp * 0.0);
  t55_re_tmp_tmp = 3.1415926535897931 * (t81 * 0.0 + t55_re_tmp_tmp * d);
  t78 = b_t68_re_tmp_tmp * t113_re;
  b_t44_re_tmp_tmp = b_t68_re_tmp_tmp * t113_im;
  i_t48_re = t120 * (t78 * t116_re - b_t44_re_tmp_tmp * t116_im);
  b_t44_re_tmp_tmp = t120 * (t78 * t116_im + b_t44_re_tmp_tmp * t116_re);
  t78 = i_t48_re * t123_re - b_t44_re_tmp_tmp * t123_im;
  b_t44_re_tmp_tmp = i_t48_re * t123_im + b_t44_re_tmp_tmp * t123_re;
  t73_re = t120 * (b_t73_re_tmp * t117_re - b_t73_im_tmp * t117_im);
  t73_im = t120 * (b_t73_re_tmp * t117_im + b_t73_im_tmp * t117_re);
  b_t56_re = t73_re * t123_re - t73_im * t123_im;
  t73_im = t73_re * t123_im + t73_im * t123_re;
  t73_re = t73_re_tmp_tmp * t114_re;
  t50_re = t73_re_tmp_tmp * t114_im;
  e_t42_im = t120 * (t73_re * t116_re - t50_re * t116_im);
  t50_re = t120 * (t73_re * t116_im + t50_re * t116_re);
  t73_re = e_t42_im * t123_re - t50_re * t123_im;
  t50_re = e_t42_im * t123_im + t50_re * t123_re;
  b_t48_im = t103 * t114_re;
  ai = t103 * t114_im;
  t81 = t120 * (b_t48_im * t117_re - ai * t117_im);
  ai = t120 * (b_t48_im * t117_im + ai * t117_re);
  b_t48_im = t81 * t123_re - ai * t123_im;
  ai = t81 * t123_im + ai * t123_re;
  e_t48_im = t41_tmp * t113_re;
  b_t48_re = t41_tmp * t113_im;
  t81 = t120 * (e_t48_im * t116_re - b_t48_re * t116_im);
  b_t48_re = t120 * (e_t48_im * t116_im + b_t48_re * t116_re);
  e_t48_im = t81 * t123_re - b_t48_re * t123_im;
  b_t48_re = t81 * t123_im + b_t48_re * t123_re;
  e_t42_im = c_t73_re_tmp * t123_re - c_t73_im_tmp * t123_im;
  i_t42_re = c_t73_re_tmp * t123_im + c_t73_im_tmp * t123_re;
  re_tmp = 9.869604401089358 * t44 * t58_tmp * t58_tmp * t74 * t88 * t105;
  f_re = re_tmp * t123_re;
  f_im = re_tmp * t123_im;
  re_tmp = 31.006276680299816 * t53 * t58_tmp * t58_tmp * t74 * t88 * t100;
  g_re = re_tmp * t123_re;
  g_im = re_tmp * t123_im;
  h_re = g_re * d - g_im * 0.0;
  g_im = g_re * 0.0 + g_im * d;
  re_tmp = 9.869604401089358 * t53 * t58_tmp * t58_tmp * t74 * t88 * t106;
  g_re = re_tmp * t123_re;
  h_im = re_tmp * t123_im;
  t58_re_tmp = t58_re_tmp_tmp_tmp * t74 * t93 * t105 * t119;
  t41_tmp = t58_re_tmp * t123_re;
  c_t48_re = t58_re_tmp * t123_im;
  b_t54_re = t41_tmp * t128.re - c_t48_re * t128.im;
  c_t48_re = t41_tmp * t128.im + c_t48_re * t128.re;
  t44_re_tmp = t44_re_tmp_tmp * t93 * t105 * t119;
  c_t48_re_tmp = t44_re_tmp * t123_re;
  c_t43_re_tmp_tmp = t44_re_tmp * t123_im;
  c_t55_re = c_t48_re_tmp * t128.re - c_t43_re_tmp_tmp * t128.im;
  c_t43_re_tmp_tmp = c_t48_re_tmp * t128.im + c_t43_re_tmp_tmp * t128.re;
  t44_re_tmp_tmp = t44_re_tmp_tmp_tmp * t88 * t93 * t105;
  t44_re_tmp = t44_re_tmp_tmp * t119;
  c_t48_re_tmp = t44_re_tmp * t123_re;
  t55_re_tmp = t44_re_tmp * t123_im;
  c_t44_re_tmp_tmp = c_t48_re_tmp * t129.re - t55_re_tmp * t129.im;
  t55_re_tmp = c_t48_re_tmp * t129.im + t55_re_tmp * t129.re;
  t44_re_tmp = t44_re_tmp_tmp_tmp * t89 * t93 * t105 * t119;
  c_t48_re_tmp = t44_re_tmp * t123_re;
  e_t48_re_tmp = t44_re_tmp * t123_im;
  d_t43_re_tmp_tmp = c_t48_re_tmp * t128.re - e_t48_re_tmp * t128.im;
  e_t48_re_tmp = c_t48_re_tmp * t128.im + e_t48_re_tmp * t128.re;
  t44_re_tmp = b_t44_re_tmp_tmp_tmp * t88 * t93 * t105 * t119;
  c_t48_re_tmp = t44_re_tmp * t123_re;
  t111 = t44_re_tmp * t123_im;
  t50_re_tmp_tmp = c_t48_re_tmp * t128.re - t111 * t128.im;
  t111 = c_t48_re_tmp * t128.im + t111 * t128.re;
  t81 = t103 * t113_re;
  t58_re_tmp_tmp = t103 * t113_im;
  d_t54_im_tmp = t121 * (t81 * t116_re - t58_re_tmp_tmp * t116_im);
  t58_re_tmp_tmp = t121 * (t81 * t116_im + t58_re_tmp_tmp * t116_re);
  t81 = d_t54_im_tmp * t123_re - t58_re_tmp_tmp * t123_im;
  t58_re_tmp_tmp = d_t54_im_tmp * t123_im + t58_re_tmp_tmp * t123_re;
  d_t54_im_tmp = t81 * t128.re - t58_re_tmp_tmp * t128.im;
  t58_re_tmp_tmp = t81 * t128.im + t58_re_tmp_tmp * t128.re;
  re_tmp = 9.869604401089358 * t58_tmp * t58_tmp * t74 * t99 * t93 * t119;
  i_re = re_tmp * t123_re;
  i_im = re_tmp * t123_im;
  j_re = i_re * t128.re - i_im * t128.im;
  i_im = i_re * t128.im + i_im * t128.re;
  i_re = j_re * d1 - i_im * 0.0;
  i_im = j_re * 0.0 + i_im * d1;
  re_tmp = 9.869604401089358 * t44 * t68 * t88 * t99 * t93 * t119;
  j_re = re_tmp * t123_re;
  j_im = re_tmp * t123_im;
  t93_im_tmp = j_re * t128.re - j_im * t128.im;
  j_im = j_re * t128.im + j_im * t128.re;
  j_re = t93_im_tmp * d1 - j_im * 0.0;
  j_im = t93_im_tmp * 0.0 + j_im * d1;
  re_tmp_tmp = 9.869604401089358 * t44 * t74;
  b_re_tmp_tmp = re_tmp_tmp * t88 * t99 * t93;
  re_tmp = b_re_tmp_tmp * t119;
  t93_im_tmp = re_tmp * t123_re;
  t73_re_tmp_tmp_tmp = re_tmp * t123_im;
  b_t93_re_tmp = t93_im_tmp * t129.re - t73_re_tmp_tmp_tmp * t129.im;
  t73_re_tmp_tmp_tmp = t93_im_tmp * t129.im + t73_re_tmp_tmp_tmp * t129.re;
  t93_im_tmp = b_t93_re_tmp * d1 - t73_re_tmp_tmp_tmp * 0.0;
  t73_re_tmp_tmp_tmp = b_t93_re_tmp * 0.0 + t73_re_tmp_tmp_tmp * d1;
  re_tmp = re_tmp_tmp * t89 * t99 * t93 * t119;
  b_t93_re_tmp = re_tmp * t123_re;
  t54_re_tmp_tmp = re_tmp * t123_im;
  re_tmp_tmp_tmp = b_t93_re_tmp * t128.re - t54_re_tmp_tmp * t128.im;
  t54_re_tmp_tmp = b_t93_re_tmp * t128.im + t54_re_tmp_tmp * t128.re;
  b_t93_re_tmp = re_tmp_tmp_tmp * d1 - t54_re_tmp_tmp * 0.0;
  t54_re_tmp_tmp = re_tmp_tmp_tmp * 0.0 + t54_re_tmp_tmp * d1;
  re_tmp = 9.869604401089358 * t44 * t80 * t88 * t99 * t93 * t119;
  re_tmp_tmp_tmp = re_tmp * t123_re;
  e_t54_re_tmp = re_tmp * t123_im;
  d_t54_re_tmp = re_tmp_tmp_tmp * t128.re - e_t54_re_tmp * t128.im;
  e_t54_re_tmp = re_tmp_tmp_tmp * t128.im + e_t54_re_tmp * t128.re;
  re_tmp_tmp_tmp = d_t54_re_tmp * d1 - e_t54_re_tmp * 0.0;
  e_t54_re_tmp = d_t54_re_tmp * 0.0 + e_t54_re_tmp * d1;
  d_t54_re_tmp = m_re_tmp * t222_re - m_im_tmp * t222_im;
  f_t48_re_tmp = m_re_tmp * t222_im + m_im_tmp * t222_re;
  t52_re_tmp = t52_re_tmp_tmp_tmp * t74 * t93 * t106 * t119;
  i_t48_re = t52_re_tmp * t123_re;
  t49_im_tmp = t52_re_tmp * t123_im;
  b_t44_re_tmp_tmp_tmp = i_t48_re * t128.re - t49_im_tmp * t128.im;
  t49_im_tmp = i_t48_re * t128.im + t49_im_tmp * t128.re;
  c_t48_re_tmp = t44_re_tmp_tmp * t113_re;
  i_t48_re = t44_re_tmp_tmp * t113_im;
  t81 = t120 * (c_t48_re_tmp * t116_re - i_t48_re * t116_im);
  i_t48_re = t120 * (c_t48_re_tmp * t116_im + i_t48_re * t116_re);
  c_t48_re_tmp = t81 * t123_re - i_t48_re * t123_im;
  i_t48_re = t81 * t123_im + i_t48_re * t123_re;
  re_tmp = j_re_tmp_tmp * t74 * t100 * t93 * t119;
  t56_re_tmp_tmp = re_tmp * t123_re;
  t115 = re_tmp * t123_im;
  c_t54_re_tmp = t56_re_tmp_tmp * t128.re - t115 * t128.im;
  t115 = t56_re_tmp_tmp * t128.im + t115 * t128.re;
  t56_re_tmp_tmp = c_t54_re_tmp * d1 - t115 * 0.0;
  t115 = c_t54_re_tmp * 0.0 + t115 * d1;
  re_tmp = 31.006276680299816 * t52 * t58_tmp * t58_tmp * t74 * t93 * t105 *
    t119;
  c_t54_re_tmp = re_tmp * t123_re;
  t41_tmp = re_tmp * t123_im;
  t103 = c_t54_re_tmp * t128.re - t41_tmp * t128.im;
  t41_tmp = c_t54_re_tmp * t128.im + t41_tmp * t128.re;
  c_t54_re_tmp = b_re_tmp_tmp * t113_re;
  t81 = b_re_tmp_tmp * t113_im;
  t98 = t120 * (c_t54_re_tmp * t116_re - t81 * t116_im);
  t81 = t120 * (c_t54_re_tmp * t116_im + t81 * t116_re);
  c_t54_re_tmp = t98 * t123_re - t81 * t123_im;
  t81 = t98 * t123_im + t81 * t123_re;
  t98 = c_t54_re_tmp * d1 - t81 * 0.0;
  t81 = c_t54_re_tmp * 0.0 + t81 * d1;
  Mzzz->re = ((((((((((((((((((((t48_im * d - t58_im * 0.0) *
    -0.53348382301167685 + (h_t48_re * d - c_t54_re * 0.0) * 0.26674191150583842)
    + (t55_re * d - b_t55_re * 0.0) * 0.26674191150583842) + (d_t54_re * d -
    d_t55_re * 0.0) * 0.13337095575291921) + ((((t44_re * d - t44_im * 0.0) *
    0.53348382301167685 - (e_t54_re * d - f_t54_re * 0.0) * 0.26674191150583842)
    - (b_t44_re * d - b_t44_im * 0.0) * 0.40011286725875761) + (d_t56_re_tmp * d
    - d_t50_re_tmp * 0.0) * 0.26674191150583842)) + ((((g_t48_re * 0.0 -
    f_t48_re * 124.4277138219654) - (e_t48_re * 0.0 - b_t50_re_tmp *
    62.21385691098272)) - (b_t56_re_tmp * 0.0 - t70 * 62.21385691098272)) -
    (t49_re * 0.0 - d_t48_re * 31.10692845549136))) + ((((t61 * -0.0 - b_t93_re *
    -124.4277138219654) + (t49_re_tmp_tmp * 0.0 - t49_re_tmp * 62.21385691098272))
    + (t55 * 0.0 - g_t42_re_tmp_tmp * 93.320785366474084)) - (ab_t42_re_tmp *
    0.0 - d_t48_re_tmp * 62.21385691098272))) + (((3.1415926535897931 *
    (f_t93_re_tmp * t222_re - e_t93_im_tmp * t222_im) * 2.4387831909105229 +
    (t48_re * d - t54_re * 0.0) * 1.6004514690350311) - (d_t54_im * d - c_t54_im
    * 0.0) * 0.80022573451751533) - (t54_im * d - t55_im * 0.0) *
    0.80022573451751533)) + ((((t50_im * d - b_t54_im * 0.0) *
    -0.40011286725875761 + (t49 * d - o_re_tmp * 0.0) * 0.80022573451751533) -
    (d_t48_re_tmp_tmp * 0.0 - b_t55_im * 373.28314146589628)) +
    (c_t48_re_tmp_tmp * 0.0 - t52_re_tmp_tmp * 186.6415707329482))) +
                        ((((t93_im_tmp_tmp * 0.0 - t93_re_tmp_tmp *
    186.6415707329482) + (e_t55_re * 0.0 - y_t42_re_tmp * 93.320785366474084)) -
    (b_t48_re_tmp_tmp * 0.0 - c_t93_re_tmp * 186.6415707329482)) -
    3.1415926535897931 * (t68_re * d1 - t68_im * 0.0) * 0.60969579772763072)) +
                       (((3.1415926535897931 * (c_t42_re_tmp_tmp * d1 - e_t54_im
    * 0.0) * 0.457271848295723 - 3.1415926535897931 * (d_t48_im * d1 - c_t48_im *
    0.0) * 0.3048478988638153) - 3.1415926535897931 * (t73_re_tmp * t222_re -
    t73_im_tmp * t222_im) * 0.60969579772763072) + 3.1415926535897931 *
                        (d_t68_re_tmp * t97.re - d_t68_im_tmp * t97.im) *
                        0.60969579772763072)) + ((((re * d - im * 0.0) *
    -29021.041124656658 + (b_re * d - b_im * 0.0) * 14510.520562328329) + (c_re *
    d - c_im * 0.0) * 14510.520562328329) + (d_re * d - d_im * 0.0) *
    7255.2602811641664)) + ((((e_re * d - e_im * 0.0) * -14510.520562328329 +
    (c_t58_re_tmp * d - e_t42_re_tmp_tmp * 0.0) * 1.2003386017762729) - (t93_re *
    d - d_t42_re_tmp_tmp * 0.0) * 2.0005643362937882) - (t48_re_tmp_tmp * 0.0 -
    b_t54_re_tmp_tmp * 279.96235609942232))) + ((((b_t55_re_tmp_tmp * 0.0 -
    t55_re_tmp_tmp * 466.60392683237041) + 3.1415926535897931 * (t78 * d1 -
    b_t44_re_tmp_tmp * 0.0) * 1.219391595455261) - 3.1415926535897931 *
    (b_t56_re * d1 - t73_im * 0.0) * 0.60969579772763072) + 3.1415926535897931 *
    (t73_re * d1 - t50_re * 0.0) * 0.60969579772763072)) + (((3.1415926535897931
    * (b_t48_im * d1 - ai * 0.0) * 0.3048478988638153 + 3.1415926535897931 *
    (e_t48_im * d1 - b_t48_re * 0.0) * 0.60969579772763072) + 3.1415926535897931
    * (e_t42_im * t222_re - i_t42_re * t222_im) * 1.219391595455261) - (f_re * d
    - f_im * 0.0) * 21765.7808434925)) + (((h_re * -0.0 - g_im *
    -1.6921889868604471E+6) + (g_re * d - h_im * 0.0) * 43531.561686985) -
    3.1415926535897931 * (b_t54_re * d1 - c_t48_re * 0.0) * 0.457271848295723))
                 + (((3.1415926535897931 * (c_t55_re * d1 - c_t43_re_tmp_tmp *
    0.0) * -1.219391595455261 + 3.1415926535897931 * (c_t44_re_tmp_tmp * d1 -
    t55_re_tmp * 0.0) * 0.457271848295723) + 3.1415926535897931 *
                     (d_t43_re_tmp_tmp * d1 - e_t48_re_tmp * 0.0) *
                     0.457271848295723) - 3.1415926535897931 * (t50_re_tmp_tmp *
    d1 - t111 * 0.0) * 0.60969579772763072)) + (((3.1415926535897931 *
    (e_t44_re_tmp * t222_re - i_t44_im_tmp * t222_im) * -0.914543696591446 -
    3.1415926535897931 * (d_t54_im_tmp * d1 - t58_re_tmp_tmp * 0.0) *
    0.914543696591446) + (i_re * 0.0 - i_im * 106.65232613311321)) + (j_re * 0.0
    - j_im * 284.40620302163529))) + (((t93_im_tmp * -0.0 - t73_re_tmp_tmp_tmp *
    -106.65232613311321) - (b_t93_re_tmp * 0.0 - t54_re_tmp_tmp *
    106.65232613311321)) + (re_tmp_tmp_tmp * 0.0 - e_t54_re_tmp *
    142.2031015108177))) + ((((d_t54_re_tmp * 0.0 - f_t48_re_tmp *
    213.3046522662265) + 3.1415926535897931 * (b_t44_re_tmp_tmp_tmp * d1 -
    t49_im_tmp * 0.0) * 1.3718155448871689) - 3.1415926535897931 * (c_t48_re_tmp
    * d1 - i_t48_re * 0.0) * 0.914543696591446) - (t56_re_tmp_tmp * 0.0 - t115 *
    319.95697839933968))) + ((t103 * d1 - t41_tmp * 0.0) * -24875.178106848569 +
    (t98 * 0.0 - t81 * 213.3046522662265));
  Mzzz->im = ((((((((((((((((((((t48_im * 0.0 + t58_im * d) *
    -0.53348382301167685 + (h_t48_re * 0.0 + c_t54_re * d) * 0.26674191150583842)
    + (t55_re * 0.0 + b_t55_re * d) * 0.26674191150583842) + (d_t54_re * 0.0 +
    d_t55_re * d) * 0.13337095575291921) + ((((t44_re * 0.0 + t44_im * d) *
    0.53348382301167685 - (e_t54_re * 0.0 + f_t54_re * d) * 0.26674191150583842)
    - (b_t44_re * 0.0 + b_t44_im * d) * 0.40011286725875761) + (d_t56_re_tmp *
    0.0 + d_t50_re_tmp * d) * 0.26674191150583842)) + ((((g_t48_re *
    124.4277138219654 + f_t48_re * 0.0) - (e_t48_re * 62.21385691098272 +
    b_t50_re_tmp * 0.0)) - (b_t56_re_tmp * 62.21385691098272 + t70 * 0.0)) -
    (t49_re * 31.10692845549136 + d_t48_re * 0.0))) + ((((t61 *
    -124.4277138219654 + b_t93_re * -0.0) + (t49_re_tmp_tmp * 62.21385691098272
    + t49_re_tmp * 0.0)) + (t55 * 93.320785366474084 + g_t42_re_tmp_tmp * 0.0))
    - (ab_t42_re_tmp * 62.21385691098272 + d_t48_re_tmp * 0.0))) +
    (((3.1415926535897931 * (f_t93_re_tmp * t222_im + e_t93_im_tmp * t222_re) *
       2.4387831909105229 + (t48_re * 0.0 + t54_re * d) * 1.6004514690350311) -
      (d_t54_im * 0.0 + c_t54_im * d) * 0.80022573451751533) - (t54_im * 0.0 +
    t55_im * d) * 0.80022573451751533)) + ((((t50_im * 0.0 + b_t54_im * d) *
    -0.40011286725875761 + (t49 * 0.0 + o_re_tmp * d) * 0.80022573451751533) -
    (d_t48_re_tmp_tmp * 373.28314146589628 + b_t55_im * 0.0)) +
    (c_t48_re_tmp_tmp * 186.6415707329482 + t52_re_tmp_tmp * 0.0))) +
                        ((((t93_im_tmp_tmp * 186.6415707329482 + t93_re_tmp_tmp *
    0.0) + (e_t55_re * 93.320785366474084 + y_t42_re_tmp * 0.0)) -
    (b_t48_re_tmp_tmp * 186.6415707329482 + c_t93_re_tmp * 0.0)) -
    3.1415926535897931 * (t68_re * 0.0 + t68_im * d1) * 0.60969579772763072)) +
                       (((3.1415926535897931 * (c_t42_re_tmp_tmp * 0.0 +
    e_t54_im * d1) * 0.457271848295723 - 3.1415926535897931 * (d_t48_im * 0.0 +
    c_t48_im * d1) * 0.3048478988638153) - 3.1415926535897931 * (t73_re_tmp *
    t222_im + t73_im_tmp * t222_re) * 0.60969579772763072) + 3.1415926535897931 *
                        (d_t68_re_tmp * t97.im + d_t68_im_tmp * t97.re) *
                        0.60969579772763072)) + ((((re * 0.0 + im * d) *
    -29021.041124656658 + (b_re * 0.0 + b_im * d) * 14510.520562328329) + (c_re *
    0.0 + c_im * d) * 14510.520562328329) + (d_re * 0.0 + d_im * d) *
    7255.2602811641664)) + ((((e_re * 0.0 + e_im * d) * -14510.520562328329 +
    (c_t58_re_tmp * 0.0 + e_t42_re_tmp_tmp * d) * 1.2003386017762729) - (t93_re *
    0.0 + d_t42_re_tmp_tmp * d) * 2.0005643362937882) - (t48_re_tmp_tmp *
    279.96235609942232 + b_t54_re_tmp_tmp * 0.0))) + ((((b_t55_re_tmp_tmp *
    466.60392683237041 + t55_re_tmp_tmp * 0.0) + 3.1415926535897931 * (t78 * 0.0
    + b_t44_re_tmp_tmp * d1) * 1.219391595455261) - 3.1415926535897931 *
    (b_t56_re * 0.0 + t73_im * d1) * 0.60969579772763072) + 3.1415926535897931 *
    (t73_re * 0.0 + t50_re * d1) * 0.60969579772763072)) + (((3.1415926535897931
    * (b_t48_im * 0.0 + ai * d1) * 0.3048478988638153 + 3.1415926535897931 *
    (e_t48_im * 0.0 + b_t48_re * d1) * 0.60969579772763072) + 3.1415926535897931
    * (e_t42_im * t222_im + i_t42_re * t222_re) * 1.219391595455261) - (f_re *
    0.0 + f_im * d) * 21765.7808434925)) + (((h_re * -1.6921889868604471E+6 +
    g_im * -0.0) + (g_re * 0.0 + h_im * d) * 43531.561686985) -
    3.1415926535897931 * (b_t54_re * 0.0 + c_t48_re * d1) * 0.457271848295723))
                 + (((3.1415926535897931 * (c_t55_re * 0.0 + c_t43_re_tmp_tmp *
    d1) * -1.219391595455261 + 3.1415926535897931 * (c_t44_re_tmp_tmp * 0.0 +
    t55_re_tmp * d1) * 0.457271848295723) + 3.1415926535897931 *
                     (d_t43_re_tmp_tmp * 0.0 + e_t48_re_tmp * d1) *
                     0.457271848295723) - 3.1415926535897931 * (t50_re_tmp_tmp *
    0.0 + t111 * d1) * 0.60969579772763072)) + (((3.1415926535897931 *
    (e_t44_re_tmp * t222_im + i_t44_im_tmp * t222_re) * -0.914543696591446 -
    3.1415926535897931 * (d_t54_im_tmp * 0.0 + t58_re_tmp_tmp * d1) *
    0.914543696591446) + (i_re * 106.65232613311321 + i_im * 0.0)) + (j_re *
    284.40620302163529 + j_im * 0.0))) + (((t93_im_tmp * -106.65232613311321 +
    t73_re_tmp_tmp_tmp * -0.0) - (b_t93_re_tmp * 106.65232613311321 +
    t54_re_tmp_tmp * 0.0)) + (re_tmp_tmp_tmp * 142.2031015108177 + e_t54_re_tmp *
    0.0))) + ((((d_t54_re_tmp * 213.3046522662265 + f_t48_re_tmp * 0.0) +
                3.1415926535897931 * (b_t44_re_tmp_tmp_tmp * 0.0 + t49_im_tmp *
    d1) * 1.3718155448871689) - 3.1415926535897931 * (c_t48_re_tmp * 0.0 +
    i_t48_re * d1) * 0.914543696591446) - (t56_re_tmp_tmp * 319.95697839933968 +
    t115 * 0.0))) + ((t103 * 0.0 + t41_tmp * d1) * -24875.178106848569 + (t98 *
    213.3046522662265 + t81 * 0.0));
}

static void b_rand(double varargin_2, emxArray_real_T *r)
{
  double b_r;
  double *r_data;
  unsigned int u[2];
  int i;
  int j;
  int k;
  int kk;
  unsigned int mti;
  unsigned int y;
  k = r->size[0] * r->size[1];
  r->size[0] = 1;
  i = (int)varargin_2;
  r->size[1] = (int)varargin_2;
  emxEnsureCapacity_real_T(r, k);
  r_data = r->data;
  for (k = 0; k < i; k++) {
    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    do {
      for (j = 0; j < 2; j++) {
        mti = state[624] + 1U;
        if (state[624] + 1U >= 625U) {
          for (kk = 0; kk < 227; kk++) {
            y = (state[kk] & 2147483648U) | (state[kk + 1] & 2147483647U);
            if ((y & 1U) == 0U) {
              y >>= 1U;
            } else {
              y = y >> 1U ^ 2567483615U;
            }

            state[kk] = state[kk + 397] ^ y;
          }

          for (kk = 0; kk < 396; kk++) {
            y = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
            if ((y & 1U) == 0U) {
              y >>= 1U;
            } else {
              y = y >> 1U ^ 2567483615U;
            }

            state[kk + 227] = state[kk] ^ y;
          }

          y = (state[623] & 2147483648U) | (state[0] & 2147483647U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          state[623] = state[396] ^ y;
          mti = 1U;
        }

        y = state[(int)mti - 1];
        state[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        u[j] = y ^ y >> 18U;
      }

      u[0] >>= 5U;
      u[1] >>= 6U;
      b_r = 1.1102230246251565E-16 * ((double)u[0] * 6.7108864E+7 + (double)u[1]);
    } while (b_r == 0.0);

    r_data[k] = b_r;
  }
}

static void b_sqrt(creal_T *x)
{
  double absxi;
  double absxr;
  double xi;
  double xr;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      absxi = 0.0;
      xr = sqrt(-xr);
    } else {
      absxi = sqrt(xr);
      xr = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      absxi = sqrt(-xi / 2.0);
      xr = -absxi;
    } else {
      absxi = sqrt(xi / 2.0);
      xr = absxi;
    }
  } else if (rtIsNaN(xr)) {
    absxi = xr;
  } else if (rtIsNaN(xi)) {
    absxi = xi;
    xr = xi;
  } else if (rtIsInf(xi)) {
    absxi = fabs(xi);
    xr = xi;
  } else if (rtIsInf(xr)) {
    if (xr < 0.0) {
      absxi = 0.0;
      xr = xi * -xr;
    } else {
      absxi = xr;
      xr = 0.0;
    }
  } else {
    absxr = fabs(xr);
    absxi = fabs(xi);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi * 0.5);
      if (absxi > absxr) {
        absxi = sqrt(absxi) * sqrt(absxr / absxi + 1.0);
      } else {
        absxi = sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxi = sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (xr > 0.0) {
      xr = 0.5 * (xi / absxi);
    } else {
      if (xi < 0.0) {
        xr = -absxi;
      } else {
        xr = absxi;
      }

      absxi = 0.5 * (xi / xr);
    }
  }

  x->re = absxi;
  x->im = xr;
}

static void c_eml_rand_mt19937ar_stateful_i(void)
{
  int mti;
  unsigned int r;
  memset(&state[0], 0, 625U * sizeof(unsigned int));
  r = 5489U;
  state[0] = 5489U;
  for (mti = 0; mti < 623; mti++) {
    r = ((r ^ r >> 30U) * 1812433253U + mti) + 1U;
    state[mti + 1] = r;
  }

  state[624] = 624U;
}

static int casyi(const creal_T z, double fnu, creal_T *y)
{
  creal_T ak1;
  double aa;
  double aez;
  double ak;
  double b_atol;
  double b_re;
  double bb;
  double bk;
  double brm;
  double cs1_im;
  double cs1_re;
  double cs2_im;
  double cs2_re;
  double dk_im;
  double dk_re;
  double dnu2;
  double ez_im;
  double ez_re;
  double im;
  double p1_re;
  double r;
  double re;
  double sgn;
  double sqk;
  double tmp_im;
  double tmp_re;
  int i;
  int nz;
  boolean_T errflag;
  boolean_T exitg1;
  nz = 0;
  if (z.im == 0.0) {
    ak1.re = 0.15915494309189535 / z.re;
    ak1.im = 0.0;
  } else if (z.re == 0.0) {
    ak1.re = 0.0;
    ak1.im = -(0.15915494309189535 / z.im);
  } else {
    brm = fabs(z.re);
    dnu2 = fabs(z.im);
    if (brm > dnu2) {
      dnu2 = z.im / z.re;
      r = z.re + dnu2 * z.im;
      ak1.re = (dnu2 * 0.0 + 0.15915494309189535) / r;
      ak1.im = (0.0 - dnu2 * 0.15915494309189535) / r;
    } else if (dnu2 == brm) {
      if (z.re > 0.0) {
        dnu2 = 0.5;
      } else {
        dnu2 = -0.5;
      }

      if (z.im > 0.0) {
        r = 0.5;
      } else {
        r = -0.5;
      }

      ak1.re = (0.15915494309189535 * dnu2 + 0.0 * r) / brm;
      ak1.im = (0.0 * dnu2 - 0.15915494309189535 * r) / brm;
    } else {
      dnu2 = z.re / z.im;
      r = z.im + dnu2 * z.re;
      ak1.re = dnu2 * 0.15915494309189535 / r;
      ak1.im = (dnu2 * 0.0 - 0.15915494309189535) / r;
    }
  }

  b_sqrt(&ak1);
  if (fabs(z.re) > 701.61506577445994) {
    nz = -1;
    y->re = rtNaN;
    y->im = 0.0;
  } else {
    dnu2 = fnu + fnu;
    if (z.im == 0.0) {
      tmp_re = exp(z.re);
      tmp_im = 0.0;
    } else if (rtIsInf(z.im) && rtIsInf(z.re) && (z.re < 0.0)) {
      tmp_re = 0.0;
      tmp_im = 0.0;
    } else {
      r = exp(z.re / 2.0);
      tmp_re = r * (r * cos(z.im));
      tmp_im = r * (r * sin(z.im));
    }

    re = ak1.re * tmp_re - ak1.im * tmp_im;
    im = ak1.re * tmp_im + ak1.im * tmp_re;
    r = 0.0;
    if (dnu2 > 4.7170688552396617E-153) {
      r = dnu2 * dnu2;
    }

    ez_re = 8.0 * z.re;
    ez_im = 8.0 * z.im;
    aez = 8.0 * rt_hypotd_snf(z.re, z.im);
    if (z.im != 0.0) {
      cospiAndSinpi(fnu - (double)(int)fnu, &bk, &p1_re);
      if (z.im < 0.0) {
        bk = -bk;
      }

      if ((int)fnu != 0) {
        bk = -bk;
      } else {
        p1_re = -p1_re;
      }
    } else {
      p1_re = 0.0;
      bk = 0.0;
    }

    sqk = r - 1.0;
    b_atol = 2.2204460492503131E-16 / aez * fabs(r - 1.0);
    sgn = 1.0;
    cs1_re = 1.0;
    cs1_im = 0.0;
    cs2_re = 1.0;
    cs2_im = 0.0;
    tmp_re = 1.0;
    tmp_im = 0.0;
    ak = 0.0;
    aa = 1.0;
    bb = aez;
    dk_re = ez_re;
    dk_im = ez_im;
    errflag = true;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 45)) {
      tmp_re *= sqk;
      tmp_im *= sqk;
      if (dk_im == 0.0) {
        if (tmp_im == 0.0) {
          b_re = tmp_re / dk_re;
          tmp_im = 0.0;
        } else if (tmp_re == 0.0) {
          b_re = 0.0;
          tmp_im /= dk_re;
        } else {
          b_re = tmp_re / dk_re;
          tmp_im /= dk_re;
        }
      } else if (dk_re == 0.0) {
        if (tmp_re == 0.0) {
          b_re = tmp_im / dk_im;
          tmp_im = 0.0;
        } else if (tmp_im == 0.0) {
          b_re = 0.0;
          tmp_im = -(tmp_re / dk_im);
        } else {
          b_re = tmp_im / dk_im;
          tmp_im = -(tmp_re / dk_im);
        }
      } else {
        brm = fabs(dk_re);
        dnu2 = fabs(dk_im);
        if (brm > dnu2) {
          dnu2 = dk_im / dk_re;
          r = dk_re + dnu2 * dk_im;
          b_re = (tmp_re + dnu2 * tmp_im) / r;
          tmp_im = (tmp_im - dnu2 * tmp_re) / r;
        } else if (dnu2 == brm) {
          if (dk_re > 0.0) {
            dnu2 = 0.5;
          } else {
            dnu2 = -0.5;
          }

          if (dk_im > 0.0) {
            r = 0.5;
          } else {
            r = -0.5;
          }

          b_re = (tmp_re * dnu2 + tmp_im * r) / brm;
          tmp_im = (tmp_im * dnu2 - tmp_re * r) / brm;
        } else {
          dnu2 = dk_re / dk_im;
          r = dk_im + dnu2 * dk_re;
          b_re = (dnu2 * tmp_re + tmp_im) / r;
          tmp_im = (dnu2 * tmp_im - tmp_re) / r;
        }
      }

      tmp_re = b_re;
      cs2_re += b_re;
      cs2_im += tmp_im;
      sgn = -sgn;
      cs1_re += b_re * sgn;
      cs1_im += tmp_im * sgn;
      dk_re += ez_re;
      dk_im += ez_im;
      aa = aa * fabs(sqk) / bb;
      bb += aez;
      ak += 8.0;
      sqk -= ak;
      if (aa <= b_atol) {
        errflag = false;
        exitg1 = true;
      } else {
        i++;
      }
    }

    if (errflag) {
      nz = -2;
    } else {
      if (z.re + z.re < 701.61506577445994) {
        tmp_re = -2.0 * z.re;
        tmp_im = -2.0 * z.im;
        if (tmp_im == 0.0) {
          tmp_re = exp(tmp_re);
          tmp_im = 0.0;
        } else if (rtIsInf(tmp_im) && rtIsInf(tmp_re) && (tmp_re < 0.0)) {
          tmp_re = 0.0;
          tmp_im = 0.0;
        } else {
          r = exp(tmp_re / 2.0);
          tmp_re = r * (r * cos(tmp_im));
          tmp_im = r * (r * sin(tmp_im));
        }

        b_re = tmp_re * cs2_re - tmp_im * cs2_im;
        dnu2 = tmp_re * cs2_im + tmp_im * cs2_re;
        cs1_re += b_re * p1_re - dnu2 * bk;
        cs1_im += b_re * bk + dnu2 * p1_re;
      }

      y->re = cs1_re * re - cs1_im * im;
      y->im = cs1_re * im + cs1_im * re;
    }
  }

  return nz;
}

static void cbesj(const creal_T z, double fnu, creal_T *cy, int *nz, int *ierr)
{
  creal_T zn;
  double acz;
  double ak;
  double ascle;
  double az;
  double az_tmp;
  double b_atol;
  double b_carg;
  double coef_im;
  double coef_re;
  double crsc_re;
  double csgn_im;
  double cz_im;
  double cz_re;
  double hz_im;
  double hz_re;
  double re;
  double rs;
  double s;
  double s1_im;
  double s1_re;
  int nw;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T iflag;
  *ierr = 0;
  az = rt_hypotd_snf(z.re, z.im);
  if (az > 1.0737418235E+9) {
    *ierr = 4;
  } else if (az > 32767.999992370605) {
    *ierr = 3;
  }

  cospiAndSinpi(fnu * 0.5, &b_carg, &csgn_im);
  zn.re = -z.re * 0.0 - (-z.im);
  zn.im = -z.re + -z.im * 0.0;
  if (z.im < 0.0) {
    zn.re = -zn.re;
    zn.im = -zn.im;
    csgn_im = -csgn_im;
  }

  cy->re = 0.0;
  cy->im = 0.0;
  az_tmp = rt_hypotd_snf(zn.re, zn.im);
  guard1 = false;
  if ((az_tmp <= 2.0) || (az_tmp * az_tmp * 0.25 <= fnu + 1.0)) {
    nw = 0;
    if (az_tmp == 0.0) {
      if (fnu == 0.0) {
        cy->re = 1.0;
        cy->im = 0.0;
      }
    } else {
      crsc_re = 1.0;
      iflag = false;
      if (az_tmp < 2.2250738585072014E-305) {
        nw = 1;
        if (fnu == 0.0) {
          nw = 0;
          cy->re = 1.0;
          cy->im = 0.0;
        }
      } else {
        hz_re = 0.5 * zn.re;
        hz_im = 0.5 * zn.im;
        if (az_tmp > 4.7170688552396617E-153) {
          cz_re = hz_re * hz_re - hz_im * hz_im;
          az = hz_re * hz_im;
          cz_im = az + az;
          acz = rt_hypotd_snf(cz_re, cz_im);
        } else {
          cz_re = 0.0;
          cz_im = 0.0;
          acz = 0.0;
        }

        if (hz_im == 0.0) {
          if (hz_re < 0.0) {
            hz_re = log(fabs(hz_re));
            hz_im = 3.1415926535897931;
          } else {
            hz_re = log(hz_re);
            hz_im = 0.0;
          }
        } else if ((fabs(hz_re) > 8.9884656743115785E+307) || (fabs(hz_im) >
                    8.9884656743115785E+307)) {
          az = hz_re;
          hz_re = log(rt_hypotd_snf(hz_re / 2.0, hz_im / 2.0)) +
            0.69314718055994529;
          hz_im = rt_atan2d_snf(hz_im, az);
        } else {
          az = hz_re;
          hz_re = log(rt_hypotd_snf(hz_re, hz_im));
          hz_im = rt_atan2d_snf(hz_im, az);
        }

        az = ((fnu + 1.0) - 1.0) + 1.0;
        gammaln(&az);
        hz_re = hz_re * ((fnu + 1.0) - 1.0) - az;
        hz_im *= (fnu + 1.0) - 1.0;
        if (hz_re > -701.61506577445994) {
          ascle = 0.0;
          if (hz_re <= -665.56491761372422) {
            iflag = true;
            crsc_re = 2.2204460492503131E-16;
            ascle = 1.0020841800044864E-289;
          }

          az = exp(hz_re);
          if (iflag) {
            az /= 2.2204460492503131E-16;
          }

          coef_re = az * cos(hz_im);
          coef_im = az * sin(hz_im);
          b_atol = 2.2204460492503131E-16 * acz / (((fnu + 1.0) - 1.0) + 1.0);
          s1_re = 1.0;
          s1_im = 0.0;
          if (!(acz < 2.2204460492503131E-16 * (fnu + 1.0))) {
            hz_re = 1.0;
            hz_im = 0.0;
            ak = (fnu + 1.0) + 2.0;
            s = fnu + 1.0;
            az = 2.0;
            do {
              rs = 1.0 / s;
              re = hz_re * cz_re - hz_im * cz_im;
              hz_im = hz_re * cz_im + hz_im * cz_re;
              hz_re = rs * re;
              hz_im *= rs;
              s1_re += hz_re;
              s1_im += hz_im;
              s += ak;
              ak += 2.0;
              az = az * acz * rs;
            } while (!!(az > b_atol));
          }

          hz_re = s1_re * coef_re - s1_im * coef_im;
          hz_im = s1_re * coef_im + s1_im * coef_re;
          guard2 = false;
          if (iflag) {
            az = fabs(hz_re);
            coef_re = fabs(hz_im);
            if (az > coef_re) {
              coef_im = coef_re;
              coef_re = az;
            } else {
              coef_im = az;
            }

            if ((!(coef_im <= ascle)) || (!(coef_re < coef_im /
                  2.2204460492503131E-16))) {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }

          if (guard2) {
            cy->re = hz_re * crsc_re - hz_im * 0.0;
            cy->im = hz_re * 0.0 + hz_im * crsc_re;
          }
        } else {
          nw = 1;
          if (acz > (fnu + 1.0) - 1.0) {
            nw = -1;
          }
        }
      }
    }

    if (nw < 0) {
      *nz = 1;
    } else {
      *nz = nw;
    }

    if ((1 - *nz != 0) && (nw < 0)) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    if (az_tmp < 21.784271729432426) {
      nw = cmlri(zn, fnu, cy);
      if (nw < 0) {
        if (nw == -2) {
          *nz = -2;
        } else {
          *nz = -1;
        }
      } else {
        *nz = 0;
      }
    } else {
      nw = casyi(zn, fnu, cy);
      if (nw < 0) {
        if (nw == -2) {
          *nz = -2;
        } else {
          *nz = -1;
        }
      } else {
        *nz = 0;
      }
    }
  }

  if (*nz < 0) {
    if (*nz == -2) {
      *nz = 0;
      *ierr = 5;
      cy->re = rtNaN;
      cy->im = rtNaN;
    } else {
      *nz = 0;
      *ierr = 2;
      cy->re = rtInf;
      cy->im = 0.0;
    }
  } else if (*nz != 1) {
    zn = *cy;
    if (fmax(fabs(cy->re), fabs(cy->im)) <= 1.0020841800044864E-289) {
      zn.re = 4.503599627370496E+15 * cy->re;
      zn.im = 4.503599627370496E+15 * cy->im;
      az = 2.2204460492503131E-16;
    } else {
      az = 1.0;
    }

    re = zn.re * b_carg - zn.im * csgn_im;
    hz_im = zn.re * csgn_im + zn.im * b_carg;
    cy->re = az * re;
    cy->im = az * hz_im;
  }
}

static int cmlri(const creal_T z, double fnu, creal_T *y)
{
  double ack;
  double ak;
  double az;
  double bk;
  double ck_im;
  double ck_re;
  double fixfnu;
  double flooraz;
  double fnf;
  double p1_im;
  double p1_re;
  double p2_im;
  double p2_re;
  double pt_im;
  double pt_re;
  double rho2;
  double rz_im;
  double rz_re;
  double s_im;
  double tst;
  int i;
  int icounter;
  int itime;
  int kcounter;
  int nz;
  boolean_T earlyExit;
  boolean_T exitg1;
  boolean_T guard1 = false;
  nz = 0;
  az = rt_hypotd_snf(z.re, z.im);
  flooraz = floor(az);
  fixfnu = trunc(fnu);
  if (z.im == 0.0) {
    ck_re = (flooraz + 1.0) / z.re;
    ck_im = 0.0;
    rz_re = 2.0 / z.re;
    rz_im = 0.0;
  } else if (z.re == 0.0) {
    ck_re = 0.0;
    ck_im = -((flooraz + 1.0) / z.im);
    rz_re = 0.0;
    rz_im = -(2.0 / z.im);
  } else {
    ack = fabs(z.re);
    rho2 = fabs(z.im);
    if (ack > rho2) {
      tst = z.im / z.re;
      bk = z.re + tst * z.im;
      ck_re = ((flooraz + 1.0) + tst * 0.0) / bk;
      ck_im = (0.0 - tst * (flooraz + 1.0)) / bk;
      tst = z.im / z.re;
      bk = z.re + tst * z.im;
      rz_re = (tst * 0.0 + 2.0) / bk;
      rz_im = (0.0 - tst * 2.0) / bk;
    } else {
      if (rho2 == ack) {
        if (z.re > 0.0) {
          tst = 0.5;
        } else {
          tst = -0.5;
        }

        if (z.im > 0.0) {
          bk = 0.5;
        } else {
          bk = -0.5;
        }

        ck_re = ((flooraz + 1.0) * tst + 0.0 * bk) / ack;
        ck_im = (0.0 * tst - (flooraz + 1.0) * bk) / ack;
      } else {
        tst = z.re / z.im;
        bk = z.im + tst * z.re;
        ck_re = tst * (flooraz + 1.0) / bk;
        ck_im = (tst * 0.0 - (flooraz + 1.0)) / bk;
      }

      if (rho2 == ack) {
        if (z.re > 0.0) {
          tst = 0.5;
        } else {
          tst = -0.5;
        }

        if (z.im > 0.0) {
          bk = 0.5;
        } else {
          bk = -0.5;
        }

        rz_re = (2.0 * tst + 0.0 * bk) / ack;
        rz_im = (0.0 * tst - 2.0 * bk) / ack;
      } else {
        tst = z.re / z.im;
        bk = z.im + tst * z.re;
        rz_re = tst * 2.0 / bk;
        rz_im = (tst * 0.0 - 2.0) / bk;
      }
    }
  }

  p1_re = 0.0;
  p1_im = 0.0;
  p2_re = 1.0;
  p2_im = 0.0;
  ack = ((flooraz + 1.0) + 1.0) / az;
  ack += sqrt(ack * ack - 1.0);
  rho2 = ack * ack;
  tst = (rho2 + rho2) / ((rho2 - 1.0) * (ack - 1.0)) / 2.2204460492503131E-16;
  ak = flooraz + 1.0;
  earlyExit = true;
  icounter = 1;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 80)) {
    icounter++;
    pt_re = p2_re;
    pt_im = p2_im;
    ack = ck_re * p2_re - ck_im * p2_im;
    rho2 = ck_re * p2_im + ck_im * p2_re;
    p2_re = p1_re - ack;
    p2_im = p1_im - rho2;
    p1_re = pt_re;
    p1_im = pt_im;
    ck_re += rz_re;
    ck_im += rz_im;
    if (rt_hypotd_snf(p2_re, p2_im) > tst * ak * ak) {
      earlyExit = false;
      exitg1 = true;
    } else {
      ak++;
      i++;
    }
  }

  if (earlyExit) {
    nz = -2;
  } else {
    kcounter = 1;
    guard1 = false;
    if ((int)fixfnu >= (int)flooraz) {
      p1_re = 0.0;
      p1_im = 0.0;
      p2_re = 1.0;
      p2_im = 0.0;
      if (z.im == 0.0) {
        ck_re = ((double)(int)fixfnu + 1.0) / z.re;
        ck_im = 0.0;
      } else if (z.re == 0.0) {
        if ((double)(int)fixfnu + 1.0 == 0.0) {
          ck_re = 0.0 / z.im;
          ck_im = 0.0;
        } else {
          ck_re = 0.0;
          ck_im = -(((double)(int)fixfnu + 1.0) / z.im);
        }
      } else {
        ack = fabs(z.re);
        rho2 = fabs(z.im);
        if (ack > rho2) {
          tst = z.im / z.re;
          bk = z.re + tst * z.im;
          ck_re = (((double)(int)fixfnu + 1.0) + tst * 0.0) / bk;
          ck_im = (0.0 - tst * ((double)(int)fixfnu + 1.0)) / bk;
        } else if (rho2 == ack) {
          if (z.re > 0.0) {
            tst = 0.5;
          } else {
            tst = -0.5;
          }

          if (z.im > 0.0) {
            bk = 0.5;
          } else {
            bk = -0.5;
          }

          ck_re = (((double)(int)fixfnu + 1.0) * tst + 0.0 * bk) / ack;
          ck_im = (0.0 * tst - ((double)(int)fixfnu + 1.0) * bk) / ack;
        } else {
          tst = z.re / z.im;
          bk = z.im + tst * z.re;
          ck_re = tst * ((double)(int)fixfnu + 1.0) / bk;
          ck_im = (tst * 0.0 - ((double)(int)fixfnu + 1.0)) / bk;
        }
      }

      tst = sqrt(((double)(int)fixfnu + 1.0) / az / 2.2204460492503131E-16);
      itime = 1;
      earlyExit = true;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 80)) {
        kcounter++;
        pt_re = p2_re;
        pt_im = p2_im;
        ack = ck_re * p2_re - ck_im * p2_im;
        rho2 = ck_re * p2_im + ck_im * p2_re;
        p2_re = p1_re - ack;
        p2_im = p1_im - rho2;
        p1_re = pt_re;
        p1_im = pt_im;
        ck_re += rz_re;
        ck_im += rz_im;
        rho2 = rt_hypotd_snf(p2_re, p2_im);
        if (rho2 >= tst * ak * ak) {
          if (itime == 2) {
            earlyExit = false;
            exitg1 = true;
          } else {
            ack = rt_hypotd_snf(ck_re, ck_im);
            ack = fmin(ack + sqrt(ack * ack - 1.0), rho2 / rt_hypotd_snf(pt_re,
                        pt_im));
            tst *= sqrt(ack / (ack * ack - 1.0));
            itime = 2;
            i++;
          }
        } else {
          i++;
        }
      }

      if (earlyExit) {
        nz = -2;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      itime = icounter + (int)flooraz;
      icounter = kcounter + (int)fixfnu;
      if (itime >= icounter) {
        icounter = itime;
      }

      ak = icounter;
      p1_re = 0.0;
      p1_im = 0.0;
      p2_re = 1.0020841800044864E-289;
      p2_im = 0.0;
      fnf = fnu - fixfnu;
      az = fnf + fnf;
      tst = ((double)icounter + az) + 1.0;
      gammaln(&tst);
      ack = (double)icounter + 1.0;
      gammaln(&ack);
      rho2 = az + 1.0;
      gammaln(&rho2);
      bk = exp((tst - ack) - rho2);
      flooraz = 0.0;
      s_im = 0.0;
      icounter -= (int)fixfnu;
      for (i = 0; i < icounter; i++) {
        pt_re = p2_re;
        pt_im = p2_im;
        rho2 = ak + fnf;
        tst = rho2 * rz_re;
        rho2 *= rz_im;
        ack = tst * p2_re - rho2 * p2_im;
        rho2 = tst * p2_im + rho2 * p2_re;
        p2_re = p1_re + ack;
        p2_im = p1_im + rho2;
        p1_re = pt_re;
        p1_im = pt_im;
        ack = bk * (1.0 - az / (ak + az));
        flooraz += (ack + bk) * pt_re;
        s_im += (ack + bk) * pt_im;
        bk = ack;
        ak--;
      }

      y->re = p2_re;
      y->im = p2_im;
      if ((int)fixfnu > 0) {
        pt_re = p2_re;
        pt_im = p2_im;
        tst = (ak + fnf) * rz_re;
        rho2 = (ak + fnf) * rz_im;
        ack = tst * p2_re - rho2 * p2_im;
        rho2 = tst * p2_im + rho2 * p2_re;
        p2_re = p1_re + ack;
        p2_im = p1_im + rho2;
        rho2 = bk * (1.0 - az / (ak + az)) + bk;
        flooraz += rho2 * pt_re;
        s_im += rho2 * pt_im;
      }

      if (rz_im == 0.0) {
        if (rz_re < 0.0) {
          ck_re = log(fabs(rz_re));
          ck_im = 3.1415926535897931;
        } else {
          ck_re = log(rz_re);
          ck_im = 0.0;
        }
      } else if ((fabs(rz_re) > 8.9884656743115785E+307) || (fabs(rz_im) >
                  8.9884656743115785E+307)) {
        ck_re = log(rt_hypotd_snf(rz_re / 2.0, rz_im / 2.0)) +
          0.69314718055994529;
        ck_im = rt_atan2d_snf(rz_im, rz_re);
      } else {
        ck_re = log(rt_hypotd_snf(rz_re, rz_im));
        ck_im = rt_atan2d_snf(rz_im, rz_re);
      }

      rho2 = -fnf * ck_re - -0.0 * ck_im;
      ack = -fnf * ck_im + -0.0 * ck_re;
      tst = fnf + 1.0;
      gammaln(&tst);
      ck_re = (rho2 + z.re) - tst;
      ck_im = ack + z.im;
      p2_re += flooraz;
      p2_im += s_im;
      p1_re = 1.0 / rt_hypotd_snf(p2_re, p2_im);
      if (ck_im == 0.0) {
        ck_re = exp(ck_re);
        ck_im = 0.0;
      } else if (rtIsInf(ck_im) && rtIsInf(ck_re) && (ck_re < 0.0)) {
        ck_re = 0.0;
        ck_im = 0.0;
      } else {
        rho2 = exp(ck_re / 2.0);
        ck_re = rho2 * (rho2 * cos(ck_im));
        ck_im = rho2 * (rho2 * sin(ck_im));
      }

      ack = ck_re * p1_re - ck_im * 0.0;
      ck_im = ck_re * 0.0 + ck_im * p1_re;
      rho2 = p2_re * p1_re + p2_im * 0.0;
      p2_im = p2_re * 0.0 - p2_im * p1_re;
      ck_re = ack * rho2 - ck_im * p2_im;
      ck_im = ack * p2_im + ck_im * rho2;
      ack = y->re * ck_im + y->im * ck_re;
      y->re = y->re * ck_re - y->im * ck_im;
      y->im = ack;
    }
  }

  return nz;
}

static void cospiAndSinpi(double x, double *c, double *s)
{
  double r;
  boolean_T negateSinpi;
  if (x < 0.0) {
    x = -x;
    negateSinpi = true;
  } else {
    negateSinpi = false;
  }

  if (x < 0.25) {
    *c = cos(x * 3.1415926535897931);
    *s = sin(x * 3.1415926535897931);
  } else {
    r = x - 2.0 * floor(x / 2.0);
    if (r < 0.25) {
      r *= 3.1415926535897931;
      *c = cos(r);
      *s = sin(r);
    } else if (r < 0.75) {
      r = 0.5 - r;
      r *= 3.1415926535897931;
      *c = sin(r);
      *s = cos(r);
    } else if (r < 1.25) {
      r = 1.0 - r;
      r *= 3.1415926535897931;
      *c = -cos(r);
      *s = sin(r);
    } else if (r < 1.75) {
      r -= 1.5;
      r *= 3.1415926535897931;
      *c = sin(r);
      *s = -cos(r);
    } else {
      r -= 2.0;
      r *= 3.1415926535897931;
      *c = cos(r);
      *s = sin(r);
    }
  }

  if (negateSinpi) {
    *s = -*s;
  }
}

static double fminsearch(double funfcn_workspace_P_x, double
  funfcn_workspace_P_y, double funfcn_workspace_P_z, const Transducer3D
  funfcn_workspace_Tarray_active[1442], double funfcn_workspace_numT,
  emxArray_real_T *x)
{
  emxArray_int32_T *idx;
  emxArray_int32_T *idxb;
  emxArray_real_T *fv;
  emxArray_real_T *fvt;
  emxArray_real_T *v;
  emxArray_real_T *xbar;
  emxArray_real_T *xe;
  emxArray_real_T *xr;
  double absx_tmp;
  double cfv;
  double fun_evals;
  double fval;
  double itercount;
  double maxfun;
  double maxfv;
  double maxiter;
  double maxv;
  double *fv_data;
  double *fvt_data;
  double *v_data;
  double *x_data;
  double *xbar_data;
  double *xe_data;
  double *xr_data;
  int b_exponent;
  int b_firstCol;
  int c_exponent;
  int colIdx;
  int exponent;
  int firstCol;
  int i;
  int j;
  int k;
  int lastCol;
  int ncols;
  int nrows;
  int nstride;
  int nvars;
  int *idx_data;
  int *idxb_data;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T p;
  x_data = x->data;
  emxInit_real_T(&v, 2);
  emxInit_real_T(&fv, 2);
  nvars = x->size[1];
  maxfun = 200.0 * (double)x->size[1];
  maxiter = 200.0 * (double)x->size[1];
  cfv = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
                        funfcn_workspace_P_z, funfcn_workspace_Tarray_active,
                        funfcn_workspace_numT, x);
  i = v->size[0] * v->size[1];
  v->size[0] = x->size[1];
  v->size[1] = x->size[1] + 1;
  emxEnsureCapacity_real_T(v, i);
  v_data = v->data;
  i = fv->size[0] * fv->size[1];
  fv->size[0] = 1;
  fv->size[1] = x->size[1] + 1;
  emxEnsureCapacity_real_T(fv, i);
  fv_data = fv->data;
  fv_data[0] = cfv;
  i = x->size[1];
  for (k = 0; k <= i; k++) {
    for (j = 0; j < nvars; j++) {
      v_data[(int)(((double)j + 1.0) + (double)nvars * (((double)k + 1.0) - 1.0))
        - 1] = x_data[j];
    }
  }

  emxInit_real_T(&xr, 2);
  emxInit_real_T(&xe, 2);
  i = xr->size[0] * xr->size[1];
  xr->size[0] = 1;
  xr->size[1] = x->size[1];
  emxEnsureCapacity_real_T(xr, i);
  xr_data = xr->data;
  i = xe->size[0] * xe->size[1];
  xe->size[0] = 1;
  xe->size[1] = x->size[1];
  emxEnsureCapacity_real_T(xe, i);
  xe_data = xe->data;
  nrows = x->size[1] - 1;
  for (k = 0; k <= nrows; k++) {
    colIdx = nvars * (k + 1);
    v_data[k + colIdx] = 1.05 * x_data[k];
    for (j = 0; j <= nrows; j++) {
      xr_data[j] = v_data[j + colIdx];
    }

    fv_data[k + 1] = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
      funfcn_workspace_P_z, funfcn_workspace_Tarray_active,
      funfcn_workspace_numT, xr);
  }

  emxInit_int32_T(&idx, 2);
  emxInit_real_T(&xbar, 1);
  sortIdx(fv, idx);
  idx_data = idx->data;
  itercount = 1.0;
  fun_evals = (double)x->size[1] + 1.0;
  i = xbar->size[0];
  xbar->size[0] = x->size[1];
  emxEnsureCapacity_real_T(xbar, i);
  xbar_data = xbar->data;
  lastCol = x->size[1] * (idx_data[idx->size[1] - 1] - 1);
  firstCol = x->size[1] * (idx_data[0] - 1);
  emxInit_real_T(&fvt, 2);
  emxInit_int32_T(&idxb, 2);
  exitg1 = false;
  while ((!exitg1) && ((fun_evals < maxfun) && (itercount < maxiter))) {
    maxfv = 0.0;
    i = fv->size[1];
    for (k = 0; k <= i - 2; k++) {
      cfv = fabs(fv_data[idx_data[0] - 1] - fv_data[idx_data[k + 1] - 1]);
      if (cfv > maxfv) {
        maxfv = cfv;
      }
    }

    absx_tmp = fv_data[idx_data[0] - 1];
    cfv = fabs(absx_tmp);
    if ((!rtIsInf(cfv)) && (!rtIsNaN(cfv))) {
      if (cfv <= 2.2250738585072014E-308) {
        cfv = 4.94065645841247E-324;
      } else {
        frexp(cfv, &exponent);
        cfv = ldexp(1.0, exponent - 53);
      }
    } else {
      cfv = rtNaN;
    }

    cfv *= 10.0;
    if ((!rtIsInf(cfv)) && (!rtIsNaN(cfv))) {
      if (cfv <= 2.2250738585072014E-308) {
        cfv = 4.94065645841247E-324;
      } else {
        frexp(cfv, &b_exponent);
        cfv = ldexp(1.0, b_exponent - 53);
      }
    } else {
      cfv = rtNaN;
    }

    if (maxfv > fmax(0.0001, 10.0 * cfv)) {
      p = false;
    } else {
      nstride = v->size[0];
      ncols = v->size[1];
      maxv = 0.0;
      b_firstCol = (idx_data[0] - 1) * v->size[0];
      for (j = 2; j <= ncols; j++) {
        colIdx = (idx_data[j - 1] - 1) * nstride;
        for (k = 0; k < nstride; k++) {
          cfv = fabs(v_data[k + b_firstCol] - v_data[k + colIdx]);
          if (cfv > maxv) {
            maxv = cfv;
          }
        }
      }

      maxfv = v_data[b_firstCol];
      for (k = 2; k <= nstride; k++) {
        cfv = v_data[(k + b_firstCol) - 1];
        if (cfv > maxfv) {
          maxfv = cfv;
        }
      }

      cfv = fabs(maxfv);
      if ((!rtIsInf(cfv)) && (!rtIsNaN(cfv))) {
        if (cfv <= 2.2250738585072014E-308) {
          cfv = 4.94065645841247E-324;
        } else {
          frexp(cfv, &c_exponent);
          cfv = ldexp(1.0, c_exponent - 53);
        }
      } else {
        cfv = rtNaN;
      }

      p = (maxv <= fmax(0.0001, 10.0 * cfv));
    }

    if (!p) {
      for (k = 0; k <= nrows; k++) {
        xbar_data[k] = v_data[k + firstCol];
      }

      for (k = 2; k <= nrows + 1; k++) {
        colIdx = nvars * (idx_data[k - 1] - 1);
        for (j = 0; j <= nrows; j++) {
          xbar_data[j] += v_data[j + colIdx];
        }
      }

      for (k = 0; k <= nrows; k++) {
        cfv = xbar_data[k] / (double)nvars;
        xbar_data[k] = cfv;
        xr_data[k] = 2.0 * cfv - v_data[k + lastCol];
      }

      maxfv = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
        funfcn_workspace_P_z, funfcn_workspace_Tarray_active,
        funfcn_workspace_numT, xr);
      fun_evals++;
      guard1 = false;
      guard2 = false;
      if (maxfv < absx_tmp) {
        for (k = 0; k <= nrows; k++) {
          xe_data[k] = 3.0 * xbar_data[k] - 2.0 * v_data[k + lastCol];
        }

        cfv = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
                              funfcn_workspace_P_z,
                              funfcn_workspace_Tarray_active,
                              funfcn_workspace_numT, xe);
        fun_evals++;
        if (cfv < maxfv) {
          for (k = 0; k <= nrows; k++) {
            v_data[k + lastCol] = xe_data[k];
          }

          fv_data[idx_data[idx->size[1] - 1] - 1] = cfv;
        } else {
          for (k = 0; k <= nrows; k++) {
            v_data[k + lastCol] = xr_data[k];
          }

          fv_data[idx_data[idx->size[1] - 1] - 1] = maxfv;
        }

        guard1 = true;
      } else if (maxfv < fv_data[idx_data[nvars - 1] - 1]) {
        for (k = 0; k <= nrows; k++) {
          v_data[k + lastCol] = xr_data[k];
        }

        fv_data[idx_data[idx->size[1] - 1] - 1] = maxfv;
        guard1 = true;
      } else if (maxfv < fv_data[idx_data[idx->size[1] - 1] - 1]) {
        for (k = 0; k <= nrows; k++) {
          x_data[k] = 1.5 * xbar_data[k] - 0.5 * v_data[k + lastCol];
        }

        cfv = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
                              funfcn_workspace_P_z,
                              funfcn_workspace_Tarray_active,
                              funfcn_workspace_numT, x);
        fun_evals++;
        if (cfv <= maxfv) {
          for (k = 0; k <= nrows; k++) {
            v_data[k + lastCol] = x_data[k];
          }

          fv_data[idx_data[idx->size[1] - 1] - 1] = cfv;
          guard1 = true;
        } else {
          guard2 = true;
        }
      } else {
        for (k = 0; k <= nrows; k++) {
          x_data[k] = 0.5 * xbar_data[k] + 0.5 * v_data[k + lastCol];
        }

        cfv = solver_anonFcn1(funfcn_workspace_P_x, funfcn_workspace_P_y,
                              funfcn_workspace_P_z,
                              funfcn_workspace_Tarray_active,
                              funfcn_workspace_numT, x);
        fun_evals++;
        if (cfv < fv_data[idx_data[idx->size[1] - 1] - 1]) {
          for (k = 0; k <= nrows; k++) {
            v_data[k + lastCol] = x_data[k];
          }

          fv_data[idx_data[idx->size[1] - 1] - 1] = cfv;
          guard1 = true;
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        i = nvars + 1;
        for (k = 2; k <= i; k++) {
          nstride = idx_data[k - 1] - 1;
          colIdx = nvars * nstride - 1;
          for (j = 0; j <= nrows; j++) {
            ncols = j + firstCol;
            b_firstCol = (j + colIdx) + 1;
            v_data[b_firstCol] = v_data[ncols] + 0.5 * (v_data[b_firstCol] -
              v_data[ncols]);
            x_data[j] = v_data[b_firstCol];
          }

          fv_data[nstride] = solver_anonFcn1(funfcn_workspace_P_x,
            funfcn_workspace_P_y, funfcn_workspace_P_z,
            funfcn_workspace_Tarray_active, funfcn_workspace_numT, x);
        }

        fun_evals += (double)nvars;
        i = fvt->size[0] * fvt->size[1];
        fvt->size[0] = 1;
        fvt->size[1] = fv->size[1];
        emxEnsureCapacity_real_T(fvt, i);
        fvt_data = fvt->data;
        i = fv->size[1];
        for (k = 0; k < i; k++) {
          fvt_data[k] = fv_data[idx_data[k] - 1];
        }

        i = idxb->size[0] * idxb->size[1];
        idxb->size[0] = 1;
        idxb->size[1] = idx->size[1];
        emxEnsureCapacity_int32_T(idxb, i);
        idxb_data = idxb->data;
        nstride = idx->size[1];
        for (i = 0; i < nstride; i++) {
          idxb_data[i] = idx_data[i];
        }

        sortIdx(fvt, idx);
        idx_data = idx->data;
        i = idx->size[1];
        for (k = 0; k < i; k++) {
          idx_data[k] = idxb_data[idx_data[k] - 1];
        }
      }

      if (guard1) {
        i = idx->size[1] - 2;
        ncols = idx->size[1];
        for (k = 0; k <= ncols - 2; k++) {
          nstride = i - k;
          b_firstCol = idx_data[nstride + 1];
          if (fv_data[b_firstCol - 1] < fv_data[idx_data[nstride] - 1]) {
            idx_data[nstride + 1] = idx_data[nstride];
            idx_data[nstride] = b_firstCol;
          }
        }
      }

      itercount++;
      lastCol = nvars * (idx_data[idx->size[1] - 1] - 1);
      firstCol = nvars * (idx_data[0] - 1);
    } else {
      exitg1 = true;
    }
  }

  emxFree_int32_T(&idxb);
  emxFree_real_T(&fvt);
  emxFree_real_T(&xbar);
  emxFree_real_T(&xe);
  emxFree_real_T(&xr);
  colIdx = nvars * (idx_data[0] - 1);
  for (k = 0; k <= nrows; k++) {
    x_data[k] = v_data[k + colIdx];
  }

  emxFree_real_T(&v);
  fval = fv_data[idx_data[0] - 1];
  emxFree_int32_T(&idx);
  emxFree_real_T(&fv);
  return fval;
}

static void gammaln(double *x)
{
  static const double table100[100] = { 0.0, 0.0, 0.69314718055994529,
    1.791759469228055, 3.1780538303479458, 4.7874917427820458,
    6.5792512120101012, 8.5251613610654147, 10.604602902745251,
    12.801827480081469, 15.104412573075516, 17.502307845873887,
    19.987214495661885, 22.552163853123425, 25.19122118273868, 27.89927138384089,
    30.671860106080672, 33.505073450136891, 36.395445208033053,
    39.339884187199495, 42.335616460753485, 45.380138898476908,
    48.471181351835227, 51.606675567764377, 54.784729398112319,
    58.003605222980518, 61.261701761002, 64.557538627006338, 67.88974313718154,
    71.257038967168015, 74.658236348830158, 78.0922235533153, 81.557959456115043,
    85.054467017581516, 88.580827542197682, 92.1361756036871, 95.7196945421432,
    99.330612454787428, 102.96819861451381, 106.63176026064346,
    110.32063971475739, 114.03421178146171, 117.77188139974507,
    121.53308151543864, 125.3172711493569, 129.12393363912722,
    132.95257503561632, 136.80272263732635, 140.67392364823425,
    144.5657439463449, 148.47776695177302, 152.40959258449735, 156.3608363030788,
    160.3311282166309, 164.32011226319517, 168.32744544842765,
    172.35279713916279, 176.39584840699735, 180.45629141754378,
    184.53382886144948, 188.6281734236716, 192.7390472878449, 196.86618167289,
    201.00931639928152, 205.1681994826412, 209.34258675253685,
    213.53224149456327, 217.73693411395422, 221.95644181913033,
    226.1905483237276, 230.43904356577696, 234.70172344281826,
    238.97838956183432, 243.26884900298271, 247.57291409618688,
    251.89040220972319, 256.22113555000954, 260.56494097186322,
    264.92164979855278, 269.29109765101981, 273.67312428569369,
    278.06757344036612, 282.4742926876304, 286.893133295427, 291.32395009427029,
    295.76660135076065, 300.22094864701415, 304.68685676566872,
    309.1641935801469, 313.65282994987905, 318.1526396202093, 322.66349912672615,
    327.1852877037752, 331.71788719692847, 336.26118197919845, 340.815058870799,
    345.37940706226686, 349.95411804077025, 354.53908551944079,
    359.1342053695754 };

  static const double p1[8] = { 4.9452353592967269, 201.8112620856775,
    2290.8383738313464, 11319.672059033808, 28557.246356716354,
    38484.962284437934, 26377.487876241954, 7225.8139797002877 };

  static const double p2[8] = { 4.974607845568932, 542.4138599891071,
    15506.938649783649, 184793.29044456323, 1.0882047694688288E+6,
    3.33815296798703E+6, 5.1066616789273527E+6, 3.0741090548505397E+6 };

  static const double p4[8] = { 14745.0216605994, 2.4268133694867045E+6,
    1.2147555740450932E+8, 2.6634324496309772E+9, 2.9403789566345539E+10,
    1.7026657377653989E+11, 4.926125793377431E+11, 5.6062518562239514E+11 };

  static const double q1[8] = { 67.482125503037778, 1113.3323938571993,
    7738.7570569353984, 27639.870744033407, 54993.102062261576,
    61611.221800660023, 36351.2759150194, 8785.5363024310136 };

  static const double q2[8] = { 183.03283993705926, 7765.0493214450062,
    133190.38279660742, 1.1367058213219696E+6, 5.2679641174379466E+6,
    1.3467014543111017E+7, 1.7827365303532742E+7, 9.5330955918443538E+6 };

  static const double q4[8] = { 2690.5301758708993, 639388.56543000927,
    4.1355999302413881E+7, 1.120872109616148E+9, 1.4886137286788137E+10,
    1.0168035862724382E+11, 3.4174763455073773E+11, 4.4631581874197131E+11 };

  static const double c[7] = { -0.001910444077728, 0.00084171387781295,
    -0.00059523799130430121, 0.0007936507935003503, -0.0027777777777776816,
    0.083333333333333329, 0.0057083835261 };

  double r;
  double t;
  int i;
  if ((!rtIsNaN(*x)) && (!(*x < 0.0))) {
    if (*x <= 2.2204460492503131E-16) {
      *x = -log(*x);
    } else if (*x <= 0.5) {
      t = 1.0;
      r = 0.0;
      for (i = 0; i < 8; i++) {
        r = r * *x + p1[i];
        t = t * *x + q1[i];
      }

      *x = -log(*x) + *x * (*x * (r / t) + -0.57721566490153287);
    } else if (*x <= 0.6796875) {
      t = 1.0;
      r = 0.0;
      for (i = 0; i < 8; i++) {
        r = r * ((*x - 0.5) - 0.5) + p2[i];
        t = t * ((*x - 0.5) - 0.5) + q2[i];
      }

      *x = -log(*x) + ((*x - 0.5) - 0.5) * (((*x - 0.5) - 0.5) * (r / t) +
        0.42278433509846713);
    } else if ((*x == floor(*x)) && (*x <= 100.0)) {
      *x = table100[(int)*x - 1];
    } else if (*x <= 1.5) {
      t = 1.0;
      r = 0.0;
      for (i = 0; i < 8; i++) {
        r = r * ((*x - 0.5) - 0.5) + p1[i];
        t = t * ((*x - 0.5) - 0.5) + q1[i];
      }

      *x = ((*x - 0.5) - 0.5) * (((*x - 0.5) - 0.5) * (r / t) +
        -0.57721566490153287);
    } else if (*x <= 4.0) {
      t = 1.0;
      r = 0.0;
      for (i = 0; i < 8; i++) {
        r = r * (*x - 2.0) + p2[i];
        t = t * (*x - 2.0) + q2[i];
      }

      *x = (*x - 2.0) * ((*x - 2.0) * (r / t) + 0.42278433509846713);
    } else if (*x <= 12.0) {
      t = -1.0;
      r = 0.0;
      for (i = 0; i < 8; i++) {
        r = r * (*x - 4.0) + p4[i];
        t = t * (*x - 4.0) + q4[i];
      }

      *x = (*x - 4.0) * (r / t) + 1.791759469228055;
    } else {
      r = 0.0057083835261;
      t = 1.0 / (*x * *x);
      for (i = 0; i < 6; i++) {
        r = r * t + c[i];
      }

      r /= *x;
      t = log(*x);
      *x = ((r + 0.91893853320467278) - 0.5 * t) + *x * (t - 1.0);
    }
  }
}

static creal_T power(const creal_T a)
{
  creal_T y;
  double b_im;
  double d;
  double r;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 3.0);
    y.im = 0.0;
  } else if (a.re == 0.0) {
    y.re = 0.0;
    y.im = -rt_powd_snf(a.im, 3.0);
  } else {
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = log(fabs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = log(a.re);
        b_im = 0.0;
      }
    } else if ((fabs(a.re) > 8.9884656743115785E+307) || (fabs(a.im) >
                8.9884656743115785E+307)) {
      r = log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }

    y.re = 3.0 * r;
    y.im = 3.0 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      r = exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * cos(b_im));
      y.im = r * (r * sin(d));
    }
  }

  return y;
}

static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static double solver_anonFcn1(double P_x, double P_y, double P_z, const
  Transducer3D Tarray_active[1442], double numT, const emxArray_real_T *phi)
{
  emxArray_creal_T *activation;
  emxArray_creal_T *p;
  emxArray_creal_T *px;
  emxArray_creal_T *pxx;
  emxArray_creal_T *pxxx;
  emxArray_creal_T *pxy;
  emxArray_creal_T *pxyy;
  emxArray_creal_T *pxz;
  emxArray_creal_T *pxzz;
  emxArray_creal_T *py;
  emxArray_creal_T *pyxx;
  emxArray_creal_T *pyy;
  emxArray_creal_T *pyyy;
  emxArray_creal_T *pyz;
  emxArray_creal_T *pyzz;
  emxArray_creal_T *pz;
  emxArray_creal_T *pzxx;
  emxArray_creal_T *pzyy;
  emxArray_creal_T *pzz;
  emxArray_creal_T *pzzz;
  creal_T P;
  creal_T Px;
  creal_T Pxx;
  creal_T Pxxx;
  creal_T Pxy;
  creal_T Pxyy;
  creal_T Pxz;
  creal_T Pxzz;
  creal_T Py;
  creal_T Pyxx;
  creal_T Pyy;
  creal_T Pyyy;
  creal_T Pyz;
  creal_T Pyzz;
  creal_T Pz;
  creal_T Pzxx;
  creal_T Pzyy;
  creal_T Pzz;
  creal_T Pzzz;
  creal_T *activation_data;
  creal_T *p_data;
  creal_T *px_data;
  creal_T *pxx_data;
  creal_T *pxxx_data;
  creal_T *pxy_data;
  creal_T *pxyy_data;
  creal_T *pxz_data;
  creal_T *pxzz_data;
  creal_T *py_data;
  creal_T *pyxx_data;
  creal_T *pyy_data;
  creal_T *pyyy_data;
  creal_T *pyz_data;
  creal_T *pyzz_data;
  creal_T *pz_data;
  creal_T *pzxx_data;
  creal_T *pzyy_data;
  creal_T *pzz_data;
  creal_T *pzzz_data;
  double Pc[3];
  double b_Tarray_active[3];
  const double *phi_data;
  double b_re_tmp;
  double c_re_tmp;
  double r;
  double re_tmp;
  double varargout_1;
  int i;
  int nx;
  phi_data = phi->data;
  emxInit_creal_T(&p);

  /* %% Objective function to be minimised for an ideal acoustic trap */
  /* %% Mathematical formulae are derived from the HAE paper */
  /*  function [O,g] = ObjectiveFunction(Particle, Tarray, phi, K, w) */
  /*  Number of transducers */
  /*  N = size(Tarray,2); */
  /*  Get location */
  Pc[0] = P_x;
  Pc[1] = P_y;
  Pc[2] = P_z;

  /*  Preallocate arrays */
  i = p->size[0] * p->size[1];
  p->size[0] = 1;
  nx = (int)numT;
  p->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(p, i);
  p_data = p->data;
  for (i = 0; i < nx; i++) {
    p_data[i].re = 0.0;
    p_data[i].im = 0.0;
  }

  emxInit_creal_T(&px);
  i = px->size[0] * px->size[1];
  px->size[0] = 1;
  px->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(px, i);
  px_data = px->data;
  for (i = 0; i < nx; i++) {
    px_data[i].re = 0.0;
    px_data[i].im = 0.0;
  }

  emxInit_creal_T(&py);
  i = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(py, i);
  py_data = py->data;
  for (i = 0; i < nx; i++) {
    py_data[i].re = 0.0;
    py_data[i].im = 0.0;
  }

  emxInit_creal_T(&pz);
  i = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pz, i);
  pz_data = pz->data;
  for (i = 0; i < nx; i++) {
    pz_data[i].re = 0.0;
    pz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxx);
  i = pxx->size[0] * pxx->size[1];
  pxx->size[0] = 1;
  pxx->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxx, i);
  pxx_data = pxx->data;
  for (i = 0; i < nx; i++) {
    pxx_data[i].re = 0.0;
    pxx_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxy);
  i = pxy->size[0] * pxy->size[1];
  pxy->size[0] = 1;
  pxy->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxy, i);
  pxy_data = pxy->data;
  for (i = 0; i < nx; i++) {
    pxy_data[i].re = 0.0;
    pxy_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxz);
  i = pxz->size[0] * pxz->size[1];
  pxz->size[0] = 1;
  pxz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxz, i);
  pxz_data = pxz->data;
  for (i = 0; i < nx; i++) {
    pxz_data[i].re = 0.0;
    pxz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pyy);
  i = pyy->size[0] * pyy->size[1];
  pyy->size[0] = 1;
  pyy->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pyy, i);
  pyy_data = pyy->data;
  for (i = 0; i < nx; i++) {
    pyy_data[i].re = 0.0;
    pyy_data[i].im = 0.0;
  }

  emxInit_creal_T(&pyz);
  i = pyz->size[0] * pyz->size[1];
  pyz->size[0] = 1;
  pyz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pyz, i);
  pyz_data = pyz->data;
  for (i = 0; i < nx; i++) {
    pyz_data[i].re = 0.0;
    pyz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pzz);
  i = pzz->size[0] * pzz->size[1];
  pzz->size[0] = 1;
  pzz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pzz, i);
  pzz_data = pzz->data;
  for (i = 0; i < nx; i++) {
    pzz_data[i].re = 0.0;
    pzz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxxx);
  i = pxxx->size[0] * pxxx->size[1];
  pxxx->size[0] = 1;
  pxxx->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxxx, i);
  pxxx_data = pxxx->data;
  for (i = 0; i < nx; i++) {
    pxxx_data[i].re = 0.0;
    pxxx_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxyy);
  i = pxyy->size[0] * pxyy->size[1];
  pxyy->size[0] = 1;
  pxyy->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxyy, i);
  pxyy_data = pxyy->data;
  for (i = 0; i < nx; i++) {
    pxyy_data[i].re = 0.0;
    pxyy_data[i].im = 0.0;
  }

  emxInit_creal_T(&pxzz);
  i = pxzz->size[0] * pxzz->size[1];
  pxzz->size[0] = 1;
  pxzz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pxzz, i);
  pxzz_data = pxzz->data;
  for (i = 0; i < nx; i++) {
    pxzz_data[i].re = 0.0;
    pxzz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pyxx);
  i = pyxx->size[0] * pyxx->size[1];
  pyxx->size[0] = 1;
  pyxx->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pyxx, i);
  pyxx_data = pyxx->data;
  for (i = 0; i < nx; i++) {
    pyxx_data[i].re = 0.0;
    pyxx_data[i].im = 0.0;
  }

  emxInit_creal_T(&pyyy);
  i = pyyy->size[0] * pyyy->size[1];
  pyyy->size[0] = 1;
  pyyy->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pyyy, i);
  pyyy_data = pyyy->data;
  for (i = 0; i < nx; i++) {
    pyyy_data[i].re = 0.0;
    pyyy_data[i].im = 0.0;
  }

  emxInit_creal_T(&pyzz);
  i = pyzz->size[0] * pyzz->size[1];
  pyzz->size[0] = 1;
  pyzz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pyzz, i);
  pyzz_data = pyzz->data;
  for (i = 0; i < nx; i++) {
    pyzz_data[i].re = 0.0;
    pyzz_data[i].im = 0.0;
  }

  emxInit_creal_T(&pzxx);
  i = pzxx->size[0] * pzxx->size[1];
  pzxx->size[0] = 1;
  pzxx->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pzxx, i);
  pzxx_data = pzxx->data;
  for (i = 0; i < nx; i++) {
    pzxx_data[i].re = 0.0;
    pzxx_data[i].im = 0.0;
  }

  emxInit_creal_T(&pzyy);
  i = pzyy->size[0] * pzyy->size[1];
  pzyy->size[0] = 1;
  pzyy->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pzyy, i);
  pzyy_data = pzyy->data;
  for (i = 0; i < nx; i++) {
    pzyy_data[i].re = 0.0;
    pzyy_data[i].im = 0.0;
  }

  emxInit_creal_T(&pzzz);
  i = pzzz->size[0] * pzzz->size[1];
  pzzz->size[0] = 1;
  pzzz->size[1] = (int)numT;
  emxEnsureCapacity_creal_T(pzzz, i);
  pzzz_data = pzzz->data;
  for (i = 0; i < nx; i++) {
    pzzz_data[i].re = 0.0;
    pzzz_data[i].im = 0.0;
  }

  /*  Construct pressure arrays */
  for (i = 0; i < nx; i++) {
    /*  Current transducer */
    /*  Get location */
    /*  Calculate pressure (+ etc) from current transducer at the given point */
    b_Tarray_active[0] = Tarray_active[i].x;
    b_Tarray_active[1] = Tarray_active[i].y;
    b_Tarray_active[2] = Tarray_active[i].z;
    TryThisOneNoNorm(Pc, b_Tarray_active, &p_data[i], &px_data[i], &py_data[i],
                     &pz_data[i], &pxx_data[i], &pxy_data[i], &pxz_data[i],
                     &pyy_data[i], &pyz_data[i], &pzz_data[i], &pxxx_data[i],
                     &pxyy_data[i], &pxzz_data[i], &pyxx_data[i], &pyyy_data[i],
                     &pyzz_data[i], &pzxx_data[i], &pzyy_data[i], &pzzz_data[i]);
  }

  emxInit_creal_T(&activation);

  /*  Apply transducer activations (above values aren't actually pressures */
  /*  until now) */
  i = activation->size[0] * activation->size[1];
  activation->size[0] = 1;
  activation->size[1] = phi->size[1];
  emxEnsureCapacity_creal_T(activation, i);
  activation_data = activation->data;
  nx = phi->size[1];
  for (i = 0; i < nx; i++) {
    activation_data[i].re = phi_data[i] * 0.0;
    activation_data[i].im = phi_data[i];
  }

  nx = activation->size[1];
  for (i = 0; i < nx; i++) {
    if (activation_data[i].im == 0.0) {
      activation_data[i].re = exp(activation_data[i].re);
      activation_data[i].im = 0.0;
    } else if (rtIsInf(activation_data[i].im) && rtIsInf(activation_data[i].re) &&
               (activation_data[i].re < 0.0)) {
      activation_data[i].re = 0.0;
      activation_data[i].im = 0.0;
    } else {
      r = exp(activation_data[i].re / 2.0);
      activation_data[i].re = r * (r * cos(activation_data[i].im));
      activation_data[i].im = r * (r * sin(activation_data[i].im));
    }
  }

  if (p->size[1] == activation->size[1]) {
    nx = p->size[1] - 1;
    i = p->size[0] * p->size[1];
    p->size[0] = 1;
    emxEnsureCapacity_creal_T(p, i);
    p_data = p->data;
    for (i = 0; i <= nx; i++) {
      r = p_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = p_data[i].im;
      c_re_tmp = activation_data[i].re;
      p_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      p_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(p, activation);
  }

  if (px->size[1] == activation->size[1]) {
    nx = px->size[1] - 1;
    i = px->size[0] * px->size[1];
    px->size[0] = 1;
    emxEnsureCapacity_creal_T(px, i);
    px_data = px->data;
    for (i = 0; i <= nx; i++) {
      r = px_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = px_data[i].im;
      c_re_tmp = activation_data[i].re;
      px_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      px_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(px, activation);
  }

  if (py->size[1] == activation->size[1]) {
    nx = py->size[1] - 1;
    i = py->size[0] * py->size[1];
    py->size[0] = 1;
    emxEnsureCapacity_creal_T(py, i);
    py_data = py->data;
    for (i = 0; i <= nx; i++) {
      r = py_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = py_data[i].im;
      c_re_tmp = activation_data[i].re;
      py_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      py_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(py, activation);
  }

  if (pz->size[1] == activation->size[1]) {
    nx = pz->size[1] - 1;
    i = pz->size[0] * pz->size[1];
    pz->size[0] = 1;
    emxEnsureCapacity_creal_T(pz, i);
    pz_data = pz->data;
    for (i = 0; i <= nx; i++) {
      r = pz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pz, activation);
  }

  if (pxx->size[1] == activation->size[1]) {
    nx = pxx->size[1] - 1;
    i = pxx->size[0] * pxx->size[1];
    pxx->size[0] = 1;
    emxEnsureCapacity_creal_T(pxx, i);
    pxx_data = pxx->data;
    for (i = 0; i <= nx; i++) {
      r = pxx_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxx_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxx_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxx_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxx, activation);
  }

  if (pxy->size[1] == activation->size[1]) {
    nx = pxy->size[1] - 1;
    i = pxy->size[0] * pxy->size[1];
    pxy->size[0] = 1;
    emxEnsureCapacity_creal_T(pxy, i);
    pxy_data = pxy->data;
    for (i = 0; i <= nx; i++) {
      r = pxy_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxy_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxy_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxy_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxy, activation);
  }

  if (pxz->size[1] == activation->size[1]) {
    nx = pxz->size[1] - 1;
    i = pxz->size[0] * pxz->size[1];
    pxz->size[0] = 1;
    emxEnsureCapacity_creal_T(pxz, i);
    pxz_data = pxz->data;
    for (i = 0; i <= nx; i++) {
      r = pxz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxz, activation);
  }

  if (pyy->size[1] == activation->size[1]) {
    nx = pyy->size[1] - 1;
    i = pyy->size[0] * pyy->size[1];
    pyy->size[0] = 1;
    emxEnsureCapacity_creal_T(pyy, i);
    pyy_data = pyy->data;
    for (i = 0; i <= nx; i++) {
      r = pyy_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pyy_data[i].im;
      c_re_tmp = activation_data[i].re;
      pyy_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pyy_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pyy, activation);
  }

  if (pyz->size[1] == activation->size[1]) {
    nx = pyz->size[1] - 1;
    i = pyz->size[0] * pyz->size[1];
    pyz->size[0] = 1;
    emxEnsureCapacity_creal_T(pyz, i);
    pyz_data = pyz->data;
    for (i = 0; i <= nx; i++) {
      r = pyz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pyz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pyz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pyz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pyz, activation);
  }

  if (pzz->size[1] == activation->size[1]) {
    nx = pzz->size[1] - 1;
    i = pzz->size[0] * pzz->size[1];
    pzz->size[0] = 1;
    emxEnsureCapacity_creal_T(pzz, i);
    pzz_data = pzz->data;
    for (i = 0; i <= nx; i++) {
      r = pzz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pzz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pzz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pzz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pzz, activation);
  }

  if (pxxx->size[1] == activation->size[1]) {
    nx = pxxx->size[1] - 1;
    i = pxxx->size[0] * pxxx->size[1];
    pxxx->size[0] = 1;
    emxEnsureCapacity_creal_T(pxxx, i);
    pxxx_data = pxxx->data;
    for (i = 0; i <= nx; i++) {
      r = pxxx_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxxx_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxxx_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxxx_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxxx, activation);
  }

  if (pxyy->size[1] == activation->size[1]) {
    nx = pxyy->size[1] - 1;
    i = pxyy->size[0] * pxyy->size[1];
    pxyy->size[0] = 1;
    emxEnsureCapacity_creal_T(pxyy, i);
    pxyy_data = pxyy->data;
    for (i = 0; i <= nx; i++) {
      r = pxyy_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxyy_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxyy_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxyy_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxyy, activation);
  }

  if (pxzz->size[1] == activation->size[1]) {
    nx = pxzz->size[1] - 1;
    i = pxzz->size[0] * pxzz->size[1];
    pxzz->size[0] = 1;
    emxEnsureCapacity_creal_T(pxzz, i);
    pxzz_data = pxzz->data;
    for (i = 0; i <= nx; i++) {
      r = pxzz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pxzz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pxzz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pxzz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pxzz, activation);
  }

  if (pyxx->size[1] == activation->size[1]) {
    nx = pyxx->size[1] - 1;
    i = pyxx->size[0] * pyxx->size[1];
    pyxx->size[0] = 1;
    emxEnsureCapacity_creal_T(pyxx, i);
    pyxx_data = pyxx->data;
    for (i = 0; i <= nx; i++) {
      r = pyxx_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pyxx_data[i].im;
      c_re_tmp = activation_data[i].re;
      pyxx_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pyxx_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pyxx, activation);
  }

  if (pyyy->size[1] == activation->size[1]) {
    nx = pyyy->size[1] - 1;
    i = pyyy->size[0] * pyyy->size[1];
    pyyy->size[0] = 1;
    emxEnsureCapacity_creal_T(pyyy, i);
    pyyy_data = pyyy->data;
    for (i = 0; i <= nx; i++) {
      r = pyyy_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pyyy_data[i].im;
      c_re_tmp = activation_data[i].re;
      pyyy_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pyyy_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pyyy, activation);
  }

  if (pyzz->size[1] == activation->size[1]) {
    nx = pyzz->size[1] - 1;
    i = pyzz->size[0] * pyzz->size[1];
    pyzz->size[0] = 1;
    emxEnsureCapacity_creal_T(pyzz, i);
    pyzz_data = pyzz->data;
    for (i = 0; i <= nx; i++) {
      r = pyzz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pyzz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pyzz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pyzz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pyzz, activation);
  }

  if (pzxx->size[1] == activation->size[1]) {
    nx = pzxx->size[1] - 1;
    i = pzxx->size[0] * pzxx->size[1];
    pzxx->size[0] = 1;
    emxEnsureCapacity_creal_T(pzxx, i);
    pzxx_data = pzxx->data;
    for (i = 0; i <= nx; i++) {
      r = pzxx_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pzxx_data[i].im;
      c_re_tmp = activation_data[i].re;
      pzxx_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pzxx_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pzxx, activation);
  }

  if (pzyy->size[1] == activation->size[1]) {
    nx = pzyy->size[1] - 1;
    i = pzyy->size[0] * pzyy->size[1];
    pzyy->size[0] = 1;
    emxEnsureCapacity_creal_T(pzyy, i);
    pzyy_data = pzyy->data;
    for (i = 0; i <= nx; i++) {
      r = pzyy_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pzyy_data[i].im;
      c_re_tmp = activation_data[i].re;
      pzyy_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pzyy_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pzyy, activation);
  }

  if (pzzz->size[1] == activation->size[1]) {
    nx = pzzz->size[1] - 1;
    i = pzzz->size[0] * pzzz->size[1];
    pzzz->size[0] = 1;
    emxEnsureCapacity_creal_T(pzzz, i);
    pzzz_data = pzzz->data;
    for (i = 0; i <= nx; i++) {
      r = pzzz_data[i].re;
      re_tmp = activation_data[i].im;
      b_re_tmp = pzzz_data[i].im;
      c_re_tmp = activation_data[i].re;
      pzzz_data[i].re = r * c_re_tmp - b_re_tmp * re_tmp;
      pzzz_data[i].im = r * re_tmp + b_re_tmp * c_re_tmp;
    }
  } else {
    times(pzzz, activation);
  }

  emxFree_creal_T(&activation);

  /*  Precompute sums */
  P = sum(p);
  Px = sum(px);
  Py = sum(py);
  Pz = sum(pz);
  Pxx = sum(pxx);
  Pxy = sum(pxy);
  Pxz = sum(pxz);
  Pyy = sum(pyy);
  Pyz = sum(pyz);
  Pzz = sum(pzz);
  Pxxx = sum(pxxx);
  Pxyy = sum(pxyy);
  Pxzz = sum(pxzz);
  Pyxx = sum(pyxx);
  Pyyy = sum(pyyy);
  Pyzz = sum(pyzz);
  Pzxx = sum(pzxx);
  Pzyy = sum(pzyy);
  Pzzz = sum(pzzz);

  /*  Calculate |p| term */
  /*  Calculate Laplacian of Gor'kov terms */
  r = Pxy.re * Pxy.re + Pxy.im * Pxy.im;
  re_tmp = Pxz.re * Pxz.re + Pxz.im * Pxz.im;
  b_re_tmp = Pyz.re * Pyz.re + Pyz.im * Pyz.im;

  /*  Weights */
  /*  Evaluate objective function */
  varargout_1 = (((P.re * P.re + P.im * P.im) - 10.0 * (((1.374906904580574E-8 *
    ((Px.re * Px.re + Px.im * Px.im) + (P.re * Pxx.re + P.im * Pxx.im)) -
    -1.7426065355617798E-14 * ((Pxx.re * Pxx.re + Pxx.im * Pxx.im) + (Px.re *
    Pxxx.re + Px.im * Pxxx.im))) - -1.7426065355617798E-14 * (r + (Px.re *
    Pyxx.re + Px.im * Pyxx.im))) - -1.7426065355617798E-14 * (re_tmp + (Px.re *
    Pzxx.re + Px.im * Pzxx.im)))) - 10.0 * (((1.374906904580574E-8 * ((Py.re *
    Py.re + Py.im * Py.im) + (P.re * Pyy.re + P.im * Pyy.im)) -
    -1.7426065355617798E-14 * (r + (Py.re * Pxyy.re + Py.im * Pxyy.im))) -
    -1.7426065355617798E-14 * ((Pyy.re * Pyy.re + Pyy.im * Pyy.im) + (Py.re *
    Pyyy.re + Py.im * Pyyy.im))) - -1.7426065355617798E-14 * (b_re_tmp + (Py.re *
    Pzyy.re + Py.im * Pzyy.im)))) - 1000.0 * (((1.374906904580574E-8 * ((Pz.re *
    Pz.re + Pz.im * Pz.im) + (P.re * Pzz.re + P.im * Pzz.im)) -
    -1.7426065355617798E-14 * (re_tmp + (Pz.re * Pxzz.re + Pz.im * Pxzz.im))) -
    -1.7426065355617798E-14 * (b_re_tmp + (Pz.re * Pyzz.re + Pz.im * Pyzz.im)))
    - -1.7426065355617798E-14 * ((Pzz.re * Pzz.re + Pzz.im * Pzz.im) + (Pz.re *
    Pzzz.re + Pz.im * Pzzz.im)));

  /*  % Preallocate gradient array (partial derivative for each phi) */
  /*  g = zeros(1,N); */
  /*  for i = 1:N */
  /*      % Calculate gradient of |p| term */
  /*      gpAbs = dDotOp(P,P,p(i),p(i)); */
  /*      % Calculate gradient of Laplacian of Gor'kov terms */
  /*      [gUxx, gUyy, gUzz] = gLaplacianOfGorkov(K,P,Px,Py,Pz,Pxx,Pxy,Pxz,Pyy,Pyz,Pzz,Pxxx,Pxyy,Pxzz,Pyxx,Pyyy,Pyzz,Pzxx,Pzyy,Pzzz,p(i),px(i),py(i),pz(i),pxx(i),pxy(i),pxz(i),pyy(i),pyz(i),pzz(i),pxxx(i),pxyy(i),pxzz(i),pyxx(i),pyyy(i),pyzz(i),pzxx(i),pzyy(i),pzzz(i)); */
  /*      % Evaluate gradient of objective function */
  /*      g(i) = wp*gpAbs - wx*gUxx - wy*gUyy - wz*gUzz; */
  /*  end */
  emxFree_creal_T(&pzzz);
  emxFree_creal_T(&pzyy);
  emxFree_creal_T(&pzxx);
  emxFree_creal_T(&pyzz);
  emxFree_creal_T(&pyyy);
  emxFree_creal_T(&pyxx);
  emxFree_creal_T(&pxzz);
  emxFree_creal_T(&pxyy);
  emxFree_creal_T(&pxxx);
  emxFree_creal_T(&pzz);
  emxFree_creal_T(&pyz);
  emxFree_creal_T(&pyy);
  emxFree_creal_T(&pxz);
  emxFree_creal_T(&pxy);
  emxFree_creal_T(&pxx);
  emxFree_creal_T(&pz);
  emxFree_creal_T(&py);
  emxFree_creal_T(&px);
  emxFree_creal_T(&p);
  return varargout_1;
}

static void sortIdx(const emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_int32_T *iwork;
  const double *x_data;
  double d;
  int b_i;
  int i;
  int i2;
  int j;
  int k;
  int kEnd;
  int n;
  int p;
  int pEnd;
  int q;
  int qEnd;
  int *idx_data;
  int *iwork_data;
  x_data = x->data;
  n = x->size[1] + 1;
  i = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  idx->size[1] = x->size[1];
  emxEnsureCapacity_int32_T(idx, i);
  idx_data = idx->data;
  b_i = x->size[1];
  for (i = 0; i < b_i; i++) {
    idx_data[i] = 0;
  }

  emxInit_int32_T(&iwork, 1);
  i = iwork->size[0];
  iwork->size[0] = x->size[1];
  emxEnsureCapacity_int32_T(iwork, i);
  iwork_data = iwork->data;
  i = x->size[1] - 1;
  for (k = 1; k <= i; k += 2) {
    d = x_data[k];
    if ((x_data[k - 1] <= d) || rtIsNaN(d)) {
      idx_data[k - 1] = k;
      idx_data[k] = k + 1;
    } else {
      idx_data[k - 1] = k + 1;
      idx_data[k] = k;
    }
  }

  if ((x->size[1] & 1) != 0) {
    idx_data[x->size[1] - 1] = x->size[1];
  }

  b_i = 2;
  while (b_i < n - 1) {
    i2 = b_i << 1;
    j = 1;
    for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
      p = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > n) {
        qEnd = n;
      }

      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        d = x_data[idx_data[q] - 1];
        i = idx_data[p - 1];
        if ((x_data[i - 1] <= d) || rtIsNaN(d)) {
          iwork_data[k] = i;
          p++;
          if (p == pEnd) {
            while (q + 1 < qEnd) {
              k++;
              iwork_data[k] = idx_data[q];
              q++;
            }
          }
        } else {
          iwork_data[k] = idx_data[q];
          q++;
          if (q + 1 == qEnd) {
            while (p < pEnd) {
              k++;
              iwork_data[k] = idx_data[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (k = 0; k < kEnd; k++) {
        idx_data[(j + k) - 1] = iwork_data[k];
      }

      j = qEnd;
    }

    b_i = i2;
  }

  emxFree_int32_T(&iwork);
}

static creal_T sum(const emxArray_creal_T *x)
{
  const creal_T *x_data;
  creal_T y;
  double bsum_im;
  double bsum_re;
  int bsum_re_tmp;
  int firstBlockLength;
  int hi;
  int ib;
  int k;
  int lastBlockLength;
  int nblocks;
  x_data = x->data;
  if (x->size[1] == 0) {
    y.re = 0.0;
    y.im = 0.0;
  } else {
    if (x->size[1] <= 1024) {
      firstBlockLength = x->size[1];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = x->size[1] / 1024;
      lastBlockLength = x->size[1] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }

    y = x_data[0];
    for (k = 2; k <= firstBlockLength; k++) {
      y.re += x_data[k - 1].re;
      y.im += x_data[k - 1].im;
    }

    for (ib = 2; ib <= nblocks; ib++) {
      firstBlockLength = (ib - 1) << 10;
      bsum_re = x_data[firstBlockLength].re;
      bsum_im = x_data[firstBlockLength].im;
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }

      for (k = 2; k <= hi; k++) {
        bsum_re_tmp = (firstBlockLength + k) - 1;
        bsum_re += x_data[bsum_re_tmp].re;
        bsum_im += x_data[bsum_re_tmp].im;
      }

      y.re += bsum_re;
      y.im += bsum_im;
    }
  }

  return y;
}

static void times(emxArray_creal_T *pzzz, const emxArray_creal_T *activation)
{
  emxArray_creal_T *b_pzzz;
  const creal_T *activation_data;
  creal_T *b_pzzz_data;
  creal_T *pzzz_data;
  double d;
  double d1;
  int i;
  int i1;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  activation_data = activation->data;
  pzzz_data = pzzz->data;
  emxInit_creal_T(&b_pzzz);
  i = b_pzzz->size[0] * b_pzzz->size[1];
  b_pzzz->size[0] = 1;
  if (activation->size[1] == 1) {
    b_pzzz->size[1] = pzzz->size[1];
  } else {
    b_pzzz->size[1] = activation->size[1];
  }

  emxEnsureCapacity_creal_T(b_pzzz, i);
  b_pzzz_data = b_pzzz->data;
  stride_0_1 = (pzzz->size[1] != 1);
  stride_1_1 = (activation->size[1] != 1);
  if (activation->size[1] == 1) {
    loop_ub = pzzz->size[1];
  } else {
    loop_ub = activation->size[1];
  }

  for (i = 0; i < loop_ub; i++) {
    i1 = i * stride_1_1;
    d = activation_data[i1].im;
    d1 = activation_data[i1].re;
    b_pzzz_data[i].re = pzzz_data[i * stride_0_1].re * d1 - pzzz_data[i *
      stride_0_1].im * d;
    b_pzzz_data[i].im = pzzz_data[i * stride_0_1].re * d + pzzz_data[i *
      stride_0_1].im * d1;
  }

  i = pzzz->size[0] * pzzz->size[1];
  pzzz->size[0] = 1;
  pzzz->size[1] = b_pzzz->size[1];
  emxEnsureCapacity_creal_T(pzzz, i);
  pzzz_data = pzzz->data;
  loop_ub = b_pzzz->size[1];
  for (i = 0; i < loop_ub; i++) {
    pzzz_data[i] = b_pzzz_data[i];
  }

  emxFree_creal_T(&b_pzzz);
}

void solver(double particlePosX, double particlePosY, double particlePosZ,
            double outputArg1[1442])
{
  static const double x[12] = { -0.0054, -0.0108, -0.0054, 0.0054, 0.0108,
    0.0054, 0.0093530743608719377, 0.0, -0.0093530743608719377,
    -0.0093530743608719377, 0.0, 0.0093530743608719377 };

  Transducer3D Tarray[1442];
  Transducer3D Tarray_active[1442];
  emxArray_real_T *a;
  emxArray_real_T *phi_active;
  double T_loc[4326];
  double coords[1442];
  double vectors_from_point[1442];
  double b_ring_cords_data[180];
  double ring_cords_data[180];
  double P_control_loc_unadj[3];
  double i_data[2];
  double P_control_loc_idx_1;
  double buffer_data_idx_0;
  double e_ind;
  double s_ind;
  double *a_data;
  double *phi_active_data;
  int b_i;
  int colIdx;
  int i;
  int j;
  int k;
  unsigned int r;
  int ring_cords_size_idx_0;
  int rowIdx;
  boolean_T active_mask_half[721];
  boolean_T b;
  if (!isInitialized_solver) {
    solver_initialize();
  }

  r = 1U;
  state[0] = 1U;
  for (colIdx = 0; colIdx < 623; colIdx++) {
    r = ((r ^ r >> 30U) * 1812433253U + colIdx) + 1U;
    state[colIdx + 1] = r;
  }

  state[624] = 624U;

  /*  addpath 'internal' */
  /*  addpath 'Robot hell' */
  /*  Define parameters */
  /*  Transducer frequency (Hz) */
  /*  Speed of sound (m/s) */
  /*  Wavelength (m) */
  /*  Transducer diameters (m) */
  /*  Spacing gap between two adjacent transducers (m) */
  /*  Number of rings (about the centre) */
  /*  Centre point included (bool) */
  /*  Distance between the two array plates (m) */
  /*  Make hex grid */
  memset(&coords[0], 0, 1442U * sizeof(double));

  /*  Add the origin coordinate if desired */
  coords[0] = 0.0;
  coords[721] = 0.0;

  /*  Base unit distance for coordinate calculations (m) */
  /*  Vectors describing 'moving' around the first ring */
  /*  Multiply move vectors by ring number, do maths to convert to coordinates */
  /*  and then add them to the array */
  s_ind = 2.0;
  e_ind = 7.0;
  i_data[1] = 0.0;
  for (i = 0; i < 15; i++) {
    ring_cords_size_idx_0 = 6 * (i + 1);
    colIdx = -1;
    for (j = 0; j < 2; j++) {
      rowIdx = -1;
      colIdx++;
      for (b_i = 0; b_i < 6; b_i++) {
        for (k = 0; k <= i; k++) {
          ring_cords_data[((rowIdx + k) + ring_cords_size_idx_0 * colIdx) + 1] =
            x[b_i + 6 * j];
        }

        rowIdx = (rowIdx + i) + 1;
      }
    }

    for (k = 0; k < 2; k++) {
      for (rowIdx = 0; rowIdx <= ring_cords_size_idx_0 - 2; rowIdx++) {
        colIdx = rowIdx + ring_cords_size_idx_0 * k;
        ring_cords_data[colIdx + 1] += ring_cords_data[colIdx];
      }
    }

    i_data[0] = ((double)i + 1.0) * 2.0 * 0.0054;
    for (j = 0; j < 2; j++) {
      for (rowIdx = 0; rowIdx < ring_cords_size_idx_0; rowIdx++) {
        b_ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] =
          ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] + i_data[j];
      }
    }

    for (j = 0; j < 2; j++) {
      for (rowIdx = 0; rowIdx < ring_cords_size_idx_0; rowIdx++) {
        ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] =
          b_ring_cords_data[rowIdx + ring_cords_size_idx_0 * j];
      }
    }

    /*      coords = [coords;round(circshift(ring_cords,1,1),5)]; */
    /*      coords(s_ind:e_ind,:) = round(circshift(ring_cords,1,1),5); */
    if (s_ind > e_ind) {
      j = 1;
    } else {
      j = (int)s_ind;
    }

    for (b_i = 0; b_i < 2; b_i++) {
      colIdx = b_i * ring_cords_size_idx_0;
      buffer_data_idx_0 = ring_cords_data[(colIdx + ring_cords_size_idx_0) - 1];
      for (k = ring_cords_size_idx_0; k >= 2; k--) {
        rowIdx = colIdx + k;
        ring_cords_data[rowIdx - 1] = ring_cords_data[rowIdx - 2];
      }

      ring_cords_data[colIdx] = buffer_data_idx_0;
    }

    colIdx = ring_cords_size_idx_0 << 1;
    for (k = 0; k < colIdx; k++) {
      ring_cords_data[k] = rt_roundd_snf(ring_cords_data[k]);
    }

    for (rowIdx = 0; rowIdx < 2; rowIdx++) {
      for (colIdx = 0; colIdx < ring_cords_size_idx_0; colIdx++) {
        coords[((j + colIdx) + 721 * rowIdx) - 1] = ring_cords_data[colIdx +
          ring_cords_size_idx_0 * rowIdx] / 100000.0;
      }
    }

    s_ind += ((double)i + 1.0) * 6.0;
    e_ind += (((double)i + 1.0) + 1.0) * 6.0;
  }

  /*  You can run the line(s) below just to verify the coordinate order */
  /* figure; hold on; xlim([-0.2 0.2]); ylim([-0.15 0.15]);for i = 1:721 plot(coords(i,1),coords(i,2),'kx'); pause(0.1); end */
  /*  Make transducer array */
  /*  Number of transducers */
  /*  Transducer coordinates (x,y,z) */
  for (j = 0; j < 2; j++) {
    memcpy(&T_loc[j * 1442], &coords[j * 721], 721U * sizeof(double));
  }

  memset(&T_loc[2884], 0, 721U * sizeof(double));
  for (j = 0; j < 2; j++) {
    memcpy(&T_loc[j * 1442 + 721], &coords[j * 721], 721U * sizeof(double));
  }

  for (j = 0; j < 721; j++) {
    T_loc[j + 3605] = 0.0343;
  }

  /*  Transducer normals */
  /*  Construct transducer object array */
  for (colIdx = 0; colIdx < 1442; colIdx++) {
    /*  LOCATION */
    Tarray[colIdx].x = T_loc[colIdx];
    Tarray[colIdx].y = T_loc[colIdx + 1442];
    Tarray[colIdx].z = T_loc[colIdx + 2884];

    /*  NORMAL */
    /*  RADIUS */
    /*  REFERENCE PRESSURE */
  }

  /*  Create control point */
  /*  Desired location of acoustic trap */
  P_control_loc_unadj[0] = particlePosX;
  P_control_loc_unadj[1] = particlePosY;
  P_control_loc_unadj[2] = particlePosZ;
  buffer_data_idx_0 = particlePosX;
  P_control_loc_idx_1 = particlePosY;

  /*  Avoid any divide-by-0 errors by adding a small value where necessary */
  if (particlePosX == 0.0) {
    buffer_data_idx_0 = 1.0E-9;
  }

  if (particlePosY == 0.0) {
    P_control_loc_idx_1 = 1.0E-9;
  }

  for (i = 0; i < 721; i++) {
    if (coords[i] == buffer_data_idx_0) {
      buffer_data_idx_0 += 1.0E-9;
    } else if (coords[i + 721] == P_control_loc_idx_1) {
      P_control_loc_idx_1 += 1.0E-9;
    }
  }

  /*  Contruct particle object */
  /*  Constructor */
  /*  Determine 'active' transducers */
  /*  Vectors describing translation from each transducer coord to the trap point */
  for (j = 0; j < 2; j++) {
    for (rowIdx = 0; rowIdx < 721; rowIdx++) {
      colIdx = rowIdx + 721 * j;
      vectors_from_point[colIdx] = coords[colIdx] - P_control_loc_unadj[j];
    }
  }

  /*  Calculate distances (just x,y -- 2D distance) (m) */
  /*  Radius around trap coordinate of ring of active transducers (m) */
  /*  Establish bitmask of active transducers (within active radius) */
  for (k = 0; k < 721; k++) {
    s_ind = vectors_from_point[k + 721];
    e_ind = vectors_from_point[k];
    active_mask_half[k] = (sqrt(e_ind * e_ind + s_ind * s_ind) <= 0.05);
  }

  for (j = 0; j < 721; j++) {
    b = active_mask_half[j];
    coords[j] = b;
    coords[j + 721] = b;
  }

  e_ind = coords[0];
  for (k = 0; k < 1023; k++) {
    e_ind += coords[k + 1];
  }

  s_ind = coords[1024];
  for (k = 0; k < 417; k++) {
    s_ind += coords[k + 1025];
  }

  e_ind += s_ind;
  for (colIdx = 0; colIdx < 1442; colIdx++) {
    /*  LOCATION */
    Tarray_active[colIdx].x = -1.0;
    Tarray_active[colIdx].y = -1.0;
    Tarray_active[colIdx].z = -1.0;

    /*  NORMAL */
    /*  RADIUS */
    /*  REFERENCE PRESSURE */
  }

  /*  Tarray_active = cell(1,N); */
  /*  Construct new array of active-only transducers */
  /*  Tarray_active(:) = {Tarray{active_mask}}; */
  s_ind = 1.0;
  for (i = 0; i < 1442; i++) {
    if (coords[i] == 1.0) {
      Tarray_active[(int)s_ind - 1] = Tarray[i];
      s_ind++;
    }
  }

  emxInit_real_T(&a, 2);

  /*  Number of active transducers */
  /*  Call solver */
  /*  Some more parameters for calculations */
  /*  Volume of levitated particle (assuming sphere, r=1mm) (m) */
  /*  Speed of sound in host medium (m/s) */
  /*  Speed of sound in particle (m/s) */
  /*  Density of host medium (kg/m^3) */
  /*  Density of particle (kg/m^3) */
  /*  Precompute K values (Gor'kov) */
  /*  Objective function weighting parameters */
  /*  Initialise with random phi set */
  b_rand(e_ind, a);

  /*  Define cost function handle */
  /*  phi_active = zeros(N,1); */
  /*  Set optimiser options */
  /*  options = optimset('MaxFunEvals',Inf); */
  /*  options.SpecifyObjectiveGradient = true; */
  /*  options.MaxFunctionEvaluations = Inf; */
  /*  %options.MaxIter = 25000; */
  /*  %options.DerivativeCheck = 'on'; */
  /*  %options.FiniteDifferenceType = 'central'; */
  /*   */
  /*  % Call optimiser function (the solver) */
  j = a->size[0] * a->size[1];
  a->size[0] = 1;
  emxEnsureCapacity_real_T(a, j);
  a_data = a->data;
  colIdx = a->size[1] - 1;
  for (j = 0; j <= colIdx; j++) {
    a_data[j] = a_data[j] * 2.0 * 3.1415926535897931;
  }

  emxInit_real_T(&phi_active, 1);
  fminsearch(buffer_data_idx_0, P_control_loc_idx_1, particlePosZ, Tarray_active,
             e_ind, a);
  a_data = a->data;
  j = phi_active->size[0];
  phi_active->size[0] = a->size[1];
  emxEnsureCapacity_real_T(phi_active, j);
  phi_active_data = phi_active->data;
  colIdx = a->size[1];
  for (j = 0; j < colIdx; j++) {
    phi_active_data[j] = a_data[j];
  }

  emxFree_real_T(&a);

  /*  Contruct full phi array (for sending commands to the hardware) */
  /*  Go through the bitmask and construct the full phase array */
  r = 1U;
  for (i = 0; i < 1442; i++) {
    outputArg1[i] = 0.0;
    if (coords[i] == 1.0) {
      outputArg1[i] = phi_active_data[(int)r - 1];
      r++;
    }
  }

  emxFree_real_T(&phi_active);
}

void solver_initialize(void)
{
  c_eml_rand_mt19937ar_stateful_i();
  isInitialized_solver = true;
}

void solver_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_solver = false;
}

/* End of code generation (solver.c) */
