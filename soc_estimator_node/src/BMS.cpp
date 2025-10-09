//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS.cpp
//
// Code generated for Simulink model 'BMS'.
//
// Model version                  : 10.80
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jun 25 19:23:09 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "BMS.h"
#include "rtmodel.h"
#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include <cstring>
#include <emmintrin.h>
#include "cmath"
#include "limits"



// Named constants for MATLAB Function: '<S44>/optimizer'
const double RMDscale{ 0.033333333333333333 };

const double RMVscale{ 0.16666666666666666 };

const int32_t degrees{ 11 };

const int32_t ny{ 2 };

const int32_t p{ 100 };

const double voff{ 0.66666666666666663 };

// Used by FromWorkspace Block: '<S1>/From Workspace19'
#ifndef rtInterpolate
# define rtInterpolate(v1,v2,f1,f2)    (((v1)==(v2))?((double)(v1)): (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif

#ifndef rtRound
# define rtRound(v)                    ( ((v) >= 0) ? std::floor((v) + 0.5) : std::ceil((v) - 0.5) )
#endif

static double look1_binlg(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);
static double look1_binlag(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);
static double look2_binlg(double u0, double u1, const double bp0[], const double
  bp1[], const double table[], const uint32_t maxIndex[], uint32_t stride);
static int32_t div_nde_s32_floor(int32_t numerator, int32_t denominator);

// private model entry point functions
extern void BMS_derivatives();
extern "C"
{
  double rtNaN { -std::numeric_limits<double>::quiet_NaN() };

  double rtInf { std::numeric_limits<double>::infinity() };

  double rtMinusInf { -std::numeric_limits<double>::infinity() };

  float rtNaNF { -std::numeric_limits<float>::quiet_NaN() };

  float rtInfF { std::numeric_limits<float>::infinity() };

  float rtMinusInfF { -std::numeric_limits<float>::infinity() };
}

extern "C"
{
  // Return rtNaN needed by the generated code.
  static double rtGetNaN(void)
  {
    return rtNaN;
  }

  // Return rtNaNF needed by the generated code.
  static float rtGetNaNF(void)
  {
    return rtNaNF;
  }
}

extern "C"
{
  // Test if value is infinite
  static bool rtIsInf(double value)
  {
    return std::isinf(value);
  }

  // Test if single-precision value is infinite
  static bool rtIsInfF(float value)
  {
    return std::isinf(value);
  }

  // Test if value is not a number
  static bool rtIsNaN(double value)
  {
    return std::isnan(value);
  }

  // Test if single-precision value is not a number
  static bool rtIsNaNF(float value)
  {
    return std::isnan(value);
  }
}

static double look1_binlg(double u0, const double bp0[], const double table[],
  uint32_t maxIndex)
{
  uint32_t bpIdx;
  uint32_t iLeft;
  uint32_t iRght;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'on'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'on'

  // Binary Search
  bpIdx = (maxIndex >> 1U);
  iLeft = 0U;
  iRght = maxIndex;
  while ((iRght - iLeft) > 1U) {
    if (u0 < bp0[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft = bpIdx;
    }

    bpIdx = ((iRght + iLeft) >> 1U);
  }

  double yL_0d0;

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  yL_0d0 = table[iLeft];
  return (((u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft])) * (table[iLeft +
           1U] - yL_0d0)) + yL_0d0;
}

static double look1_binlag(double u0, const double bp0[], const double table[],
  uint32_t maxIndex)
{
  double frac;
  double y;
  uint32_t iLeft;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'on'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'on'

  if (u0 < bp0[maxIndex]) {
    uint32_t bpIdx;
    uint32_t iRght;

    // Binary Search
    bpIdx = (maxIndex >> 1U);
    iLeft = 0U;
    iRght = maxIndex;
    while ((iRght - iLeft) > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = ((iRght + iLeft) >> 1U);
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex;
    frac = 0.0;
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Overflow mode: 'wrapping'

  if (iLeft == maxIndex) {
    y = table[iLeft];
  } else {
    double yL_0d0;
    yL_0d0 = table[iLeft];
    y = ((table[iLeft + 1U] - yL_0d0) * frac) + yL_0d0;
  }

  return y;
}

static double look2_binlg(double u0, double u1, const double bp0[], const double
  bp1[], const double table[], const uint32_t maxIndex[], uint32_t stride)
{
  double fractions[2];
  uint32_t bpIdx;
  uint32_t iLeft;
  uint32_t iLeft_0;
  uint32_t iRght;

  // Column-major Lookup 2-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'on'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'on'

  // Binary Search
  bpIdx = (maxIndex[0U] >> 1U);
  iLeft = 0U;
  iRght = maxIndex[0U];
  while ((iRght - iLeft) > 1U) {
    if (u0 < bp0[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft = bpIdx;
    }

    bpIdx = ((iRght + iLeft) >> 1U);
  }

  fractions[0U] = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'on'

  // Binary Search
  bpIdx = (maxIndex[1U] >> 1U);
  iLeft_0 = 0U;
  iRght = maxIndex[1U];
  while ((iRght - iLeft_0) > 1U) {
    if (u1 < bp1[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft_0 = bpIdx;
    }

    bpIdx = ((iRght + iLeft_0) >> 1U);
  }

  double yL_0d0;
  double yL_0d1;

  // Column-major Interpolation 2-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  bpIdx = (iLeft_0 * stride) + iLeft;
  yL_0d0 = table[bpIdx];
  yL_0d0 += (table[bpIdx + 1U] - yL_0d0) * fractions[0U];
  bpIdx += stride;
  yL_0d1 = table[bpIdx];
  return ((((table[bpIdx + 1U] - yL_0d1) * fractions[0U]) + yL_0d1 - yL_0d0) *
          ((u1 - bp1[iLeft_0]) / (bp1[iLeft_0 + 1U] - bp1[iLeft_0]))) + yL_0d0;
}

static int32_t div_nde_s32_floor(int32_t numerator, int32_t denominator)
{
  return ((((numerator < 0) != (denominator < 0)) && ((numerator % denominator)
            != 0)) ? -1 : 0) + (numerator / denominator);
}

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
void BMS::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const double rt_ODE3_A[3]{
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const double rt_ODE3_B[3][3]{
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  double t { rtsiGetT(si) };

  double tnew { rtsiGetSolverStopTime(si) };

  double h { rtsiGetStepSize(si) };

  double *x { rtsiGetContStates(si) };

  ODE3_IntgData *id { static_cast<ODE3_IntgData *>(rtsiGetSolverData(si)) };

  double *y { id->y };

  double *f0 { id->f[0] };

  double *f1 { id->f[1] };

  double *f2 { id->f[2] };

  double hB[3];
  int i;
  int nXc { 2 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<unsigned int>(nXc)*sizeof(double));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  BMS_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  BMS_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  BMS_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

//
// Output and update for atomic system:
//    '<S11>/DataTypeConversion_L0'
//    '<S11>/DataTypeConversion_amax'
//    '<S11>/DataTypeConversion_amin'
//    '<S11>/DataTypeConversion_atrack'
//    '<S11>/DataTypeConversion_vset'
//
void BMS::DataTypeConversion_L0(double rtu_u, double *rty_y)
{
  *rty_y = rtu_u;
}

//
// Output and update for atomic system:
//    '<S76>/Sigmoid Layer'
//    '<S80>/Sigmoid Layer'
//    '<S84>/Sigmoid Layer'
//
void BMS::SigmoidLayer(const double rtu_In1[256], double rty_Out1[256])
{
  // Product: '<S79>/DivideOut' incorporates:
  //   Constant: '<S79>/Constant'
  //   Gain: '<S79>/Gain'
  //   Math: '<S79>/Exp'
  //   Sum: '<S79>/Add'
  //
  //  About '<S79>/Exp':
  //   Operator: exp

  for (int32_t i{0}; i < 256; i++) {
    rty_Out1[i] = 1.0 / (std::exp(-rtu_In1[i]) + 1.0);
  }

  // End of Product: '<S79>/DivideOut'
}

//
// Output and update for atomic system:
//    '<S88>/Tanh Layer'
//    '<S92>/Tanh Layer'
//
void BMS::TanhLayer(const double rtu_In1[256], double rty_Out1[256])
{
  // Trigonometry: '<S91>/Tanh'
  for (int32_t i{0}; i < 256; i++) {
    rty_Out1[i] = std::tanh(rtu_In1[i]);
  }

  // End of Trigonometry: '<S91>/Tanh'
}

//
// Output and update for atomic system:
//    '<S110>/Sigmoid Layer'
//    '<S114>/Sigmoid Layer'
//    '<S118>/Sigmoid Layer'
//
void BMS::SigmoidLayer_p(const double rtu_In1[128], double rty_Out1[128])
{
  // Product: '<S113>/DivideOut' incorporates:
  //   Constant: '<S113>/Constant'
  //   Gain: '<S113>/Gain'
  //   Math: '<S113>/Exp'
  //   Sum: '<S113>/Add'
  //
  //  About '<S113>/Exp':
  //   Operator: exp

  for (int32_t i{0}; i < 128; i++) {
    rty_Out1[i] = 1.0 / (std::exp(-rtu_In1[i]) + 1.0);
  }

  // End of Product: '<S113>/DivideOut'
}

//
// Output and update for atomic system:
//    '<S122>/Tanh Layer'
//    '<S126>/Tanh Layer'
//
void BMS::TanhLayer_b(const double rtu_In1[128], double rty_Out1[128])
{
  // Trigonometry: '<S125>/Tanh'
  for (int32_t i{0}; i < 128; i++) {
    rty_Out1[i] = std::tanh(rtu_In1[i]);
  }

  // End of Trigonometry: '<S125>/Tanh'
}

// Function for MATLAB Function: '<S44>/optimizer'
double BMS::norm(const double x[11])
{
  double y;
  y = 0.0;
  rtDW.scale_n = 3.3121686421112381E-170;
  for (rtDW.k_j = 0; rtDW.k_j < 11; rtDW.k_j++) {
    rtDW.absxk_b = std::abs(x[rtDW.k_j]);
    if (rtDW.absxk_b > rtDW.scale_n) {
      rtDW.t_ln = rtDW.scale_n / rtDW.absxk_b;
      y = ((y * rtDW.t_ln) * rtDW.t_ln) + 1.0;
      rtDW.scale_n = rtDW.absxk_b;
    } else {
      rtDW.t_ln = rtDW.absxk_b / rtDW.scale_n;
      y += rtDW.t_ln * rtDW.t_ln;
    }
  }

  return rtDW.scale_n * std::sqrt(y);
}

// Function for MATLAB Function: '<S44>/optimizer'
double BMS::maximum(const double x[11])
{
  double ex;
  int32_t idx;
  int32_t k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while (!exitg1 && (k < 12)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 12; k++) {
      double x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

// Function for MATLAB Function: '<S44>/optimizer'
double BMS::xnrm2(int32_t n, const double x[121], int32_t ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      rtDW.scale = 3.3121686421112381E-170;
      rtDW.kend = ix0 + n;
      for (rtDW.k = ix0; rtDW.k < rtDW.kend; rtDW.k++) {
        rtDW.absxk = std::abs(x[rtDW.k - 1]);
        if (rtDW.absxk > rtDW.scale) {
          rtDW.t_l = rtDW.scale / rtDW.absxk;
          y = ((y * rtDW.t_l) * rtDW.t_l) + 1.0;
          rtDW.scale = rtDW.absxk;
        } else {
          rtDW.t_l = rtDW.absxk / rtDW.scale;
          y += rtDW.t_l * rtDW.t_l;
        }
      }

      y = rtDW.scale * std::sqrt(y);
    }
  }

  return y;
}

double BMS::rt_hypotd_snf_e(double u0, double u1)
{
  double y;
  rtDW.a = std::abs(u0);
  rtDW.b = std::abs(u1);
  if (rtDW.a < rtDW.b) {
    rtDW.a /= rtDW.b;
    y = std::sqrt((rtDW.a * rtDW.a) + 1.0) * rtDW.b;
  } else if (rtDW.a > rtDW.b) {
    rtDW.b /= rtDW.a;
    y = std::sqrt((rtDW.b * rtDW.b) + 1.0) * rtDW.a;
  } else if (std::isnan(rtDW.b)) {
    y = (rtNaN);
  } else {
    y = rtDW.a * 1.4142135623730951;
  }

  return y;
}

// Function for MATLAB Function: '<S44>/optimizer'
void BMS::xgemv(int32_t b_m, int32_t n, const double b_A[121], int32_t ia0,
                const double x[121], int32_t ix0, double y[11])
{
  if ((b_m != 0) && (n != 0)) {
    if ((n - 1) >= 0) {
      std::memset(&y[0], 0, static_cast<uint32_t>(n) * sizeof(double));
    }

    rtDW.b_a = ((n - 1) * 11) + ia0;
    for (rtDW.b_iy = ia0; rtDW.b_iy <= rtDW.b_a; rtDW.b_iy += 11) {
      rtDW.c = 0.0;
      rtDW.d = rtDW.b_iy + b_m - 1;
      for (rtDW.ia = rtDW.b_iy; rtDW.ia <= rtDW.d; rtDW.ia++) {
        rtDW.c += x[ix0 + rtDW.ia - rtDW.b_iy - 1] * b_A[rtDW.ia - 1];
      }

      rtDW.ia = div_nde_s32_floor(rtDW.b_iy - ia0, 11);
      y[rtDW.ia] += rtDW.c;
    }
  }
}

// Function for MATLAB Function: '<S44>/optimizer'
void BMS::xgerc(int32_t b_m, int32_t n, double alpha1, int32_t ix0, const double
                y[11], double b_A[121], int32_t ia0)
{
  if (!(alpha1 == 0.0)) {
    rtDW.jA = ia0;
    for (rtDW.j = 0; rtDW.j < n; rtDW.j++) {
      rtDW.temp = y[rtDW.j];
      if (rtDW.temp != 0.0) {
        rtDW.temp *= alpha1;
        rtDW.b_f = b_m + rtDW.jA - 1;
        for (rtDW.ijA = rtDW.jA; rtDW.ijA <= rtDW.b_f; rtDW.ijA++) {
          b_A[rtDW.ijA - 1] += b_A[ix0 + rtDW.ijA - rtDW.jA - 1] * rtDW.temp;
        }
      }

      rtDW.jA += 11;
    }
  }
}

// Function for MATLAB Function: '<S44>/optimizer'
void BMS::KWIKfactor(const double b_Ac[3520], const int32_t iC[320], int32_t nA,
                     const double b_Linv[121], double D[121], double b_H[121],
                     int32_t n, double RLinv[121], double *Status)
{
  __m128d tmp;
  int32_t exitg1;
  bool exitg2;
  *Status = 1.0;
  std::memset(&RLinv[0], 0, 121U * sizeof(double));
  for (rtDW.k_i = 0; rtDW.k_i < nA; rtDW.k_i++) {
    rtDW.b_lastv = iC[rtDW.k_i];
    for (rtDW.b_coltop = 0; rtDW.b_coltop < 11; rtDW.b_coltop++) {
      rtDW.RLinv_g = 0.0;
      for (rtDW.knt_b = 0; rtDW.knt_b < 11; rtDW.knt_b++) {
        rtDW.RLinv_g += b_Ac[(320 * rtDW.knt_b) + rtDW.b_lastv - 1] * b_Linv[(11
          * rtDW.knt_b) + rtDW.b_coltop];
      }

      RLinv[rtDW.b_coltop + (11 * rtDW.k_i)] = rtDW.RLinv_g;
    }
  }

  std::memcpy(&rtDW.b_A[0], &RLinv[0], 121U * sizeof(double));
  std::memset(&rtDW.tau[0], 0, 11U * sizeof(double));
  std::memset(&rtDW.work[0], 0, 11U * sizeof(double));
  for (rtDW.k_i = 0; rtDW.k_i < 11; rtDW.k_i++) {
    rtDW.ii_e = (rtDW.k_i * 11) + rtDW.k_i;
    if ((rtDW.k_i + 1) < 11) {
      rtDW.RLinv_g = rtDW.b_A[rtDW.ii_e];
      rtDW.b_lastv = rtDW.ii_e + 2;
      rtDW.tau[rtDW.k_i] = 0.0;
      rtDW.beta1 = xnrm2(10 - rtDW.k_i, rtDW.b_A, rtDW.ii_e + 2);
      if (rtDW.beta1 != 0.0) {
        rtDW.b_A_d = rtDW.b_A[rtDW.ii_e];
        rtDW.beta1 = rt_hypotd_snf_e(rtDW.b_A_d, rtDW.beta1);
        if (rtDW.b_A_d >= 0.0) {
          rtDW.beta1 = -rtDW.beta1;
        }

        if (std::abs(rtDW.beta1) < 1.0020841800044864E-292) {
          rtDW.knt_b = 0;
          rtDW.coltop = rtDW.ii_e - rtDW.k_i + 11;
          do {
            rtDW.knt_b++;
            rtDW.vectorUB = (((rtDW.coltop - rtDW.ii_e - 1) / 2) << 1) +
              rtDW.ii_e + 2;
            rtDW.vectorUB_tmp = rtDW.vectorUB - 2;
            for (rtDW.b_coltop = rtDW.b_lastv; rtDW.b_coltop <=
                 rtDW.vectorUB_tmp; rtDW.b_coltop += 2) {
              tmp = _mm_loadu_pd(&rtDW.b_A[rtDW.b_coltop - 1]);
              _mm_storeu_pd(&rtDW.b_A[rtDW.b_coltop - 1], _mm_mul_pd(tmp,
                _mm_set1_pd(9.9792015476736E+291)));
            }

            for (rtDW.b_coltop = rtDW.vectorUB; rtDW.b_coltop <= rtDW.coltop;
                 rtDW.b_coltop++) {
              rtDW.b_A[rtDW.b_coltop - 1] *= 9.9792015476736E+291;
            }

            rtDW.beta1 *= 9.9792015476736E+291;
            rtDW.RLinv_g *= 9.9792015476736E+291;
          } while ((std::abs(rtDW.beta1) < 1.0020841800044864E-292) &&
                   (rtDW.knt_b < 20));

          rtDW.beta1 = rt_hypotd_snf_e(rtDW.RLinv_g, xnrm2(10 - rtDW.k_i,
            rtDW.b_A, rtDW.ii_e + 2));
          if (rtDW.RLinv_g >= 0.0) {
            rtDW.beta1 = -rtDW.beta1;
          }

          rtDW.tau[rtDW.k_i] = (rtDW.beta1 - rtDW.RLinv_g) / rtDW.beta1;
          rtDW.RLinv_g = 1.0 / (rtDW.RLinv_g - rtDW.beta1);
          for (rtDW.b_coltop = rtDW.b_lastv; rtDW.b_coltop <= rtDW.vectorUB_tmp;
               rtDW.b_coltop += 2) {
            tmp = _mm_loadu_pd(&rtDW.b_A[rtDW.b_coltop - 1]);
            _mm_storeu_pd(&rtDW.b_A[rtDW.b_coltop - 1], _mm_mul_pd(tmp,
              _mm_set1_pd(rtDW.RLinv_g)));
          }

          for (rtDW.b_coltop = rtDW.vectorUB; rtDW.b_coltop <= rtDW.coltop;
               rtDW.b_coltop++) {
            rtDW.b_A[rtDW.b_coltop - 1] *= rtDW.RLinv_g;
          }

          for (rtDW.b_lastv = 0; rtDW.b_lastv < rtDW.knt_b; rtDW.b_lastv++) {
            rtDW.beta1 *= 1.0020841800044864E-292;
          }

          rtDW.RLinv_g = rtDW.beta1;
        } else {
          rtDW.tau[rtDW.k_i] = (rtDW.beta1 - rtDW.b_A_d) / rtDW.beta1;
          rtDW.RLinv_g = 1.0 / (rtDW.b_A_d - rtDW.beta1);
          rtDW.b_coltop = rtDW.ii_e - rtDW.k_i + 11;
          rtDW.coltop = (((rtDW.b_coltop - rtDW.ii_e - 1) / 2) << 1) + rtDW.ii_e
            + 2;
          rtDW.vectorUB = rtDW.coltop - 2;
          for (rtDW.knt_b = rtDW.b_lastv; rtDW.knt_b <= rtDW.vectorUB;
               rtDW.knt_b += 2) {
            tmp = _mm_loadu_pd(&rtDW.b_A[rtDW.knt_b - 1]);
            _mm_storeu_pd(&rtDW.b_A[rtDW.knt_b - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (rtDW.RLinv_g)));
          }

          for (rtDW.knt_b = rtDW.coltop; rtDW.knt_b <= rtDW.b_coltop; rtDW.knt_b
               ++) {
            rtDW.b_A[rtDW.knt_b - 1] *= rtDW.RLinv_g;
          }

          rtDW.RLinv_g = rtDW.beta1;
        }
      }

      rtDW.b_A[rtDW.ii_e] = 1.0;
      if (rtDW.tau[rtDW.k_i] != 0.0) {
        rtDW.b_lastv = 11 - rtDW.k_i;
        rtDW.knt_b = rtDW.ii_e - rtDW.k_i + 10;
        while ((rtDW.b_lastv > 0) && (rtDW.b_A[rtDW.knt_b] == 0.0)) {
          rtDW.b_lastv--;
          rtDW.knt_b--;
        }

        rtDW.knt_b = 10 - rtDW.k_i;
        exitg2 = false;
        while (!exitg2 && (rtDW.knt_b > 0)) {
          rtDW.b_coltop = ((rtDW.knt_b - 1) * 11) + rtDW.ii_e + 11;
          rtDW.coltop = rtDW.b_coltop;
          do {
            exitg1 = 0;
            if ((rtDW.coltop + 1) <= (rtDW.b_coltop + rtDW.b_lastv)) {
              if (rtDW.b_A[rtDW.coltop] != 0.0) {
                exitg1 = 1;
              } else {
                rtDW.coltop++;
              }
            } else {
              rtDW.knt_b--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        rtDW.b_lastv = 0;
        rtDW.knt_b = 0;
      }

      if (rtDW.b_lastv > 0) {
        xgemv(rtDW.b_lastv, rtDW.knt_b, rtDW.b_A, rtDW.ii_e + 12, rtDW.b_A,
              rtDW.ii_e + 1, rtDW.work);
        xgerc(rtDW.b_lastv, rtDW.knt_b, -rtDW.tau[rtDW.k_i], rtDW.ii_e + 1,
              rtDW.work, rtDW.b_A, rtDW.ii_e + 12);
      }

      rtDW.b_A[rtDW.ii_e] = rtDW.RLinv_g;
    } else {
      rtDW.tau[10] = 0.0;
    }
  }

  for (rtDW.k_i = 0; rtDW.k_i < 11; rtDW.k_i++) {
    for (rtDW.ii_e = 0; rtDW.ii_e <= rtDW.k_i; rtDW.ii_e++) {
      rtDW.R[rtDW.ii_e + (11 * rtDW.k_i)] = rtDW.b_A[(11 * rtDW.k_i) + rtDW.ii_e];
    }

    for (rtDW.ii_e = rtDW.k_i + 2; rtDW.ii_e < 12; rtDW.ii_e++) {
      rtDW.R[rtDW.ii_e + (11 * rtDW.k_i) - 1] = 0.0;
    }

    rtDW.work[rtDW.k_i] = 0.0;
  }

  for (rtDW.k_i = 10; rtDW.k_i >= 0; rtDW.k_i--) {
    rtDW.b_lastv = (rtDW.k_i * 11) + rtDW.k_i + 12;
    if ((rtDW.k_i + 1) < 11) {
      rtDW.b_A[rtDW.b_lastv - 12] = 1.0;
      if (rtDW.tau[rtDW.k_i] != 0.0) {
        rtDW.knt_b = 11 - rtDW.k_i;
        rtDW.b_coltop = rtDW.b_lastv - rtDW.k_i;
        while ((rtDW.knt_b > 0) && (rtDW.b_A[rtDW.b_coltop - 2] == 0.0)) {
          rtDW.knt_b--;
          rtDW.b_coltop--;
        }

        rtDW.b_coltop = 10 - rtDW.k_i;
        exitg2 = false;
        while (!exitg2 && (rtDW.b_coltop > 0)) {
          rtDW.coltop = ((rtDW.b_coltop - 1) * 11) + rtDW.b_lastv;
          rtDW.ii_e = rtDW.coltop;
          do {
            exitg1 = 0;
            if (rtDW.ii_e <= (rtDW.coltop + rtDW.knt_b - 1)) {
              if (rtDW.b_A[rtDW.ii_e - 1] != 0.0) {
                exitg1 = 1;
              } else {
                rtDW.ii_e++;
              }
            } else {
              rtDW.b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        rtDW.knt_b = 0;
        rtDW.b_coltop = 0;
      }

      if (rtDW.knt_b > 0) {
        xgemv(rtDW.knt_b, rtDW.b_coltop, rtDW.b_A, rtDW.b_lastv, rtDW.b_A,
              rtDW.b_lastv - 11, rtDW.work);
        xgerc(rtDW.knt_b, rtDW.b_coltop, -rtDW.tau[rtDW.k_i], rtDW.b_lastv - 11,
              rtDW.work, rtDW.b_A, rtDW.b_lastv);
      }

      rtDW.b_coltop = rtDW.b_lastv - rtDW.k_i;
      rtDW.coltop = (((rtDW.b_coltop - rtDW.b_lastv + 10) / 2) << 1) +
        rtDW.b_lastv - 10;
      rtDW.vectorUB = rtDW.coltop - 2;
      for (rtDW.knt_b = rtDW.b_lastv - 10; rtDW.knt_b <= rtDW.vectorUB;
           rtDW.knt_b += 2) {
        tmp = _mm_loadu_pd(&rtDW.b_A[rtDW.knt_b - 1]);
        _mm_storeu_pd(&rtDW.b_A[rtDW.knt_b - 1], _mm_mul_pd(tmp, _mm_set1_pd
          (-rtDW.tau[rtDW.k_i])));
      }

      for (rtDW.knt_b = rtDW.coltop; rtDW.knt_b < rtDW.b_coltop; rtDW.knt_b++) {
        rtDW.b_A[rtDW.knt_b - 1] *= -rtDW.tau[rtDW.k_i];
      }
    }

    rtDW.b_A[rtDW.b_lastv - 12] = 1.0 - rtDW.tau[rtDW.k_i];
    for (rtDW.knt_b = 0; rtDW.knt_b < rtDW.k_i; rtDW.knt_b++) {
      rtDW.b_A[rtDW.b_lastv - rtDW.knt_b - 13] = 0.0;
    }
  }

  rtDW.k_i = 0;
  do {
    exitg1 = 0;
    if (rtDW.k_i <= (nA - 1)) {
      if (std::abs(rtDW.R[(11 * rtDW.k_i) + rtDW.k_i]) < 1.0E-12) {
        *Status = -2.0;
        exitg1 = 1;
      } else {
        rtDW.k_i++;
      }
    } else {
      for (rtDW.k_i = 0; rtDW.k_i < n; rtDW.k_i++) {
        for (rtDW.ii_e = 0; rtDW.ii_e < n; rtDW.ii_e++) {
          rtDW.RLinv_g = 0.0;
          for (rtDW.b_coltop = 0; rtDW.b_coltop < 11; rtDW.b_coltop++) {
            rtDW.RLinv_g += b_Linv[(11 * rtDW.k_i) + rtDW.b_coltop] * rtDW.b_A
              [(11 * rtDW.ii_e) + rtDW.b_coltop];
          }

          rtDW.TL[rtDW.k_i + (11 * rtDW.ii_e)] = rtDW.RLinv_g;
        }
      }

      std::memset(&RLinv[0], 0, 121U * sizeof(double));
      for (rtDW.k_i = nA; rtDW.k_i >= 1; rtDW.k_i--) {
        rtDW.b_coltop = (rtDW.k_i - 1) * 11;
        rtDW.knt_b = rtDW.k_i + rtDW.b_coltop - 1;
        RLinv[rtDW.knt_b] = 1.0;
        for (rtDW.ii_e = rtDW.k_i; rtDW.ii_e <= nA; rtDW.ii_e++) {
          rtDW.coltop = ((rtDW.ii_e - 1) * 11) + rtDW.k_i - 1;
          RLinv[rtDW.coltop] /= rtDW.R[rtDW.knt_b];
        }

        if (rtDW.k_i > 1) {
          for (rtDW.ii_e = 0; rtDW.ii_e <= (rtDW.k_i - 2); rtDW.ii_e++) {
            for (rtDW.b_lastv = rtDW.k_i; rtDW.b_lastv <= nA; rtDW.b_lastv++) {
              rtDW.knt_b = (rtDW.b_lastv - 1) * 11;
              rtDW.coltop = rtDW.knt_b + rtDW.ii_e;
              RLinv[rtDW.coltop] -= RLinv[rtDW.knt_b + rtDW.k_i - 1] *
                rtDW.R[rtDW.b_coltop + rtDW.ii_e];
            }
          }
        }
      }

      for (rtDW.k_i = 0; rtDW.k_i < n; rtDW.k_i++) {
        for (rtDW.ii_e = rtDW.k_i + 1; rtDW.ii_e <= n; rtDW.ii_e++) {
          rtDW.b_coltop = ((rtDW.ii_e - 1) * 11) + rtDW.k_i;
          b_H[rtDW.b_coltop] = 0.0;
          for (rtDW.b_lastv = nA + 1; rtDW.b_lastv <= n; rtDW.b_lastv++) {
            rtDW.knt_b = (rtDW.b_lastv - 1) * 11;
            b_H[rtDW.b_coltop] -= rtDW.TL[rtDW.knt_b + rtDW.ii_e - 1] *
              rtDW.TL[rtDW.knt_b + rtDW.k_i];
          }

          b_H[rtDW.ii_e + (11 * rtDW.k_i) - 1] = b_H[rtDW.b_coltop];
        }
      }

      for (rtDW.k_i = 0; rtDW.k_i < nA; rtDW.k_i++) {
        for (rtDW.ii_e = 0; rtDW.ii_e < n; rtDW.ii_e++) {
          rtDW.b_coltop = (11 * rtDW.k_i) + rtDW.ii_e;
          D[rtDW.b_coltop] = 0.0;
          for (rtDW.b_lastv = rtDW.k_i + 1; rtDW.b_lastv <= nA; rtDW.b_lastv++)
          {
            rtDW.knt_b = (rtDW.b_lastv - 1) * 11;
            D[rtDW.b_coltop] += rtDW.TL[rtDW.knt_b + rtDW.ii_e] *
              RLinv[rtDW.knt_b + rtDW.k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S44>/optimizer'
void BMS::DropConstraint(int32_t kDrop, bool iA[320], int32_t *nA, int32_t iC
  [320])
{
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      for (int32_t i{kDrop}; i < *nA; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

// Function for MATLAB Function: '<S44>/optimizer'
void BMS::qpkwik(const double b_Linv[121], const double b_Hinv[121], const
                 double f[11], const double b_Ac[3520], const double b[320],
                 bool iA[320], int32_t maxiter, double FeasTol, double x[11],
                 double lambda[320], int32_t *status)
{
  __m128d tmp;
  __m128d tmp_0;
  int32_t exitg1;
  int32_t exitg3;
  bool exitg2;
  bool exitg4;
  bool guard1;
  bool guard2;
  std::memset(&x[0], 0, 11U * sizeof(double));
  std::memset(&lambda[0], 0, 320U * sizeof(double));
  *status = 1;
  std::memset(&rtDW.r[0], 0, 11U * sizeof(double));
  rtDW.rMin = 0.0;
  rtDW.cTolComputed = false;
  for (rtDW.i_b = 0; rtDW.i_b < 320; rtDW.i_b++) {
    rtDW.cTol[rtDW.i_b] = 1.0;
    rtDW.iC[rtDW.i_b] = 0;
  }

  rtDW.nA = 0;
  for (rtDW.tmp = 0; rtDW.tmp < 320; rtDW.tmp++) {
    if (iA[rtDW.tmp]) {
      rtDW.nA++;
      rtDW.iC[rtDW.nA - 1] = rtDW.tmp + 1;
    }
  }

  guard1 = false;
  if (rtDW.nA > 0) {
    std::memset(&rtDW.Opt[0], 0, 22U * sizeof(double));
    for (rtDW.i_b = 0; rtDW.i_b < 11; rtDW.i_b++) {
      rtDW.Rhs[rtDW.i_b] = f[rtDW.i_b];
      rtDW.Rhs[rtDW.i_b + 11] = 0.0;
    }

    rtDW.DualFeasible = false;
    rtDW.tmp = static_cast<int32_t>(std::round(0.3 * static_cast<double>(rtDW.nA)));
    rtDW.ColdReset = false;
    do {
      exitg3 = 0;
      if (!rtDW.DualFeasible && (rtDW.nA > 0) && (*status <= maxiter)) {
        KWIKfactor(b_Ac, rtDW.iC, rtDW.nA, b_Linv, rtDW.D, rtDW.b_H, degrees,
                   rtDW.RLinv, &rtDW.Xnorm0);
        if (rtDW.Xnorm0 < 0.0) {
          if (rtDW.ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            rtDW.nA = 0;
            std::memset(&iA[0], 0, 320U * sizeof(bool));
            std::memset(&rtDW.iC[0], 0, 320U * sizeof(int32_t));
            rtDW.ColdReset = true;
          }
        } else {
          for (rtDW.i_b = 0; rtDW.i_b < rtDW.nA; rtDW.i_b++) {
            rtDW.Rhs[rtDW.i_b + 11] = b[rtDW.iC[rtDW.i_b] - 1];
            for (rtDW.iSave = rtDW.i_b + 1; rtDW.iSave <= rtDW.nA; rtDW.iSave++)
            {
              rtDW.U_tmp = (11 * rtDW.i_b) + rtDW.iSave - 1;
              rtDW.U[rtDW.U_tmp] = 0.0;
              for (rtDW.iC_d = 0; rtDW.iC_d < rtDW.nA; rtDW.iC_d++) {
                rtDW.U[rtDW.U_tmp] += rtDW.RLinv[(11 * rtDW.iC_d) + rtDW.iSave -
                  1] * rtDW.RLinv[(11 * rtDW.iC_d) + rtDW.i_b];
              }

              rtDW.U[rtDW.i_b + (11 * (rtDW.iSave - 1))] = rtDW.U[rtDW.U_tmp];
            }
          }

          for (rtDW.i_b = 0; rtDW.i_b < 11; rtDW.i_b++) {
            rtDW.Xnorm0 = 0.0;
            for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
              rtDW.Xnorm0 += rtDW.b_H[(11 * rtDW.iC_d) + rtDW.i_b] *
                rtDW.Rhs[rtDW.iC_d];
            }

            rtDW.Opt[rtDW.i_b] = rtDW.Xnorm0;
            for (rtDW.iSave = 0; rtDW.iSave < rtDW.nA; rtDW.iSave++) {
              rtDW.Opt[rtDW.i_b] += rtDW.D[(11 * rtDW.iSave) + rtDW.i_b] *
                rtDW.Rhs[rtDW.iSave + 11];
            }
          }

          for (rtDW.i_b = 0; rtDW.i_b < rtDW.nA; rtDW.i_b++) {
            rtDW.Xnorm0 = 0.0;
            for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
              rtDW.Xnorm0 += rtDW.D[(11 * rtDW.i_b) + rtDW.iC_d] *
                rtDW.Rhs[rtDW.iC_d];
            }

            rtDW.Opt[rtDW.i_b + 11] = rtDW.Xnorm0;
            for (rtDW.iSave = 0; rtDW.iSave < rtDW.nA; rtDW.iSave++) {
              rtDW.Opt[rtDW.i_b + 11] += rtDW.U[(11 * rtDW.iSave) + rtDW.i_b] *
                rtDW.Rhs[rtDW.iSave + 11];
            }
          }

          rtDW.Xnorm0 = -1.0E-12;
          rtDW.i_b = -1;
          for (rtDW.iSave = 0; rtDW.iSave < rtDW.nA; rtDW.iSave++) {
            rtDW.cMin = rtDW.Opt[rtDW.iSave + 11];
            lambda[rtDW.iC[rtDW.iSave] - 1] = rtDW.cMin;
            if ((rtDW.cMin < rtDW.Xnorm0) && ((rtDW.iSave + 1) <= rtDW.nA)) {
              rtDW.i_b = rtDW.iSave;
              rtDW.Xnorm0 = rtDW.cMin;
            }
          }

          if ((rtDW.i_b + 1) <= 0) {
            rtDW.DualFeasible = true;
            std::memcpy(&x[0], &rtDW.Opt[0], 11U * sizeof(double));
          } else {
            (*status)++;
            if (rtDW.tmp <= 5) {
              rtDW.iC_d = 5;
            } else {
              rtDW.iC_d = rtDW.tmp;
            }

            if (*status > rtDW.iC_d) {
              rtDW.nA = 0;
              std::memset(&iA[0], 0, 320U * sizeof(bool));
              std::memset(&rtDW.iC[0], 0, 320U * sizeof(int32_t));
              rtDW.ColdReset = true;
            } else {
              lambda[rtDW.iC[rtDW.i_b] - 1] = 0.0;
              DropConstraint(rtDW.i_b + 1, iA, &rtDW.nA, rtDW.iC);
            }
          }
        }
      } else {
        if (rtDW.nA <= 0) {
          std::memset(&lambda[0], 0, 320U * sizeof(double));
          for (rtDW.tmp = 0; rtDW.tmp < 11; rtDW.tmp++) {
            rtDW.Xnorm0 = 0.0;
            for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
              rtDW.Xnorm0 += -b_Hinv[(11 * rtDW.iC_d) + rtDW.tmp] * f[rtDW.iC_d];
            }

            x[rtDW.tmp] = rtDW.Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (rtDW.tmp = 0; rtDW.tmp < 11; rtDW.tmp++) {
      rtDW.Xnorm0 = 0.0;
      for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
        rtDW.Xnorm0 += -b_Hinv[(11 * rtDW.iC_d) + rtDW.tmp] * f[rtDW.iC_d];
      }

      x[rtDW.tmp] = rtDW.Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    rtDW.Xnorm0 = norm(x);
    exitg2 = false;
    while (!exitg2 && (*status <= maxiter)) {
      rtDW.cMin = -FeasTol;
      rtDW.tmp = -1;
      for (rtDW.i_b = 0; rtDW.i_b < 320; rtDW.i_b++) {
        if (!rtDW.cTolComputed) {
          for (rtDW.iSave = 0; rtDW.iSave < 11; rtDW.iSave++) {
            rtDW.z[rtDW.iSave] = std::abs(b_Ac[(320 * rtDW.iSave) + rtDW.i_b] *
              x[rtDW.iSave]);
          }

          rtDW.cTol[rtDW.i_b] = std::fmax(rtDW.cTol[rtDW.i_b], maximum(rtDW.z));
        }

        if (!iA[rtDW.i_b]) {
          rtDW.t = 0.0;
          for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
            rtDW.t += b_Ac[(320 * rtDW.iC_d) + rtDW.i_b] * x[rtDW.iC_d];
          }

          rtDW.cVal = (rtDW.t - b[rtDW.i_b]) / rtDW.cTol[rtDW.i_b];
          if (rtDW.cVal < rtDW.cMin) {
            rtDW.cMin = rtDW.cVal;
            rtDW.tmp = rtDW.i_b;
          }
        }
      }

      rtDW.cTolComputed = true;
      if ((rtDW.tmp + 1) <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if (((rtDW.tmp + 1) > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (rtDW.nA == 0) {
              for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
                rtDW.cMin = 0.0;
                for (rtDW.i_b = 0; rtDW.i_b < 11; rtDW.i_b++) {
                  rtDW.cMin += b_Hinv[(11 * rtDW.i_b) + rtDW.iC_d] * b_Ac[(320 *
                    rtDW.i_b) + rtDW.tmp];
                }

                rtDW.z[rtDW.iC_d] = rtDW.cMin;
              }

              guard2 = true;
            } else {
              KWIKfactor(b_Ac, rtDW.iC, rtDW.nA, b_Linv, rtDW.D, rtDW.b_H,
                         degrees, rtDW.RLinv, &rtDW.cMin);
              if (rtDW.cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (rtDW.iC_d = 0; rtDW.iC_d <= 118; rtDW.iC_d += 2) {
                  tmp_0 = _mm_loadu_pd(&rtDW.b_H[rtDW.iC_d]);
                  _mm_storeu_pd(&rtDW.U[rtDW.iC_d], _mm_mul_pd(tmp_0,
                    _mm_set1_pd(-1.0)));
                }

                for (rtDW.iC_d = 120; rtDW.iC_d < 121; rtDW.iC_d++) {
                  rtDW.U[rtDW.iC_d] = -rtDW.b_H[rtDW.iC_d];
                }

                for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
                  rtDW.cMin = 0.0;
                  for (rtDW.i_b = 0; rtDW.i_b < 11; rtDW.i_b++) {
                    rtDW.cMin += rtDW.U[(11 * rtDW.i_b) + rtDW.iC_d] * b_Ac[(320
                      * rtDW.i_b) + rtDW.tmp];
                  }

                  rtDW.z[rtDW.iC_d] = rtDW.cMin;
                }

                for (rtDW.i_b = 0; rtDW.i_b < rtDW.nA; rtDW.i_b++) {
                  rtDW.t = 0.0;
                  for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
                    rtDW.t += b_Ac[(320 * rtDW.iC_d) + rtDW.tmp] * rtDW.D[(11 *
                      rtDW.i_b) + rtDW.iC_d];
                  }

                  rtDW.r[rtDW.i_b] = rtDW.t;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              rtDW.i_b = 0;
              rtDW.cMin = 0.0;
              rtDW.DualFeasible = true;
              rtDW.ColdReset = true;
              if (rtDW.nA > 0) {
                rtDW.iSave = 0;
                exitg4 = false;
                while (!exitg4 && (rtDW.iSave <= (rtDW.nA - 1))) {
                  if (rtDW.r[rtDW.iSave] >= 1.0E-12) {
                    rtDW.ColdReset = false;
                    exitg4 = true;
                  } else {
                    rtDW.iSave++;
                  }
                }
              }

              if ((rtDW.nA != 0) && !rtDW.ColdReset) {
                for (rtDW.iSave = 0; rtDW.iSave < rtDW.nA; rtDW.iSave++) {
                  rtDW.cVal = rtDW.r[rtDW.iSave];
                  if (rtDW.cVal > 1.0E-12) {
                    rtDW.cVal = lambda[rtDW.iC[rtDW.iSave] - 1] / rtDW.cVal;
                    if ((rtDW.i_b == 0) || (rtDW.cVal < rtDW.rMin)) {
                      rtDW.rMin = rtDW.cVal;
                      rtDW.i_b = rtDW.iSave + 1;
                    }
                  }
                }

                if (rtDW.i_b > 0) {
                  rtDW.cMin = rtDW.rMin;
                  rtDW.DualFeasible = false;
                }
              }

              rtDW.cVal = 0.0;
              for (rtDW.iSave = 0; rtDW.iSave < 11; rtDW.iSave++) {
                rtDW.cVal += b_Ac[(320 * rtDW.iSave) + rtDW.tmp] *
                  rtDW.z[rtDW.iSave];
              }

              if (rtDW.cVal <= 0.0) {
                rtDW.cVal = 0.0;
                rtDW.ColdReset = true;
              } else {
                rtDW.t = 0.0;
                for (rtDW.iC_d = 0; rtDW.iC_d < 11; rtDW.iC_d++) {
                  rtDW.t += b_Ac[(320 * rtDW.iC_d) + rtDW.tmp] * x[rtDW.iC_d];
                }

                rtDW.cVal = (b[rtDW.tmp] - rtDW.t) / rtDW.cVal;
                rtDW.ColdReset = false;
              }

              if (rtDW.DualFeasible && rtDW.ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (rtDW.ColdReset) {
                  rtDW.t = rtDW.cMin;
                } else if (rtDW.DualFeasible) {
                  rtDW.t = rtDW.cVal;
                } else if (rtDW.cMin < rtDW.cVal) {
                  rtDW.t = rtDW.cMin;
                } else {
                  rtDW.t = rtDW.cVal;
                }

                for (rtDW.iSave = 0; rtDW.iSave < rtDW.nA; rtDW.iSave++) {
                  rtDW.iC_d = rtDW.iC[rtDW.iSave];
                  lambda[rtDW.iC_d - 1] -= rtDW.t * rtDW.r[rtDW.iSave];
                  if ((rtDW.iC_d <= 320) && (lambda[rtDW.iC_d - 1] < 0.0)) {
                    lambda[rtDW.iC_d - 1] = 0.0;
                  }
                }

                lambda[rtDW.tmp] += rtDW.t;
                std::frexp(1.0, &rtDW.exponent);
                if (std::abs(rtDW.t - rtDW.cMin) < 2.2204460492503131E-16) {
                  DropConstraint(rtDW.i_b, iA, &rtDW.nA, rtDW.iC);
                }

                if (!rtDW.ColdReset) {
                  for (rtDW.iC_d = 0; rtDW.iC_d <= 8; rtDW.iC_d += 2) {
                    tmp_0 = _mm_loadu_pd(&rtDW.z[rtDW.iC_d]);
                    tmp = _mm_loadu_pd(&x[rtDW.iC_d]);
                    _mm_storeu_pd(&x[rtDW.iC_d], _mm_add_pd(_mm_mul_pd
                      (_mm_set1_pd(rtDW.t), tmp_0), tmp));
                  }

                  for (rtDW.iC_d = 10; rtDW.iC_d < 11; rtDW.iC_d++) {
                    x[rtDW.iC_d] += rtDW.t * rtDW.z[rtDW.iC_d];
                  }

                  std::frexp(1.0, &rtDW.b_exponent);
                  if (std::abs(rtDW.t - rtDW.cVal) < 2.2204460492503131E-16) {
                    if (rtDW.nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      rtDW.nA++;
                      rtDW.iC[rtDW.nA - 1] = rtDW.tmp + 1;
                      rtDW.i_b = rtDW.nA - 1;
                      exitg4 = false;
                      while (!exitg4 && ((rtDW.i_b + 1) > 1)) {
                        rtDW.iC_d = rtDW.iC[rtDW.i_b - 1];
                        if (rtDW.iC[rtDW.i_b] > rtDW.iC_d) {
                          exitg4 = true;
                        } else {
                          rtDW.iSave = rtDW.iC[rtDW.i_b];
                          rtDW.iC[rtDW.i_b] = rtDW.iC_d;
                          rtDW.iC[rtDW.i_b - 1] = rtDW.iSave;
                          rtDW.i_b--;
                        }
                      }

                      iA[rtDW.tmp] = true;
                      rtDW.tmp = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            rtDW.cMin = norm(x);
            if (std::abs(rtDW.cMin - rtDW.Xnorm0) > 0.001) {
              rtDW.Xnorm0 = rtDW.cMin;
              for (rtDW.tmp = 0; rtDW.tmp < 320; rtDW.tmp++) {
                rtDW.cTol[rtDW.tmp] = std::fmax(std::abs(b[rtDW.tmp]), 1.0);
              }

              rtDW.cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

double BMS::xnrm2_e(int32_t n, const double x[2], int32_t ix0)
{
  double y;

  // Start for MATLABSystem: '<S46>/MATLAB System' incorporates:
  //   MATLABSystem: '<S48>/MATLAB System'

  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      int32_t kend;
      rtDW.scale_l = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_t k{ix0}; k < kend; k++) {
        rtDW.absxk_o = std::abs(x[k - 1]);
        if (rtDW.absxk_o > rtDW.scale_l) {
          rtDW.t_b = rtDW.scale_l / rtDW.absxk_o;
          y = ((y * rtDW.t_b) * rtDW.t_b) + 1.0;
          rtDW.scale_l = rtDW.absxk_o;
        } else {
          rtDW.t_b = rtDW.absxk_o / rtDW.scale_l;
          y += rtDW.t_b * rtDW.t_b;
        }
      }

      y = rtDW.scale_l * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<S46>/MATLAB System'
  return y;
}

double BMS::qrFactor(double A, double S, double Ns)
{
  int32_t jA;

  // Start for MATLABSystem: '<S46>/MATLAB System'
  rtDW.M[0] = S * A;
  rtDW.M[1] = Ns;

  // Start for MATLABSystem: '<S46>/MATLAB System'
  for (rtDW.b_i = 0; rtDW.b_i < 1; rtDW.b_i++) {
    rtDW.b_atmp = rtDW.M[0];
    rtDW.beta1_d = xnrm2_e(1, rtDW.M, 2);
    if (rtDW.beta1_d != 0.0) {
      rtDW.beta1_d = rt_hypotd_snf_e(rtDW.M[0], rtDW.beta1_d);
      if (rtDW.M[0] >= 0.0) {
        rtDW.beta1_d = -rtDW.beta1_d;
      }

      if (std::abs(rtDW.beta1_d) < 1.0020841800044864E-292) {
        rtDW.knt_j = -1;
        do {
          rtDW.knt_j++;
          rtDW.M_d = rtDW.M[1];
          for (jA = 2; jA < 3; jA++) {
            rtDW.M_d *= 9.9792015476736E+291;
          }

          rtDW.M[1] = rtDW.M_d;
          rtDW.beta1_d *= 9.9792015476736E+291;
          rtDW.b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(rtDW.beta1_d) < 1.0020841800044864E-292) &&
                 ((rtDW.knt_j + 1) < 20));

        rtDW.beta1_d = rt_hypotd_snf_e(rtDW.b_atmp, xnrm2_e(1, rtDW.M, 2));
        if (rtDW.b_atmp >= 0.0) {
          rtDW.beta1_d = -rtDW.beta1_d;
        }

        rtDW.b_atmp = 1.0 / (rtDW.b_atmp - rtDW.beta1_d);
        for (jA = 2; jA < 3; jA++) {
          rtDW.M_d *= rtDW.b_atmp;
        }

        rtDW.M[1] = rtDW.M_d;
        for (jA = 0; jA <= rtDW.knt_j; jA++) {
          rtDW.beta1_d *= 1.0020841800044864E-292;
        }

        rtDW.b_atmp = rtDW.beta1_d;
      } else {
        rtDW.b_atmp = 1.0 / (rtDW.M[0] - rtDW.beta1_d);
        rtDW.M_d = rtDW.M[1];
        for (jA = 2; jA < 3; jA++) {
          rtDW.M_d *= rtDW.b_atmp;
        }

        rtDW.M[1] = rtDW.M_d;
        rtDW.b_atmp = rtDW.beta1_d;
      }
    }

    rtDW.M[0] = rtDW.b_atmp;
  }

  return rtDW.M[0];
}

double BMS::trisolve(double A, double B_0)
{
  double b_B;

  // Start for MATLABSystem: '<S46>/MATLAB System'
  b_B = B_0;
  if (B_0 != 0.0) {
    b_B = B_0 / A;
  }

  // End of Start for MATLABSystem: '<S46>/MATLAB System'
  return b_B;
}

// Model step function
void BMS::step()
{
  static const double a_0[8]{ -0.3301555661035559, 0.70310075687703233,
    2.3953684765784464, 0.0168427389514272, 0.014998064620874017,
    -0.12773928729260592, -0.03629495755980721, 0.093561029792070483 };

  static const double b_a_0[8]{ -2.6815467761687617E-18, -0.00083695356892573463,
    2.1213201784522595, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double c_a_0[4]{ -1.3401977508309273, -0.758946048219423,
    -0.0016456771654181981, 0.0 };

  static const double d_a_0[16]{ 0.81873075307798171, 7.9960268863730558E-6,
    -0.020266516340000176, 0.0, -0.090634616406701243, 0.99998194235173521,
    0.04576843335426678, 0.0, -3.5759319333468191E-5, -7.1245318434864537E-9,
    1.0000180576482651, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const double e_a_0[8]{ -0.32531408031414283, 0.70311607150920608,
    2.3565524186002498, 0.016842738951427183, 0.0041761780917740953,
    -0.12774162761514304, -0.030363238519522526, 0.093561029792070413 };

  static const double f_a[8]{ -4.5730129127467358E-20, 0.010540925533894536,
    -1.8598968198349683E-5, -0.021081849426939074, 0.04714044841005019,
    -8.3177114405734167E-6, 0.0, 1.0 };

  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  static const double b_Mlim[320]{ 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.33333333333333337,
    0.33333333333333337, 0.33333333333333337, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.93333333333333346, 0.66666666666666663,
    0.93333333333333346, 0.66666666666666663, 0.93333333333333346,
    0.66666666666666663, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };

  static const double b_Mv[64640]{ 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 2.507217523872729E-19,
    2.574980159653073E-19, 2.574980159653073E-19, 2.6766241133235891E-19,
    2.6427427954334171E-19, 2.7105054312137611E-19, 2.7443867491039331E-19,
    2.8121493848842771E-19, 2.8121493848842771E-19, 2.8460307027744491E-19,
    2.8460307027744491E-19, 2.8460307027744491E-19, 2.8799120206646212E-19, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.10000000000000107, -2.6427427954334171E-19,
    0.1000000000000011, -2.7105054312137611E-19, 0.10000000000000112,
    -2.7443867491039331E-19, 0.10000000000000113, -2.8121493848842771E-19,
    0.10000000000000113, -2.8121493848842771E-19, 0.10000000000000113,
    -2.8460307027744491E-19, 0.10000000000000114, -2.8460307027744491E-19,
    0.10000000000000116, -2.8460307027744491E-19, 0.10000000000000119,
    -2.8799120206646212E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 2.7443867491039331E-19, 2.8121493848842771E-19,
    2.8121493848842771E-19, 2.8460307027744491E-19, 2.8460307027744491E-19,
    2.8460307027744491E-19, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3716922523120409E-19, 0.100000000000001,
    -2.439454888092385E-19, 0.10000000000000102, -2.473336205982557E-19,
    0.10000000000000103, -2.473336205982557E-19, 0.10000000000000105,
    -2.473336205982557E-19, 0.10000000000000106, -2.507217523872729E-19,
    0.10000000000000105, -2.574980159653073E-19, 0.10000000000000106,
    -2.574980159653073E-19, 0.10000000000000106, -2.6766241133235891E-19,
    0.10000000000000107, -2.6427427954334171E-19, 0.1000000000000011,
    -2.7105054312137611E-19, 0.10000000000000112, -2.7443867491039331E-19,
    0.10000000000000113, -2.8121493848842771E-19, 0.10000000000000113,
    -2.8121493848842771E-19, 0.10000000000000113, -2.8460307027744491E-19,
    0.10000000000000114, -2.8460307027744491E-19, 0.10000000000000116,
    -2.8460307027744491E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 2.7443867491039331E-19, 2.8121493848842771E-19,
    2.8121493848842771E-19, 2.8460307027744491E-19, 2.8460307027744491E-19, 0.0,
    0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3716922523120409E-19, 0.100000000000001, -2.439454888092385E-19,
    0.10000000000000102, -2.473336205982557E-19, 0.10000000000000103,
    -2.473336205982557E-19, 0.10000000000000105, -2.473336205982557E-19,
    0.10000000000000106, -2.507217523872729E-19, 0.10000000000000105,
    -2.574980159653073E-19, 0.10000000000000106, -2.574980159653073E-19,
    0.10000000000000106, -2.6766241133235891E-19, 0.10000000000000107,
    -2.6427427954334171E-19, 0.1000000000000011, -2.7105054312137611E-19,
    0.10000000000000112, -2.7443867491039331E-19, 0.10000000000000113,
    -2.8121493848842771E-19, 0.10000000000000113, -2.8121493848842771E-19,
    0.10000000000000113, -2.8460307027744491E-19, 0.10000000000000114,
    -2.8460307027744491E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 2.7443867491039331E-19, 2.8121493848842771E-19,
    2.8121493848842771E-19, 2.8460307027744491E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.10000000000000107, -2.6427427954334171E-19,
    0.1000000000000011, -2.7105054312137611E-19, 0.10000000000000112,
    -2.7443867491039331E-19, 0.10000000000000113, -2.8121493848842771E-19,
    0.10000000000000113, -2.8121493848842771E-19, 0.10000000000000113,
    -2.8460307027744491E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 2.7443867491039331E-19, 2.8121493848842771E-19,
    2.8121493848842771E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.10000000000000107, -2.6427427954334171E-19,
    0.1000000000000011, -2.7105054312137611E-19, 0.10000000000000112,
    -2.7443867491039331E-19, 0.10000000000000113, -2.8121493848842771E-19,
    0.10000000000000113, -2.8121493848842771E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 2.507217523872729E-19,
    2.574980159653073E-19, 2.574980159653073E-19, 2.6766241133235891E-19,
    2.6427427954334171E-19, 2.7105054312137611E-19, 2.7443867491039331E-19,
    2.8121493848842771E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.10000000000000107, -2.6427427954334171E-19,
    0.1000000000000011, -2.7105054312137611E-19, 0.10000000000000112,
    -2.7443867491039331E-19, 0.10000000000000113, -2.8121493848842771E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 2.7443867491039331E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3716922523120409E-19, 0.100000000000001,
    -2.439454888092385E-19, 0.10000000000000102, -2.473336205982557E-19,
    0.10000000000000103, -2.473336205982557E-19, 0.10000000000000105,
    -2.473336205982557E-19, 0.10000000000000106, -2.507217523872729E-19,
    0.10000000000000105, -2.574980159653073E-19, 0.10000000000000106,
    -2.574980159653073E-19, 0.10000000000000106, -2.6766241133235891E-19,
    0.10000000000000107, -2.6427427954334171E-19, 0.1000000000000011,
    -2.7105054312137611E-19, 0.10000000000000112, -2.7443867491039331E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19,
    2.7105054312137611E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3716922523120409E-19, 0.100000000000001, -2.439454888092385E-19,
    0.10000000000000102, -2.473336205982557E-19, 0.10000000000000103,
    -2.473336205982557E-19, 0.10000000000000105, -2.473336205982557E-19,
    0.10000000000000106, -2.507217523872729E-19, 0.10000000000000105,
    -2.574980159653073E-19, 0.10000000000000106, -2.574980159653073E-19,
    0.10000000000000106, -2.6766241133235891E-19, 0.10000000000000107,
    -2.6427427954334171E-19, 0.1000000000000011, -2.7105054312137611E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 2.6427427954334171E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.10000000000000107, -2.6427427954334171E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.507217523872729E-19, 2.574980159653073E-19,
    2.574980159653073E-19, 2.6766241133235891E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19,
    0.10000000000000106, -2.574980159653073E-19, 0.10000000000000106,
    -2.6766241133235891E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 2.0328790734103208E-19,
    2.0667603913004928E-19, 2.0667603913004928E-19, 2.1006417091906648E-19,
    2.1345230270808369E-19, 2.1684043449710089E-19, 2.2361669807513529E-19,
    2.2700482986415249E-19, 2.3039296165316969E-19, 2.3378109344218689E-19,
    2.3378109344218689E-19, 2.3716922523120409E-19, 2.439454888092385E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 2.473336205982557E-19,
    2.507217523872729E-19, 2.574980159653073E-19, 2.574980159653073E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3716922523120409E-19, 0.100000000000001, -2.439454888092385E-19,
    0.10000000000000102, -2.473336205982557E-19, 0.10000000000000103,
    -2.473336205982557E-19, 0.10000000000000105, -2.473336205982557E-19,
    0.10000000000000106, -2.507217523872729E-19, 0.10000000000000105,
    -2.574980159653073E-19, 0.10000000000000106, -2.574980159653073E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 2.507217523872729E-19,
    2.574980159653073E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.10000000000000103, -2.473336205982557E-19,
    0.10000000000000105, -2.473336205982557E-19, 0.10000000000000106,
    -2.507217523872729E-19, 0.10000000000000105, -2.574980159653073E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 2.507217523872729E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3716922523120409E-19, 0.100000000000001,
    -2.439454888092385E-19, 0.10000000000000102, -2.473336205982557E-19,
    0.10000000000000103, -2.473336205982557E-19, 0.10000000000000105,
    -2.473336205982557E-19, 0.10000000000000106, -2.507217523872729E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3716922523120409E-19, 0.100000000000001, -2.439454888092385E-19,
    0.10000000000000102, -2.473336205982557E-19, 0.10000000000000103,
    -2.473336205982557E-19, 0.10000000000000105, -2.473336205982557E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 2.0328790734103208E-19,
    2.0667603913004928E-19, 2.0667603913004928E-19, 2.1006417091906648E-19,
    2.1345230270808369E-19, 2.1684043449710089E-19, 2.2361669807513529E-19,
    2.2700482986415249E-19, 2.3039296165316969E-19, 2.3378109344218689E-19,
    2.3378109344218689E-19, 2.3716922523120409E-19, 2.439454888092385E-19,
    2.473336205982557E-19, 2.473336205982557E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3716922523120409E-19, 0.100000000000001,
    -2.439454888092385E-19, 0.10000000000000102, -2.473336205982557E-19,
    0.10000000000000103, -2.473336205982557E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19,
    2.3716922523120409E-19, 2.439454888092385E-19, 2.473336205982557E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3716922523120409E-19,
    0.100000000000001, -2.439454888092385E-19, 0.10000000000000102,
    -2.473336205982557E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19,
    2.439454888092385E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3716922523120409E-19, 0.100000000000001,
    -2.439454888092385E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 2.3039296165316969E-19,
    2.3378109344218689E-19, 2.3378109344218689E-19, 2.3716922523120409E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.10000000000000099,
    -2.3716922523120409E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 2.3378109344218689E-19, 2.3378109344218689E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19,
    0.10000000000000096, -2.2700482986415249E-19, 0.10000000000000098,
    -2.3039296165316969E-19, 0.10000000000000099, -2.3378109344218689E-19,
    0.10000000000000099, -2.3378109344218689E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 2.0328790734103208E-19,
    2.0667603913004928E-19, 2.0667603913004928E-19, 2.1006417091906648E-19,
    2.1345230270808369E-19, 2.1684043449710089E-19, 2.2361669807513529E-19,
    2.2700482986415249E-19, 2.3039296165316969E-19, 2.3378109344218689E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19,
    0.10000000000000098, -2.3039296165316969E-19, 0.10000000000000099,
    -2.3378109344218689E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 2.2361669807513529E-19, 2.2700482986415249E-19,
    2.3039296165316969E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.10000000000000094, -2.1684043449710089E-19,
    0.10000000000000095, -2.2361669807513529E-19, 0.10000000000000096,
    -2.2700482986415249E-19, 0.10000000000000098, -2.3039296165316969E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 2.2700482986415249E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.10000000000000095,
    -2.2361669807513529E-19, 0.10000000000000096, -2.2700482986415249E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 2.1684043449710089E-19,
    2.2361669807513529E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19,
    0.10000000000000091, -2.1345230270808369E-19, 0.10000000000000094,
    -2.1684043449710089E-19, 0.10000000000000095, -2.2361669807513529E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 2.1006417091906648E-19, 2.1345230270808369E-19,
    2.1684043449710089E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.10000000000000087,
    -2.0328790734103208E-19, 0.10000000000000088, -2.0667603913004928E-19,
    0.10000000000000089, -2.0667603913004928E-19, 0.10000000000000089,
    -2.1006417091906648E-19, 0.10000000000000091, -2.1345230270808369E-19,
    0.10000000000000094, -2.1684043449710089E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 2.0667603913004928E-19, 2.0667603913004928E-19,
    2.1006417091906648E-19, 2.1345230270808369E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19,
    0.10000000000000089, -2.1006417091906648E-19, 0.10000000000000091,
    -2.1345230270808369E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 2.0328790734103208E-19,
    2.0667603913004928E-19, 2.0667603913004928E-19, 2.1006417091906648E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19,
    0.10000000000000088, -2.0667603913004928E-19, 0.10000000000000089,
    -2.0667603913004928E-19, 0.10000000000000089, -2.1006417091906648E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 2.0328790734103208E-19, 2.0667603913004928E-19,
    2.0667603913004928E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.10000000000000089, -2.0667603913004928E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 2.0328790734103208E-19,
    2.0667603913004928E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000085, -1.9989977555201488E-19,
    0.10000000000000087, -2.0328790734103208E-19, 0.10000000000000088,
    -2.0667603913004928E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    1.9651164376299768E-19, 1.9651164376299768E-19, 1.9989977555201488E-19,
    2.0328790734103208E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19,
    0.10000000000000081, -1.9651164376299768E-19, 0.10000000000000082,
    -1.9989977555201488E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000085,
    -1.9989977555201488E-19, 0.10000000000000087, -2.0328790734103208E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 1.9651164376299768E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.10000000000000081,
    -1.9312351197398048E-19, 0.10000000000000081, -1.9651164376299768E-19,
    0.10000000000000082, -1.9989977555201488E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.10000000000000084, -1.9651164376299768E-19,
    0.10000000000000085, -1.9989977555201488E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 1.9651164376299768E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.10000000000000084,
    -1.9651164376299768E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 1.9651164376299768E-19,
    1.9989977555201488E-19, 1.9651164376299768E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19,
    0.10000000000000084, -1.9651164376299768E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8973538018496328E-19, 1.9312351197398048E-19,
    1.9651164376299768E-19, 1.9989977555201488E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.10000000000000082, -1.9989977555201488E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19,
    1.9312351197398048E-19, 1.9651164376299768E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.10000000000000075, -1.8634724839594607E-19,
    0.10000000000000077, -1.8634724839594607E-19, 0.10000000000000078,
    -1.8634724839594607E-19, 0.1000000000000008, -1.8973538018496328E-19,
    0.10000000000000081, -1.9312351197398048E-19, 0.10000000000000081,
    -1.9651164376299768E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8973538018496328E-19, 1.9312351197398048E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.1000000000000008,
    -1.8973538018496328E-19, 0.10000000000000081, -1.9312351197398048E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 1.8634724839594607E-19, 1.8973538018496328E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.10000000000000077,
    -1.8634724839594607E-19, 0.10000000000000078, -1.8634724839594607E-19,
    0.1000000000000008, -1.8973538018496328E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.8295911660692887E-19, 1.8634724839594607E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19,
    0.10000000000000078, -1.8634724839594607E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 1.8634724839594607E-19,
    1.8634724839594607E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000074, -1.7618285302889447E-19,
    0.10000000000000075, -1.8295911660692887E-19, 0.10000000000000075,
    -1.8634724839594607E-19, 0.10000000000000077, -1.8634724839594607E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.7618285302889447E-19, 1.8295911660692887E-19,
    1.8634724839594607E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.10000000000000071,
    -1.7279472123987727E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000073, -1.7618285302889447E-19, 0.10000000000000074,
    -1.7618285302889447E-19, 0.10000000000000075, -1.8295911660692887E-19,
    0.10000000000000075, -1.8634724839594607E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 1.8295911660692887E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.10000000000000075,
    -1.8295911660692887E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 1.7618285302889447E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19,
    0.10000000000000074, -1.7618285302889447E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5924219408380846E-19, 1.5924219408380846E-19, 1.5585406229479126E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6601845766184287E-19, 1.6601845766184287E-19, 1.6940658945086007E-19,
    1.6940658945086007E-19, 1.7279472123987727E-19, 1.7618285302889447E-19,
    1.7618285302889447E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.10000000000000073, -1.7618285302889447E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 1.7279472123987727E-19,
    1.7618285302889447E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000069,
    -1.6940658945086007E-19, 0.1000000000000007, -1.6940658945086007E-19,
    0.10000000000000071, -1.7279472123987727E-19, 0.10000000000000073,
    -1.7618285302889447E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19,
    1.6601845766184287E-19, 1.6940658945086007E-19, 1.6940658945086007E-19,
    1.7279472123987727E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.1000000000000007,
    -1.6940658945086007E-19, 0.10000000000000071, -1.7279472123987727E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 1.6940658945086007E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.10000000000000066, -1.6263032587282567E-19,
    0.10000000000000067, -1.6601845766184287E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000069, -1.6940658945086007E-19,
    0.1000000000000007, -1.6940658945086007E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19,
    1.6940658945086007E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19,
    0.10000000000000069, -1.6940658945086007E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6601845766184287E-19, 1.6601845766184287E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.10000000000000067, -1.6601845766184287E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 1.5585406229479126E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 1.6263032587282567E-19, 1.6601845766184287E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.10000000000000064, -1.6263032587282567E-19,
    0.10000000000000066, -1.6263032587282567E-19, 0.10000000000000067,
    -1.6601845766184287E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19,
    1.6263032587282567E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19,
    0.1000000000000006, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5585406229479126E-19, 0.10000000000000063, -1.6263032587282567E-19,
    0.10000000000000064, -1.6263032587282567E-19, 0.10000000000000066,
    -1.6263032587282567E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 1.6263032587282567E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19,
    0.10000000000000057, -1.5924219408380846E-19, 0.1000000000000006,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5585406229479126E-19,
    0.10000000000000063, -1.6263032587282567E-19, 0.10000000000000064,
    -1.6263032587282567E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 1.6263032587282567E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.10000000000000063,
    -1.6263032587282567E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 1.5924219408380846E-19,
    1.5585406229479126E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19,
    0.1000000000000006, -1.5585406229479126E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 1.4907779871675686E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5246593050577406E-19, 1.5924219408380846E-19,
    1.5924219408380846E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000057, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5924219408380846E-19, 0.1000000000000006, -1.5924219408380846E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 1.5924219408380846E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19,
    0.10000000000000051, -1.4568966692773966E-19, 0.10000000000000052,
    -1.4907779871675686E-19, 0.10000000000000053, -1.5246593050577406E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000057,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5924219408380846E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19,
    1.5246593050577406E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19,
    0.10000000000000053, -1.5246593050577406E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000057, -1.5246593050577406E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 1.5246593050577406E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.10000000000000053, -1.5246593050577406E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.4568966692773966E-19,
    1.4907779871675686E-19, 1.5246593050577406E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19,
    0.10000000000000052, -1.4907779871675686E-19, 0.10000000000000053,
    -1.5246593050577406E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19,
    1.3552527156068805E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.3891340334970526E-19, 1.4568966692773966E-19, 1.4907779871675686E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19,
    0.10000000000000049, -1.3891340334970526E-19, 0.10000000000000051,
    -1.4568966692773966E-19, 0.10000000000000052, -1.4907779871675686E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19,
    1.4568966692773966E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.10000000000000045,
    -1.3552527156068805E-19, 0.10000000000000046, -1.3891340334970526E-19,
    0.10000000000000048, -1.3891340334970526E-19, 0.10000000000000049,
    -1.3891340334970526E-19, 0.10000000000000051, -1.4568966692773966E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 1.3891340334970526E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19,
    0.10000000000000046, -1.3891340334970526E-19, 0.10000000000000048,
    -1.3891340334970526E-19, 0.10000000000000049, -1.3891340334970526E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 1.3891340334970526E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.10000000000000048, -1.3891340334970526E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 1.2874900798265365E-19, 1.3552527156068805E-19,
    1.3891340334970526E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.10000000000000044,
    -1.2536087619363645E-19, 0.10000000000000045, -1.2874900798265365E-19,
    0.10000000000000045, -1.3552527156068805E-19, 0.10000000000000046,
    -1.3891340334970526E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19,
    1.1858461261560205E-19, 1.2874900798265365E-19, 1.2536087619363645E-19,
    1.2874900798265365E-19, 1.3552527156068805E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.10000000000000045,
    -1.2874900798265365E-19, 0.10000000000000045, -1.3552527156068805E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 1.1858461261560205E-19, 1.1858461261560205E-19,
    1.2874900798265365E-19, 1.2536087619363645E-19, 1.2874900798265365E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.10000000000000035,
    -1.1519648082658485E-19, 0.10000000000000038, -1.1858461261560205E-19,
    0.10000000000000039, -1.1858461261560205E-19, 0.10000000000000041,
    -1.2874900798265365E-19, 0.10000000000000044, -1.2536087619363645E-19,
    0.10000000000000045, -1.2874900798265365E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19,
    1.2536087619363645E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.10000000000000039,
    -1.1858461261560205E-19, 0.10000000000000041, -1.2874900798265365E-19,
    0.10000000000000044, -1.2536087619363645E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 1.2874900798265365E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19,
    0.10000000000000041, -1.2874900798265365E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.1180834903756764E-19, 1.1519648082658485E-19, 1.1519648082658485E-19,
    1.1858461261560205E-19, 1.1858461261560205E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.10000000000000038,
    -1.1858461261560205E-19, 0.10000000000000039, -1.1858461261560205E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 1.1180834903756764E-19,
    1.1519648082658485E-19, 1.1519648082658485E-19, 1.1858461261560205E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.10000000000000037,
    -1.1519648082658485E-19, 0.10000000000000035, -1.1519648082658485E-19,
    0.10000000000000038, -1.1858461261560205E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19,
    1.1519648082658485E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000032, -1.0503208545953324E-19, 0.10000000000000034,
    -1.1180834903756764E-19, 0.10000000000000037, -1.1519648082658485E-19,
    0.10000000000000035, -1.1519648082658485E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 1.1519648082658485E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.10000000000000034, -1.1180834903756764E-19,
    0.10000000000000037, -1.1519648082658485E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.1180834903756764E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000032, -1.0503208545953324E-19,
    0.10000000000000034, -1.1180834903756764E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20,
    9.8255821881498839E-20, 1.0164395367051604E-19, 1.0503208545953324E-19,
    1.0503208545953324E-19, 1.0503208545953324E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19,
    0.10000000000000031, -1.0503208545953324E-19, 0.10000000000000032,
    -1.0503208545953324E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20,
    1.0164395367051604E-19, 1.0503208545953324E-19, 1.0503208545953324E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.10000000000000027,
    -9.4867690092481638E-20, 0.10000000000000028, -9.8255821881498839E-20,
    0.10000000000000028, -1.0164395367051604E-19, 0.10000000000000031,
    -1.0503208545953324E-19, 0.10000000000000031, -1.0503208545953324E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19,
    1.0503208545953324E-19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20,
    0.10000000000000028, -9.8255821881498839E-20, 0.10000000000000028,
    -1.0164395367051604E-19, 0.10000000000000031, -1.0503208545953324E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 8.4703294725430034E-20,
    9.4867690092481638E-20, 9.8255821881498839E-20, 1.0164395367051604E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.10000000000000028, -1.0164395367051604E-19, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 9.4867690092481638E-20, 9.8255821881498839E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000027, -8.4703294725430034E-20,
    0.10000000000000027, -9.4867690092481638E-20, 0.10000000000000028,
    -9.8255821881498839E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 8.1315162936412833E-20,
    8.1315162936412833E-20, 8.4703294725430034E-20, 9.4867690092481638E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000026,
    -7.453889935837843E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000027,
    -8.4703294725430034E-20, 0.10000000000000027, -9.4867690092481638E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 8.1315162936412833E-20, 8.1315162936412833E-20,
    8.4703294725430034E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000026, -7.453889935837843E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.10000000000000026, -8.1315162936412833E-20,
    0.10000000000000027, -8.4703294725430034E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 8.1315162936412833E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.10000000000000026,
    -8.1315162936412833E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    7.453889935837843E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    8.1315162936412833E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20,
    0.10000000000000026, -8.1315162936412833E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.1000000000000002,
    -6.7762635780344027E-20, 0.10000000000000021, -6.7762635780344027E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.10000000000000026, -7.453889935837843E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 7.453889935837843E-20,
    7.453889935837843E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.10000000000000019, -6.7762635780344027E-20,
    0.1000000000000002, -6.7762635780344027E-20, 0.10000000000000021,
    -6.7762635780344027E-20, 0.10000000000000023, -7.453889935837843E-20,
    0.10000000000000023, -7.453889935837843E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 7.453889935837843E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.10000000000000023,
    -7.453889935837843E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20,
    5.7598240413292423E-20, 6.0986372202309624E-20, 6.4374503991326826E-20,
    6.7762635780344027E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20,
    0.10000000000000021, -6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -1.6940658945086007E-20,
    0.10000000000000003, -2.0328790734103208E-20, 0.10000000000000005,
    -2.3716922523120409E-20, 0.10000000000000006, -3.0493186101154812E-20,
    0.10000000000000007, -3.0493186101154812E-20, 0.10000000000000009,
    -3.3881317890172014E-20, 0.10000000000000009, -4.0657581468206416E-20,
    0.1000000000000001, -4.4045713257223618E-20, 0.10000000000000012,
    -5.082197683525802E-20, 0.10000000000000014, -5.082197683525802E-20,
    0.10000000000000014, -5.7598240413292423E-20, 0.10000000000000016,
    -6.0986372202309624E-20, 0.10000000000000016, -6.4374503991326826E-20,
    0.10000000000000017, -6.7762635780344027E-20, 0.10000000000000019,
    -6.7762635780344027E-20, 0.1000000000000002, -6.7762635780344027E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20,
    6.0986372202309624E-20, 6.4374503991326826E-20, 6.7762635780344027E-20,
    6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.10000000000000017, -6.7762635780344027E-20,
    0.10000000000000019, -6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20,
    0.10000000000000014, -5.082197683525802E-20, 0.10000000000000014,
    -5.7598240413292423E-20, 0.10000000000000016, -6.0986372202309624E-20,
    0.10000000000000016, -6.4374503991326826E-20, 0.10000000000000017,
    -6.7762635780344027E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20,
    6.4374503991326826E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.10000000000000016,
    -6.4374503991326826E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20,
    5.082197683525802E-20, 5.7598240413292423E-20, 6.0986372202309624E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20,
    0.10000000000000016, -6.0986372202309624E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 4.0657581468206416E-20, 4.4045713257223618E-20,
    5.082197683525802E-20, 5.082197683525802E-20, 5.7598240413292423E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.10000000000000014, -5.7598240413292423E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 3.3881317890172014E-20, 4.0657581468206416E-20,
    4.4045713257223618E-20, 5.082197683525802E-20, 5.082197683525802E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20,
    0.10000000000000012, -5.082197683525802E-20, 0.10000000000000014,
    -5.082197683525802E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 5.082197683525802E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.10000000000000007,
    -3.0493186101154812E-20, 0.10000000000000009, -3.3881317890172014E-20,
    0.10000000000000009, -4.0657581468206416E-20, 0.1000000000000001,
    -4.4045713257223618E-20, 0.10000000000000012, -5.082197683525802E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 4.4045713257223618E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.1000000000000001, -4.4045713257223618E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 3.0493186101154812E-20, 3.3881317890172014E-20,
    4.0657581468206416E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.10000000000000009,
    -4.0657581468206416E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20,
    2.3716922523120409E-20, 3.0493186101154812E-20, 3.0493186101154812E-20,
    3.3881317890172014E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20,
    0.10000000000000009, -3.3881317890172014E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20,
    2.0328790734103208E-20, 2.3716922523120409E-20, 3.0493186101154812E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.10000000000000006,
    -3.0493186101154812E-20, 0.10000000000000007, -3.0493186101154812E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20,
    3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, -1.6940658945086007E-20, 0.10000000000000003,
    -2.0328790734103208E-20, 0.10000000000000005, -2.3716922523120409E-20,
    0.10000000000000006, -3.0493186101154812E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 2.3716922523120409E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20,
    0.10000000000000005, -2.3716922523120409E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.6940658945086007E-20, 2.0328790734103208E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.10000000000000003, -2.0328790734103208E-20, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    -1.6940658945086007E-20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double b_Mx[1280]{ -0.00863017990050439, -0.0070657936891384094,
    -0.0057849825882019291, -0.0047363431509815663, -0.0038777897948388666,
    -0.0031748657590065254, -0.0025993602337928984, -0.0021281761617342057,
    -0.0017424032715792424, -0.0014265591427055997, -0.0011679678412176229,
    -0.00095625119021095623, -0.00078291225709311966, -0.0006409943418438194,
    -0.00052480178021650249, -0.00042967135673330952, -0.00035178515347428761,
    -0.00028801732362564387, -0.00023580864027151514, -0.00019306378563177943,
    -0.00015806725860237973, -0.00012941452567248539, -0.00010595565206305069,
    -8.6749150806437E-5, -7.1024197568616486E-5, -5.8149694762099646E-5,
    -4.7608943383815519E-5, -3.8978906069865185E-5, -3.1913229120723544E-5,
    -2.6128342111147064E-5, -2.139207721332548E-5, -1.75143514867552E-5,
    -1.4339538182410454E-5, -1.1740220894862282E-5, -9.61207989453935E-6,
    -7.869705410688826E-6, -6.4431698373820264E-6, -5.2752212931560192E-6,
    -4.318985901985528E-6, -3.5360865800526926E-6, -2.8951028286223813E-6,
    -2.3703097191030926E-6, -1.9406454613362295E-6, -1.5888661200040732E-6,
    -1.3008535549579206E-6, -1.0650488106817626E-6, -8.7198821482118308E-7,
    -7.1392356778256686E-7, -5.8451118027763514E-7, -4.7855727879810281E-7,
    -3.91809561248215E-7, -3.2078653713079934E-7, -2.6263780310927196E-7,
    -2.150296463132956E-7, -1.7605138424707123E-7, -1.4413868239192045E-7,
    -1.1801077196929976E-7, -9.6619048192633492E-8, -7.9104986075327376E-8,
    -6.4765684808570774E-8, -5.302565788382712E-8, -4.3413736798575982E-8,
    -3.5544161409922222E-8, -2.9101098025565759E-8, -2.3825963888762438E-8,
    -1.9507049344350137E-8, -1.5971021186924026E-8, -1.3075966190689629E-8,
    -1.0705695633420471E-8, -8.7650822350688936E-9, -7.1762423660033487E-9,
    -5.875410303482981E-9, -4.8103790893077127E-9, -3.9384052812744478E-9,
    -3.2244935087591609E-9, -2.639991985616507E-9, -2.1614426133987385E-9,
    -1.7696395254979263E-9, -1.4488582882826506E-9, -1.1862248243641355E-9,
    -9.7119873066668032E-10, -7.9515025504237779E-10, -6.510139540162967E-10,
    -5.3300513173137176E-10, -4.3638767979223992E-10, -3.5728400060566068E-10,
    -2.9251938577394636E-10, -2.3949460390002609E-10, -1.9608158430462187E-10,
    -1.6053801007793403E-10, -1.3143739278430511E-10, -1.0761182247244124E-10,
    -8.81050953485708E-11, -7.21343379603874E-11, -5.9058587736798819E-11,
    -4.8353068909185728E-11, -3.9588131417380192E-11, -3.24120075440151E-11,
    -2.6536694241029998E-11, -2.1726394555977569E-11, -0.00095537281669506727,
    0.00863017990050439, -0.0017375659223780513, 0.0070657936891384094,
    -0.0023779714728462851, 0.0057849825882019291, -0.0029022911914564609,
    0.0047363431509815663, -0.0033315678695278045, 0.0038777897948388666,
    -0.0036830298874439686, 0.0031748657590065254, -0.0039707826500507753,
    0.0025993602337928984, -0.0042063746860801152, 0.0021281761617342057,
    -0.0043992611311575907, 0.0017424032715792424, -0.0045571831955944055,
    0.0014265591427055997, -0.0046864788463383874, 0.0011679678412176229,
    -0.0047923371718417143, 0.00095625119021095623, -0.0048790066384006257,
    0.00078291225709311966, -0.00494996559602527, 0.0006409943418438194,
    -0.0050080618768389221, 0.00052480178021650249, -0.0050556270885805128,
    0.00042967135673330952, -0.0050945701902100167, 0.00035178515347428761,
    -0.0051264541051343326, 0.00028801732362564387, -0.00515255844681139,
    0.00023580864027151514, -0.00517393087413125, 0.00019306378563177943,
    -0.0051914291376459441, 0.00015806725860237973, -0.0052057555041108844,
    0.00012941452567248539, -0.0052174849409155942, 0.00010595565206305069,
    -0.005227088191543894, 8.6749150806437E-5, -0.005234950668162798,
    7.1024197568616486E-5, -0.0052413879195660493, 5.8149694762099646E-5,
    -0.0052466582952551844, 4.7608943383815519E-5, -0.0052509733139121523,
    3.8978906069865185E-5, -0.0052545061523867169, 3.1913229120723544E-5,
    -0.0052573985958914984, 2.6128342111147064E-5, -0.0052597667283404027,
    2.139207721332548E-5, -0.0052617055912036817, 1.75143514867552E-5,
    -0.0052632929978558467, 1.4339538182410454E-5, -0.0052645926564996144,
    1.1740220894862282E-5, -0.00526565672699977, 9.61207989453935E-6,
    -0.0052665279142416884, 7.869705410688826E-6, -0.0052672411820283357,
    6.4431698373820264E-6, -0.0052678251563004425, 5.2752212931560192E-6,
    -0.0052683032739960205, 4.318985901985528E-6, -0.0052686947236569807,
    3.5360865800526926E-6, -0.0052690152155326891, 2.8951028286223813E-6,
    -0.0052692776120874425, 2.3703097191030926E-6, -0.00526949244421632,
    1.9406454613362295E-6, -0.0052696683338869788, 1.5888661200040732E-6,
    -0.0052698123401694955, 1.3008535549579206E-6, -0.0052699302425416273,
    1.0650488106817626E-6, -0.0052700267728395516, 8.7198821482118308E-7,
    -0.0052701058051630644, 7.1392356778256686E-7, -0.0052701705113568096,
    5.8451118027763514E-7, -0.0052702234883075429, 4.7855727879810281E-7,
    -0.0052702668621663123, 3.91809561248215E-7, -0.0052703023736783644,
    3.2078653713079934E-7, -0.0052703314480453678, 2.6263780310927196E-7,
    -0.0052703552521237594, 2.150296463132956E-7, -0.0052703747412547863,
    1.7605138424707123E-7, -0.0052703906976057065, 1.4413868239192045E-7,
    -0.0052704037615609114, 1.1801077196929976E-7, -0.0052704144574227925,
    9.6619048192633492E-8, -0.0052704232144538442, 7.9104986075327376E-8,
    -0.0052704303841044709, 6.4765684808570774E-8, -0.0052704362541179262,
    5.302565788382712E-8, -0.005270441060078462, 4.3413736798575982E-8,
    -0.00527044499486615, 3.5544161409922222E-8, -0.0052704482163978359,
    2.9101098025565759E-8, -0.0052704508539648982, 2.3825963888762438E-8,
    -0.005270453013422164, 1.9507049344350137E-8, -0.005270454781436236,
    1.5971021186924026E-8, -0.0052704562289637271, 1.3075966190689629E-8,
    -0.0052704574140989991, 1.0705695633420471E-8, -0.0052704583844056917,
    8.7650822350688936E-9, -0.00527045917882562, 7.1762423660033487E-9,
    -0.005270459829241644, 5.875410303482981E-9, -0.0052704603617572442,
    4.8103790893077127E-9, -0.0052704607977441418, 3.9384052812744478E-9,
    -0.0052704611547000216, 3.2244935087591609E-9, -0.0052704614469507765,
    2.639991985616507E-9, -0.0052704616862254564, 2.1614426133987385E-9,
    -0.0052704618821269936, 1.7696395254979263E-9, -0.005270462042517605,
    1.4488582882826506E-9, -0.0052704621738343307, 1.1862248243641355E-9,
    -0.0052704622813473717, 9.7119873066668032E-10, -0.0052704623693716028,
    7.9515025504237779E-10, -0.0052704624414397468, 6.510139540162967E-10,
    -0.0052704625004441506, 5.3300513173137176E-10, -0.00527046254875287,
    4.3638767979223992E-10, -0.0052704625883047028, 3.5728400060566068E-10,
    -0.0052704626206870036, 2.9251938577394636E-10, -0.005270462647199387,
    2.3949460390002609E-10, -0.0052704626689058908, 1.9608158430462187E-10,
    -0.0052704626866776714, 1.6053801007793403E-10, -0.0052704627012279742,
    1.3143739278430511E-10, -0.0052704627131407522, 1.0761182247244124E-10,
    -0.0052704627228941092, 8.81050953485708E-11, -0.005270462730879482,
    7.21343379603874E-11, -0.00527046273741735, 5.9058587736798819E-11,
    -0.0052704627427701033, 4.8353068909185728E-11, -0.0052704627471525647,
    3.9588131417380192E-11, -0.005270462750740619, 3.24120075440151E-11,
    -0.0052704627536782692, 2.6536694241029998E-11, -0.005270462756083413,
    2.1726394555977569E-11, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.022037222169275211,
    0.022819415214078254, 0.023459820714702206, 0.02398414039250334,
    0.024413417037163068, 0.024764879027724111, 0.025052631767934441,
    0.025288223785627097, 0.025481110215691768, 0.025639032267837137,
    0.025768327908517737, 0.025874186225781866, 0.025960855685595096,
    0.02603181463769684, 0.026089910913988726, 0.02613747612202821,
    0.026176419220626686, 0.026208303133069408, 0.026234407472714709,
    0.02625577989837111, 0.02627327816052388, 0.026287604525873771,
    0.026299333961765554, 0.026308937211646413, 0.02631679968765337,
    0.0263232369385556, 0.026328507313834539, 0.026332822332155666,
    0.026336355170355263, 0.026339247613634927, 0.026341615745899521,
    0.026343554608611902, 0.026345142015140523, 0.026346441673683139,
    0.026347505744100483, 0.026348376931274602, 0.026349090199005738,
    0.026349674173232397, 0.026350152290890771, 0.02635054374052127,
    0.026350864232372036, 0.026351126628906374, 0.026351341461018536,
    0.026351517350675511, 0.026351661356946821, 0.026351779259309784,
    0.0263518757896002, 0.026351954821917569, 0.026352019528106282,
    0.0263520725050529, 0.0263521158789083, 0.026352151390417595,
    0.026352180464782344, 0.026352204268858891, 0.026352223757988406,
    0.026352239714338092, 0.026352252778292288, 0.026352263474153346,
    0.026352272231183724, 0.026352279400833796, 0.026352285270846803,
    0.026352290076806971, 0.026352294011594359, 0.0263522972331258,
    0.02635229987069266, 0.026352302030149762, 0.026352303798163702,
    0.026352305245691088, 0.026352306430826276, 0.026352307401132896,
    0.02635230819555277, 0.026352308845968748, 0.026352309378484314,
    0.026352309814471186, 0.02635231017142704, 0.026352310463677777,
    0.026352310702952443, 0.026352310898853971, 0.026352311059244574,
    0.026352311190561296, 0.026352311298074336, 0.026352311386098566,
    0.026352311458166709, 0.026352311517171111, 0.026352311565479833,
    0.02635231160503167, 0.026352311637413976, 0.026352311663926362,
    0.026352311685632866, 0.02635231170340465, 0.026352311717954959,
    0.026352311729867742, 0.026352311739621107, 0.026352311747606483,
    0.026352311754144358, 0.026352311759497118, 0.026352311763879588,
    0.026352311767467648, 0.026352311770405302, 0.026352311772810451,
    0.0021389458390009097, -0.022037222169275211, 0.0043830804949667179,
    -0.022819415214078254, 0.00669810892302207, -0.023459820714702206,
    0.0090711802624888326, -0.02398414039250334, 0.0114917731185263,
    -0.024413417037163068, 0.013951273301613108, -0.024764879027724111,
    0.016442628109875273, -0.025052631767934441, 0.018960063279396273,
    -0.025288223785627097, 0.021498851242731268, -0.025481110215691768,
    0.024055121395025914, -0.025639032267837137, 0.026625704753052946,
    -0.025768327908517737, 0.029208006772788217, -0.025874186225781866,
    0.031799903221248933, -0.025960855685595096, 0.034399654923565393,
    -0.02603181463769684, 0.037005837963786779, -0.026089910913988726,
    0.039617286538134372, -0.02613747612202821, 0.042233046167202463,
    -0.026176419220626686, 0.044852335389348436, -0.026208303133069408,
    0.047474514397893124, -0.026234407472714709, 0.050099059363432258,
    -0.02625577989837111, 0.052725541410723206, -0.02627327816052388,
    0.055353609406415594, -0.026287604525873771, 0.057982975866837039,
    -0.026299333961765554, 0.060613405420263951, -0.026308937211646413,
    0.0632447053606278, -0.02631679968765337, 0.06587671791354402,
    -0.0263232369385556, 0.0685093139042719, -0.026328507313834539,
    0.071142387573478669, -0.026332822332155666, 0.0737758523327462,
    -0.026336355170355263, 0.0764096372894737, -0.026339247613634927,
    0.079043684401708739, -0.026341615745899521, 0.0816779461487199,
    -0.026343554608611902, 0.084312383623822929, -0.026345142015140523,
    0.086946964972918941, -0.026346441673683139, 0.089581664116077611,
    -0.026347505744100483, 0.092216459700857886, -0.026348376931274602,
    0.094851334245359659, -0.026349090199005738, 0.097486273436613674,
    -0.026349674173232397, 0.10012126555615182, -0.026350152290890771,
    0.10275630100970391, -0.02635054374052127, 0.10539137194214587,
    -0.026350864232372036, 0.10802647192224604, -0.026351126628906374,
    0.1106615956845573, -0.026351341461018536, 0.11329673891809615,
    -0.026351517350675511, 0.11593189809332782, -0.026351661356946821,
    0.11856707032051368, -0.026351779259309784, 0.1212022532337358,
    -0.0263518757896002, 0.12383744489594445, -0.026351954821917569,
    0.12647264372121744, -0.026352019528106282, 0.12910784841111148,
    -0.0263520725050529, 0.13174305790255111, -0.0263521158789083,
    0.13437827132516383, -0.026352151390417595, 0.13701348796634882,
    -0.026352180464782344, 0.13964870724267792, -0.026352204268858891,
    0.14228392867648049, -0.026352223757988406, 0.144919151876673,
    -0.026352239714338092, 0.14755437652306327, -0.026352252778292288,
    0.15018960235350007, -0.026352263474153346, 0.15282482915335224,
    -0.026352272231183724, 0.15546005674689456, -0.026352279400833796,
    0.15809528499025541, -0.026352285270846803, 0.16073051376564265,
    -0.026352290076806971, 0.1633657429766163, -0.026352294011594359,
    0.16600097254421792, -0.0263522972331258, 0.16863620240380184,
    -0.02635229987069266, 0.17127143250244065, -0.026352302030149762,
    0.173906662796801, -0.026352303798163702, 0.17654189325140465,
    -0.026352305245691088, 0.17917712383720441, -0.026352306430826276,
    0.18181235453041844, -0.026352307401132896, 0.18444758531157585,
    -0.02635230819555277, 0.18708281616473521, -0.026352308845968748,
    0.18971804707684478, -0.026352309378484314, 0.19235327803721869,
    -0.026352309814471186, 0.1949885090371081, -0.02635231017142704,
    0.19762374006935007, -0.026352310463677777, 0.20025897112808011,
    -0.026352310702952443, 0.20289420220849669, -0.026352310898853971,
    0.20552943330666873, -0.026352311059244574, 0.2081646644193777,
    -0.026352311190561296, 0.21079989554398851, -0.026352311298074336,
    0.21343512667834375, -0.026352311386098566, 0.21607035782067702,
    -0.026352311458166709, 0.21870558896954218, -0.026352311517171111,
    0.22134082012375519, -0.026352311565479833, 0.22397605128234666,
    -0.02635231160503167, 0.22661128244452283, -0.026352311637413976,
    0.229246513609634, -0.026352311663926362, 0.2318817447771481,
    -0.026352311685632866, 0.23451697594662957, -0.02635231170340465,
    0.23715220711772178, -0.026352311717954959, 0.23978743829013274,
    -0.026352311729867742, 0.2424226694636234, -0.026352311739621107,
    0.24505790063799807, -0.026352311747606483, 0.24769313181309646,
    -0.026352311754144358, 0.25032836298878747, -0.026352311759497118,
    0.25296359416496361, -0.026352311763879588, 0.255598825341537,
    -0.026352311767467648, 0.25823405651843551, -0.026352311770405302,
    0.26086928769560025, -0.026352311772810451, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    8.6946477628102624E-6, 9.0032571217776809E-6, 9.2559250946519887E-6,
    9.4627921343620582E-6, 9.6321605415708974E-6, 9.770827665152607E-6,
    9.8843587036698188E-6, 9.97731005633274E-6, 1.0053412187298072E-5,
    1.0115719342294158E-5, 1.016673212622625E-5, 1.0208497861231577E-5,
    1.0242692752905343E-5, 1.0270689162316825E-5, 1.0293610683677768E-5,
    1.0312377238123306E-5, 1.0327741993377181E-5, 1.0340321591017043E-5,
    1.0350620894466147E-5, 1.0359053250935212E-5, 1.036595708049735E-5,
    1.0371609458073883E-5, 1.03762372334238E-5, 1.0380026135421112E-5,
    1.038312822600671E-5, 1.0385668002967973E-5, 1.0387747396472118E-5,
    1.0389449859881714E-5, 1.0390843719031139E-5, 1.0391984914382232E-5,
    1.0392919246111442E-5, 1.0393684212231725E-5, 1.0394310513519461E-5,
    1.0394823285644425E-5, 1.0395243107952453E-5, 1.0395586829386865E-5,
    1.039586824469571E-5, 1.0396098648063449E-5, 1.0396287286386229E-5,
    1.0396441730382299E-5, 1.0396568178431509E-5, 1.0396671705338064E-5,
    1.0396756466000232E-5, 1.0396825862161E-5, 1.0396882678931965E-5,
    1.0396929196569648E-5, 1.0396967281990177E-5, 1.0396998463695209E-5,
    1.0397023993116053E-5, 1.0397044894838005E-5, 1.039706200772056E-5,
    1.0397076018563783E-5, 1.0397087489672005E-5, 1.039709688142108E-5,
    1.0397104570734872E-5, 1.0397110866212544E-5, 1.0397116020513721E-5,
    1.0397120240498604E-5, 1.0397123695530005E-5, 1.0397126524270467E-5,
    1.0397128840247276E-5, 1.0397130736408713E-5, 1.0397132288854393E-5,
    1.0397133559889414E-5, 1.0397134600524874E-5, 1.0397135452525129E-5,
    1.0397136150083939E-5, 1.0397136721196788E-5, 1.0397137188784442E-5,
    1.0397137571612834E-5, 1.0397137885046212E-5, 1.0397138141663758E-5,
    1.0397138351764434E-5, 1.0397138523780321E-5, 1.0397138664615015E-5,
    1.0397138779920712E-5, 1.0397138874325033E-5, 1.0397138951616754E-5,
    1.0397139014897862E-5, 1.0397139066708052E-5, 1.0397139109126649E-5,
    1.0397139143856058E-5, 1.0397139172290094E-5, 1.0397139195569913E-5,
    1.0397139214629817E-5, 1.0397139230234745E-5, 1.0397139243010982E-5,
    1.039713925347128E-5, 1.0397139262035447E-5, 1.0397139269047196E-5,
    1.0397139274787929E-5, 1.0397139279488046E-5, 1.0397139283336176E-5,
    1.0397139286486759E-5, 1.0397139289066237E-5, 1.0397139291178136E-5,
    1.0397139292907211E-5, 1.0397139294322859E-5, 1.0397139295481895E-5,
    1.0397139296430832E-5, 0.047141299655819154, -8.6946477628102624E-6,
    0.047142185065069751, -9.0032571217776809E-6, 0.047143098445013389,
    -9.2559250946519887E-6, 0.047144034725423611, -9.4627921343620582E-6,
    0.047144989755150087, -9.6321605415708974E-6, 0.047145960135518368,
    -9.770827665152607E-6, 0.047146943083929187, -9.8843587036698188E-6,
    0.047147936322182935, -9.97731005633274E-6, 0.047148937985047532,
    -1.0053412187298072E-5, 0.047149946545400105, -1.0115719342294158E-5,
    0.047150960752938217, -1.016673212622625E-5, 0.047151979584000789,
    -1.0208497861231577E-5, 0.047153002200485031, -1.0242692752905343E-5,
    0.0471540279162104, -1.0270689162316825E-5, 0.0471550561693798,
    -1.0293610683677768E-5, 0.047156086500032654, -1.0312377238123306E-5,
    0.047157118531585107, -1.0327741993377181E-5, 0.047158151955716363,
    -1.0340321591017043E-5, 0.047159186519994713, -1.0350620894466147E-5,
    0.047160222017746556, -1.0359053250935212E-5, 0.04716125827976185,
    -1.036595708049735E-5, 0.047162295167503136, -1.0371609458073883E-5,
    0.047163332567545534, -1.03762372334238E-5, 0.047164370387024611,
    -1.0380026135421112E-5, 0.047165408549909395, -1.038312822600671E-5,
    0.047166446993950992, -1.0385668002967973E-5, 0.047167485668184315,
    -1.0387747396472118E-5, 0.047168524530882688, -1.0389449859881714E-5,
    0.047169563547883195, -1.0390843719031139E-5, 0.0471706026912156,
    -1.0391984914382232E-5, 0.047171641937979805, -1.0392919246111442E-5,
    0.047172681269426822, -1.0393684212231725E-5, 0.047173720670206261,
    -1.0394310513519461E-5, 0.047174760127750279, -1.0394823285644425E-5,
    0.047175799631769204, -1.0395243107952453E-5, 0.047176839173838567,
    -1.0395586829386865E-5, 0.047177878747060989, -1.039586824469571E-5,
    0.047178918345789383, -1.0396098648063449E-5, 0.047179957965400296,
    -1.0396287286386229E-5, 0.047180997602108379, -1.0396441730382299E-5,
    0.047182037252814432, -1.0396568178431509E-5, 0.047183076914981054,
    -1.0396671705338064E-5, 0.0471841165865308, -1.0396756466000232E-5,
    0.0471851562657628, -1.0396825862161E-5, 0.047186195951284496,
    -1.0396882678931965E-5, 0.047187235641955753, -1.0396929196569648E-5,
    0.047188275336843125, -1.0396967281990177E-5, 0.047189315035182346,
    -1.0396998463695209E-5, 0.047190354736347716, -1.0397023993116053E-5,
    0.047191394439826936, -1.0397044894838005E-5, 0.047192434145200572,
    -1.039706200772056E-5, 0.047193473852125231, -1.0397076018563783E-5,
    0.047194513560319749, -1.0397087489672005E-5, 0.04719555326955395,
    -1.039709688142108E-5, 0.047196592979639372, -1.0397104570734872E-5,
    0.047197632690421709, -1.0397110866212544E-5, 0.047198672401774638,
    -1.0397116020513721E-5, 0.047199712113594722, -1.0397120240498604E-5,
    0.047200751825797284, -1.0397123695530005E-5, 0.047201791538312991,
    -1.0397126524270467E-5, 0.047202831251085077, -1.0397128840247276E-5,
    0.047203870964067071, -1.0397130736408713E-5, 0.047204910677220928,
    -1.0397132288854393E-5, 0.047205950390515485, -1.0397133559889414E-5,
    0.047206990103925248, -1.0397134600524874E-5, 0.047208029817429317,
    -1.0397135452525129E-5, 0.047209069531010617, -1.0397136150083939E-5,
    0.047210109244655137, -1.0397136721196788E-5, 0.04721114895835142,
    -1.0397137188784442E-5, 0.047212188672090087, -1.0397137571612834E-5,
    0.047213228385863441, -1.0397137885046212E-5, 0.04721426809966521,
    -1.0397138141663758E-5, 0.047215307813490238, -1.0397138351764434E-5,
    0.047216347527334307, -1.0397138523780321E-5, 0.047217387241193967,
    -1.0397138664615015E-5, 0.047218426955066395, -1.0397138779920712E-5,
    0.047219466668949273, -1.0397138874325033E-5, 0.047220506382840706,
    -1.0397138951616754E-5, 0.047221546096739141, -1.0397139014897862E-5,
    0.047222585810643314, -1.0397139066708052E-5, 0.047223625524552178,
    -1.0397139109126649E-5, 0.047224665238464886, -1.0397139143856058E-5,
    0.047225704952380744, -1.0397139172290094E-5, 0.047226744666299184,
    -1.0397139195569913E-5, 0.047227784380219733, -1.0397139214629817E-5,
    0.047228824094142009, -1.0397139230234745E-5, 0.047229863808065695,
    -1.0397139243010982E-5, 0.047230903521990539, -1.039713925347128E-5,
    0.047231943235916334, -1.0397139262035447E-5, 0.0472329829498429,
    -1.0397139269047196E-5, 0.047234022663770109, -1.0397139274787929E-5,
    0.04723506237769784, -1.0397139279488046E-5, 0.047236102091625994,
    -1.0397139283336176E-5, 0.047237141805554494, -1.0397139286486759E-5,
    0.04723818151948328, -1.0397139289066237E-5, 0.047239221233412294,
    -1.0397139291178136E-5, 0.04724026094734151, -1.0397139292907211E-5,
    0.047241300661270878, -1.0397139294322859E-5, 0.047242340375200378,
    -1.0397139295481895E-5, 0.047243380089129983, -1.0397139296430832E-5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double b_Mu1[320]{ -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, -1.8000000005602927,
    -1.8200000004587313, -1.8400000003755799, -1.8600000003075012,
    -1.8800000002517632, -1.9000000002061288, -6.3462346100897924E-5,
    0.0018730753077983561, -0.000483997698217998, 0.0070320046035642634,
    -0.0015594181952985956, 0.014881163609403138, -0.0035335517941387723,
    0.024932896411722812, -0.0066060279414276515, 0.03678794411714504,
    -0.010940289404389559, 0.050119421191221168, -0.016670151802919218,
    0.064659696394161748, -0.023905174100266633, 0.080189651799466785,
    -0.032735055588919919, 0.096529888822160043, -0.043233235838168432,
    0.1135335283236628, -0.055459842081882177, 0.13108031583623506,
    -0.069464102335528036, 0.14907179532894305, -0.085286321089281733,
    0.16742735782143534, -0.10295949686873729, 0.18608100626252388,
    -0.12251064658160472, 0.20497870683678862, -0.14396188980107932,
    0.22407622039783898, -0.16733133650198104, 0.24333732699603511,
    -0.1926338138776324, 0.26273237224473189, -0.21988146140718839,
    0.28223707718561936, -0.24908421805555961, 0.30183156388887633,
    -0.28025022115897208, 0.32149955768205085, -0.31338613300484214,
    0.34122773399031009, -0.34849740821276348, 0.36100518357446676,
    -0.38558851264754374, 0.38082297470490556, -0.42466310265004004,
    0.40067379469991227, -0.46572417177895586, 0.42055165644207992,
    -0.50877417095286281, 0.44045165809426523, -0.55381510681416879,
    0.46036978637165238, -0.60084862226272362, 0.48030275547454182,
    -0.64987606239115858, 0.500247875217671, -0.70089852846817657,
    0.52020294306363413, -0.7539169221363321, 0.54016615572732207,
    -0.80893198159811286, 0.56013603680375956, -0.86594431124259752,
    0.58011137751478936, -0.92495440590171141, 0.60009118819656049,
    -0.98596267070956967, 0.62007465858084287, -1.0489694373619314,
    0.64006112527611836, -1.1139749774283152, 0.66005004514334964,
    -1.1809795132510377, 0.68004097349790371, -1.2499832268685909,
    0.70003354626279612, -1.3209862673214867, 0.72002746535700324,
    -1.3939887566337756, 0.740022486732424, -1.4689907947103005,
    0.760018410579373, -1.5459924633462283, 0.780015073307516,
    -1.6249938295097781, 0.80001234098041518, -1.70599494802989,
    0.82001010394019036, -1.7889958637967032, 0.8400082724065624,
    -1.8739966135631556, 0.8600067728736559, -1.9609972274200078,
    0.88000554515995011, -2.0499977300034904, 0.90000453999298324,
    -2.1409981414840433, 0.92000371703187556, -2.2339984783758262,
    0.94000304324830808, -2.3289987541994894, 0.9600024916009805,
    -2.4259989800248047, 0.98000203995034862, -2.5249991649149348,
    1.0000016701700867, -2.6259993162901702, 1.0200013674196144,
    -2.7289994402257305, 1.0400011195484922, -2.8339995416955852,
    1.0600009166087816, -2.9409996247720755, 1.0800007504557996,
    -3.0499996927893527, 1.1000006144212435, -3.160999748477189,
    1.1200005030455691, -3.2739997940705332, 1.1400004118588793,
    -3.388999831399206, 1.1600003372015322, -3.5059998619613384,
    1.1800002760772661, -3.6249998869834958, 1.2000002260329496,
    -3.7459999074699053, 1.2200001850601288, -3.8689999242427588,
    1.2400001515144203, -3.9939999379752096, 1.2600001240495173,
    -4.1209999492183895, 1.2800001015631566, -4.2499999584235262,
    1.3000000831528815, -4.3809999659600543, 1.3200000680798232,
    -4.513999972130442, 1.3400000557390468, -4.6489999771823278,
    1.3600000456352737, -4.7859999813184615, 1.3800000373630039,
    -4.9249999847048418, 1.4000000305902423, -5.0659999874773751,
    1.4200000250451741, -5.2089999897473334, 1.4400000205052563,
    -5.3539999916058179, 1.4600000167882861, -5.5009999931274161,
    1.4800000137450882, -5.6499999943731947, 1.5000000112535286,
    -5.8009999953931519, 1.5200000092136121, -5.9539999962282222,
    1.5400000075434697, -6.10899999691192, 1.5600000061760728,
    -6.2659999974716838, 1.580000005056543, -6.4249999979299792,
    1.6000000041399496, -6.5859999983051996, 1.6200000033895063,
    -6.7489999986124039, 1.6400000027750954, -6.9139999988639218,
    1.6600000022720582, -7.0809999990698467, 1.6800000018602064,
    -7.2499999992384438, 1.7000000015230106, -7.4209999993764795,
    1.7200000012469381, -7.5939999994894931, 1.7400000010209089,
    -7.7689999995820207, 1.7600000008358521, -7.9459999996577757,
    1.7800000006843402, -8.1249999997198, 1.8000000005602927, -8.30599999977058,
    1.8200000004587313, -8.4889999998121546, 1.8400000003755799,
    -8.6739999998461936, 1.8600000003075012, -8.8609999998740623,
    1.8800000002517632, -9.0499999998968779, 1.9000000002061288, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0 };

  static const int16_t b_Mrows[320]{ 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24,
    26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
    64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100,
    102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 130,
    132, 134, 136, 138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160,
    162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188, 190,
    192, 194, 196, 198, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225,
    226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
    241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255,
    256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270,
    271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285,
    286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300,
    301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315,
    316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330,
    331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345,
    346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360,
    361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375,
    376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390,
    391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405,
    406, 407, 408, 409, 410, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510 };

  static const double b_Linv[121]{ 0.58500230612726134, -10.960282875094192,
    -5.6953926589635868, -3.0218960756836308, -1.2040943872041658,
    0.059697397714376939, 0.882145781058583, 1.3696598112559317,
    1.6237045361659921, 1.7244869482658161, 0.0, 0.0, 11.150259904693019,
    -6.5707760438102722, -4.19484043667472, -2.5143949227504954,
    -1.249094347194238, -0.33418847009729158, 0.28901074051994058,
    0.68963450626409639, 0.93105271165274317, 0.0, 0.0, 0.0, 12.565132356734681,
    -5.3451526801579172, -3.8034807465789133, -2.5405728057861863,
    -1.5381964385637179, -0.78435122192278284, -0.24169918586150174,
    0.1364852525460741, 0.0, 0.0, 0.0, 0.0, 12.941399795614391,
    -5.0668653620508977, -3.8109290808171505, -2.7270498831427945,
    -1.8486382820679275, -1.1694751724688284, -0.65923239070664075, 0.0, 0.0,
    0.0, 0.0, 0.0, 13.009084554407975, -5.0556852573975331, -3.8972064108693862,
    -2.9014115379440355, -2.0923366325867789, -1.4557162348689494, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 13.02104684130245, -5.0443976811406515, -3.9394214585456004,
    -3.0081504688796032, -2.251903263897042, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    13.063870707163995, -4.9586027869148479, -3.9138536618866264,
    -3.045812882115285, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    13.148967572449113, -4.8054740579016837, -3.8344189839345586, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 13.262024995246172, -4.613708101515507, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 13.386008359967882, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00079432823472428142 };

  static const double b_Hinv[121]{ 171.75720973797033, -66.331085684420856,
    -53.571071314887973, -41.20746456782139, -29.285528049783316,
    -17.835964120963602, -6.8747670353622956, 3.59453692527024,
    13.577330739356231, 23.083996706141715, 0.0, -66.331085684420856,
    194.52016261081107, -47.155987812875964, -37.830014885467115,
    -28.729382014483228, -19.888436576958863, -11.333825309351862,
    -3.0838740667047748, 4.850344620968456, 12.463079381754385, 0.0,
    -53.571071314887973, -47.155987812875964, 210.43248613110754,
    -34.382684353743528, -28.058039184422391, -21.812036991777646,
    -15.675246537434697, -9.6752712583898735, -3.8351237596350982,
    1.8269927315940755, 0.0, -41.20746456782139, -37.830014885467115,
    -34.382684353743528, 220.33265781315117, -27.25029418346427,
    -23.58091178515005, -19.874110953730085, -16.160029027690054,
    -12.468103146816766, -8.8244902931607054, 0.0, -29.285528049783316,
    -28.729382014483228, -28.058039184422391, -27.25029418346427,
    224.89962342756476, -25.169177133911788, -23.902714680474613,
    -22.514070852259465, -21.032370933512546, -19.486229689696724, 0.0,
    -17.835964120963602, -19.888436576958863, -21.812036991777646,
    -23.58091178515005, -25.169177133911788, 224.63268779030997,
    -27.732996180177494, -28.708995347089562, -29.504542375271797,
    -30.143995916364762, 0.0, -6.8747670353622956, -11.333825309351862,
    -15.675246537434697, -19.874110953730085, -23.902714680474613,
    -27.732996180177494, 219.84768605141994, -34.713662274484165,
    -37.853133521760611, -40.771276702893076, 0.0, 3.59453692527024,
    -3.0838740667047748, -9.6752712583898735, -16.160029027690054,
    -22.514070852259465, -28.708995347089562, -34.713662274484165,
    210.69069808684213, -46.039427139115446, -51.327564574567553, 0.0,
    13.577330739356231, 4.850344620968456, -3.8351237596350982,
    -12.468103146816766, -21.032370933512546, -29.504542375271797,
    -37.853133521760611, -46.039427139115446, 197.16760942052406,
    -61.75913521733812, 0.0, 23.083996706141715, 12.463079381754385,
    1.8269927315940755, -8.8244902931607054, -19.486229689696724,
    -30.143995916364762, -40.771276702893076, -51.327564574567553,
    -61.75913521733812, 179.18521981313, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 6.3095734448019318E-7 };

  static const double b_Ac[3520]{ -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, -1.8000000005602927,
    -1.8200000004587313, -1.8400000003755799, -1.8600000003075012,
    -1.8800000002517632, -1.9000000002061288, -6.3462346100897924E-5,
    0.0018730753077983561, -0.000483997698217998, 0.0070320046035642634,
    -0.0015594181952985956, 0.014881163609403138, -0.0035335517941387723,
    0.024932896411722812, -0.0066060279414276515, 0.03678794411714504,
    -0.010940289404389559, 0.050119421191221168, -0.016670151802919218,
    0.064659696394161748, -0.023905174100266633, 0.080189651799466785,
    -0.032735055588919919, 0.096529888822160043, -0.043233235838168432,
    0.1135335283236628, -0.055459842081882177, 0.13108031583623506,
    -0.069464102335528036, 0.14907179532894305, -0.085286321089281733,
    0.16742735782143534, -0.10295949686873729, 0.18608100626252388,
    -0.12251064658160472, 0.20497870683678862, -0.14396188980107932,
    0.22407622039783898, -0.16733133650198104, 0.24333732699603511,
    -0.1926338138776324, 0.26273237224473189, -0.21988146140718839,
    0.28223707718561936, -0.24908421805555961, 0.30183156388887633,
    -0.28025022115897208, 0.32149955768205085, -0.31338613300484214,
    0.34122773399031009, -0.34849740821276348, 0.36100518357446676,
    -0.38558851264754374, 0.38082297470490556, -0.42466310265004004,
    0.40067379469991227, -0.46572417177895586, 0.42055165644207992,
    -0.50877417095286281, 0.44045165809426523, -0.55381510681416879,
    0.46036978637165238, -0.60084862226272362, 0.48030275547454182,
    -0.64987606239115858, 0.500247875217671, -0.70089852846817657,
    0.52020294306363413, -0.7539169221363321, 0.54016615572732207,
    -0.80893198159811286, 0.56013603680375956, -0.86594431124259752,
    0.58011137751478936, -0.92495440590171141, 0.60009118819656049,
    -0.98596267070956967, 0.62007465858084287, -1.0489694373619314,
    0.64006112527611836, -1.1139749774283152, 0.66005004514334964,
    -1.1809795132510377, 0.68004097349790371, -1.2499832268685909,
    0.70003354626279612, -1.3209862673214867, 0.72002746535700324,
    -1.3939887566337756, 0.740022486732424, -1.4689907947103005,
    0.760018410579373, -1.5459924633462283, 0.780015073307516,
    -1.6249938295097781, 0.80001234098041518, -1.70599494802989,
    0.82001010394019036, -1.7889958637967032, 0.8400082724065624,
    -1.8739966135631556, 0.8600067728736559, -1.9609972274200078,
    0.88000554515995011, -2.0499977300034904, 0.90000453999298324,
    -2.1409981414840433, 0.92000371703187556, -2.2339984783758262,
    0.94000304324830808, -2.3289987541994894, 0.9600024916009805,
    -2.4259989800248047, 0.98000203995034862, -2.5249991649149348,
    1.0000016701700867, -2.6259993162901702, 1.0200013674196144,
    -2.7289994402257305, 1.0400011195484922, -2.8339995416955852,
    1.0600009166087816, -2.9409996247720755, 1.0800007504557996,
    -3.0499996927893527, 1.1000006144212435, -3.160999748477189,
    1.1200005030455691, -3.2739997940705332, 1.1400004118588793,
    -3.388999831399206, 1.1600003372015322, -3.5059998619613384,
    1.1800002760772661, -3.6249998869834958, 1.2000002260329496,
    -3.7459999074699053, 1.2200001850601288, -3.8689999242427588,
    1.2400001515144203, -3.9939999379752096, 1.2600001240495173,
    -4.1209999492183895, 1.2800001015631566, -4.2499999584235262,
    1.3000000831528815, -4.3809999659600543, 1.3200000680798232,
    -4.513999972130442, 1.3400000557390468, -4.6489999771823278,
    1.3600000456352737, -4.7859999813184615, 1.3800000373630039,
    -4.9249999847048418, 1.4000000305902423, -5.0659999874773751,
    1.4200000250451741, -5.2089999897473334, 1.4400000205052563,
    -5.3539999916058179, 1.4600000167882861, -5.5009999931274161,
    1.4800000137450882, -5.6499999943731947, 1.5000000112535286,
    -5.8009999953931519, 1.5200000092136121, -5.9539999962282222,
    1.5400000075434697, -6.10899999691192, 1.5600000061760728,
    -6.2659999974716838, 1.580000005056543, -6.4249999979299792,
    1.6000000041399496, -6.5859999983051996, 1.6200000033895063,
    -6.7489999986124039, 1.6400000027750954, -6.9139999988639218,
    1.6600000022720582, -7.0809999990698467, 1.6800000018602064,
    -7.2499999992384438, 1.7000000015230106, -7.4209999993764795,
    1.7200000012469381, -7.5939999994894931, 1.7400000010209089,
    -7.7689999995820207, 1.7600000008358521, -7.9459999996577757,
    1.7800000006843402, -8.1249999997198, 1.8000000005602927, -8.30599999977058,
    1.8200000004587313, -8.4889999998121546, 1.8400000003755799,
    -8.6739999998461936, 1.8600000003075012, -8.8609999998740623,
    1.8800000002517632, -9.0499999998968779, 1.9000000002061288, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, -0.0, -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, -1.8000000005602927,
    -1.8200000004587313, -1.8400000003755799, -1.8600000003075012,
    -1.8800000002517632, 0.0, 0.0, -6.3462346100897924E-5, 0.0018730753077983561,
    -0.000483997698217998, 0.0070320046035642634, -0.0015594181952985956,
    0.014881163609403138, -0.0035335517941387723, 0.024932896411722812,
    -0.0066060279414276515, 0.03678794411714504, -0.010940289404389559,
    0.050119421191221168, -0.016670151802919218, 0.064659696394161748,
    -0.023905174100266633, 0.080189651799466785, -0.032735055588919919,
    0.096529888822160043, -0.043233235838168432, 0.1135335283236628,
    -0.055459842081882177, 0.13108031583623506, -0.069464102335528036,
    0.14907179532894305, -0.085286321089281733, 0.16742735782143534,
    -0.10295949686873729, 0.18608100626252388, -0.12251064658160472,
    0.20497870683678862, -0.14396188980107932, 0.22407622039783898,
    -0.16733133650198104, 0.24333732699603511, -0.1926338138776324,
    0.26273237224473189, -0.21988146140718839, 0.28223707718561936,
    -0.24908421805555961, 0.30183156388887633, -0.28025022115897208,
    0.32149955768205085, -0.31338613300484214, 0.34122773399031009,
    -0.34849740821276348, 0.36100518357446676, -0.38558851264754374,
    0.38082297470490556, -0.42466310265004004, 0.40067379469991227,
    -0.46572417177895586, 0.42055165644207992, -0.50877417095286281,
    0.44045165809426523, -0.55381510681416879, 0.46036978637165238,
    -0.60084862226272362, 0.48030275547454182, -0.64987606239115858,
    0.500247875217671, -0.70089852846817657, 0.52020294306363413,
    -0.7539169221363321, 0.54016615572732207, -0.80893198159811286,
    0.56013603680375956, -0.86594431124259752, 0.58011137751478936,
    -0.92495440590171141, 0.60009118819656049, -0.98596267070956967,
    0.62007465858084287, -1.0489694373619314, 0.64006112527611836,
    -1.1139749774283152, 0.66005004514334964, -1.1809795132510377,
    0.68004097349790371, -1.2499832268685909, 0.70003354626279612,
    -1.3209862673214867, 0.72002746535700324, -1.3939887566337756,
    0.740022486732424, -1.4689907947103005, 0.760018410579373,
    -1.5459924633462283, 0.780015073307516, -1.6249938295097781,
    0.80001234098041518, -1.70599494802989, 0.82001010394019036,
    -1.7889958637967032, 0.8400082724065624, -1.8739966135631556,
    0.8600067728736559, -1.9609972274200078, 0.88000554515995011,
    -2.0499977300034904, 0.90000453999298324, -2.1409981414840433,
    0.92000371703187556, -2.2339984783758262, 0.94000304324830808,
    -2.3289987541994894, 0.9600024916009805, -2.4259989800248047,
    0.98000203995034862, -2.5249991649149348, 1.0000016701700867,
    -2.6259993162901702, 1.0200013674196144, -2.7289994402257305,
    1.0400011195484922, -2.8339995416955852, 1.0600009166087816,
    -2.9409996247720755, 1.0800007504557996, -3.0499996927893527,
    1.1000006144212435, -3.160999748477189, 1.1200005030455691,
    -3.2739997940705332, 1.1400004118588793, -3.388999831399206,
    1.1600003372015322, -3.5059998619613384, 1.1800002760772661,
    -3.6249998869834958, 1.2000002260329496, -3.7459999074699053,
    1.2200001850601288, -3.8689999242427588, 1.2400001515144203,
    -3.9939999379752096, 1.2600001240495173, -4.1209999492183895,
    1.2800001015631566, -4.2499999584235262, 1.3000000831528815,
    -4.3809999659600543, 1.3200000680798232, -4.513999972130442,
    1.3400000557390468, -4.6489999771823278, 1.3600000456352737,
    -4.7859999813184615, 1.3800000373630039, -4.9249999847048418,
    1.4000000305902423, -5.0659999874773751, 1.4200000250451741,
    -5.2089999897473334, 1.4400000205052563, -5.3539999916058179,
    1.4600000167882861, -5.5009999931274161, 1.4800000137450882,
    -5.6499999943731947, 1.5000000112535286, -5.8009999953931519,
    1.5200000092136121, -5.9539999962282222, 1.5400000075434697,
    -6.10899999691192, 1.5600000061760728, -6.2659999974716838,
    1.580000005056543, -6.4249999979299792, 1.6000000041399496,
    -6.5859999983051996, 1.6200000033895063, -6.7489999986124039,
    1.6400000027750954, -6.9139999988639218, 1.6600000022720582,
    -7.0809999990698467, 1.6800000018602064, -7.2499999992384438,
    1.7000000015230106, -7.4209999993764795, 1.7200000012469381,
    -7.5939999994894931, 1.7400000010209089, -7.7689999995820207,
    1.7600000008358521, -7.9459999996577757, 1.7800000006843402,
    -8.1249999997198, 1.8000000005602927, -8.30599999977058, 1.8200000004587313,
    -8.4889999998121546, 1.8400000003755799, -8.6739999998461936,
    1.8600000003075012, -8.8609999998740623, 1.8800000002517632, -0.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, -0.0, -0.0, -0.0018730753077983561,
    -0.0070320046035642634, -0.014881163609403138, -0.024932896411722812,
    -0.03678794411714504, -0.050119421191221168, -0.064659696394161748,
    -0.080189651799466785, -0.096529888822160043, -0.1135335283236628,
    -0.13108031583623506, -0.14907179532894305, -0.16742735782143534,
    -0.18608100626252388, -0.20497870683678862, -0.22407622039783898,
    -0.24333732699603511, -0.26273237224473189, -0.28223707718561936,
    -0.30183156388887633, -0.32149955768205085, -0.34122773399031009,
    -0.36100518357446676, -0.38082297470490556, -0.40067379469991227,
    -0.42055165644207992, -0.44045165809426523, -0.46036978637165238,
    -0.48030275547454182, -0.500247875217671, -0.52020294306363413,
    -0.54016615572732207, -0.56013603680375956, -0.58011137751478936,
    -0.60009118819656049, -0.62007465858084287, -0.64006112527611836,
    -0.66005004514334964, -0.68004097349790371, -0.70003354626279612,
    -0.72002746535700324, -0.740022486732424, -0.760018410579373,
    -0.780015073307516, -0.80001234098041518, -0.82001010394019036,
    -0.8400082724065624, -0.8600067728736559, -0.88000554515995011,
    -0.90000453999298324, -0.92000371703187556, -0.94000304324830808,
    -0.9600024916009805, -0.98000203995034862, -1.0000016701700867,
    -1.0200013674196144, -1.0400011195484922, -1.0600009166087816,
    -1.0800007504557996, -1.1000006144212435, -1.1200005030455691,
    -1.1400004118588793, -1.1600003372015322, -1.1800002760772661,
    -1.2000002260329496, -1.2200001850601288, -1.2400001515144203,
    -1.2600001240495173, -1.2800001015631566, -1.3000000831528815,
    -1.3200000680798232, -1.3400000557390468, -1.3600000456352737,
    -1.3800000373630039, -1.4000000305902423, -1.4200000250451741,
    -1.4400000205052563, -1.4600000167882861, -1.4800000137450882,
    -1.5000000112535286, -1.5200000092136121, -1.5400000075434697,
    -1.5600000061760728, -1.580000005056543, -1.6000000041399496,
    -1.6200000033895063, -1.6400000027750954, -1.6600000022720582,
    -1.6800000018602064, -1.7000000015230106, -1.7200000012469381,
    -1.7400000010209089, -1.7600000008358521, -1.7800000006843402,
    -1.8000000005602927, -1.8200000004587313, -1.8400000003755799,
    -1.8600000003075012, 0.0, 0.0, 0.0, 0.0, -6.3462346100897924E-5,
    0.0018730753077983561, -0.000483997698217998, 0.0070320046035642634,
    -0.0015594181952985956, 0.014881163609403138, -0.0035335517941387723,
    0.024932896411722812, -0.0066060279414276515, 0.03678794411714504,
    -0.010940289404389559, 0.050119421191221168, -0.016670151802919218,
    0.064659696394161748, -0.023905174100266633, 0.080189651799466785,
    -0.032735055588919919, 0.096529888822160043, -0.043233235838168432,
    0.1135335283236628, -0.055459842081882177, 0.13108031583623506,
    -0.069464102335528036, 0.14907179532894305, -0.085286321089281733,
    0.16742735782143534, -0.10295949686873729, 0.18608100626252388,
    -0.12251064658160472, 0.20497870683678862, -0.14396188980107932,
    0.22407622039783898, -0.16733133650198104, 0.24333732699603511,
    -0.1926338138776324, 0.26273237224473189, -0.21988146140718839,
    0.28223707718561936, -0.24908421805555961, 0.30183156388887633,
    -0.28025022115897208, 0.32149955768205085, -0.31338613300484214,
    0.34122773399031009, -0.34849740821276348, 0.36100518357446676,
    -0.38558851264754374, 0.38082297470490556, -0.42466310265004004,
    0.40067379469991227, -0.46572417177895586, 0.42055165644207992,
    -0.50877417095286281, 0.44045165809426523, -0.55381510681416879,
    0.46036978637165238, -0.60084862226272362, 0.48030275547454182,
    -0.64987606239115858, 0.500247875217671, -0.70089852846817657,
    0.52020294306363413, -0.7539169221363321, 0.54016615572732207,
    -0.80893198159811286, 0.56013603680375956, -0.86594431124259752,
    0.58011137751478936, -0.92495440590171141, 0.60009118819656049,
    -0.98596267070956967, 0.62007465858084287, -1.0489694373619314,
    0.64006112527611836, -1.1139749774283152, 0.66005004514334964,
    -1.1809795132510377, 0.68004097349790371, -1.2499832268685909,
    0.70003354626279612, -1.3209862673214867, 0.72002746535700324,
    -1.3939887566337756, 0.740022486732424, -1.4689907947103005,
    0.760018410579373, -1.5459924633462283, 0.780015073307516,
    -1.6249938295097781, 0.80001234098041518, -1.70599494802989,
    0.82001010394019036, -1.7889958637967032, 0.8400082724065624,
    -1.8739966135631556, 0.8600067728736559, -1.9609972274200078,
    0.88000554515995011, -2.0499977300034904, 0.90000453999298324,
    -2.1409981414840433, 0.92000371703187556, -2.2339984783758262,
    0.94000304324830808, -2.3289987541994894, 0.9600024916009805,
    -2.4259989800248047, 0.98000203995034862, -2.5249991649149348,
    1.0000016701700867, -2.6259993162901702, 1.0200013674196144,
    -2.7289994402257305, 1.0400011195484922, -2.8339995416955852,
    1.0600009166087816, -2.9409996247720755, 1.0800007504557996,
    -3.0499996927893527, 1.1000006144212435, -3.160999748477189,
    1.1200005030455691, -3.2739997940705332, 1.1400004118588793,
    -3.388999831399206, 1.1600003372015322, -3.5059998619613384,
    1.1800002760772661, -3.6249998869834958, 1.2000002260329496,
    -3.7459999074699053, 1.2200001850601288, -3.8689999242427588,
    1.2400001515144203, -3.9939999379752096, 1.2600001240495173,
    -4.1209999492183895, 1.2800001015631566, -4.2499999584235262,
    1.3000000831528815, -4.3809999659600543, 1.3200000680798232,
    -4.513999972130442, 1.3400000557390468, -4.6489999771823278,
    1.3600000456352737, -4.7859999813184615, 1.3800000373630039,
    -4.9249999847048418, 1.4000000305902423, -5.0659999874773751,
    1.4200000250451741, -5.2089999897473334, 1.4400000205052563,
    -5.3539999916058179, 1.4600000167882861, -5.5009999931274161,
    1.4800000137450882, -5.6499999943731947, 1.5000000112535286,
    -5.8009999953931519, 1.5200000092136121, -5.9539999962282222,
    1.5400000075434697, -6.10899999691192, 1.5600000061760728,
    -6.2659999974716838, 1.580000005056543, -6.4249999979299792,
    1.6000000041399496, -6.5859999983051996, 1.6200000033895063,
    -6.7489999986124039, 1.6400000027750954, -6.9139999988639218,
    1.6600000022720582, -7.0809999990698467, 1.6800000018602064,
    -7.2499999992384438, 1.7000000015230106, -7.4209999993764795,
    1.7200000012469381, -7.5939999994894931, 1.7400000010209089,
    -7.7689999995820207, 1.7600000008358521, -7.9459999996577757,
    1.7800000006843402, -8.1249999997198, 1.8000000005602927, -8.30599999977058,
    1.8200000004587313, -8.4889999998121546, 1.8400000003755799,
    -8.6739999998461936, 1.8600000003075012, -0.0, -0.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    -0.0, -0.0, -0.0, -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, -1.8000000005602927,
    -1.8200000004587313, -1.8400000003755799, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -6.3462346100897924E-5, 0.0018730753077983561, -0.000483997698217998,
    0.0070320046035642634, -0.0015594181952985956, 0.014881163609403138,
    -0.0035335517941387723, 0.024932896411722812, -0.0066060279414276515,
    0.03678794411714504, -0.010940289404389559, 0.050119421191221168,
    -0.016670151802919218, 0.064659696394161748, -0.023905174100266633,
    0.080189651799466785, -0.032735055588919919, 0.096529888822160043,
    -0.043233235838168432, 0.1135335283236628, -0.055459842081882177,
    0.13108031583623506, -0.069464102335528036, 0.14907179532894305,
    -0.085286321089281733, 0.16742735782143534, -0.10295949686873729,
    0.18608100626252388, -0.12251064658160472, 0.20497870683678862,
    -0.14396188980107932, 0.22407622039783898, -0.16733133650198104,
    0.24333732699603511, -0.1926338138776324, 0.26273237224473189,
    -0.21988146140718839, 0.28223707718561936, -0.24908421805555961,
    0.30183156388887633, -0.28025022115897208, 0.32149955768205085,
    -0.31338613300484214, 0.34122773399031009, -0.34849740821276348,
    0.36100518357446676, -0.38558851264754374, 0.38082297470490556,
    -0.42466310265004004, 0.40067379469991227, -0.46572417177895586,
    0.42055165644207992, -0.50877417095286281, 0.44045165809426523,
    -0.55381510681416879, 0.46036978637165238, -0.60084862226272362,
    0.48030275547454182, -0.64987606239115858, 0.500247875217671,
    -0.70089852846817657, 0.52020294306363413, -0.7539169221363321,
    0.54016615572732207, -0.80893198159811286, 0.56013603680375956,
    -0.86594431124259752, 0.58011137751478936, -0.92495440590171141,
    0.60009118819656049, -0.98596267070956967, 0.62007465858084287,
    -1.0489694373619314, 0.64006112527611836, -1.1139749774283152,
    0.66005004514334964, -1.1809795132510377, 0.68004097349790371,
    -1.2499832268685909, 0.70003354626279612, -1.3209862673214867,
    0.72002746535700324, -1.3939887566337756, 0.740022486732424,
    -1.4689907947103005, 0.760018410579373, -1.5459924633462283,
    0.780015073307516, -1.6249938295097781, 0.80001234098041518,
    -1.70599494802989, 0.82001010394019036, -1.7889958637967032,
    0.8400082724065624, -1.8739966135631556, 0.8600067728736559,
    -1.9609972274200078, 0.88000554515995011, -2.0499977300034904,
    0.90000453999298324, -2.1409981414840433, 0.92000371703187556,
    -2.2339984783758262, 0.94000304324830808, -2.3289987541994894,
    0.9600024916009805, -2.4259989800248047, 0.98000203995034862,
    -2.5249991649149348, 1.0000016701700867, -2.6259993162901702,
    1.0200013674196144, -2.7289994402257305, 1.0400011195484922,
    -2.8339995416955852, 1.0600009166087816, -2.9409996247720755,
    1.0800007504557996, -3.0499996927893527, 1.1000006144212435,
    -3.160999748477189, 1.1200005030455691, -3.2739997940705332,
    1.1400004118588793, -3.388999831399206, 1.1600003372015322,
    -3.5059998619613384, 1.1800002760772661, -3.6249998869834958,
    1.2000002260329496, -3.7459999074699053, 1.2200001850601288,
    -3.8689999242427588, 1.2400001515144203, -3.9939999379752096,
    1.2600001240495173, -4.1209999492183895, 1.2800001015631566,
    -4.2499999584235262, 1.3000000831528815, -4.3809999659600543,
    1.3200000680798232, -4.513999972130442, 1.3400000557390468,
    -4.6489999771823278, 1.3600000456352737, -4.7859999813184615,
    1.3800000373630039, -4.9249999847048418, 1.4000000305902423,
    -5.0659999874773751, 1.4200000250451741, -5.2089999897473334,
    1.4400000205052563, -5.3539999916058179, 1.4600000167882861,
    -5.5009999931274161, 1.4800000137450882, -5.6499999943731947,
    1.5000000112535286, -5.8009999953931519, 1.5200000092136121,
    -5.9539999962282222, 1.5400000075434697, -6.10899999691192,
    1.5600000061760728, -6.2659999974716838, 1.580000005056543,
    -6.4249999979299792, 1.6000000041399496, -6.5859999983051996,
    1.6200000033895063, -6.7489999986124039, 1.6400000027750954,
    -6.9139999988639218, 1.6600000022720582, -7.0809999990698467,
    1.6800000018602064, -7.2499999992384438, 1.7000000015230106,
    -7.4209999993764795, 1.7200000012469381, -7.5939999994894931,
    1.7400000010209089, -7.7689999995820207, 1.7600000008358521,
    -7.9459999996577757, 1.7800000006843402, -8.1249999997198,
    1.8000000005602927, -8.30599999977058, 1.8200000004587313,
    -8.4889999998121546, 1.8400000003755799, -0.0, -0.0, -0.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, -1.8000000005602927,
    -1.8200000004587313, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -6.3462346100897924E-5, 0.0018730753077983561, -0.000483997698217998,
    0.0070320046035642634, -0.0015594181952985956, 0.014881163609403138,
    -0.0035335517941387723, 0.024932896411722812, -0.0066060279414276515,
    0.03678794411714504, -0.010940289404389559, 0.050119421191221168,
    -0.016670151802919218, 0.064659696394161748, -0.023905174100266633,
    0.080189651799466785, -0.032735055588919919, 0.096529888822160043,
    -0.043233235838168432, 0.1135335283236628, -0.055459842081882177,
    0.13108031583623506, -0.069464102335528036, 0.14907179532894305,
    -0.085286321089281733, 0.16742735782143534, -0.10295949686873729,
    0.18608100626252388, -0.12251064658160472, 0.20497870683678862,
    -0.14396188980107932, 0.22407622039783898, -0.16733133650198104,
    0.24333732699603511, -0.1926338138776324, 0.26273237224473189,
    -0.21988146140718839, 0.28223707718561936, -0.24908421805555961,
    0.30183156388887633, -0.28025022115897208, 0.32149955768205085,
    -0.31338613300484214, 0.34122773399031009, -0.34849740821276348,
    0.36100518357446676, -0.38558851264754374, 0.38082297470490556,
    -0.42466310265004004, 0.40067379469991227, -0.46572417177895586,
    0.42055165644207992, -0.50877417095286281, 0.44045165809426523,
    -0.55381510681416879, 0.46036978637165238, -0.60084862226272362,
    0.48030275547454182, -0.64987606239115858, 0.500247875217671,
    -0.70089852846817657, 0.52020294306363413, -0.7539169221363321,
    0.54016615572732207, -0.80893198159811286, 0.56013603680375956,
    -0.86594431124259752, 0.58011137751478936, -0.92495440590171141,
    0.60009118819656049, -0.98596267070956967, 0.62007465858084287,
    -1.0489694373619314, 0.64006112527611836, -1.1139749774283152,
    0.66005004514334964, -1.1809795132510377, 0.68004097349790371,
    -1.2499832268685909, 0.70003354626279612, -1.3209862673214867,
    0.72002746535700324, -1.3939887566337756, 0.740022486732424,
    -1.4689907947103005, 0.760018410579373, -1.5459924633462283,
    0.780015073307516, -1.6249938295097781, 0.80001234098041518,
    -1.70599494802989, 0.82001010394019036, -1.7889958637967032,
    0.8400082724065624, -1.8739966135631556, 0.8600067728736559,
    -1.9609972274200078, 0.88000554515995011, -2.0499977300034904,
    0.90000453999298324, -2.1409981414840433, 0.92000371703187556,
    -2.2339984783758262, 0.94000304324830808, -2.3289987541994894,
    0.9600024916009805, -2.4259989800248047, 0.98000203995034862,
    -2.5249991649149348, 1.0000016701700867, -2.6259993162901702,
    1.0200013674196144, -2.7289994402257305, 1.0400011195484922,
    -2.8339995416955852, 1.0600009166087816, -2.9409996247720755,
    1.0800007504557996, -3.0499996927893527, 1.1000006144212435,
    -3.160999748477189, 1.1200005030455691, -3.2739997940705332,
    1.1400004118588793, -3.388999831399206, 1.1600003372015322,
    -3.5059998619613384, 1.1800002760772661, -3.6249998869834958,
    1.2000002260329496, -3.7459999074699053, 1.2200001850601288,
    -3.8689999242427588, 1.2400001515144203, -3.9939999379752096,
    1.2600001240495173, -4.1209999492183895, 1.2800001015631566,
    -4.2499999584235262, 1.3000000831528815, -4.3809999659600543,
    1.3200000680798232, -4.513999972130442, 1.3400000557390468,
    -4.6489999771823278, 1.3600000456352737, -4.7859999813184615,
    1.3800000373630039, -4.9249999847048418, 1.4000000305902423,
    -5.0659999874773751, 1.4200000250451741, -5.2089999897473334,
    1.4400000205052563, -5.3539999916058179, 1.4600000167882861,
    -5.5009999931274161, 1.4800000137450882, -5.6499999943731947,
    1.5000000112535286, -5.8009999953931519, 1.5200000092136121,
    -5.9539999962282222, 1.5400000075434697, -6.10899999691192,
    1.5600000061760728, -6.2659999974716838, 1.580000005056543,
    -6.4249999979299792, 1.6000000041399496, -6.5859999983051996,
    1.6200000033895063, -6.7489999986124039, 1.6400000027750954,
    -6.9139999988639218, 1.6600000022720582, -7.0809999990698467,
    1.6800000018602064, -7.2499999992384438, 1.7000000015230106,
    -7.4209999993764795, 1.7200000012469381, -7.5939999994894931,
    1.7400000010209089, -7.7689999995820207, 1.7600000008358521,
    -7.9459999996577757, 1.7800000006843402, -8.1249999997198,
    1.8000000005602927, -8.30599999977058, 1.8200000004587313, -0.0, -0.0, -0.0,
    -0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0018730753077983561,
    -0.0070320046035642634, -0.014881163609403138, -0.024932896411722812,
    -0.03678794411714504, -0.050119421191221168, -0.064659696394161748,
    -0.080189651799466785, -0.096529888822160043, -0.1135335283236628,
    -0.13108031583623506, -0.14907179532894305, -0.16742735782143534,
    -0.18608100626252388, -0.20497870683678862, -0.22407622039783898,
    -0.24333732699603511, -0.26273237224473189, -0.28223707718561936,
    -0.30183156388887633, -0.32149955768205085, -0.34122773399031009,
    -0.36100518357446676, -0.38082297470490556, -0.40067379469991227,
    -0.42055165644207992, -0.44045165809426523, -0.46036978637165238,
    -0.48030275547454182, -0.500247875217671, -0.52020294306363413,
    -0.54016615572732207, -0.56013603680375956, -0.58011137751478936,
    -0.60009118819656049, -0.62007465858084287, -0.64006112527611836,
    -0.66005004514334964, -0.68004097349790371, -0.70003354626279612,
    -0.72002746535700324, -0.740022486732424, -0.760018410579373,
    -0.780015073307516, -0.80001234098041518, -0.82001010394019036,
    -0.8400082724065624, -0.8600067728736559, -0.88000554515995011,
    -0.90000453999298324, -0.92000371703187556, -0.94000304324830808,
    -0.9600024916009805, -0.98000203995034862, -1.0000016701700867,
    -1.0200013674196144, -1.0400011195484922, -1.0600009166087816,
    -1.0800007504557996, -1.1000006144212435, -1.1200005030455691,
    -1.1400004118588793, -1.1600003372015322, -1.1800002760772661,
    -1.2000002260329496, -1.2200001850601288, -1.2400001515144203,
    -1.2600001240495173, -1.2800001015631566, -1.3000000831528815,
    -1.3200000680798232, -1.3400000557390468, -1.3600000456352737,
    -1.3800000373630039, -1.4000000305902423, -1.4200000250451741,
    -1.4400000205052563, -1.4600000167882861, -1.4800000137450882,
    -1.5000000112535286, -1.5200000092136121, -1.5400000075434697,
    -1.5600000061760728, -1.580000005056543, -1.6000000041399496,
    -1.6200000033895063, -1.6400000027750954, -1.6600000022720582,
    -1.6800000018602064, -1.7000000015230106, -1.7200000012469381,
    -1.7400000010209089, -1.7600000008358521, -1.7800000006843402,
    -1.8000000005602927, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -6.3462346100897924E-5, 0.0018730753077983561, -0.000483997698217998,
    0.0070320046035642634, -0.0015594181952985956, 0.014881163609403138,
    -0.0035335517941387723, 0.024932896411722812, -0.0066060279414276515,
    0.03678794411714504, -0.010940289404389559, 0.050119421191221168,
    -0.016670151802919218, 0.064659696394161748, -0.023905174100266633,
    0.080189651799466785, -0.032735055588919919, 0.096529888822160043,
    -0.043233235838168432, 0.1135335283236628, -0.055459842081882177,
    0.13108031583623506, -0.069464102335528036, 0.14907179532894305,
    -0.085286321089281733, 0.16742735782143534, -0.10295949686873729,
    0.18608100626252388, -0.12251064658160472, 0.20497870683678862,
    -0.14396188980107932, 0.22407622039783898, -0.16733133650198104,
    0.24333732699603511, -0.1926338138776324, 0.26273237224473189,
    -0.21988146140718839, 0.28223707718561936, -0.24908421805555961,
    0.30183156388887633, -0.28025022115897208, 0.32149955768205085,
    -0.31338613300484214, 0.34122773399031009, -0.34849740821276348,
    0.36100518357446676, -0.38558851264754374, 0.38082297470490556,
    -0.42466310265004004, 0.40067379469991227, -0.46572417177895586,
    0.42055165644207992, -0.50877417095286281, 0.44045165809426523,
    -0.55381510681416879, 0.46036978637165238, -0.60084862226272362,
    0.48030275547454182, -0.64987606239115858, 0.500247875217671,
    -0.70089852846817657, 0.52020294306363413, -0.7539169221363321,
    0.54016615572732207, -0.80893198159811286, 0.56013603680375956,
    -0.86594431124259752, 0.58011137751478936, -0.92495440590171141,
    0.60009118819656049, -0.98596267070956967, 0.62007465858084287,
    -1.0489694373619314, 0.64006112527611836, -1.1139749774283152,
    0.66005004514334964, -1.1809795132510377, 0.68004097349790371,
    -1.2499832268685909, 0.70003354626279612, -1.3209862673214867,
    0.72002746535700324, -1.3939887566337756, 0.740022486732424,
    -1.4689907947103005, 0.760018410579373, -1.5459924633462283,
    0.780015073307516, -1.6249938295097781, 0.80001234098041518,
    -1.70599494802989, 0.82001010394019036, -1.7889958637967032,
    0.8400082724065624, -1.8739966135631556, 0.8600067728736559,
    -1.9609972274200078, 0.88000554515995011, -2.0499977300034904,
    0.90000453999298324, -2.1409981414840433, 0.92000371703187556,
    -2.2339984783758262, 0.94000304324830808, -2.3289987541994894,
    0.9600024916009805, -2.4259989800248047, 0.98000203995034862,
    -2.5249991649149348, 1.0000016701700867, -2.6259993162901702,
    1.0200013674196144, -2.7289994402257305, 1.0400011195484922,
    -2.8339995416955852, 1.0600009166087816, -2.9409996247720755,
    1.0800007504557996, -3.0499996927893527, 1.1000006144212435,
    -3.160999748477189, 1.1200005030455691, -3.2739997940705332,
    1.1400004118588793, -3.388999831399206, 1.1600003372015322,
    -3.5059998619613384, 1.1800002760772661, -3.6249998869834958,
    1.2000002260329496, -3.7459999074699053, 1.2200001850601288,
    -3.8689999242427588, 1.2400001515144203, -3.9939999379752096,
    1.2600001240495173, -4.1209999492183895, 1.2800001015631566,
    -4.2499999584235262, 1.3000000831528815, -4.3809999659600543,
    1.3200000680798232, -4.513999972130442, 1.3400000557390468,
    -4.6489999771823278, 1.3600000456352737, -4.7859999813184615,
    1.3800000373630039, -4.9249999847048418, 1.4000000305902423,
    -5.0659999874773751, 1.4200000250451741, -5.2089999897473334,
    1.4400000205052563, -5.3539999916058179, 1.4600000167882861,
    -5.5009999931274161, 1.4800000137450882, -5.6499999943731947,
    1.5000000112535286, -5.8009999953931519, 1.5200000092136121,
    -5.9539999962282222, 1.5400000075434697, -6.10899999691192,
    1.5600000061760728, -6.2659999974716838, 1.580000005056543,
    -6.4249999979299792, 1.6000000041399496, -6.5859999983051996,
    1.6200000033895063, -6.7489999986124039, 1.6400000027750954,
    -6.9139999988639218, 1.6600000022720582, -7.0809999990698467,
    1.6800000018602064, -7.2499999992384438, 1.7000000015230106,
    -7.4209999993764795, 1.7200000012469381, -7.5939999994894931,
    1.7400000010209089, -7.7689999995820207, 1.7600000008358521,
    -7.9459999996577757, 1.7800000006843402, -8.1249999997198,
    1.8000000005602927, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0018730753077983561, -0.0070320046035642634,
    -0.014881163609403138, -0.024932896411722812, -0.03678794411714504,
    -0.050119421191221168, -0.064659696394161748, -0.080189651799466785,
    -0.096529888822160043, -0.1135335283236628, -0.13108031583623506,
    -0.14907179532894305, -0.16742735782143534, -0.18608100626252388,
    -0.20497870683678862, -0.22407622039783898, -0.24333732699603511,
    -0.26273237224473189, -0.28223707718561936, -0.30183156388887633,
    -0.32149955768205085, -0.34122773399031009, -0.36100518357446676,
    -0.38082297470490556, -0.40067379469991227, -0.42055165644207992,
    -0.44045165809426523, -0.46036978637165238, -0.48030275547454182,
    -0.500247875217671, -0.52020294306363413, -0.54016615572732207,
    -0.56013603680375956, -0.58011137751478936, -0.60009118819656049,
    -0.62007465858084287, -0.64006112527611836, -0.66005004514334964,
    -0.68004097349790371, -0.70003354626279612, -0.72002746535700324,
    -0.740022486732424, -0.760018410579373, -0.780015073307516,
    -0.80001234098041518, -0.82001010394019036, -0.8400082724065624,
    -0.8600067728736559, -0.88000554515995011, -0.90000453999298324,
    -0.92000371703187556, -0.94000304324830808, -0.9600024916009805,
    -0.98000203995034862, -1.0000016701700867, -1.0200013674196144,
    -1.0400011195484922, -1.0600009166087816, -1.0800007504557996,
    -1.1000006144212435, -1.1200005030455691, -1.1400004118588793,
    -1.1600003372015322, -1.1800002760772661, -1.2000002260329496,
    -1.2200001850601288, -1.2400001515144203, -1.2600001240495173,
    -1.2800001015631566, -1.3000000831528815, -1.3200000680798232,
    -1.3400000557390468, -1.3600000456352737, -1.3800000373630039,
    -1.4000000305902423, -1.4200000250451741, -1.4400000205052563,
    -1.4600000167882861, -1.4800000137450882, -1.5000000112535286,
    -1.5200000092136121, -1.5400000075434697, -1.5600000061760728,
    -1.580000005056543, -1.6000000041399496, -1.6200000033895063,
    -1.6400000027750954, -1.6600000022720582, -1.6800000018602064,
    -1.7000000015230106, -1.7200000012469381, -1.7400000010209089,
    -1.7600000008358521, -1.7800000006843402, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -6.3462346100897924E-5, 0.0018730753077983561,
    -0.000483997698217998, 0.0070320046035642634, -0.0015594181952985956,
    0.014881163609403138, -0.0035335517941387723, 0.024932896411722812,
    -0.0066060279414276515, 0.03678794411714504, -0.010940289404389559,
    0.050119421191221168, -0.016670151802919218, 0.064659696394161748,
    -0.023905174100266633, 0.080189651799466785, -0.032735055588919919,
    0.096529888822160043, -0.043233235838168432, 0.1135335283236628,
    -0.055459842081882177, 0.13108031583623506, -0.069464102335528036,
    0.14907179532894305, -0.085286321089281733, 0.16742735782143534,
    -0.10295949686873729, 0.18608100626252388, -0.12251064658160472,
    0.20497870683678862, -0.14396188980107932, 0.22407622039783898,
    -0.16733133650198104, 0.24333732699603511, -0.1926338138776324,
    0.26273237224473189, -0.21988146140718839, 0.28223707718561936,
    -0.24908421805555961, 0.30183156388887633, -0.28025022115897208,
    0.32149955768205085, -0.31338613300484214, 0.34122773399031009,
    -0.34849740821276348, 0.36100518357446676, -0.38558851264754374,
    0.38082297470490556, -0.42466310265004004, 0.40067379469991227,
    -0.46572417177895586, 0.42055165644207992, -0.50877417095286281,
    0.44045165809426523, -0.55381510681416879, 0.46036978637165238,
    -0.60084862226272362, 0.48030275547454182, -0.64987606239115858,
    0.500247875217671, -0.70089852846817657, 0.52020294306363413,
    -0.7539169221363321, 0.54016615572732207, -0.80893198159811286,
    0.56013603680375956, -0.86594431124259752, 0.58011137751478936,
    -0.92495440590171141, 0.60009118819656049, -0.98596267070956967,
    0.62007465858084287, -1.0489694373619314, 0.64006112527611836,
    -1.1139749774283152, 0.66005004514334964, -1.1809795132510377,
    0.68004097349790371, -1.2499832268685909, 0.70003354626279612,
    -1.3209862673214867, 0.72002746535700324, -1.3939887566337756,
    0.740022486732424, -1.4689907947103005, 0.760018410579373,
    -1.5459924633462283, 0.780015073307516, -1.6249938295097781,
    0.80001234098041518, -1.70599494802989, 0.82001010394019036,
    -1.7889958637967032, 0.8400082724065624, -1.8739966135631556,
    0.8600067728736559, -1.9609972274200078, 0.88000554515995011,
    -2.0499977300034904, 0.90000453999298324, -2.1409981414840433,
    0.92000371703187556, -2.2339984783758262, 0.94000304324830808,
    -2.3289987541994894, 0.9600024916009805, -2.4259989800248047,
    0.98000203995034862, -2.5249991649149348, 1.0000016701700867,
    -2.6259993162901702, 1.0200013674196144, -2.7289994402257305,
    1.0400011195484922, -2.8339995416955852, 1.0600009166087816,
    -2.9409996247720755, 1.0800007504557996, -3.0499996927893527,
    1.1000006144212435, -3.160999748477189, 1.1200005030455691,
    -3.2739997940705332, 1.1400004118588793, -3.388999831399206,
    1.1600003372015322, -3.5059998619613384, 1.1800002760772661,
    -3.6249998869834958, 1.2000002260329496, -3.7459999074699053,
    1.2200001850601288, -3.8689999242427588, 1.2400001515144203,
    -3.9939999379752096, 1.2600001240495173, -4.1209999492183895,
    1.2800001015631566, -4.2499999584235262, 1.3000000831528815,
    -4.3809999659600543, 1.3200000680798232, -4.513999972130442,
    1.3400000557390468, -4.6489999771823278, 1.3600000456352737,
    -4.7859999813184615, 1.3800000373630039, -4.9249999847048418,
    1.4000000305902423, -5.0659999874773751, 1.4200000250451741,
    -5.2089999897473334, 1.4400000205052563, -5.3539999916058179,
    1.4600000167882861, -5.5009999931274161, 1.4800000137450882,
    -5.6499999943731947, 1.5000000112535286, -5.8009999953931519,
    1.5200000092136121, -5.9539999962282222, 1.5400000075434697,
    -6.10899999691192, 1.5600000061760728, -6.2659999974716838,
    1.580000005056543, -6.4249999979299792, 1.6000000041399496,
    -6.5859999983051996, 1.6200000033895063, -6.7489999986124039,
    1.6400000027750954, -6.9139999988639218, 1.6600000022720582,
    -7.0809999990698467, 1.6800000018602064, -7.2499999992384438,
    1.7000000015230106, -7.4209999993764795, 1.7200000012469381,
    -7.5939999994894931, 1.7400000010209089, -7.7689999995820207,
    1.7600000008358521, -7.9459999996577757, 1.7800000006843402, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0, 1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0018730753077983561, -0.0070320046035642634, -0.014881163609403138,
    -0.024932896411722812, -0.03678794411714504, -0.050119421191221168,
    -0.064659696394161748, -0.080189651799466785, -0.096529888822160043,
    -0.1135335283236628, -0.13108031583623506, -0.14907179532894305,
    -0.16742735782143534, -0.18608100626252388, -0.20497870683678862,
    -0.22407622039783898, -0.24333732699603511, -0.26273237224473189,
    -0.28223707718561936, -0.30183156388887633, -0.32149955768205085,
    -0.34122773399031009, -0.36100518357446676, -0.38082297470490556,
    -0.40067379469991227, -0.42055165644207992, -0.44045165809426523,
    -0.46036978637165238, -0.48030275547454182, -0.500247875217671,
    -0.52020294306363413, -0.54016615572732207, -0.56013603680375956,
    -0.58011137751478936, -0.60009118819656049, -0.62007465858084287,
    -0.64006112527611836, -0.66005004514334964, -0.68004097349790371,
    -0.70003354626279612, -0.72002746535700324, -0.740022486732424,
    -0.760018410579373, -0.780015073307516, -0.80001234098041518,
    -0.82001010394019036, -0.8400082724065624, -0.8600067728736559,
    -0.88000554515995011, -0.90000453999298324, -0.92000371703187556,
    -0.94000304324830808, -0.9600024916009805, -0.98000203995034862,
    -1.0000016701700867, -1.0200013674196144, -1.0400011195484922,
    -1.0600009166087816, -1.0800007504557996, -1.1000006144212435,
    -1.1200005030455691, -1.1400004118588793, -1.1600003372015322,
    -1.1800002760772661, -1.2000002260329496, -1.2200001850601288,
    -1.2400001515144203, -1.2600001240495173, -1.2800001015631566,
    -1.3000000831528815, -1.3200000680798232, -1.3400000557390468,
    -1.3600000456352737, -1.3800000373630039, -1.4000000305902423,
    -1.4200000250451741, -1.4400000205052563, -1.4600000167882861,
    -1.4800000137450882, -1.5000000112535286, -1.5200000092136121,
    -1.5400000075434697, -1.5600000061760728, -1.580000005056543,
    -1.6000000041399496, -1.6200000033895063, -1.6400000027750954,
    -1.6600000022720582, -1.6800000018602064, -1.7000000015230106,
    -1.7200000012469381, -1.7400000010209089, -1.7600000008358521, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -6.3462346100897924E-5, 0.0018730753077983561, -0.000483997698217998,
    0.0070320046035642634, -0.0015594181952985956, 0.014881163609403138,
    -0.0035335517941387723, 0.024932896411722812, -0.0066060279414276515,
    0.03678794411714504, -0.010940289404389559, 0.050119421191221168,
    -0.016670151802919218, 0.064659696394161748, -0.023905174100266633,
    0.080189651799466785, -0.032735055588919919, 0.096529888822160043,
    -0.043233235838168432, 0.1135335283236628, -0.055459842081882177,
    0.13108031583623506, -0.069464102335528036, 0.14907179532894305,
    -0.085286321089281733, 0.16742735782143534, -0.10295949686873729,
    0.18608100626252388, -0.12251064658160472, 0.20497870683678862,
    -0.14396188980107932, 0.22407622039783898, -0.16733133650198104,
    0.24333732699603511, -0.1926338138776324, 0.26273237224473189,
    -0.21988146140718839, 0.28223707718561936, -0.24908421805555961,
    0.30183156388887633, -0.28025022115897208, 0.32149955768205085,
    -0.31338613300484214, 0.34122773399031009, -0.34849740821276348,
    0.36100518357446676, -0.38558851264754374, 0.38082297470490556,
    -0.42466310265004004, 0.40067379469991227, -0.46572417177895586,
    0.42055165644207992, -0.50877417095286281, 0.44045165809426523,
    -0.55381510681416879, 0.46036978637165238, -0.60084862226272362,
    0.48030275547454182, -0.64987606239115858, 0.500247875217671,
    -0.70089852846817657, 0.52020294306363413, -0.7539169221363321,
    0.54016615572732207, -0.80893198159811286, 0.56013603680375956,
    -0.86594431124259752, 0.58011137751478936, -0.92495440590171141,
    0.60009118819656049, -0.98596267070956967, 0.62007465858084287,
    -1.0489694373619314, 0.64006112527611836, -1.1139749774283152,
    0.66005004514334964, -1.1809795132510377, 0.68004097349790371,
    -1.2499832268685909, 0.70003354626279612, -1.3209862673214867,
    0.72002746535700324, -1.3939887566337756, 0.740022486732424,
    -1.4689907947103005, 0.760018410579373, -1.5459924633462283,
    0.780015073307516, -1.6249938295097781, 0.80001234098041518,
    -1.70599494802989, 0.82001010394019036, -1.7889958637967032,
    0.8400082724065624, -1.8739966135631556, 0.8600067728736559,
    -1.9609972274200078, 0.88000554515995011, -2.0499977300034904,
    0.90000453999298324, -2.1409981414840433, 0.92000371703187556,
    -2.2339984783758262, 0.94000304324830808, -2.3289987541994894,
    0.9600024916009805, -2.4259989800248047, 0.98000203995034862,
    -2.5249991649149348, 1.0000016701700867, -2.6259993162901702,
    1.0200013674196144, -2.7289994402257305, 1.0400011195484922,
    -2.8339995416955852, 1.0600009166087816, -2.9409996247720755,
    1.0800007504557996, -3.0499996927893527, 1.1000006144212435,
    -3.160999748477189, 1.1200005030455691, -3.2739997940705332,
    1.1400004118588793, -3.388999831399206, 1.1600003372015322,
    -3.5059998619613384, 1.1800002760772661, -3.6249998869834958,
    1.2000002260329496, -3.7459999074699053, 1.2200001850601288,
    -3.8689999242427588, 1.2400001515144203, -3.9939999379752096,
    1.2600001240495173, -4.1209999492183895, 1.2800001015631566,
    -4.2499999584235262, 1.3000000831528815, -4.3809999659600543,
    1.3200000680798232, -4.513999972130442, 1.3400000557390468,
    -4.6489999771823278, 1.3600000456352737, -4.7859999813184615,
    1.3800000373630039, -4.9249999847048418, 1.4000000305902423,
    -5.0659999874773751, 1.4200000250451741, -5.2089999897473334,
    1.4400000205052563, -5.3539999916058179, 1.4600000167882861,
    -5.5009999931274161, 1.4800000137450882, -5.6499999943731947,
    1.5000000112535286, -5.8009999953931519, 1.5200000092136121,
    -5.9539999962282222, 1.5400000075434697, -6.10899999691192,
    1.5600000061760728, -6.2659999974716838, 1.580000005056543,
    -6.4249999979299792, 1.6000000041399496, -6.5859999983051996,
    1.6200000033895063, -6.7489999986124039, 1.6400000027750954,
    -6.9139999988639218, 1.6600000022720582, -7.0809999990698467,
    1.6800000018602064, -7.2499999992384438, 1.7000000015230106,
    -7.4209999993764795, 1.7200000012469381, -7.5939999994894931,
    1.7400000010209089, -7.7689999995820207, 1.7600000008358521, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0018730753077983561, -0.0070320046035642634, -0.014881163609403138,
    -0.024932896411722812, -0.03678794411714504, -0.050119421191221168,
    -0.064659696394161748, -0.080189651799466785, -0.096529888822160043,
    -0.1135335283236628, -0.13108031583623506, -0.14907179532894305,
    -0.16742735782143534, -0.18608100626252388, -0.20497870683678862,
    -0.22407622039783898, -0.24333732699603511, -0.26273237224473189,
    -0.28223707718561936, -0.30183156388887633, -0.32149955768205085,
    -0.34122773399031009, -0.36100518357446676, -0.38082297470490556,
    -0.40067379469991227, -0.42055165644207992, -0.44045165809426523,
    -0.46036978637165238, -0.48030275547454182, -0.500247875217671,
    -0.52020294306363413, -0.54016615572732207, -0.56013603680375956,
    -0.58011137751478936, -0.60009118819656049, -0.62007465858084287,
    -0.64006112527611836, -0.66005004514334964, -0.68004097349790371,
    -0.70003354626279612, -0.72002746535700324, -0.740022486732424,
    -0.760018410579373, -0.780015073307516, -0.80001234098041518,
    -0.82001010394019036, -0.8400082724065624, -0.8600067728736559,
    -0.88000554515995011, -0.90000453999298324, -0.92000371703187556,
    -0.94000304324830808, -0.9600024916009805, -0.98000203995034862,
    -1.0000016701700867, -1.0200013674196144, -1.0400011195484922,
    -1.0600009166087816, -1.0800007504557996, -1.1000006144212435,
    -1.1200005030455691, -1.1400004118588793, -1.1600003372015322,
    -1.1800002760772661, -1.2000002260329496, -1.2200001850601288,
    -1.2400001515144203, -1.2600001240495173, -1.2800001015631566,
    -1.3000000831528815, -1.3200000680798232, -1.3400000557390468,
    -1.3600000456352737, -1.3800000373630039, -1.4000000305902423,
    -1.4200000250451741, -1.4400000205052563, -1.4600000167882861,
    -1.4800000137450882, -1.5000000112535286, -1.5200000092136121,
    -1.5400000075434697, -1.5600000061760728, -1.580000005056543,
    -1.6000000041399496, -1.6200000033895063, -1.6400000027750954,
    -1.6600000022720582, -1.6800000018602064, -1.7000000015230106,
    -1.7200000012469381, -1.7400000010209089, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -6.3462346100897924E-5,
    0.0018730753077983561, -0.000483997698217998, 0.0070320046035642634,
    -0.0015594181952985956, 0.014881163609403138, -0.0035335517941387723,
    0.024932896411722812, -0.0066060279414276515, 0.03678794411714504,
    -0.010940289404389559, 0.050119421191221168, -0.016670151802919218,
    0.064659696394161748, -0.023905174100266633, 0.080189651799466785,
    -0.032735055588919919, 0.096529888822160043, -0.043233235838168432,
    0.1135335283236628, -0.055459842081882177, 0.13108031583623506,
    -0.069464102335528036, 0.14907179532894305, -0.085286321089281733,
    0.16742735782143534, -0.10295949686873729, 0.18608100626252388,
    -0.12251064658160472, 0.20497870683678862, -0.14396188980107932,
    0.22407622039783898, -0.16733133650198104, 0.24333732699603511,
    -0.1926338138776324, 0.26273237224473189, -0.21988146140718839,
    0.28223707718561936, -0.24908421805555961, 0.30183156388887633,
    -0.28025022115897208, 0.32149955768205085, -0.31338613300484214,
    0.34122773399031009, -0.34849740821276348, 0.36100518357446676,
    -0.38558851264754374, 0.38082297470490556, -0.42466310265004004,
    0.40067379469991227, -0.46572417177895586, 0.42055165644207992,
    -0.50877417095286281, 0.44045165809426523, -0.55381510681416879,
    0.46036978637165238, -0.60084862226272362, 0.48030275547454182,
    -0.64987606239115858, 0.500247875217671, -0.70089852846817657,
    0.52020294306363413, -0.7539169221363321, 0.54016615572732207,
    -0.80893198159811286, 0.56013603680375956, -0.86594431124259752,
    0.58011137751478936, -0.92495440590171141, 0.60009118819656049,
    -0.98596267070956967, 0.62007465858084287, -1.0489694373619314,
    0.64006112527611836, -1.1139749774283152, 0.66005004514334964,
    -1.1809795132510377, 0.68004097349790371, -1.2499832268685909,
    0.70003354626279612, -1.3209862673214867, 0.72002746535700324,
    -1.3939887566337756, 0.740022486732424, -1.4689907947103005,
    0.760018410579373, -1.5459924633462283, 0.780015073307516,
    -1.6249938295097781, 0.80001234098041518, -1.70599494802989,
    0.82001010394019036, -1.7889958637967032, 0.8400082724065624,
    -1.8739966135631556, 0.8600067728736559, -1.9609972274200078,
    0.88000554515995011, -2.0499977300034904, 0.90000453999298324,
    -2.1409981414840433, 0.92000371703187556, -2.2339984783758262,
    0.94000304324830808, -2.3289987541994894, 0.9600024916009805,
    -2.4259989800248047, 0.98000203995034862, -2.5249991649149348,
    1.0000016701700867, -2.6259993162901702, 1.0200013674196144,
    -2.7289994402257305, 1.0400011195484922, -2.8339995416955852,
    1.0600009166087816, -2.9409996247720755, 1.0800007504557996,
    -3.0499996927893527, 1.1000006144212435, -3.160999748477189,
    1.1200005030455691, -3.2739997940705332, 1.1400004118588793,
    -3.388999831399206, 1.1600003372015322, -3.5059998619613384,
    1.1800002760772661, -3.6249998869834958, 1.2000002260329496,
    -3.7459999074699053, 1.2200001850601288, -3.8689999242427588,
    1.2400001515144203, -3.9939999379752096, 1.2600001240495173,
    -4.1209999492183895, 1.2800001015631566, -4.2499999584235262,
    1.3000000831528815, -4.3809999659600543, 1.3200000680798232,
    -4.513999972130442, 1.3400000557390468, -4.6489999771823278,
    1.3600000456352737, -4.7859999813184615, 1.3800000373630039,
    -4.9249999847048418, 1.4000000305902423, -5.0659999874773751,
    1.4200000250451741, -5.2089999897473334, 1.4400000205052563,
    -5.3539999916058179, 1.4600000167882861, -5.5009999931274161,
    1.4800000137450882, -5.6499999943731947, 1.5000000112535286,
    -5.8009999953931519, 1.5200000092136121, -5.9539999962282222,
    1.5400000075434697, -6.10899999691192, 1.5600000061760728,
    -6.2659999974716838, 1.580000005056543, -6.4249999979299792,
    1.6000000041399496, -6.5859999983051996, 1.6200000033895063,
    -6.7489999986124039, 1.6400000027750954, -6.9139999988639218,
    1.6600000022720582, -7.0809999990698467, 1.6800000018602064,
    -7.2499999992384438, 1.7000000015230106, -7.4209999993764795,
    1.7200000012469381, -7.5939999994894931, 1.7400000010209089, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0018730753077983561, -0.0070320046035642634, -0.014881163609403138,
    -0.024932896411722812, -0.03678794411714504, -0.050119421191221168,
    -0.064659696394161748, -0.080189651799466785, -0.096529888822160043,
    -0.1135335283236628, -0.13108031583623506, -0.14907179532894305,
    -0.16742735782143534, -0.18608100626252388, -0.20497870683678862,
    -0.22407622039783898, -0.24333732699603511, -0.26273237224473189,
    -0.28223707718561936, -0.30183156388887633, -0.32149955768205085,
    -0.34122773399031009, -0.36100518357446676, -0.38082297470490556,
    -0.40067379469991227, -0.42055165644207992, -0.44045165809426523,
    -0.46036978637165238, -0.48030275547454182, -0.500247875217671,
    -0.52020294306363413, -0.54016615572732207, -0.56013603680375956,
    -0.58011137751478936, -0.60009118819656049, -0.62007465858084287,
    -0.64006112527611836, -0.66005004514334964, -0.68004097349790371,
    -0.70003354626279612, -0.72002746535700324, -0.740022486732424,
    -0.760018410579373, -0.780015073307516, -0.80001234098041518,
    -0.82001010394019036, -0.8400082724065624, -0.8600067728736559,
    -0.88000554515995011, -0.90000453999298324, -0.92000371703187556,
    -0.94000304324830808, -0.9600024916009805, -0.98000203995034862,
    -1.0000016701700867, -1.0200013674196144, -1.0400011195484922,
    -1.0600009166087816, -1.0800007504557996, -1.1000006144212435,
    -1.1200005030455691, -1.1400004118588793, -1.1600003372015322,
    -1.1800002760772661, -1.2000002260329496, -1.2200001850601288,
    -1.2400001515144203, -1.2600001240495173, -1.2800001015631566,
    -1.3000000831528815, -1.3200000680798232, -1.3400000557390468,
    -1.3600000456352737, -1.3800000373630039, -1.4000000305902423,
    -1.4200000250451741, -1.4400000205052563, -1.4600000167882861,
    -1.4800000137450882, -1.5000000112535286, -1.5200000092136121,
    -1.5400000075434697, -1.5600000061760728, -1.580000005056543,
    -1.6000000041399496, -1.6200000033895063, -1.6400000027750954,
    -1.6600000022720582, -1.6800000018602064, -1.7000000015230106,
    -1.7200000012469381, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -6.3462346100897924E-5,
    0.0018730753077983561, -0.000483997698217998, 0.0070320046035642634,
    -0.0015594181952985956, 0.014881163609403138, -0.0035335517941387723,
    0.024932896411722812, -0.0066060279414276515, 0.03678794411714504,
    -0.010940289404389559, 0.050119421191221168, -0.016670151802919218,
    0.064659696394161748, -0.023905174100266633, 0.080189651799466785,
    -0.032735055588919919, 0.096529888822160043, -0.043233235838168432,
    0.1135335283236628, -0.055459842081882177, 0.13108031583623506,
    -0.069464102335528036, 0.14907179532894305, -0.085286321089281733,
    0.16742735782143534, -0.10295949686873729, 0.18608100626252388,
    -0.12251064658160472, 0.20497870683678862, -0.14396188980107932,
    0.22407622039783898, -0.16733133650198104, 0.24333732699603511,
    -0.1926338138776324, 0.26273237224473189, -0.21988146140718839,
    0.28223707718561936, -0.24908421805555961, 0.30183156388887633,
    -0.28025022115897208, 0.32149955768205085, -0.31338613300484214,
    0.34122773399031009, -0.34849740821276348, 0.36100518357446676,
    -0.38558851264754374, 0.38082297470490556, -0.42466310265004004,
    0.40067379469991227, -0.46572417177895586, 0.42055165644207992,
    -0.50877417095286281, 0.44045165809426523, -0.55381510681416879,
    0.46036978637165238, -0.60084862226272362, 0.48030275547454182,
    -0.64987606239115858, 0.500247875217671, -0.70089852846817657,
    0.52020294306363413, -0.7539169221363321, 0.54016615572732207,
    -0.80893198159811286, 0.56013603680375956, -0.86594431124259752,
    0.58011137751478936, -0.92495440590171141, 0.60009118819656049,
    -0.98596267070956967, 0.62007465858084287, -1.0489694373619314,
    0.64006112527611836, -1.1139749774283152, 0.66005004514334964,
    -1.1809795132510377, 0.68004097349790371, -1.2499832268685909,
    0.70003354626279612, -1.3209862673214867, 0.72002746535700324,
    -1.3939887566337756, 0.740022486732424, -1.4689907947103005,
    0.760018410579373, -1.5459924633462283, 0.780015073307516,
    -1.6249938295097781, 0.80001234098041518, -1.70599494802989,
    0.82001010394019036, -1.7889958637967032, 0.8400082724065624,
    -1.8739966135631556, 0.8600067728736559, -1.9609972274200078,
    0.88000554515995011, -2.0499977300034904, 0.90000453999298324,
    -2.1409981414840433, 0.92000371703187556, -2.2339984783758262,
    0.94000304324830808, -2.3289987541994894, 0.9600024916009805,
    -2.4259989800248047, 0.98000203995034862, -2.5249991649149348,
    1.0000016701700867, -2.6259993162901702, 1.0200013674196144,
    -2.7289994402257305, 1.0400011195484922, -2.8339995416955852,
    1.0600009166087816, -2.9409996247720755, 1.0800007504557996,
    -3.0499996927893527, 1.1000006144212435, -3.160999748477189,
    1.1200005030455691, -3.2739997940705332, 1.1400004118588793,
    -3.388999831399206, 1.1600003372015322, -3.5059998619613384,
    1.1800002760772661, -3.6249998869834958, 1.2000002260329496,
    -3.7459999074699053, 1.2200001850601288, -3.8689999242427588,
    1.2400001515144203, -3.9939999379752096, 1.2600001240495173,
    -4.1209999492183895, 1.2800001015631566, -4.2499999584235262,
    1.3000000831528815, -4.3809999659600543, 1.3200000680798232,
    -4.513999972130442, 1.3400000557390468, -4.6489999771823278,
    1.3600000456352737, -4.7859999813184615, 1.3800000373630039,
    -4.9249999847048418, 1.4000000305902423, -5.0659999874773751,
    1.4200000250451741, -5.2089999897473334, 1.4400000205052563,
    -5.3539999916058179, 1.4600000167882861, -5.5009999931274161,
    1.4800000137450882, -5.6499999943731947, 1.5000000112535286,
    -5.8009999953931519, 1.5200000092136121, -5.9539999962282222,
    1.5400000075434697, -6.10899999691192, 1.5600000061760728,
    -6.2659999974716838, 1.580000005056543, -6.4249999979299792,
    1.6000000041399496, -6.5859999983051996, 1.6200000033895063,
    -6.7489999986124039, 1.6400000027750954, -6.9139999988639218,
    1.6600000022720582, -7.0809999990698467, 1.6800000018602064,
    -7.2499999992384438, 1.7000000015230106, -7.4209999993764795,
    1.7200000012469381, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  static const double b_Kr[2000]{ 0.0, -4.7049524508543223E-5, 0.0,
    -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646,
    0.0, -0.043706823933910781, 0.0, -0.044209201215564278, 0.0,
    -0.044711578498060386, 0.0, -0.045213955781246373, 0.0,
    -0.045716333064997179, 0.0, -0.046218710349210428, 0.0,
    -0.046721087633802283, 0.0, -0.047223464918704126, 0.0, -0.04772584220385976,
    -0.0, -0.0, 0.0, -4.7049524508543223E-5, 0.0, -0.00017663596950005981, 0.0,
    -0.00037379792955533883, 0.0, -0.00062628604194840448, 0.0,
    -0.00092407137670989326, 0.0, -0.001258942940453422, 0.0,
    -0.0016241781403802387, 0.0, -0.0020142729830255847, 0.0,
    -0.0024247211796751215, 0.0, -0.0028518332931761728, 0.0,
    -0.0032925886678702929, 0.0, -0.0037445142000754536, 0.0,
    -0.0042055850837516293, 0.0, -0.0046741435479248309, 0.0,
    -0.0051488323245170945, 0.0, -0.0056285401764128206, 0.0,
    -0.0061123572996115063, 0.0, -0.0065995388095986644, 0.0,
    -0.007089474846514795, 0.0, -0.0075816660993378564, 0.0,
    -0.0080757037667787534, 0.0, -0.008571253150650205, 0.0,
    -0.0090680402232532851, 0.0, -0.0095658406296836862, 0.0,
    -0.01006447068368165, 0.0, -0.010563779995657394, 0.0, -0.011063645437028818,
    0.0, -0.01156396619863918, 0.0, -0.012064659744931656, 0.0,
    -0.012565658501507657, 0.0, -0.013066907143128935, 0.0,
    -0.013568360373321516, 0.0, -0.014069981106469148, 0.0,
    -0.014571738979437315, 0.0, -0.015073609132994025, 0.0, -0.01557557121412155,
    0.0, -0.016077608559178357, 0.0, -0.016579707525128662, 0.0,
    -0.017081856941999509, 0.0, -0.01758404766459052, 0.0, -0.018086272205444912,
    0.0, -0.018588524434351546, 0.0, -0.019090799332318047, 0.0,
    -0.019593092790141004, 0.0, -0.020095401443489209, 0.0,
    -0.020597722537881252, 0.0, -0.021100053818138478, 0.0,
    -0.021602393437876781, 0.0, -0.022104739885404703, 0.0,
    -0.022607091923053966, 0.0, -0.023109448537507481, 0.0,
    -0.023611808899131389, 0.0, -0.024114172328678931, 0.0,
    -0.024616538270029908, 0.0, -0.0251189062678716, 0.0, -0.025621275949425482,
    0.0, -0.026123647009486311, 0.0, -0.026626019198173173, 0.0,
    -0.027128392310900876, 0.0, -0.027630766180169238, 0.0, -0.0281331406688407,
    0.0, -0.02863551566463653, 0.0, -0.029137891075630671, 0.0,
    -0.02964026682656044, 0.0, -0.030142642855805962, 0.0, -0.03064501911291715,
    0.0, -0.031147395556588967, 0.0, -0.031649772153003713, 0.0,
    -0.032152148874473784, 0.0, -0.0326545256983305, 0.0, -0.03315690260601431,
    0.0, -0.033659279582329935, 0.0, -0.034161656614836548, 0.0,
    -0.034664033693348438, 0.0, -0.035166410809526275, 0.0, -0.03566878795654238,
    0.0, -0.036171165128806713, 0.0, -0.03667354232174256, 0.0,
    -0.0371759195316028, 0.0, -0.037678296755319571, 0.0, -0.0381806739903811,
    0.0, -0.038683051234730932, 0.0, -0.039185428486685395, 0.0,
    -0.039687805744865995, 0.0, -0.040190183008144122, 0.0,
    -0.040692560275595757, 0.0, -0.041194937546464368, 0.0,
    -0.041697314820130567, 0.0, -0.042199692096087232, 0.0,
    -0.042702069373919169, 0.0, -0.04320444665328646, 0.0, -0.043706823933910781,
    0.0, -0.044209201215564278, 0.0, -0.044711578498060386, 0.0,
    -0.045213955781246373, 0.0, -0.045716333064997179, 0.0,
    -0.046218710349210428, 0.0, -0.046721087633802283, 0.0,
    -0.047223464918704126, -0.0, -0.0, -0.0, -0.0, 0.0, -4.7049524508543223E-5,
    0.0, -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646,
    0.0, -0.043706823933910781, 0.0, -0.044209201215564278, 0.0,
    -0.044711578498060386, 0.0, -0.045213955781246373, 0.0,
    -0.045716333064997179, 0.0, -0.046218710349210428, 0.0,
    -0.046721087633802283, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
    -4.7049524508543223E-5, 0.0, -0.00017663596950005981, 0.0,
    -0.00037379792955533883, 0.0, -0.00062628604194840448, 0.0,
    -0.00092407137670989326, 0.0, -0.001258942940453422, 0.0,
    -0.0016241781403802387, 0.0, -0.0020142729830255847, 0.0,
    -0.0024247211796751215, 0.0, -0.0028518332931761728, 0.0,
    -0.0032925886678702929, 0.0, -0.0037445142000754536, 0.0,
    -0.0042055850837516293, 0.0, -0.0046741435479248309, 0.0,
    -0.0051488323245170945, 0.0, -0.0056285401764128206, 0.0,
    -0.0061123572996115063, 0.0, -0.0065995388095986644, 0.0,
    -0.007089474846514795, 0.0, -0.0075816660993378564, 0.0,
    -0.0080757037667787534, 0.0, -0.008571253150650205, 0.0,
    -0.0090680402232532851, 0.0, -0.0095658406296836862, 0.0,
    -0.01006447068368165, 0.0, -0.010563779995657394, 0.0, -0.011063645437028818,
    0.0, -0.01156396619863918, 0.0, -0.012064659744931656, 0.0,
    -0.012565658501507657, 0.0, -0.013066907143128935, 0.0,
    -0.013568360373321516, 0.0, -0.014069981106469148, 0.0,
    -0.014571738979437315, 0.0, -0.015073609132994025, 0.0, -0.01557557121412155,
    0.0, -0.016077608559178357, 0.0, -0.016579707525128662, 0.0,
    -0.017081856941999509, 0.0, -0.01758404766459052, 0.0, -0.018086272205444912,
    0.0, -0.018588524434351546, 0.0, -0.019090799332318047, 0.0,
    -0.019593092790141004, 0.0, -0.020095401443489209, 0.0,
    -0.020597722537881252, 0.0, -0.021100053818138478, 0.0,
    -0.021602393437876781, 0.0, -0.022104739885404703, 0.0,
    -0.022607091923053966, 0.0, -0.023109448537507481, 0.0,
    -0.023611808899131389, 0.0, -0.024114172328678931, 0.0,
    -0.024616538270029908, 0.0, -0.0251189062678716, 0.0, -0.025621275949425482,
    0.0, -0.026123647009486311, 0.0, -0.026626019198173173, 0.0,
    -0.027128392310900876, 0.0, -0.027630766180169238, 0.0, -0.0281331406688407,
    0.0, -0.02863551566463653, 0.0, -0.029137891075630671, 0.0,
    -0.02964026682656044, 0.0, -0.030142642855805962, 0.0, -0.03064501911291715,
    0.0, -0.031147395556588967, 0.0, -0.031649772153003713, 0.0,
    -0.032152148874473784, 0.0, -0.0326545256983305, 0.0, -0.03315690260601431,
    0.0, -0.033659279582329935, 0.0, -0.034161656614836548, 0.0,
    -0.034664033693348438, 0.0, -0.035166410809526275, 0.0, -0.03566878795654238,
    0.0, -0.036171165128806713, 0.0, -0.03667354232174256, 0.0,
    -0.0371759195316028, 0.0, -0.037678296755319571, 0.0, -0.0381806739903811,
    0.0, -0.038683051234730932, 0.0, -0.039185428486685395, 0.0,
    -0.039687805744865995, 0.0, -0.040190183008144122, 0.0,
    -0.040692560275595757, 0.0, -0.041194937546464368, 0.0,
    -0.041697314820130567, 0.0, -0.042199692096087232, 0.0,
    -0.042702069373919169, 0.0, -0.04320444665328646, 0.0, -0.043706823933910781,
    0.0, -0.044209201215564278, 0.0, -0.044711578498060386, 0.0,
    -0.045213955781246373, 0.0, -0.045716333064997179, 0.0,
    -0.046218710349210428, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
    -4.7049524508543223E-5, 0.0, -0.00017663596950005981, 0.0,
    -0.00037379792955533883, 0.0, -0.00062628604194840448, 0.0,
    -0.00092407137670989326, 0.0, -0.001258942940453422, 0.0,
    -0.0016241781403802387, 0.0, -0.0020142729830255847, 0.0,
    -0.0024247211796751215, 0.0, -0.0028518332931761728, 0.0,
    -0.0032925886678702929, 0.0, -0.0037445142000754536, 0.0,
    -0.0042055850837516293, 0.0, -0.0046741435479248309, 0.0,
    -0.0051488323245170945, 0.0, -0.0056285401764128206, 0.0,
    -0.0061123572996115063, 0.0, -0.0065995388095986644, 0.0,
    -0.007089474846514795, 0.0, -0.0075816660993378564, 0.0,
    -0.0080757037667787534, 0.0, -0.008571253150650205, 0.0,
    -0.0090680402232532851, 0.0, -0.0095658406296836862, 0.0,
    -0.01006447068368165, 0.0, -0.010563779995657394, 0.0, -0.011063645437028818,
    0.0, -0.01156396619863918, 0.0, -0.012064659744931656, 0.0,
    -0.012565658501507657, 0.0, -0.013066907143128935, 0.0,
    -0.013568360373321516, 0.0, -0.014069981106469148, 0.0,
    -0.014571738979437315, 0.0, -0.015073609132994025, 0.0, -0.01557557121412155,
    0.0, -0.016077608559178357, 0.0, -0.016579707525128662, 0.0,
    -0.017081856941999509, 0.0, -0.01758404766459052, 0.0, -0.018086272205444912,
    0.0, -0.018588524434351546, 0.0, -0.019090799332318047, 0.0,
    -0.019593092790141004, 0.0, -0.020095401443489209, 0.0,
    -0.020597722537881252, 0.0, -0.021100053818138478, 0.0,
    -0.021602393437876781, 0.0, -0.022104739885404703, 0.0,
    -0.022607091923053966, 0.0, -0.023109448537507481, 0.0,
    -0.023611808899131389, 0.0, -0.024114172328678931, 0.0,
    -0.024616538270029908, 0.0, -0.0251189062678716, 0.0, -0.025621275949425482,
    0.0, -0.026123647009486311, 0.0, -0.026626019198173173, 0.0,
    -0.027128392310900876, 0.0, -0.027630766180169238, 0.0, -0.0281331406688407,
    0.0, -0.02863551566463653, 0.0, -0.029137891075630671, 0.0,
    -0.02964026682656044, 0.0, -0.030142642855805962, 0.0, -0.03064501911291715,
    0.0, -0.031147395556588967, 0.0, -0.031649772153003713, 0.0,
    -0.032152148874473784, 0.0, -0.0326545256983305, 0.0, -0.03315690260601431,
    0.0, -0.033659279582329935, 0.0, -0.034161656614836548, 0.0,
    -0.034664033693348438, 0.0, -0.035166410809526275, 0.0, -0.03566878795654238,
    0.0, -0.036171165128806713, 0.0, -0.03667354232174256, 0.0,
    -0.0371759195316028, 0.0, -0.037678296755319571, 0.0, -0.0381806739903811,
    0.0, -0.038683051234730932, 0.0, -0.039185428486685395, 0.0,
    -0.039687805744865995, 0.0, -0.040190183008144122, 0.0,
    -0.040692560275595757, 0.0, -0.041194937546464368, 0.0,
    -0.041697314820130567, 0.0, -0.042199692096087232, 0.0,
    -0.042702069373919169, 0.0, -0.04320444665328646, 0.0, -0.043706823933910781,
    0.0, -0.044209201215564278, 0.0, -0.044711578498060386, 0.0,
    -0.045213955781246373, 0.0, -0.045716333064997179, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, -4.7049524508543223E-5, 0.0,
    -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646,
    0.0, -0.043706823933910781, 0.0, -0.044209201215564278, 0.0,
    -0.044711578498060386, 0.0, -0.045213955781246373, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, -4.7049524508543223E-5,
    0.0, -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646,
    0.0, -0.043706823933910781, 0.0, -0.044209201215564278, 0.0,
    -0.044711578498060386, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, -4.7049524508543223E-5, 0.0,
    -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646,
    0.0, -0.043706823933910781, 0.0, -0.044209201215564278, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, -4.7049524508543223E-5, 0.0, -0.00017663596950005981, 0.0,
    -0.00037379792955533883, 0.0, -0.00062628604194840448, 0.0,
    -0.00092407137670989326, 0.0, -0.001258942940453422, 0.0,
    -0.0016241781403802387, 0.0, -0.0020142729830255847, 0.0,
    -0.0024247211796751215, 0.0, -0.0028518332931761728, 0.0,
    -0.0032925886678702929, 0.0, -0.0037445142000754536, 0.0,
    -0.0042055850837516293, 0.0, -0.0046741435479248309, 0.0,
    -0.0051488323245170945, 0.0, -0.0056285401764128206, 0.0,
    -0.0061123572996115063, 0.0, -0.0065995388095986644, 0.0,
    -0.007089474846514795, 0.0, -0.0075816660993378564, 0.0,
    -0.0080757037667787534, 0.0, -0.008571253150650205, 0.0,
    -0.0090680402232532851, 0.0, -0.0095658406296836862, 0.0,
    -0.01006447068368165, 0.0, -0.010563779995657394, 0.0, -0.011063645437028818,
    0.0, -0.01156396619863918, 0.0, -0.012064659744931656, 0.0,
    -0.012565658501507657, 0.0, -0.013066907143128935, 0.0,
    -0.013568360373321516, 0.0, -0.014069981106469148, 0.0,
    -0.014571738979437315, 0.0, -0.015073609132994025, 0.0, -0.01557557121412155,
    0.0, -0.016077608559178357, 0.0, -0.016579707525128662, 0.0,
    -0.017081856941999509, 0.0, -0.01758404766459052, 0.0, -0.018086272205444912,
    0.0, -0.018588524434351546, 0.0, -0.019090799332318047, 0.0,
    -0.019593092790141004, 0.0, -0.020095401443489209, 0.0,
    -0.020597722537881252, 0.0, -0.021100053818138478, 0.0,
    -0.021602393437876781, 0.0, -0.022104739885404703, 0.0,
    -0.022607091923053966, 0.0, -0.023109448537507481, 0.0,
    -0.023611808899131389, 0.0, -0.024114172328678931, 0.0,
    -0.024616538270029908, 0.0, -0.0251189062678716, 0.0, -0.025621275949425482,
    0.0, -0.026123647009486311, 0.0, -0.026626019198173173, 0.0,
    -0.027128392310900876, 0.0, -0.027630766180169238, 0.0, -0.0281331406688407,
    0.0, -0.02863551566463653, 0.0, -0.029137891075630671, 0.0,
    -0.02964026682656044, 0.0, -0.030142642855805962, 0.0, -0.03064501911291715,
    0.0, -0.031147395556588967, 0.0, -0.031649772153003713, 0.0,
    -0.032152148874473784, 0.0, -0.0326545256983305, 0.0, -0.03315690260601431,
    0.0, -0.033659279582329935, 0.0, -0.034161656614836548, 0.0,
    -0.034664033693348438, 0.0, -0.035166410809526275, 0.0, -0.03566878795654238,
    0.0, -0.036171165128806713, 0.0, -0.03667354232174256, 0.0,
    -0.0371759195316028, 0.0, -0.037678296755319571, 0.0, -0.0381806739903811,
    0.0, -0.038683051234730932, 0.0, -0.039185428486685395, 0.0,
    -0.039687805744865995, 0.0, -0.040190183008144122, 0.0,
    -0.040692560275595757, 0.0, -0.041194937546464368, 0.0,
    -0.041697314820130567, 0.0, -0.042199692096087232, 0.0,
    -0.042702069373919169, 0.0, -0.04320444665328646, 0.0, -0.043706823933910781,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, -4.7049524508543223E-5, 0.0,
    -0.00017663596950005981, 0.0, -0.00037379792955533883, 0.0,
    -0.00062628604194840448, 0.0, -0.00092407137670989326, 0.0,
    -0.001258942940453422, 0.0, -0.0016241781403802387, 0.0,
    -0.0020142729830255847, 0.0, -0.0024247211796751215, 0.0,
    -0.0028518332931761728, 0.0, -0.0032925886678702929, 0.0,
    -0.0037445142000754536, 0.0, -0.0042055850837516293, 0.0,
    -0.0046741435479248309, 0.0, -0.0051488323245170945, 0.0,
    -0.0056285401764128206, 0.0, -0.0061123572996115063, 0.0,
    -0.0065995388095986644, 0.0, -0.007089474846514795, 0.0,
    -0.0075816660993378564, 0.0, -0.0080757037667787534, 0.0,
    -0.008571253150650205, 0.0, -0.0090680402232532851, 0.0,
    -0.0095658406296836862, 0.0, -0.01006447068368165, 0.0,
    -0.010563779995657394, 0.0, -0.011063645437028818, 0.0, -0.01156396619863918,
    0.0, -0.012064659744931656, 0.0, -0.012565658501507657, 0.0,
    -0.013066907143128935, 0.0, -0.013568360373321516, 0.0,
    -0.014069981106469148, 0.0, -0.014571738979437315, 0.0,
    -0.015073609132994025, 0.0, -0.01557557121412155, 0.0, -0.016077608559178357,
    0.0, -0.016579707525128662, 0.0, -0.017081856941999509, 0.0,
    -0.01758404766459052, 0.0, -0.018086272205444912, 0.0, -0.018588524434351546,
    0.0, -0.019090799332318047, 0.0, -0.019593092790141004, 0.0,
    -0.020095401443489209, 0.0, -0.020597722537881252, 0.0,
    -0.021100053818138478, 0.0, -0.021602393437876781, 0.0,
    -0.022104739885404703, 0.0, -0.022607091923053966, 0.0,
    -0.023109448537507481, 0.0, -0.023611808899131389, 0.0,
    -0.024114172328678931, 0.0, -0.024616538270029908, 0.0, -0.0251189062678716,
    0.0, -0.025621275949425482, 0.0, -0.026123647009486311, 0.0,
    -0.026626019198173173, 0.0, -0.027128392310900876, 0.0,
    -0.027630766180169238, 0.0, -0.0281331406688407, 0.0, -0.02863551566463653,
    0.0, -0.029137891075630671, 0.0, -0.02964026682656044, 0.0,
    -0.030142642855805962, 0.0, -0.03064501911291715, 0.0, -0.031147395556588967,
    0.0, -0.031649772153003713, 0.0, -0.032152148874473784, 0.0,
    -0.0326545256983305, 0.0, -0.03315690260601431, 0.0, -0.033659279582329935,
    0.0, -0.034161656614836548, 0.0, -0.034664033693348438, 0.0,
    -0.035166410809526275, 0.0, -0.03566878795654238, 0.0, -0.036171165128806713,
    0.0, -0.03667354232174256, 0.0, -0.0371759195316028, 0.0,
    -0.037678296755319571, 0.0, -0.0381806739903811, 0.0, -0.038683051234730932,
    0.0, -0.039185428486685395, 0.0, -0.039687805744865995, 0.0,
    -0.040190183008144122, 0.0, -0.040692560275595757, 0.0,
    -0.041194937546464368, 0.0, -0.041697314820130567, 0.0,
    -0.042199692096087232, 0.0, -0.042702069373919169, 0.0, -0.04320444665328646
  };

  static const double b_Kv[2020]{ -4.6835614255142838E-19, 0.0,
    -4.6231927182212708E-19, 0.0, -4.5631367485123293E-19, 0.0,
    -4.5017537518800707E-19, 0.0, -4.43902510213755E-19, 0.0,
    -4.3765695850884976E-19, 0.0, -4.312757700795144E-19, 0.0,
    -4.250847298260948E-19, 0.0, -4.1892300149738577E-19, 0.0,
    -4.1295500555829507E-19, 0.0, -4.0669337454901917E-19, 0.0,
    -4.0078956128731068E-19, 0.0, -3.9475808538755821E-19, 0.0,
    -3.8892544608812338E-19, 0.0, -3.831313919883029E-19, 0.0,
    -3.7721401526093725E-19, 0.0, -3.7117314693307335E-19, 0.0,
    -3.65172052224937E-19, 0.0, -3.5937572355460111E-19, 0.0,
    -3.5362406886506663E-19, 0.0, -3.4775531078876463E-19, 0.0,
    -3.4193279072760051E-19, 0.0, -3.3615815990835126E-19, 0.0,
    -3.3043307878325228E-19, 0.0, -3.2492261892128485E-19, 0.0,
    -3.1946675306873279E-19, 0.0, -3.1406716047804729E-19, 0.0,
    -3.0872552454694522E-19, 0.0, -3.0328012850342063E-19, 0.0,
    -2.9789436336082941E-19, 0.0, -2.9256992096436881E-19, 0.0,
    -2.873084950218245E-19, 0.0, -2.8194837720236059E-19, 0.0,
    -2.763261583032203E-19, 0.0, -2.7076693871414273E-19, 0.0,
    -2.6527241677552462E-19, 0.0, -2.598442915129708E-19, 0.0,
    -2.5448426251308733E-19, 0.0, -2.4903062625820956E-19, 0.0,
    -2.434833810498511E-19, 0.0, -2.38005929060988E-19, 0.0,
    -2.3276337483712111E-19, 0.0, -2.2743231455984588E-19, 0.0,
    -2.2201274746598439E-19, 0.0, -2.166680764942787E-19, 0.0,
    -2.1140000325360655E-19, 0.0, -2.060468258819996E-19, 0.0,
    -2.0077194760011998E-19, 0.0, -1.9541366668408306E-19, 0.0,
    -1.9013538646760429E-19, 0.0, -1.8477540531935918E-19, 0.0,
    -1.7933372308526591E-19, 0.0, -1.7413714676633138E-19, 0.0,
    -1.6869886980943638E-19, 0.0, -1.6318059357314798E-19, 0.0,
    -1.5790912511539139E-19, 0.0, -1.5256106149322739E-19, 0.0,
    -1.4713640266024862E-19, 0.0, -1.4179855214203901E-19, 0.0,
    -1.36549212027942E-19, 0.0, -1.3155348797651947E-19, 0.0,
    -1.264879770806642E-19, 0.0, -1.2135267932330553E-19, 0.0,
    -1.1631099825404634E-19, 0.0, -1.1152803954547404E-19, 0.0,
    -1.0684380386554384E-19, 0.0, -1.0193318619987963E-19, 0.0,
    -9.7448098676068393E-20, 0.0, -9.2903436959585488E-20, 0.0,
    -8.8462604609802556E-20, 0.0, -8.3963900180146857E-20, 0.0,
    -7.9570727231377646E-20, 0.0, -7.5448191445216461E-20, 0.0,
    -7.1272889933517737E-20, 0.0, -6.7044822694735133E-20, 0.0,
    -6.2927393291181853E-20, 0.0, -5.8922303842272588E-20, 0.0,
    -5.503125646761042E-20, 0.0, -5.1582760414110857E-20, 0.0,
    -4.8255114915401566E-20, 0.0, -4.4886618527891258E-20, 0.0,
    -4.1804078378359339E-20, 0.0, -3.86840915802397E-20, 0.0,
    -3.5526658133279333E-20, 0.0, -3.2658585164429323E-20, 0.0,
    -2.9756469787264192E-20, 0.0, -2.6820312001646011E-20, 0.0,
    -2.3850111807461868E-20, 0.0, -2.1009272768198437E-20, 0.0,
    -1.8299497004231228E-20, 0.0, -1.5722486635949932E-20, 0.0,
    -1.3443347347335002E-20, 0.0, -1.1138676252090195E-20, 0.0,
    -9.1352804773416718E-21, 0.0, -7.27316070038641E-21, 0.0,
    -5.7174226052341564E-21, 0.0, -4.3080668691917995E-21, 0.0,
    -2.8833920491189326E-21, 0.0, -1.7702052721640965E-21, 0.0,
    -8.0850721564258008E-22, 0.0, 0.0, 0.0, -4.6051903951020508E-19, 0.0,
    -4.5461151988550453E-19, 0.0, -4.487363505993318E-19, 0.0,
    -4.4273075362843754E-19, 0.0, -4.3659245396521169E-19, 0.0,
    -4.3048129043408822E-19, 0.0, -4.2423573872918294E-19, 0.0,
    -4.1817795318610467E-19, 0.0, -4.1214861437581359E-19, 0.0,
    -4.0631028893336149E-19, 0.0, -4.0018059155114232E-19, 0.0,
    -3.9440406487125193E-19, 0.0, -3.8850025160954344E-19, 0.0,
    -3.8279217859604805E-19, 0.0, -3.7712124073974169E-19, 0.0,
    -3.7132718663992121E-19, 0.0, -3.6540980991255551E-19, 0.0,
    -3.5953064302782018E-19, 0.0, -3.5385295120594093E-19, 0.0,
    -3.4821832397873349E-19, 0.0, -3.4246666928919896E-19, 0.0,
    -3.3675961265602553E-19, 0.0, -3.3109879403798992E-19, 0.0,
    -3.254858646618692E-19, 0.0, -3.2008418642302725E-19, 0.0,
    -3.1473542800418833E-19, 0.0, -3.0944126359476479E-19, 0.0,
    -3.0420337244720781E-19, 0.0, -2.9886173651610575E-19, 0.0,
    -2.9357804191570967E-19, 0.0, -2.8835397821624697E-19, 0.0,
    -2.8319123726291489E-19, 0.0, -2.7792981132037057E-19, 0.0,
    -2.7240799205777815E-19, 0.0, -2.6694747460176643E-19, 0.0,
    -2.6154995645581728E-19, 0.0, -2.5621713596032768E-19, 0.0,
    -2.5095071214090243E-19, 0.0, -2.4559068314101896E-19, 0.0,
    -2.4013704688614124E-19, 0.0, -2.3475150312091129E-19, 0.0,
    -2.2959745401830518E-19, 0.0, -2.2435489979443829E-19, 0.0,
    -2.19023839517163E-19, 0.0, -2.1376597386643006E-19, 0.0,
    -2.0858300433785289E-19, 0.0, -2.0331493109718072E-19, 0.0,
    -1.9812345516870231E-19, 0.0, -1.9284857688682267E-19, 0.0,
    -1.8765199741391429E-19, 0.0, -1.8237371719743552E-19, 0.0,
    -1.770137360491904E-19, 0.0, -1.7189545670135417E-19, 0.0,
    -1.6653717893929112E-19, 0.0, -1.6109890198239612E-19, 0.0,
    -1.5590402863236476E-19, 0.0, -1.5063256017460817E-19, 0.0,
    -1.4528449655244417E-19, 0.0, -1.4002153916259392E-19, 0.0,
    -1.3484539008751283E-19, 0.0, -1.2991945285967286E-19, 0.0,
    -1.2492372880825032E-19, 0.0, -1.1985821791239505E-19, 0.0,
    -1.148846215981649E-19, 0.0, -1.1016634341516272E-19, 0.0,
    -1.0554508614971895E-19, 0.0, -1.0069914902666023E-19, 0.0,
    -9.627363569038157E-20, 0.0, -9.1788548166570337E-20, 0.0,
    -8.7405587893215937E-20, 0.0, -8.2964755543433006E-20, 0.0,
    -7.8627752556905825E-20, 0.0, -7.4557982494393648E-20, 0.0,
    -7.0435446708232464E-20, 0.0, -6.626014519653374E-20, 0.0,
    -6.2193779400879654E-20, 0.0, -5.8238051440454891E-20, 0.0,
    -5.4394663434674144E-20, 0.0, -5.0988720389397516E-20, 0.0,
    -4.7701925779026471E-20, 0.0, -4.437428028031718E-20, 0.0,
    -4.1329186779063907E-20, 0.0, -3.8246646629531988E-20, 0.0,
    -3.5126659831412346E-20, 0.0, -3.2292629270709011E-20, 0.0,
    -2.9424556301859E-20, 0.0, -2.652244092469387E-20, 0.0,
    -2.3586283139075689E-20, 0.0, -2.0777784388020064E-20, 0.0,
    -1.8098646791885147E-20, 0.0, -1.5550572471046456E-20, 0.0,
    -1.3296964989022192E-20, 0.0, -1.1017825700407262E-20, 0.0,
    -9.0365574914194871E-21, 0.0, -7.19486315979948E-21, 0.0,
    -5.6561462691012485E-21, 0.0, -4.2621096170775124E-21, 0.0,
    -2.8527538810351551E-21, 0.0, -1.7514819472193204E-21, 0.0,
    -7.9999661339300032E-22, 0.0, 0.0, 0.0, -4.5269974876335473E-19, 0.0,
    -4.46919097082524E-19, 0.0, -4.4117157678050209E-19, 0.0,
    -4.3529640749432935E-19, 0.0, -4.292908105234352E-19, 0.0,
    -4.2331251018288793E-19, 0.0, -4.1720134665176442E-19, 0.0,
    -4.1127579359221632E-19, 0.0, -4.0537800737181669E-19, 0.0,
    -3.9966866720688279E-19, 0.0, -3.9367034244175215E-19, 0.0,
    -3.880206430275687E-19, 0.0, -3.8224411634767831E-19, 0.0,
    -3.76660301731327E-19, 0.0, -3.7111222804051021E-19, 0.0,
    -3.654412901842039E-19, 0.0, -3.5964723608438341E-19, 0.0,
    -3.5388985867969636E-19, 0.0, -3.4833069044031816E-19, 0.0,
    -3.428129979411175E-19, 0.0, -3.3717837071391011E-19, 0.0,
    -3.3158671534705417E-19, 0.0, -3.2603965803655933E-19, 0.0,
    -3.2053883874120222E-19, 0.0, -3.1524590801043877E-19, 0.0,
    -3.1000422909427551E-19, 0.0, -3.0481546999811514E-19, 0.0,
    -2.9968130491137019E-19, 0.0, -2.9444341376381321E-19, 0.0,
    -2.8926177715538979E-19, 0.0, -2.8413808187767225E-19, 0.0,
    -2.7907401750088809E-19, 0.0, -2.7391127654755611E-19, 0.0,
    -2.6848985128233311E-19, 0.0, -2.6312803134241932E-19, 0.0,
    -2.5782751320908624E-19, 0.0, -2.5258999438581568E-19, 0.0,
    -2.4741717321300468E-19, 0.0, -2.4215074939357942E-19, 0.0,
    -2.3679072039369596E-19, 0.0, -2.3149708346149677E-19, 0.0,
    -2.2643153834162406E-19, 0.0, -2.21277489239018E-19, 0.0,
    -2.1603493501515111E-19, 0.0, -2.1086387406055445E-19, 0.0,
    -2.0576600773250006E-19, 0.0, -2.0058303820392289E-19, 0.0,
    -1.9547496428592933E-19, 0.0, -1.902834883574509E-19, 0.0,
    -1.8516860939824987E-19, 0.0, -1.7997202992534152E-19, 0.0,
    -1.7469374970886272E-19, 0.0, -1.6965376720597479E-19, 0.0,
    -1.6437548853545997E-19, 0.0, -1.5901721077339692E-19, 0.0,
    -1.5389893246185913E-19, 0.0, -1.4870405911182777E-19, 0.0,
    -1.4343259065407117E-19, 0.0, -1.3824452635458577E-19, 0.0,
    -1.3314156828741411E-19, 0.0, -1.2828541785769022E-19, 0.0,
    -1.2335948062985025E-19, 0.0, -1.1836375657842771E-19, 0.0,
    -1.1345824500525106E-19, 0.0, -1.0880464733637809E-19, 0.0,
    -1.0424636847605451E-19, 0.0, -9.946511188793214E-20, 0.0,
    -9.5099172732909236E-20, 0.0, -9.0673659396630572E-20, 0.0,
    -8.6348571195497931E-20, 0.0, -8.1965610922143531E-20, 0.0,
    -7.76847778950392E-20, 0.0, -7.3667773553869229E-20, 0.0,
    -6.9598003491357052E-20, 0.0, -6.5475467705195868E-20, 0.0,
    -6.1460165516175735E-20, 0.0, -5.7553799043200252E-20, 0.0,
    -5.3758070405454087E-20, 0.0, -5.0394680367709143E-20, 0.0,
    -4.7148736645111114E-20, 0.0, -4.3861942034740068E-20, 0.0,
    -4.0854295181387978E-20, 0.0, -3.7809201680134705E-20, 0.0,
    -3.4726661530602786E-20, 0.0, -3.1926673377840345E-20, 0.0,
    -2.909264281713701E-20, 0.0, -2.6224569848287003E-20, 0.0,
    -2.3322454471121869E-20, 0.0, -2.0546296008182288E-20, 0.0,
    -1.7897796579805263E-20, 0.0, -1.5378658306348947E-20, 0.0,
    -1.3150582630867455E-20, 0.0, -1.0896975148843191E-20, 0.0,
    -8.9378345055854619E-21, 0.0, -7.1165656192762858E-21, 0.0,
    -5.59486993301348E-21, 0.0, -4.2161523649938482E-21, 0.0,
    -2.8221157129701121E-21, 0.0, -1.7327586222849548E-21, 0.0,
    -7.9148601114771986E-22, 0.0, 0.0, 0.0, -4.449001475808025E-19, 0.0,
    -4.3924448657384343E-19, 0.0, -4.3362213209524237E-19, 0.0,
    -4.2787461179322046E-19, 0.0, -4.2199944250704768E-19, 0.0,
    -4.1615214273838305E-19, 0.0, -4.1017384239783574E-19, 0.0,
    -4.0437927327117133E-19, 0.0, -3.9861201741385287E-19, 0.0,
    -3.9303082559791225E-19, 0.0, -3.8716318823074882E-19, 0.0,
    -3.8163975507230682E-19, 0.0, -3.7599005565812338E-19, 0.0,
    -3.7053012338269205E-19, 0.0, -3.6510460596857032E-19, 0.0,
    -3.5955653227775353E-19, 0.0, -3.5388559442144717E-19, 0.0,
    -3.4824983752385626E-19, 0.0, -3.4280905452362827E-19, 0.0,
    -3.374081834864796E-19, 0.0, -3.31890490987279E-19, 0.0,
    -3.2641416096230113E-19, 0.0, -3.2098080279767478E-19, 0.0,
    -3.1559204268940942E-19, 0.0, -3.1040781779851147E-19, 0.0,
    -3.052731842699775E-19, 0.0, -3.0018980255604372E-19, 0.0,
    -2.95159340662113E-19, 0.0, -2.9002517557536803E-19, 0.0,
    -2.8494558163004058E-19, 0.0, -2.7992224222384664E-19, 0.0,
    -2.7495684414835868E-19, 0.0, -2.6989277977157453E-19, 0.0,
    -2.6457174161601297E-19, 0.0, -2.5930861355301954E-19, 0.0,
    -2.5410509081533534E-19, 0.0, -2.4896286988423179E-19, 0.0,
    -2.4388364826319076E-19, 0.0, -2.3871082709037976E-19, 0.0,
    -2.334444032709545E-19, 0.0, -2.2824267147330057E-19, 0.0,
    -2.232656289455605E-19, 0.0, -2.1820008382568779E-19, 0.0,
    -2.1304603472308174E-19, 0.0, -2.0796177770144437E-19, 0.0,
    -2.0294901394907726E-19, 0.0, -1.9785114762102287E-19, 0.0,
    -1.9282647529467523E-19, 0.0, -1.8771840137668165E-19, 0.0,
    -1.8268522265043277E-19, 0.0, -1.7757034369123177E-19, 0.0,
    -1.7237376421832342E-19, 0.0, -1.6741207840630371E-19, 0.0,
    -1.6221379870118622E-19, 0.0, -1.569355200306714E-19, 0.0,
    -1.5189383667306744E-19, 0.0, -1.4677555836152964E-19, 0.0,
    -1.4158068501149828E-19, 0.0, -1.3646751375597125E-19, 0.0,
    -1.3143774665871537E-19, 0.0, -1.266513829960028E-19, 0.0,
    -1.2179523256627892E-19, 0.0, -1.1686929533843894E-19, 0.0,
    -1.1203186848924596E-19, 0.0, -1.0744295132052838E-19, 0.0,
    -1.0294765085388496E-19, 0.0, -9.8231074791331838E-20, 0.0,
    -9.39247098098981E-20, 0.0, -8.9558770654875191E-20, 0.0,
    -8.5291554520826072E-20, 0.0, -8.096646631969343E-20, 0.0,
    -7.6741803248568574E-20, 0.0, -7.2777564625923346E-20, 0.0,
    -6.8760560284753371E-20, 0.0, -6.46907902222412E-20, 0.0,
    -6.0726551638309554E-20, 0.0, -5.6869546651518965E-20, 0.0,
    -5.312147738077302E-20, 0.0, -4.9800640349715486E-20, 0.0,
    -4.6595547514200087E-20, 0.0, -4.3349603791602057E-20, 0.0,
    -4.03794035856901E-20, 0.0, -3.7371756732338009E-20, 0.0,
    -3.4326663231084736E-20, 0.0, -3.1560717486011904E-20, 0.0,
    -2.8760729333249463E-20, 0.0, -2.5926698772546131E-20, 0.0,
    -2.3058625803696124E-20, 0.0, -2.0314807628760534E-20, 0.0,
    -1.7696946368050497E-20, 0.0, -1.5206744141903016E-20, 0.0,
    -1.3004200272905789E-20, 0.0, -1.0776124597424296E-20, 0.0,
    -8.8391115198591187E-21, 0.0, -7.038268078830935E-21, 0.0,
    -5.5335935969808464E-21, 0.0, -4.1701951129475848E-21, 0.0,
    -2.7914775449279527E-21, 0.0, -1.7140352973633045E-21, 0.0,
    -7.8297540890769085E-22, 0.0, 0.0, 0.0, -4.3712215201104956E-19, 0.0,
    -4.3158956562937175E-19, 0.0, -4.2609049970419426E-19, 0.0,
    -4.2046814522559315E-19, 0.0, -4.1472062492357124E-19, 0.0,
    -4.0900205071918E-19, 0.0, -4.0315475095051532E-19, 0.0,
    -3.9748964077353121E-19, 0.0, -3.9185166672864829E-19, 0.0,
    -3.8639760103489289E-19, 0.0, -3.8065981413717074E-19, 0.0,
    -3.752619620153519E-19, 0.0, -3.697385288569099E-19, 0.0,
    -3.6440201960628957E-19, 0.0, -3.5909868241263977E-19, 0.0,
    -3.53673164998518E-19, 0.0, -3.4812509130770125E-19, 0.0,
    -3.4261074853317642E-19, 0.0, -3.3728818179914858E-19, 0.0,
    -3.3200399388070212E-19, 0.0, -3.2660312284355345E-19, 0.0,
    -3.2124202542613433E-19, 0.0, -3.1592229048293814E-19, 0.0,
    -3.1064552740009322E-19, 0.0, -3.05569957455391E-19, 0.0,
    -3.005423276462746E-19, 0.0, -2.9556428919952212E-19, 0.0,
    -2.9063750256736987E-19, 0.0, -2.8560704067343913E-19, 0.0,
    -2.8062947066847571E-19, 0.0, -2.7570647180492984E-19, 0.0,
    -2.7083972748051743E-19, 0.0, -2.6587432940502953E-19, 0.0,
    -2.6065366994646379E-19, 0.0, -2.5548922687268376E-19, 0.0,
    -2.5038269389147192E-19, 0.0, -2.4533576623556924E-19, 0.0,
    -2.4035014038624717E-19, 0.0, -2.3527091876520619E-19, 0.0,
    -2.3009809759239519E-19, 0.0, -2.2498826885475142E-19, 0.0,
    -2.2009972722066055E-19, 0.0, -2.1512268469292053E-19, 0.0,
    -2.1005713957304779E-19, 0.0, -2.0505968555222328E-19, 0.0,
    -2.0013202361236743E-19, 0.0, -1.9511925986000032E-19, 0.0,
    -1.901779886137275E-19, 0.0, -1.8515331628737985E-19, 0.0,
    -1.8020183745116785E-19, 0.0, -1.7516865872491898E-19, 0.0,
    -1.7005377976571795E-19, 0.0, -1.6517039045637266E-19, 0.0,
    -1.6005210956257142E-19, 0.0, -1.5485382985745393E-19, 0.0,
    -1.4988874135050217E-19, 0.0, -1.4484705799289821E-19, 0.0,
    -1.3972877968136042E-19, 0.0, -1.3469050141311061E-19, 0.0,
    -1.2973392523936511E-19, 0.0, -1.2501734830567232E-19, 0.0,
    -1.2023098464295975E-19, 0.0, -1.1537483421323586E-19, 0.0,
    -1.1060549206717742E-19, 0.0, -1.0608125538154751E-19, 0.0,
    -1.0164893329461147E-19, 0.0, -9.699703774618651E-20, 0.0,
    -9.2750246928978E-20, 0.0, -8.8443881947544255E-20, 0.0,
    -8.42345378743029E-20, 0.0, -7.9967321740253775E-20, 0.0,
    -7.5798828620902677E-20, 0.0, -7.18873557133409E-20, 0.0,
    -6.7923117090695667E-20, 0.0, -6.3906112749525693E-20, 0.0,
    -5.9992937768795048E-20, 0.0, -5.618529426664495E-20, 0.0,
    -5.24848843616359E-20, 0.0, -4.9206600336234566E-20, 0.0,
    -4.604235838695857E-20, 0.0, -4.283726555144317E-20, 0.0,
    -3.9904511992408215E-20, 0.0, -3.6934311786496257E-20, 0.0,
    -3.3926664933144167E-20, 0.0, -3.1194761595453975E-20, 0.0,
    -2.8428815850381143E-20, 0.0, -2.5628827697618703E-20, 0.0,
    -2.2794797136915367E-20, 0.0, -2.0083319249846898E-20, 0.0,
    -1.7496096156692848E-20, 0.0, -1.5034829977764348E-20, 0.0,
    -1.2857817915179942E-20, 0.0, -1.0655274046182717E-20, 0.0,
    -8.7403885342643E-21, 0.0, -6.95997053848066E-21, 0.0,
    -5.4723172610155531E-21, 0.0, -4.1242378609470027E-21, 0.0,
    -2.7608393769137411E-21, 0.0, -1.6953119724571854E-21, 0.0,
    -7.7446480667407562E-22, 0.0, 0.0, 0.0, -4.2936754331649519E-19, 0.0,
    -4.2395625029759026E-19, 0.0, -4.1857855687724729E-19, 0.0,
    -4.130794909520698E-19, 0.0, -4.0745713647346869E-19, 0.0,
    -4.0186450913278161E-19, 0.0, -3.9614593492839041E-19, 0.0,
    -3.9060842108239535E-19, 0.0, -3.8509820386674603E-19, 0.0,
    -3.7977001574453276E-19, 0.0, -3.7416105708944243E-19, 0.0,
    -3.6888794907572478E-19, 0.0, -3.63490096953906E-19, 0.0,
    -3.5827644971813361E-19, 0.0, -3.5309483342884806E-19, 0.0,
    -3.4779149623519831E-19, 0.0, -3.4236597882107653E-19, 0.0,
    -3.3697279809159452E-19, 0.0, -3.3176824123973936E-19, 0.0,
    -3.2660056746704634E-19, 0.0, -3.2131637954859988E-19, 0.0,
    -3.1607040147278605E-19, 0.0, -3.1086419701670171E-19, 0.0,
    -3.056993550348403E-19, 0.0, -3.0073237787466509E-19, 0.0,
    -2.9581170089129763E-19, 0.0, -2.9093896404351604E-19, 0.0,
    -2.8611581855809839E-19, 0.0, -2.8118903192594609E-19, 0.0,
    -2.7631346299335023E-19, 0.0, -2.7149078594972164E-19, 0.0,
    -2.6672268004751055E-19, 0.0, -2.6185593572309819E-19, 0.0,
    -2.5673564468627541E-19, 0.0, -2.516698781890445E-19, 0.0,
    -2.466603280765993E-19, 0.0, -2.4170868805672224E-19, 0.0,
    -2.3681665336215435E-19, 0.0, -2.3183102751283233E-19, 0.0,
    -2.2675180589179135E-19, 0.0, -2.2173387768031518E-19, 0.0,
    -2.1693383486534107E-19, 0.0, -2.1204529323125015E-19, 0.0,
    -2.0706825070351013E-19, 0.0, -2.0215759854497223E-19, 0.0,
    -1.9731503748548252E-19, 0.0, -1.9238737554562667E-19, 0.0,
    -1.875295047545944E-19, 0.0, -1.8258823350832157E-19, 0.0,
    -1.7771845414330873E-19, 0.0, -1.7276697530709673E-19, 0.0,
    -1.6773379658084785E-19, 0.0, -1.6292870354431647E-19, 0.0,
    -1.5789042127363634E-19, 0.0, -1.527721403798351E-19, 0.0,
    -1.4788364659738726E-19, 0.0, -1.429185580904355E-19, 0.0,
    -1.3787687473283153E-19, 0.0, -1.3291348938262855E-19, 0.0,
    -1.2803010407571357E-19, 0.0, -1.2338331382463768E-19, 0.0,
    -1.186667368909449E-19, 0.0, -1.1388037322823232E-19, 0.0,
    -1.0917911575984327E-19, 0.0, -1.0471955953645444E-19, 0.0,
    -1.0035021581215935E-19, 0.0, -9.5763000763888489E-20, 0.0,
    -9.1575784099467975E-20, 0.0, -8.7328993282259463E-20, 0.0,
    -8.317752126216054E-20, 0.0, -7.8968177188919179E-20, 0.0,
    -7.4855854016204878E-20, 0.0, -7.0997146819523407E-20, 0.0,
    -6.7085673911961625E-20, 0.0, -6.31214352893164E-20, 0.0,
    -5.9259323909481241E-20, 0.0, -5.55010418900854E-20, 0.0,
    -5.1848291349270131E-20, 0.0, -4.8612560328265519E-20, 0.0,
    -4.5489169264199005E-20, 0.0, -4.2324927314923009E-20, 0.0,
    -3.9429620402077235E-20, 0.0, -3.649686684304228E-20, 0.0,
    -3.3526666637130323E-20, 0.0, -3.0828805706447864E-20, 0.0,
    -2.8096902368757672E-20, 0.0, -2.5330956623684841E-20, 0.0,
    -2.25309684709224E-20, 0.0, -1.985183087155388E-20, 0.0,
    -1.7295245945820224E-20, 0.0, -1.486291581400099E-20, 0.0,
    -1.2711435557742123E-20, 0.0, -1.0534423495157715E-20, 0.0,
    -8.64166554883012E-21, 0.0, -6.881672998246518E-21, 0.0,
    -5.41104092513251E-21, 0.0, -4.078280609002217E-21, 0.0,
    -2.7302012089336666E-21, 0.0, -1.6765886475700353E-21, 0.0,
    -7.6595420444829454E-22, 0.0, 0.0, 0.0, -4.2163810772834375E-19, 0.0,
    -4.1634632184087412E-19, 0.0, -4.1108821966285891E-19, 0.0,
    -4.0571052624251594E-19, 0.0, -4.002114603173385E-19, 0.0,
    -3.9474229667962696E-19, 0.0, -3.8914966933893993E-19, 0.0,
    -3.83737476816328E-19, 0.0, -3.7835315381122259E-19, 0.0,
    -3.7314931827735266E-19, 0.0, -3.6766793931424968E-19, 0.0,
    -3.6251855318182836E-19, 0.0, -3.5724544516811076E-19, 0.0,
    -3.5215397472807122E-19, 0.0, -3.470935183331885E-19, 0.0,
    -3.4191190204390295E-19, 0.0, -3.3660856485025315E-19, 0.0,
    -3.3133623827702105E-19, 0.0, -3.2624943922931843E-19, 0.0,
    -3.2119807321835289E-19, 0.0, -3.1603039944565988E-19, 0.0,
    -3.1089940236810304E-19, 0.0, -3.0580661513317892E-19, 0.0,
    -3.007536015179842E-19, 0.0, -2.9589514121790215E-19, 0.0,
    -2.9108135489861656E-19, 0.0, -2.8631386875613877E-19, 0.0,
    -2.815943227492468E-19, 0.0, -2.767711772638292E-19, 0.0,
    -2.7199758147256662E-19, 0.0, -2.6727520338086029E-19, 0.0,
    -2.6260571717812142E-19, 0.0, -2.5783761127591033E-19, 0.0,
    -2.5281767611060825E-19, 0.0, -2.4785057591467518E-19, 0.0,
    -2.4293800025833395E-19, 0.0, -2.3808164098677842E-19, 0.0,
    -2.33283191807791E-19, 0.0, -2.2839115711322309E-19, 0.0,
    -2.2340553126390107E-19, 0.0, -2.1847950048374976E-19, 0.0,
    -2.1376795395405289E-19, 0.0, -2.0896791113907878E-19, 0.0,
    -2.0407936950498788E-19, 0.0, -1.9925551781813756E-19, 0.0,
    -1.9449805650048928E-19, 0.0, -1.8965549544099957E-19, 0.0,
    -1.8488102434203342E-19, 0.0, -1.8002315355100109E-19, 0.0,
    -1.7523507314561794E-19, 0.0, -1.703652937806051E-19, 0.0,
    -1.654138149443931E-19, 0.0, -1.6068701789992354E-19, 0.0,
    -1.5572873402250251E-19, 0.0, -1.5069045175182238E-19, 0.0,
    -1.4587855253980046E-19, 0.0, -1.4099005875735262E-19, 0.0,
    -1.3602497025040086E-19, 0.0, -1.3113647773368654E-19, 0.0,
    -1.2632628322437323E-19, 0.0, -1.2174927959923755E-19, 0.0,
    -1.1710248934816165E-19, 0.0, -1.1238591241446887E-19, 0.0,
    -1.0775273959264596E-19, 0.0, -1.0335786380603622E-19, 0.0,
    -9.9051498423537066E-20, 0.0, -9.4528963858352318E-20, 0.0,
    -9.0401321332750432E-20, 0.0, -8.6214104668329918E-20, 0.0,
    -8.2120504692011065E-20, 0.0, -7.7969032671912141E-20, 0.0,
    -7.391287943956044E-20, 0.0, -7.0106937948625447E-20, 0.0,
    -6.6248230751943975E-20, 0.0, -6.2336757844382194E-20, 0.0,
    -5.8525710062626625E-20, 0.0, -5.4816789523681129E-20, 0.0,
    -5.121169834517495E-20, 0.0, -4.8018520327028656E-20, 0.0,
    -4.4935980146913704E-20, 0.0, -4.181258908284719E-20, 0.0,
    -3.8954728815350513E-20, 0.0, -3.6059421902504739E-20, 0.0,
    -3.3126668343469784E-20, 0.0, -3.0462849819337145E-20, 0.0,
    -2.7764988888654687E-20, 0.0, -2.5033085550964495E-20, 0.0,
    -2.2267139805891663E-20, 0.0, -1.9620342494018882E-20, 0.0,
    -1.7094395735540019E-20, 0.0, -1.4691001650696022E-20, 0.0,
    -1.2565053200656106E-20, 0.0, -1.041357294439724E-20, 0.0,
    -8.54294256359215E-21, 0.0, -6.8033754581542127E-21, 0.0,
    -5.3497645893499285E-21, 0.0, -4.03232335712558E-21, 0.0,
    -2.699563040995287E-21, 0.0, -1.6578653227060544E-21, 0.0,
    -7.574436022320819E-22, 0.0, 0.0, 0.0, -4.1393584480395375E-19, 0.0,
    -4.0876156649039837E-19, 0.0, -4.0362126932337517E-19, 0.0,
    -3.9836316714535992E-19, 0.0, -3.92985473725017E-19, 0.0,
    -3.8763789652028593E-19, 0.0, -3.8216873288257445E-19, 0.0,
    -3.768790829827802E-19, 0.0, -3.7161837918061473E-19, 0.0,
    -3.6653703361640217E-19, 0.0, -3.6118170936208575E-19, 0.0,
    -3.5615479656032204E-19, 0.0, -3.5100541042790077E-19, 0.0,
    -3.4603527985507594E-19, 0.0, -3.4109529813548274E-19, 0.0,
    -3.3603484174060017E-19, 0.0, -3.3085322545131462E-19, 0.0,
    -3.2570137697811122E-19, 0.0, -3.2073202784577194E-19, 0.0,
    -3.1579671751851566E-19, 0.0, -3.1074535150755017E-19, 0.0,
    -3.0572916645530355E-19, 0.0, -3.0074965809819319E-19, 0.0,
    -2.9580835958371542E-19, 0.0, -2.9105832340941358E-19, 0.0,
    -2.8635135182977796E-19, 0.0, -2.8168905423093876E-19, 0.0,
    -2.7707305680890736E-19, 0.0, -2.7235351080201544E-19, 0.0,
    -2.6768185403704423E-19, 0.0, -2.6305974696622804E-19, 0.0,
    -2.5848885759496819E-19, 0.0, -2.5381937139222932E-19, 0.0,
    -2.488997767695717E-19, 0.0, -2.4403133032471611E-19, 0.0,
    -2.3921571884922943E-19, 0.0, -2.3445463191333468E-19, 0.0,
    -2.2974976136222554E-19, 0.0, -2.249513121832381E-19, 0.0,
    -2.2005927748867021E-19, 0.0, -2.1522514035979458E-19, 0.0,
    -2.106020870205361E-19, 0.0, -2.0589054049083923E-19, 0.0,
    -2.0109049767586512E-19, 0.0, -1.9635344476222066E-19, 0.0,
    -1.9168108179581675E-19, 0.0, -1.8692362047816847E-19, 0.0,
    -1.8223254813912517E-19, 0.0, -1.77458077040159E-19, 0.0,
    -1.7275169496957311E-19, 0.0, -1.6796361456418996E-19, 0.0,
    -1.6309383519917712E-19, 0.0, -1.5844533380385795E-19, 0.0,
    -1.5356704803894197E-19, 0.0, -1.4860876416152094E-19, 0.0,
    -1.4387345933173364E-19, 0.0, -1.3906156011971172E-19, 0.0,
    -1.3417306633726388E-19, 0.0, -1.2935946655075853E-19, 0.0,
    -1.2462246275449063E-19, 0.0, -1.2011524568607017E-19, 0.0,
    -1.1553824206093449E-19, 0.0, -1.1089145180985859E-19, 0.0,
    -1.0632636359661221E-19, 0.0, -1.0199616821568214E-19, 0.0,
    -9.7752781149518816E-20, 0.0, -9.3294927046573246E-20, 0.0,
    -8.9226858642727739E-20, 0.0, -8.5099216117125853E-20, 0.0,
    -8.1063488173151765E-20, 0.0, -7.6969888196832912E-20, 0.0,
    -7.29699048971804E-20, 0.0, -6.9216729105721529E-20, 0.0,
    -6.5410787614786535E-20, 0.0, -6.1552080418105063E-20, 0.0,
    -5.7792096230989708E-20, 0.0, -5.4132537169680547E-20, 0.0,
    -5.0575105351181465E-20, 0.0, -4.7424480334014539E-20, 0.0,
    -4.4382791036314665E-20, 0.0, -4.1300250856199713E-20, 0.0,
    -3.8479837233026033E-20, 0.0, -3.5621976965529355E-20, 0.0,
    -3.2726670052683581E-20, 0.0, -3.0096893934541454E-20, 0.0,
    -2.7433075410408815E-20, 0.0, -2.4735214479726357E-20, 0.0,
    -2.2003311142036168E-20, 0.0, -1.9388854117409753E-20, 0.0,
    -1.6893545525983389E-20, 0.0, -1.4519087487950939E-20, 0.0,
    -1.2418670843999776E-20, 0.0, -1.029272239395986E-20, 0.0,
    -8.4442195785938273E-21, 0.0, -6.7250779182351524E-21, 0.0,
    -5.2884882536900493E-21, 0.0, -3.9863661053321812E-21, 0.0,
    -2.6689248731078328E-21, 0.0, -1.6391419978703723E-21, 0.0,
    -7.4893300002755611E-22, 0.0, 0.0, 0.0, -4.0626263780367226E-19, 0.0,
    -4.0120398380348528E-19, 0.0, -3.9617949208993537E-19, 0.0,
    -3.9103919492291222E-19, 0.0, -3.8578109274489697E-19, 0.0,
    -3.805531859245595E-19, 0.0, -3.7520560871982845E-19, 0.0,
    -3.70036018282128E-19, 0.0, -3.6489615498233931E-19, 0.0,
    -3.5993502438018482E-19, 0.0, -3.547038922159668E-19, 0.0,
    -3.4979792776166688E-19, 0.0, -3.4477101495990316E-19, 0.0,
    -3.3992120202749293E-19, 0.0, -3.3510085805467361E-19, 0.0,
    -3.3016087633508046E-19, 0.0, -3.2510041994019784E-19, 0.0,
    -3.2006859025091771E-19, 0.0, -3.1521631497772543E-19, 0.0,
    -3.1039675244539167E-19, 0.0, -3.0546144211813544E-19, 0.0,
    -3.0055986270717546E-19, 0.0, -2.9569346425493436E-19, 0.0,
    -2.9086374249782942E-19, 0.0, -2.8622201718336263E-19, 0.0,
    -2.816217676090664E-19, 0.0, -2.7706458262943625E-19, 0.0,
    -2.7255207163060256E-19, 0.0, -2.6793607420857116E-19, 0.0,
    -2.6336631480168471E-19, 0.0, -2.5884444463671911E-19, 0.0,
    -2.5437212416590839E-19, 0.0, -2.4980123479464854E-19, 0.0,
    -2.4498196199190411E-19, 0.0, -2.402121539692521E-19, 0.0,
    -2.3549349412440197E-19, 0.0, -2.3082766924892086E-19, 0.0,
    -2.2621636891303153E-19, 0.0, -2.2151149836192244E-19, 0.0,
    -2.16713049182935E-19, 0.0, -2.1197080108837262E-19, 0.0,
    -2.07436237159508E-19, 0.0, -2.0281318382024952E-19, 0.0,
    -1.9810163729055262E-19, 0.0, -1.9345138107558405E-19, 0.0,
    -1.8886411476194511E-19, 0.0, -1.8419175179554115E-19, 0.0,
    -1.7958407707789841E-19, 0.0, -1.7489300473885511E-19, 0.0,
    -1.7026832023989446E-19, 0.0, -1.6556193816930859E-19, 0.0,
    -1.6077385776392541E-19, 0.0, -1.5620365159892361E-19, 0.0,
    -1.5140536360359892E-19, 0.0, -1.4652707783868295E-19, 0.0,
    -1.4186836716127294E-19, 0.0, -1.3713306233148565E-19, 0.0,
    -1.3232116311946373E-19, 0.0, -1.2758245593702137E-19, 0.0,
    -1.2291864275052154E-19, 0.0, -1.1848121215426467E-19, 0.0,
    -1.1397399508584421E-19, 0.0, -1.0939699146070853E-19, 0.0,
    -1.0489998780963815E-19, 0.0, -1.0063447279640278E-19, 0.0,
    -9.6454064015478225E-20, 0.0, -9.206089034930939E-20, 0.0,
    -8.8052396046380354E-20, 0.0, -8.3984327642534848E-20, 0.0,
    -8.0006471716938476E-20, 0.0, -7.5970743772964389E-20, 0.0,
    -7.2026930396651039E-20, 0.0, -6.832652029700956E-20, 0.0,
    -6.4573344505550686E-20, 0.0, -6.0767403014615693E-20, 0.0,
    -5.7058482417939724E-20, 0.0, -5.3448284830829878E-20, 0.0,
    -4.9938512369526232E-20, 0.0, -4.6830440351043684E-20, 0.0,
    -4.3829601933882268E-20, 0.0, -4.0787912636182394E-20, 0.0,
    -3.8004945656078466E-20, 0.0, -3.5184532032904786E-20, 0.0,
    -3.2326671765408108E-20, 0.0, -2.9730938052573359E-20, 0.0,
    -2.7101161934431229E-20, 0.0, -2.443734341029859E-20, 0.0,
    -2.1739482479616132E-20, 0.0, -1.9157365741931455E-20, 0.0,
    -1.6692695317310549E-20, 0.0, -1.4347173325889697E-20, 0.0,
    -1.2272288487868271E-20, 0.0, -1.0171871843917108E-20, 0.0,
    -8.3454965938882135E-21, 0.0, -6.6467803785276924E-21, 0.0,
    -5.22721191818004E-21, 0.0, -3.9404088536404483E-21, 0.0,
    -2.63828670528258E-21, 0.0, -1.6204186730692543E-21, 0.0,
    -7.4042239783730487E-22, 0.0, 0.0, 0.0, -3.9862021538891742E-19, 0.0,
    -3.9367545704043805E-19, 0.0, -3.8876488751981856E-19, 0.0,
    -3.8374039580626869E-19, 0.0, -3.7860009863924549E-19, 0.0,
    -3.7349008094079772E-19, 0.0, -3.6826217412046026E-19, 0.0,
    -3.6321076587486412E-19, 0.0, -3.5818925991673115E-19, 0.0,
    -3.5334556557607738E-19, 0.0, -3.4823635049435541E-19, 0.0,
    -3.4344947176883973E-19, 0.0, -3.3854350731453986E-19, 0.0,
    -3.3381276347191106E-19, 0.0, -3.2911103501906821E-19, 0.0,
    -3.242906910462489E-19, 0.0, -3.1935070932665579E-19, 0.0,
    -3.1443833741134061E-19, 0.0, -3.0970267668119549E-19, 0.0,
    -3.0499848588757064E-19, 0.0, -3.0017892335523688E-19, 0.0,
    -2.9539169750754809E-19, 0.0, -2.9063820257615559E-19, 0.0,
    -2.8591988860348186E-19, 0.0, -2.8138633580551194E-19, 0.0,
    -2.7689269497061263E-19, 0.0, -2.7244052987588384E-19, 0.0,
    -2.6803142937582112E-19, 0.0, -2.6351891837698743E-19, 0.0,
    -2.5905100543452351E-19, 0.0, -2.5462933050720454E-19, 0.0,
    -2.5025554482180638E-19, 0.0, -2.4578322435099566E-19, 0.0,
    -2.4106425050016833E-19, 0.0, -2.3639306217699138E-19, 0.0,
    -2.317713386339068E-19, 0.0, -2.2720076326862415E-19, 0.0,
    -2.2268302287271047E-19, 0.0, -2.1807172253682119E-19, 0.0,
    -2.1336685198571205E-19, 0.0, -2.087164872862921E-19, 0.0,
    -2.0427040815086464E-19, 0.0, -1.9973584422200004E-19, 0.0,
    -1.9511279088274156E-19, 0.0, -1.9054932883261214E-19, 0.0,
    -1.86047157097211E-19, 0.0, -1.8145989078357206E-19, 0.0,
    -1.7693561229673556E-19, 0.0, -1.7232793757909282E-19, 0.0,
    -1.6778494971961698E-19, 0.0, -1.6316026522065635E-19, 0.0,
    -1.5845388315007046E-19, 0.0, -1.5396197170382222E-19, 0.0,
    -1.4924368105925293E-19, 0.0, -1.4444539306392824E-19, 0.0,
    -1.3986327625814721E-19, 0.0, -1.3520456558073721E-19, 0.0,
    -1.3046926075094991E-19, 0.0, -1.2580544601849545E-19, 0.0,
    -1.2121482331562055E-19, 0.0, -1.1684717908825563E-19, 0.0,
    -1.1240974849199876E-19, 0.0, -1.0790253142357831E-19, 0.0,
    -1.0347361227801008E-19, 0.0, -9.9272777586074628E-20, 0.0,
    -9.5155347052406733E-20, 0.0, -9.08268537919147E-20, 0.0,
    -8.6877933564448253E-20, 0.0, -8.2869439261519217E-20, 0.0,
    -7.8949455337241181E-20, 0.0, -7.4971599411644809E-20, 0.0,
    -7.1083955947238179E-20, 0.0, -6.7436311530059745E-20, 0.0,
    -6.3735901430418266E-20, 0.0, -5.9982725638959392E-20, 0.0,
    -5.6324868627591868E-20, 0.0, -5.2764032510483357E-20, 0.0,
    -4.9301919402940969E-20, 0.0, -4.6236400380339708E-20, 0.0,
    -4.3276412841424623E-20, 0.0, -4.0275574424263207E-20, 0.0,
    -3.7530054085698254E-20, 0.0, -3.4747087105594326E-20, 0.0,
    -3.1926673482420646E-20, 0.0, -2.936498217405889E-20, 0.0,
    -2.6769248461224141E-20, 0.0, -2.4139472343082011E-20, 0.0,
    -2.1475653818949372E-20, 0.0, -1.8925877367834374E-20, 0.0,
    -1.6491845109717158E-20, 0.0, -1.4175259164663713E-20, 0.0,
    -1.2125906132377784E-20, 0.0, -1.0051021294356356E-20, 0.0,
    -8.2467736095401154E-21, 0.0, -6.5684828390786823E-21, 0.0,
    -5.1659355828530824E-21, 0.0, -3.8944516020728913E-21, 0.0,
    -2.6076485375332997E-21, 0.0, -1.6016953483103532E-21, 0.0,
    -7.3191179566448846E-22, 0.0, 0.0, 0.0 };

  static const double b_Kx[40]{ 6.6192845073638257E-5, -0.060502432254616163,
    -2.3870855101720111E-5, 2.2971619784827082, 5.4194117046534042E-5,
    -0.059250745344268232, -2.3377009883334619E-5, 2.2494361362788489,
    4.4370389421862064E-5, -0.058011209736655196, -2.2887958885215509E-5,
    2.2022126713601442, 3.6327401514618333E-5, -0.056784022562097518,
    -2.2403779883894113E-5, 2.155491583726342, 2.9742359977260522E-5,
    -0.055569345217248239, -2.1924536557407393E-5, 2.1092728733771313,
    2.4350983969266688E-5, -0.054367309842507337, -2.1450281040921244E-5,
    2.0635565403121348, 1.9936898639052237E-5, -0.05317802462528004,
    -2.0981056019097687E-5, 2.0183425845308882, 1.6322951241432205E-5,
    -0.0520015781419171, -2.051689643918027E-5, 1.9736310060328277,
    1.3364101375931429E-5, -0.050838042912595, -2.0057830913549677E-5,
    1.9294218048172631, 1.0941600006242196E-5, -0.049687478311805033,
    -1.960388286803899E-5, 1.8857149808833529 };

  static const double b_Ku1[10]{ 2.9180500944887964, 2.8722458870982677,
    2.8264723611167111, 2.7807358238040729, 2.7350432604188808,
    2.6894022113179057, 2.6438206713338652, 2.5983070073928456,
    2.5528698910651295, 2.5075182433424761 };

  if ((&rtM)->isMajorTimeStep()) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if ((&rtM)->isMinorTimeStep()) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  // Outputs for Atomic SubSystem: '<Root>/BMS'
  rtDW.b_j = ((&rtM)->isMajorTimeStep());
  if (rtDW.b_j) {
    // DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
    //   UnitDelay: '<S1>/SOC_t-1'

    rtDW.DiscreteTimeIntegrator = rtDW.SOC_t1_DSTATE;

    // Lookup_n-D: '<S1>/SOC-Temp' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtDW.SOCTemp = look1_binlg(rtDW.DiscreteTimeIntegrator,
      rtConstP.SOCTemp_bp01Data, rtConstP.SOCTemp_tableData, 3987U);

    // Lookup_n-D: '<S1>/SOC-Vol' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtDW.Vsoc = look1_binlag(rtDW.DiscreteTimeIntegrator,
      rtConstP.SOCVol_bp01Data, rtConstP.SOCVol_tableData, 100U);
  }

  // FromWorkspace: '<S1>/From Workspace19'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace19_PWORK.DataPtr };

    rtDW.SpeedDifferent = pDataValues[0];
  }

  // Sum: '<S1>/Add13' incorporates:
  //   Constant: '<S1>/Constant8'

  rtDW.Add13 = 1.0 - rtDW.SpeedDifferent;
  if (rtDW.b_j) {
    // Gain: '<S1>/Gain1' incorporates:
    //   Inport: '<Root>/vel_ego'

    rtDW.Gain1 = 3.6 * rtU.vel_ego;
  }

  // Sum: '<S1>/Speed Different' incorporates:
  //   Inport: '<Root>/Target speed'

  rtDW.SpeedDifferent = rtU.Targetspeed - rtDW.Gain1;

  // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
  //   Integrator: '<S162>/Filter'

  rtDW.rtb_uDLookupTable1_c = rtX.Filter_CSTATE;

  // Gain: '<S170>/Filter Coefficient' incorporates:
  //   Gain: '<S160>/Derivative Gain'
  //   Integrator: '<S162>/Filter'
  //   Sum: '<S162>/SumD'

  rtDW.FilterCoefficient = ((0.0001 * rtDW.SpeedDifferent) - rtX.Filter_CSTATE) *
    10.0;
  if (rtDW.b_j) {
    // Outputs for Atomic SubSystem: '<S1>/ACC system'
    // MATLAB Function: '<S11>/DataTypeConversion_L0' incorporates:
    //   Constant: '<S11>/Default spacing constant'

    DataTypeConversion_L0(20.0, &rtDW.rtb_y_cs_g);

    // MATLAB Function: '<S11>/DataTypeConversion_amax' incorporates:
    //   Constant: '<S11>/Maximum longitudinal acceleration constant'

    DataTypeConversion_L0(3.0, &rtDW.rtb_y_h_p);

    // MATLAB Function: '<S11>/DataTypeConversion_amin' incorporates:
    //   Constant: '<S11>/Minimum longitudinal acceleration constant'

    DataTypeConversion_L0(-3.0, &rtDW.rtb_y_d4_b);

    // SignalConversion generated from: '<S45>/ SFunction ' incorporates:
    //   Constant: '<S11>/Default spacing constant'
    //   Constant: '<S3>/t_gap(s)'
    //   Inport: '<Root>/vel_ego'
    //   MATLAB Function: '<S11>/DataTypeConversion_dmin'
    //   MATLAB Function: '<S44>/optimizer'
    //   Product: '<S11>/Product2'
    //   Sum: '<S11>/Sum1'

    DataTypeConversion_L0((rtU.vel_ego * 0.5) + 20.0,
                          &rtDW.TmpSignalConversionAtSFun_a[0]);

    // Sum: '<S3>/Sum' incorporates:
    //   Inport: '<Root>/pos_ego'
    //   Inport: '<Root>/pos_front'

    rtDW.rtb_Sum_f_c = rtU.pos_front - rtU.pos_ego;

    // MATLAB Function: '<S11>/DataTypeConversion_reldist'
    DataTypeConversion_L0(rtDW.rtb_Sum_f_c, &rtDW.rtb_y_g_f);

    // MATLAB Function: '<S11>/DataTypeConversion_vego' incorporates:
    //   Inport: '<Root>/vel_ego'

    DataTypeConversion_L0(rtU.vel_ego, &rtDW.rtb_y_n_g);

    // MATLAB Function: '<S11>/DataTypeConversion_vlead' incorporates:
    //   Inport: '<Root>/vel_ego'
    //   Inport: '<Root>/vel_front'
    //   Sum: '<S11>/Sum6'
    //   Sum: '<S3>/Sum1'

    DataTypeConversion_L0(rtU.vel_ego + (rtU.vel_front - rtU.vel_ego),
                          &rtDW.DiscreteTimeIntegrator);

    // MATLAB Function: '<S11>/DataTypeConversion_vset' incorporates:
    //   Constant: '<S3>/velocity(m//s)'

    DataTypeConversion_L0(30.0, &rtDW.rtb_y_m);

    // SignalConversion generated from: '<S45>/ SFunction ' incorporates:
    //   Constant: '<S11>/Minimum velocity constant'
    //   MATLAB Function: '<S44>/optimizer'

    rtDW.TmpSignalConversionAtSFun_a[1] = 0.0;

    // SignalConversion generated from: '<S45>/ SFunction ' incorporates:
    //   Constant: '<S11>/Maximum velocity constant'
    //   Constant: '<S11>/Unconstrained'
    //   MATLAB Function: '<S44>/optimizer'

    rtDW.TmpSignalConversionAtSFun_m[0] = 0;
    rtDW.TmpSignalConversionAtSFun_m[1] = 30;

    // MATLAB Function: '<S44>/optimizer' incorporates:
    //   Memory: '<S24>/last_x'
    //   SignalConversion generated from: '<S45>/ SFunction '

    std::memset(&rtDW.vseq[0], 0, 202U * sizeof(double));
    for (rtDW.lastc = 0; rtDW.lastc < 101; rtDW.lastc++) {
      rtDW.vseq[(rtDW.lastc << 1) + 1] = 1.0;
    }

    for (rtDW.lastc = 0; rtDW.lastc < 100; rtDW.lastc++) {
      rtDW.ii = (rtDW.lastc << 1);
      rtDW.rseq[rtDW.ii] = (rtDW.rtb_y_cs_g * 0.033333333333333333) - 1.6;
      rtDW.rseq[rtDW.ii + 1] = (rtDW.rtb_y_m * 0.033333333333333333) -
        0.66666666666666663;
    }

    for (rtDW.ii = 0; rtDW.ii < 101; rtDW.ii++) {
      rtDW.vseq[rtDW.ii << 1] = (RMDscale * rtDW.DiscreteTimeIntegrator) - voff;
    }

    rtDW.rtb_y_m = rtDW.vseq[0];
    rtDW.rtb_TmpSignalConversionAtSFun_n = rtDW.vseq[1];
    rtDW.xk[0] = rtDW.last_x_PreviousInput[0];
    rtDW.xk[1] = rtDW.last_x_PreviousInput[1];
    rtDW.xk[2] = rtDW.last_x_PreviousInput[2];
    rtDW.xk[3] = rtDW.last_x_PreviousInput[3];

    // SignalConversion generated from: '<S45>/ SFunction ' incorporates:
    //   MATLAB Function: '<S44>/optimizer'

    rtDW.ymax_incr[0] = (rtDW.rtb_y_g_f * 0.033333333333333333) - 1.6;
    rtDW.ymax_incr[1] = (rtDW.rtb_y_n_g * 0.033333333333333333) -
      0.66666666666666663;

    // MATLAB Function: '<S44>/optimizer' incorporates:
    //   Memory: '<S24>/Memory'
    //   Memory: '<S24>/last_x'
    //   UnitDelay: '<S24>/last_mv'

    rtDW.rtb_y_cs_g = rtDW.last_x_PreviousInput[1];
    rtDW.rtb_y_g_f = rtDW.last_x_PreviousInput[0];
    rtDW.rtb_y_n_g = rtDW.last_x_PreviousInput[2];
    rtDW.xk_j = rtDW.last_x_PreviousInput[3];
    for (rtDW.knt = 0; rtDW.knt <= 0; rtDW.knt += 2) {
      tmp_1 = _mm_loadu_pd(&rtDW.ymax_incr[rtDW.knt]);
      _mm_storeu_pd(&rtDW.y_innov[rtDW.knt], _mm_sub_pd(tmp_1, _mm_add_pd
        (_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&f_a[rtDW.knt
        + 2]), _mm_set1_pd(rtDW.rtb_y_cs_g)), _mm_mul_pd(_mm_loadu_pd
        (&f_a[rtDW.knt]), _mm_set1_pd(rtDW.rtb_y_g_f))), _mm_mul_pd(_mm_loadu_pd
        (&f_a[rtDW.knt + 4]), _mm_set1_pd(rtDW.rtb_y_n_g))), _mm_mul_pd
                    (_mm_loadu_pd(&f_a[rtDW.knt + 6]), _mm_set1_pd(rtDW.xk_j))),
         _mm_set1_pd((0.0 * rtDW.rtb_y_m) + (0.0 *
        rtDW.rtb_TmpSignalConversionAtSFun_n)))));
    }

    rtDW.y_innov_p = rtDW.y_innov[1];
    rtDW.y_innov_l = rtDW.y_innov[0];
    for (rtDW.knt = 0; rtDW.knt <= 2; rtDW.knt += 2) {
      tmp_1 = _mm_loadu_pd(&rtDW.xk[rtDW.knt]);
      _mm_storeu_pd(&rtDW.xest[rtDW.knt], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&e_a_0[rtDW.knt + 4]), _mm_set1_pd(rtDW.y_innov_p)),
        _mm_mul_pd(_mm_loadu_pd(&e_a_0[rtDW.knt]), _mm_set1_pd(rtDW.y_innov_l))),
        tmp_1));
    }

    rtDW.ymax_incr_flag[0] = false;
    rtDW.ymax_incr[0] = 0.0;
    rtDW.ymin_incr_flag[0] = false;
    rtDW.ymin_incr[0] = 0.0;
    rtDW.ymax_incr_flag[1] = false;
    rtDW.ymax_incr[1] = 0.0;
    rtDW.ymin_incr_flag[1] = false;
    rtDW.ymin_incr[1] = 0.0;
    rtDW.umax_incr_flag = false;
    rtDW.rtb_y_cs_g = 0.0;
    rtDW.umin_incr_flag = false;
    rtDW.rtb_y_n_g = 0.0;
    for (rtDW.i = 0; rtDW.i < 320; rtDW.i++) {
      rtDW.xk_j = 0.0;
      for (rtDW.knt = 0; rtDW.knt < 202; rtDW.knt++) {
        rtDW.xk_j += b_Mv[(320 * rtDW.knt) + rtDW.i] * rtDW.vseq[rtDW.knt];
      }

      rtDW.rtb_y_g_f = b_Mlim[rtDW.i];
      rtDW.xk_j = -((b_Mx[rtDW.i + 320] * rtDW.xest[1]) + (b_Mx[rtDW.i] *
        rtDW.xest[0]) + (b_Mx[rtDW.i + 640] * rtDW.xest[2]) + (b_Mx[rtDW.i + 960]
        * rtDW.xest[3]) + rtDW.rtb_y_g_f + (b_Mu1[rtDW.i] * rtDW.last_mv_DSTATE)
                    + rtDW.xk_j);
      rtDW.Bc[rtDW.i] = rtDW.xk_j;
      rtDW.b_Mrows = b_Mrows[rtDW.i];
      if (rtDW.b_Mrows <= 200) {
        rtDW.ii = rtDW.b_Mrows - (((rtDW.b_Mrows - 1) / ny) << 1) - 1;
        rtDW.b_Del_Save_Flag0 = rtDW.ymax_incr_flag[rtDW.ii];
        if (!rtDW.ymax_incr_flag[rtDW.ii]) {
          rtDW.rtb_y_g_f = -((0.033333333333333333 * static_cast<double>
                              (rtDW.TmpSignalConversionAtSFun_m[rtDW.ii])) -
                             ((-0.93333333333333346 * static_cast<double>
                               (rtDW.ii)) + 1.6)) - (-rtDW.rtb_y_g_f);
          rtDW.b_Del_Save_Flag0 = true;
        } else {
          rtDW.rtb_y_g_f = rtDW.ymax_incr[rtDW.ii];
        }

        rtDW.ymax_incr[rtDW.ii] = rtDW.rtb_y_g_f;
        rtDW.ymax_incr_flag[rtDW.ii] = rtDW.b_Del_Save_Flag0;
        rtDW.Bc[rtDW.i] = rtDW.xk_j + rtDW.rtb_y_g_f;
      } else if (rtDW.b_Mrows <= 400) {
        rtDW.ii = rtDW.b_Mrows - ((rtDW.b_Mrows - 201) >> 1 << 1) - 201;
        rtDW.b_Del_Save_Flag0 = rtDW.ymin_incr_flag[rtDW.ii];
        if (!rtDW.ymin_incr_flag[rtDW.ii]) {
          rtDW.rtb_y_g_f = (0.033333333333333333 *
                            rtDW.TmpSignalConversionAtSFun_a[rtDW.ii]) -
            ((-0.93333333333333346 * static_cast<double>(rtDW.ii)) + 1.6) -
            (-rtDW.rtb_y_g_f);
          rtDW.b_Del_Save_Flag0 = true;
        } else {
          rtDW.rtb_y_g_f = rtDW.ymin_incr[rtDW.ii];
        }

        rtDW.ymin_incr[rtDW.ii] = rtDW.rtb_y_g_f;
        rtDW.ymin_incr_flag[rtDW.ii] = rtDW.b_Del_Save_Flag0;
        rtDW.Bc[rtDW.i] = rtDW.xk_j + rtDW.rtb_y_g_f;
      } else if (rtDW.b_Mrows <= 500) {
        if (!rtDW.umax_incr_flag) {
          rtDW.rtb_y_cs_g = -(RMVscale * rtDW.rtb_y_h_p) - (-rtDW.rtb_y_g_f);
          rtDW.umax_incr_flag = true;
        }

        rtDW.Bc[rtDW.i] = rtDW.xk_j + rtDW.rtb_y_cs_g;
      } else {
        if (!rtDW.umin_incr_flag) {
          rtDW.rtb_y_n_g = (RMVscale * rtDW.rtb_y_d4_b) - (-rtDW.rtb_y_g_f);
          rtDW.umin_incr_flag = true;
        }

        rtDW.Bc[rtDW.i] = rtDW.xk_j + rtDW.rtb_y_n_g;
      }
    }

    std::memset(&rtDW.f[0], 0, 11U * sizeof(double));
    for (rtDW.ii = 0; rtDW.ii < 10; rtDW.ii++) {
      rtDW.rtb_y_h_p = 0.0;
      for (rtDW.knt = 0; rtDW.knt < 200; rtDW.knt++) {
        rtDW.rtb_y_h_p += b_Kr[(200 * rtDW.ii) + rtDW.knt] * rtDW.rseq[rtDW.knt];
      }

      rtDW.rtb_y_d4_b = 0.0;
      for (rtDW.knt = 0; rtDW.knt < 202; rtDW.knt++) {
        rtDW.rtb_y_d4_b += b_Kv[(202 * rtDW.ii) + rtDW.knt] * rtDW.vseq[rtDW.knt];
      }

      rtDW.lastc = (rtDW.ii << 2);
      rtDW.f[rtDW.ii] = (b_Kx[rtDW.lastc + 1] * rtDW.xest[1]) + (b_Kx[rtDW.lastc]
        * rtDW.xest[0]) + (b_Kx[rtDW.lastc + 2] * rtDW.xest[2]) +
        (b_Kx[rtDW.lastc + 3] * rtDW.xest[3]) + rtDW.rtb_y_h_p + (b_Ku1[rtDW.ii]
        * rtDW.last_mv_DSTATE) + rtDW.rtb_y_d4_b;
    }

    std::memcpy(&rtDW.iAout[0], &rtDW.Memory_PreviousInput[0], 320U * sizeof
                (bool));
    qpkwik(b_Linv, b_Hinv, rtDW.f, b_Ac, rtDW.Bc, rtDW.iAout, 1324, 1.0E-6,
           rtDW.zopt, rtDW.a__1, &rtDW.ii);
    if ((rtDW.ii < 0) || (rtDW.ii == 0)) {
      std::memset(&rtDW.zopt[0], 0, 11U * sizeof(double));
    }

    rtDW.rtb_y_h_p = rtDW.last_mv_DSTATE + rtDW.zopt[0];

    // Gain: '<S24>/umin_scale1' incorporates:
    //   MATLAB Function: '<S44>/optimizer'

    rtDW.rtb_y_d4_b = 6.0 * rtDW.rtb_y_h_p;

    // MATLAB Function: '<S11>/DataTypeConversion_atrack' incorporates:
    //   Constant: '<S11>/External control signal constant'

    DataTypeConversion_L0(0.0, &rtDW.rtb_y_cs_g);

    // MATLAB Function: '<S3>/MATLAB Function'
    if (rtDW.rtb_y_d4_b > 0.0) {
      rtDW.rtb_y_d4_b = (rtDW.rtb_y_d4_b * 100.0) / 0.15;
    } else if (rtDW.rtb_y_d4_b < 0.0) {
      rtDW.rtb_y_d4_b = (rtDW.rtb_y_d4_b * 100.0) / 0.33;
    } else {
      rtDW.rtb_y_d4_b = 0.0;
    }

    // End of MATLAB Function: '<S3>/MATLAB Function'

    // Update for Memory: '<S24>/Memory'
    std::memcpy(&rtDW.Memory_PreviousInput[0], &rtDW.iAout[0], 320U * sizeof
                (bool));

    // Update for UnitDelay: '<S24>/last_mv' incorporates:
    //   MATLAB Function: '<S44>/optimizer'

    rtDW.last_mv_DSTATE = rtDW.rtb_y_h_p;

    // MATLAB Function: '<S44>/optimizer'
    rtDW.rtb_y_cs_g = rtDW.xk[1];
    rtDW.rtb_y_g_f = rtDW.xk[0];
    rtDW.rtb_y_n_g = rtDW.xk[2];
    rtDW.xk_j = rtDW.xk[3];
    rtDW.y_innov_p = rtDW.y_innov[1];
    rtDW.y_innov_l = rtDW.y_innov[0];
    for (rtDW.knt = 0; rtDW.knt <= 2; rtDW.knt += 2) {
      // Update for Memory: '<S24>/last_x'
      _mm_storeu_pd(&rtDW.last_x_PreviousInput[rtDW.knt], _mm_add_pd(_mm_add_pd
        (_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&d_a_0[rtDW.knt + 4]), _mm_set1_pd(rtDW.rtb_y_cs_g)), _mm_mul_pd
        (_mm_loadu_pd(&d_a_0[rtDW.knt]), _mm_set1_pd(rtDW.rtb_y_g_f))),
        _mm_mul_pd(_mm_loadu_pd(&d_a_0[rtDW.knt + 8]), _mm_set1_pd
                   (rtDW.rtb_y_n_g))), _mm_mul_pd(_mm_loadu_pd(&d_a_0[rtDW.knt +
        12]), _mm_set1_pd(rtDW.xk_j))), _mm_mul_pd(_mm_loadu_pd(&c_a_0[rtDW.knt]),
        _mm_set1_pd(rtDW.rtb_y_h_p))), _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&b_a_0[rtDW.knt]), _mm_set1_pd(rtDW.rtb_y_m)), _mm_set1_pd(0.0 *
        rtDW.rtb_TmpSignalConversionAtSFun_n))), _mm_add_pd(_mm_mul_pd
        (_mm_loadu_pd(&a_0[rtDW.knt + 4]), _mm_set1_pd(rtDW.y_innov_p)),
        _mm_mul_pd(_mm_loadu_pd(&a_0[rtDW.knt]), _mm_set1_pd(rtDW.y_innov_l)))));
    }

    // End of Outputs for SubSystem: '<S1>/ACC system'

    // SwitchCase: '<S1>/Switch Case' incorporates:
    //   Inport: '<Root>/Mode'

    rtDW.rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&rtM)->solverInfo)) {
      switch (static_cast<int32_t>(rtU.Mode)) {
       case 0:
        rtDW.rtAction = 0;
        break;
       
       case 1:
       case 2:
        rtDW.rtAction = 1;
        break;
      }

      rtDW.SwitchCase_ActiveSubsystem = rtDW.rtAction;
    } else {
      rtDW.rtAction = rtDW.SwitchCase_ActiveSubsystem;
    }

    switch (rtDW.rtAction) {
     case 0:
      // Outputs for IfAction SubSystem: '<S1>/LV' incorporates:
      //   ActionPort: '<S9>/Action Port'

      // SignalConversion generated from: '<S9>/Acceleration_in' incorporates:
      //   Gain: '<S172>/Proportional Gain'
      //   Integrator: '<S167>/Integrator'
      //   Sum: '<S176>/Sum'

      rtDW.Acceleration_in_c = (100.0 * rtDW.SpeedDifferent) +
        rtX.Integrator_CSTATE + rtDW.FilterCoefficient;

      // End of Outputs for SubSystem: '<S1>/LV'

      // Sum: '<S1>/Sum' incorporates:
      //   Inport: '<Root>/pos_ego'
      //   Inport: '<Root>/pos_follow'

      rtDW.rtb_y_h_p = rtU.pos_ego - rtU.pos_follow;

      // Outputs for IfAction SubSystem: '<S1>/LV' incorporates:
      //   ActionPort: '<S9>/Action Port'

      // Saturate: '<S9>/Saturation1'
      if (rtDW.rtb_y_h_p > 100.0) {
        rtDW.rtb_y_h_p = 100.0;
      } else if (rtDW.rtb_y_h_p < 0.0) {
        rtDW.rtb_y_h_p = 0.0;
      }

      // MATLAB Function: '<S9>/Drag coefficient1' incorporates:
      //   Saturate: '<S9>/Saturation1'

      rtDW.C_drag_e = (std::exp((rtDW.rtb_y_h_p - 8.0) * -0.1002) * -0.1012) +
        0.6441;

      // End of Outputs for SubSystem: '<S1>/LV'
      break;

     case 1:
      // Outputs for IfAction SubSystem: '<S1>/FV' incorporates:
      //   ActionPort: '<S7>/Action Port'

      // SignalConversion generated from: '<S7>/Acceleration_in'
      rtDW.Acceleration_in = rtDW.rtb_y_d4_b;

      // Saturate: '<S7>/Saturation1'
      if (rtDW.rtb_Sum_f_c > 100.0) {
        rtDW.rtb_Sum_f_c = 100.0;
      } else if (rtDW.rtb_Sum_f_c < 0.0) {
        rtDW.rtb_Sum_f_c = 0.0;
      }

      // MATLAB Function: '<S7>/Drag coefficient' incorporates:
      //   Saturate: '<S7>/Saturation1'

      rtDW.C_drag = (std::exp((rtDW.rtb_Sum_f_c - 8.0) * -0.068) * -0.4629) +
        0.6441;

      // End of Outputs for SubSystem: '<S1>/FV'
      break;
    }

    // End of SwitchCase: '<S1>/Switch Case'

    // Switch: '<S1>/Switch3' incorporates:
    //   Inport: '<Root>/Mode'

    if (rtU.Mode > 0.0) {
      // DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator = rtDW.Acceleration_in;
    } else {
      // DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator = rtDW.Acceleration_in_c;
    }

    // End of Switch: '<S1>/Switch3'

    // Saturate: '<S1>/Brake[%]1'
    if (rtDW.DiscreteTimeIntegrator > 0.0) {
      // Saturate: '<S1>/Brake[%]1'
      rtDW.Brake1 = 0.0;
    } else if (rtDW.DiscreteTimeIntegrator < -100.0) {
      // Saturate: '<S1>/Brake[%]1'
      rtDW.Brake1 = -100.0;
    } else {
      // Saturate: '<S1>/Brake[%]1'
      rtDW.Brake1 = rtDW.DiscreteTimeIntegrator;
    }

    // End of Saturate: '<S1>/Brake[%]1'
  }

  // FromWorkspace: '<S1>/From Workspace1'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace1_PWORK.DataPtr };

    rtDW.rtb_uDLookupTable1_c = pDataValues[0];
  }

  // Product: '<S1>/Divide2'
  rtDW.rtb_y_h_p = rtDW.Brake1 * rtDW.rtb_uDLookupTable1_c;
  if (rtDW.b_j) {
    // Outport: '<Root>/Speed[ms]' incorporates:
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

    rtY.Speedms = rtDW.DiscreteTimeIntegrator_DSTATE;
  }

  // FromWorkspace: '<S1>/From Workspace21'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace21_PWORK.DataPtr };

    rtDW.rtb_uDLookupTable1_c = pDataValues[0];
  }

  // Product: '<S1>/Divide36' incorporates:
  //   Constant: '<S1>/Constant10'
  //   Constant: '<S1>/Constant9'
  //   Outport: '<Root>/Speed[ms]'
  //   Product: '<S1>/Divide35'

  rtDW.rtb_Sum_f_c = rtY.Speedms / (6.2831853071795862 *
    rtDW.rtb_uDLookupTable1_c);

  // FromWorkspace: '<S1>/From Workspace22'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace22_PWORK.DataPtr };

    rtDW.rtb_uDLookupTable1_c = pDataValues[0];
  }

  // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
  //   Gain: '<S1>/Gain20'
  //   Product: '<S1>/Divide37'

  rtDW.rtb_uDLookupTable1_c = (rtDW.rtb_Sum_f_c * rtDW.rtb_uDLookupTable1_c) *
    60.0;

  // Sum: '<S1>/Add14' incorporates:
  //   Lookup_n-D: '<S1>/2-D Lookup Table1'
  //   Lookup_n-D: '<S1>/MAX Torque1'

  rtDW.rtb_Sum_f_c = look1_binlg(rtDW.rtb_uDLookupTable1_c,
    rtConstP.MAXTorque1_bp01Data, rtConstP.MAXTorque1_tableData, 5U);

  // Product: '<S1>/Divide27' incorporates:
  //   Gain: '<S1>/Gain13'

  rtDW.rtb_y_h_p = (0.01 * rtDW.rtb_y_h_p) * rtDW.rtb_Sum_f_c;
  if (rtDW.b_j) {
    // Saturate: '<S1>/Accel[%]1'
    if (rtDW.DiscreteTimeIntegrator > 100.0) {
      // DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator = 100.0;
    } else if (rtDW.DiscreteTimeIntegrator < 0.0) {
      // DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator = 0.0;
    }

    // End of Saturate: '<S1>/Accel[%]1'

    // Gain: '<S1>/Accel1'
    rtDW.Accel1 = 0.01 * rtDW.DiscreteTimeIntegrator;
  }

  // Product: '<S1>/Divide1'
  rtDW.rtb_y_d4_b = rtDW.rtb_Sum_f_c * rtDW.Accel1;

  // FromWorkspace: '<S1>/From Workspace18'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace18_PWORK.DataPtr };

    rtDW.rtb_Sum_f_c = pDataValues[0];
  }

  // Product: '<S1>/Divide33' incorporates:
  //   Sum: '<S1>/Add1'

  rtDW.rtb_y_h_p = (rtDW.rtb_y_h_p + rtDW.rtb_y_d4_b) * rtDW.rtb_Sum_f_c;

  // FromWorkspace: '<S1>/From Workspace20'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace20_PWORK.DataPtr };

    rtDW.rtb_Sum_f_c = pDataValues[0];
  }

  // Sum: '<S1>/Add14' incorporates:
  //   Gain: '<S1>/BPP[%]*MAX_Brake_Force[N]1'
  //   Product: '<S1>/Divide25'
  //   Product: '<S1>/Divide34'

  rtDW.rtb_Sum_f_c = ((rtDW.Add13 * rtDW.Brake1) * 100.0) + (rtDW.rtb_y_h_p /
    rtDW.rtb_Sum_f_c);

  // FromWorkspace: '<S1>/Rolling_Loss1'
  {
    double *pDataValues{ (double *) rtDW.Rolling_Loss1_PWORK.DataPtr };

    rtDW.Rolling_Loss1 = pDataValues[0];
  }

  // FromWorkspace: '<S1>/From Workspace3'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace3_PWORK.DataPtr };

    rtDW.FromWorkspace3 = pDataValues[0];
  }

  // FromWorkspace: '<S1>/From Workspace4'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace4_PWORK.DataPtr };

    rtDW.FromWorkspace4 = pDataValues[0];
  }

  if (rtDW.b_j) {
    // RelationalOperator: '<S4>/Compare' incorporates:
    //   Constant: '<S4>/Constant'
    //   Outport: '<Root>/Speed[ms]'

    rtDW.Compare = (rtY.Speedms > 1.0);
  }

  // FromWorkspace: '<S1>/Friction_Loss1'
  {
    double *pDataValues{ (double *) rtDW.Friction_Loss1_PWORK.DataPtr };

    rtDW.UnitConversion4 = pDataValues[0];
  }

  // Product: '<S1>/Divide22' incorporates:
  //   Outport: '<Root>/Speed[ms]'

  rtDW.rtb_y_h_p = rtDW.UnitConversion4 * rtY.Speedms;
  if (rtDW.b_j) {
    // Switch: '<S1>/Switch5' incorporates:
    //   Inport: '<Root>/Mode'

    if (rtU.Mode > 1.0) {
      // Switch: '<S1>/Switch5'
      rtDW.Switch5 = rtDW.C_drag;
    } else {
      // Switch: '<S1>/Switch5'
      rtDW.Switch5 = rtDW.C_drag_e;
    }

    // End of Switch: '<S1>/Switch5'

    // Math: '<S1>/Square1' incorporates:
    //   Outport: '<Root>/Speed[ms]'

    rtDW.Square1 = rtY.Speedms * rtY.Speedms;
  }

  // FromWorkspace: '<S1>/From Workspace'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace_PWORK.DataPtr };

    rtDW.UnitConversion4 = pDataValues[0];
  }

  // FromWorkspace: '<S1>/From Workspace2'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace2_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Product: '<S1>/Divide23'
  rtDW.rtb_y_d4_b = ((rtDW.Switch5 * rtDW.Square1) * rtDW.UnitConversion4) *
    rtDW.Step;

  // FromWorkspace: '<S1>/From Workspace15'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace15_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Switch: '<S1>/Switch1' incorporates:
  //   Constant: '<S1>/Constant3'
  //   Product: '<S1>/Divide3'

  if (rtDW.Compare) {
    rtDW.Add13 = (rtDW.Rolling_Loss1 * rtDW.FromWorkspace3) *
      rtDW.FromWorkspace4;
  } else {
    rtDW.Add13 = 10.0;
  }

  // Product: '<S1>/Divide28' incorporates:
  //   Gain: '<S1>/Gain'
  //   Sum: '<S1>/Add11'
  //   Sum: '<S1>/Add15'
  //   Switch: '<S1>/Switch1'

  rtDW.Add13 = (rtDW.Add13 + rtDW.rtb_y_h_p + (0.5 * rtDW.rtb_y_d4_b) +
                rtDW.rtb_Sum_f_c) * rtDW.Step;

  // FromWorkspace: '<S1>/From Workspace14'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace14_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Step: '<S1>/Step' incorporates:
  //   Product: '<S1>/Divide29'

  rtDW.Step = rtDW.Add13 / rtDW.Step;

  // UnitConversion: '<S1>/Unit Conversion4'
  // Unit Conversion - from: rpm to: rad/s
  // Expression: output = (0.10472*input) + (0)
  rtDW.UnitConversion4 = 0.10471975511965977 * rtDW.rtb_uDLookupTable1_c;
  if (rtDW.b_j) {
    // Gain: '<S1>/Vol_pack' incorporates:
    //   Lookup_n-D: '<S1>/SOC-Vol'

    rtDW.Vol_pack = 18.0 * rtDW.Vsoc;
  }

  // Gain: '<S1>/Gain17' incorporates:
  //   Gain: '<S1>/Gain23'
  //   Lookup_n-D: '<S1>/2-D Lookup Table1'
  //   Product: '<S1>/Divide12'
  //   Product: '<S1>/Divide38'
  //   Product: '<S1>/Divide39'
  //   Step: '<S1>/Step'

  rtDW.Motor_Current_cell = (((rtDW.Step * rtDW.UnitConversion4) / (0.01 *
    look2_binlg(rtDW.rtb_uDLookupTable1_c, rtDW.Step,
                rtConstP.uDLookupTable1_bp01Data,
                rtConstP.uDLookupTable1_bp02Data,
                rtConstP.uDLookupTable1_tableData,
                rtConstP.uDLookupTable1_maxIndex, 51U))) / rtDW.Vol_pack) *
    0.055555555555555552;

  // MATLAB Function: '<S1>/ Normalization' incorporates:
  //   Lookup_n-D: '<S1>/SOC-Vol'

  rtDW.rtb_uDLookupTable1_c = rtDW.SOCTemp;
  rtDW.Add13 = rtDW.Vsoc;
  rtDW.rtb_y_h_p = rtDW.Motor_Current_cell;
  if (rtDW.SOCTemp < 19.1) {
    rtDW.rtb_uDLookupTable1_c = 19.1;
  } else if (rtDW.SOCTemp > 25.0) {
    rtDW.rtb_uDLookupTable1_c = 25.0;
  }

  if (rtDW.Vsoc > 4.1679) {
    rtDW.Add13 = 4.1679;
  }

  if (rtDW.Motor_Current_cell < -1.0) {
    rtDW.rtb_y_h_p = -1.0;
  } else if (rtDW.Motor_Current_cell > 7.0) {
    rtDW.rtb_y_h_p = 7.0;
  }

  if (rtDW.b_j) {
    // DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTATE_e;

    // Sum: '<S5>/Minus1' incorporates:
    //   Constant: '<S5>/Constant13'
    //   Constant: '<S5>/Constant14'
    //   Product: '<S5>/Divide10'

    rtDW.Minus1 = 1.0 - (rtDW.DiscreteTimeIntegrator / 180000.0);

    // DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1 = rtDW.DiscreteTimeIntegrator1_DSTATE;
  }

  // FromWorkspace: '<S1>/From Workspace9'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace9_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Product: '<S1>/Divide15'
  rtDW.Veh_acceleartion = rtDW.rtb_Sum_f_c / rtDW.Step;

  // Outputs for Atomic SubSystem: '<S1>/LSTM'
  // Outputs for Atomic SubSystem: '<S8>/lstm_1'
  // SignalConversion generated from: '<S65>/MaskMM' incorporates:
  //   MATLAB Function: '<S1>/ Normalization'

  rtDW.rtb_uDLookupTable1_c = (rtDW.rtb_uDLookupTable1_c - 19.1) /
    5.8999999999999986;
  rtDW.Add13 = (rtDW.Add13 - 3.1724) / 0.99550000000000027;
  rtDW.rtb_y_h_p = (rtDW.rtb_y_h_p - -1.0) / 8.0;
  for (rtDW.knt = 0; rtDW.knt <= 1022; rtDW.knt += 2) {
    // Product: '<S65>/W*x' incorporates:
    //   Constant: '<S65>/InputWeights'
    //   SignalConversion generated from: '<S65>/MaskMM'

    _mm_storeu_pd(&rtDW.Wx[rtDW.knt], _mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.InputWeights_Value[rtDW.knt + 1024]), _mm_set1_pd
       (rtDW.Add13)), _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.InputWeights_Value[rtDW.knt]), _mm_set1_pd
      (rtDW.rtb_uDLookupTable1_c))), _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.InputWeights_Value[rtDW.knt + 2048]), _mm_set1_pd
      (rtDW.rtb_y_h_p))));
  }

  if ((&rtM)->isMajorTimeStep()) {
    // Outputs for Iterator SubSystem: '<S55>/ForIteratorSubsystem' incorporates:
    //   ForIterator: '<S64>/ForIterator'

    if (rtDW.ProbeDimension_n[1] < 2.147483648E+9) {
      if (rtDW.ProbeDimension_n[1] >= -2.147483648E+9) {
        rtDW.ii = static_cast<int32_t>(rtDW.ProbeDimension_n[1]);
      } else {
        rtDW.ii = INT32_MIN;
      }
    } else {
      rtDW.ii = INT32_MAX;
    }

    if (rtDW.ii > 2147483646) {
      rtDW.ii = 2147483646;
    } else if (rtDW.ii < 0) {
      rtDW.ii = 0;
    }

    rtDW.lastc = 1;
    while (rtDW.lastc <= rtDW.ii) {
      for (rtDW.i = 0; rtDW.i < 256; rtDW.i++) {
        // Delay: '<S67>/CellStateDelay'
        if (rtDW.icLoad_e) {
          rtDW.CellStateDelay_DSTATE_d[rtDW.i] = 0.0;
        }

        // Product: '<S113>/DivideOut' incorporates:
        //   Delay: '<S67>/CellStateDelay'

        rtDW.DivideOut_f[rtDW.i] = rtDW.CellStateDelay_DSTATE_d[rtDW.i];

        // Delay: '<S67>/HiddenStateDelay'
        if (rtDW.icLoad_e2) {
          rtDW.HiddenStateDelay_DSTATE_p[rtDW.i] = 0.0;
        }

        // Trigonometry: '<S125>/Tanh' incorporates:
        //   Delay: '<S67>/HiddenStateDelay'

        rtDW.rtb_Tanh_k[rtDW.i] = rtDW.HiddenStateDelay_DSTATE_p[rtDW.i];
      }

      // Sum: '<S72>/Wx+Rh+b' incorporates:
      //   Constant: '<S72>/Bias'
      //   Constant: '<S73>/RecurrentWeights'
      //   Delay: '<S67>/HiddenStateDelay'
      //   Product: '<S65>/W*x'
      //   Product: '<S73>/R*h_t-1'
      //   Selector: '<S64>/Selector1'
      //   Sum: '<S106>/Wx+Rh+b'

      for (rtDW.knt = 0; rtDW.knt < 1024; rtDW.knt++) {
        rtDW.Add13 = 0.0;
        for (rtDW.i = 0; rtDW.i < 256; rtDW.i++) {
          rtDW.Add13 += rtConstP.RecurrentWeights_Value[(rtDW.i << 10) +
            rtDW.knt] * rtDW.HiddenStateDelay_DSTATE_p[rtDW.i];
        }

        rtDW.WxRhb[rtDW.knt] = rtDW.Wx[rtDW.knt] + rtDW.Add13 +
          rtConstP.Bias_Value_d[rtDW.knt];
      }

      // End of Sum: '<S72>/Wx+Rh+b'

      // Outputs for Atomic SubSystem: '<S76>/Sigmoid Layer'
      // Selector: '<S67>/Selector_f' incorporates:
      //   Sum: '<S106>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[256], rtDW.rtb_DivideOut_k_c);

      // End of Outputs for SubSystem: '<S76>/Sigmoid Layer'

      // Product: '<S67>/f*c_t-1' incorporates:
      //   Product: '<S113>/DivideOut'
      //   Product: '<S121>/DivideOut'

      for (rtDW.knt = 0; rtDW.knt <= 254; rtDW.knt += 2) {
        tmp_1 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.knt]);
        _mm_storeu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.knt], _mm_mul_pd(tmp_1, tmp_0));
      }

      // End of Product: '<S67>/f*c_t-1'

      // Outputs for Atomic SubSystem: '<S80>/Sigmoid Layer'
      // Selector: '<S67>/Selector_i' incorporates:
      //   Sum: '<S106>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[0], rtDW.rtb_Tanh_k);

      // End of Outputs for SubSystem: '<S80>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S92>/Tanh Layer'
      // Selector: '<S67>/Selector_g' incorporates:
      //   Sum: '<S106>/Wx+Rh+b'

      TanhLayer(&rtDW.WxRhb[512], rtDW.DivideOut_f);

      // End of Outputs for SubSystem: '<S92>/Tanh Layer'
      for (rtDW.knt = 0; rtDW.knt <= 254; rtDW.knt += 2) {
        // Product: '<S67>/i*g' incorporates:
        //   Product: '<S113>/DivideOut'
        //   Product: '<S67>/f*c_t-1'
        //   Sum: '<S67>/CellAdd'
        //   Trigonometry: '<S125>/Tanh'

        tmp_1 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);
        tmp_0 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);

        // Sum: '<S67>/CellAdd' incorporates:
        //   Product: '<S67>/f*c_t-1'

        tmp = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.knt]);

        // Sum: '<S67>/CellAdd' incorporates:
        //   Product: '<S67>/f*c_t-1'
        //   Product: '<S67>/i*g'

        _mm_storeu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.knt], _mm_add_pd(_mm_mul_pd
          (tmp_1, tmp_0), tmp));
      }

      // Outputs for Atomic SubSystem: '<S84>/Sigmoid Layer'
      // Selector: '<S67>/Selector_o' incorporates:
      //   Sum: '<S106>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[768], rtDW.rtb_Tanh_k);

      // End of Outputs for SubSystem: '<S84>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S88>/Tanh Layer'
      TanhLayer(rtDW.rtb_DivideOut_k_c, rtDW.DivideOut_f);

      // End of Outputs for SubSystem: '<S88>/Tanh Layer'

      // Update for Delay: '<S67>/CellStateDelay'
      rtDW.icLoad_e = false;

      // Update for Delay: '<S67>/HiddenStateDelay'
      rtDW.icLoad_e2 = false;
      for (rtDW.i = 0; rtDW.i <= 254; rtDW.i += 2) {
        // Product: '<S67>/HiddenStateProduct' incorporates:
        //   Product: '<S113>/DivideOut'
        //   Trigonometry: '<S125>/Tanh'

        tmp_1 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.i]);
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.i]);
        tmp_1 = _mm_mul_pd(tmp_1, tmp_0);
        _mm_storeu_pd(&rtDW.rtb_Tanh_k[rtDW.i], tmp_1);

        // Assignment: '<S96>/Assignment' incorporates:
        //   Product: '<S113>/DivideOut'
        //   Product: '<S67>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.Assignment_i[rtDW.i], tmp_1);

        // Update for Delay: '<S67>/CellStateDelay' incorporates:
        //   Product: '<S113>/DivideOut'
        //   Product: '<S67>/HiddenStateProduct'
        //   Sum: '<S67>/CellAdd'

        tmp_0 = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.i]);
        _mm_storeu_pd(&rtDW.CellStateDelay_DSTATE_d[rtDW.i], tmp_0);

        // Update for Delay: '<S67>/HiddenStateDelay' incorporates:
        //   Product: '<S113>/DivideOut'
        //   Product: '<S67>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.HiddenStateDelay_DSTATE_p[rtDW.i], tmp_1);
      }

      rtDW.lastc++;
    }

    // End of Outputs for SubSystem: '<S55>/ForIteratorSubsystem'

    // Outputs for Atomic SubSystem: '<S8>/lstm_2'
    // Product: '<S99>/W*x' incorporates:
    //   Assignment: '<S96>/Assignment'
    //   Constant: '<S99>/InputWeights'
    //   SignalConversion generated from: '<S51>/In1'

    for (rtDW.knt = 0; rtDW.knt < 512; rtDW.knt++) {
      // Product: '<S99>/W*x' incorporates:
      //   Constant: '<S99>/InputWeights'

      rtDW.Add13 = 0.0;
      for (rtDW.i = 0; rtDW.i < 256; rtDW.i++) {
        // Outputs for Atomic SubSystem: '<S8>/dropout_1'
        rtDW.Add13 += rtConstP.InputWeights_Value_p[(rtDW.i << 9) + rtDW.knt] *
          rtDW.Assignment_i[rtDW.i];

        // End of Outputs for SubSystem: '<S8>/dropout_1'
      }

      // Product: '<S99>/W*x' incorporates:
      //   Assignment: '<S96>/Assignment'
      //   Constant: '<S99>/InputWeights'
      //   SignalConversion generated from: '<S51>/In1'

      rtDW.Wx_m[rtDW.knt] = rtDW.Add13;
    }

    // End of Product: '<S99>/W*x'

    // Outputs for Iterator SubSystem: '<S56>/ForIteratorSubsystem' incorporates:
    //   ForIterator: '<S98>/ForIterator'

    if (rtDW.ProbeDimension[1] < 2.147483648E+9) {
      if (rtDW.ProbeDimension[1] >= -2.147483648E+9) {
        rtDW.ii = static_cast<int32_t>(rtDW.ProbeDimension[1]);
      } else {
        rtDW.ii = INT32_MIN;
      }
    } else {
      rtDW.ii = INT32_MAX;
    }

    if (rtDW.ii > 2147483646) {
      rtDW.ii = 2147483646;
    } else if (rtDW.ii < 0) {
      rtDW.ii = 0;
    }

    rtDW.lastc = 1;
    while (rtDW.lastc <= rtDW.ii) {
      for (rtDW.i = 0; rtDW.i < 128; rtDW.i++) {
        // Delay: '<S101>/CellStateDelay' incorporates:
        //   Product: '<S121>/DivideOut'

        if (rtDW.icLoad) {
          rtDW.CellStateDelay_DSTATE[rtDW.i] = 0.0;
        }

        rtDW.rtb_DivideOut_k_c[rtDW.i] = rtDW.CellStateDelay_DSTATE[rtDW.i];

        // End of Delay: '<S101>/CellStateDelay'

        // Delay: '<S101>/HiddenStateDelay' incorporates:
        //   Trigonometry: '<S125>/Tanh'

        if (rtDW.icLoad_d) {
          rtDW.HiddenStateDelay_DSTATE[rtDW.i] = 0.0;
        }

        rtDW.rtb_Tanh_k[rtDW.i] = rtDW.HiddenStateDelay_DSTATE[rtDW.i];

        // End of Delay: '<S101>/HiddenStateDelay'
      }

      for (rtDW.knt = 0; rtDW.knt < 512; rtDW.knt++) {
        // Product: '<S107>/R*h_t-1' incorporates:
        //   Constant: '<S107>/RecurrentWeights'

        rtDW.Add13 = 0.0;
        for (rtDW.i = 0; rtDW.i < 128; rtDW.i++) {
          rtDW.Add13 += rtConstP.RecurrentWeights_Value_g[(rtDW.i << 9) +
            rtDW.knt] * rtDW.rtb_Tanh_k[rtDW.i];
        }

        // Sum: '<S106>/Wx+Rh+b' incorporates:
        //   Constant: '<S106>/Bias'
        //   Product: '<S107>/R*h_t-1'
        //   Product: '<S99>/W*x'
        //   Selector: '<S98>/Selector1'

        rtDW.WxRhb[rtDW.knt] = rtDW.Wx_m[rtDW.knt] + rtDW.Add13 +
          rtConstP.Bias_Value_p[rtDW.knt];
      }

      // Outputs for Atomic SubSystem: '<S110>/Sigmoid Layer'
      // Selector: '<S101>/Selector_f' incorporates:
      //   Product: '<S113>/DivideOut'

      SigmoidLayer_p(&rtDW.WxRhb[128], &rtDW.DivideOut_f[0]);

      // End of Outputs for SubSystem: '<S110>/Sigmoid Layer'

      // Product: '<S101>/f*c_t-1'
      for (rtDW.i = 0; rtDW.i <= 126; rtDW.i += 2) {
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.i]);
        tmp_0 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.i]);
        _mm_storeu_pd(&rtDW.fc_t1[rtDW.i], _mm_mul_pd(tmp_1, tmp_0));
      }

      // End of Product: '<S101>/f*c_t-1'

      // Outputs for Atomic SubSystem: '<S114>/Sigmoid Layer'
      // Selector: '<S101>/Selector_i' incorporates:
      //   Product: '<S121>/DivideOut'

      SigmoidLayer_p(&rtDW.WxRhb[0], &rtDW.rtb_DivideOut_k_c[0]);

      // End of Outputs for SubSystem: '<S114>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S126>/Tanh Layer'
      // Selector: '<S101>/Selector_g' incorporates:
      //   Trigonometry: '<S125>/Tanh'

      TanhLayer_b(&rtDW.WxRhb[256], &rtDW.rtb_Tanh_k[0]);

      // End of Outputs for SubSystem: '<S126>/Tanh Layer'
      for (rtDW.knt = 0; rtDW.knt <= 126; rtDW.knt += 2) {
        // Product: '<S101>/i*g' incorporates:
        //   Product: '<S101>/f*c_t-1'
        //   Sum: '<S101>/CellAdd'

        tmp_1 = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.knt]);
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);

        // Sum: '<S101>/CellAdd' incorporates:
        //   Product: '<S101>/f*c_t-1'

        tmp = _mm_loadu_pd(&rtDW.fc_t1[rtDW.knt]);

        // Sum: '<S101>/CellAdd' incorporates:
        //   Product: '<S101>/f*c_t-1'
        //   Product: '<S101>/i*g'

        _mm_storeu_pd(&rtDW.fc_t1[rtDW.knt], _mm_add_pd(_mm_mul_pd(tmp_1, tmp_0),
          tmp));
      }

      // Outputs for Atomic SubSystem: '<S118>/Sigmoid Layer'
      // Selector: '<S101>/Selector_o' incorporates:
      //   Product: '<S121>/DivideOut'

      SigmoidLayer_p(&rtDW.WxRhb[384], &rtDW.rtb_DivideOut_k_c[0]);

      // End of Outputs for SubSystem: '<S118>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S122>/Tanh Layer'
      // Trigonometry: '<S125>/Tanh'
      TanhLayer_b(rtDW.fc_t1, &rtDW.rtb_Tanh_k[0]);

      // End of Outputs for SubSystem: '<S122>/Tanh Layer'

      // Update for Delay: '<S101>/CellStateDelay'
      rtDW.icLoad = false;

      // Update for Delay: '<S101>/HiddenStateDelay'
      rtDW.icLoad_d = false;
      for (rtDW.i = 0; rtDW.i <= 126; rtDW.i += 2) {
        // Product: '<S101>/HiddenStateProduct'
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.i]);
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_DivideOut_k_c[rtDW.i]);
        tmp_1 = _mm_mul_pd(tmp_1, tmp_0);

        // Assignment: '<S130>/Assignment' incorporates:
        //   Product: '<S101>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.Assignment[rtDW.i], tmp_1);

        // Update for Delay: '<S101>/CellStateDelay' incorporates:
        //   Product: '<S101>/HiddenStateProduct'
        //   Sum: '<S101>/CellAdd'

        tmp_0 = _mm_loadu_pd(&rtDW.fc_t1[rtDW.i]);
        _mm_storeu_pd(&rtDW.CellStateDelay_DSTATE[rtDW.i], tmp_0);

        // Update for Delay: '<S101>/HiddenStateDelay' incorporates:
        //   Product: '<S101>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.HiddenStateDelay_DSTATE[rtDW.i], tmp_1);
      }

      rtDW.lastc++;
    }

    // End of Outputs for SubSystem: '<S56>/ForIteratorSubsystem'
    // End of Outputs for SubSystem: '<S8>/lstm_2'

    // Outputs for Atomic SubSystem: '<S8>/fc'
    // Product: '<S53>/Matrix Multiply' incorporates:
    //   Assignment: '<S130>/Assignment'
    //   Constant: '<S53>/Weights'
    //   SignalConversion generated from: '<S52>/In1'

    rtDW.Add13 = 0.0;
    for (rtDW.knt = 0; rtDW.knt < 128; rtDW.knt++) {
      // Outputs for Atomic SubSystem: '<S8>/dropout_2'
      rtDW.Add13 += rtConstP.Weights_Value[rtDW.knt] * rtDW.Assignment[rtDW.knt];

      // End of Outputs for SubSystem: '<S8>/dropout_2'
    }

    // Outputs for Iterator SubSystem: '<S59>/AddForEachSeq' incorporates:
    //   ForEach: '<S60>/For Each'

    for (rtDW.ForEach_itr_h = 0; rtDW.ForEach_itr_h < 1; rtDW.ForEach_itr_h++) {
      // ForEachSliceAssignment generated from: '<S60>/Out1' incorporates:
      //   Product: '<S53>/Matrix Multiply'
      //   Sum: '<S60>/Add'

      rtDW.ImpAsg_InsertedFor_Out1_at_ = rtDW.Add13 - 0.015775660052895546;
    }

    // End of Outputs for SubSystem: '<S59>/AddForEachSeq'
    // End of Outputs for SubSystem: '<S8>/fc'

    // Outputs for Enabled SubSystem: '<S6>/Correct1' incorporates:
    //   EnablePort: '<S46>/Enable'

    // MATLABSystem: '<S46>/MATLAB System' incorporates:
    //   Constant: '<S6>/R1'
    //   DataStoreRead: '<S46>/Data Store ReadP'
    //   DataStoreRead: '<S46>/Data Store ReadX'

    //  ������ SOC�� �״�� �������� ����
    rtDW.rtb_uDLookupTable1_c = std::fmax(1.4901161193847656E-8,
      1.4901161193847656E-8 * std::abs(rtDW.x));

    //  ������ SOC�� �״�� �������� ����
    rtDW.rtb_uDLookupTable1_c = (rtDW.x + rtDW.rtb_uDLookupTable1_c - rtDW.x) /
      rtDW.rtb_uDLookupTable1_c;

    //  ������ SOC�� �״�� �������� ����
    rtDW.Add13 = qrFactor(rtDW.rtb_uDLookupTable1_c, rtDW.P_d, 0.1);
    rtDW.Add13 = trisolve(rtDW.Add13, trisolve(rtDW.Add13, (rtDW.P_d * rtDW.P_d)
      * rtDW.rtb_uDLookupTable1_c));

    // DataStoreWrite: '<S46>/Data Store WriteP' incorporates:
    //   Constant: '<S6>/R1'
    //   DataStoreRead: '<S46>/Data Store ReadP'
    //   MATLABSystem: '<S46>/MATLAB System'
    //
    rtDW.P_d = qrFactor((-rtDW.Add13 * rtDW.rtb_uDLookupTable1_c) + 1.0,
                        rtDW.P_d, rtDW.Add13 * 0.1);

    // Outputs for Atomic SubSystem: '<S8>/layer'
    // DataStoreWrite: '<S46>/Data Store WriteX' incorporates:
    //   Constant: '<S63>/Constant'
    //   DataStoreRead: '<S46>/Data Store ReadX'
    //   Gain: '<S63>/Gain'
    //   MATLABSystem: '<S46>/MATLAB System'
    //   Math: '<S63>/Exp'
    //   Product: '<S63>/DivideOut'
    //   Sum: '<S63>/Add'
    //  *
    //  About '<S63>/Exp':
    //   Operator: exp

    rtDW.x += ((1.0 / (std::exp(-rtDW.ImpAsg_InsertedFor_Out1_at_) + 1.0)) -
               rtDW.x) * rtDW.Add13;

    // End of Outputs for SubSystem: '<S8>/layer'
    // End of Outputs for SubSystem: '<S6>/Correct1'

    // Outputs for Atomic SubSystem: '<S6>/Output'
    // DataStoreRead: '<S47>/Data Store Read'
    rtDW.ImpAsg_InsertedFor_Out1_at_ = rtDW.x;

    // End of Outputs for SubSystem: '<S6>/Output'

    // Outputs for Atomic SubSystem: '<S6>/Predict'
    // MATLABSystem: '<S48>/MATLAB System' incorporates:
    //   Constant: '<S6>/Q'
    //   DataStoreRead: '<S48>/Data Store ReadP'
    //   DataStoreRead: '<S48>/Data Store ReadX'

    rtDW.TmpSignalConversionAtSFun_a[0] = ((rtDW.Minus1 - rtDW.Minus1) / std::
      fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::abs(rtDW.x))) *
      rtDW.P_d;
    rtDW.TmpSignalConversionAtSFun_a[1] = 0.022360679774997897;
    for (rtDW.lastc = 0; rtDW.lastc < 1; rtDW.lastc++) {
      rtDW.rtb_Sum_f_c = rtDW.TmpSignalConversionAtSFun_a[0];
      rtDW.Add13 = xnrm2_e(1, rtDW.TmpSignalConversionAtSFun_a, 2);
      if (rtDW.Add13 != 0.0) {
        rtDW.rtb_y_h_p = rt_hypotd_snf_e(rtDW.TmpSignalConversionAtSFun_a[0],
          rtDW.Add13);
        if (rtDW.TmpSignalConversionAtSFun_a[0] >= 0.0) {
          rtDW.rtb_y_h_p = -rtDW.rtb_y_h_p;
        }

        if (std::abs(rtDW.rtb_y_h_p) < 1.0020841800044864E-292) {
          rtDW.knt = -1;
          do {
            rtDW.knt++;
            rtDW.Add13 = rtDW.TmpSignalConversionAtSFun_a[1];
            for (rtDW.ii = 2; rtDW.ii < 3; rtDW.ii++) {
              rtDW.Add13 *= 9.9792015476736E+291;
            }

            rtDW.TmpSignalConversionAtSFun_a[1] = rtDW.Add13;
            rtDW.rtb_y_h_p *= 9.9792015476736E+291;
            rtDW.rtb_Sum_f_c *= 9.9792015476736E+291;
          } while ((std::abs(rtDW.rtb_y_h_p) < 1.0020841800044864E-292) &&
                   ((rtDW.knt + 1) < 20));

          rtDW.rtb_y_h_p = rt_hypotd_snf_e(rtDW.rtb_Sum_f_c, xnrm2_e(1,
            rtDW.TmpSignalConversionAtSFun_a, 2));
          if (rtDW.rtb_Sum_f_c >= 0.0) {
            rtDW.rtb_y_h_p = -rtDW.rtb_y_h_p;
          }

          rtDW.rtb_Sum_f_c = 1.0 / (rtDW.rtb_Sum_f_c - rtDW.rtb_y_h_p);
          for (rtDW.ii = 2; rtDW.ii < 3; rtDW.ii++) {
            rtDW.Add13 *= rtDW.rtb_Sum_f_c;
          }

          rtDW.TmpSignalConversionAtSFun_a[1] = rtDW.Add13;
          for (rtDW.ii = 0; rtDW.ii <= rtDW.knt; rtDW.ii++) {
            rtDW.rtb_y_h_p *= 1.0020841800044864E-292;
          }

          rtDW.rtb_Sum_f_c = rtDW.rtb_y_h_p;
        } else {
          rtDW.rtb_Sum_f_c = 1.0 / (rtDW.TmpSignalConversionAtSFun_a[0] -
            rtDW.rtb_y_h_p);
          rtDW.Add13 = rtDW.TmpSignalConversionAtSFun_a[1];
          for (rtDW.ii = 2; rtDW.ii < 3; rtDW.ii++) {
            rtDW.Add13 *= rtDW.rtb_Sum_f_c;
          }

          rtDW.TmpSignalConversionAtSFun_a[1] = rtDW.Add13;
          rtDW.rtb_Sum_f_c = rtDW.rtb_y_h_p;
        }
      }

      rtDW.TmpSignalConversionAtSFun_a[0] = rtDW.rtb_Sum_f_c;
    }

    // DataStoreWrite: '<S48>/Data Store WriteP' incorporates:
    //   MATLABSystem: '<S48>/MATLAB System'
    //
    rtDW.P_d = rtDW.TmpSignalConversionAtSFun_a[0];

    // DataStoreWrite: '<S48>/Data Store WriteX' incorporates:
    //   MATLABSystem: '<S48>/MATLAB System'
    //
    rtDW.x = rtDW.Minus1;

    // End of Outputs for SubSystem: '<S6>/Predict'

    // Gain: '<S1>/Gain5'
    rtDW.ImpAsg_InsertedFor_Out1_at_ *= 100.0;
  }

  // End of Outputs for SubSystem: '<S8>/lstm_1'
  // End of Outputs for SubSystem: '<S1>/LSTM'

  // Gain: '<S164>/Integral Gain'
  rtDW.IntegralGain = 0.01 * rtDW.SpeedDifferent;

  // Step: '<S1>/Step'
  rtDW.Step = !((&rtM)->Timing.t[0] < 10.0);

  // Switch: '<S1>/SOC switch'
  if (rtDW.Step >= 1.0) {
    // Switch: '<S1>/SOC switch' incorporates:
    //   Inport: '<Root>/soc_ego'

    rtDW.SOCswitch = rtU.soc_ego;
  } else {
    // Switch: '<S1>/SOC switch'
    rtDW.SOCswitch = rtDW.Minus1;
  }

  // End of Switch: '<S1>/SOC switch'
  // End of Outputs for SubSystem: '<Root>/BMS'

  // Outport: '<Root>/Accel[ms2]'
  rtY.Accelms2 = rtDW.Veh_acceleartion;
  if ((&rtM)->isMajorTimeStep()) {
    // Outport: '<Root>/Position[m]'
    rtY.Positionm = rtDW.DiscreteTimeIntegrator1;

    // Outport: '<Root>/Battery_SOC[%]'
    rtY.Battery_SOC = rtDW.ImpAsg_InsertedFor_Out1_at_;
  }

  if ((&rtM)->isMajorTimeStep()) {
    // Update for Atomic SubSystem: '<Root>/BMS'
    if ((&rtM)->isMajorTimeStep()) {
      // Update for UnitDelay: '<S1>/SOC_t-1'
      rtDW.SOC_t1_DSTATE = rtDW.SOCswitch;

      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE += 0.1 * rtDW.Veh_acceleartion;

      // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE_e += 0.1 * rtDW.Motor_Current_cell;

      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' incorporates:
      //   Outport: '<Root>/Speed[ms]'

      rtDW.DiscreteTimeIntegrator1_DSTATE += 0.1 * rtY.Speedms;
    }

    // End of Update for SubSystem: '<Root>/BMS'
  }                                    // end MajorTimeStep

  if ((&rtM)->isMajorTimeStep()) {
    rt_ertODEUpdateContinuousStates(&(&rtM)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&rtM)->Timing.clockTick0;
    (&rtM)->Timing.t[0] = rtsiGetSolverStopTime(&(&rtM)->solverInfo);

    {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&rtM)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void BMS::BMS_derivatives()
{
  BMS::XDot *_rtXdot;
  _rtXdot = ((XDot *) (&rtM)->derivs);

  // Derivatives for Atomic SubSystem: '<Root>/BMS'
  // Derivatives for Integrator: '<S167>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.IntegralGain;

  // Derivatives for Integrator: '<S162>/Filter'
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;

  // End of Derivatives for SubSystem: '<Root>/BMS'
}

// Model initialize function
void BMS::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&rtM)->solverInfo, &(&rtM)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rtM)->solverInfo, (&rtM)->getTPtrPtr());
    rtsiSetStepSizePtr(&(&rtM)->solverInfo, &(&rtM)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rtM)->solverInfo, &(&rtM)->derivs);
    rtsiSetContStatesPtr(&(&rtM)->solverInfo, (double **) &(&rtM)->contStates);
    rtsiSetNumContStatesPtr(&(&rtM)->solverInfo, &(&rtM)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&rtM)->solverInfo, (bool**) &(&rtM)
      ->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&rtM)->solverInfo, (&rtM)->getErrorStatusPtr());
    rtsiSetRTModelPtr(&(&rtM)->solverInfo, (&rtM));
  }

  rtsiSetSimTimeStep(&(&rtM)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&rtM)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&rtM)->solverInfo, false);
  (&rtM)->intgData.y = (&rtM)->odeY;
  (&rtM)->intgData.f[0] = (&rtM)->odeF[0];
  (&rtM)->intgData.f[1] = (&rtM)->odeF[1];
  (&rtM)->intgData.f[2] = (&rtM)->odeF[2];
  (&rtM)->contStates = ((X *) &rtX);
  (&rtM)->contStateDisabled = ((XDis *) &rtXDis);
  (&rtM)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&rtM)->solverInfo, static_cast<void *>(&(&rtM)->intgData));
  rtsiSetSolverName(&(&rtM)->solverInfo,"ode3");
  (&rtM)->setTPtr(&(&rtM)->Timing.tArray[0]);
  (&rtM)->Timing.stepSize0 = 0.1;

  // SystemInitialize for Atomic SubSystem: '<Root>/BMS'
  // Start for FromWorkspace: '<S1>/From Workspace19'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.7 };

    rtDW.FromWorkspace19_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace19_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace19_IWORK.PrevIndex = 0;
  }

  // Start for SwitchCase: '<S1>/Switch Case'
  rtDW.SwitchCase_ActiveSubsystem = -1;

  // Start for FromWorkspace: '<S1>/From Workspace1'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.7 };

    rtDW.FromWorkspace1_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace1_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace1_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace21'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.52 };

    rtDW.FromWorkspace21_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace21_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace21_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace22'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 5.29 };

    rtDW.FromWorkspace22_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace22_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace22_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace18'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 5.29 };

    rtDW.FromWorkspace18_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace18_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace18_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace20'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.52 };

    rtDW.FromWorkspace20_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace20_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace20_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/Rolling_Loss1'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.006 };

    rtDW.Rolling_Loss1_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.Rolling_Loss1_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.Rolling_Loss1_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace3'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 9.81 };

    rtDW.FromWorkspace3_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace3_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace3_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace4'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 44000.0 };

    rtDW.FromWorkspace4_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace4_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace4_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/Friction_Loss1'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.21857 };

    rtDW.Friction_Loss1_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.Friction_Loss1_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.Friction_Loss1_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 9.84 };

    rtDW.FromWorkspace_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace2'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 1.225 };

    rtDW.FromWorkspace2_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace2_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace2_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace15'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.52 };

    rtDW.FromWorkspace15_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace15_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace15_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace14'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 5.29 };

    rtDW.FromWorkspace14_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace14_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace14_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace9'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 44000.0 };

    rtDW.FromWorkspace9_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace9_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace9_IWORK.PrevIndex = 0;
  }

  // Start for DataStoreMemory: '<S6>/DataStoreMemory - P'
  rtDW.P_d = 0.1414213562373095;

  // Start for DataStoreMemory: '<S6>/DataStoreMemory - x'
  rtDW.x = 1.0;

  // InitializeConditions for Integrator: '<S167>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S162>/Filter'
  rtX.Filter_CSTATE = 0.0;

  // SystemInitialize for Atomic SubSystem: '<S1>/LSTM'
  // SystemInitialize for Atomic SubSystem: '<S8>/lstm_1'
  // SystemInitialize for Iterator SubSystem: '<S55>/ForIteratorSubsystem'
  // Start for Probe: '<S64>/Probe Dimension'
  rtDW.ProbeDimension_n[0] = 1024.0;
  rtDW.ProbeDimension_n[1] = 1.0;

  // InitializeConditions for Delay: '<S67>/CellStateDelay'
  rtDW.icLoad_e = true;

  // InitializeConditions for Delay: '<S67>/HiddenStateDelay'
  rtDW.icLoad_e2 = true;

  // End of SystemInitialize for SubSystem: '<S55>/ForIteratorSubsystem'
  // End of SystemInitialize for SubSystem: '<S8>/lstm_1'

  // SystemInitialize for Atomic SubSystem: '<S8>/lstm_2'
  // SystemInitialize for Iterator SubSystem: '<S56>/ForIteratorSubsystem'
  // Start for Probe: '<S98>/Probe Dimension'
  rtDW.ProbeDimension[0] = 512.0;
  rtDW.ProbeDimension[1] = 1.0;

  // InitializeConditions for Delay: '<S101>/CellStateDelay'
  rtDW.icLoad = true;

  // InitializeConditions for Delay: '<S101>/HiddenStateDelay'
  rtDW.icLoad_d = true;

  // End of SystemInitialize for SubSystem: '<S56>/ForIteratorSubsystem'
  // End of SystemInitialize for SubSystem: '<S8>/lstm_2'
  // End of SystemInitialize for SubSystem: '<S1>/LSTM'
  // End of SystemInitialize for SubSystem: '<Root>/BMS'
}

BMS::XDis* BMS::RT_MODEL::getContStateDisabled() const
{
  return contStateDisabled;
}

void BMS::RT_MODEL::setContStateDisabled(XDis* aContStateDisabled)
{
  contStateDisabled = aContStateDisabled;
}

const char** BMS::RT_MODEL::getErrorStatusPtr()
{
  return &errorStatus;
}

BMS::X* BMS::RT_MODEL::getContStates() const
{
  return contStates;
}

void BMS::RT_MODEL::setContStates(X* aContStates)
{
  contStates = aContStates;
}

bool BMS::RT_MODEL::getStopRequested() const
{
  return (Timing.stopRequestedFlag);
}

void BMS::RT_MODEL::setStopRequested(bool aStopRequested)
{
  (Timing.stopRequestedFlag = aStopRequested);
}

ODE3_IntgData BMS::RT_MODEL::getIntgData() const
{
  return intgData;
}

void BMS::RT_MODEL::setIntgData(ODE3_IntgData aIntgData)
{
  intgData = aIntgData;
}

bool BMS::RT_MODEL::getDerivCacheNeedsReset() const
{
  return derivCacheNeedsReset;
}

void BMS::RT_MODEL::setDerivCacheNeedsReset(bool aDerivCacheNeedsReset)
{
  derivCacheNeedsReset = aDerivCacheNeedsReset;
}

const char* BMS::RT_MODEL::getErrorStatus() const
{
  return (errorStatus);
}

void BMS::RT_MODEL::setErrorStatus(const char* const aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

bool BMS::RT_MODEL::getContTimeOutputInconsistentWithStateAtMajorStepFlag()
  const
{
  return CTOutputIncnstWithState;
}

void BMS::RT_MODEL::setContTimeOutputInconsistentWithStateAtMajorStepFlag(bool
  aContTimeOutputInconsistentWithStateAtMajorStepFlag)
{
  CTOutputIncnstWithState = aContTimeOutputInconsistentWithStateAtMajorStepFlag;
}

bool BMS::RT_MODEL::isMajorTimeStep() const
{
  return ((Timing.simTimeStep) == MAJOR_TIME_STEP);
}

const BMS::odeFSubArray* BMS::RT_MODEL::getOdeF() const
{
  return odeF;
}

bool BMS::RT_MODEL::isMinorTimeStep() const
{
  return ((Timing.simTimeStep) == MINOR_TIME_STEP);
}

const double* BMS::RT_MODEL::getOdeY() const
{
  return odeY;
}

int* BMS::RT_MODEL::getPeriodicContStateIndices() const
{
  return periodicContStateIndices;
}

void BMS::RT_MODEL::setPeriodicContStateIndices(int* aPeriodicContStateIndices)
{
  periodicContStateIndices = aPeriodicContStateIndices;
}

double* BMS::RT_MODEL::getTPtr() const
{
  return (Timing.t);
}

void BMS::RT_MODEL::setTPtr(double* aTPtr)
{
  (Timing.t = aTPtr);
}

double* BMS::RT_MODEL::getPeriodicContStateRanges() const
{
  return periodicContStateRanges;
}

void BMS::RT_MODEL::setPeriodicContStateRanges(double* aPeriodicContStateRanges)
{
  periodicContStateRanges = aPeriodicContStateRanges;
}

bool* BMS::RT_MODEL::getStopRequestedPtr()
{
  return (&(Timing.stopRequestedFlag));
}

double** BMS::RT_MODEL::getTPtrPtr()
{
  return &(Timing.t);
}

double BMS::RT_MODEL::getTStart() const
{
  return (Timing.tStart);
}

bool BMS::RT_MODEL::getZCCacheNeedsReset() const
{
  return zCCacheNeedsReset;
}

void BMS::RT_MODEL::setZCCacheNeedsReset(bool aZCCacheNeedsReset)
{
  zCCacheNeedsReset = aZCCacheNeedsReset;
}

double* BMS::RT_MODEL::getdX() const
{
  return derivs;
}

void BMS::RT_MODEL::setdX(double* adX)
{
  derivs = adX;
}

// Constructor
BMS::BMS() :
  rtU(),
  rtY(),
  rtDW(),
  rtX(),
  rtXDis(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
BMS::~BMS() = default;

// Real-Time Model get method
BMS::RT_MODEL * BMS::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
