//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS.cpp
//
// Code generated for Simulink model 'BMS'.
//
// Model version                  : 10.118
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Nov  6 15:11:50 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "BMS.h"
#include <stdint.h>
#include <cmath>
#include <emmintrin.h>
#include <stdbool.h>
#include "cmath"
#include "limits"

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
//    '<S40>/Sigmoid Layer'
//    '<S44>/Sigmoid Layer'
//    '<S48>/Sigmoid Layer'
//
void BMS::SigmoidLayer(const double rtu_In1[256], double rty_Out1[256])
{
  // Product: '<S43>/DivideOut' incorporates:
  //   Constant: '<S43>/Constant'
  //   Gain: '<S43>/Gain'
  //   Math: '<S43>/Exp'
  //   Sum: '<S43>/Add'
  //
  //  About '<S43>/Exp':
  //   Operator: exp

  for (int32_t i{0}; i < 256; i++) {
    rty_Out1[i] = 1.0 / (std::exp(-rtu_In1[i]) + 1.0);
  }

  // End of Product: '<S43>/DivideOut'
}

//
// Output and update for atomic system:
//    '<S52>/Tanh Layer'
//    '<S56>/Tanh Layer'
//
void BMS::TanhLayer(const double rtu_In1[256], double rty_Out1[256])
{
  // Trigonometry: '<S55>/Tanh'
  for (int32_t i{0}; i < 256; i++) {
    rty_Out1[i] = std::tanh(rtu_In1[i]);
  }

  // End of Trigonometry: '<S55>/Tanh'
}

//
// Output and update for atomic system:
//    '<S74>/Sigmoid Layer'
//    '<S78>/Sigmoid Layer'
//    '<S82>/Sigmoid Layer'
//
void BMS::SigmoidLayer_n(const double rtu_In1[128], double rty_Out1[128])
{
  // Product: '<S77>/DivideOut' incorporates:
  //   Constant: '<S77>/Constant'
  //   Gain: '<S77>/Gain'
  //   Math: '<S77>/Exp'
  //   Sum: '<S77>/Add'
  //
  //  About '<S77>/Exp':
  //   Operator: exp

  for (int32_t i{0}; i < 128; i++) {
    rty_Out1[i] = 1.0 / (std::exp(-rtu_In1[i]) + 1.0);
  }

  // End of Product: '<S77>/DivideOut'
}

//
// Output and update for atomic system:
//    '<S86>/Tanh Layer'
//    '<S90>/Tanh Layer'
//
void BMS::TanhLayer_m(const double rtu_In1[128], double rty_Out1[128])
{
  // Trigonometry: '<S89>/Tanh'
  for (int32_t i{0}; i < 128; i++) {
    rty_Out1[i] = std::tanh(rtu_In1[i]);
  }

  // End of Trigonometry: '<S89>/Tanh'
}

double BMS::xnrm2(int32_t n, const double x[2], int32_t ix0)
{
  double y;

  // Start for MATLABSystem: '<S11>/MATLAB System' incorporates:
  //   MATLABSystem: '<S13>/MATLAB System'

  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      int32_t kend;
      rtDW.scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_t k{ix0}; k < kend; k++) {
        rtDW.absxk = std::abs(x[k - 1]);
        if (rtDW.absxk > rtDW.scale) {
          rtDW.t = rtDW.scale / rtDW.absxk;
          y = ((y * rtDW.t) * rtDW.t) + 1.0;
          rtDW.scale = rtDW.absxk;
        } else {
          rtDW.t = rtDW.absxk / rtDW.scale;
          y += rtDW.t * rtDW.t;
        }
      }

      y = rtDW.scale * std::sqrt(y);
    }
  }

  // End of Start for MATLABSystem: '<S11>/MATLAB System'
  return y;
}

double BMS::rt_hypotd_snf_e(double u0, double u1)
{
  double b;
  double y;
  rtDW.a = std::abs(u0);
  b = std::abs(u1);
  if (rtDW.a < b) {
    rtDW.a /= b;
    y = std::sqrt((rtDW.a * rtDW.a) + 1.0) * b;
  } else if (rtDW.a > b) {
    b /= rtDW.a;
    y = std::sqrt((b * b) + 1.0) * rtDW.a;
  } else if (std::isnan(b)) {
    y = (rtNaN);
  } else {
    y = rtDW.a * 1.4142135623730951;
  }

  return y;
}

double BMS::qrFactor(double A, double S, double Ns)
{
  // Start for MATLABSystem: '<S11>/MATLAB System'
  rtDW.M_c[0] = S * A;
  rtDW.M_c[1] = Ns;

  // Start for MATLABSystem: '<S11>/MATLAB System'
  for (rtDW.b_i_p = 0; rtDW.b_i_p < 1; rtDW.b_i_p++) {
    rtDW.b_atmp = rtDW.M_c[0];
    rtDW.beta1 = xnrm2(1, rtDW.M_c, 2);
    if (rtDW.beta1 != 0.0) {
      rtDW.beta1 = rt_hypotd_snf_e(rtDW.M_c[0], rtDW.beta1);
      if (rtDW.M_c[0] >= 0.0) {
        rtDW.beta1 = -rtDW.beta1;
      }

      if (std::abs(rtDW.beta1) < 1.0020841800044864E-292) {
        rtDW.knt_c = -1;
        do {
          rtDW.knt_c++;
          rtDW.M_b = rtDW.M_c[1];
          for (rtDW.jA = 2; rtDW.jA < 3; rtDW.jA++) {
            rtDW.M_b *= 9.9792015476736E+291;
          }

          rtDW.M_c[1] = rtDW.M_b;
          rtDW.beta1 *= 9.9792015476736E+291;
          rtDW.b_atmp *= 9.9792015476736E+291;
        } while ((std::abs(rtDW.beta1) < 1.0020841800044864E-292) &&
                 ((rtDW.knt_c + 1) < 20));

        rtDW.beta1 = rt_hypotd_snf_e(rtDW.b_atmp, xnrm2(1, rtDW.M_c, 2));
        if (rtDW.b_atmp >= 0.0) {
          rtDW.beta1 = -rtDW.beta1;
        }

        rtDW.b_atmp = 1.0 / (rtDW.b_atmp - rtDW.beta1);
        for (rtDW.jA = 2; rtDW.jA < 3; rtDW.jA++) {
          rtDW.M_b *= rtDW.b_atmp;
        }

        rtDW.M_c[1] = rtDW.M_b;
        for (rtDW.jA = 0; rtDW.jA <= rtDW.knt_c; rtDW.jA++) {
          rtDW.beta1 *= 1.0020841800044864E-292;
        }

        rtDW.b_atmp = rtDW.beta1;
      } else {
        rtDW.b_atmp = 1.0 / (rtDW.M_c[0] - rtDW.beta1);
        rtDW.M_b = rtDW.M_c[1];
        for (rtDW.jA = 2; rtDW.jA < 3; rtDW.jA++) {
          rtDW.M_b *= rtDW.b_atmp;
        }

        rtDW.M_c[1] = rtDW.M_b;
        rtDW.b_atmp = rtDW.beta1;
      }
    }

    rtDW.M_c[0] = rtDW.b_atmp;
  }

  return rtDW.M_c[0];
}

double BMS::trisolve(double A, double B_0)
{
  double b_B;

  // Start for MATLABSystem: '<S11>/MATLAB System'
  b_B = B_0;
  if (B_0 != 0.0) {
    b_B = B_0 / A;
  }

  // End of Start for MATLABSystem: '<S11>/MATLAB System'
  return b_B;
}

// Model step function
void BMS::step()
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  bool b;
  if ((&rtM)->isMajorTimeStep()) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if ((&rtM)->isMinorTimeStep()) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  // Outputs for Atomic SubSystem: '<Root>/BMS with LSTM'
  b = ((&rtM)->isMajorTimeStep());
  if (b) {
    // UnitDelay: '<S1>/SOC_t-2'
    rtDW.DiscreteTimeIntegrator_d = rtDW.SOC_t2_DSTATE;

    // Lookup_n-D: '<S1>/SOC-Temp' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtDW.SOCTemp = look1_binlg(rtDW.DiscreteTimeIntegrator_d,
      rtConstP.SOCTemp_bp01Data, rtConstP.SOCTemp_tableData, 3987U);

    // Lookup_n-D: '<S1>/SOC-Vol1' incorporates:
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'

    rtDW.Vsoc = look1_binlag(rtDW.DiscreteTimeIntegrator_d,
      rtConstP.SOCVol1_bp01Data, rtConstP.SOCVol1_tableData, 100U);
  }

  // FromWorkspace: '<S1>/From Workspace19'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace19_PWORK.DataPtr };

    rtDW.SpeedDifferent = pDataValues[0];
  }

  // Sum: '<S1>/Add13' incorporates:
  //   Constant: '<S1>/Constant8'

  rtDW.Add13 = 1.0 - rtDW.SpeedDifferent;

  // Sum: '<S1>/Speed Different' incorporates:
  //   Gain: '<S1>/Gain1'
  //   Inport: '<Root>/ego_velocity'
  //   Inport: '<Root>/target_velocity'

  rtDW.SpeedDifferent = rtU.target_velocity - (3.6 * rtU.ego_velocity);

  // Gain: '<S133>/Filter Coefficient' incorporates:
  //   Gain: '<S123>/Derivative Gain'
  //   Integrator: '<S125>/Filter'
  //   Sum: '<S125>/SumD'

  rtDW.FilterCoefficient = ((0.0001 * rtDW.SpeedDifferent) - rtX.Filter_CSTATE) *
    10.0;

  // Sum: '<S1>/Add14' incorporates:
  //   Gain: '<S135>/Proportional Gain'
  //   Gain: '<S1>/Accel2'
  //   Integrator: '<S130>/Integrator'
  //   Sum: '<S139>/Sum'

  rtDW.DiscreteTimeIntegrator_d = ((100.0 * rtDW.SpeedDifferent) +
    rtX.Integrator_CSTATE + rtDW.FilterCoefficient) * 100.0;

  // Saturate: '<S1>/Brake[%]1'
  if (rtDW.DiscreteTimeIntegrator_d > 0.0) {
    rtDW.F_rolling = 0.0;
  } else if (rtDW.DiscreteTimeIntegrator_d < -100.0) {
    rtDW.F_rolling = -100.0;
  } else {
    rtDW.F_rolling = rtDW.DiscreteTimeIntegrator_d;
  }

  // End of Saturate: '<S1>/Brake[%]1'

  // Product: '<S1>/Divide25'
  rtDW.Add13 *= rtDW.F_rolling;

  // FromWorkspace: '<S1>/From Workspace1'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace1_PWORK.DataPtr };

    rtDW.uDLookupTable1 = pDataValues[0];
  }

  // Product: '<S1>/Divide2'
  rtDW.Divide2 = rtDW.F_rolling * rtDW.uDLookupTable1;

  // FromWorkspace: '<S1>/From Workspace21'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace21_PWORK.DataPtr };

    rtDW.uDLookupTable1 = pDataValues[0];
  }

  // Product: '<S1>/Divide36' incorporates:
  //   Constant: '<S1>/Constant10'
  //   Constant: '<S1>/Constant9'
  //   Inport: '<Root>/ego_velocity'
  //   Product: '<S1>/Divide35'

  rtDW.F_rolling = rtU.ego_velocity / (6.2831853071795862 * rtDW.uDLookupTable1);

  // FromWorkspace: '<S1>/From Workspace22'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace22_PWORK.DataPtr };

    rtDW.uDLookupTable1 = pDataValues[0];
  }

  // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
  //   Gain: '<S1>/Gain20'
  //   Product: '<S1>/Divide37'

  rtDW.uDLookupTable1 = (rtDW.F_rolling * rtDW.uDLookupTable1) * 60.0;

  // Lookup_n-D: '<S1>/MAX Torque1' incorporates:
  //   Lookup_n-D: '<S1>/2-D Lookup Table1'

  rtDW.F_rolling = look1_binlg(rtDW.uDLookupTable1, rtConstP.MAXTorque1_bp01Data,
    rtConstP.MAXTorque1_tableData, 5U);

  // Saturate: '<S1>/Accel[%]1'
  if (rtDW.DiscreteTimeIntegrator_d > 100.0) {
    // Sum: '<S1>/Add14'
    rtDW.DiscreteTimeIntegrator_d = 100.0;
  } else if (rtDW.DiscreteTimeIntegrator_d < 0.0) {
    // Sum: '<S1>/Add14'
    rtDW.DiscreteTimeIntegrator_d = 0.0;
  }

  // End of Saturate: '<S1>/Accel[%]1'

  // Product: '<S1>/Divide1' incorporates:
  //   Gain: '<S1>/Accel1'

  rtDW.Motor_TorqueNm_d = (0.01 * rtDW.DiscreteTimeIntegrator_d) *
    rtDW.F_rolling;

  // FromWorkspace: '<S1>/From Workspace18'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace18_PWORK.DataPtr };

    rtDW.DiscreteTimeIntegrator_d = pDataValues[0];
  }

  // Product: '<S1>/Divide33' incorporates:
  //   Gain: '<S1>/Gain13'
  //   Product: '<S1>/Divide27'
  //   Sum: '<S1>/Add1'

  rtDW.F_rolling = (((0.01 * rtDW.Divide2) * rtDW.F_rolling) +
                    rtDW.Motor_TorqueNm_d) * rtDW.DiscreteTimeIntegrator_d;

  // FromWorkspace: '<S1>/From Workspace20'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace20_PWORK.DataPtr };

    rtDW.DiscreteTimeIntegrator_d = pDataValues[0];
  }

  // Sum: '<S1>/Add14' incorporates:
  //   Gain: '<S1>/BPP[%]*MAX_Brake_Force[N]1'
  //   Product: '<S1>/Divide34'

  rtDW.DiscreteTimeIntegrator_d = (100.0 * rtDW.Add13) + (rtDW.F_rolling /
    rtDW.DiscreteTimeIntegrator_d);

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

  // FromWorkspace: '<S1>/Friction_Loss1'
  {
    double *pDataValues{ (double *) rtDW.Friction_Loss1_PWORK.DataPtr };

    rtDW.UnitConversion4 = pDataValues[0];
  }

  // Product: '<S1>/Divide22' incorporates:
  //   Inport: '<Root>/ego_velocity'

  rtDW.Add13 = rtDW.UnitConversion4 * rtU.ego_velocity;

  // FromWorkspace: '<S1>/From Workspace4'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace4_PWORK.DataPtr };

    rtDW.UnitConversion4 = pDataValues[0];
  }

  // Product: '<S1>/Divide4' incorporates:
  //   Inport: '<Root>/Mass_kg'

  rtDW.Divide2 = (rtU.Mass_kg * rtDW.UnitConversion4) * 0.0;
  if (b) {
    // Saturate: '<S1>/Saturation2' incorporates:
    //   Inport: '<Root>/IVD'

    if (rtU.IVD > 100.0) {
      rtDW.Motor_TorqueNm_d = 100.0;
    } else if (rtU.IVD < 0.0) {
      rtDW.Motor_TorqueNm_d = 0.0;
    } else {
      rtDW.Motor_TorqueNm_d = rtU.IVD;
    }

    // End of Saturate: '<S1>/Saturation2'

    // Switch: '<S1>/Switch5' incorporates:
    //   Inport: '<Root>/Mode'

    if (rtU.Mode == 1.0) {
      // Outport: '<Root>/Aero' incorporates:
      //   MATLAB Function: '<S1>/Drag coefficient for FV'

      rtY.Aero = (std::exp((rtDW.Motor_TorqueNm_d - 8.0) * -0.068) * -0.4629) +
        0.6441;
    } else if (rtU.Mode == 0.0) {
      // Outport: '<Root>/Aero' incorporates:
      //   MATLAB Function: '<S1>/Drag coefficient for LV'

      rtY.Aero = (std::exp((rtDW.Motor_TorqueNm_d - 8.0) * -0.1002) * -0.1012) +
        0.6441;
    } else {
      rtY.Aero = 0.6441;
    }

    // End of Switch: '<S1>/Switch5'
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

  // Product: '<S1>/Divide23' incorporates:
  //   Inport: '<Root>/ego_velocity'
  //   Math: '<S1>/Square1'
  //   Outport: '<Root>/Aero'

  rtDW.Motor_TorqueNm_d = (((rtU.ego_velocity * rtU.ego_velocity) * rtY.Aero) *
    rtDW.UnitConversion4) * rtDW.Step;

  // FromWorkspace: '<S1>/From Workspace15'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace15_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Switch: '<S1>/Switch1' incorporates:
  //   Constant: '<S1>/Constant3'
  //   Constant: '<S4>/Constant'
  //   Inport: '<Root>/Mass_kg'
  //   Inport: '<Root>/ego_velocity'
  //   Product: '<S1>/Divide3'
  //   RelationalOperator: '<S4>/Compare'

  if (rtU.ego_velocity > 1.0) {
    rtDW.F_rolling = (rtDW.Rolling_Loss1 * rtDW.FromWorkspace3) * rtU.Mass_kg;
  } else {
    rtDW.F_rolling = 10.0;
  }

  // Product: '<S1>/Divide28' incorporates:
  //   Gain: '<S1>/Gain'
  //   Sum: '<S1>/Add11'
  //   Sum: '<S1>/Add15'
  //   Switch: '<S1>/Switch1'

  rtDW.F_rolling = (rtDW.F_rolling + rtDW.Add13 + rtDW.Divide2 + (0.5 *
    rtDW.Motor_TorqueNm_d) + rtDW.DiscreteTimeIntegrator_d) * rtDW.Step;

  // FromWorkspace: '<S1>/From Workspace14'
  {
    double *pDataValues{ (double *) rtDW.FromWorkspace14_PWORK.DataPtr };

    rtDW.Step = pDataValues[0];
  }

  // Step: '<S1>/Step' incorporates:
  //   Product: '<S1>/Divide29'

  rtDW.Step = rtDW.F_rolling / rtDW.Step;

  // UnitConversion: '<S1>/Unit Conversion4'
  // Unit Conversion - from: rpm to: rad/s
  // Expression: output = (0.10472*input) + (0)
  rtDW.UnitConversion4 = 0.10471975511965977 * rtDW.uDLookupTable1;

  // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
  //   Step: '<S1>/Step'

  rtDW.uDLookupTable1 = look2_binlg(rtDW.uDLookupTable1, rtDW.Step,
    rtConstP.uDLookupTable1_bp01Data, rtConstP.uDLookupTable1_bp02Data,
    rtConstP.uDLookupTable1_tableData, rtConstP.uDLookupTable1_maxIndex, 51U);
  if (b) {
    // Gain: '<S1>/Vol_pack1' incorporates:
    //   Lookup_n-D: '<S1>/SOC-Vol1'

    rtDW.Vol_pack1 = 18.0 * rtDW.Vsoc;
  }

  // Gain: '<S1>/Gain17' incorporates:
  //   Gain: '<S1>/Gain23'
  //   Product: '<S1>/Divide38'
  //   Product: '<S1>/Divide39'
  //   Product: '<S1>/Divide5'

  rtDW.Motor_Current_cell = (((rtDW.Step * rtDW.UnitConversion4) / (0.01 *
    rtDW.uDLookupTable1)) / rtDW.Vol_pack1) * 0.055555555555555552;

  // MATLAB Function: '<S1>/ Normalization' incorporates:
  //   Lookup_n-D: '<S1>/SOC-Vol1'

  rtDW.F_rolling = rtDW.SOCTemp;
  rtDW.Add13 = rtDW.Vsoc;
  rtDW.Divide2 = rtDW.Motor_Current_cell;
  if (rtDW.SOCTemp < 19.1) {
    rtDW.F_rolling = 19.1;
  } else if (rtDW.SOCTemp > 25.0) {
    rtDW.F_rolling = 25.0;
  }

  if (rtDW.Vsoc > 4.1679) {
    rtDW.Add13 = 4.1679;
  }

  if (rtDW.Motor_Current_cell < -1.0) {
    rtDW.Divide2 = -1.0;
  } else if (rtDW.Motor_Current_cell > 7.0) {
    rtDW.Divide2 = 7.0;
  }

  // Outputs for Atomic SubSystem: '<S1>/LSTM'
  // Outputs for Atomic SubSystem: '<S9>/lstm_1'
  // SignalConversion generated from: '<S29>/MaskMM' incorporates:
  //   MATLAB Function: '<S1>/ Normalization'

  rtDW.Motor_TorqueNm_d = (rtDW.F_rolling - 19.1) / 5.8999999999999986;
  rtDW.Add13 = (rtDW.Add13 - 3.1724) / 0.99550000000000027;
  rtDW.Divide2 = (rtDW.Divide2 - -1.0) / 8.0;
  for (rtDW.knt = 0; rtDW.knt <= 1022; rtDW.knt += 2) {
    // Product: '<S29>/W*x' incorporates:
    //   Constant: '<S29>/InputWeights'
    //   SignalConversion generated from: '<S29>/MaskMM'

    _mm_storeu_pd(&rtDW.Wx[rtDW.knt], _mm_add_pd(_mm_add_pd(_mm_mul_pd
      (_mm_loadu_pd(&rtConstP.InputWeights_Value[rtDW.knt + 1024]), _mm_set1_pd
       (rtDW.Add13)), _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.InputWeights_Value[rtDW.knt]), _mm_set1_pd
      (rtDW.Motor_TorqueNm_d))), _mm_mul_pd(_mm_loadu_pd
      (&rtConstP.InputWeights_Value[rtDW.knt + 2048]), _mm_set1_pd(rtDW.Divide2))));
  }

  if ((&rtM)->isMajorTimeStep()) {
    // Outputs for Iterator SubSystem: '<S19>/ForIteratorSubsystem' incorporates:
    //   ForIterator: '<S28>/ForIterator'

    if (rtDW.ProbeDimension_o[1] < 2.147483648E+9) {
      if (rtDW.ProbeDimension_o[1] >= -2.147483648E+9) {
        rtDW.b_i = static_cast<int32_t>(rtDW.ProbeDimension_o[1]);
      } else {
        rtDW.b_i = INT32_MIN;
      }
    } else {
      rtDW.b_i = INT32_MAX;
    }

    if (rtDW.b_i > 2147483646) {
      rtDW.b_i = 2147483646;
    } else if (rtDW.b_i < 0) {
      rtDW.b_i = 0;
    }

    rtDW.s28_iter = 1;
    while (rtDW.s28_iter <= rtDW.b_i) {
      for (rtDW.knt = 0; rtDW.knt < 256; rtDW.knt++) {
        // Delay: '<S31>/CellStateDelay'
        if (rtDW.icLoad_b) {
          rtDW.CellStateDelay_DSTATE_l[rtDW.knt] = 0.0;
        }

        // Product: '<S77>/DivideOut' incorporates:
        //   Delay: '<S31>/CellStateDelay'

        rtDW.DivideOut_f[rtDW.knt] = rtDW.CellStateDelay_DSTATE_l[rtDW.knt];

        // Delay: '<S31>/HiddenStateDelay'
        if (rtDW.icLoad_h) {
          rtDW.HiddenStateDelay_DSTATE_i[rtDW.knt] = 0.0;
        }

        // Trigonometry: '<S89>/Tanh' incorporates:
        //   Delay: '<S31>/HiddenStateDelay'

        rtDW.rtb_Tanh_k[rtDW.knt] = rtDW.HiddenStateDelay_DSTATE_i[rtDW.knt];
      }

      // Sum: '<S36>/Wx+Rh+b' incorporates:
      //   Constant: '<S36>/Bias'
      //   Constant: '<S37>/RecurrentWeights'
      //   Delay: '<S31>/HiddenStateDelay'
      //   Product: '<S29>/W*x'
      //   Product: '<S37>/R*h_t-1'
      //   Selector: '<S28>/Selector1'
      //   Sum: '<S70>/Wx+Rh+b'

      for (rtDW.knt = 0; rtDW.knt < 1024; rtDW.knt++) {
        rtDW.F_rolling = 0.0;
        for (rtDW.i = 0; rtDW.i < 256; rtDW.i++) {
          rtDW.F_rolling += rtConstP.RecurrentWeights_Value[(rtDW.i << 10) +
            rtDW.knt] * rtDW.HiddenStateDelay_DSTATE_i[rtDW.i];
        }

        rtDW.WxRhb[rtDW.knt] = rtDW.Wx[rtDW.knt] + rtDW.F_rolling +
          rtConstP.Bias_Value_g[rtDW.knt];
      }

      // End of Sum: '<S36>/Wx+Rh+b'

      // Outputs for Atomic SubSystem: '<S40>/Sigmoid Layer'
      // Selector: '<S31>/Selector_f' incorporates:
      //   Sum: '<S70>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[256], rtDW.rtb_DivideOut_j_c);

      // End of Outputs for SubSystem: '<S40>/Sigmoid Layer'

      // Product: '<S31>/f*c_t-1' incorporates:
      //   Product: '<S77>/DivideOut'
      //   Product: '<S85>/DivideOut'

      for (rtDW.knt = 0; rtDW.knt <= 254; rtDW.knt += 2) {
        tmp_0 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);
        _mm_storeu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt], _mm_mul_pd(tmp_0, tmp_1));
      }

      // End of Product: '<S31>/f*c_t-1'

      // Outputs for Atomic SubSystem: '<S44>/Sigmoid Layer'
      // Selector: '<S31>/Selector_i' incorporates:
      //   Sum: '<S70>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[0], rtDW.rtb_Tanh_k);

      // End of Outputs for SubSystem: '<S44>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S56>/Tanh Layer'
      // Selector: '<S31>/Selector_g' incorporates:
      //   Sum: '<S70>/Wx+Rh+b'

      TanhLayer(&rtDW.WxRhb[512], rtDW.DivideOut_f);

      // End of Outputs for SubSystem: '<S56>/Tanh Layer'
      for (rtDW.knt = 0; rtDW.knt <= 254; rtDW.knt += 2) {
        // Product: '<S31>/i*g' incorporates:
        //   Product: '<S31>/f*c_t-1'
        //   Product: '<S77>/DivideOut'
        //   Sum: '<S31>/CellAdd'
        //   Trigonometry: '<S89>/Tanh'

        tmp_0 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);

        // Sum: '<S31>/CellAdd' incorporates:
        //   Product: '<S31>/f*c_t-1'

        tmp = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);

        // Sum: '<S31>/CellAdd' incorporates:
        //   Product: '<S31>/f*c_t-1'
        //   Product: '<S31>/i*g'

        _mm_storeu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt], _mm_add_pd(_mm_mul_pd
          (tmp_0, tmp_1), tmp));
      }

      // Outputs for Atomic SubSystem: '<S48>/Sigmoid Layer'
      // Selector: '<S31>/Selector_o' incorporates:
      //   Sum: '<S70>/Wx+Rh+b'

      SigmoidLayer(&rtDW.WxRhb[768], rtDW.rtb_Tanh_k);

      // End of Outputs for SubSystem: '<S48>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S52>/Tanh Layer'
      TanhLayer(rtDW.rtb_DivideOut_j_c, rtDW.DivideOut_f);

      // End of Outputs for SubSystem: '<S52>/Tanh Layer'

      // Update for Delay: '<S31>/CellStateDelay'
      rtDW.icLoad_b = false;

      // Update for Delay: '<S31>/HiddenStateDelay'
      rtDW.icLoad_h = false;
      for (rtDW.knt = 0; rtDW.knt <= 254; rtDW.knt += 2) {
        // Product: '<S31>/HiddenStateProduct' incorporates:
        //   Product: '<S77>/DivideOut'
        //   Trigonometry: '<S89>/Tanh'

        tmp_0 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);
        tmp_0 = _mm_mul_pd(tmp_0, tmp_1);
        _mm_storeu_pd(&rtDW.rtb_Tanh_k[rtDW.knt], tmp_0);

        // Assignment: '<S60>/Assignment' incorporates:
        //   Product: '<S31>/HiddenStateProduct'
        //   Product: '<S77>/DivideOut'

        _mm_storeu_pd(&rtDW.Assignment_l[rtDW.knt], tmp_0);

        // Update for Delay: '<S31>/CellStateDelay' incorporates:
        //   Product: '<S31>/HiddenStateProduct'
        //   Product: '<S77>/DivideOut'
        //   Sum: '<S31>/CellAdd'

        tmp_1 = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);
        _mm_storeu_pd(&rtDW.CellStateDelay_DSTATE_l[rtDW.knt], tmp_1);

        // Update for Delay: '<S31>/HiddenStateDelay' incorporates:
        //   Product: '<S31>/HiddenStateProduct'
        //   Product: '<S77>/DivideOut'

        _mm_storeu_pd(&rtDW.HiddenStateDelay_DSTATE_i[rtDW.knt], tmp_0);
      }

      rtDW.s28_iter++;
    }

    // End of Outputs for SubSystem: '<S19>/ForIteratorSubsystem'

    // Outputs for Atomic SubSystem: '<S9>/lstm_2'
    // Product: '<S63>/W*x' incorporates:
    //   Assignment: '<S60>/Assignment'
    //   Constant: '<S63>/InputWeights'
    //   SignalConversion generated from: '<S15>/In1'

    for (rtDW.knt = 0; rtDW.knt < 512; rtDW.knt++) {
      // Product: '<S63>/W*x' incorporates:
      //   Constant: '<S63>/InputWeights'

      rtDW.Divide2 = 0.0;
      for (rtDW.i = 0; rtDW.i < 256; rtDW.i++) {
        // Outputs for Atomic SubSystem: '<S9>/dropout_1'
        rtDW.Divide2 += rtConstP.InputWeights_Value_g[(rtDW.i << 9) + rtDW.knt] *
          rtDW.Assignment_l[rtDW.i];

        // End of Outputs for SubSystem: '<S9>/dropout_1'
      }

      // Product: '<S63>/W*x' incorporates:
      //   Assignment: '<S60>/Assignment'
      //   Constant: '<S63>/InputWeights'
      //   SignalConversion generated from: '<S15>/In1'

      rtDW.Wx_m[rtDW.knt] = rtDW.Divide2;
    }

    // End of Product: '<S63>/W*x'

    // Outputs for Iterator SubSystem: '<S20>/ForIteratorSubsystem' incorporates:
    //   ForIterator: '<S62>/ForIterator'

    if (rtDW.ProbeDimension[1] < 2.147483648E+9) {
      if (rtDW.ProbeDimension[1] >= -2.147483648E+9) {
        rtDW.b_i = static_cast<int32_t>(rtDW.ProbeDimension[1]);
      } else {
        rtDW.b_i = INT32_MIN;
      }
    } else {
      rtDW.b_i = INT32_MAX;
    }

    if (rtDW.b_i > 2147483646) {
      rtDW.b_i = 2147483646;
    } else if (rtDW.b_i < 0) {
      rtDW.b_i = 0;
    }

    rtDW.s28_iter = 1;
    while (rtDW.s28_iter <= rtDW.b_i) {
      for (rtDW.knt = 0; rtDW.knt < 128; rtDW.knt++) {
        // Delay: '<S65>/CellStateDelay' incorporates:
        //   Product: '<S85>/DivideOut'

        if (rtDW.icLoad) {
          rtDW.CellStateDelay_DSTATE[rtDW.knt] = 0.0;
        }

        rtDW.rtb_DivideOut_j_c[rtDW.knt] = rtDW.CellStateDelay_DSTATE[rtDW.knt];

        // End of Delay: '<S65>/CellStateDelay'

        // Delay: '<S65>/HiddenStateDelay' incorporates:
        //   Trigonometry: '<S89>/Tanh'

        if (rtDW.icLoad_l) {
          rtDW.HiddenStateDelay_DSTATE[rtDW.knt] = 0.0;
        }

        rtDW.rtb_Tanh_k[rtDW.knt] = rtDW.HiddenStateDelay_DSTATE[rtDW.knt];

        // End of Delay: '<S65>/HiddenStateDelay'
      }

      for (rtDW.knt = 0; rtDW.knt < 512; rtDW.knt++) {
        // Product: '<S71>/R*h_t-1' incorporates:
        //   Constant: '<S71>/RecurrentWeights'

        rtDW.Divide2 = 0.0;
        for (rtDW.i = 0; rtDW.i < 128; rtDW.i++) {
          rtDW.Divide2 += rtConstP.RecurrentWeights_Value_b[(rtDW.i << 9) +
            rtDW.knt] * rtDW.rtb_Tanh_k[rtDW.i];
        }

        // Sum: '<S70>/Wx+Rh+b' incorporates:
        //   Constant: '<S70>/Bias'
        //   Product: '<S63>/W*x'
        //   Product: '<S71>/R*h_t-1'
        //   Selector: '<S62>/Selector1'

        rtDW.WxRhb[rtDW.knt] = rtDW.Wx_m[rtDW.knt] + rtDW.Divide2 +
          rtConstP.Bias_Value_f[rtDW.knt];
      }

      // Outputs for Atomic SubSystem: '<S74>/Sigmoid Layer'
      // Selector: '<S65>/Selector_f' incorporates:
      //   Product: '<S77>/DivideOut'

      SigmoidLayer_n(&rtDW.WxRhb[128], &rtDW.DivideOut_f[0]);

      // End of Outputs for SubSystem: '<S74>/Sigmoid Layer'

      // Product: '<S65>/f*c_t-1'
      for (rtDW.knt = 0; rtDW.knt <= 126; rtDW.knt += 2) {
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.DivideOut_f[rtDW.knt]);
        _mm_storeu_pd(&rtDW.fc_t1[rtDW.knt], _mm_mul_pd(tmp_0, tmp_1));
      }

      // End of Product: '<S65>/f*c_t-1'

      // Outputs for Atomic SubSystem: '<S78>/Sigmoid Layer'
      // Selector: '<S65>/Selector_i' incorporates:
      //   Product: '<S85>/DivideOut'

      SigmoidLayer_n(&rtDW.WxRhb[0], &rtDW.rtb_DivideOut_j_c[0]);

      // End of Outputs for SubSystem: '<S78>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S90>/Tanh Layer'
      // Selector: '<S65>/Selector_g' incorporates:
      //   Trigonometry: '<S89>/Tanh'

      TanhLayer_m(&rtDW.WxRhb[256], &rtDW.rtb_Tanh_k[0]);

      // End of Outputs for SubSystem: '<S90>/Tanh Layer'
      for (rtDW.knt = 0; rtDW.knt <= 126; rtDW.knt += 2) {
        // Product: '<S65>/i*g' incorporates:
        //   Product: '<S65>/f*c_t-1'
        //   Sum: '<S65>/CellAdd'

        tmp_0 = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);

        // Sum: '<S65>/CellAdd' incorporates:
        //   Product: '<S65>/f*c_t-1'

        tmp = _mm_loadu_pd(&rtDW.fc_t1[rtDW.knt]);

        // Sum: '<S65>/CellAdd' incorporates:
        //   Product: '<S65>/f*c_t-1'
        //   Product: '<S65>/i*g'

        _mm_storeu_pd(&rtDW.fc_t1[rtDW.knt], _mm_add_pd(_mm_mul_pd(tmp_0, tmp_1),
          tmp));
      }

      // Outputs for Atomic SubSystem: '<S82>/Sigmoid Layer'
      // Selector: '<S65>/Selector_o' incorporates:
      //   Product: '<S85>/DivideOut'

      SigmoidLayer_n(&rtDW.WxRhb[384], &rtDW.rtb_DivideOut_j_c[0]);

      // End of Outputs for SubSystem: '<S82>/Sigmoid Layer'

      // Outputs for Atomic SubSystem: '<S86>/Tanh Layer'
      // Trigonometry: '<S89>/Tanh'
      TanhLayer_m(rtDW.fc_t1, &rtDW.rtb_Tanh_k[0]);

      // End of Outputs for SubSystem: '<S86>/Tanh Layer'

      // Update for Delay: '<S65>/CellStateDelay'
      rtDW.icLoad = false;

      // Update for Delay: '<S65>/HiddenStateDelay'
      rtDW.icLoad_l = false;
      for (rtDW.knt = 0; rtDW.knt <= 126; rtDW.knt += 2) {
        // Product: '<S65>/HiddenStateProduct'
        tmp_0 = _mm_loadu_pd(&rtDW.rtb_Tanh_k[rtDW.knt]);
        tmp_1 = _mm_loadu_pd(&rtDW.rtb_DivideOut_j_c[rtDW.knt]);
        tmp_0 = _mm_mul_pd(tmp_0, tmp_1);

        // Assignment: '<S94>/Assignment' incorporates:
        //   Product: '<S65>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.Assignment[rtDW.knt], tmp_0);

        // Update for Delay: '<S65>/CellStateDelay' incorporates:
        //   Product: '<S65>/HiddenStateProduct'
        //   Sum: '<S65>/CellAdd'

        tmp_1 = _mm_loadu_pd(&rtDW.fc_t1[rtDW.knt]);
        _mm_storeu_pd(&rtDW.CellStateDelay_DSTATE[rtDW.knt], tmp_1);

        // Update for Delay: '<S65>/HiddenStateDelay' incorporates:
        //   Product: '<S65>/HiddenStateProduct'

        _mm_storeu_pd(&rtDW.HiddenStateDelay_DSTATE[rtDW.knt], tmp_0);
      }

      rtDW.s28_iter++;
    }

    // End of Outputs for SubSystem: '<S20>/ForIteratorSubsystem'
    // End of Outputs for SubSystem: '<S9>/lstm_2'

    // Outputs for Atomic SubSystem: '<S9>/fc'
    // Product: '<S17>/Matrix Multiply' incorporates:
    //   Assignment: '<S94>/Assignment'
    //   Constant: '<S17>/Weights'
    //   SignalConversion generated from: '<S16>/In1'

    rtDW.F_rolling = 0.0;
    for (rtDW.knt = 0; rtDW.knt < 128; rtDW.knt++) {
      // Outputs for Atomic SubSystem: '<S9>/dropout_2'
      rtDW.F_rolling += rtConstP.Weights_Value[rtDW.knt] *
        rtDW.Assignment[rtDW.knt];

      // End of Outputs for SubSystem: '<S9>/dropout_2'
    }

    // Outputs for Iterator SubSystem: '<S23>/AddForEachSeq' incorporates:
    //   ForEach: '<S24>/For Each'

    for (rtDW.ForEach_itr = 0; rtDW.ForEach_itr < 1; rtDW.ForEach_itr++) {
      // ForEachSliceAssignment generated from: '<S24>/Out1' incorporates:
      //   Product: '<S17>/Matrix Multiply'
      //   Sum: '<S24>/Add'

      rtDW.ImpAsg_InsertedFor_Out1_at_ = rtDW.F_rolling - 0.015775660052895546;
    }

    // End of Outputs for SubSystem: '<S23>/AddForEachSeq'
    // End of Outputs for SubSystem: '<S9>/fc'

    // Outputs for Enabled SubSystem: '<S8>/Correct1' incorporates:
    //   EnablePort: '<S11>/Enable'

    // MATLABSystem: '<S11>/MATLAB System' incorporates:
    //   Constant: '<S8>/R1'
    //   DataStoreRead: '<S11>/Data Store ReadP'
    //   DataStoreRead: '<S11>/Data Store ReadX'

    //  ������ SOC�� �״�� �������� ����
    rtDW.F_rolling = std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 *
      std::abs(rtDW.x));

    //  ������ SOC�� �״�� �������� ����
    rtDW.F_rolling = (rtDW.x + rtDW.F_rolling - rtDW.x) / rtDW.F_rolling;

    //  ������ SOC�� �״�� �������� ����
    rtDW.Add13 = qrFactor(rtDW.F_rolling, rtDW.P_k, 0.1);
    rtDW.Add13 = trisolve(rtDW.Add13, trisolve(rtDW.Add13, (rtDW.P_k * rtDW.P_k)
      * rtDW.F_rolling));

    // DataStoreWrite: '<S11>/Data Store WriteP' incorporates:
    //   Constant: '<S8>/R1'
    //   DataStoreRead: '<S11>/Data Store ReadP'
    //   MATLABSystem: '<S11>/MATLAB System'
    //
    rtDW.P_k = qrFactor((-rtDW.Add13 * rtDW.F_rolling) + 1.0, rtDW.P_k,
                        rtDW.Add13 * 0.1);

    // Outputs for Atomic SubSystem: '<S9>/layer'
    // DataStoreWrite: '<S11>/Data Store WriteX' incorporates:
    //   Constant: '<S27>/Constant'
    //   DataStoreRead: '<S11>/Data Store ReadX'
    //   Gain: '<S27>/Gain'
    //   MATLABSystem: '<S11>/MATLAB System'
    //   Math: '<S27>/Exp'
    //   Product: '<S27>/DivideOut'
    //   Sum: '<S27>/Add'
    //  *
    //  About '<S27>/Exp':
    //   Operator: exp

    rtDW.x += ((1.0 / (std::exp(-rtDW.ImpAsg_InsertedFor_Out1_at_) + 1.0)) -
               rtDW.x) * rtDW.Add13;

    // End of Outputs for SubSystem: '<S9>/layer'
    // End of Outputs for SubSystem: '<S8>/Correct1'

    // Outport: '<Root>/SOC' incorporates:
    //   DataStoreRead: '<S12>/Data Store Read'
    //   Gain: '<S1>/Gain5'

    rtY.SOC = 100.0 * rtDW.x;

    // Stop: '<S1>/Stop Simulation' incorporates:
    //   Constant: '<S3>/Constant'
    //   Outport: '<Root>/SOC'
    //   RelationalOperator: '<S3>/Compare'

    if (rtY.SOC <= 20.0) {
      (&rtM)->setStopRequested(1);
    }

    // End of Stop: '<S1>/Stop Simulation'

    // Sum: '<S5>/Minus1' incorporates:
    //   Constant: '<S5>/Constant13'
    //   Constant: '<S5>/Constant14'
    //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
    //   Product: '<S5>/Divide10'

    rtDW.Minus1 = 1.0 - (rtDW.DiscreteTimeIntegrator_DSTATE / 180000.0);

    // DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTATE_f;

    // DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    rtDW.ImpAsg_InsertedFor_Out1_at_ = rtDW.DiscreteTimeIntegrator1_DSTATE;
  }

  // End of Outputs for SubSystem: '<S9>/lstm_1'
  // End of Outputs for SubSystem: '<S1>/LSTM'

  // Product: '<S1>/Divide15' incorporates:
  //   Inport: '<Root>/Mass_kg'

  rtDW.Veh_acceleartion = rtDW.DiscreteTimeIntegrator_d / rtU.Mass_kg;
  if (b) {
    // Outputs for Atomic SubSystem: '<S8>/Predict'
    // MATLABSystem: '<S13>/MATLAB System' incorporates:
    //   Constant: '<S8>/Q'
    //   DataStoreRead: '<S13>/Data Store ReadP'
    //   DataStoreRead: '<S13>/Data Store ReadX'

    rtDW.M[0] = ((rtDW.Minus1 - rtDW.Minus1) / std::fmax(1.4901161193847656E-8,
      1.4901161193847656E-8 * std::abs(rtDW.x))) * rtDW.P_k;
    rtDW.M[1] = 0.022360679774997897;
    for (rtDW.b_i = 0; rtDW.b_i < 1; rtDW.b_i++) {
      rtDW.Add13 = rtDW.M[0];
      rtDW.DiscreteTimeIntegrator_d = xnrm2(1, rtDW.M, 2);
      if (rtDW.DiscreteTimeIntegrator_d != 0.0) {
        rtDW.Divide2 = rt_hypotd_snf_e(rtDW.M[0], rtDW.DiscreteTimeIntegrator_d);
        if (rtDW.M[0] >= 0.0) {
          rtDW.Divide2 = -rtDW.Divide2;
        }

        if (std::abs(rtDW.Divide2) < 1.0020841800044864E-292) {
          rtDW.knt = -1;
          do {
            rtDW.knt++;
            rtDW.DiscreteTimeIntegrator_d = rtDW.M[1];
            for (rtDW.s28_iter = 2; rtDW.s28_iter < 3; rtDW.s28_iter++) {
              rtDW.DiscreteTimeIntegrator_d *= 9.9792015476736E+291;
            }

            rtDW.M[1] = rtDW.DiscreteTimeIntegrator_d;
            rtDW.Divide2 *= 9.9792015476736E+291;
            rtDW.Add13 *= 9.9792015476736E+291;
          } while ((std::abs(rtDW.Divide2) < 1.0020841800044864E-292) &&
                   ((rtDW.knt + 1) < 20));

          rtDW.Divide2 = rt_hypotd_snf_e(rtDW.Add13, xnrm2(1, rtDW.M, 2));
          if (rtDW.Add13 >= 0.0) {
            rtDW.Divide2 = -rtDW.Divide2;
          }

          rtDW.Add13 = 1.0 / (rtDW.Add13 - rtDW.Divide2);
          for (rtDW.s28_iter = 2; rtDW.s28_iter < 3; rtDW.s28_iter++) {
            rtDW.DiscreteTimeIntegrator_d *= rtDW.Add13;
          }

          rtDW.M[1] = rtDW.DiscreteTimeIntegrator_d;
          for (rtDW.s28_iter = 0; rtDW.s28_iter <= rtDW.knt; rtDW.s28_iter++) {
            rtDW.Divide2 *= 1.0020841800044864E-292;
          }

          rtDW.Add13 = rtDW.Divide2;
        } else {
          rtDW.Add13 = 1.0 / (rtDW.M[0] - rtDW.Divide2);
          rtDW.DiscreteTimeIntegrator_d = rtDW.M[1];
          for (rtDW.s28_iter = 2; rtDW.s28_iter < 3; rtDW.s28_iter++) {
            rtDW.DiscreteTimeIntegrator_d *= rtDW.Add13;
          }

          rtDW.M[1] = rtDW.DiscreteTimeIntegrator_d;
          rtDW.Add13 = rtDW.Divide2;
        }
      }

      rtDW.M[0] = rtDW.Add13;
    }

    // DataStoreWrite: '<S13>/Data Store WriteP' incorporates:
    //   MATLABSystem: '<S13>/MATLAB System'
    //
    rtDW.P_k = rtDW.M[0];

    // DataStoreWrite: '<S13>/Data Store WriteX' incorporates:
    //   MATLABSystem: '<S13>/MATLAB System'
    //
    rtDW.x = rtDW.Minus1;

    // End of Outputs for SubSystem: '<S8>/Predict'
  }

  // Gain: '<S127>/Integral Gain'
  rtDW.IntegralGain = 0.01 * rtDW.SpeedDifferent;

  // Step: '<S1>/Step'
  rtDW.Step = !((&rtM)->Timing.t[0] < 10.0);

  // Switch: '<S1>/SOC switch'
  if (rtDW.Step >= 1.0) {
    // Switch: '<S1>/SOC switch' incorporates:
    //   Outport: '<Root>/SOC'

    rtDW.SOCswitch = rtY.SOC;
  } else {
    // Switch: '<S1>/SOC switch'
    rtDW.SOCswitch = rtDW.Minus1;
  }

  // End of Switch: '<S1>/SOC switch'
  // End of Outputs for SubSystem: '<Root>/BMS with LSTM'

  // Outport: '<Root>/Accel[ms2]'
  rtY.Accelms2 = rtDW.Veh_acceleartion;
  if ((&rtM)->isMajorTimeStep()) {
    // Outport: '<Root>/Speed[ms]'
    rtY.Speedms = rtDW.DiscreteTimeIntegrator;

    // Outport: '<Root>/Position[m]'
    rtY.Positionm = rtDW.ImpAsg_InsertedFor_Out1_at_;
  }

  if ((&rtM)->isMajorTimeStep()) {
    // Update for Atomic SubSystem: '<Root>/BMS with LSTM'
    if ((&rtM)->isMajorTimeStep()) {
      // Update for UnitDelay: '<S1>/SOC_t-2'
      rtDW.SOC_t2_DSTATE = rtDW.SOCswitch;

      // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE += 0.01 * rtDW.Motor_Current_cell;

      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE_f += 0.01 * rtDW.Veh_acceleartion;

      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1_DSTATE += 0.01 * rtDW.DiscreteTimeIntegrator;
    }

    // End of Update for SubSystem: '<Root>/BMS with LSTM'
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
      // Update absolute timer for sample time: [0.01s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.01, which is the step size
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

  // Derivatives for Atomic SubSystem: '<Root>/BMS with LSTM'
  // Derivatives for Integrator: '<S130>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.IntegralGain;

  // Derivatives for Integrator: '<S125>/Filter'
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;

  // End of Derivatives for SubSystem: '<Root>/BMS with LSTM'
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
  (&rtM)->Timing.stepSize0 = 0.01;

  // SystemInitialize for Atomic SubSystem: '<Root>/BMS with LSTM'
  // Start for FromWorkspace: '<S1>/From Workspace19'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.7 };

    rtDW.FromWorkspace19_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace19_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace19_IWORK.PrevIndex = 0;
  }

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

  // Start for FromWorkspace: '<S1>/Friction_Loss1'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 0.21857 };

    rtDW.Friction_Loss1_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.Friction_Loss1_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.Friction_Loss1_IWORK.PrevIndex = 0;
  }

  // Start for FromWorkspace: '<S1>/From Workspace4'
  {
    static double pTimeValues0[]{ 1.0 } ;

    static double pDataValues0[]{ 9.81 };

    rtDW.FromWorkspace4_PWORK.TimePtr = static_cast<void *>(pTimeValues0);
    rtDW.FromWorkspace4_PWORK.DataPtr = static_cast<void *>(pDataValues0);
    rtDW.FromWorkspace4_IWORK.PrevIndex = 0;
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

  // Start for DataStoreMemory: '<S8>/DataStoreMemory - P'
  rtDW.P_k = 0.1414213562373095;

  // Start for DataStoreMemory: '<S8>/DataStoreMemory - x'
  rtDW.x = 1.0;

  // InitializeConditions for Integrator: '<S130>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S125>/Filter'
  rtX.Filter_CSTATE = 0.0;

  // SystemInitialize for Atomic SubSystem: '<S1>/LSTM'
  // SystemInitialize for Atomic SubSystem: '<S9>/lstm_1'
  // SystemInitialize for Iterator SubSystem: '<S19>/ForIteratorSubsystem'
  // Start for Probe: '<S28>/Probe Dimension'
  rtDW.ProbeDimension_o[0] = 1024.0;
  rtDW.ProbeDimension_o[1] = 1.0;

  // InitializeConditions for Delay: '<S31>/CellStateDelay'
  rtDW.icLoad_b = true;

  // InitializeConditions for Delay: '<S31>/HiddenStateDelay'
  rtDW.icLoad_h = true;

  // End of SystemInitialize for SubSystem: '<S19>/ForIteratorSubsystem'
  // End of SystemInitialize for SubSystem: '<S9>/lstm_1'

  // SystemInitialize for Atomic SubSystem: '<S9>/lstm_2'
  // SystemInitialize for Iterator SubSystem: '<S20>/ForIteratorSubsystem'
  // Start for Probe: '<S62>/Probe Dimension'
  rtDW.ProbeDimension[0] = 512.0;
  rtDW.ProbeDimension[1] = 1.0;

  // InitializeConditions for Delay: '<S65>/CellStateDelay'
  rtDW.icLoad = true;

  // InitializeConditions for Delay: '<S65>/HiddenStateDelay'
  rtDW.icLoad_l = true;

  // End of SystemInitialize for SubSystem: '<S20>/ForIteratorSubsystem'
  // End of SystemInitialize for SubSystem: '<S9>/lstm_2'
  // End of SystemInitialize for SubSystem: '<S1>/LSTM'
  // End of SystemInitialize for SubSystem: '<Root>/BMS with LSTM'
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
