//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS.cpp
//
// Code generated for Simulink model 'BMS'.
//
// Model version                  : 10.119
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Dec  1 19:48:07 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "BMS.h"
#include <cmath>
#include <stdbool.h>
#include "limits"
#include "cmath"

// Used by FromWorkspace Block: '<S1>/From Workspace19'
#ifndef rtInterpolate
# define rtInterpolate(v1,v2,f1,f2)    (((v1)==(v2))?((double)(v1)): (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif

#ifndef rtRound
# define rtRound(v)                    ( ((v) >= 0) ? std::floor((v) + 0.5) : std::ceil((v) - 0.5) )
#endif

static double look1_binlg(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);
static double look2_binlg(double u0, double u1, const double bp0[], const double
  bp1[], const double table[], const uint32_t maxIndex[], uint32_t stride);
static double look1_binlag(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);

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

// Model step function
void BMS::step()
{
  // local block i/o variables
  double rtb_Rolling_Loss1;
  double rtb_FromWorkspace3;
  double rtb_SpeedDifferent;
  double rtb_UnitConversion4;
  double rtb_uDLookupTable1;
  double rtb_Divide39;
  double rtb_DiscreteTimeIntegrator;
  if ((&rtM)->isMajorTimeStep()) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if ((&rtM)->isMinorTimeStep()) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  {
    bool tmp;

    // Outputs for Atomic SubSystem: '<Root>/BMS with C.C'
    // FromWorkspace: '<S1>/From Workspace19'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace19_PWORK.DataPtr };

      rtb_SpeedDifferent = pDataValues[0];
    }

    // Sum: '<S1>/Add13' incorporates:
    //   Constant: '<S1>/Constant8'

    rtDW.Add13 = 1.0 - rtb_SpeedDifferent;

    // Sum: '<S1>/Speed Different' incorporates:
    //   Gain: '<S1>/Gain1'
    //   Inport: '<Root>/ego_velocity'
    //   Inport: '<Root>/target_velocity'

    rtb_SpeedDifferent = rtU.target_velocity - (3.6 * rtU.ego_velocity);

    // Gain: '<S45>/Filter Coefficient' incorporates:
    //   Gain: '<S35>/Derivative Gain'
    //   Integrator: '<S37>/Filter'
    //   Sum: '<S37>/SumD'

    rtDW.FilterCoefficient = ((0.0001 * rtb_SpeedDifferent) - rtX.Filter_CSTATE)
      * 10.0;

    // Sum: '<S1>/Add14' incorporates:
    //   Gain: '<S1>/Accel2'
    //   Gain: '<S47>/Proportional Gain'
    //   Integrator: '<S42>/Integrator'
    //   Sum: '<S51>/Sum'

    rtDW.rtb_Add14_m = ((100.0 * rtb_SpeedDifferent) + rtX.Integrator_CSTATE +
                        rtDW.FilterCoefficient) * 100.0;

    // Saturate: '<S1>/Brake[%]1'
    if (rtDW.rtb_Add14_m > 0.0) {
      rtDW.F_rolling = 0.0;
    } else if (rtDW.rtb_Add14_m < -100.0) {
      rtDW.F_rolling = -100.0;
    } else {
      rtDW.F_rolling = rtDW.rtb_Add14_m;
    }

    // End of Saturate: '<S1>/Brake[%]1'

    // Product: '<S1>/Divide25'
    rtDW.Add13 *= rtDW.F_rolling;

    // FromWorkspace: '<S1>/From Workspace1'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace1_PWORK.DataPtr };

      rtb_uDLookupTable1 = pDataValues[0];
    }

    // Product: '<S1>/Divide2'
    rtDW.Divide2 = rtDW.F_rolling * rtb_uDLookupTable1;

    // FromWorkspace: '<S1>/From Workspace21'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace21_PWORK.DataPtr };

      rtb_uDLookupTable1 = pDataValues[0];
    }

    // Product: '<S1>/Divide36' incorporates:
    //   Constant: '<S1>/Constant10'
    //   Constant: '<S1>/Constant9'
    //   Inport: '<Root>/ego_velocity'
    //   Product: '<S1>/Divide35'

    rtDW.F_rolling = rtU.ego_velocity / (6.2831853071795862 * rtb_uDLookupTable1);

    // FromWorkspace: '<S1>/From Workspace22'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace22_PWORK.DataPtr };

      rtb_uDLookupTable1 = pDataValues[0];
    }

    // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
    //   Gain: '<S1>/Gain20'
    //   Product: '<S1>/Divide37'

    rtb_uDLookupTable1 = (rtDW.F_rolling * rtb_uDLookupTable1) * 60.0;

    // Lookup_n-D: '<S1>/MAX Torque1' incorporates:
    //   Lookup_n-D: '<S1>/2-D Lookup Table1'

    rtDW.F_rolling = look1_binlg(rtb_uDLookupTable1,
      rtConstP.MAXTorque1_bp01Data, rtConstP.MAXTorque1_tableData, 5U);

    // Saturate: '<S1>/Accel[%]1'
    if (rtDW.rtb_Add14_m > 100.0) {
      // Sum: '<S1>/Add14'
      rtDW.rtb_Add14_m = 100.0;
    } else if (rtDW.rtb_Add14_m < 0.0) {
      // Sum: '<S1>/Add14'
      rtDW.rtb_Add14_m = 0.0;
    }

    // End of Saturate: '<S1>/Accel[%]1'

    // Product: '<S1>/Divide1' incorporates:
    //   Gain: '<S1>/Accel1'

    rtDW.Motor_TorqueNm_g = (0.01 * rtDW.rtb_Add14_m) * rtDW.F_rolling;

    // FromWorkspace: '<S1>/From Workspace18'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace18_PWORK.DataPtr };

      rtDW.rtb_Add14_m = pDataValues[0];
    }

    // Product: '<S1>/Divide33' incorporates:
    //   Gain: '<S1>/Gain13'
    //   Product: '<S1>/Divide27'
    //   Sum: '<S1>/Add1'

    rtDW.F_rolling = (((0.01 * rtDW.Divide2) * rtDW.F_rolling) +
                      rtDW.Motor_TorqueNm_g) * rtDW.rtb_Add14_m;

    // FromWorkspace: '<S1>/From Workspace20'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace20_PWORK.DataPtr };

      rtDW.rtb_Add14_m = pDataValues[0];
    }

    // Sum: '<S1>/Add14' incorporates:
    //   Gain: '<S1>/BPP[%]*MAX_Brake_Force[N]1'
    //   Product: '<S1>/Divide34'

    rtDW.rtb_Add14_m = (100.0 * rtDW.Add13) + (rtDW.F_rolling / rtDW.rtb_Add14_m);

    // FromWorkspace: '<S1>/Rolling_Loss1'
    {
      double *pDataValues{ (double *) rtDW.Rolling_Loss1_PWORK.DataPtr };

      rtb_Rolling_Loss1 = pDataValues[0];
    }

    // FromWorkspace: '<S1>/From Workspace3'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace3_PWORK.DataPtr };

      rtb_FromWorkspace3 = pDataValues[0];
    }

    // FromWorkspace: '<S1>/Friction_Loss1'
    {
      double *pDataValues{ (double *) rtDW.Friction_Loss1_PWORK.DataPtr };

      rtb_UnitConversion4 = pDataValues[0];
    }

    // Product: '<S1>/Divide22' incorporates:
    //   Inport: '<Root>/ego_velocity'

    rtDW.Add13 = rtb_UnitConversion4 * rtU.ego_velocity;

    // FromWorkspace: '<S1>/From Workspace4'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace4_PWORK.DataPtr };

      rtb_UnitConversion4 = pDataValues[0];
    }

    // Product: '<S1>/Divide4' incorporates:
    //   Inport: '<Root>/Mass_kg'

    rtDW.Divide2 = (rtU.Mass_kg * rtb_UnitConversion4) * 0.0;
    tmp = ((&rtM)->isMajorTimeStep());
    if (tmp) {
      // Saturate: '<S1>/Saturation2' incorporates:
      //   Inport: '<Root>/IVD'

      if (rtU.IVD > 100.0) {
        rtDW.Motor_TorqueNm_g = 100.0;
      } else if (rtU.IVD < 0.0) {
        rtDW.Motor_TorqueNm_g = 0.0;
      } else {
        rtDW.Motor_TorqueNm_g = rtU.IVD;
      }

      // End of Saturate: '<S1>/Saturation2'

      // Switch: '<S1>/Switch5' incorporates:
      //   Inport: '<Root>/Mode'

      if (rtU.Mode >= 1.0) {
        // Outport: '<Root>/Aero' incorporates:
        //   MATLAB Function: '<S1>/Drag coefficient for FV'

        rtY.Aero = (std::exp((rtDW.Motor_TorqueNm_g - 8.0) * -0.068) * -0.4629)
          + 0.6441;
      } else {
        // Outport: '<Root>/Aero' incorporates:
        //   MATLAB Function: '<S1>/Drag coefficient for LV'

        rtY.Aero = (std::exp((rtDW.Motor_TorqueNm_g - 8.0) * -0.1002) * -0.1012)
          + 0.6441;
      }

      // End of Switch: '<S1>/Switch5'
    }

    // FromWorkspace: '<S1>/From Workspace'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace_PWORK.DataPtr };

      rtb_UnitConversion4 = pDataValues[0];
    }

    // FromWorkspace: '<S1>/From Workspace2'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace2_PWORK.DataPtr };

      rtb_Divide39 = pDataValues[0];
    }

    // Product: '<S1>/Divide23' incorporates:
    //   Inport: '<Root>/ego_velocity'
    //   Math: '<S1>/Square1'
    //   Outport: '<Root>/Aero'

    rtDW.Motor_TorqueNm_g = (((rtU.ego_velocity * rtU.ego_velocity) * rtY.Aero) *
      rtb_UnitConversion4) * rtb_Divide39;

    // FromWorkspace: '<S1>/From Workspace15'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace15_PWORK.DataPtr };

      rtb_Divide39 = pDataValues[0];
    }

    // Switch: '<S1>/Switch1' incorporates:
    //   Constant: '<S1>/Constant3'
    //   Constant: '<S3>/Constant'
    //   Inport: '<Root>/Mass_kg'
    //   Inport: '<Root>/ego_velocity'
    //   Product: '<S1>/Divide3'
    //   RelationalOperator: '<S3>/Compare'

    if (rtU.ego_velocity > 1.0) {
      rtDW.F_rolling = (rtb_Rolling_Loss1 * rtb_FromWorkspace3) * rtU.Mass_kg;
    } else {
      rtDW.F_rolling = 10.0;
    }

    // Product: '<S1>/Divide28' incorporates:
    //   Gain: '<S1>/Gain'
    //   Sum: '<S1>/Add11'
    //   Sum: '<S1>/Add15'
    //   Switch: '<S1>/Switch1'

    rtDW.F_rolling = (rtDW.F_rolling + rtDW.Add13 + rtDW.Divide2 + (0.5 *
      rtDW.Motor_TorqueNm_g) + rtDW.rtb_Add14_m) * rtb_Divide39;

    // FromWorkspace: '<S1>/From Workspace14'
    {
      double *pDataValues{ (double *) rtDW.FromWorkspace14_PWORK.DataPtr };

      rtb_Divide39 = pDataValues[0];
    }

    // Product: '<S1>/Divide39' incorporates:
    //   Product: '<S1>/Divide29'

    rtb_Divide39 = rtDW.F_rolling / rtb_Divide39;

    // UnitConversion: '<S1>/Unit Conversion4'
    // Unit Conversion - from: rpm to: rad/s
    // Expression: output = (0.10472*input) + (0)
    rtb_UnitConversion4 = 0.10471975511965977 * rtb_uDLookupTable1;

    // Lookup_n-D: '<S1>/2-D Lookup Table1' incorporates:
    //   Product: '<S1>/Divide39'

    rtb_uDLookupTable1 = look2_binlg(rtb_uDLookupTable1, rtb_Divide39,
      rtConstP.uDLookupTable1_bp01Data, rtConstP.uDLookupTable1_bp02Data,
      rtConstP.uDLookupTable1_tableData, rtConstP.uDLookupTable1_maxIndex, 51U);

    // Product: '<S1>/Divide39' incorporates:
    //   Gain: '<S1>/Gain23'
    //   Product: '<S1>/Divide38'

    rtb_Divide39 = (rtb_Divide39 * rtb_UnitConversion4) / (0.01 *
      rtb_uDLookupTable1);
    if (tmp) {
      // Gain: '<S1>/Gain7' incorporates:
      //   UnitDelay: '<S1>/SOC_t-1'

      rtDW.F_rolling = 0.01 * rtDW.SOC_t1_DSTATE;

      // Lookup_n-D: '<S1>/SOC-Vol' incorporates:
      //   Gain: '<S7>/Gain1'

      rtDW.V0_cell = look1_binlag(rtDW.F_rolling, rtConstP.pooled6,
        rtConstP.SOCVol_tableData, 100U);

      // Gain: '<S1>/V0_pack' incorporates:
      //   Lookup_n-D: '<S1>/SOC-Vol'

      rtDW.V0_pack = 18.0 * rtDW.V0_cell;

      // Lookup_n-D: '<S1>/SOC-R0' incorporates:
      //   Gain: '<S7>/Gain1'

      rtDW.R0_cell = look1_binlag(rtDW.F_rolling, rtConstP.pooled6,
        rtConstP.SOCR0_tableData, 100U);
    }

    // Gain: '<S1>/Gain5' incorporates:
    //   Gain: '<S1>/I_Cell'
    //   Gain: '<S1>/V_pack'
    //   Lookup_n-D: '<S1>/SOC-R0'
    //   Lookup_n-D: '<S1>/SOC-Vol'
    //   Product: '<S1>/Divide12'
    //   Product: '<S1>/Divide5'
    //   Product: '<S1>/Divide6'
    //   Sum: '<S1>/Minus'

    rtDW.Add13 = -(rtb_Divide39 / ((rtDW.V0_cell - (((rtb_Divide39 /
      rtDW.V0_pack) * 0.055555555555555552) * rtDW.R0_cell)) * 18.0));

    // Outputs for Iterator SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' incorporates:
    //   ForEach: '<S7>/For Each'

    if ((&rtM)->isMajorTimeStep()) {
      // Saturate: '<S7>/Prevent division by 0'
      rtDW.CoreSubsys[0].Preventdivisionby0 = 3.266568E+6;

      // DiscreteIntegrator: '<S61>/Integrator' incorporates:
      //   Constant: '<S1>/Constant5'
      //   ForEachSliceSelector generated from: '<S7>/InitialSOC'

      if (rtDW.CoreSubsys[0].Integrator_IC_LOADING != 0) {
        rtDW.CoreSubsys[0].Integrator_DSTATE = 1.0;
      }

      // ForEachSliceAssignment generated from: '<S7>/SOC' incorporates:
      //   DiscreteIntegrator: '<S61>/Integrator'

      rtDW.ImpAsg_InsertedFor_SOC_at_i = rtDW.CoreSubsys[0].Integrator_DSTATE;
    }

    // Product: '<S7>/Product'
    rtDW.CoreSubsys[0].Product = rtDW.Add13 / rtDW.CoreSubsys[0].
      Preventdivisionby0;

    // End of Outputs for SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
    if (tmp) {
      // Gain: '<S1>/Gain3'
      rtDW.Gain3 = 100.0 * rtDW.ImpAsg_InsertedFor_SOC_at_i;

      // Stop: '<S1>/Stop Simulation' incorporates:
      //   Constant: '<S2>/Constant'
      //   RelationalOperator: '<S2>/Compare'

      if (rtDW.Gain3 <= 90.0) {
        (&rtM)->setStopRequested(1);
      }

      // End of Stop: '<S1>/Stop Simulation'

      // DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
      rtb_DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTATE;

      // DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1 = rtDW.DiscreteTimeIntegrator1_DSTATE;
    }

    // Product: '<S1>/Divide15' incorporates:
    //   Inport: '<Root>/Mass_kg'

    rtDW.Veh_acceleartion = rtDW.rtb_Add14_m / rtU.Mass_kg;

    // Gain: '<S39>/Integral Gain'
    rtDW.IntegralGain = 0.01 * rtb_SpeedDifferent;

    // End of Outputs for SubSystem: '<Root>/BMS with C.C'

    // Outport: '<Root>/Accel[ms2]'
    rtY.Accelms2 = rtDW.Veh_acceleartion;
    if ((&rtM)->isMajorTimeStep()) {
      // Outport: '<Root>/Speed[ms]'
      rtY.Speedms = rtb_DiscreteTimeIntegrator;

      // Outport: '<Root>/Position[m]'
      rtY.Positionm = rtDW.DiscreteTimeIntegrator1;

      // Outport: '<Root>/Battery_SOC[%]'
      rtY.Battery_SOC = rtDW.Gain3;
    }
  }

  if ((&rtM)->isMajorTimeStep()) {
    bool tmp;

    // Update for Atomic SubSystem: '<Root>/BMS with C.C'
    tmp = ((&rtM)->isMajorTimeStep());
    if (tmp) {
      // Update for UnitDelay: '<S1>/SOC_t-1' incorporates:
      //   Inport: '<Root>/soc_ego'

      rtDW.SOC_t1_DSTATE = rtU.soc_ego;
    }

    // Update for Iterator SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
    if ((&rtM)->isMajorTimeStep()) {
      // Update for DiscreteIntegrator: '<S61>/Integrator'
      rtDW.CoreSubsys[0].Integrator_IC_LOADING = 0U;
      rtDW.CoreSubsys[0].Integrator_DSTATE += 0.01 * rtDW.CoreSubsys[0].Product;
    }

    // End of Update for SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
    if (tmp) {
      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE += 0.01 * rtDW.Veh_acceleartion;

      // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1_DSTATE += 0.01 * rtb_DiscreteTimeIntegrator;
    }

    // End of Update for SubSystem: '<Root>/BMS with C.C'
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

  // Derivatives for Atomic SubSystem: '<Root>/BMS with C.C'
  // Derivatives for Integrator: '<S42>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.IntegralGain;

  // Derivatives for Integrator: '<S37>/Filter'
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;

  // End of Derivatives for SubSystem: '<Root>/BMS with C.C'
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

  // SystemInitialize for Atomic SubSystem: '<Root>/BMS with C.C'
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

  // InitializeConditions for Integrator: '<S42>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S37>/Filter'
  rtX.Filter_CSTATE = 0.0;

  // SystemInitialize for Iterator SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
  // InitializeConditions for DiscreteIntegrator: '<S61>/Integrator'
  rtDW.CoreSubsys[0].Integrator_IC_LOADING = 1U;

  // End of SystemInitialize for SubSystem: '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
  // End of SystemInitialize for SubSystem: '<Root>/BMS with C.C'
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

//
// File trailer for generated code.
//
// [EOF]
//
