//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: BMS.h
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
#ifndef BMS_h_
#define BMS_h_
#include <stdbool.h>
#include <stdint.h>
#include <cmath>
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include <cstring>
#define BMS_M                          (rtM)
#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
struct ODE3_IntgData {
  double *y;                           // output
  double *f[3];                        // derivatives
};

#endif

// Class declaration for model BMS
class BMS final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)' 
  struct DW_CoreSubsys {
    double Preventdivisionby0;         // '<S7>/Prevent division by 0'
    double Product;                    // '<S7>/Product'
    double Integrator_DSTATE;          // '<S61>/Integrator'
    uint8_t Integrator_IC_LOADING;     // '<S61>/Integrator'
  };

  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    DW_CoreSubsys CoreSubsys[1];
                  // '<S1>/SOC Estimator (Coulomb Counting, Variable Capacity)'
    double FilterCoefficient;          // '<S45>/Filter Coefficient'
    double V0_cell;                    // '<S1>/SOC-Vol'
    double V0_pack;                    // '<S1>/V0_pack'
    double R0_cell;                    // '<S1>/SOC-R0'
    double Veh_acceleartion;           // '<S1>/Divide15'
    double IntegralGain;               // '<S39>/Integral Gain'
    double SOC_t1_DSTATE;              // '<S1>/SOC_t-1'
    double DiscreteTimeIntegrator_DSTATE;// '<S1>/Discrete-Time Integrator'
    double DiscreteTimeIntegrator1_DSTATE;// '<S1>/Discrete-Time Integrator1'
    double Add13;                      // '<S1>/Add13'
    double F_rolling;                  // '<S1>/Switch1'
    double Divide2;                    // '<S1>/Divide2'
    double Motor_TorqueNm_g;           // '<S1>/Divide1'
    double Gain3;                      // '<S1>/Gain3'
    double ImpAsg_InsertedFor_SOC_at_i;// '<S7>/Integrator'
    double DiscreteTimeIntegrator1;    // '<S1>/Discrete-Time Integrator1'
    double rtb_Add14_m;
    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace19_PWORK;           // '<S1>/From Workspace19'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace1_PWORK;            // '<S1>/From Workspace1'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace21_PWORK;           // '<S1>/From Workspace21'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace22_PWORK;           // '<S1>/From Workspace22'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace18_PWORK;           // '<S1>/From Workspace18'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace20_PWORK;           // '<S1>/From Workspace20'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } Rolling_Loss1_PWORK;             // '<S1>/Rolling_Loss1'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace3_PWORK;            // '<S1>/From Workspace3'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } Friction_Loss1_PWORK;            // '<S1>/Friction_Loss1'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace4_PWORK;            // '<S1>/From Workspace4'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace_PWORK;             // '<S1>/From Workspace'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace2_PWORK;            // '<S1>/From Workspace2'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace15_PWORK;           // '<S1>/From Workspace15'

    struct {
      void *TimePtr;
      void *DataPtr;
      void *RSimInfoPtr;
    } FromWorkspace14_PWORK;           // '<S1>/From Workspace14'

    struct {
      int PrevIndex;
    } FromWorkspace19_IWORK;           // '<S1>/From Workspace19'

    struct {
      int PrevIndex;
    } FromWorkspace1_IWORK;            // '<S1>/From Workspace1'

    struct {
      int PrevIndex;
    } FromWorkspace21_IWORK;           // '<S1>/From Workspace21'

    struct {
      int PrevIndex;
    } FromWorkspace22_IWORK;           // '<S1>/From Workspace22'

    struct {
      int PrevIndex;
    } FromWorkspace18_IWORK;           // '<S1>/From Workspace18'

    struct {
      int PrevIndex;
    } FromWorkspace20_IWORK;           // '<S1>/From Workspace20'

    struct {
      int PrevIndex;
    } Rolling_Loss1_IWORK;             // '<S1>/Rolling_Loss1'

    struct {
      int PrevIndex;
    } FromWorkspace3_IWORK;            // '<S1>/From Workspace3'

    struct {
      int PrevIndex;
    } Friction_Loss1_IWORK;            // '<S1>/Friction_Loss1'

    struct {
      int PrevIndex;
    } FromWorkspace4_IWORK;            // '<S1>/From Workspace4'

    struct {
      int PrevIndex;
    } FromWorkspace_IWORK;             // '<S1>/From Workspace'

    struct {
      int PrevIndex;
    } FromWorkspace2_IWORK;            // '<S1>/From Workspace2'

    struct {
      int PrevIndex;
    } FromWorkspace15_IWORK;           // '<S1>/From Workspace15'

    struct {
      int PrevIndex;
    } FromWorkspace14_IWORK;           // '<S1>/From Workspace14'
  };

  // Continuous states (default storage)
  struct X {
    double Integrator_CSTATE;          // '<S42>/Integrator'
    double Filter_CSTATE;              // '<S37>/Filter'
  };

  // State derivatives (default storage)
  struct XDot {
    double Integrator_CSTATE;          // '<S42>/Integrator'
    double Filter_CSTATE;              // '<S37>/Filter'
  };

  // State disabled
  struct XDis {
    bool Integrator_CSTATE;            // '<S42>/Integrator'
    bool Filter_CSTATE;                // '<S37>/Filter'
  };

  // Constant parameters (default storage)
  struct ConstP {
    // Expression: motor_torque_RPM
    //  Referenced by: '<S1>/MAX Torque1'

    double MAXTorque1_tableData[6];

    // Expression: motor_RPM_torque
    //  Referenced by: '<S1>/MAX Torque1'

    double MAXTorque1_bp01Data[6];

    // Expression: motor_efficiency_efficiency
    //  Referenced by: '<S1>/2-D Lookup Table1'

    double uDLookupTable1_tableData[3111];

    // Expression: motor_efficiency_RPM
    //  Referenced by: '<S1>/2-D Lookup Table1'

    double uDLookupTable1_bp01Data[51];

    // Expression: motor_efficiency_torque
    //  Referenced by: '<S1>/2-D Lookup Table1'

    double uDLookupTable1_bp02Data[61];

    // Expression: battery_SOC_voltage
    //  Referenced by: '<S1>/SOC-Vol'

    double SOCVol_tableData[101];

    // Pooled Parameter (Expression: battery_SOC_SOC)
    //  Referenced by:
    //    '<S1>/SOC-R0'
    //    '<S1>/SOC-Vol'

    double pooled6[101];

    // Expression: battery_SOC_resistance
    //  Referenced by: '<S1>/SOC-R0'

    double SOCR0_tableData[101];

    // Computed Parameter: uDLookupTable1_maxIndex
    //  Referenced by: '<S1>/2-D Lookup Table1'

    uint32_t uDLookupTable1_maxIndex[2];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    double target_velocity;            // '<Root>/target_velocity'
    double ego_velocity;               // '<Root>/ego_velocity'
    double IVD;                        // '<Root>/IVD'
    double Mode;                       // '<Root>/Mode'
    double Mass_kg;                    // '<Root>/Mass_kg'
    double soc_ego;                    // '<Root>/soc_ego'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    double Accelms2;                   // '<Root>/Accel[ms2]'
    double Speedms;                    // '<Root>/Speed[ms]'
    double Positionm;                  // '<Root>/Position[m]'
    double Aero;                       // '<Root>/Aero'
    double Battery_SOC;                // '<Root>/Battery_SOC[%]'
  };

  // Real-time Model Data Structure
  using odeFSubArray = double[2];
  struct RT_MODEL {
    const char *errorStatus;
    RTWSolverInfo solverInfo;
    X *contStates;
    int *periodicContStateIndices;
    double *periodicContStateRanges;
    double *derivs;
    XDis *contStateDisabled;
    bool zCCacheNeedsReset;
    bool derivCacheNeedsReset;
    bool CTOutputIncnstWithState;
    double odeY[2];
    double odeF[3][2];
    ODE3_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int numContStates;
      int numPeriodicContStates;
      int numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_t clockTick0;
      double stepSize0;
      uint32_t clockTick1;
      double tStart;
      SimTimeStep simTimeStep;
      bool stopRequestedFlag;
      double *t;
      double tArray[2];
    } Timing;

    XDis* getContStateDisabled() const;
    void setContStateDisabled(XDis* aContStateDisabled);
    const char** getErrorStatusPtr();
    X* getContStates() const;
    void setContStates(X* aContStates);
    bool getStopRequested() const;
    void setStopRequested(bool aStopRequested);
    ODE3_IntgData getIntgData() const;
    void setIntgData(ODE3_IntgData aIntgData);
    bool getDerivCacheNeedsReset() const;
    void setDerivCacheNeedsReset(bool aDerivCacheNeedsReset);
    const char* getErrorStatus() const;
    void setErrorStatus(const char* const aErrorStatus);
    bool getContTimeOutputInconsistentWithStateAtMajorStepFlag() const;
    void setContTimeOutputInconsistentWithStateAtMajorStepFlag(bool
      aContTimeOutputInconsistentWithStateAtMajorStepFlag);
    bool isMajorTimeStep() const;
    const odeFSubArray* getOdeF() const;
    bool isMinorTimeStep() const;
    const double* getOdeY() const;
    int* getPeriodicContStateIndices() const;
    void setPeriodicContStateIndices(int* aPeriodicContStateIndices);
    double* getTPtr() const;
    void setTPtr(double* aTPtr);
    double* getPeriodicContStateRanges() const;
    void setPeriodicContStateRanges(double* aPeriodicContStateRanges);
    bool* getStopRequestedPtr();
    double** getTPtrPtr();
    double getTStart() const;
    bool getZCCacheNeedsReset() const;
    void setZCCacheNeedsReset(bool aZCCacheNeedsReset);
    double* getdX() const;
    void setdX(double* adX);
  };

  // Copy Constructor
  BMS(BMS const&) = delete;

  // Assignment Operator
  BMS& operator= (BMS const&) & = delete;

  // Move Constructor
  BMS(BMS &&) = delete;

  // Move Assignment Operator
  BMS& operator= (BMS &&) = delete;

  // Real-Time Model get method
  BMS::RT_MODEL * getRTM();

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  BMS();

  // Destructor
  ~BMS();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Block continuous states
  X rtX;

  // Block Continuous state disabled vector
  XDis rtXDis;

  // Global mass matrix

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void BMS_derivatives();

  // Real-Time Model
  RT_MODEL rtM;
};

// Constant parameters (default storage)
extern const BMS::ConstP rtConstP;
extern "C"
{
  extern double rtInf;
  extern double rtMinusInf;
  extern double rtNaN;
  extern float rtInfF;
  extern float rtMinusInfF;
  extern float rtNaNF;
  static bool rtIsInf(double value);
  static bool rtIsInfF(float value);
  static bool rtIsNaN(double value);
  static bool rtIsNaNF(float value);
}                                      // extern "C"

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('EV_Model/BMS with C.C')    - opens subsystem EV_Model/BMS with C.C
//  hilite_system('EV_Model/BMS with C.C/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'EV_Model'
//  '<S1>'   : 'EV_Model/BMS with C.C'
//  '<S2>'   : 'EV_Model/BMS with C.C/Compare To Constant'
//  '<S3>'   : 'EV_Model/BMS with C.C/Compare To Constant3'
//  '<S4>'   : 'EV_Model/BMS with C.C/Drag coefficient for FV'
//  '<S5>'   : 'EV_Model/BMS with C.C/Drag coefficient for LV'
//  '<S6>'   : 'EV_Model/BMS with C.C/PID Controller'
//  '<S7>'   : 'EV_Model/BMS with C.C/SOC Estimator (Coulomb Counting, Variable Capacity)'
//  '<S8>'   : 'EV_Model/BMS with C.C/PID Controller/Anti-windup'
//  '<S9>'   : 'EV_Model/BMS with C.C/PID Controller/D Gain'
//  '<S10>'  : 'EV_Model/BMS with C.C/PID Controller/External Derivative'
//  '<S11>'  : 'EV_Model/BMS with C.C/PID Controller/Filter'
//  '<S12>'  : 'EV_Model/BMS with C.C/PID Controller/Filter ICs'
//  '<S13>'  : 'EV_Model/BMS with C.C/PID Controller/I Gain'
//  '<S14>'  : 'EV_Model/BMS with C.C/PID Controller/Ideal P Gain'
//  '<S15>'  : 'EV_Model/BMS with C.C/PID Controller/Ideal P Gain Fdbk'
//  '<S16>'  : 'EV_Model/BMS with C.C/PID Controller/Integrator'
//  '<S17>'  : 'EV_Model/BMS with C.C/PID Controller/Integrator ICs'
//  '<S18>'  : 'EV_Model/BMS with C.C/PID Controller/N Copy'
//  '<S19>'  : 'EV_Model/BMS with C.C/PID Controller/N Gain'
//  '<S20>'  : 'EV_Model/BMS with C.C/PID Controller/P Copy'
//  '<S21>'  : 'EV_Model/BMS with C.C/PID Controller/Parallel P Gain'
//  '<S22>'  : 'EV_Model/BMS with C.C/PID Controller/Reset Signal'
//  '<S23>'  : 'EV_Model/BMS with C.C/PID Controller/Saturation'
//  '<S24>'  : 'EV_Model/BMS with C.C/PID Controller/Saturation Fdbk'
//  '<S25>'  : 'EV_Model/BMS with C.C/PID Controller/Sum'
//  '<S26>'  : 'EV_Model/BMS with C.C/PID Controller/Sum Fdbk'
//  '<S27>'  : 'EV_Model/BMS with C.C/PID Controller/Tracking Mode'
//  '<S28>'  : 'EV_Model/BMS with C.C/PID Controller/Tracking Mode Sum'
//  '<S29>'  : 'EV_Model/BMS with C.C/PID Controller/Tsamp - Integral'
//  '<S30>'  : 'EV_Model/BMS with C.C/PID Controller/Tsamp - Ngain'
//  '<S31>'  : 'EV_Model/BMS with C.C/PID Controller/postSat Signal'
//  '<S32>'  : 'EV_Model/BMS with C.C/PID Controller/preInt Signal'
//  '<S33>'  : 'EV_Model/BMS with C.C/PID Controller/preSat Signal'
//  '<S34>'  : 'EV_Model/BMS with C.C/PID Controller/Anti-windup/Passthrough'
//  '<S35>'  : 'EV_Model/BMS with C.C/PID Controller/D Gain/Internal Parameters'
//  '<S36>'  : 'EV_Model/BMS with C.C/PID Controller/External Derivative/Error'
//  '<S37>'  : 'EV_Model/BMS with C.C/PID Controller/Filter/Cont. Filter'
//  '<S38>'  : 'EV_Model/BMS with C.C/PID Controller/Filter ICs/Internal IC - Filter'
//  '<S39>'  : 'EV_Model/BMS with C.C/PID Controller/I Gain/Internal Parameters'
//  '<S40>'  : 'EV_Model/BMS with C.C/PID Controller/Ideal P Gain/Passthrough'
//  '<S41>'  : 'EV_Model/BMS with C.C/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S42>'  : 'EV_Model/BMS with C.C/PID Controller/Integrator/Continuous'
//  '<S43>'  : 'EV_Model/BMS with C.C/PID Controller/Integrator ICs/Internal IC'
//  '<S44>'  : 'EV_Model/BMS with C.C/PID Controller/N Copy/Disabled'
//  '<S45>'  : 'EV_Model/BMS with C.C/PID Controller/N Gain/Internal Parameters'
//  '<S46>'  : 'EV_Model/BMS with C.C/PID Controller/P Copy/Disabled'
//  '<S47>'  : 'EV_Model/BMS with C.C/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S48>'  : 'EV_Model/BMS with C.C/PID Controller/Reset Signal/Disabled'
//  '<S49>'  : 'EV_Model/BMS with C.C/PID Controller/Saturation/Passthrough'
//  '<S50>'  : 'EV_Model/BMS with C.C/PID Controller/Saturation Fdbk/Disabled'
//  '<S51>'  : 'EV_Model/BMS with C.C/PID Controller/Sum/Sum_PID'
//  '<S52>'  : 'EV_Model/BMS with C.C/PID Controller/Sum Fdbk/Disabled'
//  '<S53>'  : 'EV_Model/BMS with C.C/PID Controller/Tracking Mode/Disabled'
//  '<S54>'  : 'EV_Model/BMS with C.C/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S55>'  : 'EV_Model/BMS with C.C/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S56>'  : 'EV_Model/BMS with C.C/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S57>'  : 'EV_Model/BMS with C.C/PID Controller/postSat Signal/Forward_Path'
//  '<S58>'  : 'EV_Model/BMS with C.C/PID Controller/preInt Signal/Internal PreInt'
//  '<S59>'  : 'EV_Model/BMS with C.C/PID Controller/preSat Signal/Forward_Path'
//  '<S60>'  : 'EV_Model/BMS with C.C/SOC Estimator (Coulomb Counting, Variable Capacity)/Integrator'
//  '<S61>'  : 'EV_Model/BMS with C.C/SOC Estimator (Coulomb Counting, Variable Capacity)/Integrator/Discrete'

#endif                                 // BMS_h_

//
// File trailer for generated code.
//
// [EOF]
//
