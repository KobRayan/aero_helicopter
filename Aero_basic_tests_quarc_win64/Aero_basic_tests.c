/*
 * Aero_basic_tests.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Aero_basic_tests".
 *
 * Model version              : 1.70
 * Simulink Coder version : 25.1 (R2025a) 21-Nov-2024
 * C source code generated on : Mon Sep 28 11:22:48 2026
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Aero_basic_tests.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include <math.h>
#include "rt_nonfinite.h"
#include "Aero_basic_tests_private.h"
#include <string.h>
#include "Aero_basic_tests_dt.h"

/* Block signals (default storage) */
B_Aero_basic_tests_T Aero_basic_tests_B;

/* Continuous states */
X_Aero_basic_tests_T Aero_basic_tests_X;

/* Disabled State Vector */
XDis_Aero_basic_tests_T Aero_basic_tests_XDis;

/* Block states (default storage) */
DW_Aero_basic_tests_T Aero_basic_tests_DW;

/* Real-time model */
static RT_MODEL_Aero_basic_tests_T Aero_basic_tests_M_;
RT_MODEL_Aero_basic_tests_T *const Aero_basic_tests_M = &Aero_basic_tests_M_;

/*
 * Writes out MAT-file header.  Returns success or failure.
 * Returns:
 * 0 - success
 * 1 - failure
 */
int_T rt_WriteMat4FileHeader(FILE *fp, int32_T m, int32_T n, const char *name)
{
  typedef enum { ELITTLE_ENDIAN, EBIG_ENDIAN } ByteOrder;

  int16_T one = 1;
  ByteOrder byteOrder = (*((int8_T *)&one)==1) ? ELITTLE_ENDIAN : EBIG_ENDIAN;
  int32_T type = (byteOrder == ELITTLE_ENDIAN) ? 0: 1000;
  int32_T imagf = 0;
  int32_T name_len = (int32_T)strlen(name) + 1;
  if ((fwrite(&type, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&m, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&n, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&imagf, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&name_len, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(name, sizeof(char), name_len, fp) == 0)) {
    return(1);
  } else {
    return(0);
  }
}                                      /* end rt_WriteMat4FileHeader */

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  Aero_basic_tests_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void Aero_basic_tests_output(void)
{
  /* local block i/o variables */
  real_T rtb_HILRead_o12;
  real_T rtb_HILRead_o13;
  real_T rtb_HILRead_o14;
  __m128d tmp_2;
  real_T rtb_TmpSignalConversionAtSaveof[9];
  real_T tmp_1[2];
  real_T rtb_HILRead_o5;
  real_T rtb_HILRead_o6;
  real_T rtb_HILRead_o7;
  real_T rtb_HILRead_o8;
  real_T rtb_HILRead_o9;
  real_T rtb_Stepend_time;
  real_T rtb_Stepstart_time;
  int32_T tmp_0;
  int8_T rtAction;
  int8_T rtPrevAction;
  boolean_T tmp;
  boolean_T tmp_3;
  if (rtmIsMajorTimeStep(Aero_basic_tests_M)) {
    /* set solver stop time */
    if (!(Aero_basic_tests_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Aero_basic_tests_M->solverInfo,
                            ((Aero_basic_tests_M->Timing.clockTickH0 + 1) *
        Aero_basic_tests_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Aero_basic_tests_M->solverInfo,
                            ((Aero_basic_tests_M->Timing.clockTick0 + 1) *
        Aero_basic_tests_M->Timing.stepSize0 +
        Aero_basic_tests_M->Timing.clockTickH0 *
        Aero_basic_tests_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Aero_basic_tests_M)) {
    Aero_basic_tests_M->Timing.t[0] = rtsiGetT(&Aero_basic_tests_M->solverInfo);
  }

  /* Reset subsysRan breadcrumbs */
  srClearBC(Aero_basic_tests_DW.EnabledMovingAverage_SubsysRanB);

  /* Reset subsysRan breadcrumbs */
  srClearBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem_Subsy);

  /* Reset subsysRan breadcrumbs */
  srClearBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem1_Subs);

  /* Reset subsysRan breadcrumbs */
  srClearBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem2_Subs);
  tmp = rtmIsMajorTimeStep(Aero_basic_tests_M);
  if (tmp) {
    /* Constant: '<S3>/x0' */
    Aero_basic_tests_B.x0 = Aero_basic_tests_P.x0_Value;
  }

  /* Integrator: '<S3>/Integrator1' */
  if (Aero_basic_tests_DW.Integrator1_DWORK1) {
    Aero_basic_tests_X.Integrator1_CSTATE[0] = Aero_basic_tests_B.x0;
    Aero_basic_tests_X.Integrator1_CSTATE[1] = Aero_basic_tests_B.x0;
    Aero_basic_tests_X.Integrator1_CSTATE[2] = Aero_basic_tests_B.x0;
  }

  /* Integrator: '<S3>/Integrator1' */
  Aero_basic_tests_B.Integrator1[0] = Aero_basic_tests_X.Integrator1_CSTATE[0];
  Aero_basic_tests_B.Integrator1[1] = Aero_basic_tests_X.Integrator1_CSTATE[1];
  Aero_basic_tests_B.Integrator1[2] = Aero_basic_tests_X.Integrator1_CSTATE[2];
  if (tmp) {
    /* S-Function (hil_read_block): '<Root>/HIL Read' */

    /* S-Function Block: Aero_basic_tests/HIL Read (hil_read_block) */
    {
      t_error result = hil_read(Aero_basic_tests_DW.HILInitialize_Card,
        Aero_basic_tests_P.HILRead_analog_channels, 2U,
        Aero_basic_tests_P.HILRead_encoder_channels, 4U,
        NULL, 0U,
        Aero_basic_tests_P.HILRead_other_channels, 8U,
        &Aero_basic_tests_DW.HILRead_AnalogBuffer[0],
        &Aero_basic_tests_DW.HILRead_EncoderBuffer[0],
        NULL,
        &Aero_basic_tests_DW.HILRead_OtherBuffer[0]
        );
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
      } else {
        Aero_basic_tests_B.HILRead_o1 =
          Aero_basic_tests_DW.HILRead_AnalogBuffer[0];
        Aero_basic_tests_B.HILRead_o2 =
          Aero_basic_tests_DW.HILRead_AnalogBuffer[1];
        Aero_basic_tests_B.HILRead_o3 =
          Aero_basic_tests_DW.HILRead_EncoderBuffer[0];
        Aero_basic_tests_B.HILRead_o4 =
          Aero_basic_tests_DW.HILRead_EncoderBuffer[1];
        rtb_HILRead_o5 = Aero_basic_tests_DW.HILRead_EncoderBuffer[2];
        rtb_HILRead_o6 = Aero_basic_tests_DW.HILRead_EncoderBuffer[3];
        rtb_HILRead_o7 = Aero_basic_tests_DW.HILRead_OtherBuffer[0];
        rtb_HILRead_o8 = Aero_basic_tests_DW.HILRead_OtherBuffer[1];
        rtb_HILRead_o9 = Aero_basic_tests_DW.HILRead_OtherBuffer[2];
        rtb_Stepend_time = Aero_basic_tests_DW.HILRead_OtherBuffer[3];
        rtb_Stepstart_time = Aero_basic_tests_DW.HILRead_OtherBuffer[4];
        rtb_HILRead_o12 = Aero_basic_tests_DW.HILRead_OtherBuffer[5];
        rtb_HILRead_o13 = Aero_basic_tests_DW.HILRead_OtherBuffer[6];
        rtb_HILRead_o14 = Aero_basic_tests_DW.HILRead_OtherBuffer[7];
      }
    }

    /* Gain: '<Root>/Pitch: counts to deg' */
    Aero_basic_tests_B.Pitchcountstodeg =
      Aero_basic_tests_P.Pitchcountstodeg_Gain * rtb_HILRead_o5;

    /* Gain: '<Root>/Yaw: counts to deg' */
    Aero_basic_tests_B.Yawcountstodeg = Aero_basic_tests_P.Yawcountstodeg_Gain *
      rtb_HILRead_o6;

    /* Gain: '<Root>/Gyro Cal' */
    _mm_storeu_pd(&Aero_basic_tests_B.GyroCal[0], _mm_mul_pd(_mm_loadu_pd
      (&Aero_basic_tests_P.GyroCal_Gain[0]), _mm_set_pd(rtb_Stepend_time,
      rtb_HILRead_o9)));
    Aero_basic_tests_B.GyroCal[2] = Aero_basic_tests_P.GyroCal_Gain[2] *
      rtb_Stepstart_time;

    /* Gain: '<Root>/counts//s to rad//s' */
    Aero_basic_tests_B.countsstorads = Aero_basic_tests_P.countsstorads_Gain *
      rtb_HILRead_o7;

    /* Gain: '<Root>/counts//s to rad//s1' */
    Aero_basic_tests_B.countsstorads1 = Aero_basic_tests_P.countsstorads1_Gain *
      rtb_HILRead_o8;

    /* SignalConversion generated from: '<Root>/Save of the data in  the data_step_Qube.mat file' */
    rtb_TmpSignalConversionAtSaveof[0] = Aero_basic_tests_B.HILRead_o1;
    rtb_TmpSignalConversionAtSaveof[1] = Aero_basic_tests_B.HILRead_o2;
    rtb_TmpSignalConversionAtSaveof[2] = Aero_basic_tests_B.Pitchcountstodeg;
    rtb_TmpSignalConversionAtSaveof[3] = Aero_basic_tests_B.Yawcountstodeg;
    rtb_TmpSignalConversionAtSaveof[4] = Aero_basic_tests_B.countsstorads;
    rtb_TmpSignalConversionAtSaveof[5] = Aero_basic_tests_B.countsstorads1;
    rtb_TmpSignalConversionAtSaveof[6] = Aero_basic_tests_B.GyroCal[0];
    rtb_TmpSignalConversionAtSaveof[7] = Aero_basic_tests_B.GyroCal[1];
    rtb_TmpSignalConversionAtSaveof[8] = Aero_basic_tests_B.GyroCal[2];

    /* ToFile: '<Root>/Save of the data in  the data_step_Qube.mat file' */
    {
      if (!(++Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Decimation % 1)
          && (Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Count * (9 + 1))
          + 1 < 100000000 ) {
        FILE *fp = (FILE *)
          Aero_basic_tests_DW.Saveofthedatainthedata_step_Qub.FilePtr;
        if (fp != (NULL)) {
          real_T u[9 + 1];
          Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Decimation = 0;
          u[0] = Aero_basic_tests_M->Timing.t[1];
          u[1] = rtb_TmpSignalConversionAtSaveof[0];
          u[2] = rtb_TmpSignalConversionAtSaveof[1];
          u[3] = rtb_TmpSignalConversionAtSaveof[2];
          u[4] = rtb_TmpSignalConversionAtSaveof[3];
          u[5] = rtb_TmpSignalConversionAtSaveof[4];
          u[6] = rtb_TmpSignalConversionAtSaveof[5];
          u[7] = rtb_TmpSignalConversionAtSaveof[6];
          u[8] = rtb_TmpSignalConversionAtSaveof[7];
          u[9] = rtb_TmpSignalConversionAtSaveof[8];
          if (fwrite(u, sizeof(real_T), 9 + 1, fp) != 9 + 1) {
            rtmSetErrorStatus(Aero_basic_tests_M,
                              "Error writing to MAT-file data_step_AERO.mat");
            return;
          }

          if (((++Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Count) *
               (9 + 1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file data_step_AERO.mat.\n");
          }
        }
      }
    }

    /* Outputs for Atomic SubSystem: '<Root>/Bias Removal' */
    /* Step: '<S2>/Step: start_time' incorporates:
     *  Step: '<S2>/Step: end_time'
     */
    rtb_HILRead_o5 = Aero_basic_tests_M->Timing.t[1];

    /* Outputs for Atomic SubSystem: '<Root>/Bias Removal' */
    if (rtb_HILRead_o5 < Aero_basic_tests_P.BiasRemoval_start_time) {
      rtb_Stepstart_time = Aero_basic_tests_P.Stepstart_time_Y0;
    } else {
      rtb_Stepstart_time = Aero_basic_tests_P.Stepstart_time_YFinal;
    }

    /* End of Step: '<S2>/Step: start_time' */

    /* Step: '<S2>/Step: end_time' */
    if (rtb_HILRead_o5 < Aero_basic_tests_P.BiasRemoval_end_time) {
      rtb_Stepend_time = Aero_basic_tests_P.Stepend_time_Y0;
    } else {
      rtb_Stepend_time = Aero_basic_tests_P.Stepend_time_YFinal;
    }

    /* End of Outputs for SubSystem: '<Root>/Bias Removal' */

    /* Logic: '<S2>/Logical Operator1' incorporates:
     *  Logic: '<S2>/Logical Operator2'
     */
    tmp_3 = !(rtb_Stepend_time != 0.0);

    /* Outputs for Enabled SubSystem: '<S2>/Enabled Moving Average' incorporates:
     *  EnablePort: '<S4>/Enable'
     */
    /* Logic: '<S2>/Logical Operator' incorporates:
     *  Logic: '<S2>/Logical Operator1'
     *  Product: '<S4>/div'
     *  Sum: '<S4>/Sum'
     *  UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )'
     */
    if ((rtb_Stepstart_time != 0.0) && tmp_3) {
      if (!Aero_basic_tests_DW.EnabledMovingAverage_MODE) {
        /* InitializeConditions for UnitDelay: '<S8>/Unit Delay' */
        Aero_basic_tests_DW.UnitDelay_DSTATE =
          Aero_basic_tests_P.UnitDelay_InitialCondition;

        /* InitializeConditions for UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )' */
        Aero_basic_tests_DW.Sumk1n1xk_DSTATE[0] =
          Aero_basic_tests_P.Sumk1n1xk_InitialCondition;
        Aero_basic_tests_DW.Sumk1n1xk_DSTATE[1] =
          Aero_basic_tests_P.Sumk1n1xk_InitialCondition;
        Aero_basic_tests_DW.Sumk1n1xk_DSTATE[2] =
          Aero_basic_tests_P.Sumk1n1xk_InitialCondition;
        Aero_basic_tests_DW.EnabledMovingAverage_MODE = true;
      }

      /* Sum: '<S8>/Count' incorporates:
       *  Constant: '<S8>/unity'
       *  UnitDelay: '<S8>/Unit Delay'
       */
      Aero_basic_tests_B.Count = Aero_basic_tests_P.unity_Value +
        Aero_basic_tests_DW.UnitDelay_DSTATE;
      tmp_2 = _mm_add_pd(_mm_set_pd(rtb_HILRead_o13, rtb_HILRead_o12),
                         _mm_loadu_pd(&Aero_basic_tests_DW.Sumk1n1xk_DSTATE[0]));
      _mm_storeu_pd(&Aero_basic_tests_B.Sum[0], tmp_2);

      /* Sum: '<S4>/Sum' incorporates:
       *  UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )'
       */
      Aero_basic_tests_B.Sum[2] = rtb_HILRead_o14 +
        Aero_basic_tests_DW.Sumk1n1xk_DSTATE[2];
      tmp_2 = _mm_div_pd(_mm_loadu_pd(&Aero_basic_tests_B.Sum[0]), _mm_set1_pd
                         (Aero_basic_tests_B.Count));
      _mm_storeu_pd(&Aero_basic_tests_B.div[0], tmp_2);

      /* Product: '<S4>/div' */
      Aero_basic_tests_B.div[2] = Aero_basic_tests_B.Sum[2] /
        Aero_basic_tests_B.Count;
      srUpdateBC(Aero_basic_tests_DW.EnabledMovingAverage_SubsysRanB);
    } else {
      Aero_basic_tests_DW.EnabledMovingAverage_MODE = false;
    }

    /* End of Logic: '<S2>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S2>/Enabled Moving Average' */

    /* Sum: '<S2>/Sum' */
    tmp_2 = _mm_sub_pd(_mm_set_pd(rtb_HILRead_o13, rtb_HILRead_o12),
                       _mm_loadu_pd(&Aero_basic_tests_B.div[0]));

    /* End of Outputs for SubSystem: '<Root>/Bias Removal' */
    _mm_storeu_pd(&tmp_1[0], tmp_2);

    /* Outputs for Atomic SubSystem: '<Root>/Bias Removal' */
    /* Sum: '<S2>/Sum' */
    rtb_HILRead_o5 = rtb_HILRead_o14 - Aero_basic_tests_B.div[2];

    /* SwitchCase: '<S2>/Switch Case' incorporates:
     *  Constant: '<S5>/Constant'
     */
    rtPrevAction = Aero_basic_tests_DW.SwitchCase_ActiveSubsystem;
    rtAction = -1;

    /* Outputs for Atomic SubSystem: '<Root>/Bias Removal' */
    if (Aero_basic_tests_P.BiasRemoval_switch_id < 0.0) {
      rtb_HILRead_o6 = ceil(Aero_basic_tests_P.BiasRemoval_switch_id);
    } else {
      rtb_HILRead_o6 = floor(Aero_basic_tests_P.BiasRemoval_switch_id);
    }

    /* End of Outputs for SubSystem: '<Root>/Bias Removal' */
    if (rtIsNaN(rtb_HILRead_o6) || rtIsInf(rtb_HILRead_o6)) {
      rtb_HILRead_o6 = 0.0;
    } else {
      rtb_HILRead_o6 = fmod(rtb_HILRead_o6, 4.294967296E+9);
    }

    if (rtb_HILRead_o6 < 0.0) {
      tmp_0 = -(int32_T)(uint32_T)-rtb_HILRead_o6;
    } else {
      tmp_0 = (int32_T)(uint32_T)rtb_HILRead_o6;
    }

    switch (tmp_0) {
     case 1:
      rtAction = 0;
      break;

     case 2:
      rtAction = 1;
      break;

     case 3:
      rtAction = 2;
      break;
    }

    Aero_basic_tests_DW.SwitchCase_ActiveSubsystem = rtAction;
    if (rtPrevAction != rtAction) {
      switch (rtPrevAction) {
       case 0:
        /* Disable for Outport: '<S5>/zero' incorporates:
         *  Constant: '<S5>/Constant'
         * */
        Aero_basic_tests_B.Constant = Aero_basic_tests_P.zero_Y0;
        break;

       case 1:
        /* Disable for SignalConversion generated from: '<S6>/Vin' incorporates:
         *  Outport: '<S6>/Vbiased'
         * */
        Aero_basic_tests_B.Vin[0] = Aero_basic_tests_P.Vbiased_Y0;
        Aero_basic_tests_B.Vin[1] = Aero_basic_tests_P.Vbiased_Y0;
        Aero_basic_tests_B.Vin[2] = Aero_basic_tests_P.Vbiased_Y0;
        break;

       case 2:
        /* Disable for SignalConversion generated from: '<S7>/V-Vavg' incorporates:
         *  Outport: '<S7>/Vunbiased'
         * */
        Aero_basic_tests_B.VVavg[0] = Aero_basic_tests_P.Vunbiased_Y0;
        Aero_basic_tests_B.VVavg[1] = Aero_basic_tests_P.Vunbiased_Y0;
        Aero_basic_tests_B.VVavg[2] = Aero_basic_tests_P.Vunbiased_Y0;
        break;
      }
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S2>/Switch Case Action Subsystem' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      Aero_basic_tests_B.Constant = Aero_basic_tests_P.Constant_Value;
      srUpdateBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem_Subsy);

      /* End of Outputs for SubSystem: '<S2>/Switch Case Action Subsystem' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S2>/Switch Case Action Subsystem1' incorporates:
       *  ActionPort: '<S6>/Action Port'
       */
      /* SignalConversion generated from: '<S6>/Vin' */
      Aero_basic_tests_B.Vin[0] = rtb_HILRead_o12;
      Aero_basic_tests_B.Vin[1] = rtb_HILRead_o13;
      Aero_basic_tests_B.Vin[2] = rtb_HILRead_o14;
      srUpdateBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem1_Subs);

      /* End of Outputs for SubSystem: '<S2>/Switch Case Action Subsystem1' */
      break;

     case 2:
      /* Outputs for IfAction SubSystem: '<S2>/Switch Case Action Subsystem2' incorporates:
       *  ActionPort: '<S7>/Action Port'
       */
      /* SignalConversion generated from: '<S7>/V-Vavg' */
      Aero_basic_tests_B.VVavg[0] = tmp_1[0];
      Aero_basic_tests_B.VVavg[1] = tmp_1[1];
      Aero_basic_tests_B.VVavg[2] = rtb_HILRead_o5;
      srUpdateBC(Aero_basic_tests_DW.SwitchCaseActionSubsystem2_Subs);

      /* End of Outputs for SubSystem: '<S2>/Switch Case Action Subsystem2' */
      break;
    }

    /* End of SwitchCase: '<S2>/Switch Case' */

    /* Switch: '<S2>/Switch' incorporates:
     *  Sum: '<S2>/Sum1'
     */
    if (tmp_3) {
      tmp_2 = _mm_add_pd(_mm_add_pd(_mm_set1_pd(Aero_basic_tests_B.Constant),
        _mm_loadu_pd(&Aero_basic_tests_B.Vin[0])), _mm_loadu_pd
                         (&Aero_basic_tests_B.VVavg[0]));
      _mm_storeu_pd(&Aero_basic_tests_B.Switch[0], tmp_2);

      /* Switch: '<S2>/Switch' incorporates:
       *  Sum: '<S2>/Sum1'
       */
      Aero_basic_tests_B.Switch[2] = (Aero_basic_tests_B.Constant +
        Aero_basic_tests_B.Vin[2]) + Aero_basic_tests_B.VVavg[2];
    } else {
      /* Switch: '<S2>/Switch' */
      Aero_basic_tests_B.Switch[0] = tmp_1[0];
      Aero_basic_tests_B.Switch[1] = tmp_1[1];
      Aero_basic_tests_B.Switch[2] = rtb_HILRead_o5;
    }

    /* End of Switch: '<S2>/Switch' */
    /* End of Outputs for SubSystem: '<Root>/Bias Removal' */
  }

  /* Product: '<S3>/Product' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/wn'
   *  Constant: '<S3>/zt'
   *  Integrator: '<S3>/Integrator2'
   *  Product: '<S3>/Product2'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   */
  Aero_basic_tests_B.Product[0] = ((Aero_basic_tests_B.Switch[0] -
    Aero_basic_tests_B.Integrator1[0]) - Aero_basic_tests_X.Integrator2_CSTATE[0]
    * Aero_basic_tests_P.Constant_Value_m *
    Aero_basic_tests_P.SecondOrderLowPassFilter_inpu_m) *
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_;

  /* Product: '<S3>/Product1' incorporates:
   *  Constant: '<S3>/wn'
   *  Integrator: '<S3>/Integrator2'
   */
  Aero_basic_tests_B.Product1[0] =
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_ *
    Aero_basic_tests_X.Integrator2_CSTATE[0];

  /* Product: '<S3>/Product' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/wn'
   *  Constant: '<S3>/zt'
   *  Integrator: '<S3>/Integrator2'
   *  Product: '<S3>/Product2'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   */
  Aero_basic_tests_B.Product[1] = ((Aero_basic_tests_B.Switch[1] -
    Aero_basic_tests_B.Integrator1[1]) - Aero_basic_tests_X.Integrator2_CSTATE[1]
    * Aero_basic_tests_P.Constant_Value_m *
    Aero_basic_tests_P.SecondOrderLowPassFilter_inpu_m) *
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_;

  /* Product: '<S3>/Product1' incorporates:
   *  Constant: '<S3>/wn'
   *  Integrator: '<S3>/Integrator2'
   */
  Aero_basic_tests_B.Product1[1] =
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_ *
    Aero_basic_tests_X.Integrator2_CSTATE[1];

  /* Product: '<S3>/Product' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/wn'
   *  Constant: '<S3>/zt'
   *  Integrator: '<S3>/Integrator2'
   *  Product: '<S3>/Product2'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   */
  Aero_basic_tests_B.Product[2] = ((Aero_basic_tests_B.Switch[2] -
    Aero_basic_tests_B.Integrator1[2]) - Aero_basic_tests_X.Integrator2_CSTATE[2]
    * Aero_basic_tests_P.Constant_Value_m *
    Aero_basic_tests_P.SecondOrderLowPassFilter_inpu_m) *
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_;

  /* Product: '<S3>/Product1' incorporates:
   *  Constant: '<S3>/wn'
   *  Integrator: '<S3>/Integrator2'
   */
  Aero_basic_tests_B.Product1[2] =
    Aero_basic_tests_P.SecondOrderLowPassFilter_input_ *
    Aero_basic_tests_X.Integrator2_CSTATE[2];
  if (tmp) {
    /* Saturate: '<Root>/P: +//- 24V' incorporates:
     *  Constant: '<Root>/Pitch Cmd (V)'
     */
    if (Aero_basic_tests_P.PitchCmdV_Value > Aero_basic_tests_P.P24V_UpperSat) {
      /* Saturate: '<Root>/P: +//- 24V' */
      Aero_basic_tests_B.P24V = Aero_basic_tests_P.P24V_UpperSat;
    } else if (Aero_basic_tests_P.PitchCmdV_Value <
               Aero_basic_tests_P.P24V_LowerSat) {
      /* Saturate: '<Root>/P: +//- 24V' */
      Aero_basic_tests_B.P24V = Aero_basic_tests_P.P24V_LowerSat;
    } else {
      /* Saturate: '<Root>/P: +//- 24V' */
      Aero_basic_tests_B.P24V = Aero_basic_tests_P.PitchCmdV_Value;
    }

    /* End of Saturate: '<Root>/P: +//- 24V' */

    /* Saturate: '<Root>/Y: +//- 24V' incorporates:
     *  Constant: '<Root>/Yaw Cmd (V)'
     */
    if (Aero_basic_tests_P.YawCmdV_Value > Aero_basic_tests_P.Y24V_UpperSat) {
      /* Saturate: '<Root>/Y: +//- 24V' */
      Aero_basic_tests_B.Y24V = Aero_basic_tests_P.Y24V_UpperSat;
    } else if (Aero_basic_tests_P.YawCmdV_Value <
               Aero_basic_tests_P.Y24V_LowerSat) {
      /* Saturate: '<Root>/Y: +//- 24V' */
      Aero_basic_tests_B.Y24V = Aero_basic_tests_P.Y24V_LowerSat;
    } else {
      /* Saturate: '<Root>/Y: +//- 24V' */
      Aero_basic_tests_B.Y24V = Aero_basic_tests_P.YawCmdV_Value;
    }

    /* End of Saturate: '<Root>/Y: +//- 24V' */

    /* S-Function (hil_write_block): '<Root>/HIL Write' incorporates:
     *  Constant: '<Root>/Pitch Motor Enable'
     *  Constant: '<Root>/Yaw Motor Enable'
     *  Constant: '<S1>/Constant'
     */

    /* S-Function Block: Aero_basic_tests/HIL Write (hil_write_block) */
    {
      t_error result;
      Aero_basic_tests_DW.HILWrite_AnalogBuffer[0] = Aero_basic_tests_B.P24V;
      Aero_basic_tests_DW.HILWrite_AnalogBuffer[1] = Aero_basic_tests_B.Y24V;
      Aero_basic_tests_DW.HILWrite_DigitalBuffer[0] =
        (Aero_basic_tests_P.PitchMotorEnable_Value != 0);
      Aero_basic_tests_DW.HILWrite_DigitalBuffer[1] =
        (Aero_basic_tests_P.YawMotorEnable_Value != 0);
      Aero_basic_tests_DW.HILWrite_OtherBuffer[0] =
        Aero_basic_tests_P.BaseLEDColour_color[0];
      Aero_basic_tests_DW.HILWrite_OtherBuffer[1] =
        Aero_basic_tests_P.BaseLEDColour_color[1];
      Aero_basic_tests_DW.HILWrite_OtherBuffer[2] =
        Aero_basic_tests_P.BaseLEDColour_color[2];
      result = hil_write(Aero_basic_tests_DW.HILInitialize_Card,
                         Aero_basic_tests_P.HILWrite_analog_channels, 2U,
                         NULL, 0U,
                         Aero_basic_tests_P.HILWrite_digital_channels, 2U,
                         Aero_basic_tests_P.HILWrite_other_channels, 3U,
                         &Aero_basic_tests_DW.HILWrite_AnalogBuffer[0],
                         NULL,
                         &Aero_basic_tests_DW.HILWrite_DigitalBuffer[0],
                         &Aero_basic_tests_DW.HILWrite_OtherBuffer[0]
                         );
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
      }
    }
  }
}

/* Model update function */
void Aero_basic_tests_update(void)
{
  /* Update for Integrator: '<S3>/Integrator1' */
  Aero_basic_tests_DW.Integrator1_DWORK1 = false;
  if (rtmIsMajorTimeStep(Aero_basic_tests_M)) {
    /* Update for Atomic SubSystem: '<Root>/Bias Removal' */
    /* Update for Enabled SubSystem: '<S2>/Enabled Moving Average' incorporates:
     *  EnablePort: '<S4>/Enable'
     */
    if (Aero_basic_tests_DW.EnabledMovingAverage_MODE) {
      /* Update for UnitDelay: '<S8>/Unit Delay' */
      Aero_basic_tests_DW.UnitDelay_DSTATE = Aero_basic_tests_B.Count;

      /* Update for UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )' */
      Aero_basic_tests_DW.Sumk1n1xk_DSTATE[0] = Aero_basic_tests_B.Sum[0];
      Aero_basic_tests_DW.Sumk1n1xk_DSTATE[1] = Aero_basic_tests_B.Sum[1];
      Aero_basic_tests_DW.Sumk1n1xk_DSTATE[2] = Aero_basic_tests_B.Sum[2];
    }

    /* End of Update for SubSystem: '<S2>/Enabled Moving Average' */
    /* End of Update for SubSystem: '<Root>/Bias Removal' */
  }

  if (rtmIsMajorTimeStep(Aero_basic_tests_M)) {
    rt_ertODEUpdateContinuousStates(&Aero_basic_tests_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Aero_basic_tests_M->Timing.clockTick0)) {
    ++Aero_basic_tests_M->Timing.clockTickH0;
  }

  Aero_basic_tests_M->Timing.t[0] = rtsiGetSolverStopTime
    (&Aero_basic_tests_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Aero_basic_tests_M->Timing.clockTick1)) {
      ++Aero_basic_tests_M->Timing.clockTickH1;
    }

    Aero_basic_tests_M->Timing.t[1] = Aero_basic_tests_M->Timing.clockTick1 *
      Aero_basic_tests_M->Timing.stepSize1 +
      Aero_basic_tests_M->Timing.clockTickH1 *
      Aero_basic_tests_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void Aero_basic_tests_derivatives(void)
{
  XDot_Aero_basic_tests_T *_rtXdot;
  _rtXdot = ((XDot_Aero_basic_tests_T *) Aero_basic_tests_M->derivs);

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[0] = Aero_basic_tests_B.Product1[0];

  /* Derivatives for Integrator: '<S3>/Integrator2' */
  _rtXdot->Integrator2_CSTATE[0] = Aero_basic_tests_B.Product[0];

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[1] = Aero_basic_tests_B.Product1[1];

  /* Derivatives for Integrator: '<S3>/Integrator2' */
  _rtXdot->Integrator2_CSTATE[1] = Aero_basic_tests_B.Product[1];

  /* Derivatives for Integrator: '<S3>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[2] = Aero_basic_tests_B.Product1[2];

  /* Derivatives for Integrator: '<S3>/Integrator2' */
  _rtXdot->Integrator2_CSTATE[2] = Aero_basic_tests_B.Product[2];
}

/* Model initialize function */
void Aero_basic_tests_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Aero_basic_tests/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("quanser_aero_usb", "0",
                      &Aero_basic_tests_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_watchdog_clear(Aero_basic_tests_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
      return;
    }

    if ((Aero_basic_tests_P.HILInitialize_AIPStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_AIPEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_AIMinimums[0] =
        (Aero_basic_tests_P.HILInitialize_AILow);
      Aero_basic_tests_DW.HILInitialize_AIMinimums[1] =
        (Aero_basic_tests_P.HILInitialize_AILow);
      Aero_basic_tests_DW.HILInitialize_AIMaximums[0] =
        Aero_basic_tests_P.HILInitialize_AIHigh;
      Aero_basic_tests_DW.HILInitialize_AIMaximums[1] =
        Aero_basic_tests_P.HILInitialize_AIHigh;
      result = hil_set_analog_input_ranges
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_AIChannels, 2U,
         &Aero_basic_tests_DW.HILInitialize_AIMinimums[0],
         &Aero_basic_tests_DW.HILInitialize_AIMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if ((Aero_basic_tests_P.HILInitialize_AOPStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_AOPEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_AOMinimums[0] =
        (Aero_basic_tests_P.HILInitialize_AOLow);
      Aero_basic_tests_DW.HILInitialize_AOMinimums[1] =
        (Aero_basic_tests_P.HILInitialize_AOLow);
      Aero_basic_tests_DW.HILInitialize_AOMaximums[0] =
        Aero_basic_tests_P.HILInitialize_AOHigh;
      Aero_basic_tests_DW.HILInitialize_AOMaximums[1] =
        Aero_basic_tests_P.HILInitialize_AOHigh;
      result = hil_set_analog_output_ranges
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_AOChannels, 2U,
         &Aero_basic_tests_DW.HILInitialize_AOMinimums[0],
         &Aero_basic_tests_DW.HILInitialize_AOMaximums[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if ((Aero_basic_tests_P.HILInitialize_AOStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_AOEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_AOVoltages[0] =
        Aero_basic_tests_P.HILInitialize_AOInitial;
      Aero_basic_tests_DW.HILInitialize_AOVoltages[1] =
        Aero_basic_tests_P.HILInitialize_AOInitial;
      result = hil_write_analog(Aero_basic_tests_DW.HILInitialize_Card,
        Aero_basic_tests_P.HILInitialize_AOChannels, 2U,
        &Aero_basic_tests_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if (Aero_basic_tests_P.HILInitialize_AOReset) {
      Aero_basic_tests_DW.HILInitialize_AOVoltages[0] =
        Aero_basic_tests_P.HILInitialize_AOWatchdog;
      Aero_basic_tests_DW.HILInitialize_AOVoltages[1] =
        Aero_basic_tests_P.HILInitialize_AOWatchdog;
      result = hil_watchdog_set_analog_expiration_state
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_AOChannels, 2U,
         &Aero_basic_tests_DW.HILInitialize_AOVoltages[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions(Aero_basic_tests_DW.HILInitialize_Card,
      NULL, 0U, Aero_basic_tests_P.HILInitialize_DOChannels, 2U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
      return;
    }

    if ((Aero_basic_tests_P.HILInitialize_DOStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_DOEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_DOBits[0] =
        Aero_basic_tests_P.HILInitialize_DOInitial;
      Aero_basic_tests_DW.HILInitialize_DOBits[1] =
        Aero_basic_tests_P.HILInitialize_DOInitial;
      result = hil_write_digital(Aero_basic_tests_DW.HILInitialize_Card,
        Aero_basic_tests_P.HILInitialize_DOChannels, 2U, (t_boolean *)
        &Aero_basic_tests_DW.HILInitialize_DOBits[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if (Aero_basic_tests_P.HILInitialize_DOReset) {
      Aero_basic_tests_DW.HILInitialize_DOStates[0] =
        Aero_basic_tests_P.HILInitialize_DOWatchdog;
      Aero_basic_tests_DW.HILInitialize_DOStates[1] =
        Aero_basic_tests_P.HILInitialize_DOWatchdog;
      result = hil_watchdog_set_digital_expiration_state
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_DOChannels, 2U, (const t_digital_state
          *) &Aero_basic_tests_DW.HILInitialize_DOStates[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if ((Aero_basic_tests_P.HILInitialize_EIPStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_EIPEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_QuadratureModes[0] =
        Aero_basic_tests_P.HILInitialize_EIQuadrature;
      Aero_basic_tests_DW.HILInitialize_QuadratureModes[1] =
        Aero_basic_tests_P.HILInitialize_EIQuadrature;
      Aero_basic_tests_DW.HILInitialize_QuadratureModes[2] =
        Aero_basic_tests_P.HILInitialize_EIQuadrature;
      Aero_basic_tests_DW.HILInitialize_QuadratureModes[3] =
        Aero_basic_tests_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_EIChannels, 4U,
         (t_encoder_quadrature_mode *)
         &Aero_basic_tests_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if ((Aero_basic_tests_P.HILInitialize_EIStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_EIEnter && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_InitialEICounts[0] =
        Aero_basic_tests_P.HILInitialize_EIInitial;
      Aero_basic_tests_DW.HILInitialize_InitialEICounts[1] =
        Aero_basic_tests_P.HILInitialize_EIInitial;
      Aero_basic_tests_DW.HILInitialize_InitialEICounts[2] =
        Aero_basic_tests_P.HILInitialize_EIInitial;
      Aero_basic_tests_DW.HILInitialize_InitialEICounts[3] =
        Aero_basic_tests_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(Aero_basic_tests_DW.HILInitialize_Card,
        Aero_basic_tests_P.HILInitialize_EIChannels, 4U,
        &Aero_basic_tests_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if ((Aero_basic_tests_P.HILInitialize_OOStart && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(Aero_basic_tests_DW.HILInitialize_Card,
        Aero_basic_tests_P.HILInitialize_OOChannels, 3U,
        Aero_basic_tests_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }

    if (Aero_basic_tests_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (Aero_basic_tests_DW.HILInitialize_Card,
         Aero_basic_tests_P.HILInitialize_OOChannels, 3U,
         Aero_basic_tests_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for Constant: '<S3>/x0' */
  Aero_basic_tests_B.x0 = Aero_basic_tests_P.x0_Value;

  /* Start for ToFile: '<Root>/Save of the data in  the data_step_Qube.mat file' */
  {
    FILE *fp = (NULL);
    char fileName[509] = "data_step_AERO.mat";
    if ((fp = fopen(fileName, "wb")) == (NULL)) {
      rtmSetErrorStatus(Aero_basic_tests_M,
                        "Error creating .mat file data_step_AERO.mat");
      return;
    }

    if (rt_WriteMat4FileHeader(fp, 9 + 1, 0, "data0")) {
      fclose(fp);
      rtmSetErrorStatus(Aero_basic_tests_M,
                        "Error writing mat file header to file data_step_AERO.mat");
      return;
    }

    Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Count = 0;
    Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Decimation = -1;
    Aero_basic_tests_DW.Saveofthedatainthedata_step_Qub.FilePtr = fp;
  }

  /* Start for Atomic SubSystem: '<Root>/Bias Removal' */
  /* Start for Enabled SubSystem: '<S2>/Enabled Moving Average' */
  Aero_basic_tests_DW.EnabledMovingAverage_MODE = false;

  /* End of Start for SubSystem: '<S2>/Enabled Moving Average' */

  /* Start for SwitchCase: '<S2>/Switch Case' */
  Aero_basic_tests_DW.SwitchCase_ActiveSubsystem = -1;

  /* End of Start for SubSystem: '<Root>/Bias Removal' */

  /* InitializeConditions for Integrator: '<S3>/Integrator1' */
  if (rtmIsFirstInitCond(Aero_basic_tests_M)) {
    Aero_basic_tests_X.Integrator1_CSTATE[0] = 0.0;
    Aero_basic_tests_X.Integrator1_CSTATE[1] = 0.0;
    Aero_basic_tests_X.Integrator1_CSTATE[2] = 0.0;
  }

  Aero_basic_tests_DW.Integrator1_DWORK1 = true;

  /* End of InitializeConditions for Integrator: '<S3>/Integrator1' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/Bias Removal' */
  /* SystemInitialize for Enabled SubSystem: '<S2>/Enabled Moving Average' */
  /* InitializeConditions for UnitDelay: '<S8>/Unit Delay' */
  Aero_basic_tests_DW.UnitDelay_DSTATE =
    Aero_basic_tests_P.UnitDelay_InitialCondition;

  /* End of SystemInitialize for SubSystem: '<S2>/Enabled Moving Average' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem' */
  /* SystemInitialize for Outport: '<S5>/zero' incorporates:
   *  Constant: '<S5>/Constant'
   */
  Aero_basic_tests_B.Constant = Aero_basic_tests_P.zero_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem' */
  /* End of SystemInitialize for SubSystem: '<Root>/Bias Removal' */

  /* InitializeConditions for Integrator: '<S3>/Integrator2' */
  Aero_basic_tests_X.Integrator2_CSTATE[0] = Aero_basic_tests_P.Integrator2_IC;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Bias Removal' */
  /* SystemInitialize for Enabled SubSystem: '<S2>/Enabled Moving Average' */
  /* InitializeConditions for UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )' */
  Aero_basic_tests_DW.Sumk1n1xk_DSTATE[0] =
    Aero_basic_tests_P.Sumk1n1xk_InitialCondition;

  /* SystemInitialize for Product: '<S4>/div' incorporates:
   *  Outport: '<S4>/x_avg_n'
   */
  Aero_basic_tests_B.div[0] = Aero_basic_tests_P.x_avg_n_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Enabled Moving Average' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem1' */
  /* SystemInitialize for SignalConversion generated from: '<S6>/Vin' incorporates:
   *  Outport: '<S6>/Vbiased'
   */
  Aero_basic_tests_B.Vin[0] = Aero_basic_tests_P.Vbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* SystemInitialize for SignalConversion generated from: '<S7>/V-Vavg' incorporates:
   *  Outport: '<S7>/Vunbiased'
   */
  Aero_basic_tests_B.VVavg[0] = Aero_basic_tests_P.Vunbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* End of SystemInitialize for SubSystem: '<Root>/Bias Removal' */

  /* InitializeConditions for Integrator: '<S3>/Integrator2' */
  Aero_basic_tests_X.Integrator2_CSTATE[1] = Aero_basic_tests_P.Integrator2_IC;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Bias Removal' */
  /* SystemInitialize for Enabled SubSystem: '<S2>/Enabled Moving Average' */
  /* InitializeConditions for UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )' */
  Aero_basic_tests_DW.Sumk1n1xk_DSTATE[1] =
    Aero_basic_tests_P.Sumk1n1xk_InitialCondition;

  /* SystemInitialize for Product: '<S4>/div' incorporates:
   *  Outport: '<S4>/x_avg_n'
   */
  Aero_basic_tests_B.div[1] = Aero_basic_tests_P.x_avg_n_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Enabled Moving Average' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem1' */
  /* SystemInitialize for SignalConversion generated from: '<S6>/Vin' incorporates:
   *  Outport: '<S6>/Vbiased'
   */
  Aero_basic_tests_B.Vin[1] = Aero_basic_tests_P.Vbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* SystemInitialize for SignalConversion generated from: '<S7>/V-Vavg' incorporates:
   *  Outport: '<S7>/Vunbiased'
   */
  Aero_basic_tests_B.VVavg[1] = Aero_basic_tests_P.Vunbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* End of SystemInitialize for SubSystem: '<Root>/Bias Removal' */

  /* InitializeConditions for Integrator: '<S3>/Integrator2' */
  Aero_basic_tests_X.Integrator2_CSTATE[2] = Aero_basic_tests_P.Integrator2_IC;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Bias Removal' */
  /* SystemInitialize for Enabled SubSystem: '<S2>/Enabled Moving Average' */
  /* InitializeConditions for UnitDelay: '<S4>/Sum( k=1,n-1, x(k) )' */
  Aero_basic_tests_DW.Sumk1n1xk_DSTATE[2] =
    Aero_basic_tests_P.Sumk1n1xk_InitialCondition;

  /* SystemInitialize for Product: '<S4>/div' incorporates:
   *  Outport: '<S4>/x_avg_n'
   */
  Aero_basic_tests_B.div[2] = Aero_basic_tests_P.x_avg_n_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Enabled Moving Average' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem1' */
  /* SystemInitialize for SignalConversion generated from: '<S6>/Vin' incorporates:
   *  Outport: '<S6>/Vbiased'
   */
  Aero_basic_tests_B.Vin[2] = Aero_basic_tests_P.Vbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* SystemInitialize for SignalConversion generated from: '<S7>/V-Vavg' incorporates:
   *  Outport: '<S7>/Vunbiased'
   */
  Aero_basic_tests_B.VVavg[2] = Aero_basic_tests_P.Vunbiased_Y0;

  /* End of SystemInitialize for SubSystem: '<S2>/Switch Case Action Subsystem2' */
  /* End of SystemInitialize for SubSystem: '<Root>/Bias Removal' */

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(Aero_basic_tests_M)) {
    rtmSetFirstInitCond(Aero_basic_tests_M, 0);
  }
}

/* Model terminate function */
void Aero_basic_tests_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<Root>/HIL Initialize' */

  /* S-Function Block: Aero_basic_tests/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(Aero_basic_tests_DW.HILInitialize_Card);
    hil_monitor_stop_all(Aero_basic_tests_DW.HILInitialize_Card);
    is_switching = false;
    if ((Aero_basic_tests_P.HILInitialize_AOTerminate && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_AOExit && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_AOVoltages[0] =
        Aero_basic_tests_P.HILInitialize_AOFinal;
      Aero_basic_tests_DW.HILInitialize_AOVoltages[1] =
        Aero_basic_tests_P.HILInitialize_AOFinal;
      num_final_analog_outputs = 2U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((Aero_basic_tests_P.HILInitialize_DOTerminate && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_DOExit && is_switching)) {
      Aero_basic_tests_DW.HILInitialize_DOBits[0] =
        Aero_basic_tests_P.HILInitialize_DOFinal;
      Aero_basic_tests_DW.HILInitialize_DOBits[1] =
        Aero_basic_tests_P.HILInitialize_DOFinal;
      num_final_digital_outputs = 2U;
    } else {
      num_final_digital_outputs = 0;
    }

    if ((Aero_basic_tests_P.HILInitialize_OOTerminate && !is_switching) ||
        (Aero_basic_tests_P.HILInitialize_OOExit && is_switching)) {
      num_final_other_outputs = 3U;
    } else {
      num_final_other_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_digital_outputs > 0
        || num_final_other_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(Aero_basic_tests_DW.HILInitialize_Card
                         , Aero_basic_tests_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , Aero_basic_tests_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , Aero_basic_tests_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &Aero_basic_tests_DW.HILInitialize_AOVoltages[0]
                         , NULL
                         , (t_boolean *)
                         &Aero_basic_tests_DW.HILInitialize_DOBits[0]
                         , Aero_basic_tests_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(Aero_basic_tests_DW.HILInitialize_Card,
            Aero_basic_tests_P.HILInitialize_AOChannels,
            num_final_analog_outputs,
            &Aero_basic_tests_DW.HILInitialize_AOVoltages[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital
            (Aero_basic_tests_DW.HILInitialize_Card,
             Aero_basic_tests_P.HILInitialize_DOChannels,
             num_final_digital_outputs, (t_boolean *)
             &Aero_basic_tests_DW.HILInitialize_DOBits[0]);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other(Aero_basic_tests_DW.HILInitialize_Card,
            Aero_basic_tests_P.HILInitialize_OOChannels, num_final_other_outputs,
            Aero_basic_tests_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(Aero_basic_tests_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(Aero_basic_tests_DW.HILInitialize_Card);
    hil_monitor_delete_all(Aero_basic_tests_DW.HILInitialize_Card);
    hil_close(Aero_basic_tests_DW.HILInitialize_Card);
    Aero_basic_tests_DW.HILInitialize_Card = NULL;
  }

  /* Terminate for ToFile: '<Root>/Save of the data in  the data_step_Qube.mat file' */
  {
    FILE *fp = (FILE *)
      Aero_basic_tests_DW.Saveofthedatainthedata_step_Qub.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "data_step_AERO.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Aero_basic_tests_M,
                          "Error closing MAT-file data_step_AERO.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(Aero_basic_tests_M,
                          "Error reopening MAT-file data_step_AERO.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 9 + 1,
           Aero_basic_tests_DW.Saveofthedatainthedata_step_Q_a.Count, "data0"))
      {
        rtmSetErrorStatus(Aero_basic_tests_M,
                          "Error writing header for data0 to MAT-file data_step_AERO.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(Aero_basic_tests_M,
                          "Error closing MAT-file data_step_AERO.mat");
        return;
      }

      Aero_basic_tests_DW.Saveofthedatainthedata_step_Qub.FilePtr = (NULL);
    }
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  Aero_basic_tests_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  Aero_basic_tests_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  Aero_basic_tests_initialize();
}

void MdlTerminate(void)
{
  Aero_basic_tests_terminate();
}

/* Registration function */
RT_MODEL_Aero_basic_tests_T *Aero_basic_tests(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Aero_basic_tests_M, 0,
                sizeof(RT_MODEL_Aero_basic_tests_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Aero_basic_tests_M->solverInfo,
                          &Aero_basic_tests_M->Timing.simTimeStep);
    rtsiSetTPtr(&Aero_basic_tests_M->solverInfo, &rtmGetTPtr(Aero_basic_tests_M));
    rtsiSetStepSizePtr(&Aero_basic_tests_M->solverInfo,
                       &Aero_basic_tests_M->Timing.stepSize0);
    rtsiSetdXPtr(&Aero_basic_tests_M->solverInfo, &Aero_basic_tests_M->derivs);
    rtsiSetContStatesPtr(&Aero_basic_tests_M->solverInfo, (real_T **)
                         &Aero_basic_tests_M->contStates);
    rtsiSetNumContStatesPtr(&Aero_basic_tests_M->solverInfo,
      &Aero_basic_tests_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Aero_basic_tests_M->solverInfo,
      &Aero_basic_tests_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Aero_basic_tests_M->solverInfo,
      &Aero_basic_tests_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Aero_basic_tests_M->solverInfo,
      &Aero_basic_tests_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&Aero_basic_tests_M->solverInfo, (boolean_T**)
      &Aero_basic_tests_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&Aero_basic_tests_M->solverInfo, (&rtmGetErrorStatus
      (Aero_basic_tests_M)));
    rtsiSetRTModelPtr(&Aero_basic_tests_M->solverInfo, Aero_basic_tests_M);
  }

  rtsiSetSimTimeStep(&Aero_basic_tests_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&Aero_basic_tests_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&Aero_basic_tests_M->solverInfo, false);
  Aero_basic_tests_M->intgData.f[0] = Aero_basic_tests_M->odeF[0];
  Aero_basic_tests_M->contStates = ((real_T *) &Aero_basic_tests_X);
  Aero_basic_tests_M->contStateDisabled = ((boolean_T *) &Aero_basic_tests_XDis);
  Aero_basic_tests_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&Aero_basic_tests_M->solverInfo, (void *)
                    &Aero_basic_tests_M->intgData);
  rtsiSetSolverName(&Aero_basic_tests_M->solverInfo,"ode1");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = Aero_basic_tests_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    Aero_basic_tests_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    Aero_basic_tests_M->Timing.sampleTimes =
      (&Aero_basic_tests_M->Timing.sampleTimesArray[0]);
    Aero_basic_tests_M->Timing.offsetTimes =
      (&Aero_basic_tests_M->Timing.offsetTimesArray[0]);

    /* task periods */
    Aero_basic_tests_M->Timing.sampleTimes[0] = (0.0);
    Aero_basic_tests_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    Aero_basic_tests_M->Timing.offsetTimes[0] = (0.0);
    Aero_basic_tests_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(Aero_basic_tests_M, &Aero_basic_tests_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = Aero_basic_tests_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    Aero_basic_tests_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(Aero_basic_tests_M, 40.0);
  Aero_basic_tests_M->Timing.stepSize0 = 0.002;
  Aero_basic_tests_M->Timing.stepSize1 = 0.002;
  rtmSetFirstInitCond(Aero_basic_tests_M, 1);

  /* External mode info */
  Aero_basic_tests_M->Sizes.checksums[0] = (1559603274U);
  Aero_basic_tests_M->Sizes.checksums[1] = (1409213396U);
  Aero_basic_tests_M->Sizes.checksums[2] = (2156159343U);
  Aero_basic_tests_M->Sizes.checksums[3] = (408196979U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[6];
    Aero_basic_tests_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = (sysRanDType *)
      &Aero_basic_tests_DW.EnabledMovingAverage_SubsysRanB;
    systemRan[2] = (sysRanDType *)
      &Aero_basic_tests_DW.SwitchCaseActionSubsystem_Subsy;
    systemRan[3] = (sysRanDType *)
      &Aero_basic_tests_DW.SwitchCaseActionSubsystem1_Subs;
    systemRan[4] = (sysRanDType *)
      &Aero_basic_tests_DW.SwitchCaseActionSubsystem2_Subs;
    systemRan[5] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(Aero_basic_tests_M->extModeInfo,
      &Aero_basic_tests_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(Aero_basic_tests_M->extModeInfo,
                        Aero_basic_tests_M->Sizes.checksums);
    rteiSetTPtr(Aero_basic_tests_M->extModeInfo, rtmGetTPtr(Aero_basic_tests_M));
  }

  Aero_basic_tests_M->solverInfoPtr = (&Aero_basic_tests_M->solverInfo);
  Aero_basic_tests_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&Aero_basic_tests_M->solverInfo, 0.002);
  rtsiSetSolverMode(&Aero_basic_tests_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  Aero_basic_tests_M->blockIO = ((void *) &Aero_basic_tests_B);
  (void) memset(((void *) &Aero_basic_tests_B), 0,
                sizeof(B_Aero_basic_tests_T));

  /* parameters */
  Aero_basic_tests_M->defaultParam = ((real_T *)&Aero_basic_tests_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &Aero_basic_tests_X;
    Aero_basic_tests_M->contStates = (x);
    (void) memset((void *)&Aero_basic_tests_X, 0,
                  sizeof(X_Aero_basic_tests_T));
  }

  /* disabled states */
  {
    boolean_T *xdis = (boolean_T *) &Aero_basic_tests_XDis;
    Aero_basic_tests_M->contStateDisabled = (xdis);
    (void) memset((void *)&Aero_basic_tests_XDis, 0,
                  sizeof(XDis_Aero_basic_tests_T));
  }

  /* states (dwork) */
  Aero_basic_tests_M->dwork = ((void *) &Aero_basic_tests_DW);
  (void) memset((void *)&Aero_basic_tests_DW, 0,
                sizeof(DW_Aero_basic_tests_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    Aero_basic_tests_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 21;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  Aero_basic_tests_M->Sizes.numContStates = (6);/* Number of continuous states */
  Aero_basic_tests_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  Aero_basic_tests_M->Sizes.numY = (0);/* Number of model outputs */
  Aero_basic_tests_M->Sizes.numU = (0);/* Number of model inputs */
  Aero_basic_tests_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  Aero_basic_tests_M->Sizes.numSampTimes = (2);/* Number of sample times */
  Aero_basic_tests_M->Sizes.numBlocks = (65);/* Number of blocks */
  Aero_basic_tests_M->Sizes.numBlockIO = (23);/* Number of block outputs */
  Aero_basic_tests_M->Sizes.numBlockPrms = (133);/* Sum of parameter "widths" */
  return Aero_basic_tests_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
