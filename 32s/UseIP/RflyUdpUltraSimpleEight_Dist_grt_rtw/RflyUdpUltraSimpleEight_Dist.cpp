/*
 * RflyUdpUltraSimpleEight_Dist.cpp
 *
 * Code generation for model "RflyUdpUltraSimpleEight_Dist".
 *
 * Model version              : 10.1
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Tue Jan 10 10:08:34 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "RflyUdpUltraSimpleEight_Dist.h"
#include "rtwtypes.h"
#include <cmath>
#include "RflyUdpUltraSimpleEight_Dist_private.h"

extern "C" {

#include "rt_nonfinite.h"

}
/*
 * Time delay interpolation routine
 *
 * The linear interpolation is performed using the formula:
 *
 * (t2 - tMinusDelay)         (tMinusDelay - t1)
 * u(t)  =  ----------------- * u1  +  ------------------- * u2
 * (t2 - t1)                  (t2 - t1)
 */
  real_T rt_TDelayInterpolate(
  real_T tMinusDelay,                 /* tMinusDelay = currentSimTime - delay */
  real_T tStart,
  real_T *uBuf,
  int_T bufSz,
  int_T *lastIdx,
  int_T oldestIdx,
  int_T newIdx,
  real_T initOutput,
  boolean_T discrete,
  boolean_T minorStepAndTAtLastMajorOutput)
{
  int_T i;
  real_T yout, t1, t2, u1, u2;
  real_T* tBuf = uBuf + bufSz;

  /*
   * If there is only one data point in the buffer, this data point must be
   * the t= 0 and tMinusDelay > t0, it ask for something unknown. The best
   * guess if initial output as well
   */
  if ((newIdx == 0) && (oldestIdx ==0 ) && (tMinusDelay > tStart))
    return initOutput;

  /*
   * If tMinusDelay is less than zero, should output initial value
   */
  if (tMinusDelay <= tStart)
    return initOutput;

  /* For fixed buffer extrapolation:
   * if tMinusDelay is small than the time at oldestIdx, if discrete, output
   * tailptr value,  else use tailptr and tailptr+1 value to extrapolate
   * It is also for fixed buffer. Note: The same condition can happen for transport delay block where
   * use tStart and and t[tail] other than using t[tail] and t[tail+1].
   * See below
   */
  if ((tMinusDelay <= tBuf[oldestIdx] ) ) {
    if (discrete) {
      return(uBuf[oldestIdx]);
    } else {
      int_T tempIdx= oldestIdx + 1;
      if (oldestIdx == bufSz-1)
        tempIdx = 0;
      t1= tBuf[oldestIdx];
      t2= tBuf[tempIdx];
      u1= uBuf[oldestIdx];
      u2= uBuf[tempIdx];
      if (t2 == t1) {
        if (tMinusDelay >= t2) {
          yout = u2;
        } else {
          yout = u1;
        }
      } else {
        real_T f1 = (t2-tMinusDelay) / (t2-t1);
        real_T f2 = 1.0 - f1;

        /*
         * Use Lagrange's interpolation formula.  Exact outputs at t1, t2.
         */
        yout = f1*u1 + f2*u2;
      }

      return yout;
    }
  }

  /*
   * When block does not have direct feedthrough, we use the table of
   * values to extrapolate off the end of the table for delays that are less
   * than 0 (less then step size).  This is not completely accurate.  The
   * chain of events is as follows for a given time t.  Major output - look
   * in table.  Update - add entry to table.  Now, if we call the output at
   * time t again, there is a new entry in the table. For very small delays,
   * this means that we will have a different answer from the previous call
   * to the output fcn at the same time t.  The following code prevents this
   * from happening.
   */
  if (minorStepAndTAtLastMajorOutput) {
    /* pretend that the new entry has not been added to table */
    if (newIdx != 0) {
      if (*lastIdx == newIdx) {
        (*lastIdx)--;
      }

      newIdx--;
    } else {
      if (*lastIdx == newIdx) {
        *lastIdx = bufSz-1;
      }

      newIdx = bufSz - 1;
    }
  }

  i = *lastIdx;
  if (tBuf[i] < tMinusDelay) {
    /* Look forward starting at last index */
    while (tBuf[i] < tMinusDelay) {
      /* May occur if the delay is less than step-size - extrapolate */
      if (i == newIdx)
        break;
      i = ( i < (bufSz-1) ) ? (i+1) : 0;/* move through buffer */
    }
  } else {
    /*
     * Look backwards starting at last index which can happen when the
     * delay time increases.
     */
    while (tBuf[i] >= tMinusDelay) {
      /*
       * Due to the entry condition at top of function, we
       * should never hit the end.
       */
      i = (i > 0) ? i-1 : (bufSz-1);   /* move through buffer */
    }

    i = ( i < (bufSz-1) ) ? (i+1) : 0;
  }

  *lastIdx = i;
  if (discrete) {
    /*
     * tempEps = 128 * eps;
     * localEps = max(tempEps, tempEps*fabs(tBuf[i]))/2;
     */
    double tempEps = (DBL_EPSILON) * 128.0;
    double localEps = tempEps * std::abs(tBuf[i]);
    if (tempEps > localEps) {
      localEps = tempEps;
    }

    localEps = localEps / 2.0;
    if (tMinusDelay >= (tBuf[i] - localEps)) {
      yout = uBuf[i];
    } else {
      if (i == 0) {
        yout = uBuf[bufSz-1];
      } else {
        yout = uBuf[i-1];
      }
    }
  } else {
    if (i == 0) {
      t1 = tBuf[bufSz-1];
      u1 = uBuf[bufSz-1];
    } else {
      t1 = tBuf[i-1];
      u1 = uBuf[i-1];
    }

    t2 = tBuf[i];
    u2 = uBuf[i];
    if (t2 == t1) {
      if (tMinusDelay >= t2) {
        yout = u2;
      } else {
        yout = u1;
      }
    } else {
      real_T f1 = (t2-tMinusDelay) / (t2-t1);
      real_T f2 = 1.0 - f1;

      /*
       * Use Lagrange's interpolation formula.  Exact outputs at t1, t2.
       */
      yout = f1*u1 + f2*u2;
    }
  }

  return(yout);
}

/* Model step function */
void RflyUdpUltraSimpleEight_Dist::step()
{
  /* local block i/o variables */
  real_T rtb_SineWave;
  real_T rtb_SineWave1;
  real_T rtb_SineWave2;
  real_T rtb_SineWave3;
  real_T rtb_SineWave4;
  real_T rtb_SineWave5;
  real_T rtb_SineWave6;
  real_T rtb_SineWave7;
  real_T rtb_SineWave10;
  real_T rtb_SineWave11;
  real_T rtb_SineWave12;
  real_T rtb_SineWave13;
  real_T rtb_SineWave14;
  real_T rtb_SineWave15;
  real_T rtb_SineWave8;
  real_T rtb_SineWave9;
  real_T rtb_TransportDelay15;
  real_T rtb_TransportDelay14;
  real_T rtb_SineWave_tmp;

  /* S-Function (RflyUdpFast): '<Root>/UDP Recv1' */

  /* Level2 S-Function Block: '<Root>/UDP Recv1' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0];
    sfcnOutputs(rts,0);
  }

  /* Gain: '<Root>/Gain1' incorporates:
   *  Constant: '<Root>/期望高度'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain1 =
    RflyUdpUltraSimpleEight_Dist_P.Gain1_Gain *
    RflyUdpUltraSimpleEight_Dist_P._Value;

  /* Gain: '<Root>/Gain2' incorporates:
   *  Constant: '<Root>/期望高度1'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain2 =
    RflyUdpUltraSimpleEight_Dist_P.Gain2_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value;

  /* Gain: '<Root>/Gain3' incorporates:
   *  Constant: '<Root>/期望高度2'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain3 =
    RflyUdpUltraSimpleEight_Dist_P.Gain3_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_e;

  /* Gain: '<Root>/Gain4' incorporates:
   *  Constant: '<Root>/期望高度3'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain4 =
    RflyUdpUltraSimpleEight_Dist_P.Gain4_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_j;

  /* Sin: '<Root>/Sine Wave' incorporates:
   *  Sin: '<Root>/Sine Wave1'
   *  Sin: '<Root>/Sine Wave10'
   *  Sin: '<Root>/Sine Wave11'
   *  Sin: '<Root>/Sine Wave12'
   *  Sin: '<Root>/Sine Wave13'
   *  Sin: '<Root>/Sine Wave14'
   *  Sin: '<Root>/Sine Wave15'
   *  Sin: '<Root>/Sine Wave2'
   *  Sin: '<Root>/Sine Wave3'
   *  Sin: '<Root>/Sine Wave4'
   *  Sin: '<Root>/Sine Wave5'
   *  Sin: '<Root>/Sine Wave6'
   *  Sin: '<Root>/Sine Wave7'
   *  Sin: '<Root>/Sine Wave8'
   *  Sin: '<Root>/Sine Wave9'
   */
  rtb_SineWave_tmp = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];

  /* Sin: '<Root>/Sine Wave' */
  rtb_SineWave = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave_Bias;

  /* Sin: '<Root>/Sine Wave1' */
  rtb_SineWave1 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave1_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave1_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave1_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave1_Bias;

  /* Sin: '<Root>/Sine Wave2' */
  rtb_SineWave2 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave2_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave2_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave2_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave2_Bias;

  /* Sin: '<Root>/Sine Wave3' */
  rtb_SineWave3 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave3_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave3_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave3_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave3_Bias;

  /* Sin: '<Root>/Sine Wave4' */
  rtb_SineWave4 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave4_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave4_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave4_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave4_Bias;

  /* Sin: '<Root>/Sine Wave5' */
  rtb_SineWave5 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave5_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave5_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave5_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave5_Bias;

  /* Sin: '<Root>/Sine Wave6' */
  rtb_SineWave6 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave6_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave6_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave6_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave6_Bias;

  /* Sin: '<Root>/Sine Wave7' */
  rtb_SineWave7 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave7_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave7_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave7_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave7_Bias;

  /* TransportDelay: '<Root>/Transport Delay' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay_Delay;
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay1' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay1_Delay;
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay1_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum' */
  RflyUdpUltraSimpleEight_Dist_B.Sum[0] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o1[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum[1] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o1[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum[2] = RflyUdpUltraSimpleEight_Dist_B.Gain1 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o1[5];

  /* TransportDelay: '<Root>/Transport Delay2' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay2_Delay;
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay2_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay3' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay3_Delay;
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay3_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum1' */
  RflyUdpUltraSimpleEight_Dist_B.Sum1[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o2[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum1[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o2[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum1[2] = RflyUdpUltraSimpleEight_Dist_B.Gain2
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o2[5];

  /* TransportDelay: '<Root>/Transport Delay4' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay4_Delay;
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay4_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay5' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay5_Delay;
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay5_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum2' */
  RflyUdpUltraSimpleEight_Dist_B.Sum2[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o3[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum2[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o3[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum2[2] = RflyUdpUltraSimpleEight_Dist_B.Gain3
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o3[5];

  /* TransportDelay: '<Root>/Transport Delay6' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay6_Delay;
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay6_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay7' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay7_Delay;
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay7_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum3' */
  RflyUdpUltraSimpleEight_Dist_B.Sum3[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o4[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum3[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o4[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum3[2] = RflyUdpUltraSimpleEight_Dist_B.Gain4
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o4[5];

  /* Constant: '<Root>/vel1' */
  RflyUdpUltraSimpleEight_Dist_B.vel1 =
    RflyUdpUltraSimpleEight_Dist_P.vel1_Value;

  /* Constant: '<Root>/vel2' */
  RflyUdpUltraSimpleEight_Dist_B.vel2 =
    RflyUdpUltraSimpleEight_Dist_P.vel2_Value;

  /* Constant: '<Root>/vel3' */
  RflyUdpUltraSimpleEight_Dist_B.vel3 =
    RflyUdpUltraSimpleEight_Dist_P.vel3_Value;

  /* Constant: '<Root>/vel4' */
  RflyUdpUltraSimpleEight_Dist_B.vel4 =
    RflyUdpUltraSimpleEight_Dist_P.vel4_Value;

  /* Constant: '<Root>/vel5' */
  RflyUdpUltraSimpleEight_Dist_B.vel5 =
    RflyUdpUltraSimpleEight_Dist_P.vel5_Value;

  /* Constant: '<Root>/vel6' */
  RflyUdpUltraSimpleEight_Dist_B.vel6 =
    RflyUdpUltraSimpleEight_Dist_P.vel6_Value;

  /* Constant: '<Root>/vel7' */
  RflyUdpUltraSimpleEight_Dist_B.vel7 =
    RflyUdpUltraSimpleEight_Dist_P.vel7_Value;

  /* Constant: '<Root>/vel8' */
  RflyUdpUltraSimpleEight_Dist_B.vel8 =
    RflyUdpUltraSimpleEight_Dist_P.vel8_Value;

  /* S-Function (RflyUdpFast): '<Root>/UDP Recv2' */

  /* Level2 S-Function Block: '<Root>/UDP Recv2' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1];
    sfcnOutputs(rts,0);
  }

  /* Gain: '<Root>/Gain5' incorporates:
   *  Constant: '<Root>/期望高度4'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain5 =
    RflyUdpUltraSimpleEight_Dist_P.Gain5_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_g;

  /* Gain: '<Root>/Gain6' incorporates:
   *  Constant: '<Root>/期望高度5'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain6 =
    RflyUdpUltraSimpleEight_Dist_P.Gain6_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_k;

  /* Gain: '<Root>/Gain7' incorporates:
   *  Constant: '<Root>/期望高度6'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain7 =
    RflyUdpUltraSimpleEight_Dist_P.Gain7_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_o;

  /* Gain: '<Root>/Gain8' incorporates:
   *  Constant: '<Root>/期望高度7'
   */
  RflyUdpUltraSimpleEight_Dist_B.Gain8 =
    RflyUdpUltraSimpleEight_Dist_P.Gain8_Gain *
    RflyUdpUltraSimpleEight_Dist_P.u_Value_c;

  /* Sin: '<Root>/Sine Wave10' */
  rtb_SineWave10 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave10_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave10_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave10_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave10_Bias;

  /* Sin: '<Root>/Sine Wave11' */
  rtb_SineWave11 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave11_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave11_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave11_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave11_Bias;

  /* Sin: '<Root>/Sine Wave12' */
  rtb_SineWave12 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave12_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave12_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave12_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave12_Bias;

  /* Sin: '<Root>/Sine Wave13' */
  rtb_SineWave13 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave13_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave13_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave13_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave13_Bias;

  /* Sin: '<Root>/Sine Wave14' */
  rtb_SineWave14 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave14_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave14_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave14_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave14_Bias;

  /* Sin: '<Root>/Sine Wave15' */
  rtb_SineWave15 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave15_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave15_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave15_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave15_Bias;

  /* Sin: '<Root>/Sine Wave8' */
  rtb_SineWave8 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave8_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave8_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave8_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave8_Bias;

  /* Sin: '<Root>/Sine Wave9' */
  rtb_SineWave9 = std::sin(RflyUdpUltraSimpleEight_Dist_P.SineWave9_Freq *
    rtb_SineWave_tmp + RflyUdpUltraSimpleEight_Dist_P.SineWave9_Phase) *
    RflyUdpUltraSimpleEight_Dist_P.SineWave9_Amp +
    RflyUdpUltraSimpleEight_Dist_P.SineWave9_Bias;

  /* TransportDelay: '<Root>/Transport Delay8' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay8_Delay);
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay8_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay9' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay9_Delay);
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay9_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum4' */
  RflyUdpUltraSimpleEight_Dist_B.Sum4[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o1[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum4[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o1[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum4[2] = RflyUdpUltraSimpleEight_Dist_B.Gain5
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o1[5];

  /* TransportDelay: '<Root>/Transport Delay10' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay10_Delay);
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay10_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay11' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay11_Delay);
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay11_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum5' */
  RflyUdpUltraSimpleEight_Dist_B.Sum5[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o2[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum5[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o2[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum5[2] = RflyUdpUltraSimpleEight_Dist_B.Gain6
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o2[5];

  /* TransportDelay: '<Root>/Transport Delay12' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay12_Delay);
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay12_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay13' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay13_Delay);
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay13_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum6' */
  RflyUdpUltraSimpleEight_Dist_B.Sum6[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o3[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum6[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o3[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum6[2] = RflyUdpUltraSimpleEight_Dist_B.Gain7
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o3[5];

  /* TransportDelay: '<Root>/Transport Delay14' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay14_Delay);
    rtb_TransportDelay14 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay14_InitOutput,
      0,
      0);
  }

  /* TransportDelay: '<Root>/Transport Delay15' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    real_T tMinusDelay = simTime -
      (RflyUdpUltraSimpleEight_Dist_P.TransportDelay15_Delay);
    rtb_TransportDelay15 = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *uBuffer,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.CircularBufSize,
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Last,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail,
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head,
      RflyUdpUltraSimpleEight_Dist_P.TransportDelay15_InitOutput,
      0,
      0);
  }

  /* Sum: '<Root>/Sum7' */
  RflyUdpUltraSimpleEight_Dist_B.Sum7[0] = rtb_TransportDelay14 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o4[3];
  RflyUdpUltraSimpleEight_Dist_B.Sum7[1] = rtb_TransportDelay15 -
    RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o4[4];
  RflyUdpUltraSimpleEight_Dist_B.Sum7[2] = RflyUdpUltraSimpleEight_Dist_B.Gain8
    - RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o4[5];

  /* Constant: '<Root>/vel10' */
  RflyUdpUltraSimpleEight_Dist_B.vel10 =
    RflyUdpUltraSimpleEight_Dist_P.vel10_Value;

  /* Constant: '<Root>/vel11' */
  RflyUdpUltraSimpleEight_Dist_B.vel11 =
    RflyUdpUltraSimpleEight_Dist_P.vel11_Value;

  /* Constant: '<Root>/vel12' */
  RflyUdpUltraSimpleEight_Dist_B.vel12 =
    RflyUdpUltraSimpleEight_Dist_P.vel12_Value;

  /* Constant: '<Root>/vel13' */
  RflyUdpUltraSimpleEight_Dist_B.vel13 =
    RflyUdpUltraSimpleEight_Dist_P.vel13_Value;

  /* Constant: '<Root>/vel14' */
  RflyUdpUltraSimpleEight_Dist_B.vel14 =
    RflyUdpUltraSimpleEight_Dist_P.vel14_Value;

  /* Constant: '<Root>/vel15' */
  RflyUdpUltraSimpleEight_Dist_B.vel15 =
    RflyUdpUltraSimpleEight_Dist_P.vel15_Value;

  /* Constant: '<Root>/vel16' */
  RflyUdpUltraSimpleEight_Dist_B.vel16 =
    RflyUdpUltraSimpleEight_Dist_P.vel16_Value;

  /* Constant: '<Root>/vel9' */
  RflyUdpUltraSimpleEight_Dist_B.vel9 =
    RflyUdpUltraSimpleEight_Dist_P.vel9_Value;

  /* S-Function (saeroclockpacer): '<Root>/Simulation Pace1' */
  /*
   * The Clock Pacer generates no code, it is only active in
   * interpreted simulation.
   */

  /* Matfile logging */
  rt_UpdateTXYLogVars((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo,
                      ((&RflyUdpUltraSimpleEight_Dist_M)->Timing.t));

  /* Update for S-Function (RflyUdpFast): '<Root>/UDP Recv1' */
  /* Level2 S-Function Block: '<Root>/UDP Recv1' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0];
    sfcnUpdate(rts,0);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.CircularBufSize-
           1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail+1) :
         0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head] =
      rtb_SineWave;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay1' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head] =
      rtb_SineWave1;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay2' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head] =
      rtb_SineWave2;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay3' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head] =
      rtb_SineWave3;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay4' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head] =
      rtb_SineWave4;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay5' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head] =
      rtb_SineWave5;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay6' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head] =
      rtb_SineWave6;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay7' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head] =
      rtb_SineWave7;
  }

  /* Update for S-Function (RflyUdpFast): '<Root>/UDP Recv2' */
  /* Level2 S-Function Block: '<Root>/UDP Recv2' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1];
    sfcnUpdate(rts,0);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay8' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head] =
      rtb_SineWave8;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay9' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.CircularBufSize-1))
       ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head+1) : 0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head] =
      rtb_SineWave9;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay10' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head] =
      rtb_SineWave10;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay11' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head] =
      rtb_SineWave11;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay12' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head] =
      rtb_SineWave12;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay13' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head] =
      rtb_SineWave13;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay14' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head] =
      rtb_SineWave14;
  }

  /* Update for TransportDelay: '<Root>/Transport Delay15' */
  {
    real_T **uBuffer = (real_T**)
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_PWORK.TUbufferPtrs[0];
    real_T simTime = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head =
      ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head <
        (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.CircularBufSize-
         1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head+1) :
       0);
    if (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head ==
        RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail) {
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail =
        ((RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail <
          (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.CircularBufSize
           -1)) ? (RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail+1)
         : 0);
    }

    (*uBuffer +
      RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.CircularBufSize)
      [RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head] = simTime;
    (*uBuffer)[RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head] =
      rtb_SineWave15;
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal((&RflyUdpUltraSimpleEight_Dist_M))!=-1) &&
        !((rtmGetTFinal((&RflyUdpUltraSimpleEight_Dist_M))-
           (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0]) >
          (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus((&RflyUdpUltraSimpleEight_Dist_M), "Simulation finished");
    }

    if (rtmGetStopRequested((&RflyUdpUltraSimpleEight_Dist_M))) {
      rtmSetErrorStatus((&RflyUdpUltraSimpleEight_Dist_M), "Simulation finished");
    }
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
  if (!(++(&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTick0)) {
    ++(&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTickH0;
  }

  (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0] =
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTick0 *
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize0 +
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTickH0 *
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.033333333333333333s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTick1)) {
      ++(&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTickH1;
    }

    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[1] =
      (&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTick1 *
      (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize1 +
      (&RflyUdpUltraSimpleEight_Dist_M)->Timing.clockTickH1 *
      (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void RflyUdpUltraSimpleEight_Dist::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                          &(&RflyUdpUltraSimpleEight_Dist_M)->Timing.simTimeStep);
    rtsiSetTPtr(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo, &rtmGetTPtr
                ((&RflyUdpUltraSimpleEight_Dist_M)));
    rtsiSetStepSizePtr(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                       &(&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                          (&rtmGetErrorStatus((&RflyUdpUltraSimpleEight_Dist_M))));
    rtsiSetRTModelPtr(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                      (&RflyUdpUltraSimpleEight_Dist_M));
  }

  rtsiSetSimTimeStep(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                     MAJOR_TIME_STEP);
  rtsiSetSolverName(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                    "FixedStepDiscrete");
  (&RflyUdpUltraSimpleEight_Dist_M)->solverInfoPtr =
    (&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = (&RflyUdpUltraSimpleEight_Dist_M)
      ->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;

    /* polyspace +2 MISRA2012:D4.1 [Justified:Low] "(&RflyUdpUltraSimpleEight_Dist_M) points to
       static memory which is guaranteed to be non-NULL" */
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleTimes =
      (&(&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleTimesArray[0]);
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.offsetTimes =
      (&(&RflyUdpUltraSimpleEight_Dist_M)->Timing.offsetTimesArray[0]);

    /* task periods */
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleTimes[0] = (0.0);
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleTimes[1] =
      (0.033333333333333333);

    /* task offsets */
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.offsetTimes[0] = (0.0);
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr((&RflyUdpUltraSimpleEight_Dist_M),
             &(&RflyUdpUltraSimpleEight_Dist_M)->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = (&RflyUdpUltraSimpleEight_Dist_M)
      ->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal((&RflyUdpUltraSimpleEight_Dist_M), -1);
  (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize0 = 0.033333333333333333;
  (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize1 = 0.033333333333333333;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    (&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, (NULL));
    rtliSetLogT((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, "tout");
    rtliSetLogX((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, "");
    rtliSetLogXFinal((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, "");
    rtliSetLogVarNameModifier((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo,
      "rt_");
    rtliSetLogFormat((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, 0);
    rtliSetLogMaxRows((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, 0);
    rtliSetLogDecimation((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, 1);
    rtliSetLogY((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, "");
    rtliSetLogYSignalInfo((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo, (NULL));
  }

  (&RflyUdpUltraSimpleEight_Dist_M)->solverInfoPtr =
    (&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo);
  (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize = (0.033333333333333333);
  rtsiSetFixedStepSize(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                       0.033333333333333333);
  rtsiSetSolverMode(&(&RflyUdpUltraSimpleEight_Dist_M)->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &(&RflyUdpUltraSimpleEight_Dist_M)
      ->NonInlinedSFcns.sfcnInfo;
    (&RflyUdpUltraSimpleEight_Dist_M)->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus
      ((&RflyUdpUltraSimpleEight_Dist_M))));
    (&RflyUdpUltraSimpleEight_Dist_M)->Sizes.numSampTimes = (2);
    rtssSetNumRootSampTimesPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
      ->Sizes.numSampTimes);
    (&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.taskTimePtrs[0] =
      &(rtmGetTPtr((&RflyUdpUltraSimpleEight_Dist_M))[0]);
    (&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.taskTimePtrs[1] =
      &(rtmGetTPtr((&RflyUdpUltraSimpleEight_Dist_M))[1]);
    rtssSetTPtrPtr(sfcnInfo,(&RflyUdpUltraSimpleEight_Dist_M)
                   ->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart((&RflyUdpUltraSimpleEight_Dist_M)));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal((&RflyUdpUltraSimpleEight_Dist_M)));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      ((&RflyUdpUltraSimpleEight_Dist_M)));
    rtssSetStepSizePtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
                       ->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested
      ((&RflyUdpUltraSimpleEight_Dist_M)));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M
      )->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
      ->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &(&RflyUdpUltraSimpleEight_Dist_M)->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
      ->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->solverInfoPtr);
  }

  (&RflyUdpUltraSimpleEight_Dist_M)->Sizes.numSFcns = (2);

  /* register each child */
  {
    (void) std::memset(static_cast<void *>(&(&RflyUdpUltraSimpleEight_Dist_M)
      ->NonInlinedSFcns.childSFunctions[0]), 0,
                       2*sizeof(SimStruct));
    (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions =
      (&(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.childSFunctionPtrs[0]);
    (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0] =
      (&(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.childSFunctions[0]);
    (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1] =
      (&(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.childSFunctions[1]);

    /* Level2 S-Function Block: RflyUdpUltraSimpleEight_Dist/<Root>/UDP Recv1 (RflyUdpFast) */
    {
      SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->NonInlinedSFcns.blkInfo2[0]);
        ssSetBlkInfoSLSizePtr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                              ->NonInlinedSFcns.blkInfoSLSize[0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, (&RflyUdpUltraSimpleEight_Dist_M)->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods4[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->NonInlinedSFcns.statesInfo2[0]);
        ssSetPeriodicStatesInfo(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 4);
        ssSetPortInfoForInputs(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        ssSetInputPortUnit(rts, 3, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);
        ssSetInputPortIsContinuousQuantity(rts, 3, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.UPtrs0;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel5;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel2;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 5);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.UPtrs1;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel6;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum1[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum1[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum1[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel1;
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidthAsInt(rts, 1, 5);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.UPtrs2;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel7;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum2[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum2[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum2[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel3;
          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidthAsInt(rts, 2, 5);
        }

        /* port 3 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.UPtrs3;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel8;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum3[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum3[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum3[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel4;
          ssSetInputPortSignalPtrs(rts, 3, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidthAsInt(rts, 3, 5);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->outputs =
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.outputPortInfoSLSize[0];
        _ssSetNumOutputPorts(rts, 4);
        _ssSetPortInfo2ForOutputUnits(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute[0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidthAsInt(rts, 0, 12);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidthAsInt(rts, 1, 12);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o2));
        }

        /* port 2 */
        {
          _ssSetOutputPortNumDimensions(rts, 2, 1);
          ssSetOutputPortWidthAsInt(rts, 2, 12);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o3));
        }

        /* port 3 */
        {
          _ssSetOutputPortNumDimensions(rts, 3, 1);
          ssSetOutputPortWidthAsInt(rts, 3, 12);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv1_o4));
        }
      }

      /* path info */
      ssSetModelName(rts, "UDP Recv1");
      ssSetPath(rts, "RflyUdpUltraSimpleEight_Dist/UDP Recv1");
      ssSetRTModel(rts,(&RflyUdpUltraSimpleEight_Dist_M));
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **) &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 5);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv1_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv1_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv1_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv1_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv1_P5_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &RflyUdpUltraSimpleEight_Dist_DW.UDPRecv1_PWORK
                 [0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 5);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RflyUdpUltraSimpleEight_Dist_DW.UDPRecv1_PWORK[0]);
      }

      /* registration */
      RflyUdpFast(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
    }

    /* Level2 S-Function Block: RflyUdpUltraSimpleEight_Dist/<Root>/UDP Recv2 (RflyUdpFast) */
    {
      SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = (&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->NonInlinedSFcns.blkInfo2[1]);
        ssSetBlkInfoSLSizePtr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                              ->NonInlinedSFcns.blkInfoSLSize[1]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
        ->NonInlinedSFcns.inputOutputPortInfo2[1]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, (&RflyUdpUltraSimpleEight_Dist_M)->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                           ->NonInlinedSFcns.methods4[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
                         ->NonInlinedSFcns.statesInfo2[1]);
        ssSetPeriodicStatesInfo(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.periodicStatesInfo[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 4);
        ssSetPortInfoForInputs(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        ssSetInputPortUnit(rts, 1, 0);
        ssSetInputPortUnit(rts, 2, 0);
        ssSetInputPortUnit(rts, 3, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);
        ssSetInputPortIsContinuousQuantity(rts, 1, 0);
        ssSetInputPortIsContinuousQuantity(rts, 2, 0);
        ssSetInputPortIsContinuousQuantity(rts, 3, 0);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.UPtrs0;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel13;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum4[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum4[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum4[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel10;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 5);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.UPtrs1;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel14;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum5[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum5[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum5[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel9;
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidthAsInt(rts, 1, 5);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.UPtrs2;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel15;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum6[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum6[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum6[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel11;
          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidthAsInt(rts, 2, 5);
        }

        /* port 3 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.UPtrs3;
          sfcnUPtrs[0] = &RflyUdpUltraSimpleEight_Dist_B.vel16;
          sfcnUPtrs[1] = &RflyUdpUltraSimpleEight_Dist_B.Sum7[0];
          sfcnUPtrs[2] = &RflyUdpUltraSimpleEight_Dist_B.Sum7[1];
          sfcnUPtrs[3] = &RflyUdpUltraSimpleEight_Dist_B.Sum7[2];
          sfcnUPtrs[4] = &RflyUdpUltraSimpleEight_Dist_B.vel12;
          ssSetInputPortSignalPtrs(rts, 3, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidthAsInt(rts, 3, 5);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.outputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->outputs =
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.outputPortInfoSLSize[0];
        _ssSetNumOutputPorts(rts, 4);
        _ssSetPortInfo2ForOutputUnits(rts, &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        ssSetOutputPortUnit(rts, 2, 0);
        ssSetOutputPortUnit(rts, 3, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.outputPortCoSimAttribute[0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 2, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 3, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidthAsInt(rts, 0, 12);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidthAsInt(rts, 1, 12);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o2));
        }

        /* port 2 */
        {
          _ssSetOutputPortNumDimensions(rts, 2, 1);
          ssSetOutputPortWidthAsInt(rts, 2, 12);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o3));
        }

        /* port 3 */
        {
          _ssSetOutputPortNumDimensions(rts, 3, 1);
          ssSetOutputPortWidthAsInt(rts, 3, 12);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            RflyUdpUltraSimpleEight_Dist_B.UDPRecv2_o4));
        }
      }

      /* path info */
      ssSetModelName(rts, "UDP Recv2");
      ssSetPath(rts, "RflyUdpUltraSimpleEight_Dist/UDP Recv2");
      ssSetRTModel(rts,(&RflyUdpUltraSimpleEight_Dist_M));
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **) &(&RflyUdpUltraSimpleEight_Dist_M)
          ->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 5);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv2_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv2_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv2_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv2_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)
                       RflyUdpUltraSimpleEight_Dist_P.UDPRecv2_P5_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &RflyUdpUltraSimpleEight_Dist_DW.UDPRecv2_PWORK
                 [0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &(&RflyUdpUltraSimpleEight_Dist_M)->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 5);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &RflyUdpUltraSimpleEight_Dist_DW.UDPRecv2_PWORK[0]);
      }

      /* registration */
      RflyUdpFast(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.033333333333333333);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
    }
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime((&RflyUdpUltraSimpleEight_Dist_M)->rtwLogInfo,
    0.0, rtmGetTFinal((&RflyUdpUltraSimpleEight_Dist_M)),
    (&RflyUdpUltraSimpleEight_Dist_M)->Timing.stepSize0, (&rtmGetErrorStatus
    ((&RflyUdpUltraSimpleEight_Dist_M))));

  /* Start for S-Function (RflyUdpFast): '<Root>/UDP Recv1' */
  /* Level2 S-Function Block: '<Root>/UDP Recv1' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for TransportDelay: '<Root>/Transport Delay' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay_PWORK.TUbufferPtrs[0] = (void
      *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay1' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay1_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay1_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay2' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay2_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay2_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay3' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay3_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay3_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay4' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay4_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay4_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay5' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay5_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay5_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay6' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay6_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay6_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay7' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay7_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay7_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for S-Function (RflyUdpFast): '<Root>/UDP Recv2' */
  /* Level2 S-Function Block: '<Root>/UDP Recv2' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for TransportDelay: '<Root>/Transport Delay8' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay8_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay8_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay9' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_IWORK.CircularBufSize = 1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay9_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay9_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay10' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay10_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay10_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay11' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay11_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay11_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay12' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay12_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay12_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay13' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay13_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay13_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay14' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay14_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay14_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }

  /* Start for TransportDelay: '<Root>/Transport Delay15' */
  {
    real_T *pBuffer =
      &RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_RWORK.TUbufferArea[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Tail = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Head = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.Last = 0;
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_IWORK.CircularBufSize =
      1024;
    pBuffer[0] = RflyUdpUltraSimpleEight_Dist_P.TransportDelay15_InitOutput;
    pBuffer[1024] = (&RflyUdpUltraSimpleEight_Dist_M)->Timing.t[0];
    RflyUdpUltraSimpleEight_Dist_DW.TransportDelay15_PWORK.TUbufferPtrs[0] =
      (void *) &pBuffer[0];
  }
}

/* Model terminate function */
void RflyUdpUltraSimpleEight_Dist::terminate()
{
  /* Terminate for S-Function (RflyUdpFast): '<Root>/UDP Recv1' */
  /* Level2 S-Function Block: '<Root>/UDP Recv1' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (RflyUdpFast): '<Root>/UDP Recv2' */
  /* Level2 S-Function Block: '<Root>/UDP Recv2' (RflyUdpFast) */
  {
    SimStruct *rts = (&RflyUdpUltraSimpleEight_Dist_M)->childSfunctions[1];
    sfcnTerminate(rts);
  }
}

/* Constructor */
RflyUdpUltraSimpleEight_Dist::RflyUdpUltraSimpleEight_Dist() :
  RflyUdpUltraSimpleEight_Dist_B(),
  RflyUdpUltraSimpleEight_Dist_DW(),
  RflyUdpUltraSimpleEight_Dist_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
RflyUdpUltraSimpleEight_Dist::~RflyUdpUltraSimpleEight_Dist()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_RflyUdpUltraSimpleEight_Dist_T * RflyUdpUltraSimpleEight_Dist::getRTM()
{
  return (&RflyUdpUltraSimpleEight_Dist_M);
}
