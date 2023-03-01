/*
 * RflyUdpUltraSimpleEight_Dist.h
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

#ifndef RTW_HEADER_RflyUdpUltraSimpleEight_Dist_h_
#define RTW_HEADER_RflyUdpUltraSimpleEight_Dist_h_
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_logging.h"
#include "RflyUdpUltraSimpleEight_Dist_types.h"
#include <stddef.h>
#include <cfloat>
#include <cmath>
#include <cstring>

extern "C" {

#include "rt_nonfinite.h"

}
/* Macros for accessing real-time model data structure */
#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetSampleHitArray
#define rtmGetSampleHitArray(rtm)      ((rtm)->Timing.sampleHitArray)
#endif

#ifndef rtmGetStepSize
#define rtmGetStepSize(rtm)            ((rtm)->Timing.stepSize)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGet_TimeOfLastOutput
#define rtmGet_TimeOfLastOutput(rtm)   ((rtm)->Timing.timeOfLastOutput)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#ifndef rtmGetTimeOfLastOutput
#define rtmGetTimeOfLastOutput(rtm)    ((rtm)->Timing.timeOfLastOutput)
#endif

/* Block signals (default storage) */
struct B_RflyUdpUltraSimpleEight_Dist_T {
  real_T UDPRecv1_o1[12];              /* '<Root>/UDP Recv1' */
  real_T UDPRecv1_o2[12];              /* '<Root>/UDP Recv1' */
  real_T UDPRecv1_o3[12];              /* '<Root>/UDP Recv1' */
  real_T UDPRecv1_o4[12];              /* '<Root>/UDP Recv1' */
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T Gain2;                        /* '<Root>/Gain2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Sum[3];                       /* '<Root>/Sum' */
  real_T Sum1[3];                      /* '<Root>/Sum1' */
  real_T Sum2[3];                      /* '<Root>/Sum2' */
  real_T Sum3[3];                      /* '<Root>/Sum3' */
  real_T vel1;                         /* '<Root>/vel1' */
  real_T vel2;                         /* '<Root>/vel2' */
  real_T vel3;                         /* '<Root>/vel3' */
  real_T vel4;                         /* '<Root>/vel4' */
  real_T vel5;                         /* '<Root>/vel5' */
  real_T vel6;                         /* '<Root>/vel6' */
  real_T vel7;                         /* '<Root>/vel7' */
  real_T vel8;                         /* '<Root>/vel8' */
  real_T UDPRecv2_o1[12];              /* '<Root>/UDP Recv2' */
  real_T UDPRecv2_o2[12];              /* '<Root>/UDP Recv2' */
  real_T UDPRecv2_o3[12];              /* '<Root>/UDP Recv2' */
  real_T UDPRecv2_o4[12];              /* '<Root>/UDP Recv2' */
  real_T Gain5;                        /* '<Root>/Gain5' */
  real_T Gain6;                        /* '<Root>/Gain6' */
  real_T Gain7;                        /* '<Root>/Gain7' */
  real_T Gain8;                        /* '<Root>/Gain8' */
  real_T Sum4[3];                      /* '<Root>/Sum4' */
  real_T Sum5[3];                      /* '<Root>/Sum5' */
  real_T Sum6[3];                      /* '<Root>/Sum6' */
  real_T Sum7[3];                      /* '<Root>/Sum7' */
  real_T vel10;                        /* '<Root>/vel10' */
  real_T vel11;                        /* '<Root>/vel11' */
  real_T vel12;                        /* '<Root>/vel12' */
  real_T vel13;                        /* '<Root>/vel13' */
  real_T vel14;                        /* '<Root>/vel14' */
  real_T vel15;                        /* '<Root>/vel15' */
  real_T vel16;                        /* '<Root>/vel16' */
  real_T vel9;                         /* '<Root>/vel9' */
  real_T SimulationPace1;              /* '<Root>/Simulation Pace1' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_RflyUdpUltraSimpleEight_Dist_T {
  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay_RWORK;              /* '<Root>/Transport Delay' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay1_RWORK;             /* '<Root>/Transport Delay1' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay2_RWORK;             /* '<Root>/Transport Delay2' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay3_RWORK;             /* '<Root>/Transport Delay3' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay4_RWORK;             /* '<Root>/Transport Delay4' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay5_RWORK;             /* '<Root>/Transport Delay5' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay6_RWORK;             /* '<Root>/Transport Delay6' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay7_RWORK;             /* '<Root>/Transport Delay7' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay8_RWORK;             /* '<Root>/Transport Delay8' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay9_RWORK;             /* '<Root>/Transport Delay9' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay10_RWORK;            /* '<Root>/Transport Delay10' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay11_RWORK;            /* '<Root>/Transport Delay11' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay12_RWORK;            /* '<Root>/Transport Delay12' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay13_RWORK;            /* '<Root>/Transport Delay13' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay14_RWORK;            /* '<Root>/Transport Delay14' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[2048];
  } TransportDelay15_RWORK;            /* '<Root>/Transport Delay15' */

  real_T SimulationPace1_RWORK;        /* '<Root>/Simulation Pace1' */
  void *UDPRecv1_PWORK[5];             /* '<Root>/UDP Recv1' */
  struct {
    void *TUbufferPtrs[2];
  } TransportDelay_PWORK;              /* '<Root>/Transport Delay' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay1_PWORK;             /* '<Root>/Transport Delay1' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay2_PWORK;             /* '<Root>/Transport Delay2' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay3_PWORK;             /* '<Root>/Transport Delay3' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay4_PWORK;             /* '<Root>/Transport Delay4' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay5_PWORK;             /* '<Root>/Transport Delay5' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay6_PWORK;             /* '<Root>/Transport Delay6' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay7_PWORK;             /* '<Root>/Transport Delay7' */

  void *UDPRecv2_PWORK[5];             /* '<Root>/UDP Recv2' */
  struct {
    void *TUbufferPtrs[2];
  } TransportDelay8_PWORK;             /* '<Root>/Transport Delay8' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay9_PWORK;             /* '<Root>/Transport Delay9' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay10_PWORK;            /* '<Root>/Transport Delay10' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay11_PWORK;            /* '<Root>/Transport Delay11' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay12_PWORK;            /* '<Root>/Transport Delay12' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay13_PWORK;            /* '<Root>/Transport Delay13' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay14_PWORK;            /* '<Root>/Transport Delay14' */

  struct {
    void *TUbufferPtrs[2];
  } TransportDelay15_PWORK;            /* '<Root>/Transport Delay15' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay_IWORK;              /* '<Root>/Transport Delay' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay1_IWORK;             /* '<Root>/Transport Delay1' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay2_IWORK;             /* '<Root>/Transport Delay2' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay3_IWORK;             /* '<Root>/Transport Delay3' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay4_IWORK;             /* '<Root>/Transport Delay4' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay5_IWORK;             /* '<Root>/Transport Delay5' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay6_IWORK;             /* '<Root>/Transport Delay6' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay7_IWORK;             /* '<Root>/Transport Delay7' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay8_IWORK;             /* '<Root>/Transport Delay8' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay9_IWORK;             /* '<Root>/Transport Delay9' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay10_IWORK;            /* '<Root>/Transport Delay10' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay11_IWORK;            /* '<Root>/Transport Delay11' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay12_IWORK;            /* '<Root>/Transport Delay12' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay13_IWORK;            /* '<Root>/Transport Delay13' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay14_IWORK;            /* '<Root>/Transport Delay14' */

  struct {
    int_T Tail;
    int_T Head;
    int_T Last;
    int_T CircularBufSize;
  } TransportDelay15_IWORK;            /* '<Root>/Transport Delay15' */
};

/* Parameters (default storage) */
struct P_RflyUdpUltraSimpleEight_Dist_T_ {
  real_T UDPRecv1_P1_Size[2];          /* Computed Parameter: UDPRecv1_P1_Size
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P1;                  /* Expression: port
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P2_Size[2];          /* Computed Parameter: UDPRecv1_P2_Size
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P2;                  /* Expression: num
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P3_Size[2];          /* Computed Parameter: UDPRecv1_P3_Size
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P3;                  /* Expression: modeValue
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P4_Size[2];          /* Computed Parameter: UDPRecv1_P4_Size
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P4;                  /* Expression: T
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P5_Size[2];          /* Computed Parameter: UDPRecv1_P5_Size
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T UDPRecv1_P5[14];              /* Computed Parameter: UDPRecv1_P5
                                        * Referenced by: '<Root>/UDP Recv1'
                                        */
  real_T _Value;                       /* Expression: 10
                                        * Referenced by: '<Root>/期望高度'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T u_Value;                      /* Expression: 10
                                        * Referenced by: '<Root>/期望高度1'
                                        */
  real_T Gain2_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T u_Value_e;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度2'
                                        */
  real_T Gain3_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T u_Value_j;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度3'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T SineWave_Amp;                 /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Bias;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Freq;                /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Phase;               /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave1_Amp;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave1'
                                        */
  real_T SineWave1_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave1'
                                        */
  real_T SineWave1_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave1'
                                        */
  real_T SineWave1_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave1'
                                        */
  real_T SineWave2_Amp;                /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave2'
                                        */
  real_T SineWave2_Bias;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave2'
                                        */
  real_T SineWave2_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave2'
                                        */
  real_T SineWave2_Phase;              /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave2'
                                        */
  real_T SineWave3_Amp;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave3'
                                        */
  real_T SineWave3_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave3'
                                        */
  real_T SineWave3_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave3'
                                        */
  real_T SineWave3_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave3'
                                        */
  real_T SineWave4_Amp;                /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave4'
                                        */
  real_T SineWave4_Bias;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave4'
                                        */
  real_T SineWave4_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave4'
                                        */
  real_T SineWave4_Phase;              /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave4'
                                        */
  real_T SineWave5_Amp;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave5'
                                        */
  real_T SineWave5_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave5'
                                        */
  real_T SineWave5_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave5'
                                        */
  real_T SineWave5_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave5'
                                        */
  real_T SineWave6_Amp;                /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave6'
                                        */
  real_T SineWave6_Bias;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave6'
                                        */
  real_T SineWave6_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave6'
                                        */
  real_T SineWave6_Phase;              /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave6'
                                        */
  real_T SineWave7_Amp;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave7'
                                        */
  real_T SineWave7_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave7'
                                        */
  real_T SineWave7_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave7'
                                        */
  real_T SineWave7_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave7'
                                        */
  real_T TransportDelay_Delay;         /* Expression: 15
                                        * Referenced by: '<Root>/Transport Delay'
                                        */
  real_T TransportDelay_InitOutput;    /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay'
                                        */
  real_T TransportDelay1_Delay;        /* Expression: 15
                                        * Referenced by: '<Root>/Transport Delay1'
                                        */
  real_T TransportDelay1_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay1'
                                        */
  real_T TransportDelay2_Delay;        /* Expression: 20
                                        * Referenced by: '<Root>/Transport Delay2'
                                        */
  real_T TransportDelay2_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay2'
                                        */
  real_T TransportDelay3_Delay;        /* Expression: 20
                                        * Referenced by: '<Root>/Transport Delay3'
                                        */
  real_T TransportDelay3_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay3'
                                        */
  real_T TransportDelay4_Delay;        /* Expression: 25
                                        * Referenced by: '<Root>/Transport Delay4'
                                        */
  real_T TransportDelay4_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay4'
                                        */
  real_T TransportDelay5_Delay;        /* Expression: 25
                                        * Referenced by: '<Root>/Transport Delay5'
                                        */
  real_T TransportDelay5_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay5'
                                        */
  real_T TransportDelay6_Delay;        /* Expression: 30
                                        * Referenced by: '<Root>/Transport Delay6'
                                        */
  real_T TransportDelay6_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay6'
                                        */
  real_T TransportDelay7_Delay;        /* Expression: 30
                                        * Referenced by: '<Root>/Transport Delay7'
                                        */
  real_T TransportDelay7_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay7'
                                        */
  real_T vel1_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel1'
                                        */
  real_T vel2_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel2'
                                        */
  real_T vel3_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel3'
                                        */
  real_T vel4_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel4'
                                        */
  real_T vel5_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel5'
                                        */
  real_T vel6_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel6'
                                        */
  real_T vel7_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel7'
                                        */
  real_T vel8_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel8'
                                        */
  real_T UDPRecv2_P1_Size[2];          /* Computed Parameter: UDPRecv2_P1_Size
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P1;                  /* Expression: port
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P2_Size[2];          /* Computed Parameter: UDPRecv2_P2_Size
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P2;                  /* Expression: num
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P3_Size[2];          /* Computed Parameter: UDPRecv2_P3_Size
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P3;                  /* Expression: modeValue
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P4_Size[2];          /* Computed Parameter: UDPRecv2_P4_Size
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P4;                  /* Expression: T
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P5_Size[2];          /* Computed Parameter: UDPRecv2_P5_Size
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T UDPRecv2_P5[13];              /* Computed Parameter: UDPRecv2_P5
                                        * Referenced by: '<Root>/UDP Recv2'
                                        */
  real_T u_Value_g;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度4'
                                        */
  real_T Gain5_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T u_Value_k;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度5'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T u_Value_o;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度6'
                                        */
  real_T Gain7_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain7'
                                        */
  real_T u_Value_c;                    /* Expression: 10
                                        * Referenced by: '<Root>/期望高度7'
                                        */
  real_T Gain8_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain8'
                                        */
  real_T SineWave10_Amp;               /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave10'
                                        */
  real_T SineWave10_Bias;              /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave10'
                                        */
  real_T SineWave10_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave10'
                                        */
  real_T SineWave10_Phase;             /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave10'
                                        */
  real_T SineWave11_Amp;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave11'
                                        */
  real_T SineWave11_Bias;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave11'
                                        */
  real_T SineWave11_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave11'
                                        */
  real_T SineWave11_Phase;             /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave11'
                                        */
  real_T SineWave12_Amp;               /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave12'
                                        */
  real_T SineWave12_Bias;              /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave12'
                                        */
  real_T SineWave12_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave12'
                                        */
  real_T SineWave12_Phase;             /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave12'
                                        */
  real_T SineWave13_Amp;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave13'
                                        */
  real_T SineWave13_Bias;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave13'
                                        */
  real_T SineWave13_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave13'
                                        */
  real_T SineWave13_Phase;             /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave13'
                                        */
  real_T SineWave14_Amp;               /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave14'
                                        */
  real_T SineWave14_Bias;              /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave14'
                                        */
  real_T SineWave14_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave14'
                                        */
  real_T SineWave14_Phase;             /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave14'
                                        */
  real_T SineWave15_Amp;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave15'
                                        */
  real_T SineWave15_Bias;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave15'
                                        */
  real_T SineWave15_Freq;              /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave15'
                                        */
  real_T SineWave15_Phase;             /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave15'
                                        */
  real_T SineWave8_Amp;                /* Expression: -5
                                        * Referenced by: '<Root>/Sine Wave8'
                                        */
  real_T SineWave8_Bias;               /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave8'
                                        */
  real_T SineWave8_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave8'
                                        */
  real_T SineWave8_Phase;              /* Expression: pi/2
                                        * Referenced by: '<Root>/Sine Wave8'
                                        */
  real_T SineWave9_Amp;                /* Expression: 5
                                        * Referenced by: '<Root>/Sine Wave9'
                                        */
  real_T SineWave9_Bias;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave9'
                                        */
  real_T SineWave9_Freq;               /* Expression: 1
                                        * Referenced by: '<Root>/Sine Wave9'
                                        */
  real_T SineWave9_Phase;              /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave9'
                                        */
  real_T TransportDelay8_Delay;        /* Expression: 15+2.5
                                        * Referenced by: '<Root>/Transport Delay8'
                                        */
  real_T TransportDelay8_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay8'
                                        */
  real_T TransportDelay9_Delay;        /* Expression: 15+2.5
                                        * Referenced by: '<Root>/Transport Delay9'
                                        */
  real_T TransportDelay9_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay9'
                                        */
  real_T TransportDelay10_Delay;       /* Expression: 20+2.5
                                        * Referenced by: '<Root>/Transport Delay10'
                                        */
  real_T TransportDelay10_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay10'
                                        */
  real_T TransportDelay11_Delay;       /* Expression: 20+2.5
                                        * Referenced by: '<Root>/Transport Delay11'
                                        */
  real_T TransportDelay11_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay11'
                                        */
  real_T TransportDelay12_Delay;       /* Expression: 25+2.5
                                        * Referenced by: '<Root>/Transport Delay12'
                                        */
  real_T TransportDelay12_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay12'
                                        */
  real_T TransportDelay13_Delay;       /* Expression: 25+2.5
                                        * Referenced by: '<Root>/Transport Delay13'
                                        */
  real_T TransportDelay13_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay13'
                                        */
  real_T TransportDelay14_Delay;       /* Expression: 30+2.5
                                        * Referenced by: '<Root>/Transport Delay14'
                                        */
  real_T TransportDelay14_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay14'
                                        */
  real_T TransportDelay15_Delay;       /* Expression: 30+2.5
                                        * Referenced by: '<Root>/Transport Delay15'
                                        */
  real_T TransportDelay15_InitOutput;  /* Expression: 0
                                        * Referenced by: '<Root>/Transport Delay15'
                                        */
  real_T vel10_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel10'
                                        */
  real_T vel11_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel11'
                                        */
  real_T vel12_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel12'
                                        */
  real_T vel13_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel13'
                                        */
  real_T vel14_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel14'
                                        */
  real_T vel15_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel15'
                                        */
  real_T vel16_Value;                  /* Expression: 0
                                        * Referenced by: '<Root>/vel16'
                                        */
  real_T vel9_Value;                   /* Expression: 0
                                        * Referenced by: '<Root>/vel9'
                                        */
  real_T SimulationPace1_P1;           /* Expression: SimulationPace
                                        * Referenced by: '<Root>/Simulation Pace1'
                                        */
  real_T SimulationPace1_P2;           /* Expression: SleepMode
                                        * Referenced by: '<Root>/Simulation Pace1'
                                        */
  real_T SimulationPace1_P3;           /* Expression: OutputPaceError
                                        * Referenced by: '<Root>/Simulation Pace1'
                                        */
  real_T SimulationPace1_P4;           /* Expression: SampleTime
                                        * Referenced by: '<Root>/Simulation Pace1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_RflyUdpUltraSimpleEight_Dist_T {
  struct SimStruct_tag * *childSfunctions;
  const char_T *errorStatus;
  SS_SimMode simMode;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  RTWSolverInfo *solverInfoPtr;
  void *sfcnInfo;

  /*
   * NonInlinedSFcns:
   * The following substructure contains information regarding
   * non-inlined s-functions used in the model.
   */
  struct {
    RTWSfcnInfo sfcnInfo;
    time_T *taskTimePtrs[2];
    SimStruct childSFunctions[2];
    SimStruct *childSFunctionPtrs[2];
    struct _ssBlkInfo2 blkInfo2[2];
    struct _ssBlkInfoSLSize blkInfoSLSize[2];
    struct _ssSFcnModelMethods2 methods2[2];
    struct _ssSFcnModelMethods3 methods3[2];
    struct _ssSFcnModelMethods4 methods4[2];
    struct _ssStatesInfo2 statesInfo2[2];
    ssPeriodicStatesInfo periodicStatesInfo[2];
    struct _ssPortInfo2 inputOutputPortInfo2[2];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[4];
      struct _ssPortInputsSLSize inputPortInfoSLSize[4];
      struct _ssInPortUnit inputPortUnits[4];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[4];
      real_T const *UPtrs0[5];
      real_T const *UPtrs1[5];
      real_T const *UPtrs2[5];
      real_T const *UPtrs3[5];
      struct _ssPortOutputs outputPortInfo[4];
      struct _ssPortOutputsSLSize outputPortInfoSLSize[4];
      struct _ssOutPortUnit outputPortUnits[4];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[4];
      uint_T attribs[5];
      mxArray *params[5];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[4];
      struct _ssPortInputsSLSize inputPortInfoSLSize[4];
      struct _ssInPortUnit inputPortUnits[4];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[4];
      real_T const *UPtrs0[5];
      real_T const *UPtrs1[5];
      real_T const *UPtrs2[5];
      real_T const *UPtrs3[5];
      struct _ssPortOutputs outputPortInfo[4];
      struct _ssPortOutputsSLSize outputPortInfoSLSize[4];
      struct _ssOutPortUnit outputPortUnits[4];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[4];
      uint_T attribs[5];
      mxArray *params[5];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;
  } NonInlinedSFcns;

  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T options;
    int_T numContStates;
    int_T numU;
    int_T numY;
    int_T numSampTimes;
    int_T numBlocks;
    int_T numBlockIO;
    int_T numBlockPrms;
    int_T numDwork;
    int_T numSFcnPrms;
    int_T numSFcns;
    int_T numIports;
    int_T numOports;
    int_T numNonSampZCs;
    int_T sysDirFeedThru;
    int_T rtwGenSfcn;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T stepSize1;
    time_T tStart;
    time_T tFinal;
    time_T timeOfLastOutput;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *sampleTimes;
    time_T *offsetTimes;
    int_T *sampleTimeTaskIDPtr;
    int_T *sampleHits;
    int_T *perTaskSampleHits;
    time_T *t;
    time_T sampleTimesArray[2];
    time_T offsetTimesArray[2];
    int_T sampleTimeTaskIDArray[2];
    int_T sampleHitArray[2];
    int_T perTaskSampleHitsArray[4];
    time_T tArray[2];
  } Timing;
};

/* Class declaration for model RflyUdpUltraSimpleEight_Dist */
class RflyUdpUltraSimpleEight_Dist
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_RflyUdpUltraSimpleEight_Dist_T * getRTM();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  RflyUdpUltraSimpleEight_Dist();

  /* Destructor */
  ~RflyUdpUltraSimpleEight_Dist();

  /* private data and function members */
 private:
  /* Block signals */
  B_RflyUdpUltraSimpleEight_Dist_T RflyUdpUltraSimpleEight_Dist_B;

  /* Block states */
  DW_RflyUdpUltraSimpleEight_Dist_T RflyUdpUltraSimpleEight_Dist_DW;

  /* Tunable parameters */
  static P_RflyUdpUltraSimpleEight_Dist_T RflyUdpUltraSimpleEight_Dist_P;

  /* Real-Time Model */
  RT_MODEL_RflyUdpUltraSimpleEight_Dist_T RflyUdpUltraSimpleEight_Dist_M;
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'RflyUdpUltraSimpleEight_Dist'
 */
#endif                          /* RTW_HEADER_RflyUdpUltraSimpleEight_Dist_h_ */
