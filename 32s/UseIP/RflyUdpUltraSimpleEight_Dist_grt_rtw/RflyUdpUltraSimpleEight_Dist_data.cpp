/*
 * RflyUdpUltraSimpleEight_Dist_data.cpp
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

/* Block parameters (default storage) */
P_RflyUdpUltraSimpleEight_Dist_T RflyUdpUltraSimpleEight_Dist::
  RflyUdpUltraSimpleEight_Dist_P = {
  /* Computed Parameter: UDPRecv1_P1_Size
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 1.0, 1.0 },

  /* Expression: port
   * Referenced by: '<Root>/UDP Recv1'
   */
  20100.0,

  /* Computed Parameter: UDPRecv1_P2_Size
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 1.0, 1.0 },

  /* Expression: num
   * Referenced by: '<Root>/UDP Recv1'
   */
  4.0,

  /* Computed Parameter: UDPRecv1_P3_Size
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 1.0, 1.0 },

  /* Expression: modeValue
   * Referenced by: '<Root>/UDP Recv1'
   */
  2.0,

  /* Computed Parameter: UDPRecv1_P4_Size
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 1.0, 1.0 },

  /* Expression: T
   * Referenced by: '<Root>/UDP Recv1'
   */
  0.033333333333333333,

  /* Computed Parameter: UDPRecv1_P5_Size
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 1.0, 14.0 },

  /* Computed Parameter: UDPRecv1_P5
   * Referenced by: '<Root>/UDP Recv1'
   */
  { 49.0, 57.0, 50.0, 46.0, 49.0, 54.0, 56.0, 46.0, 51.0, 49.0, 46.0, 49.0, 49.0,
    56.0 },

  /* Expression: 10
   * Referenced by: '<Root>/期望高度'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain1'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度1'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain2'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度2'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain3'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度3'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain4'
   */
  -1.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave1'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave1'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave1'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave2'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave2'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave2'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave2'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave3'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave3'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave3'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave3'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave4'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave4'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave4'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave4'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave5'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave5'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave5'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave5'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave6'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave6'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave6'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave6'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave7'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave7'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave7'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave7'
   */
  0.0,

  /* Expression: 15
   * Referenced by: '<Root>/Transport Delay'
   */
  15.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay'
   */
  0.0,

  /* Expression: 15
   * Referenced by: '<Root>/Transport Delay1'
   */
  15.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay1'
   */
  0.0,

  /* Expression: 20
   * Referenced by: '<Root>/Transport Delay2'
   */
  20.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay2'
   */
  0.0,

  /* Expression: 20
   * Referenced by: '<Root>/Transport Delay3'
   */
  20.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay3'
   */
  0.0,

  /* Expression: 25
   * Referenced by: '<Root>/Transport Delay4'
   */
  25.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay4'
   */
  0.0,

  /* Expression: 25
   * Referenced by: '<Root>/Transport Delay5'
   */
  25.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay5'
   */
  0.0,

  /* Expression: 30
   * Referenced by: '<Root>/Transport Delay6'
   */
  30.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay6'
   */
  0.0,

  /* Expression: 30
   * Referenced by: '<Root>/Transport Delay7'
   */
  30.0,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay7'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel4'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel5'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel6'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel7'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel8'
   */
  0.0,

  /* Computed Parameter: UDPRecv2_P1_Size
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 1.0, 1.0 },

  /* Expression: port
   * Referenced by: '<Root>/UDP Recv2'
   */
  20108.0,

  /* Computed Parameter: UDPRecv2_P2_Size
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 1.0, 1.0 },

  /* Expression: num
   * Referenced by: '<Root>/UDP Recv2'
   */
  4.0,

  /* Computed Parameter: UDPRecv2_P3_Size
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 1.0, 1.0 },

  /* Expression: modeValue
   * Referenced by: '<Root>/UDP Recv2'
   */
  2.0,

  /* Computed Parameter: UDPRecv2_P4_Size
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 1.0, 1.0 },

  /* Expression: T
   * Referenced by: '<Root>/UDP Recv2'
   */
  0.033333333333333333,

  /* Computed Parameter: UDPRecv2_P5_Size
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 1.0, 13.0 },

  /* Computed Parameter: UDPRecv2_P5
   * Referenced by: '<Root>/UDP Recv2'
   */
  { 49.0, 57.0, 50.0, 46.0, 49.0, 54.0, 56.0, 46.0, 51.0, 49.0, 46.0, 52.0, 49.0
  },

  /* Expression: 10
   * Referenced by: '<Root>/期望高度4'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain5'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度5'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain6'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度6'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain7'
   */
  -1.0,

  /* Expression: 10
   * Referenced by: '<Root>/期望高度7'
   */
  10.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain8'
   */
  -1.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave10'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave10'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave10'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave10'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave11'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave11'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave11'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave11'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave12'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave12'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave12'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave12'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave13'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave13'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave13'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave13'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave14'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave14'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave14'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave14'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave15'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave15'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave15'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave15'
   */
  0.0,

  /* Expression: -5
   * Referenced by: '<Root>/Sine Wave8'
   */
  -5.0,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave8'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave8'
   */
  1.0,

  /* Expression: pi/2
   * Referenced by: '<Root>/Sine Wave8'
   */
  1.5707963267948966,

  /* Expression: 5
   * Referenced by: '<Root>/Sine Wave9'
   */
  5.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave9'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Sine Wave9'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave9'
   */
  0.0,

  /* Expression: 15+2.5
   * Referenced by: '<Root>/Transport Delay8'
   */
  17.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay8'
   */
  0.0,

  /* Expression: 15+2.5
   * Referenced by: '<Root>/Transport Delay9'
   */
  17.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay9'
   */
  0.0,

  /* Expression: 20+2.5
   * Referenced by: '<Root>/Transport Delay10'
   */
  22.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay10'
   */
  0.0,

  /* Expression: 20+2.5
   * Referenced by: '<Root>/Transport Delay11'
   */
  22.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay11'
   */
  0.0,

  /* Expression: 25+2.5
   * Referenced by: '<Root>/Transport Delay12'
   */
  27.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay12'
   */
  0.0,

  /* Expression: 25+2.5
   * Referenced by: '<Root>/Transport Delay13'
   */
  27.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay13'
   */
  0.0,

  /* Expression: 30+2.5
   * Referenced by: '<Root>/Transport Delay14'
   */
  32.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay14'
   */
  0.0,

  /* Expression: 30+2.5
   * Referenced by: '<Root>/Transport Delay15'
   */
  32.5,

  /* Expression: 0
   * Referenced by: '<Root>/Transport Delay15'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel10'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel11'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel12'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel13'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel14'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel15'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel16'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/vel9'
   */
  0.0,

  /* Expression: SimulationPace
   * Referenced by: '<Root>/Simulation Pace1'
   */
  1.0,

  /* Expression: SleepMode
   * Referenced by: '<Root>/Simulation Pace1'
   */
  3.0,

  /* Expression: OutputPaceError
   * Referenced by: '<Root>/Simulation Pace1'
   */
  1.0,

  /* Expression: SampleTime
   * Referenced by: '<Root>/Simulation Pace1'
   */
  -1.0
};
